# ESP32-WROOM 双轮平衡车自动启动与项目 API 设计

## 目标

在不引入尚无实车模型参数支持的 LQR、自适应控制或自动增益搜索的前提下，完成以下改动：

- ESP32-WROOM 完成 FOC 对齐和 MPU6050 静止标定后自动进入平衡。
- 去除依赖当前车型参数的启动限制和轮速停机阈值。
- 将倾倒判断改成较宽松、带延时的倒地判断，避免瞬时扰动误停。
- 保留与车型参数无关的硬故障停机和串口急停。
- 在平衡过程中允许实时调整姿态增益和直立参考角。
- 为 MPU6050、AS5600 和双电机提供项目级公共接口。
- 将 PlatformIO 默认环境改为 `balance_car_new`。

本设计不能证明未经实车调参的机器人能够一次站稳。首次运行仍需架空方向检查，并在人工扶持下逐步调参。

## 文件边界

- `V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h`：主逻辑，负责初始化、状态控制、传感器读取、平衡控制和故障处理。
- `V1_06_TwoWheel_Balance_Car_new/BalanceMath_new.h`：保持现有纯数学控制函数，不承载硬件状态。
- 新增公共 API 头文件：声明传感器数据结构、车轮枚举和项目级调用接口。
- `src/balance_car_new.cpp`：PlatformIO 入口，继续只引入 Arduino 草图入口。
- `platformio.ini`：将默认构建环境设为 `balance_car_new`；保留 `board = esp32dev`，适配常见 ESP32-WROOM-32 开发板。
- `tests/balance_car_new/balance_behavior_new_test.cpp`：增加自动启动、公共接口、在线调参和新停机逻辑的主机测试。

## 公共接口

公共接口使用独立命名空间，避免与 SimpleFOC 类型和现有内部函数冲突：

```cpp
enum class Wheel { Left, Right };

struct Mpu6050Data {
  int16_t rawAx, rawAy, rawAz;
  int16_t rawGx, rawGy, rawGz;
  float accelXG, accelYG, accelZG;
  float gyroXDps, gyroYDps, gyroZDps;
  float pitchDeg;
  float pitchRateDegPerSec;
};

struct EncoderData {
  float angleRad;
  float velocityRadPerSec;
  bool magnetHealthy;
};

bool readMpu6050(Mpu6050Data& data);
bool readEncoder(Wheel wheel, EncoderData& data);
bool setMotorVoltages(float leftVoltage, float rightVoltage);
void stopAllMotors();
```

接口约束：

- MPU6050 接口返回原始值、换算后的加速度/角速度以及当前融合俯仰角。
- `readMpu6050()`执行一次真实寄存器读取；返回的 `pitchDeg`和`pitchRateDegPerSec`是控制循环最近一次融合结果，读取接口本身不重复推进互补滤波器。
- 编码器接口返回 SimpleFOC 最近一次电机更新得到的累计机械角和机械角速度，并检查磁铁状态。
- 电机接口参数是按车体前进方向定义的左右 q 轴电压，不是目标转速；内部处理安装方向和电压限幅。
- `setMotorVoltages()`每次调用都依次执行两路 `loopFOC()`和`move()`；平衡主循环需要持续调用，而不是只调用一次。
- 电机未初始化、故障已锁定或输入不是有限值时，电机接口拒绝输出。
- `setMotorVoltages()`不会自动清除故障，也不会绕过驱动使能状态。
- `stopAllMotors()`保持先拉低硬件 EN、再清零 SimpleFOC 状态的顺序。

## 启动与状态逻辑

启动顺序为：

1. 将两路驱动 EN 拉低。
2. 初始化左右 I2C 总线。
3. 检查两只 AS5600 的磁铁状态。
4. 初始化 MPU6050。
5. 左右电机依次执行 FOC 对齐。
6. 保持机器人处于目标直立姿态并静止 1.5 秒，标定陀螺仪零偏和直立参考角。
7. 标定成功后立即使能两台电机，清零控制状态并进入平衡。

启动不再要求当前倾角小于 8°、角速度小于 20°/s 或轮速小于 1 rad/s，因为静止标定得到的姿态本身就是本次启动的直立参考。

串口 `X` 或倒地停机后不自动重新启动。操作人员扶正后可发送 `A` 重新进入平衡；复位则重新执行完整对齐和标定，并再次自动启动。

## 平衡控制

首次调试只启用姿态 PD：

```text
error = wrap(pitch - setpoint)
voltage = clamp((Kp * error + Kd * pitchRate) * rad_per_degree,
                -voltageLimit,
                +voltageLimit)
```

- 控制周期目标为 5 ms（200 Hz）。
- MPU6050 采用陀螺仪积分和加速度倾角互补融合。
- 初始 `Kp`、`Kd` 仍是起调值，不视为实车适配结果。
- 输出继续限制在 ±3 V，避免方向和参数尚未验证时直接施加全部母线电压。
- 速度 PI 外环默认关闭，姿态环稳定后再由 `V` 命令启用。
- 主循环通过公共电机接口下发左右电压，避免公共接口与实际控制路径分离。

## 在线调参与命令

运行平衡时仍接受以下命令：

- `X`：立即关闭两路输出。
- `1` / `0`：Kp 增加 / 减少 0.5。
- `3` / `2`：Kd 增加 / 减少 0.02。
- `+` / `-`：直立参考角增加 / 减少 0.1°，相对标定值限制在 ±5°。
- `?`：显示当前设置。

`V`只允许停机时切换速度外环，避免在姿态调试过程中意外改变控制结构。`L`、`R`单轮方向测试也只允许停机时执行。`A`用于停机后的人工重新使能。

## 停机和故障逻辑

删除以下依赖车型参数的判断：

- 启动时的 ±8°倾角限制。
- 启动时的 20°/s 角速度限制。
- 启动时的 1 rad/s 轮速限制。
- 运行时的 30 rad/s 轮速停机。

倾倒判断改为：相对静止标定的直立角超过 60°并连续保持 300 ms，才视为已经倒地并停止电机。角度回到阈值内时清除计时，不因单个异常样本停止。

继续保留：

- 串口 `X` 急停。
- MPU6050 或 AS5600 I2C 失败、短读和磁铁状态异常。
- 传感器或控制输出出现 NaN/无穷值。
- 控制循环在电机运行时超过 30 ms 未更新。
- 初始化或 FOC 对齐失败。

硬故障保持锁定，必须排查后复位。普通 `X` 和倒地停机不锁定硬故障，可由 `A`重新使能。

## PlatformIO 与 Windows 环境

`platformio.ini`只做必要改动：

```ini
[platformio]
default_envs = balance_car_new
```

继续使用：

```ini
board = esp32dev
```

这适用于常见 ESP32-WROOM-32 DevKit。现有位于用户目录下的短 ASCII 构建和依赖路径继续保留，以规避 Windows 长路径及中文路径问题。不会写死 COM 端口；烧录时根据 `pio device list` 的实际结果选择。

## 验证标准

- 主机测试证明健康初始化后自动进入平衡状态。
- 主机测试证明公共 MPU6050、编码器和电机接口能返回或处理预期数据。
- 主机测试证明运行中可以调整 Kp、Kd 和参考角。
- 主机测试证明旧启动阈值和轮速阈值不再触发停机。
- 主机测试证明超过倒地角度的短时扰动不停机，持续 300 ms 后停机。
- 主机测试证明 `X`、I2C 故障、非有限数值和控制超时仍会停机。
- `pio run -e balance_car_new`成功。
- 将默认环境修改后，`pio run`也构建 `balance_car_new`。
- 不以编译或主机测试结果宣称实车已经稳定；真实方向、输出能力、温升和增益仍需实车验证。
