# 作业测试新版（_new）

三个旧版 `.ino` 原样保留。新版分别放在同名的 `_new` 文件夹内，避免 Arduino IDE 把新旧两个 `setup()` / `loop()` 一起编译。

| 测试 | 新版文件 | PlatformIO 环境 |
|---|---|---|
| AS5600 手转计数 | `V1_01_AS5600_Sensor_Test_new/V1_01_AS5600_Sensor_Test_new.ino` | `sensor_test_new` |
| 三相驱动波形 | `V1_02_2804_OpenLoop_Test_new/V1_02_2804_OpenLoop_Test_new.ino` | `openloop_test_new` |
| 闭环空载测速 | `V1_03_ClosedLoop_Velocity_Test_new/V1_03_ClosedLoop_Velocity_Test_new.ino` | `closedloop_velocity_test_new` |
| 驱动占空比标定 | `V1_04_Driver_Calibration/V1_04_Driver_Calibration.ino` | `driver_calibration` |
| 电压阶跃辨识 | `V1_05_Identification_Step/V1_05_Identification_Step.ino` | `identification_step` |
| 双轮自平衡（MPU6050 + 双 AS5600） | `V1_06_TwoWheel_Balance_Car_new/V1_06_TwoWheel_Balance_Car_new.ino` | `balance_car_new` |

## 编译与烧录

在 VS Code 的 PlatformIO → Project Tasks 中，展开表中对应的 `_new` 环境，再展开 General，选择 Build 编译或 Upload 烧录。只编译不需要开发板。串口波特率为 **115200**，命令行结束符用换行（LF）。一次烧录一个测试程序。

默认环境仍是旧版 `sensor_test`，要使用新版请明确选择 `sensor_test_new` 等环境，不能只看当前打开的文件名判断将烧录哪个版本。

在 PlatformIO 终端中也可执行：

```text
pio run -e sensor_test_new
pio run -e openloop_test_new
pio run -e closedloop_velocity_test_new
pio run -e driver_calibration
pio run -e identification_step
pio run -e balance_car_new

pio run -e sensor_test_new -t upload
pio run -e balance_car_new -t upload
pio device monitor -b 115200
```

新版沿用 `esp32dev`、SimpleFOC **2.3.3** 和当前 ESP32 Arduino 2.x 框架。若使用 Arduino IDE，打开对应 `_new` 文件夹里的 `.ino`，选择 ESP32 Dev Module，并安装相同的库/框架版本。

## 1. AS5600 手转计数

接线：AS5600 VCC→3.3V，GND→GND，SDA→GPIO21，SCL→GPIO22。本测试只需 ESP32 和编码器，不给电机驱动上电。

上电或复位时自动以当前位置清零；也可保持转子不动，发送 `Z`（小写 `z` 也可）重新清零。程序连续采样跟踪多圈，每 100 ms 打印一行：

```text
angle_rad  counts  turns  velocity_rad_s  rpm  err_pos4_counts  err_neg4_counts
```

- `angle_rad`：相对零点的有符号累计角度，单位 rad。
- `counts`：I²C 角度按 **4096 计数/机械圈**换算的净累计计数，不是 ABZ 硬件脉冲。
- `turns`：净圈数；先正转 4 圈再反转 4 圈，净圈数回到 0，不是 -4。
- `velocity_rad_s`、`rpm`：带方向的速度。手转测试的速度按相邻打印时刻间的角度变化计算。
- `err_pos4_counts = counts - 16384`：相对正转 4 圈的计数偏差。
- `err_neg4_counts = counts + 16384`：相对反转 4 圈的计数偏差。

两项偏差同时显示：测正转 4 圈时读 `err_pos4_counts`，测反转 4 圈时读 `err_neg4_counts`。理想终点如下（这不是实测结果）：

| 从清零位置到终点 | counts | turns | err_pos4_counts | err_neg4_counts |
|---|---:|---:|---:|---:|
| 正转 4 圈 | 16384 | 4.0000 | 0 | 32768 |
| 反转 4 圈 | -16384 | -4.0000 | -32768 | 0 |

先完成正转测量，回到起点或在新的起点发送 `Z`，再单独反转 4 圈。若约定的正转显示负计数，在手转新版开头把 `SENSOR_DIRECTION` 从 `1` 改为 `-1`。左右轮可分别连接、分别清零测试。两个 AS5600 地址都是 `0x36`；同时连接需要独立 I²C 总线或 I²C 多路复用器，本程序一次读取一个编码器。

## 2. 三相驱动波形

新版开环程序在初始化前设置了：

```cpp
driver.pwm_frequency = PWM_FREQUENCY_HZ;  // 10000 Hz
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
```

三相 PWM 引脚为 GPIO32、33、25，驱动 EN 为 GPIO12。`POLE_PAIRS = 7` 是沿用的示例值，必须按实际电机极对数修改；基波频率计算会自动使用相同极对数。

命令：`T6.28` 正转目标、`T0` 零速度目标、`T-6.28` 反转目标；`L4` / `L5` 设置电压限幅为 4 V / 5 V。T 命令单位为 rad/s。

每 1 秒输出 `target`（rad/s）、`target_rpm`（目标转速）和 `base_hz`（计算的电气基波频率）。公式为 `abs(target) * pole_pairs / (2*pi)`；7 极对、T6.28 对应约 **6.996 Hz** 基波、**59.97 RPM** 目标转速。它们不是开环实测转速，载波的 10000 Hz 也应以实际波形测量核对。`T0` 停止旋转磁场，但仍可能保持静态励磁，并不切断驱动电源。

## 3. 闭环空载测速

同样配置 10000 Hz SVPWM。正常控制循环持续执行 `loopFOC()`、`move()`，只把打印间隔限制为 100 ms，不加入 100 ms 延时。

```text
target:6.28  actual:6.25  rpm:59.68
```

上面仅是输出格式示例。`target` 是目标机械角速度、`actual` 是编码器反馈经过库的速度滤波后的机械角速度（均为 rad/s），`rpm` 由 actual 换算，可用于记录空载转速曲线。反转时 actual 和 rpm 为负。

保持原始示例的 12 V 电源、3 V 初始电压限幅、6 V 驱动限幅和速度 PID 参数。运行电机前需要确认实际极对数、电源、驱动接线；`initFOC()` 的对齐动作会使转子转动。新版只在初始化成功后进入控制循环。

与产品标称空载转速对比时，应使用相同供电和测试条件。低速目标与 3 V 限幅下测出的约 60 RPM 只能说明该目标下的控制结果，不能直接当作电机额定空载最高转速或产品性能偏差。

## 4. 双轮自平衡

新增环境为 `balance_car_new`，详细接线、方向检查、串口调参与保护说明见 [README_balance_new.md](README_balance_new.md)。

- 左轮：PWM GPIO32/33/25、EN12、AS5600 SDA21/SCL22。
- 右轮：PWM GPIO14/27/26、EN13、AS5600 SDA19/SCL18。
- MPU6050 独立使用软件 I²C SDA16/SCL17，不与左右 AS5600 共用信号线；逐针接线见 [WIRING_I2C_new.md](WIRING_I2C_new.md)。四个 STS3215 先由外部方式固定姿态，本程序不控制舵机。
- 初始化 FOC 时两轮会转动；随后校准 MPU6050 并等待串口命令。`L/R` 为架空单轮限时测试，`A` 开始平衡，`X` 停机。
- 这是可编译烧录的实车起调版，不能保证未经方向验证和调参就站稳，也不实现从地面自动起立。

本作业工程仍需显式选择 `balance_car_new`，其默认环境没有更改。克隆仓库 `mcdgf/Code/ESP32_Balance_Car_new` 中也有一份默认选择该环境的独立工程。

```text
pio run -e balance_car_new
pio run -e balance_car_new -t upload
pio device monitor -e balance_car_new -b 115200
```

## 5. 电机模型辨识作业新增程序

`driver_calibration` 用于作业 4.1。测试时必须断开电机，通过 `D0.1` 至 `D0.8` 命令让 M1 相对 M2 输出固定占空比 PWM，记录实测母线电压和 M1-M2 平均电压。

把数据填入 `analysis/driver_calibration_template.csv` 后运行：

```matlab
result = fit_driver_calibration("analysis/driver_calibration_template.csv");
```

`identification_step` 用于作业 4.2 和 4.3。它在无速度环的 FOC 电压转矩模式下自动施加 0.8、1.2、1.6、2.0 V 四档 q 轴电压阶跃，以 921600 baud 输出 1 ms CSV。完成 `initFOC()` 后先开始保存串口日志，再发送 `S`；`X` 中止。

采集后在 MATLAB 中运行：

```matlab
addpath("analysis")
results = identify_motor_second_order("motor_id.csv");
```

要进一步计算 J 和 B，必须把实测相电阻、q 轴电感以及单位一致的 Ke、Kt 传入函数。供应商标称值只能作为复核，不应替代实测值。
