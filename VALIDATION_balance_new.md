# 验证记录 `_new`

日期：2026-10-27。本次验证版本：`encoder diagnostics v2`，分别报告左右 AS5600 的通信与磁铁状态；MPU6050 独立 GPIO16/17 软件 I²C，左右 AS5600 保留 21/22、19/18。

| 验证项目 | 结果 |
|---|---|
| 独立工程 `mcdgf/Code/ESP32_Balance_Car_new`，PlatformIO `balance_car_new` | SUCCESS |
| 原作业工程，PlatformIO `balance_car_new` | SUCCESS |
| 主机软件 I²C 通信测试（g++，C++17，Wall/Wextra/Werror） | PASS |
| 主机控制器行为回归测试（同上） | PASS |
| 两处交付的固件源码、测试与新接线说明 SHA256 一致性 | PASS，13 个文件 |
| GitHub 上游已跟踪文件改动 | 无；仅增加 `Code/ESP32_Balance_Car_new` |

构建配置：Espressif32 7.1.3、esp32dev、Arduino-ESP32 2.0.17、SimpleFOC 2.3.3。

两处新版编译统计一致：RAM 23,740 字节（7.2%），Flash 327,269 字节（25.0%）。

此前交付已验证旧 `closedloop_velocity_test_new` 环境编译通过。本次未修改其源码、依赖或构建配置，没有重复执行该环境。

主机测试直接编译实际 `SoftwareI2C_new.h` 和 `BalanceController_new.h`，在 GPIO 层模拟 MPU6050 的 I²C 从设备应答，替换 Arduino/Wire/SimpleFOC 的外设接口。两个硬件 Wire 对 `0x68` 都不应答，控制器仍可完成 MPU6050 初始化和采样，确认不再依赖编码器总线。

软件 I²C 测试覆盖开漏配置、寄存器写入、重复 START、单字节/14 字节读取、主机逐字节 ACK 与末字节 NACK、错误地址、寄存器/数据无应答、时钟拉伸、SDA/SCL 卡低、整个事务超时、错误后释放线路、模拟 CPU 暂停和 micros() 回绕。GPIO 测试验证协议状态和数据，没有测量真实电气波形。

控制器回归覆盖初始化失败时空驱动保护、旧 IMU 读数拒绝使能、X 停机、200 ms 单轮测试、编码器短读、MPU6050 失联/时钟卡低、故障锁定、磁铁丢失、控制超时、过速、过倾，以及融合角跨界、PD 输出正负与限幅、速度环抗积分饱和。

本次新增回归覆盖左右同时失败时分别报告、Wire 错误码 2/3/4/5、零字节短读、磁铁状态 0x00/0x10/0x30/0x08/0x28/0x38，以及左右运行中读取失败。测试确认故障时两路 EN 关闭并保持故障锁定；启动编码器检查失败时，尚未进入 FOC 对齐或读取 MPU6050 寄存器。模拟 Wire 的错误码用于验证日志分支，实机重复 START 下地址无应答也可能表现为零字节短读。

复现命令：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e balance_car_new
.\tests\balance_car_new\run_new.ps1
```

没有执行 Upload，没有连接真实传感器/电机，没有验证实车能站稳的结论。软件 I²C 读取会同步占用主循环，实际事务耗时及 FOC 间隔需查看 `imu_us/foc_us` 并在硬件上评估；模拟接口不验证真实上拉、时序、电机电流、机械负载或闭环稳定裕量。接线按 `WIRING_I2C_new.md`，方向确认和调参按 `README_balance_new.md`。
