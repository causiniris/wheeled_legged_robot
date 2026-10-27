# 验证记录 `_new`

日期：2026-10-27。

| 验证项目 | 结果 |
|---|---|
| 独立工程 `mcdgf/Code/ESP32_Balance_Car_new`，PlatformIO `balance_car_new` | SUCCESS |
| 原作业工程，PlatformIO `balance_car_new` | SUCCESS |
| 原作业工程，PlatformIO `closedloop_velocity_test_new` 回归 | SUCCESS |
| 主机 C++ 行为测试（g++，C++17，Wall/Wextra/Werror） | PASS |
| 两处交付的固件源码与测试文件 SHA256 一致性 | PASS |
| GitHub 上游已跟踪文件改动 | 无；仅增加 `Code/ESP32_Balance_Car_new` |

构建配置：Espressif32 7.1.3、esp32dev、Arduino-ESP32 2.0.17、SimpleFOC 2.3.3。

新版编译统计：RAM 23,668 字节（7.2%），Flash 324,829 字节（24.8%）。

主机测试直接包含实际 `BalanceController_new.h`，替换 Arduino/Wire/SimpleFOC 接口为模拟对象。覆盖初始化失败时空驱动保护、旧 IMU 读数拒绝使能、X 停机、200 ms 单轮测试、编码器与 MPU6050 短读、故障锁定、磁铁丢失、超时、过速、过倾，以及融合角跨界、PD 输出正负与限幅、速度环抗积分饱和。

复现命令：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e balance_car_new
.\tests\balance_car_new\run_new.ps1
```

没有执行 Upload，没有连接真实传感器/电机，没有验证任何实车能站稳的结论。模拟接口测试不验证真实电机电流、机械负载、I2C 电气质量或闭环稳定裕量；实际表现仍需按 `README_balance_new.md` 执行方向确认和调参。
