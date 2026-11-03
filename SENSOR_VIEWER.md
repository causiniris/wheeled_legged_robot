# MT6701 和 MPU6050 串口曲线

`sensor_viewer` 是独立的传感器诊断固件：EN12/EN13 保持低电平，不主动驱动电机。左 MT6701 和 MPU6050 共用 SDA21/SCL22，右 MT6701 为 SDA19/SCL18，MPU6050 的 AD0 接地、预期地址为 `0x68`。供电和信号上拉请查看 [WIRING_I2C_new.md](WIRING_I2C_new.md)。

1. 架空车轮，在 PlatformIO 中选择 `sensor_viewer → General → Upload`。此操作会将 ESP32 现有固件替换为诊断固件。
2. 烧录结束后关闭 PlatformIO Monitor，避免占用串口。打开 VS Code 终端，执行：

   ```powershell
   python -m pip install pyserial matplotlib
   python tools/sensor_viewer.py --port COM3
   ```

   将 `COM3` 替换为 ESP32 的实际端口；若电脑只有一个串口，也可以省略 `--port COM3`。不知道端口时运行 `python -m serial.tools.list_ports -v`。窗口关闭后脚本会释放串口。
3. 画面从上至下显示两只 MT6701 的绝对角度（0–360°）、MPU6050 三轴加速度（g）、三轴角速度（度/秒）；窗口底部显示每个设备的 OK/ERROR。角度经过 360° 回到 0° 时跳变属于正常现象。数据约每 50 ms 更新，默认显示最近 20 秒；可以用 `--seconds 60` 调整。

启动时诊断固件扫描两条总线，并在串口输出 `# LEFT GPIO21/22 I2C scan: ...` 与 `# RIGHT GPIO19/18 I2C scan: ...`。正常情况下左侧包含 `0x06` 和 `0x68`，右侧包含 `0x06`。扫描和错误信息会显示在运行 Python 的终端；需要重新显示启动扫描结果时可复位 ESP32。`0x69` 代表有设备应答该地址，不单凭扫描证明设备型号；当前读取 MPU 仍使用 `0x68`。

`nan` / ERROR 表示本次数据读取失败，不代表 0；MT6701 数据仅表示读到角度，不证明磁场或安装状态正常。三轴值是按照 ±2 g、±250 °/s 换算的原始值，未做安装方向变换、零偏校准或姿态融合。MPU 若无应答，程序会每秒重试，不影响两路编码器显示。

诊断完成后，重新烧录 `balance_car_new` 才能使用平衡车程序；在传感器、方向与保护经过实际检查和调试前不要落地平衡。
