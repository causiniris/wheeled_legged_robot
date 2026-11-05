# 已验证双电机版本
基础提交 3a8c26d0d5fd42f4dd1eec1d0a37b39027a43fe3。
实际接线：左MT6701与MPU6050共享SDA21/SCL22，右MT6701 SDA19/SCL18。
I2C 100kHz、超时10ms，电机关闭时启动最多重试5次，运行中读取失败关闭两侧。
默认dual_motor_test_new，12V电源、7极对，校准后自动输出0.5V转矩电压，X停止直到复位。
实车已验证悬空双轮转动，未完成落地带载及自平衡验证。
电脑曲线：python -m pip install -r pc/requirements.txt，随后python pc/motor_curves.py。
固件编译和8项双电机模拟检查通过。平衡及sensor_viewer此前编译通过，当前I2C速率修改后未重新验证这两个环境。
上传补丁包不含缓存、二进制、串口日志和Python依赖副本。
platformio.ini将本机../../work缓存路径改为用户目录短路径，方便其他电脑使用。
