/*
  三相驱动波形 / 开环速度测试（new）
  ESP32 + SimpleFOC mini 3PWM 驱动板。
  IN1=32，IN2=33，IN3=25，EN=12，各设备共地。
  SVPWM，载波设置为 10000 Hz；示例电源 12 V、极对数 7，需匹配实物。
  串口 115200，命令以换行结束：T6.28、T0、T-6.28、L4、L5。
  基波频率 = |目标机械角速度| * 极对数 / (2*pi)。
  开环输出的是目标值推算频率，不是编码器实测转速。
  T0 停止旋转磁场，但仍可能保持静态励磁，不等于驱动断电。
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>

// 根据实际电机修改此常量；电机对象和频率计算使用同一个极对数。
constexpr int POLE_PAIRS = 7;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float INITIAL_VOLTAGE_LIMIT = 3.0f;
// Print once per second so serial commands remain easy to enter and read.
constexpr uint32_t PRINT_INTERVAL_MS = 1000;

BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);
Commander command = Commander(Serial);
bool motorReady = false;
uint32_t lastPrint = 0;

void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
}

void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {
  Serial.setTxBufferSize(256);
  Serial.begin(115200);
  delay(1000);
  Serial.println("V1_02_2804_OpenLoop_Test_new");
  Serial.printf("SVPWM, carrier_setting=%ld Hz, pole_pairs=%d\n",
                PWM_FREQUENCY_HZ, POLE_PAIRS);
  Serial.println("IN1=GPIO32, IN2=GPIO33, IN3=GPIO25, EN=GPIO12");
  SimpleFOCDebug::enable(&Serial);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  // 必须在 driver.init() 前设置，初始化时才会配置 PWM 定时器。
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) {
    Serial.println("Driver init failed. Check configuration and reset.");
    return;
  }
  motor.linkDriver(&driver);

  motor.voltage_limit = INITIAL_VOLTAGE_LIMIT;
  motor.controller = MotionControlType::velocity_openloop;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.target = 0.0f;

  // SimpleFOC 2.3.3 的 init() 返回 void，通过状态确认初始化结果。
  motor.init();
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
    motor.disable();
    Serial.println("Motor init failed. Reset after checking configuration.");
    return;
  }
  command.add('T', doTarget, "target velocity (rad/s)");
  command.add('L', doLimit, "voltage limit (V)");
  motorReady = true;
  Serial.println("Ready: T6.28 / T0 / T-6.28; L4 / L5. Use newline.");
  Serial.println("base_hz is calculated from target, NOT measured rotor speed.");
  lastPrint = millis();
}

void loop() {
  if (!motorReady) {
    delay(1);
    return;
  }
  motor.loopFOC();
  motor.move();
  command.run();

  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    const float baseFreq = fabsf(motor.target) * motor.pole_pairs / _2PI;
    const float targetRpm = motor.target * 60.0f / _2PI;
    Serial.printf("target:%.2f\ttarget_rpm:%.2f\tbase_hz:%.3f\n",
                  motor.target, targetRpm, baseFreq);
  }
}
