/*
  AS5600 闭环空载测速（new）
  ESP32 + AS5600 + SimpleFOC mini + 三相无刷电机。
  AS5600：3.3V/GND/SDA=21/SCL=22；驱动：IN1=32/IN2=33/IN3=25/EN=12。
  SVPWM，载波设置 10000 Hz，实测速度每 100 ms 输出一次。
  示例参数：7 极对、12 V 电源、3 V 电压限幅；需按实际电机和电源确认。
  上电后 initFOC() 会执行转子对齐，期间电机可能转动。
  串口 115200，换行结束：T3、T6.28、T0、T-6.28、L4、L5。
  输出 target/actual 单位 rad/s；rpm 是带方向符号的实测转速。
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

constexpr int POLE_PAIRS = 7;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float INITIAL_VOLTAGE_LIMIT = 3.0f;
constexpr uint32_t PRINT_INTERVAL_MS = 100;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
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
  // 电压模式速度环的 PID 限幅在 init() 时设置；L 命令后同步更新。
  motor.PID_velocity.limit = motor.voltage_limit;
}

void setup() {
  Serial.setTxBufferSize(256);
  Serial.begin(115200);
  delay(1000);
  Serial.println("V1_03_ClosedLoop_Velocity_Test_new");
  Serial.printf("SVPWM, carrier_setting=%ld Hz, pole_pairs=%d\n",
                PWM_FREQUENCY_HZ, POLE_PAIRS);
  Serial.println("SDA=21, SCL=22; IN1=32, IN2=33, IN3=25, EN=12");
  SimpleFOCDebug::enable(&Serial);

  Wire.begin(21, 22);
  Wire.beginTransmission(0x36);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not found at 0x36. Check wiring, then reset ESP32.");
    return;
  }
  sensor.init(&Wire);
  sensor.update();
  if (sensor.currWireError != 0) {
    Serial.println("AS5600 I2C read failed. Check wiring, then reset ESP32.");
    return;
  }
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;  // 在 driver.init() 之前。
  if (!driver.init()) {
    Serial.println("Driver init failed. Check configuration and reset.");
    return;
  }
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = INITIAL_VOLTAGE_LIMIT;
  motor.velocity_limit = 20.0f;
  motor.voltage_sensor_align = 3.0f;
  motor.PID_velocity.P = 0.15f;
  motor.PID_velocity.I = 1.0f;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = 0.02f;
  motor.target = 0.0f;

  motor.init();
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
    motor.disable();
    Serial.println("Motor init failed. Reset after checking configuration.");
    return;
  }
  Serial.println("Running initFOC: rotor alignment may move the motor.");
  if (!motor.initFOC()) {
    motor.disable();
    Serial.println("FOC init failed. Check wiring/pole pairs, then reset.");
    return;
  }

  command.add('T', doTarget, "target velocity (rad/s)");
  command.add('L', doLimit, "voltage limit (V)");
  motorReady = true;
  Serial.println("Ready: T3 / T6.28 / T0 / T-6.28; L4 / L5. Use newline.");
  Serial.println("Telemetry every 100 ms: target(rad/s), actual(rad/s), rpm.");
  lastPrint = millis();
}

void loop() {
  if (!motorReady) {
    delay(1);
    return;
  }

  // FOC 连续运行，仅串口输出限频；禁止在正常控制循环内 delay(100)。
  motor.loopFOC();
  if (sensor.currWireError != 0) {
    motor.disable();
    motorReady = false;
    Serial.println("AS5600 I2C error. Motor disabled; check wiring and reset.");
    return;
  }
  motor.move();
  command.run();

  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    const float rpm = motor.shaft_velocity * 60.0f / _2PI;
    Serial.printf("target:%.2f\tactual:%.2f\trpm:%.2f\n",
                  motor.target, motor.shaft_velocity, rpm);
  }
}
