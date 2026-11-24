/*
  Independent two-wheel motor test.
  Support the chassis with both wheels off the floor before reset or power-up.
  Both wheels move during FOC alignment, then run forward at 0.5 V until X.
*/
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <atomic>
#include <math.h>

namespace {
constexpr int LEFT_IN1 = 32, LEFT_IN2 = 33, LEFT_IN3 = 25, LEFT_EN = 12;
constexpr int RIGHT_IN1 = 14, RIGHT_IN2 = 27, RIGHT_IN3 = 26, RIGHT_EN = 13;
constexpr int LEFT_I2C_SDA = 21, LEFT_I2C_SCL = 22;
constexpr int RIGHT_I2C_SDA = 17, RIGHT_I2C_SCL = 16;
constexpr int POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float MOTOR_VOLTAGE_LIMIT = 3.0f;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr float TEST_VOLTAGE = 0.5f;
constexpr float LEFT_MOTOR_SIGN = 1.0f;
constexpr float RIGHT_MOTOR_SIGN = -1.0f;
constexpr float MAX_WHEEL_SPEED = 30.0f;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;

static_assert(LEFT_MOTOR_SIGN * LEFT_MOTOR_SIGN == 1.0f, "Left sign must be +/-1");
static_assert(RIGHT_MOTOR_SIGN * RIGHT_MOTOR_SIGN == 1.0f, "Right sign must be +/-1");

void cutEnablePins() {
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
}

struct I2cReadDiagnostic {
  uint8_t reg = 0, expected = 0, writeCode = 0;
  size_t received = 0;
  int available = 0;
};

bool readRegisters(TwoWire& bus, uint8_t address, uint8_t reg,
                   uint8_t* data, uint8_t length, I2cReadDiagnostic& diagnostic) {
  diagnostic = {};
  diagnostic.reg = reg;
  diagnostic.expected = length;
  bus.beginTransmission(address);
  bus.write(reg);
  diagnostic.writeCode = bus.endTransmission(false);
  if (diagnostic.writeCode != 0) return false;

  diagnostic.received = bus.requestFrom(address, length, static_cast<uint8_t>(true));
  diagnostic.available = bus.available();
  if (diagnostic.received != length || diagnostic.available < length) {
    while (bus.available()) bus.read();
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) data[i] = bus.read();
  return true;
}

class CheckedMT6701 : public Sensor {
 public:
  enum class Failure { none, i2c };

  explicit CheckedMT6701(TwoWire& bus) : bus_(bus) {}

  void init() override { Sensor::init(); }

  float getSensorAngle() override {
    if (failed) {
      cutEnablePins();
      return -1.0f;
    }
    uint8_t data[2];
    if (!readRegisters(bus_, 0x06, 0x03, &data[0], 1, diagnostic) ||
        !readRegisters(bus_, 0x06, 0x04, &data[1], 1, diagnostic)) {
      latchFailure(Failure::i2c);
      return -1.0f;
    }
    return ((static_cast<uint16_t>(data[0]) << 6) | (data[1] >> 2)) * (_2PI / 16384.0f);
  }

  bool failed = false;
  Failure failure = Failure::none;
  I2cReadDiagnostic diagnostic;

 private:
  TwoWire& bus_;

  bool latchFailure(Failure reason) {
    failed = true;
    failure = reason;
    cutEnablePins();
    return false;
  }
};

CheckedMT6701 leftSensor(Wire), rightSensor(Wire1);
BLDCMotor leftMotor(POLE_PAIRS), rightMotor(POLE_PAIRS);
BLDCDriver3PWM leftDriver(LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_EN);
BLDCDriver3PWM rightDriver(RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_EN);
bool leftLinked = false, rightLinked = false;
bool running = false, faultLatched = false;
std::atomic<bool> stopRequested{false};
uint32_t lastTelemetryMs = 0;

void printEncoderDiagnostic(const char* side, int sda, int scl,
                            const CheckedMT6701& sensor) {
  Serial.printf("MT6701 %s SDA%d/SCL%d addr=0x06: ", side, sda, scl);
  if (sensor.failure == CheckedMT6701::Failure::i2c) {
    const I2cReadDiagnostic& d = sensor.diagnostic;
    if (d.writeCode != 0) {
      const char* reason = d.writeCode == 2 ? "address NACK" :
                           d.writeCode == 3 ? "data NACK" :
                           d.writeCode == 4 ? "bus error" :
                           d.writeCode == 5 ? "timeout" : "Wire error";
      Serial.printf("I2C_WRITE_FAILED reg=0x%02X wire_code=%u (%s)\n",
                    unsigned(d.reg), unsigned(d.writeCode), reason);
    } else {
      Serial.printf("I2C_SHORT_READ reg=0x%02X received=%u available=%d expected=%u\n",
                    unsigned(d.reg), unsigned(d.received), d.available, unsigned(d.expected));
    }
    return;
  }

  Serial.println("angle read OK (magnetic field not checked)");
}

void printEncoderDiagnostics(bool includeHealthy = false) {
  if (includeHealthy || leftSensor.failed)
    printEncoderDiagnostic("LEFT", LEFT_I2C_SDA, LEFT_I2C_SCL, leftSensor);
  if (includeHealthy || rightSensor.failed)
    printEncoderDiagnostic("RIGHT", RIGHT_I2C_SDA, RIGHT_I2C_SCL, rightSensor);
}

void stopMotors(const char* reason) {
  const bool wasRunning = running;
  cutEnablePins();
  if (leftLinked) {
    leftMotor.disable();
    leftMotor.target = 0;
    leftMotor.voltage.q = leftMotor.voltage.d = 0;
  }
  if (rightLinked) {
    rightMotor.disable();
    rightMotor.target = 0;
    rightMotor.voltage.q = rightMotor.voltage.d = 0;
  }
  running = false;
  if (wasRunning) Serial.printf("STOP: %s\n", reason);
}

void fault(const char* reason) {
  stopMotors(reason);
  if (!faultLatched) Serial.printf("FAULT: %s. Reset after inspection.\n", reason);
  faultLatched = true;
}

void pollStopRequest() {
  for (int n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char c = Serial.read();
    if (c == 'X' || c == 'x') {
      stopRequested.store(true, std::memory_order_release);
      cutEnablePins();
    }
  }
}

#if defined(ARDUINO_ARCH_ESP32)
void serialStopTask(void*) {
  for (;;) {
    pollStopRequest();
    if (stopRequested.load(std::memory_order_acquire)) cutEnablePins();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool startStopMonitor() {
  return xTaskCreatePinnedToCore(serialStopTask, "motor-stop", 2048, nullptr, 2,
                                nullptr, 0) == pdPASS;
}
#else
bool startStopMonitor() { return true; }
#endif

bool stopIfRequested() {
#if !defined(ARDUINO_ARCH_ESP32)
  pollStopRequest();
#endif
  if (!stopRequested.load(std::memory_order_acquire)) return false;
  stopMotors("serial X");
  return true;
}

bool initializeMotor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                     CheckedMT6701& sensor, bool& linked, const char* label) {
  if (stopIfRequested()) return false;
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) return false;

  motor.linkDriver(&driver);
  linked = true;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.voltage_sensor_align = 2.0f;
  motor.LPF_velocity.Tf = 0.02f;
  motor.init();
  if (stopIfRequested()) return false;
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) return false;

  Serial.printf("%s FOC alignment: keep wheel free; it WILL move.\n", label);
  const bool aligned = motor.initFOC();
  motor.disable();
  motor.target = 0;
  motor.voltage.q = motor.voltage.d = 0;
  if (stopIfRequested()) return false;
  if (!aligned || sensor.failed) return false;

  Serial.printf("%s ready: sensor_direction=%d, electrical_zero=%.4f\n",
                label, static_cast<int>(motor.sensor_direction), motor.zero_electric_angle);
  return true;
}

void printTelemetry() {
  if (millis() - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = millis();
  Serial.printf("state=RUN wl=%.3f wr=%.3f ul=%.2f ur=%.2f\n",
                LEFT_MOTOR_SIGN * leftMotor.shaft_velocity,
                RIGHT_MOTOR_SIGN * rightMotor.shaft_velocity,
                LEFT_MOTOR_SIGN * TEST_VOLTAGE,
                RIGHT_MOTOR_SIGN * TEST_VOLTAGE);
}
}  // namespace

void setup() {
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  cutEnablePins();

  Serial.setTxBufferSize(512);
  Serial.begin(115200);
  if (!startStopMonitor()) {
    fault("serial stop monitor task");
    return;
  }
  delay(500);
  if (stopIfRequested()) return;
  Serial.println("V1_07_DualMotor_Test_new / ESP32 / SimpleFOC 2.3.3");
  Serial.println("Support chassis; both wheels must be off the floor.");
  Serial.printf("Left PWM32/33/25 EN12 I2C21/22; Right PWM14/27/26 EN13 I2C%d/%d\n",
                RIGHT_I2C_SDA, RIGHT_I2C_SCL);
  Serial.println("After checks and FOC alignment both wheels run at 0.5 V; X stops until reset.");

  if (!Wire.begin(LEFT_I2C_SDA, LEFT_I2C_SCL, 400000) ||
      !Wire1.begin(RIGHT_I2C_SDA, RIGHT_I2C_SCL, 400000)) {
    fault("I2C initialization");
    return;
  }
  Wire.setTimeOut(3);
  Wire1.setTimeOut(3);

  leftSensor.init();
  rightSensor.init();
  if (leftSensor.failed || rightSensor.failed) {
    fault("MT6701 angle initialization");
    printEncoderDiagnostics(true);
    return;
  }
  printEncoderDiagnostics(true);

  if (!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT")) {
    if (stopRequested.load(std::memory_order_acquire)) return;
    fault("motor alignment; check phases, magnets, supply and pole pairs");
    printEncoderDiagnostics();
    return;
  }
  if (!initializeMotor(rightMotor, rightDriver, rightSensor, rightLinked, "RIGHT")) {
    if (stopRequested.load(std::memory_order_acquire)) return;
    fault("motor alignment; check phases, magnets, supply and pole pairs");
    printEncoderDiagnostics();
    return;
  }

  if (stopIfRequested()) return;
  leftMotor.enable();
  if (stopIfRequested()) return;
  rightMotor.enable();
  if (stopIfRequested()) return;
  running = true;
  leftMotor.move(LEFT_MOTOR_SIGN * TEST_VOLTAGE);
  rightMotor.move(RIGHT_MOTOR_SIGN * TEST_VOLTAGE);
  lastTelemetryMs = millis();
  Serial.println("BOTH MOTORS RUNNING FORWARD AT 0.5 V; X stops until reset.");
}

void loop() {
#if !defined(ARDUINO_ARCH_ESP32)
  pollStopRequest();
#endif
  if (stopRequested.load(std::memory_order_acquire)) stopMotors("serial X");
  if (!running || faultLatched) {
    delay(2);
    return;
  }

  leftMotor.loopFOC();
  if (leftSensor.failed) {
    fault("LEFT MT6701 I2C read");
    printEncoderDiagnostics();
    return;
  }
  rightMotor.loopFOC();
  if (rightSensor.failed) {
    fault("RIGHT MT6701 I2C read");
    printEncoderDiagnostics();
    return;
  }

  if (!isfinite(leftMotor.shaft_velocity) || !isfinite(rightMotor.shaft_velocity)) {
    fault("invalid wheel speed");
    return;
  }
  if (fabsf(leftMotor.shaft_velocity) > MAX_WHEEL_SPEED ||
      fabsf(rightMotor.shaft_velocity) > MAX_WHEEL_SPEED) {
    fault("wheel overspeed");
    return;
  }

  leftMotor.move(LEFT_MOTOR_SIGN * TEST_VOLTAGE);
  rightMotor.move(RIGHT_MOTOR_SIGN * TEST_VOLTAGE);

  printTelemetry();
}
