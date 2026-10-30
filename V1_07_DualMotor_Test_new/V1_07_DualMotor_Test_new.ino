/*
  Independent two-wheel motor test.
  Support the chassis with both wheels off the floor before reset or power-up.
  Both wheels move during FOC alignment, then run forward at 0.5 V until X.
*/
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

namespace {
constexpr int LEFT_IN1 = 32, LEFT_IN2 = 33, LEFT_IN3 = 25, LEFT_EN = 12;
constexpr int RIGHT_IN1 = 14, RIGHT_IN2 = 27, RIGHT_IN3 = 26, RIGHT_EN = 13;
constexpr int LEFT_I2C_SDA = 21, LEFT_I2C_SCL = 22;
constexpr int RIGHT_I2C_SDA = 19, RIGHT_I2C_SCL = 18;
constexpr int POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float MOTOR_VOLTAGE_LIMIT = 3.0f;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr float TEST_VOLTAGE = 0.5f;
constexpr float LEFT_MOTOR_SIGN = 1.0f;
constexpr float RIGHT_MOTOR_SIGN = -1.0f;
constexpr float MAX_WHEEL_SPEED = 30.0f;
constexpr uint32_t MAGNET_CHECK_PERIOD_MS = 100;
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

class CheckedAS5600 : public Sensor {
 public:
  enum class Failure { none, i2c, magnet };

  explicit CheckedAS5600(TwoWire& bus) : bus_(bus) {}

  void init() override { Sensor::init(); }

  bool magnetHealthy() {
    if (failed) {
      cutEnablePins();
      return false;
    }
    if (!readRegisters(bus_, 0x36, 0x0B, &magnetStatus, 1, diagnostic))
      return latchFailure(Failure::i2c);
    if ((magnetStatus & 0x38) != 0x20) return latchFailure(Failure::magnet);
    failure = Failure::none;
    return true;
  }

  float getSensorAngle() override {
    if (failed) {
      cutEnablePins();
      return -1.0f;
    }
    uint8_t data[2];
    if (!readRegisters(bus_, 0x36, 0x0C, data, 2, diagnostic)) {
      latchFailure(Failure::i2c);
      return -1.0f;
    }
    return (((data[0] & 0x0F) << 8) | data[1]) * (_2PI / 4096.0f);
  }

  bool failed = false;
  Failure failure = Failure::none;
  I2cReadDiagnostic diagnostic;
  uint8_t magnetStatus = 0;

 private:
  TwoWire& bus_;

  bool latchFailure(Failure reason) {
    failed = true;
    failure = reason;
    cutEnablePins();
    return false;
  }
};

CheckedAS5600 leftSensor(Wire), rightSensor(Wire1);
BLDCMotor leftMotor(POLE_PAIRS), rightMotor(POLE_PAIRS);
BLDCDriver3PWM leftDriver(LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_EN);
BLDCDriver3PWM rightDriver(RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_EN);
bool leftLinked = false, rightLinked = false;
bool running = false, faultLatched = false;
uint32_t lastMagnetCheckMs = 0, lastTelemetryMs = 0;

void printEncoderDiagnostic(const char* side, int sda, int scl,
                            const CheckedAS5600& sensor) {
  Serial.printf("AS5600 %s SDA%d/SCL%d addr=0x36: ", side, sda, scl);
  if (sensor.failure == CheckedAS5600::Failure::i2c) {
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

  const uint8_t status = sensor.magnetStatus;
  const char* reason = "OK";
  if (sensor.failure == CheckedAS5600::Failure::magnet) {
    reason = (status & 0x18) == 0x18 ? "MAGNET_STATUS_INVALID" :
             (status & 0x10) ? "MAGNET_TOO_WEAK" :
             (status & 0x08) ? "MAGNET_TOO_STRONG" : "MAGNET_NOT_DETECTED";
  }
  Serial.printf("%s STATUS=0x%02X MD=%d ML=%d MH=%d\n", reason, unsigned(status),
                (status & 0x20) != 0, (status & 0x10) != 0, (status & 0x08) != 0);
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

bool checkEncoderMagnets(bool logHealthy = false) {
  const bool leftHealthy = leftSensor.magnetHealthy();
  const bool rightHealthy = rightSensor.magnetHealthy();
  if (!leftHealthy || !rightHealthy) {
    fault("AS5600 check failed; see LEFT/RIGHT details below");
    printEncoderDiagnostics(true);
    return false;
  }
  if (logHealthy) printEncoderDiagnostics(true);
  return true;
}

bool initializeMotor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                     CheckedAS5600& sensor, bool& linked, const char* label) {
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
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) return false;

  Serial.printf("%s FOC alignment: keep wheel free; it WILL move.\n", label);
  const bool aligned = motor.initFOC();
  motor.disable();
  motor.target = 0;
  motor.voltage.q = motor.voltage.d = 0;
  if (!aligned || sensor.failed) return false;

  Serial.printf("%s ready: sensor_direction=%d, electrical_zero=%.4f\n",
                label, static_cast<int>(motor.sensor_direction), motor.zero_electric_angle);
  return true;
}

void processSerial() {
  for (int n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char c = Serial.read();
    if (c == 'X' || c == 'x') {
      stopMotors("serial X");
      return;
    }
  }
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
  delay(500);
  Serial.println("V1_07_DualMotor_Test_new / ESP32 / SimpleFOC 2.3.3");
  Serial.println("Support chassis; both wheels must be off the floor.");
  Serial.println("Left PWM32/33/25 EN12 I2C21/22; Right PWM14/27/26 EN13 I2C19/18");
  Serial.println("After checks and FOC alignment both wheels run at 0.5 V; X stops until reset.");

  if (!Wire.begin(LEFT_I2C_SDA, LEFT_I2C_SCL, 400000) ||
      !Wire1.begin(RIGHT_I2C_SDA, RIGHT_I2C_SCL, 400000)) {
    fault("I2C initialization");
    return;
  }
  Wire.setTimeOut(3);
  Wire1.setTimeOut(3);

  if (!checkEncoderMagnets(true)) return;
  leftSensor.init();
  rightSensor.init();
  if (leftSensor.failed || rightSensor.failed) {
    fault("AS5600 angle initialization");
    printEncoderDiagnostics();
    return;
  }

  if (!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT") ||
      !initializeMotor(rightMotor, rightDriver, rightSensor, rightLinked, "RIGHT")) {
    fault("motor alignment; check phases, magnets, supply and pole pairs");
    printEncoderDiagnostics();
    return;
  }

  leftMotor.enable();
  rightMotor.enable();
  running = true;
  leftMotor.move(LEFT_MOTOR_SIGN * TEST_VOLTAGE);
  rightMotor.move(RIGHT_MOTOR_SIGN * TEST_VOLTAGE);
  lastMagnetCheckMs = lastTelemetryMs = millis();
  Serial.println("BOTH MOTORS RUNNING FORWARD AT 0.5 V; X stops until reset.");
}

void loop() {
  processSerial();
  if (!running || faultLatched) {
    delay(2);
    return;
  }

  leftMotor.loopFOC();
  if (leftSensor.failed) {
    fault("LEFT AS5600 I2C read");
    printEncoderDiagnostics();
    return;
  }
  rightMotor.loopFOC();
  if (rightSensor.failed) {
    fault("RIGHT AS5600 I2C read");
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

  if (millis() - lastMagnetCheckMs >= MAGNET_CHECK_PERIOD_MS) {
    lastMagnetCheckMs = millis();
    if (!checkEncoderMagnets()) return;
  }
  printTelemetry();
}
