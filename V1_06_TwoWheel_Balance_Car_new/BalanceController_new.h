#pragma once
/*
  Two-wheel balance commissioning firmware (_new).
  Inspired by M创动工坊 / macolin007/mcdgf, commit fce0cbe12ae6b3b5d47ac5b12c97d7ae58bf952d.
  Uses the user's homework wiring and SimpleFOC 2.3.3, not the upstream drawings.
  STS3215 joints must already be mechanically supported / held in a fixed stance.
  No STS3215 packets are sent. This is balance from an upright pose, not self-righting.

  IMPORTANT: startup FOC alignment moves BOTH wheels before A is accepted.
  Support the chassis with wheels free, then hold it upright and still for IMU
  calibration. After Ready, L/R run short individual direction tests; A balances;
  X stops. See README_balance_new.md before putting the wheels on the floor.
*/
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>
#include "BalanceMath_new.h"
#include "SoftwareI2C_new.h"

namespace {
constexpr int LEFT_IN1 = 32, LEFT_IN2 = 33, LEFT_IN3 = 25, LEFT_EN = 12;
constexpr int RIGHT_IN1 = 14, RIGHT_IN2 = 27, RIGHT_IN3 = 26, RIGHT_EN = 13;
constexpr int LEFT_I2C_SDA = 21, LEFT_I2C_SCL = 22;
constexpr int RIGHT_I2C_SDA = 19, RIGHT_I2C_SCL = 18;
constexpr int MPU_I2C_SDA = 16, MPU_I2C_SCL = 17; // Dedicated software I2C.
constexpr uint8_t MPU6050_ADDRESS = 0x68;  // AD0=GND; use 0x69 if AD0=3.3V.
constexpr int POLE_PAIRS = 7;              // Inherited homework value; verify both motors.
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float MOTOR_VOLTAGE_LIMIT = 3.0f; // q-axis voltage, NOT a current limit.
constexpr long PWM_FREQUENCY_HZ = 10000;

// Configure the IMU rotation axis parallel to the wheel axle. Default: X axle,
// Z points approximately up. If the axle is parallel to sensor Y, choose 'Y'.
// The sign must make a physical forward lean read positive pitch AND rate.
constexpr char IMU_PITCH_AXIS = 'X';
constexpr float IMU_PITCH_SIGN = 1.0f;
// Each positive signed wheel command must propel the chassis forward.
// These are installation assumptions, verified separately by L/R commands.
constexpr float LEFT_MOTOR_SIGN = 1.0f;
constexpr float RIGHT_MOTOR_SIGN = -1.0f;
static_assert(IMU_PITCH_AXIS == 'X' || IMU_PITCH_AXIS == 'Y', "Use X or Y IMU axis");
static_assert(IMU_PITCH_SIGN == 1.0f || IMU_PITCH_SIGN == -1.0f, "IMU sign must be +/-1");
static_assert(LEFT_MOTOR_SIGN * LEFT_MOTOR_SIGN == 1.0f, "Left sign must be +/-1");
static_assert(RIGHT_MOTOR_SIGN * RIGHT_MOTOR_SIGN == 1.0f, "Right sign must be +/-1");

// Starting gains, not a measured tune for this chassis. Units: V/rad and V/(rad/s).
// Upstream also uses a 2.9-ohm phase-resistance conversion; this firmware DOES NOT
// assume that resistance, so the same numerical gains are not an equivalent tune.
constexpr float INITIAL_ANGLE_KP = 8.8f, INITIAL_ANGLE_KD = 0.29f;
constexpr float SPEED_KP = 0.20f; // deg / (wheel rad/s)
constexpr float SPEED_KI = 0.05f; // deg / wheel rad, optional slow speed loop
constexpr float MAX_SPEED_LEAN_DEG = 3.0f;
constexpr float MAX_TILT_DEG = 35.0f, ARM_TILT_DEG = 8.0f;
constexpr float MAX_WHEEL_SPEED = 30.0f; // rad/s; independent of SimpleFOC velocity_limit.
constexpr float TEST_VOLTAGE = 0.5f;
constexpr uint32_t TEST_DURATION_MS = 200;
constexpr uint32_t BALANCE_PERIOD_US = 5000, MAX_CONTROL_GAP_US = 30000;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr float FILTER_TIME_CONSTANT = 0.245f; // alpha=0.98 at 5 ms; handles variable dt.
constexpr float GYRO_SCALE = 131.0f; // +/-250 deg/s

void cutEnablePins() {
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
}

bool readRegisters(TwoWire& bus, uint8_t address, uint8_t reg,
                   uint8_t* data, uint8_t length) {
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  const size_t received = bus.requestFrom(address, length, static_cast<uint8_t>(true));
  if (received != length || bus.available() < length) {
    while (bus.available()) bus.read();
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) data[i] = bus.read();
  return true;
}

// SimpleFOC 2.3.3's stock I2C sensor checks the register-address write but not
// a short data read. This adapter rejects both and keeps errors latched.
class CheckedAS5600 : public Sensor {
 public:
  explicit CheckedAS5600(TwoWire& bus) : bus_(bus) {}
  void init() override { Sensor::init(); }
  bool failed = false;
  bool magnetHealthy() {
    uint8_t status = 0;
    if (!readRegisters(bus_, 0x36, 0x0B, &status, 1) || (status & 0x38) != 0x20) {
      failed = true;
      cutEnablePins();
      return false;
    }
    return !failed;
  }
  float getSensorAngle() override {
    uint8_t data[2];
    if (failed || !readRegisters(bus_, 0x36, 0x0C, data, 2)) {
      failed = true;
      cutEnablePins();
      return -1.0f; // Sensor::update ignores this sample instead of inventing an angle.
    }
    return (((data[0] & 0x0F) << 8) | data[1]) * (_2PI / 4096.0f);
  }
 private:
  TwoWire& bus_;
};

CheckedAS5600 leftSensor(Wire), rightSensor(Wire1);
SoftwareI2C_new mpuBus(MPU_I2C_SDA, MPU_I2C_SCL);
BLDCMotor leftMotor(POLE_PAIRS), rightMotor(POLE_PAIRS);
BLDCDriver3PWM leftDriver(LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_EN);
BLDCDriver3PWM rightDriver(RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_EN);
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; };
float gyroBias = 0, pitchAngleDeg = 0, pitchRateDegPerSec = 0;
float balanceSetpointDeg = 0, calibratedUprightDeg = 0, balanceVoltage = 0;
float angleKp = INITIAL_ANGLE_KP, angleKd = INITIAL_ANGLE_KD;
float speedFiltered = 0, speedIntegral = 0, speedLean = 0;
bool leftLinked = false, rightLinked = false, motorsReady = false;
bool imuReady = false, faultLatched = false, armed = false, speedAssist = false;
uint8_t testWheel = 0;
uint32_t testStartMs = 0, lastControlUs = 0, lastImuUs = 0;
uint32_t lastTelemetryMs = 0, lastMagnetCheckMs = 0;
uint32_t lastFocUs = 0, maxFocGapUs = 0;

bool driving() { return armed || testWheel != 0; }
void stopMotors(const char* reason) {
  const bool wasDriving = driving();
  cutEnablePins(); // Cut power-stage enable before any library or serial call.
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
  armed = false;
  testWheel = 0;
  balanceVoltage = speedIntegral = speedLean = 0;
  if (wasDriving) Serial.printf("STOP: %s\n", reason);
}
void fault(const char* reason) {
  stopMotors(reason);
  if (!faultLatched) Serial.printf("FAULT: %s. Check wiring/sensors, then reset.\n", reason);
  faultLatched = true;
}
int16_t signed16(const uint8_t* data) {
  return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}
float rawGyro(const ImuSample& s) { return IMU_PITCH_AXIS == 'X' ? s.gx : s.gy; }
float accelPitch(const ImuSample& s) {
  const float horizontal = IMU_PITCH_AXIS == 'X' ? s.ay : -static_cast<float>(s.ax);
  return BalanceMath::wrapDeg(IMU_PITCH_SIGN * atan2f(horizontal, s.az) / BalanceMath::RAD_PER_DEG);
}
bool readImu(ImuSample& s) {
  uint8_t data[14];
  if (!mpuBus.readRegisters(MPU6050_ADDRESS, 0x3B, data, 14)) return false;
  s.ax = signed16(data); s.ay = signed16(data + 2); s.az = signed16(data + 4);
  s.gx = signed16(data + 8); s.gy = signed16(data + 10); s.gz = signed16(data + 12);
  const float norm2 = float(s.ax) * s.ax + float(s.ay) * s.ay + float(s.az) * s.az;
  const float plane2 = (IMU_PITCH_AXIS == 'X' ? float(s.ay) * s.ay : float(s.ax) * s.ax)
                       + float(s.az) * s.az;
  // Reject zero/free-fall data, saturated sensors and an almost-vertical axle.
  return norm2 > 2684354.0f && plane2 > 2684354.0f &&
         abs(int(s.ax)) < 32700 && abs(int(s.ay)) < 32700 && abs(int(s.az)) < 32700 &&
         abs(int(s.gx)) < 32700 && abs(int(s.gy)) < 32700 && abs(int(s.gz)) < 32700;
}
bool writeMpu(uint8_t reg, uint8_t value) {
  return mpuBus.writeRegister(MPU6050_ADDRESS, reg, value);
}
bool initializeImu() {
  uint8_t identity = 0;
  if (!mpuBus.readRegisters(MPU6050_ADDRESS, 0x75, &identity, 1) || identity != 0x68) return false;
  if (!writeMpu(0x6B, 0x01) || // wake, X gyro PLL clock
      !writeMpu(0x1A, 0x03) || // DLPF 44 Hz gyro
      !writeMpu(0x19, 0x04) || // 1 kHz / (4 + 1) = 200 Hz
      !writeMpu(0x1B, 0x00) || // +/-250 deg/s
      !writeMpu(0x1C, 0x00)) return false; // +/-2 g
  delay(100);
  ImuSample s{};
  return readImu(s);
}
bool calibrateImu() {
  constexpr int SAMPLES = 300;
  double gyroSum = 0, gyroSquares = 0, sinSum = 0, cosSum = 0;
  float minAngle = 0, maxAngle = 0, firstAngle = 0;
  Serial.println("CALIBRATE: hold upright and STILL for about 2 seconds; motors are disabled.");
  for (int i = 0; i < SAMPLES; ++i) {
    ImuSample s{};
    if (!readImu(s)) return false;
    const float g = rawGyro(s), angle = accelPitch(s);
    const float norm = sqrtf(float(s.ax)*s.ax + float(s.ay)*s.ay + float(s.az)*s.az) / 16384.0f;
    if (norm < 0.85f || norm > 1.15f) return false;
    if (i == 0) firstAngle = angle;
    const float relative = BalanceMath::wrapDeg(angle - firstAngle);
    minAngle = fminf(minAngle, relative); maxAngle = fmaxf(maxAngle, relative);
    gyroSum += g; gyroSquares += double(g) * g;
    sinSum += sinf(angle * BalanceMath::RAD_PER_DEG);
    cosSum += cosf(angle * BalanceMath::RAD_PER_DEG);
    delay(5);
  }
  const double mean = gyroSum / SAMPLES;
  const double variance = fmax(0.0, gyroSquares / SAMPLES - mean * mean);
  if (sqrt(variance) / GYRO_SCALE > 0.8 || fabs(mean) / GYRO_SCALE > 10.0 ||
      maxAngle - minAngle > 2.0f) return false;
  gyroBias = mean;
  pitchAngleDeg = atan2f(sinSum, cosSum) / BalanceMath::RAD_PER_DEG;
  calibratedUprightDeg = balanceSetpointDeg = pitchAngleDeg;
  pitchRateDegPerSec = 0;
  lastImuUs = lastControlUs = micros();
  imuReady = true;
  Serial.printf("IMU ready: pitch=%.2f deg, bias=%.2f deg/s\n", pitchAngleDeg, gyroBias/GYRO_SCALE);
  return true;
}
bool updatePitch(float dt) {
  ImuSample s{};
  if (!readImu(s)) return false;
  pitchRateDegPerSec = IMU_PITCH_SIGN * (rawGyro(s) - gyroBias) / GYRO_SCALE;
  if (dt > MAX_CONTROL_GAP_US * 1.0e-6f) pitchAngleDeg = accelPitch(s);
  else pitchAngleDeg = BalanceMath::fuseDeg(pitchAngleDeg, pitchRateDegPerSec,
                                           accelPitch(s), dt, FILTER_TIME_CONSTANT);
  lastImuUs = micros();
  return isfinite(pitchAngleDeg) && isfinite(pitchRateDegPerSec);
}

bool initializeMotor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                     CheckedAS5600& sensor, bool& linked, const char* label) {
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) return false;
  motor.linkDriver(&driver);
  linked = true; // Only now is motor.disable() safe to call.
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

bool canStart() {
  if (driving()) return false;
  if (faultLatched || !motorsReady || !imuReady || leftSensor.failed || rightSensor.failed) {
    Serial.println("Cannot start: initialization/sensor fault; check and reset.");
    return false;
  }
  if (micros() - lastImuUs > 15000 || !isfinite(pitchAngleDeg) ||
      fabsf(BalanceMath::wrapDeg(pitchAngleDeg - balanceSetpointDeg)) > ARM_TILT_DEG ||
      fabsf(pitchRateDegPerSec) > 20.0f || fabsf(leftMotor.shaft_velocity) > 1.0f ||
      fabsf(rightMotor.shaft_velocity) > 1.0f) {
    Serial.println("Cannot start: hold upright, stop wheels, and wait for fresh IMU readings.");
    return false;
  }
  if (!leftSensor.magnetHealthy() || !rightSensor.magnetHealthy()) {
    fault("AS5600 magnet or I2C failure"); return false;
  }
  speedIntegral = speedFiltered = speedLean = balanceVoltage = 0;
  lastFocUs = maxFocGapUs = 0;
  leftMotor.voltage.q = leftMotor.voltage.d = rightMotor.voltage.q = rightMotor.voltage.d = 0;
  leftMotor.target = rightMotor.target = 0;
  return true;
}
void showSettings() {
  Serial.printf("Kp=%.2f Kd=%.2f speedAssist=%d set=%.2f axis=%c signs=%.0f/%.0f/%.0f\n",
    angleKp, angleKd, speedAssist, balanceSetpointDeg, IMU_PITCH_AXIS,
    IMU_PITCH_SIGN, LEFT_MOTOR_SIGN, RIGHT_MOTOR_SIGN);
}
void processSerial() {
  // Bound serial work; no readString()/parseFloat() or unbounded serial draining.
  for (int n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char c = Serial.read();
    if (c == 'X' || c == 'x') { stopMotors("serial X"); return; }
    if (c == '\n' || c == '\r' || c == ' ') continue;
    if (driving()) continue; // Only X is acted on while a test/balance is running.
    if (c == 'A' || c == 'a') {
      if (canStart()) {
        Serial.println("BALANCE ARMED; X stops.");
        leftMotor.enable(); rightMotor.enable(); armed = true;
      }
      return;
    }
    if (c == 'L' || c == 'l' || c == 'R' || c == 'r') {
      if (canStart()) {
        Serial.println("Wheel test: 0.5 V for 200 ms; wheels must be off the floor. X stops.");
        testWheel = (c == 'L' || c == 'l') ? 1 : 2;
        testStartMs = millis();
        if (testWheel == 1) leftMotor.enable(); else rightMotor.enable();
      }
      return;
    }
    if (c == 'C' || c == 'c') {
      if (!faultLatched && motorsReady) {
        stopMotors("recalibration");
        if (!calibrateImu()) fault("IMU calibration failed / chassis moved");
      }
      return;
    }
    if (c == '+') balanceSetpointDeg = BalanceMath::wrapDeg(calibratedUprightDeg +
      BalanceMath::clamp(BalanceMath::wrapDeg(balanceSetpointDeg-calibratedUprightDeg)+0.1f, -5, 5));
    else if (c == '-') balanceSetpointDeg = BalanceMath::wrapDeg(calibratedUprightDeg +
      BalanceMath::clamp(BalanceMath::wrapDeg(balanceSetpointDeg-calibratedUprightDeg)-0.1f, -5, 5));
    else if (c == '1') angleKp = fminf(60.0f, angleKp + 0.5f);
    else if (c == '0') angleKp = fmaxf(0.0f, angleKp - 0.5f);
    else if (c == '3') angleKd = fminf(5.0f, angleKd + 0.02f);
    else if (c == '2') angleKd = fmaxf(0.0f, angleKd - 0.02f);
    else if (c == 'V' || c == 'v') speedAssist = !speedAssist;
    else if (c != '?') continue;
    showSettings();
    return; // At most one reply per control iteration.
  }
}
void updateBalanceControl(float dt) {
  const float wheelSpeed = 0.5f * (LEFT_MOTOR_SIGN * leftMotor.shaft_velocity +
                                 RIGHT_MOTOR_SIGN * rightMotor.shaft_velocity);
  speedFiltered += dt / (0.10f + dt) * (wheelSpeed - speedFiltered);
  speedLean = speedAssist ? BalanceMath::speedLeanDeg(speedFiltered, dt, SPEED_KP, SPEED_KI,
                                                     MAX_SPEED_LEAN_DEG, speedIntegral) : 0;
  balanceVoltage = BalanceMath::voltage(pitchAngleDeg, balanceSetpointDeg + speedLean,
                                        pitchRateDegPerSec, angleKp, angleKd, MOTOR_VOLTAGE_LIMIT);
  if (!isfinite(balanceVoltage)) fault("non-finite controller output");
}
void printTelemetry() {
  const uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  char line[192];
  const int length = snprintf(line, sizeof(line),
    "pitch=%.2f set=%.2f rate=%.2f wl=%.2f wr=%.2f u=%.2f lean=%.2f A=%d T=%u F=%d imu_us=%lu foc_us=%lu\n",
    pitchAngleDeg, balanceSetpointDeg, pitchRateDegPerSec,
    LEFT_MOTOR_SIGN * leftMotor.shaft_velocity, RIGHT_MOTOR_SIGN * rightMotor.shaft_velocity,
    balanceVoltage, speedLean, armed, testWheel, faultLatched,
    static_cast<unsigned long>(mpuBus.durationUs()), static_cast<unsigned long>(maxFocGapUs));
  // Drop telemetry if full; never wait for the UART while balancing.
  if (length > 0 && length < static_cast<int>(sizeof(line)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t*>(line), length);
    maxFocGapUs = 0;
  }
}
} // namespace

void setup() {
  // EN is active HIGH on SimpleFOC Mini. GPIO12 is a boot strap pin inherited
  // from the homework: external circuitry must not force it HIGH at ESP32 reset.
  digitalWrite(LEFT_EN, LOW); digitalWrite(RIGHT_EN, LOW);
  pinMode(LEFT_EN, OUTPUT); pinMode(RIGHT_EN, OUTPUT); cutEnablePins();
  Serial.setTxBufferSize(512);
  Serial.begin(115200);
  delay(500);
  Serial.println("V1_06_TwoWheel_Balance_Car_new / ESP32 / SimpleFOC 2.3.3");
  Serial.println("Left PWM 32/33/25 EN12 I2C21/22; Right PWM14/27/26 EN13 I2C19/18");
  Serial.println("MPU6050 ONLY: software I2C SDA16/SCL17, AD0=GND, pull-ups to 3.3V");
  Serial.println("Commands: L/R=short wheel test A=balance X=stop C=calibrate +/-=trim");
  Serial.println("Disarmed tuning: 1/0=Kp +/-0.5, 3/2=Kd +/-0.02, V=speed assist, ?=settings");
  if (!Wire.begin(LEFT_I2C_SDA, LEFT_I2C_SCL, 400000) ||
      !Wire1.begin(RIGHT_I2C_SDA, RIGHT_I2C_SCL, 400000)) {
    fault("I2C initialization"); return;
  }
  Wire.setTimeOut(3); Wire1.setTimeOut(3);
  if (!mpuBus.begin()) { fault(mpuBus.errorText()); return; }
  if (!leftSensor.magnetHealthy() || !rightSensor.magnetHealthy()) {
    fault("AS5600 missing or magnet too weak/strong"); return;
  }
  leftSensor.init(); rightSensor.init();
  if (leftSensor.failed || rightSensor.failed || !initializeImu()) {
    if (mpuBus.error() != SoftwareI2C_new::Error::none) Serial.println(mpuBus.errorText());
    fault("sensor initialization / MPU6050 identity"); return;
  }
  if (!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT") ||
      !initializeMotor(rightMotor, rightDriver, rightSensor, rightLinked, "RIGHT")) {
    fault("motor alignment; check phases, magnets, supply and pole pairs"); return;
  }
  motorsReady = true;
  stopMotors("startup");
  if (!calibrateImu()) { fault("IMU calibration failed / chassis moved"); return; }
  lastTelemetryMs = lastMagnetCheckMs = millis();
  showSettings();
  lastControlUs = micros();
  Serial.println("Ready but disarmed. Verify pitch and L/R direction before A.");
}

void loop() {
  if (!motorsReady || !imuReady || faultLatched) {
    processSerial();
    delay(2);
    return;
  }
  if (driving() && micros() - lastControlUs > MAX_CONTROL_GAP_US) {
    fault("control loop overdue"); return;
  }
  if (testWheel != 0 && millis() - testStartMs >= TEST_DURATION_MS) stopMotors("wheel test complete");

  // Sensor updates also run while disarmed, so wheel telemetry is never stale.
  const uint32_t focUs = micros();
  if (driving() && lastFocUs != 0)
    maxFocGapUs = (focUs - lastFocUs > maxFocGapUs) ? focUs - lastFocUs : maxFocGapUs;
  lastFocUs = focUs;
  leftMotor.loopFOC();
  if (leftSensor.failed) { fault("LEFT AS5600 I2C read"); return; }
  rightMotor.loopFOC();
  if (rightSensor.failed) { fault("RIGHT AS5600 I2C read"); return; }
  // move() refreshes shaft_velocity even while disabled, using the latest sensor.
  leftMotor.move(LEFT_MOTOR_SIGN * (armed ? balanceVoltage : (testWheel == 1 ? TEST_VOLTAGE : 0)));
  rightMotor.move(RIGHT_MOTOR_SIGN * (armed ? balanceVoltage : (testWheel == 2 ? TEST_VOLTAGE : 0)));
  if (!isfinite(leftMotor.shaft_velocity) || !isfinite(rightMotor.shaft_velocity)) {
    fault("invalid wheel speed"); return;
  }
  if (driving() && (fabsf(leftMotor.shaft_velocity) > MAX_WHEEL_SPEED ||
                    fabsf(rightMotor.shaft_velocity) > MAX_WHEEL_SPEED)) stopMotors("wheel overspeed");

  const uint32_t nowUs = micros(), elapsed = nowUs - lastControlUs;
  if (elapsed >= BALANCE_PERIOD_US) {
    if (driving() && elapsed > MAX_CONTROL_GAP_US) { fault("control loop overdue"); return; }
    lastControlUs = nowUs;
    const float dt = elapsed * 1.0e-6f;
    if (!updatePitch(dt)) {
      fault(mpuBus.error() == SoftwareI2C_new::Error::none ? "MPU6050 data range" : mpuBus.errorText());
      return;
    }
    if (driving() && fabsf(BalanceMath::wrapDeg(pitchAngleDeg-calibratedUprightDeg)) > MAX_TILT_DEG)
      stopMotors("tilt limit");
    if (armed) updateBalanceControl(dt);
  }
  if (millis() - lastMagnetCheckMs >= 100) {
    lastMagnetCheckMs = millis();
    if (!leftSensor.magnetHealthy() || !rightSensor.magnetHealthy()) {
      fault("AS5600 magnet status / I2C"); return;
    }
  }
  processSerial();
  printTelemetry();
}
