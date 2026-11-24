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
#include "MPU6050_archive_adapter.h"

namespace {
constexpr int LEFT_IN1 = 32, LEFT_IN2 = 33, LEFT_IN3 = 25, LEFT_EN = 12;
constexpr int RIGHT_IN1 = 14, RIGHT_IN2 = 27, RIGHT_IN3 = 26, RIGHT_EN = 13;
constexpr int LEFT_I2C_SDA = 21, LEFT_I2C_SCL = 22;
constexpr int RIGHT_I2C_SDA = 17, RIGHT_I2C_SCL = 16;
constexpr uint32_t LEFT_I2C_HZ = 100000, RIGHT_I2C_HZ = 100000;
constexpr uint8_t MPU6050_ADDRESS = 0x68;  // AD0=GND; use 0x69 if AD0=3.3V.
constexpr int POLE_PAIRS = 7;              // Inherited homework value; verify both motors.
constexpr float SUPPLY_VOLTAGE = 12.0f;
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f;
constexpr float MOTOR_VOLTAGE_LIMIT = 3.0f; // q-axis voltage, NOT a current limit.
constexpr long PWM_FREQUENCY_HZ = 10000;

// Select the IMU axis parallel to the wheel axle. Startup calibration accounts
// for a fixed mounting angle around that axis.
// The sign must make a physical forward lean read positive pitch AND rate.
constexpr char IMU_PITCH_AXIS = 'Z';
constexpr float IMU_PITCH_SIGN = 1.0f;
// Each positive signed wheel command must propel the chassis forward.
// These are installation assumptions, verified separately by L/R commands.
constexpr float LEFT_MOTOR_SIGN = 1.0f;
constexpr float RIGHT_MOTOR_SIGN = -1.0f;
static_assert(IMU_PITCH_AXIS == 'X' || IMU_PITCH_AXIS == 'Y' || IMU_PITCH_AXIS == 'Z',
              "Use X, Y or Z IMU axis");
static_assert(IMU_PITCH_SIGN == 1.0f || IMU_PITCH_SIGN == -1.0f, "IMU sign must be +/-1");
static_assert(LEFT_MOTOR_SIGN * LEFT_MOTOR_SIGN == 1.0f, "Left sign must be +/-1");
static_assert(RIGHT_MOTOR_SIGN * RIGHT_MOTOR_SIGN == 1.0f, "Right sign must be +/-1");

// Fixed-leg model starting gains. See analysis/revised_motor_pid/check_balance_defaults.py.
// Angle gains use radians. The wheel-speed PI gives a lean correction in degrees.
constexpr float INITIAL_ANGLE_KP = 56.0f, INITIAL_ANGLE_KI = 0.0f, INITIAL_ANGLE_KD = 1.90f;
constexpr float SPEED_KP = 0.31f; // deg / (wheel rad/s)
constexpr float SPEED_KI = 0.20f; // deg / wheel rad
constexpr float MAX_SPEED_LEAN_DEG = 1.0f;
constexpr float MAX_TILT_DEG = 35.0f, ARM_TILT_DEG = 8.0f;
constexpr float TEST_VOLTAGE = 0.5f;
constexpr uint32_t TEST_DURATION_MS = 200;
constexpr uint32_t BALANCE_PERIOD_US = 5000, MAX_CONTROL_GAP_US = 30000;
constexpr uint32_t MAX_GYRO_ONLY_US = 30000;
constexpr uint32_t IMU_SETTLE_US = 100000;
constexpr char IMU_PITCH_PLANE_LOW[] = "MPU6050 DATA_RANGE: pitch-plane acceleration too low";
constexpr char IMU_ACCEL_SATURATION[] = "MPU6050 DATA_RANGE: accelerometer saturation";
constexpr char IMU_GYRO_SATURATION[] = "MPU6050 DATA_RANGE: gyroscope saturation";
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;
constexpr uint32_t AUTOTUNE_HEARTBEAT_TIMEOUT_MS = 500;
constexpr float FILTER_TIME_CONSTANT = 1.75f; // Slow gravity correction during wheel acceleration.
constexpr float GYRO_SCALE = 131.0f; // +/-250 deg/s

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

// SimpleFOC 2.3.3's stock I2C sensor checks the register-address write but not
// a short data read. This adapter rejects both and keeps errors latched.
class CheckedMT6701 : public Sensor {
 public:
  enum class Failure { none, i2c };
  explicit CheckedMT6701(TwoWire& bus) : bus_(bus) {}
  void init() override { Sensor::init(); }
  bool failed = false;
  Failure failure = Failure::none;
  I2cReadDiagnostic diagnostic;
  float getSensorAngle() override {
    if (failed) { cutEnablePins(); return -1.0f; }
    uint8_t data[2];
    if (!readRegisters(bus_, 0x06, 0x03, &data[0], 1, diagnostic) ||
        !readRegisters(bus_, 0x06, 0x04, &data[1], 1, diagnostic)) {
      latchFailure(Failure::i2c);
      return -1.0f; // Sensor::update ignores this sample instead of inventing an angle.
    }
    return ((static_cast<uint16_t>(data[0]) << 6) | (data[1] >> 2)) * (_2PI / 16384.0f);
  }
 private:
  TwoWire& bus_;
  bool latchFailure(Failure reason) {
    failed = true;
    failure = reason;
    cutEnablePins(); // No serial work until both power stages have been disabled.
    return false;
  }
};

CheckedMT6701 leftSensor(Wire), rightSensor(Wire1);

void printEncoderDiagnostic(const char* side, int sda, int scl, const CheckedMT6701& sensor) {
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

uint32_t mpuDurationUs = 0;
MpuArchiveAdapter mpu(Wire, MPU6050_ADDRESS);
BLDCMotor leftMotor(POLE_PAIRS), rightMotor(POLE_PAIRS);
BLDCDriver3PWM leftDriver(LEFT_IN1, LEFT_IN2, LEFT_IN3, LEFT_EN);
BLDCDriver3PWM rightDriver(RIGHT_IN1, RIGHT_IN2, RIGHT_IN3, RIGHT_EN);
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; };
float gyroBias = 0, pitchAngleDeg = 0, pitchRateDegPerSec = 0;
float balanceSetpointDeg = 0, imuPitchZeroDeg = 0, balanceVoltage = 0;
float angleKp = INITIAL_ANGLE_KP, angleKi = INITIAL_ANGLE_KI, angleKd = INITIAL_ANGLE_KD;
float angleIntegral = 0;
float speedFiltered = 0, speedIntegral = 0, speedLean = 0;
bool leftLinked = false, rightLinked = false, motorsReady = false;
bool imuReady = false, faultLatched = false, armed = false, speedAssist = true;
bool imuSettling = false;
uint32_t lastImuSaturationUs = 0;
const char* faultReason = nullptr;
bool autotuneGuard = false;
uint32_t lastAutotuneHeartbeatMs = 0;
uint8_t testWheel = 0;
uint32_t testStartMs = 0, lastControlUs = 0, lastValidPitchAccelUs = 0;
uint32_t lastTelemetryMs = 0;
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
  balanceVoltage = speedIntegral = speedLean = angleIntegral = 0;
  if (wasDriving) Serial.printf("STOP: %s\n", reason);
}
void fault(const char* reason) {
  stopMotors(reason);
  if (!faultLatched) {
    faultLatched = true;
    faultReason = reason;
    Serial.printf("FAULT: %s. Check wiring/sensors, then reset.\n", reason);
  }
}
float rawGyro(const ImuSample& s) {
  if (IMU_PITCH_AXIS == 'X') return s.gx;
  if (IMU_PITCH_AXIS == 'Y') return s.gy;
  return s.gz;
}
struct PitchAccel {
  float numerator, denominator;
};
PitchAccel projectPitchAccel(const ImuSample& s) {
  // Order the components so atan2 follows the positive gyro rotation.
  if (IMU_PITCH_AXIS == 'X') return {float(s.ay), float(s.az)};
  if (IMU_PITCH_AXIS == 'Y') return {-float(s.ax), float(s.az)};
  return {float(s.ax), float(s.ay)};
}
float accelPitch(const ImuSample& s) {
  const PitchAccel pitch = projectPitchAccel(s);
  return BalanceMath::wrapDeg(IMU_PITCH_SIGN * atan2f(pitch.numerator, pitch.denominator) /
                              BalanceMath::RAD_PER_DEG);
}
const char* readImu(ImuSample& s) {
  const uint32_t start = micros();
  const bool readOk = mpu.getMotion6(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
  mpuDurationUs = micros() - start;
  if (!readOk) {
    return mpu.diagnostic.writeCode != 0 ? "MPU6050 I2C_ERROR: register write failed"
                                         : "MPU6050 I2C_ERROR: short read";
  }
  const float norm2 = float(s.ax) * s.ax + float(s.ay) * s.ay + float(s.az) * s.az;
  const PitchAccel pitch = projectPitchAccel(s);
  const float plane2 = pitch.numerator * pitch.numerator + pitch.denominator * pitch.denominator;
  constexpr float minAccelCounts = 0.1f * MpuArchiveAdapter::ACCEL_COUNTS_PER_G;
  constexpr float minAccelSquared = minAccelCounts * minAccelCounts;
  if (norm2 <= minAccelSquared) return "MPU6050 DATA_RANGE: acceleration near zero";
  if (abs(int(s.ax)) >= 32700 || abs(int(s.ay)) >= 32700 || abs(int(s.az)) >= 32700)
    return IMU_ACCEL_SATURATION;
  if (abs(int(s.gx)) >= 32700 || abs(int(s.gy)) >= 32700 || abs(int(s.gz)) >= 32700)
    return IMU_GYRO_SATURATION;
  if (plane2 <= minAccelSquared) return IMU_PITCH_PLANE_LOW;
  return nullptr;
}
void printImuReadDiagnostic(const ImuSample& s) {
  const auto& d = mpu.diagnostic;
  Serial.printf("IMU DETAIL: addr=0x%02X reg=0x%02X wire_code=%u received=%u expected=%u available=%d read_us=%lu\n",
                unsigned(MPU6050_ADDRESS), unsigned(d.reg), unsigned(d.writeCode), unsigned(d.received),
                unsigned(d.expected), d.available, static_cast<unsigned long>(mpuDurationUs));
  if (d.writeCode == 0 && d.received == d.expected && d.available >= d.expected) {
    Serial.printf("IMU SAMPLE: accel_raw=%d/%d/%d gyro_raw=%d/%d/%d pitch_axis=%c\n",
                  s.ax, s.ay, s.az, s.gx, s.gy, s.gz, IMU_PITCH_AXIS);
  }
}
bool initializeImu() {
  uint8_t identity = 0;
  if (!mpu.readBytes(0x75, &identity, 1)) {
    Serial.printf("MPU6050 I2C_ERROR: WHO_AM_I read failed addr=0x%02X reg=0x%02X wire_code=%u received=%u expected=%u\n",
                  unsigned(MPU6050_ADDRESS), unsigned(mpu.diagnostic.reg),
                  unsigned(mpu.diagnostic.writeCode), unsigned(mpu.diagnostic.received),
                  unsigned(mpu.diagnostic.expected));
    return false;
  }
  Serial.printf("MPU6050 WHO_AM_I=0x%02X\n", unsigned(identity));
  if (identity == 0x00 || identity == 0xFF) {
    Serial.println("MPU6050 invalid identity; check wiring and sensor");
    return false;
  }
  if (identity != 0x68)
    Serial.println("WARNING: unverified MPU6050-compatible identity; verify sensor output before use");
  if (!mpu.initialize()) {
    Serial.printf("MPU6050 I2C_ERROR: config failed reg=0x%02X wire_code=%u received=%u expected=%u\n",
                  unsigned(mpu.diagnostic.reg), unsigned(mpu.diagnostic.writeCode),
                  unsigned(mpu.diagnostic.received), unsigned(mpu.diagnostic.expected));
    return false;
  }
  Serial.printf("IMU accel range configured: +/-%.0f g\n", 32768.0f / MpuArchiveAdapter::ACCEL_COUNTS_PER_G);
  delay(100);
  ImuSample s{};
  if (const char* error = readImu(s)) {
    Serial.printf("Initial IMU sample failed: %s\n", error);
    printImuReadDiagnostic(s);
    return false;
  }
  return true;
}
bool calibrateImu() {
  imuReady = false;
  constexpr int SAMPLES = 300;
  double gyroSum = 0, gyroSquares = 0, sinSum = 0, cosSum = 0;
  float minAngle = 0, maxAngle = 0, firstAngle = 0;
  Serial.println("CALIBRATE: hold the chassis upright with minimal motion for about 2 seconds to set pitch=0.");
  for (int i = 0; i < SAMPLES; ++i) {
    ImuSample s{};
    if (const char* error = readImu(s)) {
      Serial.printf("CALIBRATION FAILED sample=%d: %s\n", i, error);
      printImuReadDiagnostic(s);
      return false;
    }
    const float g = rawGyro(s), angle = accelPitch(s);
    const float norm = sqrtf(float(s.ax)*s.ax + float(s.ay)*s.ay + float(s.az)*s.az) /
                       MpuArchiveAdapter::ACCEL_COUNTS_PER_G;
    if (norm < 0.70f || norm > 1.40f) {
      Serial.printf("CALIBRATION FAILED sample=%d: MPU6050 DATA_RANGE: acceleration=%.3f g (allowed 0.70-1.40)\n",
                    i, norm);
      return false;
    }
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
  const float gyroStd = sqrt(variance) / GYRO_SCALE;
  const float gyroMean = mean / GYRO_SCALE;
  const float angleSpan = maxAngle - minAngle;
  Serial.printf("CALIBRATION stats: gyro_mean=%.3f dps std=%.3f dps angle_span=%.3f deg\n",
                gyroMean, gyroStd, angleSpan);
  if (gyroStd > 5.0f || fabsf(gyroMean) > 10.0f || angleSpan > 30.0f) {
    Serial.println("CALIBRATION FAILED: limits std<=5.0 dps |mean|<=10 dps angle_span<=30 deg");
    return false;
  }
  if (gyroStd > 1.5f) {
    Serial.printf("CALIBRATION WARNING: gyro std=%.3f dps. Handheld calibration accepted.\n", gyroStd);
  }
  gyroBias = mean;
  imuPitchZeroDeg = atan2f(sinSum, cosSum) / BalanceMath::RAD_PER_DEG;
  pitchAngleDeg = balanceSetpointDeg = 0;
  pitchRateDegPerSec = 0;
  lastValidPitchAccelUs = lastControlUs = micros();
  imuSettling = false;
  imuReady = true;
  Serial.printf("IMU ready: pitch=0.00 deg, raw_zero=%.2f deg, bias=%.2f deg/s\n",
                imuPitchZeroDeg, gyroBias / GYRO_SCALE);
  return true;
}
bool updatePitch(float dt) {
  ImuSample s{};
  const char* error = readImu(s);
  const uint32_t sampleUs = micros();
  const bool accelSaturated = error == IMU_ACCEL_SATURATION;
  if (!driving() && (accelSaturated || error == IMU_GYRO_SATURATION)) {
    lastImuSaturationUs = sampleUs;
    if (!imuSettling) {
      imuSettling = true;
      Serial.printf("IMU WAIT: %s saturated while disarmed. Hold upright and still.\n",
                    accelSaturated ? "accelerometer" : "gyroscope");
      printImuReadDiagnostic(s);
    }
    return false;
  }
  const bool gyroOnly = error == IMU_PITCH_PLANE_LOW && armed &&
                        sampleUs - lastValidPitchAccelUs <= MAX_GYRO_ONLY_US;
  if (error && !gyroOnly) {
    fault(error);
    printImuReadDiagnostic(s);
    return false;
  }
  pitchRateDegPerSec = IMU_PITCH_SIGN * (rawGyro(s) - gyroBias) / GYRO_SCALE;
  if (gyroOnly) {
    pitchAngleDeg = BalanceMath::wrapDeg(pitchAngleDeg + pitchRateDegPerSec * dt);
  } else {
    const float accelRelativeDeg = BalanceMath::wrapDeg(accelPitch(s) - imuPitchZeroDeg);
    if (imuSettling || dt > MAX_CONTROL_GAP_US * 1.0e-6f) pitchAngleDeg = accelRelativeDeg;
    else pitchAngleDeg = BalanceMath::fuseDeg(pitchAngleDeg, pitchRateDegPerSec,
                                             accelRelativeDeg, dt, FILTER_TIME_CONSTANT);
    lastValidPitchAccelUs = sampleUs;
  }
  if (!isfinite(pitchAngleDeg) || !isfinite(pitchRateDegPerSec)) {
    fault("MPU6050 invalid pitch estimate");
    printImuReadDiagnostic(s);
    return false;
  }
  if (imuSettling && sampleUs - lastImuSaturationUs >= IMU_SETTLE_US) {
    imuSettling = false;
    Serial.println("IMU RECOVERED: sample ready. Motors disarmed.");
  }
  return true;
}

bool initializeMotor(BLDCMotor& motor, BLDCDriver3PWM& driver,
                     CheckedMT6701& sensor, bool& linked, const char* label) {
  motor.linkSensor(&sensor);
  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) {
    cutEnablePins();
    Serial.printf("%s driver initialization FAILED\n", label);
    return false;
  }
  motor.linkDriver(&driver);
  linked = true; // Only now is motor.disable() safe to call.
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.voltage_sensor_align = 2.0f;
  motor.LPF_velocity.Tf = 0.02f;
  motor.init();
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
    motor.disable();
    cutEnablePins();
    Serial.printf("%s motor initialization FAILED\n", label);
    return false;
  }
  Serial.printf("%s FOC alignment: keep wheel free; it WILL move.\n", label);
  Serial.printf("%s configuration: supply=%.2f V (configured, not measured), alignment=%.2f V, pole_pairs=%d\n",
                label, SUPPLY_VOLTAGE, motor.voltage_sensor_align, POLE_PAIRS);
  const float startAngle = sensor.getSensorAngle();
  if (sensor.failed) {
    motor.disable();
    Serial.printf("%s FOC result: FAILED (encoder read before alignment)\n", label);
    return false;
  }
  const bool aligned = motor.initFOC();
  motor.disable();
  motor.target = 0;
  motor.voltage.q = motor.voltage.d = 0;
  const float endAngle = sensor.failed ? -1.0f : sensor.getSensorAngle();
  if (startAngle >= 0 && endAngle >= 0)
    Serial.printf("%s encoder start=%.3f rad end=%.3f rad (endpoint readings only)\n",
                  label, startAngle, endAngle);
  if (!aligned || sensor.failed) {
    Serial.printf("%s FOC result: FAILED (initFOC=%d encoder_fault=%d)\n",
                  label, int(aligned), int(sensor.failed));
    return false;
  }
  Serial.printf("%s FOC result: OK\n", label);
  Serial.printf("%s ready: sensor_direction=%d, electrical_zero=%.4f\n",
                label, static_cast<int>(motor.sensor_direction), motor.zero_electric_angle);
  return true;
}

bool canStart() {
  if (driving()) return false;
  if (autotuneGuard && millis() - lastAutotuneHeartbeatMs > AUTOTUNE_HEARTBEAT_TIMEOUT_MS) {
    Serial.println("Cannot start: autotune heartbeat expired.");
    return false;
  }
  if (faultLatched) {
    Serial.printf("Cannot start: %s. Correct the fault and reset.\n", faultReason);
    printEncoderDiagnostics();
    return false;
  }
  if (!motorsReady || !imuReady || leftSensor.failed || rightSensor.failed) {
    Serial.println("Cannot start: initialization/sensor fault; check and reset.");
    return false;
  }
  if (imuSettling) {
    Serial.println("Cannot start: IMU is settling. Hold upright and still.");
    return false;
  }
  // Calibration can block for seconds. Refresh wheel feedback before arming.
  leftMotor.loopFOC(); rightMotor.loopFOC();
  if (leftSensor.failed || rightSensor.failed) {
    fault("MT6701 angle read"); printEncoderDiagnostics(); return false;
  }
  leftMotor.move(0); rightMotor.move(0);
  if (micros() - lastValidPitchAccelUs > 15000 || !isfinite(pitchAngleDeg) ||
      fabsf(BalanceMath::wrapDeg(pitchAngleDeg - balanceSetpointDeg)) > ARM_TILT_DEG ||
      !isfinite(pitchRateDegPerSec) || !isfinite(leftMotor.shaft_velocity) ||
      !isfinite(rightMotor.shaft_velocity) || fabsf(pitchRateDegPerSec) > 20.0f ||
      fabsf(leftMotor.shaft_velocity) > 1.0f ||
      fabsf(rightMotor.shaft_velocity) > 1.0f) {
    Serial.println("Cannot start: hold upright, stop wheels, and wait for fresh IMU readings.");
    return false;
  }
  speedIntegral = speedFiltered = speedLean = balanceVoltage = angleIntegral = 0;
  lastFocUs = maxFocGapUs = 0;
  leftMotor.voltage.q = leftMotor.voltage.d = rightMotor.voltage.q = rightMotor.voltage.d = 0;
  leftMotor.target = rightMotor.target = 0;
  return true;
}
void showSettings() {
  Serial.printf("Kp=%.2f Ki=%.2f Kd=%.2f speedAssist=%d set=%.2f axis=%c signs=%.0f/%.0f/%.0f\n",
    angleKp, angleKi, angleKd, speedAssist, balanceSetpointDeg, IMU_PITCH_AXIS,
    IMU_PITCH_SIGN, LEFT_MOTOR_SIGN, RIGHT_MOTOR_SIGN);
}
void processSerial() {
  // Bound serial work; no readString()/parseFloat() or unbounded serial draining.
  for (int n = 0; n < 16 && Serial.available() > 0; ++n) {
    const char c = Serial.read();
    if (c == 'X' || c == 'x') { stopMotors("serial X"); return; }
    if (c == 'H') {
      if (!faultLatched && (autotuneGuard || (!driving() && motorsReady && imuReady))) {
        if (!autotuneGuard) Serial.println("AUTOTUNE GUARD ON");
        autotuneGuard = true;
        lastAutotuneHeartbeatMs = millis();
      }
      continue;
    }
    if (c == '\n' || c == '\r' || c == ' ') continue;
    if (driving()) continue; // Only X and the heartbeat are accepted while driving.
    if (c == 'Q' && autotuneGuard) {
      autotuneGuard = false;
      Serial.println("AUTOTUNE GUARD OFF");
      return;
    }
    if (c == 'A' || c == 'a') {
      if (!imuReady && motorsReady && !faultLatched) {
        Serial.println("A: retrying IMU calibration; keep chassis still.");
        if (!calibrateImu()) {
          Serial.println("Cannot start: IMU calibration failed; motors remain disabled. Send A to retry.");
          return;
        }
      }
      if (canStart()) {
        Serial.println("BALANCE ARMED; X stops.");
        leftMotor.enable(); rightMotor.enable(); armed = true;
      }
      return;
    }
    if (c == 'L' || c == 'l' || c == 'R' || c == 'r') {
      if (canStart()) {
        testWheel = (c == 'L' || c == 'l') ? 1 : 2;
        Serial.printf("Wheel test: %s %.1f V for %lu ms. Keep wheels off the floor. X stops.\n",
                      testWheel == 1 ? "LEFT" : "RIGHT", TEST_VOLTAGE,
                      static_cast<unsigned long>(TEST_DURATION_MS));
        testStartMs = millis();
        if (testWheel == 1) leftMotor.enable(); else rightMotor.enable();
      }
      return;
    }
    if (c == 'C' || c == 'c') {
      if (!faultLatched && motorsReady) {
        stopMotors("recalibration");
        if (!calibrateImu()) Serial.println("IMU calibration failed; motors remain disabled. Send C or A to retry.");
        else Serial.println("IMU calibrated. Ready but disarmed.");
      }
      return;
    }
    if (c == '+') balanceSetpointDeg = BalanceMath::clamp(balanceSetpointDeg + 0.1f, -5, 5);
    else if (c == '-') balanceSetpointDeg = BalanceMath::clamp(balanceSetpointDeg - 0.1f, -5, 5);
    else if (c == '1') angleKp = fminf(60.0f, angleKp + 0.5f);
    else if (c == '0') angleKp = fmaxf(0.0f, angleKp - 0.5f);
    else if (c == '3') angleKd = fminf(5.0f, angleKd + 0.02f);
    else if (c == '2') angleKd = fmaxf(0.0f, angleKd - 0.02f);
    else if (c == '5') angleKi = fminf(10.0f, angleKi + 0.1f);
    else if (c == '4') angleKi = fmaxf(0.0f, angleKi - 0.1f);
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
  balanceVoltage = BalanceMath::voltagePid(pitchAngleDeg, balanceSetpointDeg + speedLean,
                                           pitchRateDegPerSec, angleKp, angleKi, angleKd,
                                           dt, MOTOR_VOLTAGE_LIMIT, angleIntegral);
  if (!isfinite(balanceVoltage)) fault("non-finite controller output");
}
void printTelemetry() {
  const uint32_t now = millis();
  if (now - lastTelemetryMs < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = now;
  char line[256];
  const int length = snprintf(line, sizeof(line),
    "ms=%lu pitch=%.2f set=%.2f rate=%.2f wl=%.2f wr=%.2f u=%.2f lean=%.2f A=%d T=%u F=%d kp=%.2f ki=%.2f kd=%.2f imu_us=%lu foc_us=%lu guard=%d\n",
    static_cast<unsigned long>(now),
    pitchAngleDeg, balanceSetpointDeg, pitchRateDegPerSec,
    LEFT_MOTOR_SIGN * leftMotor.shaft_velocity, RIGHT_MOTOR_SIGN * rightMotor.shaft_velocity,
    balanceVoltage, speedLean, armed, testWheel, faultLatched, angleKp, angleKi, angleKd,
    static_cast<unsigned long>(mpuDurationUs), static_cast<unsigned long>(maxFocGapUs), autotuneGuard);
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
  Serial.println("V1_06_TwoWheel_Balance_Car_new / ESP32 / SimpleFOC 2.3.3 / encoder diagnostics v2");
  Serial.printf("Left PWM 32/33/25 EN12 I2C21/22; Right PWM14/27/26 EN13 I2C%d/%d\n",
                RIGHT_I2C_SDA, RIGHT_I2C_SCL);
  Serial.println("MPU6050 shares LEFT hardware I2C SDA21/SCL22, AD0=GND");
  Serial.printf("I2C clock: LEFT %lu kHz, RIGHT %lu kHz\n",
                static_cast<unsigned long>(LEFT_I2C_HZ / 1000),
                static_cast<unsigned long>(RIGHT_I2C_HZ / 1000));
  Serial.println("Commands: L/R=short wheel test A=balance X=stop C=calibrate +/-=trim");
  Serial.println("Disarmed tuning: 1/0=Kp +/-0.5, 5/4=Ki +/-0.1, 3/2=Kd +/-0.02, V=speed assist, ?=settings");
  if (!Wire.begin(LEFT_I2C_SDA, LEFT_I2C_SCL, LEFT_I2C_HZ) ||
      !Wire1.begin(RIGHT_I2C_SDA, RIGHT_I2C_SCL, RIGHT_I2C_HZ)) {
    fault("I2C initialization"); return;
  }
  Wire.setTimeOut(3); Wire1.setTimeOut(3);
  leftSensor.init(); rightSensor.init();
  if (leftSensor.failed || rightSensor.failed) {
    fault("MT6701 angle initialization"); printEncoderDiagnostics(true); return;
  }
  printEncoderDiagnostics(true);
  if (!initializeImu()) {
    fault("MPU6050 initialization; see details above"); return;
  }
  SimpleFOCDebug::enable(&Serial);
  const bool leftOk = initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT");
  bool rightOk = false;
  bool rightAttempted = false;
  if (!leftSensor.failed && !rightSensor.failed) {
    if (!leftOk) Serial.println("LEFT failed; continuing RIGHT alignment. Both wheels must be free.");
    rightAttempted = true;
    rightOk = initializeMotor(rightMotor, rightDriver, rightSensor, rightLinked, "RIGHT");
  } else {
    Serial.println("RIGHT alignment skipped because of encoder communication fault.");
  }
  SimpleFOCDebug::enable(nullptr);
  if (!leftOk || !rightOk) {
    Serial.printf("Alignment summary: LEFT=%s RIGHT=%s\n", leftOk ? "OK" : "FAILED",
                  rightOk ? "OK" : (rightAttempted ? "FAILED" : "SKIPPED"));
    if (leftSensor.failed) fault("LEFT MT6701 I2C during FOC alignment");
    else if (rightSensor.failed) fault("RIGHT MT6701 I2C during FOC alignment");
    else if (leftOk) fault("RIGHT motor initialization or FOC alignment failed");
    else if (rightOk) fault("LEFT motor initialization or FOC alignment failed");
    else fault("both motor initializations or FOC alignments failed");
    printEncoderDiagnostics(); return;
  }
  motorsReady = true;
  stopMotors("startup");
  if (!calibrateImu()) {
    Serial.println("IMU calibration failed; motors remain disabled. Send C or A to retry.");
    return;
  }
  lastTelemetryMs = millis();
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
  if (driving() && autotuneGuard && millis() - lastAutotuneHeartbeatMs > AUTOTUNE_HEARTBEAT_TIMEOUT_MS) {
    fault("autotune heartbeat timeout"); return;
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
  if (leftSensor.failed) { fault("LEFT MT6701 I2C read"); printEncoderDiagnostics(); return; }
  rightMotor.loopFOC();
  if (rightSensor.failed) { fault("RIGHT MT6701 I2C read"); printEncoderDiagnostics(); return; }
  // move() refreshes shaft_velocity even while disabled, using the latest sensor.
  leftMotor.move(LEFT_MOTOR_SIGN * (armed ? balanceVoltage : (testWheel == 1 ? TEST_VOLTAGE : 0)));
  rightMotor.move(RIGHT_MOTOR_SIGN * (armed ? balanceVoltage : (testWheel == 2 ? TEST_VOLTAGE : 0)));
  if (!isfinite(leftMotor.shaft_velocity) || !isfinite(rightMotor.shaft_velocity)) {
    fault("invalid wheel speed"); return;
  }

  const uint32_t nowUs = micros(), elapsed = nowUs - lastControlUs;
  if (elapsed >= BALANCE_PERIOD_US) {
    if (driving() && elapsed > MAX_CONTROL_GAP_US) { fault("control loop overdue"); return; }
    lastControlUs = nowUs;
    const float dt = elapsed * 1.0e-6f;
    if (!updatePitch(dt)) {
      if (!faultLatched) processSerial();
      return;
    }
    if (driving() && fabsf(pitchAngleDeg) > MAX_TILT_DEG)
      stopMotors("tilt limit");
    if (armed) updateBalanceControl(dt);
  }
  processSerial();
  if (!imuSettling) printTelemetry();
}
