// Host checks of the controller; they do not simulate physical balance.
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

void command(char c) { Serial.input.push_back(c); processSerial(); }
void setImuWord(uint8_t reg, int16_t raw) {
  Wire.registers[0x68][reg] = static_cast<uint16_t>(raw) >> 8;
  Wire.registers[0x68][reg + 1] = static_cast<uint8_t>(raw);
}
void setAccelY(int16_t raw) { setImuWord(0x3D, raw); }
void setAccelPitchDeg(float deg) {
  const float radians = deg * BalanceMath::RAD_PER_DEG;
  setImuWord(0x3B, static_cast<int16_t>(std::lround(8192 * std::sin(radians))));
  setAccelY(static_cast<int16_t>(std::lround(8192 * std::cos(radians))));
  setImuWord(0x3F, 0);
}
void failLeftSensorDuringAlignment() { Wire.shortReadAddress = 0x06; }
void failRightSensorDuringAlignment() {
  if (hostInitFocCalls == 2) Wire1.shortReadAddress = 0x06;
}
void prepareHealthyDevices() {
  Serial.output.clear(); Serial.input.clear();
  faultLatched = false; imuReady = motorsReady = armed = false; testWheel = 0;
  leftSensor.failed = rightSensor.failed = false;
  leftSensor.failure = rightSensor.failure = CheckedMT6701::Failure::none;
  leftDriver.failInit = rightDriver.failInit = false;
  leftMotor.failInitFoc = rightMotor.failInitFoc = false;
  hostInitFocCalls = 0; hostInitFocHook = nullptr;
  Wire.nackAddress = Wire.shortReadAddress = Wire1.nackAddress = Wire1.shortReadAddress = -1;
  Wire.nackCode = Wire1.nackCode = 2;
  Wire.registers[0x68][0x75] = 0x68;
  setAccelPitchDeg(0);
  setImuWord(0x43, 0);
  setImuWord(0x45, 0);
  setImuWord(0x47, 0);
  leftMotor.shaft_velocity = rightMotor.shaft_velocity = 0;
}
void healthySetup() {
  prepareHealthyDevices();
  setup();
  assert(motorsReady && imuReady && !faultLatched && !driving());
  assert(!hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
}
int main() {
  prepareHealthyDevices();
  setAccelPitchDeg(25); // Board mounted at 25 degrees when the body is upright.
  setup();
  assert(imuReady && !driving() && pitchAngleDeg == 0 && balanceSetpointDeg == 0);
  hostTimeUs += 50000; loop(); // An estimator restart also uses the saved zero.
  assert(std::abs(pitchAngleDeg) < 0.01f);
  assert(Serial.output.find("pitch=0.00 set=0.00") != std::string::npos);
  setAccelPitchDeg(35);
  hostTimeUs += 50000; loop();
  assert(std::abs(pitchAngleDeg - 10) < 0.01f);
  command('A'); assert(!armed); // 10 degrees from zero exceeds the arming limit.
  assert(Serial.output.find("axis=Z") != std::string::npos);

  setImuWord(0x43, 2620);
  setImuWord(0x45, -3930);
  setImuWord(0x47, 1310);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(std::abs(pitchRateDegPerSec - 10) < 0.01f);
  setImuWord(0x47, -1310);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(std::abs(pitchRateDegPerSec + 10) < 0.01f);
  setImuWord(0x43, 0); setImuWord(0x45, 0); setImuWord(0x47, 0);

  command('+'); assert(std::abs(balanceSetpointDeg - 0.1f) < 0.001f);
  setAccelPitchDeg(175);
  command('C');
  assert(imuReady && !driving() && pitchAngleDeg == 0 && balanceSetpointDeg == 0);
  setAccelPitchDeg(-175);
  hostTimeUs += 5000; loop();
  assert(pitchAngleDeg > 0 && pitchAngleDeg < 0.2f); // +10 degrees through the wrap.
  hostTimeUs += 50000; loop();
  assert(std::abs(pitchAngleDeg - 10) < 0.01f);

  prepareHealthyDevices();
  setAccelY(5898); // 0.72 g: accepted in the expanded range
  assert(calibrateImu());
  setAccelY(11305); // 1.38 g: accepted in the expanded range
  assert(calibrateImu());
  setAccelY(5652); // 0.69 g: still rejected
  assert(!calibrateImu());
  setAccelY(11551); // 1.41 g: still rejected
  assert(!calibrateImu());
  setAccelY(8192);
  setImuWord(0x47, 1441); // 1441 / 131 = 11 deg/s: bias still rejected
  assert(!calibrateImu());
  setImuWord(0x47, 0);

  prepareHealthyDevices();
  setAccelY(12288); // 1.5 g: startup calibration fails but remains disarmed
  setup();
  assert(motorsReady && !imuReady && !faultLatched && !driving());
  assert(Serial.output.find("CALIBRATION FAILED sample=") != std::string::npos);
  assert(Serial.output.find("MPU6050 DATA_RANGE: acceleration=1.500 g") != std::string::npos);
  setAccelY(8192);
  command('A'); // retries calibration and arms only after success
  assert(armed && imuReady && !faultLatched);
  command('X');
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  setAccelY(12288);
  setup();
  assert(motorsReady && !imuReady && !faultLatched && !driving());
  setAccelY(8192);
  command('C'); // permit checking telemetry and direction before arming
  assert(imuReady && !faultLatched && !driving());
  assert(Serial.output.find("IMU calibrated. Ready but disarmed.") != std::string::npos);
  leftMotor.shaft_velocity = std::numeric_limits<float>::quiet_NaN();
  command('A');
  assert(!armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  leftMotor.shaft_velocity = 0;

  prepareHealthyDevices();
  leftMotor.failInitFoc = true;
  setup();
  assert(hostInitFocCalls == 2);
  assert(Serial.output.find("LEFT FOC result: FAILED") != std::string::npos);
  assert(Serial.output.find("RIGHT FOC result: OK") != std::string::npos);
  assert(faultLatched && !motorsReady && !imuReady && !driving());
  assert(!hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  command('A'); assert(!armed);

  prepareHealthyDevices();
  leftDriver.failInit = true;
  setup();
  assert(hostInitFocCalls == 1);
  assert(Serial.output.find("LEFT driver initialization FAILED") != std::string::npos);
  assert(Serial.output.find("RIGHT FOC result: OK") != std::string::npos);
  assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  leftMotor.failInitFoc = rightMotor.failInitFoc = true;
  setup();
  assert(hostInitFocCalls == 2);
  assert(Serial.output.find("Alignment summary: LEFT=FAILED RIGHT=FAILED") != std::string::npos);
  assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  rightMotor.failInitFoc = true;
  setup();
  assert(hostInitFocCalls == 2);
  assert(Serial.output.find("LEFT FOC result: OK") != std::string::npos);
  assert(Serial.output.find("RIGHT FOC result: FAILED") != std::string::npos);
  assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  hostInitFocHook = failRightSensorDuringAlignment;
  setup();
  assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  Serial.output.clear();
  command('A');
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("Cannot start: RIGHT MT6701 I2C during FOC alignment")
         != std::string::npos);
  assert(Serial.output.find("I2C_SHORT_READ reg=0x03") != std::string::npos);

  prepareHealthyDevices();
  hostInitFocHook = failLeftSensorDuringAlignment;
  setup();
  assert(hostInitFocCalls == 1); // do not run the other motor after an I2C fault
  assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  Wire.registers[0x68][0x75] = 0x70;
  assert(initializeImu());
  assert(Serial.output.find("WHO_AM_I=0x70") != std::string::npos);

  prepareHealthyDevices();
  Wire.nackAddress = 0x68;
  assert(!initializeImu());
  assert(Serial.output.find("MPU6050 I2C_ERROR: WHO_AM_I read failed") != std::string::npos);

  prepareHealthyDevices();
  Wire.registers[0x68][0x75] = 0xFF;
  assert(!initializeImu());
  assert(Serial.output.find("WHO_AM_I=0xFF") != std::string::npos);

  prepareHealthyDevices();
  leftLinked = rightLinked = false;
  leftDriver.failInit = true;
  assert(!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT"));
  fault("test failed initialization");
  assert(!leftLinked && !rightLinked && faultLatched);

  prepareHealthyDevices();
  Wire.nackAddress = 0x06; Wire1.shortReadAddress = 0x06;
  setup();
  assert(faultLatched && !motorsReady && !driving());
  assert(!hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("MT6701 LEFT SDA21/SCL22 addr=0x06: I2C_WRITE_FAILED") != std::string::npos);
  assert(Serial.output.find("MT6701 RIGHT SDA17/SCL16 addr=0x06: I2C_SHORT_READ") != std::string::npos);
  command('A'); assert(!armed);

  healthySetup();
  command('5');
  assert(fabsf(angleKi - 0.1f) < 0.001f);
  command('4');
  assert(fabsf(angleKi) < 0.001f);
  hostTimeUs += 50000;
  printTelemetry();
  assert(Serial.output.find("ms=") != std::string::npos);
  assert(Serial.output.find("kp=") != std::string::npos);
  assert(Serial.output.find("ki=") != std::string::npos);
  assert(Serial.output.find("kd=") != std::string::npos);
  assert(updatePitch(0.005f)); lastControlUs = micros();
  hostTimeUs += 20000;
  command('A'); assert(!armed);
  assert(updatePitch(0.005f)); lastControlUs = micros();
  command('A'); assert(armed && hostPins[LEFT_EN] && hostPins[RIGHT_EN]);
  const float armedKp = angleKp;
  command('1'); assert(angleKp == armedKp); // gains only change while disarmed
  command('C'); assert(armed);
  angleIntegral = 0.5f;
  command('X'); assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(angleIntegral == 0);
  assert(leftMotor.voltage.q == 0 && rightMotor.voltage.q == 0);

  healthySetup(); command('L'); assert(testWheel == 1 && hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  for (int i = 0; i < 41; ++i) { hostTimeUs += 5000; loop(); }
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN] && !faultLatched);

  healthySetup(); command('A'); Wire.shortReadAddress = 0x06; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("MT6701 LEFT SDA21/SCL22 addr=0x06: I2C_SHORT_READ") != std::string::npos);
  Wire.shortReadAddress = -1; command('A'); assert(!armed);

  healthySetup(); command('A'); Wire1.nackAddress = 0x06; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("MT6701 RIGHT SDA17/SCL16 addr=0x06: I2C_WRITE_FAILED") != std::string::npos);

  healthySetup(); command('A'); Wire.nackAddress = 0x68; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: MPU6050 I2C_ERROR: register write failed") != std::string::npos);
  assert(Serial.output.find("reg=0x3B wire_code=2 received=0 expected=14") != std::string::npos);

  healthySetup(); command('A'); Wire.shortReadAddress = 0x68; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: MPU6050 I2C_ERROR: short read") != std::string::npos);
  assert(Serial.output.find("reg=0x3B wire_code=0 received=13 expected=14") != std::string::npos);

  healthySetup(); command('A'); setAccelY(20480); // 2.5 g is within the new full scale.
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(armed && !faultLatched);

  setImuWord(0x3F, 8192);
  setAccelY(900); // 0.110 g in the pitch plane is usable.
  ImuSample sample{};
  assert(readImu(sample) == nullptr);
  setAccelY(800); // 0.098 g in the pitch plane is too low.
  assert(readImu(sample) == IMU_PITCH_PLANE_LOW);

  healthySetup(); command('A'); setAccelPitchDeg(0); setAccelY(0); setImuWord(0x3F, 32767);
  hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: MPU6050 DATA_RANGE: accelerometer saturation") != std::string::npos);
  assert(Serial.output.find("accel_raw=0/0/32767") != std::string::npos);

  healthySetup();
  setAccelY(32767);
  setImuWord(0x3F, -26328);
  Serial.input.push_back('A');
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(!faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.input.empty());
  assert(Serial.output.find("IMU WAIT: accelerometer saturated") != std::string::npos);
  assert(Serial.output.find("Cannot start: IMU is settling") != std::string::npos);

  setAccelPitchDeg(0);
  for (int i = 0; i < 19; ++i) { hostTimeUs += BALANCE_PERIOD_US; loop(); }
  command('L'); assert(!driving());
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(!faultLatched && !driving());
  assert(Serial.output.find("IMU RECOVERED: sample ready") != std::string::npos);
  command('A'); assert(armed && hostPins[LEFT_EN] && hostPins[RIGHT_EN]);
  command('X'); assert(!driving());

  healthySetup(); command('A');
  const auto setLowPitchPlane = [] {
    setImuWord(0x3B, -424);
    setAccelY(214);
    setImuWord(0x3F, 11710);
  };
  setLowPitchPlane();
  setImuWord(0x47, 979);
  const float predictedPitch = IMU_PITCH_SIGN * (979 - gyroBias) / GYRO_SCALE * 0.005f;
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(armed && !faultLatched);
  assert(std::abs(pitchAngleDeg - predictedPitch) < 0.001f);

  setAccelPitchDeg(0); setImuWord(0x47, 0);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(armed && !faultLatched);
  setLowPitchPlane();
  for (int i = 0; i < 6; ++i) { hostTimeUs += BALANCE_PERIOD_US; loop(); }
  assert(armed && !faultLatched);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: MPU6050 DATA_RANGE: pitch-plane acceleration too low") != std::string::npos);

  healthySetup(); command('A'); hostTimeUs += MAX_CONTROL_GAP_US + 1; loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  healthySetup(); command('A');
  hostTimeUs += BALANCE_PERIOD_US;
  leftMotor.shaft_velocity = 31.0f;
  rightMotor.shaft_velocity = -32.0f;
  for (int i = 0; i < 40; ++i) { loop(); hostTimeUs += BALANCE_PERIOD_US; }
  assert(armed && !faultLatched && hostPins[LEFT_EN] && hostPins[RIGHT_EN]);
  assert(std::abs(speedLean + 1.0f) < 0.001f);
  assert(balanceVoltage > 0 && balanceVoltage < 1.0f);
  rightMotor.shaft_velocity = std::numeric_limits<float>::quiet_NaN();
  loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: invalid wheel speed") != std::string::npos);
  healthySetup(); command('A'); pitchAngleDeg = 45;
  setAccelPitchDeg(45);
  hostTimeUs += 5000; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("STOP: tilt limit") != std::string::npos);
  setImuWord(0x47, 32767);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(!faultLatched && !driving());
  assert(Serial.output.find("IMU WAIT: gyroscope saturated while disarmed") != std::string::npos);
  command('A'); assert(!driving());
  setImuWord(0x47, 0);
  setAccelPitchDeg(0);
  for (int i = 0; i < 20; ++i) { hostTimeUs += BALANCE_PERIOD_US; loop(); }
  assert(!faultLatched && !driving());
  command('A'); assert(armed);
  setImuWord(0x47, 32767);
  hostTimeUs += BALANCE_PERIOD_US; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("FAULT: MPU6050 DATA_RANGE: gyroscope saturation") != std::string::npos);

  using namespace BalanceMath;
  assert(fabsf(wrapDeg(181) + 179) < 0.001f);
  assert(fabsf(wrapDeg(-181) - 179) < 0.001f);
  assert(fabsf(fuseDeg(179, 0, -179, 0.005f, 0.245f) - 179.04f) < 0.01f);
  assert(voltage(5, 0, 0, 8.8f, 0.29f, 3) > 0);
  assert(voltage(-5, 0, 0, 8.8f, 0.29f, 3) < 0);
  assert(fabsf(voltage(0, 0, 10, 8.8f, 0.29f, 3) - 0.0506145f) < 0.0001f);
  assert(voltage(45, 0, 100, 8.8f, 0.29f, 3) == 3);
  float angleIntegral = 0;
  assert(voltagePid(5, 0, 0, 0, 1, 0, 0.1f, 3, angleIntegral) > 0);
  assert(angleIntegral > 0);
  angleIntegral = 0;
  assert(voltagePid(45, 0, 0, 60, 1, 0, 0.1f, 3, angleIntegral) == 3);
  assert(angleIntegral == 0); // No integral windup into saturation.
  float integral = 0;
  assert(speedLeanDeg(5, 0.005f, 0.2f, 0.05f, 3, integral) < 0);
  integral = 0;
  for (int i = 0; i < 10000; ++i) speedLeanDeg(100, 0.005f, 0.2f, 0.05f, 3, integral);
  assert(integral == 0);
  std::cout << "PASS: balance startup, arming, stop, encoder and IMU faults, timing and control math\n";
}
