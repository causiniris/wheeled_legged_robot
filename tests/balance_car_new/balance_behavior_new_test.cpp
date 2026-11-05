// Host checks of the controller; they do not simulate physical balance.
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

void command(char c) { Serial.input.push_back(c); processSerial(); }
void setAccelZ(int16_t raw) {
  Wire.registers[0x68][0x3F] = static_cast<uint16_t>(raw) >> 8;
  Wire.registers[0x68][0x40] = static_cast<uint8_t>(raw);
}
void failLeftSensorDuringAlignment() { Wire.shortReadAddress = 0x06; }
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
  Wire.registers[0x68][0x3F] = 0x40;
  Wire.registers[0x68][0x3D] = 0;
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
  setAccelZ(11796); // 0.72 g: accepted in the expanded range
  assert(calibrateImu());
  setAccelZ(22610); // 1.38 g: accepted in the expanded range
  assert(calibrateImu());
  setAccelZ(11305); // 0.69 g: still rejected
  assert(!calibrateImu());
  setAccelZ(23101); // 1.41 g: still rejected
  assert(!calibrateImu());
  setAccelZ(16384);
  Wire.registers[0x68][0x43] = 0x05; // 1441 / 131 = 11 deg/s: bias still rejected
  Wire.registers[0x68][0x44] = 0xA1;
  assert(!calibrateImu());
  Wire.registers[0x68][0x43] = Wire.registers[0x68][0x44] = 0;

  prepareHealthyDevices();
  setAccelZ(24576); // 1.5 g: startup calibration fails but remains disarmed
  setup();
  assert(motorsReady && !imuReady && !faultLatched && !driving());
  assert(Serial.output.find("CALIBRATION FAILED sample=") != std::string::npos);
  setAccelZ(16384);
  command('A'); // retries calibration and arms only after success
  assert(armed && imuReady && !faultLatched);
  command('X');
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  prepareHealthyDevices();
  setAccelZ(24576);
  setup();
  assert(motorsReady && !imuReady && !faultLatched && !driving());
  setAccelZ(16384);
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
  assert(Serial.output.find("MPU6050 WHO_AM_I read failed") != std::string::npos);

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
  assert(Serial.output.find("MT6701 RIGHT SDA19/SCL18 addr=0x06: I2C_SHORT_READ") != std::string::npos);
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
  assert(Serial.output.find("MT6701 RIGHT SDA19/SCL18 addr=0x06: I2C_WRITE_FAILED") != std::string::npos);

  healthySetup(); command('A'); Wire.nackAddress = 0x68; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); hostTimeUs += MAX_CONTROL_GAP_US + 1; loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  healthySetup(); command('A'); leftMotor.shaft_velocity = MAX_WHEEL_SPEED + 1; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  healthySetup(); command('A'); pitchAngleDeg = 45;
  Wire.registers[0x68][0x3D] = 0x2D; Wire.registers[0x68][0x3E] = 0x41;
  Wire.registers[0x68][0x3F] = 0x2D; Wire.registers[0x68][0x40] = 0x41;
  hostTimeUs += 5000; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

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
