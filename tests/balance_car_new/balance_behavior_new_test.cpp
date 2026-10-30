// Compiles the ACTUAL controller with fake buses/drivers. No physical plant is
// simulated, so these checks do not validate real-world stability or torque.
#include <cassert>
#include <cmath>
#include <iostream>
#include "fakes/HostMpuI2C_new.h"
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

void command(char c) { Serial.input.push_back(c); processSerial(); }
void prepareHealthyDevices() {
  Serial.output.clear(); Serial.input.clear();
  faultLatched = false; imuReady = motorsReady = armed = false; testWheel = 0;
  leftSensor.failed = rightSensor.failed = false;
  leftDriver.failInit = rightDriver.failInit = false;
  Wire.nackAddress = Wire.shortReadAddress = Wire1.nackAddress = Wire1.shortReadAddress = -1;
  Wire.nackCode = Wire1.nackCode = 2;
  Wire.registers[0x36][0x0B] = Wire1.registers[0x36][0x0B] = 0x20;
  // Neither hardware bus contains a working MPU6050. A controller accidentally
  // using Wire/Wire1 for its IMU must fail this setup.
  Wire.nackAddress = Wire1.nackAddress = 0x68;
  hostMpu = HostMpuI2C{}; attachHostMpu();
  hostMpu.registers[0x75] = 0x68;
  hostMpu.setWord(0x3F, 16384);
  leftMotor.shaft_velocity = rightMotor.shaft_velocity = 0;
}
void healthySetup() {
  prepareHealthyDevices();
  setup();
  assert(motorsReady && imuReady && !faultLatched && !driving());
  assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  assert(hostPinModes[16] == OUTPUT_OPEN_DRAIN && hostPinModes[17] == OUTPUT_OPEN_DRAIN);
  assert(hostMpu.registers[0x19] == 4 && hostMpu.registers[0x6B] == 1);
}
int main() {
  // Early initialization failure must not dereference a null motor driver.
  leftLinked = rightLinked = false;
  leftDriver.failInit = true;
  assert(!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT"));
  fault("test failed initialization");
  assert(!leftLinked && !rightLinked && faultLatched);

  // A failed LEFT must not hide a simultaneous RIGHT failure at startup.
  prepareHealthyDevices();
  Wire.nackAddress = 0x36; Wire1.shortReadAddress = 0x36;
  setup();
  assert(faultLatched && !motorsReady && !driving());
  assert(!hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("AS5600 LEFT SDA21/SCL22 addr=0x36: I2C_WRITE_FAILED reg=0x0B wire_code=2 (address NACK)") != std::string::npos);
  assert(Serial.output.find("AS5600 RIGHT SDA19/SCL18 addr=0x36: I2C_SHORT_READ reg=0x0B received=0 available=0 expected=1") != std::string::npos);
  assert(Serial.output.find("FOC alignment") == std::string::npos);
  assert(hostMpu.frames.empty()); // An idle software bus is not yet an MPU identity check.
  command('A'); assert(!armed); // Diagnostics must not bypass the fault latch.

  const struct { int code; const char* text; } wireCases[] = {
    {2, "wire_code=2 (address NACK)"}, {3, "wire_code=3 (data NACK)"},
    {4, "wire_code=4 (bus error)"}, {5, "wire_code=5 (timeout)"}
  };
  for (const auto& test : wireCases) {
    prepareHealthyDevices(); Wire1.nackAddress = 0x36; Wire1.nackCode = test.code;
    setup();
    assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
    assert(Serial.output.find("AS5600 LEFT SDA21/SCL22 addr=0x36: OK STATUS=0x20 MD=1 ML=0 MH=0") != std::string::npos);
    assert(Serial.output.find("AS5600 RIGHT SDA19/SCL18 addr=0x36: I2C_WRITE_FAILED") != std::string::npos);
    assert(Serial.output.find(test.text) != std::string::npos);
  }
  const struct { uint8_t status; const char* reason; const char* flags; } magnetCases[] = {
    {0x00, "MAGNET_NOT_DETECTED", "MD=0 ML=0 MH=0"},
    {0x10, "MAGNET_TOO_WEAK", "MD=0 ML=1 MH=0"},
    {0x30, "MAGNET_TOO_WEAK", "MD=1 ML=1 MH=0"},
    {0x08, "MAGNET_TOO_STRONG", "MD=0 ML=0 MH=1"},
    {0x28, "MAGNET_TOO_STRONG", "MD=1 ML=0 MH=1"},
    {0x38, "MAGNET_STATUS_INVALID", "MD=1 ML=1 MH=1"}
  };
  for (const auto& test : magnetCases) {
    prepareHealthyDevices(); Wire1.registers[0x36][0x0B] = test.status;
    setup();
    assert(faultLatched && !motorsReady && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
    assert(Serial.output.find(std::string("AS5600 RIGHT SDA19/SCL18 addr=0x36: ") + test.reason) != std::string::npos);
    assert(Serial.output.find(test.flags) != std::string::npos);
    assert(Serial.output.find("I2C_WRITE_FAILED") == std::string::npos);
    assert(Serial.output.find("FOC alignment") == std::string::npos);
  }

  healthySetup();
  hostTimeUs += 20000;
  command('A'); assert(!armed); // stale IMU may never arm
  assert(updatePitch(0.005f)); lastControlUs = micros();
  command('A'); assert(armed && hostPins[LEFT_EN] && hostPins[RIGHT_EN]);
  command('C'); assert(armed); // do not block to recalibrate while running
  command('X'); assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(leftMotor.voltage.q == 0 && rightMotor.voltage.q == 0);

  healthySetup(); command('L'); assert(testWheel == 1 && hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  // Run normal loops, not a single artificial 200 ms control stall.
  for (int i = 0; i < 41; ++i) { hostTimeUs += 5000; loop(); }
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN] && !faultLatched);

  healthySetup(); command('A'); Wire.shortReadAddress = 0x36; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("AS5600 LEFT SDA21/SCL22 addr=0x36: I2C_SHORT_READ reg=0x0C") != std::string::npos);
  Wire.shortReadAddress = -1; command('A'); assert(!armed); // errors stay latched

  healthySetup(); command('A'); Wire1.nackAddress = 0x36; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("AS5600 RIGHT SDA19/SCL18 addr=0x36: I2C_WRITE_FAILED reg=0x0C") != std::string::npos);

  healthySetup(); command('A'); hostMpu.present = false; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); hostMpu.holdClockLow = true; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(mpuBus.error() == SoftwareI2C_new::Error::timeout);

  healthySetup(); command('A'); hostTimeUs += MAX_CONTROL_GAP_US + 1; loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); leftMotor.shaft_velocity = MAX_WHEEL_SPEED + 1; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); pitchAngleDeg = 45;
  hostMpu.setWord(0x3D, 11585); hostMpu.setWord(0x3F, 11585);
  hostTimeUs += 5000; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); Wire1.registers[0x36][0x0B] = 0; command('R');
  assert(faultLatched && !driving()); // missing magnet, even if I2C acknowledges
  assert(Serial.output.find("AS5600 RIGHT SDA19/SCL18 addr=0x36: MAGNET_NOT_DETECTED") != std::string::npos);

  healthySetup(); command('A'); Wire.registers[0x36][0x0B] = 0x30;
  lastMagnetCheckMs = millis() - 100; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("AS5600 LEFT SDA21/SCL22 addr=0x36: MAGNET_TOO_WEAK") != std::string::npos);

  using namespace BalanceMath;
  assert(fabsf(wrapDeg(181) + 179) < 0.001f);
  assert(fabsf(wrapDeg(-181) - 179) < 0.001f);
  assert(fabsf(fuseDeg(179, 0, -179, 0.005f, 0.245f) - 179.04f) < 0.01f);
  assert(voltage(5, 0, 0, 8.8f, 0.29f, 3) > 0); // forward lean -> forward voltage
  assert(voltage(-5, 0, 0, 8.8f, 0.29f, 3) < 0);
  assert(fabsf(voltage(0, 0, 10, 8.8f, 0.29f, 3) - 0.0506145f) < 0.0001f);
  assert(voltage(45, 0, 100, 8.8f, 0.29f, 3) == 3);
  assert(voltage(-45, 0, -100, 8.8f, 0.29f, 3) == -3);
  float integral = 0;
  assert(speedLeanDeg(5, 0.005f, 0.2f, 0.05f, 3, integral) < 0);
  integral = 0;
  for (int i = 0; i < 10000; ++i) speedLeanDeg(100, 0.005f, 0.2f, 0.05f, 3, integral);
  assert(integral == 0); // no windup when the requested lean is already saturated
  std::cout << "PASS: LEFT/RIGHT encoder diagnostics, Wire errors, magnet flags, initialization,\n"
               "      stale-data arming, stop, timed test, sensor faults,\n"
               "      timing, overspeed, tilt, magnet, angle wrap, gain units and speed anti-windup.\n";
}
