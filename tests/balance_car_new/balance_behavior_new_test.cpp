// Compiles the ACTUAL controller with fake buses/drivers. No physical plant is
// simulated, so these checks do not validate real-world stability or torque.
#include <cassert>
#include <cmath>
#include <iostream>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

void command(char c) { Serial.input.push_back(c); processSerial(); }
void wordAt(TwoWire& bus, int address, int reg, int16_t value) {
  bus.registers[address][reg] = static_cast<uint16_t>(value) >> 8;
  bus.registers[address][reg + 1] = static_cast<uint16_t>(value) & 255;
}
void healthySetup() {
  faultLatched = false; imuReady = motorsReady = armed = false; testWheel = 0;
  leftSensor.failed = rightSensor.failed = false;
  leftDriver.failInit = rightDriver.failInit = false;
  Wire.nackAddress = Wire.shortReadAddress = Wire1.nackAddress = Wire1.shortReadAddress = -1;
  Wire.registers[0x36][0x0B] = Wire1.registers[0x36][0x0B] = 0x20;
  Wire.registers[0x68][0x75] = 0x68;
  wordAt(Wire, 0x68, 0x3B, 0); wordAt(Wire, 0x68, 0x3D, 0); wordAt(Wire, 0x68, 0x3F, 16384);
  wordAt(Wire, 0x68, 0x43, 0); wordAt(Wire, 0x68, 0x45, 0); wordAt(Wire, 0x68, 0x47, 0);
  leftMotor.shaft_velocity = rightMotor.shaft_velocity = 0;
  setup();
  assert(motorsReady && imuReady && !faultLatched && !driving());
  assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
}
int main() {
  // Early initialization failure must not dereference a null motor driver.
  leftLinked = rightLinked = false;
  leftDriver.failInit = true;
  assert(!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT"));
  fault("test failed initialization");
  assert(!leftLinked && !rightLinked && faultLatched);

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
  Wire.shortReadAddress = -1; command('A'); assert(!armed); // errors stay latched

  healthySetup(); command('A'); Wire.shortReadAddress = 0x68; hostTimeUs += 5000; loop();
  assert(faultLatched && !driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); hostTimeUs += MAX_CONTROL_GAP_US + 1; loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); leftMotor.shaft_velocity = MAX_WHEEL_SPEED + 1; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); command('A'); pitchAngleDeg = 45;
  wordAt(Wire, 0x68, 0x3D, 11585); wordAt(Wire, 0x68, 0x3F, 11585);
  hostTimeUs += 5000; loop();
  assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);

  healthySetup(); Wire1.registers[0x36][0x0B] = 0; command('R');
  assert(faultLatched && !driving()); // missing magnet, even if I2C acknowledges

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
  std::cout << "PASS: initialization, stale-data arming, stop, timed test, sensor faults,\n"
               "      timing, overspeed, tilt, magnet, angle wrap, gain units and speed anti-windup.\n";
}
