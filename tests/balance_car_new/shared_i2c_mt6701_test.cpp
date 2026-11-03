#include <cassert>
#include <cmath>
#include <iostream>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

int main() {
  static_assert(LEFT_I2C_SDA == 21 && LEFT_I2C_SCL == 22, "shared left bus pins");
  static_assert(RIGHT_I2C_SDA == 19 && RIGHT_I2C_SCL == 18, "right bus pins");
  Wire.registers[0x06][0x03] = 0x80;
  Wire1.registers[0x06][0x03] = 0x40;
  Wire.registers[0x68][0x75] = 0x68;
  Wire.registers[0x68][0x3F] = 0x40;
  Wire.registers[0x68][0x1B] = 0xFF;
  Wire.registers[0x68][0x1C] = 0xFF;
  setup();
  assert(Wire.sdaPin == 21 && Wire.sclPin == 22);
  assert(Wire1.sdaPin == 19 && Wire1.sclPin == 18);
  assert(motorsReady && imuReady && !faultLatched);
  assert(std::fabs(leftSensor.getSensorAngle() - _2PI * 0.5f) < 0.0001f);
  assert(std::fabs(rightSensor.getSensorAngle() - _2PI * 0.25f) < 0.0001f);
  assert(Wire.registers[0x68][0x6B] == 1);
  assert(Wire.registers[0x68][0x19] == 4);
  assert(Wire.registers[0x68][0x1B] == 0xE7);
  assert(Wire.registers[0x68][0x1C] == 0xE7);
  assert(Wire.lastStopByAddress[0x68]); // The supplied I2Cdev reads use STOP before requestFrom.
  Serial.input.push_back('A');
  processSerial();
  assert(armed);
  Wire.shortReadAddress = 0x68;
  hostTimeUs += 5000;
  loop();
  assert(faultLatched && !armed && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  std::cout << "PASS: shared MT6701/MPU6050 bus, angle decode, IMU failure stop\n";
}
