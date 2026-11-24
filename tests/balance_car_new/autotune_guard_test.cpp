#include <cassert>
#include <iostream>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

void command(char c) { Serial.input.push_back(c); processSerial(); }

int main() {
  Wire.registers[0x68][0x75] = 0x68;
  Wire.registers[0x68][0x3D] = 0x20;
  setup();
  assert(motorsReady && imuReady && !driving());

  command('H');
  assert(Serial.output.find("AUTOTUNE GUARD ON") != std::string::npos);
  for (int i = 0; i < 110; ++i) { hostTimeUs += 5000; loop(); }
  command('A');
  assert(!armed);
  assert(Serial.output.find("Cannot start: autotune heartbeat expired") != std::string::npos);
  command('H');
  command('A');
  assert(armed);
  for (int i = 0; i < 80; ++i) {
    hostTimeUs += 5000;
    if (i == 40) command('H');
    loop();
  }
  assert(armed);
  for (int i = 0; i < 110; ++i) {
    hostTimeUs += 5000;
    loop();
  }
  assert(faultLatched && !driving());
  assert(!hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
  assert(Serial.output.find("autotune heartbeat timeout") != std::string::npos);
  command('H'); command('A');
  assert(!armed);
  assert(faultLatched);
  std::cout << "PASS: autotune heartbeat timeout disables motors and latches fault\n";
}
