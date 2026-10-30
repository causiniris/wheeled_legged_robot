#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

#include "../../V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino"

void healthyEncoders() {
  Wire.registers[0x36][0x0B] = 0x20;
  Wire1.registers[0x36][0x0B] = 0x20;
}

void queueStopDuringAlignment() { Serial.input.push_back('X'); }

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string scenario = argv[1];
  healthyEncoders();

  if (scenario == "healthy") {
    setup();
    assert(running && !faultLatched);
    assert(hostPins[LEFT_EN] == HIGH && hostPins[RIGHT_EN] == HIGH);
    assert(std::fabs(leftMotor.voltage.q - TEST_VOLTAGE) < 0.0001f);
    assert(std::fabs(rightMotor.voltage.q + TEST_VOLTAGE) < 0.0001f);
  } else if (scenario == "encoder-failure") {
    Wire.nackAddress = 0x36;
    setup();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "magnet-failure") {
    Wire1.registers[0x36][0x0B] = 0x00;
    setup();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "serial-stop") {
    setup();
    Serial.input.push_back('X');
    loop();
    assert(!running && !leftMotor.enabled && !rightMotor.enabled);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "startup-stop") {
    hostInitFocHook = queueStopDuringAlignment;
    setup();
    assert(hostInitFocCalls == 1);
    assert(!running && !leftMotor.enabled && !rightMotor.enabled);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "runtime-encoder-failure") {
    setup();
    Wire.shortReadAddress = 0x36;
    loop();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "overspeed") {
    setup();
    leftMotor.shaft_velocity = MAX_WHEEL_SPEED + 1.0f;
    loop();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else {
    return 2;
  }

  std::cout << "PASS: " << scenario << '\n';
}
