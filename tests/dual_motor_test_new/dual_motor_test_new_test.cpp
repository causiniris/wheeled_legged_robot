#include <cassert>
#include <atomic>
#include <cmath>
#include <iostream>
#include <string>
#include <type_traits>

#include "../../V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino"

static_assert(std::is_same<decltype(stopRequested), std::atomic<bool>>::value,
              "The cross-core stop latch must be atomic");
static_assert(LEFT_I2C_SDA == 21 && LEFT_I2C_SCL == 22,
              "Left MT6701 must use SDA21/SCL22");
static_assert(RIGHT_I2C_SDA == 19 && RIGHT_I2C_SCL == 18,
              "Right MT6701 must use SDA19/SCL18");
static_assert(LEFT_IN1 == 32 && LEFT_IN2 == 33 && LEFT_IN3 == 25 && LEFT_EN == 12,
              "Left driver pins must match the wiring table");
static_assert(RIGHT_IN1 == 14 && RIGHT_IN2 == 27 && RIGHT_IN3 == 26 && RIGHT_EN == 13,
              "Right driver pins must match the wiring table");

void healthyEncoders() {
  Wire.registers[0x06][0x03] = 0x80;
  Wire1.registers[0x06][0x03] = 0x40;
}

void queueStopDuringAlignment() {
  Serial.input.push_back('X');
  pollStopRequest();
  assert(stopRequested.load(std::memory_order_acquire));
  assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
}

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
    assert(std::fabs(leftSensor.getSensorAngle() - _2PI * 0.5f) < 0.0001f);
    Wire.registers[0x06][0x03] = 0x00;
    Wire.registers[0x06][0x04] = 0xFC;
    assert(std::fabs(leftSensor.getSensorAngle() - 63.0f * _2PI / 16384.0f) < 0.0001f);
  } else if (scenario == "encoder-failure") {
    Wire.nackAddress = 0x06;
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
    Wire.shortReadAddress = 0x06;
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
