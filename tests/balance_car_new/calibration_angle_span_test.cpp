#include <cassert>
#include <cmath>
#include <iostream>
#include "../../V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h"

uint64_t calibrationStart;
float stepDegrees;
float gyroSwingDps;

void setWord(uint8_t reg, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  Wire.registers[0x68][reg] = raw >> 8;
  Wire.registers[0x68][reg + 1] = raw & 0xFF;
}

void updateSimulatedImu(uint8_t address, uint8_t reg) {
  if (address != 0x68 || reg != 0x3B) return;
  const bool secondHalf = hostTimeUs - calibrationStart >= 750000;
  const float degrees = secondHalf ? stepDegrees : 0;
  const float radians = degrees * BalanceMath::RAD_PER_DEG;
  setWord(0x3B, static_cast<int16_t>(lroundf(sinf(radians) * 8192)));
  setWord(0x3D, static_cast<int16_t>(lroundf(cosf(radians) * 8192)));
  const float gyro = secondHalf ? gyroSwingDps : -gyroSwingDps;
  setWord(0x47, static_cast<int16_t>(lroundf(gyro * GYRO_SCALE)));
}

int main() {
  Wire.requestHook = updateSimulatedImu;
  calibrationStart = hostTimeUs;
  stepDegrees = 29.0f;
  assert(calibrateImu());

  calibrationStart = hostTimeUs;
  stepDegrees = 31.0f;
  Serial.output.clear();
  assert(!calibrateImu());
  assert(Serial.output.find("angle_span<=30 deg") != std::string::npos);

  calibrationStart = hostTimeUs;
  stepDegrees = 0;
  gyroSwingDps = 3.0f;
  Serial.output.clear();
  assert(calibrateImu() && imuReady);
  assert(Serial.output.find("CALIBRATION WARNING:") != std::string::npos);

  calibrationStart = hostTimeUs;
  gyroSwingDps = 6.0f;
  Serial.output.clear();
  assert(!calibrateImu() && !imuReady);
  assert(Serial.output.find("std<=5.0 dps") != std::string::npos);
  Wire.requestHook = nullptr;
  std::cout << "PASS: calibration angle span, handheld warning, and excess motion rejection\n";
}
