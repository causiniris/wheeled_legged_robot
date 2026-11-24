#include <cassert>
#include <cmath>
#include <string>
#include "../V1_08_Sensor_Viewer/V1_08_Sensor_Viewer.ino"

int main() {
  setup();
  assert(Wire.sdaPin == 21 && Wire.sclPin == 22);
  assert(Wire1.sdaPin == 17 && Wire1.sclPin == 16);
  float angle = NAN;
  Wire.registers[0x06][0x03] = 0x80;
  Wire.registers[0x06][0x04] = 0x00;
  assert(readAngle(Wire, angle));
  assert(std::fabs(angle - 180.0f) < 0.001f);
  Wire.nackAddress = 0x06;
  assert(!readAngle(Wire, angle));
  Wire.nackAddress = -1;
  Wire1.registers[0x06][0x03] = 0x40;
  Wire.registers[0x68][0x75] = 0x68;
  Wire.registers[0x68][0x3F] = 0x20;
  assert(startMpu());
  assert(Wire.registers[0x68][0x1C] == 0x08);
  mpuReady = true;
  Serial.output.clear();
  sampleSensors();
  assert(Serial.output.find(",180.000,90.000,0.0000,0.0000,1.0000,") != std::string::npos);
  assert(Serial.output.find(",1,1,1\n") != std::string::npos);
  Wire.shortReadAddress = 0x68;
  Serial.output.clear();
  sampleSensors();
  assert(!mpuReady);
  assert(Serial.output.find("# MPU data read failed") != std::string::npos);
  assert(Serial.output.find(",1,1,0\n") != std::string::npos);
  assert(Serial.output.find("nan") != std::string::npos);
}
