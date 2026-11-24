// Sensor-only diagnostic. Motor EN pins remain LOW; no motor drive is started.
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "../V1_06_TwoWheel_Balance_Car_new/MPU6050_archive_adapter.h"

namespace {
constexpr uint8_t ENCODER_ADDRESS = 0x06;
constexpr uint8_t MPU_ADDRESS = 0x68; // AD0 to GND
constexpr uint32_t SAMPLE_MS = 50;
MpuArchiveAdapter mpu(Wire, MPU_ADDRESS);
bool mpuReady = false;
bool busesReady = false;
uint32_t lastSample = 0, lastMpuAttempt = 0;

bool readAngle(TwoWire& bus, float& degrees) {
  uint8_t bytes[2];
  for (int i = 0; i < 2; ++i) {
    bus.beginTransmission(ENCODER_ADDRESS);
    bus.write(0x03 + i);
    if (bus.endTransmission(false) != 0 ||
        bus.requestFrom(ENCODER_ADDRESS, uint8_t(1), uint8_t(true)) != 1 ||
        bus.available() < 1) return false;
    bytes[i] = bus.read();
  }
  degrees = ((uint16_t(bytes[0]) << 6) | (bytes[1] >> 2)) * (360.0f / 16384.0f);
  return true;
}

void scan(TwoWire& bus, const char* label) {
  Serial.printf("# %s I2C scan:", label);
  bool found = false;
  for (uint8_t address = 1; address < 127; ++address) {
    bus.beginTransmission(address);
    if (bus.endTransmission() == 0) {
      Serial.printf(" 0x%02X", unsigned(address));
      found = true;
    }
  }
  Serial.println(found ? "" : " none");
}

bool startMpu() {
  uint8_t id = 0;
  if (!mpu.readBytes(0x75, &id, 1)) {
    Serial.printf("# MPU read failed addr=0x68 reg=0x%02X wire_code=%u received=%u\n",
                  unsigned(mpu.diagnostic.reg), unsigned(mpu.diagnostic.writeCode),
                  unsigned(mpu.diagnostic.received));
    return false;
  }
  Serial.printf("# MPU WHO_AM_I=0x%02X%s\n", unsigned(id),
                id == 0x68 ? "" : " (unverified compatible device)");
  if (id == 0 || id == 0xFF || !mpu.initialize()) {
    Serial.printf("# MPU init failed reg=0x%02X wire_code=%u received=%u\n",
                  unsigned(mpu.diagnostic.reg), unsigned(mpu.diagnostic.writeCode),
                  unsigned(mpu.diagnostic.received));
    return false;
  }
  delay(100);
  return true;
}

void sampleSensors() {
  float left = NAN, right = NAN;
  const bool leftOk = readAngle(Wire, left);
  const bool rightOk = readAngle(Wire1, right);
  int16_t ax, ay, az, gx, gy, gz;
  const bool mpuOk = mpuReady && mpu.getMotion6(ax, ay, az, gx, gy, gz);
  if (mpuReady && !mpuOk) {
    Serial.printf("# MPU data read failed reg=0x%02X wire_code=%u received=%u\n",
                  unsigned(mpu.diagnostic.reg), unsigned(mpu.diagnostic.writeCode),
                  unsigned(mpu.diagnostic.received));
    mpuReady = false;
    lastMpuAttempt = millis();
  }
  Serial.printf("%lu,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%d,%d\n",
                static_cast<unsigned long>(millis()), left, right,
                mpuOk ? ax / MpuArchiveAdapter::ACCEL_COUNTS_PER_G : NAN,
                mpuOk ? ay / MpuArchiveAdapter::ACCEL_COUNTS_PER_G : NAN,
                mpuOk ? az / MpuArchiveAdapter::ACCEL_COUNTS_PER_G : NAN, mpuOk ? gx / 131.0f : NAN,
                mpuOk ? gy / 131.0f : NAN, mpuOk ? gz / 131.0f : NAN,
                int(leftOk), int(rightOk), int(mpuOk));
}
}

void setup() {
  digitalWrite(12, LOW); digitalWrite(13, LOW);
  pinMode(12, OUTPUT); pinMode(13, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("# SENSOR VIEWER: motor EN12/EN13 LOW");
  if (!Wire.begin(21, 22, 400000) || !Wire1.begin(17, 16, 400000)) {
    Serial.println("# I2C initialization failed; reset after checking");
    return;
  }
  Wire.setTimeOut(3); Wire1.setTimeOut(3);
  busesReady = true;
  scan(Wire, "LEFT GPIO21/22");
  scan(Wire1, "RIGHT GPIO17/16");
  mpuReady = startMpu();
  lastMpuAttempt = millis();
  Serial.println("ms,left_deg,right_deg,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,left_ok,right_ok,mpu_ok");
}

void loop() {
  if (!busesReady) { delay(100); return; }
  if (millis() - lastMpuAttempt >= 1000 && !mpuReady) {
    lastMpuAttempt = millis();
    mpuReady = startMpu();
  }
  if (millis() - lastSample >= SAMPLE_MS) {
    lastSample = millis();
    sampleSensors();
  }
}
