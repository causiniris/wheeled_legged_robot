#pragma once
#include <Arduino.h>

// Single-master, register-oriented software I2C for the MPU6050 ONLY.
// Independent of ESP32 Wire/Wire1, which stay connected to the two AS5600s.
// GPIO HIGH releases an open-drain line; external/module pull-ups to 3.3 V
// are required. No interrupts are disabled. This is a blocking transaction,
// bounded by a whole-transfer deadline (including any clock stretching).
class SoftwareI2C_new {
 public:
  enum class Error { none, notReady, busBusy, nack, timeout, invalidArgument };
  static constexpr uint32_t TRANSACTION_TIMEOUT_US = 3000;
  static constexpr uint32_t HALF_PERIOD_US = 3; // <=167 kHz before GPIO overhead

  SoftwareI2C_new(int sda, int scl) : sda_(sda), scl_(scl) {}

  bool begin() {
    // Input first: do not create a push-pull high level while changing modes.
    pinMode(sda_, INPUT);
    pinMode(scl_, INPUT);
    digitalWrite(sda_, HIGH);
    digitalWrite(scl_, HIGH);
    pinMode(sda_, OUTPUT_OPEN_DRAIN);
    pinMode(scl_, OUTPUT_OPEN_DRAIN);
    ready_ = true;
    resetDeadline();
    const bool clockReady = raiseClock();
    if (clockReady && digitalRead(sda_) == LOW) error_ = Error::busBusy;
    ready_ = clockReady && error_ == Error::none;
    releaseLines();
    durationUs_ = micros() - startUs_;
    return ready_;
  }

  bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    if (!prepare(address)) return false;
    const bool ok = start() && writeByte(address << 1) &&
                    writeByte(reg) && writeByte(value);
    return finish(ok);
  }

  bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint8_t length) {
    if (!prepare(address)) return false;
    if (data == nullptr || length == 0) {
      error_ = Error::invalidArgument;
      return false;
    }
    bool ok = start() && writeByte(address << 1) && writeByte(reg) &&
              start() && writeByte((address << 1) | 1); // repeated START
    for (uint8_t i = 0; ok && i < length; ++i)
      ok = readByte(data[i], i + 1 < length); // ACK every byte except final NACK
    return finish(ok);
  }

  Error error() const { return error_; }
  uint32_t durationUs() const { return durationUs_; }
  const char* errorText() const {
    switch (error_) {
      case Error::none: return "MPU software I2C OK";
      case Error::notReady: return "MPU software I2C not initialized";
      case Error::busBusy: return "MPU SDA16 stuck LOW; check wiring/pull-ups";
      case Error::nack: return "MPU no ACK on SDA16/SCL17; check power/address/wiring";
      case Error::timeout: return "MPU software I2C timeout; check SCL17/pull-ups";
      default: return "MPU software I2C invalid argument";
    }
  }

 private:
  int sda_, scl_;
  bool ready_ = false;
  Error error_ = Error::notReady;
  uint32_t startUs_ = 0, durationUs_ = 0;

  void resetDeadline() {
    error_ = Error::none;
    startUs_ = micros();
    durationUs_ = 0;
  }
  bool prepare(uint8_t address) {
    resetDeadline();
    if (!ready_) { error_ = Error::notReady; return false; }
    if (address > 0x7F) { error_ = Error::invalidArgument; return false; }
    return true;
  }
  bool inTime() {
    if (micros() - startUs_ < TRANSACTION_TIMEOUT_US) return true;
    error_ = Error::timeout;
    return false;
  }
  void releaseLines() {
    digitalWrite(scl_, HIGH);
    digitalWrite(sda_, HIGH);
  }
  bool raiseClock() {
    digitalWrite(scl_, HIGH);
    while (digitalRead(scl_) == LOW) {
      if (!inTime()) return false;
      delayMicroseconds(1);
    }
    delayMicroseconds(HALF_PERIOD_US);
    return inTime();
  }
  bool start() {
    digitalWrite(sda_, HIGH);
    delayMicroseconds(HALF_PERIOD_US);
    if (!raiseClock()) return false;
    if (digitalRead(sda_) == LOW) { error_ = Error::busBusy; return false; }
    digitalWrite(sda_, LOW);
    delayMicroseconds(HALF_PERIOD_US);
    digitalWrite(scl_, LOW);
    return inTime();
  }
  bool writeBit(bool high) {
    digitalWrite(sda_, high ? HIGH : LOW);
    delayMicroseconds(HALF_PERIOD_US);
    if (!raiseClock()) return false;
    digitalWrite(scl_, LOW);
    return true;
  }
  bool readBit(bool& high) {
    digitalWrite(sda_, HIGH);
    delayMicroseconds(HALF_PERIOD_US);
    if (!raiseClock()) return false;
    high = digitalRead(sda_) == HIGH;
    digitalWrite(scl_, LOW);
    return true;
  }
  bool writeByte(uint8_t value) {
    for (uint8_t mask = 0x80; mask; mask >>= 1)
      if (!writeBit((value & mask) != 0)) return false;
    bool nack = true;
    if (!readBit(nack)) return false;
    if (nack) { error_ = Error::nack; return false; }
    return true;
  }
  bool readByte(uint8_t& value, bool ack) {
    value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      bool high = false;
      if (!readBit(high)) return false;
      value = static_cast<uint8_t>((value << 1) | high);
    }
    return writeBit(!ack);
  }
  bool finish(bool ok) {
    // Try STOP with the original deadline, never start a second timeout.
    digitalWrite(scl_, LOW);
    digitalWrite(sda_, LOW);
    delayMicroseconds(HALF_PERIOD_US);
    const bool clockReady = raiseClock();
    digitalWrite(sda_, HIGH);
    delayMicroseconds(HALF_PERIOD_US);
    const bool idle = clockReady && digitalRead(sda_) == HIGH;
    if (clockReady && !idle && error_ == Error::none) error_ = Error::busBusy;
    releaseLines();
    durationUs_ = micros() - startUs_;
    return inTime() && ok && idle && error_ == Error::none;
  }
};
