#pragma once
#include "Arduino.h"
#include <vector>

// Pin-level I2C register-device model: independent of Wire/Wire1.
// Exercises the production bit-bang code, not a replacement bus implementation.
struct HostMpuI2C {
  static constexpr int SDA_PIN = 16, SCL_PIN = 17;
  uint8_t registers[256]{};
  uint8_t address = 0x68;
  bool present = true, holdClockLow = false, holdDataLow = false;
  int stretchReads = 0, nackByte = -1;
  uint32_t pauseOnClockUs = 0;
  std::vector<std::vector<uint8_t>> frames;
  std::vector<bool> masterAcks;
  int stops = 0;

  void setWord(int reg, int16_t value) {
    registers[reg] = static_cast<uint16_t>(value) >> 8;
    registers[reg + 1] = static_cast<uint16_t>(value) & 255;
  }
  void writePin(int pin, int value) {
    if (pin == SDA_PIN) {
      const bool before = dataHigh();
      masterDataHigh = value != LOW;
      const bool after = dataHigh();
      if (clockHigh && before && !after) {
        phase = Phase::receive; bits = 0; byte = 0; expectAddress = true;
        active = true; slaveLow = false; frames.emplace_back();
      } else if (clockHigh && !before && after) {
        active = false; phase = Phase::waitStop; slaveLow = false; ++stops;
      }
    } else if (pin == SCL_PIN) {
      masterClockHigh = value != LOW;
      if (!masterClockHigh) {
        if (clockHigh && active) falling();
        clockHigh = false;
      } else if (!holdClockLow && stretchReads == 0 && !clockHigh) {
        clockHigh = true;
        if (active) rising();
      }
    }
  }
  int readPin(int pin) {
    if (pin == SCL_PIN) {
      hostTimeUs += pauseOnClockUs; pauseOnClockUs = 0;
      if (!masterClockHigh || holdClockLow) return LOW;
      if (stretchReads > 0) { --stretchReads; return LOW; }
      if (!clockHigh) { clockHigh = true; if (active) rising(); }
      return HIGH;
    }
    if (pin == SDA_PIN) return dataHigh() ? HIGH : LOW;
    return hostPins[pin];
  }

 private:
  enum class Phase { receive, beginAck, slaveAck, send, beginMasterAck,
                     masterAck, endMasterAck, waitStop };
  Phase phase = Phase::waitStop, afterAck = Phase::receive;
  bool active = false, masterDataHigh = true, masterClockHigh = true;
  bool clockHigh = true, slaveLow = false, expectAddress = true, expectRegister = true;
  bool acknowledge = false, continueRead = false;
  uint8_t byte = 0, regPointer = 0, sendValue = 0;
  int bits = 0;
  bool dataHigh() const { return masterDataHigh && !slaveLow && !holdDataLow; }
  void received() {
    frames.back().push_back(byte);
    acknowledge = present && static_cast<int>(frames.back().size()) != nackByte;
    afterAck = Phase::receive;
    if (expectAddress) {
      acknowledge = acknowledge && (byte >> 1) == address;
      expectAddress = false;
      if (byte & 1) afterAck = Phase::send;
      else expectRegister = true;
    } else if (acknowledge) {
      if (expectRegister) { regPointer = byte; expectRegister = false; }
      else registers[regPointer++] = byte;
    }
    if (!acknowledge) afterAck = Phase::waitStop;
  }
  void beginSend() {
    phase = Phase::send; bits = 0; sendValue = registers[regPointer++];
    slaveLow = (sendValue & 0x80) == 0;
  }
  void rising() {
    switch (phase) {
      case Phase::receive:
        byte = static_cast<uint8_t>((byte << 1) | (dataHigh() ? 1 : 0));
        if (++bits == 8) { received(); phase = Phase::beginAck; }
        break;
      case Phase::send:
        if (++bits == 8) phase = Phase::beginMasterAck;
        break;
      case Phase::masterAck:
        continueRead = !dataHigh(); masterAcks.push_back(continueRead);
        phase = Phase::endMasterAck;
        break;
      default: break;
    }
  }
  void falling() {
    switch (phase) {
      case Phase::beginAck: slaveLow = acknowledge; phase = Phase::slaveAck; break;
      case Phase::slaveAck:
        slaveLow = false; phase = afterAck; bits = 0; byte = 0;
        if (phase == Phase::send) beginSend();
        break;
      case Phase::send: slaveLow = (sendValue & (0x80 >> bits)) == 0; break;
      case Phase::beginMasterAck: slaveLow = false; phase = Phase::masterAck; break;
      case Phase::endMasterAck:
        if (continueRead) beginSend();
        else { slaveLow = false; phase = Phase::waitStop; }
        break;
      default: break;
    }
  }
};
inline HostMpuI2C hostMpu;
inline void attachHostMpu() {
  hostPinWriteHook = [](int pin, int value) { hostMpu.writePin(pin, value); };
  hostPinReadHook = [](int pin) { return hostMpu.readPin(pin); };
}
