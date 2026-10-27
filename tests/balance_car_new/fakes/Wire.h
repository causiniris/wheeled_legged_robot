#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  uint8_t registers[128][256]{};
  int nackAddress = -1, shortReadAddress = -1;
  bool begin(int, int, int) { return true; }
  void setTimeOut(int) {}
  void beginTransmission(uint8_t address) { address_ = address; first_ = true; }
  void write(uint8_t value) {
    if (first_) { reg_ = value; first_ = false; }
    else registers[address_][reg_] = value;
  }
  int endTransmission(bool = true) { return address_ == nackAddress ? 4 : 0; }
  size_t requestFrom(uint8_t address, uint8_t length, uint8_t) {
    input_.clear();
    const int count = address == shortReadAddress ? length - 1 : length;
    for (int i = 0; i < count; ++i) input_.push_back(registers[address][reg_ + i]);
    return input_.size();
  }
  int available() const { return static_cast<int>(input_.size()); }
  int read() { if (input_.empty()) return -1; int v = input_.front(); input_.pop_front(); return v; }
 private:
  int address_ = 0, reg_ = 0;
  bool first_ = true;
  std::deque<uint8_t> input_;
};
inline TwoWire Wire, Wire1;
