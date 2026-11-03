#pragma once
/*
 * Adapted from the Arduino/MPU6050/MPU6050.cpp and I2Cdev.cpp in the supplied
 * GY521mpu-6050 archive (Jeff Rowberg, 2011). The archive's I2Cdev read uses
 * STOP between the register write and requestFrom; this adapter retains that
 * sequence and its initialize/getMotion6 register operations while checking
 * all transfers and lengths for use by the motor controller.
 *
 * I2Cdev library collection - MIT license
 * Copyright (c) 2011 Jeff Rowberg
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <Arduino.h>
#include <Wire.h>

struct MpuArchiveDiagnostic {
  uint8_t reg = 0, expected = 0, writeCode = 0;
  size_t received = 0;
};

class MpuArchiveAdapter {
 public:
  MpuArchiveAdapter(TwoWire& bus, uint8_t address) : bus_(bus), address_(address) {}
  MpuArchiveDiagnostic diagnostic;

  bool readBytes(uint8_t reg, uint8_t* data, uint8_t length) {
    diagnostic = {};
    diagnostic.reg = reg;
    diagnostic.expected = length;
    bus_.beginTransmission(address_);
    bus_.write(reg);
    diagnostic.writeCode = bus_.endTransmission(); // STOP, as in archive I2Cdev.cpp.
    if (diagnostic.writeCode != 0) return false;
    diagnostic.received = bus_.requestFrom(address_, length, static_cast<uint8_t>(true));
    if (diagnostic.received != length || bus_.available() < length) {
      while (bus_.available()) bus_.read();
      return false;
    }
    for (uint8_t i = 0; i < length; ++i) data[i] = bus_.read();
    return true;
  }

  bool writeByte(uint8_t reg, uint8_t value) {
    diagnostic = {};
    diagnostic.reg = reg;
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(value);
    diagnostic.writeCode = bus_.endTransmission();
    return diagnostic.writeCode == 0;
  }

  bool writeBits(uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t value) {
    uint8_t old = 0;
    if (!readBytes(reg, &old, 1)) return false;
    const uint8_t shift = bitStart - length + 1;
    const uint8_t mask = ((1U << length) - 1U) << shift;
    return writeByte(reg, (old & ~mask) | ((value << shift) & mask));
  }

  bool initialize() {
    // Same sequence as MPU6050::initialize() from the supplied archive.
    return writeBits(0x6B, 2, 3, 1) && // X gyro PLL clock
           writeBits(0x1B, 4, 2, 0) && // +/-250 deg/s
           writeBits(0x1C, 4, 2, 0) && // +/-2 g
           writeBits(0x6B, 6, 1, 0) && // wake
           writeBits(0x1A, 2, 3, 3) && // preserve existing 44 Hz filter
           writeByte(0x19, 4);          // preserve existing 200 Hz sampling
  }

  bool getMotion6(int16_t& ax, int16_t& ay, int16_t& az,
                  int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t data[14];
    if (!readBytes(0x3B, data, 14)) return false;
    ax = toInt16(data); ay = toInt16(data + 2); az = toInt16(data + 4);
    gx = toInt16(data + 8); gy = toInt16(data + 10); gz = toInt16(data + 12);
    return true;
  }

 private:
  TwoWire& bus_;
  uint8_t address_;
  static int16_t toInt16(const uint8_t* data) {
    return static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
  }
};
