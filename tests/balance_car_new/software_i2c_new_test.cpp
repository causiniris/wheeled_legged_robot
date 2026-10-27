#include <cassert>
#include <iostream>
#include "fakes/HostMpuI2C_new.h"
#include "../../V1_06_TwoWheel_Balance_Car_new/SoftwareI2C_new.h"

void resetDevice() { hostMpu = HostMpuI2C{}; attachHostMpu(); }
int main() {
  resetDevice();
  SoftwareI2C_new bus(16, 17);
  assert(bus.begin());
  assert(hostPinModes[16] == OUTPUT_OPEN_DRAIN && hostPinModes[17] == OUTPUT_OPEN_DRAIN);
  assert(bus.writeRegister(0x68, 0x6B, 0x01));
  assert(hostMpu.registers[0x6B] == 1);
  assert((hostMpu.frames.back() == std::vector<uint8_t>{0xD0, 0x6B, 0x01}));
  assert(hostMpu.stops == 1);

  // Register addressing, repeated START, MSB-first data, 13 ACKs and final NACK.
  const uint8_t expected[14] = {0x80, 0x01, 0x7F, 0xFF, 0x00, 0x40, 0xA5,
                               0x5A, 0x12, 0x34, 0xFE, 0xDC, 0x00, 0x03};
  for (int i = 0; i < 14; ++i) hostMpu.registers[0x3B + i] = expected[i];
  hostMpu.frames.clear(); hostMpu.masterAcks.clear();
  uint8_t data[14]{};
  assert(bus.readRegisters(0x68, 0x3B, data, 14));
  for (int i = 0; i < 14; ++i) assert(data[i] == expected[i]);
  assert(hostMpu.frames.size() == 2);
  assert((hostMpu.frames[0] == std::vector<uint8_t>{0xD0, 0x3B}));
  assert((hostMpu.frames[1] == std::vector<uint8_t>{0xD1}));
  assert(hostMpu.masterAcks.size() == 14);
  for (int i = 0; i < 13; ++i) assert(hostMpu.masterAcks[i]);
  assert(!hostMpu.masterAcks.back() && hostMpu.stops == 2);
  assert(bus.durationUs() < SoftwareI2C_new::TRANSACTION_TIMEOUT_US);
  hostMpu.masterAcks.clear();
  assert(bus.readRegisters(0x68, 0x3B, data, 1));
  assert(hostMpu.masterAcks.size() == 1 && !hostMpu.masterAcks[0]);

  hostMpu.stretchReads = 50;
  assert(bus.readRegisters(0x68, 0x3B, data, 1));
  hostMpu.present = false;
  assert(!bus.readRegisters(0x68, 0x3B, data, 14));
  assert(bus.error() == SoftwareI2C_new::Error::nack);
  assert(hostPins[16] == HIGH && hostPins[17] == HIGH); // both released on error
  hostMpu.present = true;
  assert(!bus.readRegisters(0x69, 0x75, data, 1)); // wrong address
  assert(bus.error() == SoftwareI2C_new::Error::nack);

  // Register and data-byte NACKs must abort, not silently report a write success.
  for (int rejected : {2, 3}) {
    resetDevice(); assert(bus.begin()); hostMpu.nackByte = rejected;
    assert(!bus.writeRegister(0x68, 0x6B, 0x01));
    assert(bus.error() == SoftwareI2C_new::Error::nack && hostMpu.registers[0x6B] == 0);
  }
  resetDevice(); assert(bus.begin()); hostMpu.holdClockLow = true;
  const auto before = hostTimeUs;
  assert(!bus.readRegisters(0x68, 0x3B, data, 14));
  assert(bus.error() == SoftwareI2C_new::Error::timeout);
  assert(hostTimeUs - before <= SoftwareI2C_new::TRANSACTION_TIMEOUT_US + 20);
  assert(hostPins[16] == HIGH && hostPins[17] == HIGH);

  resetDevice(); hostMpu.holdDataLow = true; assert(!bus.begin());
  assert(bus.error() == SoftwareI2C_new::Error::busBusy);
  resetDevice(); hostMpu.holdClockLow = true; assert(!bus.begin());
  assert(bus.error() == SoftwareI2C_new::Error::timeout);
  resetDevice(); assert(bus.begin()); hostMpu.pauseOnClockUs = 4000;
  assert(!bus.readRegisters(0x68, 0x3B, data, 1)); // preemption counts in deadline
  assert(bus.error() == SoftwareI2C_new::Error::timeout);
  resetDevice(); assert(bus.begin());
  assert(!bus.readRegisters(0x68, 0x3B, nullptr, 1));
  assert(!bus.readRegisters(0x68, 0x3B, data, 0));

  // Correct unsigned deadline arithmetic across the micros() wrap.
  hostTimeUs = 0xFFFFFFF0ULL;
  assert(bus.readRegisters(0x68, 0x3B, data, 14));
  assert(bus.durationUs() < SoftwareI2C_new::TRANSACTION_TIMEOUT_US);
  std::cout << "PASS: software I2C waveform, repeated START, burst/single read, ACK/NACK,\n"
               "      stretching, stuck lines, transfer deadline, cleanup and timer wrap.\n";
}
