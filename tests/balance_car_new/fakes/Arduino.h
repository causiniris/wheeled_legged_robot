#pragma once
// Host-only test double. Never add this directory to firmware include paths.
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <deque>
#include <string>
#include <algorithm>
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1;
constexpr int INPUT = 2, OUTPUT_OPEN_DRAIN = 3;
inline uint64_t hostTimeUs = 1000000;
inline int hostPins[40]{};
inline int hostPinModes[40]{};
inline void (*hostPinWriteHook)(int, int) = nullptr;
inline int (*hostPinReadHook)(int) = nullptr;
inline uint32_t micros() { return static_cast<uint32_t>(hostTimeUs); }
inline uint32_t millis() { return static_cast<uint32_t>(hostTimeUs / 1000); }
inline void delay(unsigned ms) { hostTimeUs += ms * 1000ULL; }
inline void delayMicroseconds(unsigned us) { hostTimeUs += us; }
inline void digitalWrite(int pin, int value) {
  hostPins[pin] = value;
  if (hostPinWriteHook) hostPinWriteHook(pin, value);
}
inline int digitalRead(int pin) { return hostPinReadHook ? hostPinReadHook(pin) : hostPins[pin]; }
inline void pinMode(int pin, int mode) { hostPinModes[pin] = mode; }
struct HostSerial {
  std::deque<char> input;
  std::string output;
  void setTxBufferSize(int) {}
  void begin(int) {}
  int available() const { return static_cast<int>(input.size()); }
  char read() { char c = input.front(); input.pop_front(); return c; }
  int availableForWrite() const { return 1024; }
  void println(const char* s) { output += s; output += '\n'; }
  void printf(const char* format, ...) {
    char text[512]; va_list args; va_start(args, format);
    vsnprintf(text, sizeof(text), format, args); va_end(args); output += text;
  }
  void write(const uint8_t* data, size_t count) { output.append(reinterpret_cast<const char*>(data), count); }
};
inline HostSerial Serial;
