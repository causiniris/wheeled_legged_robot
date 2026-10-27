#pragma once
#include <math.h>

// Pure control math, also exercised by tests/balance_car_new/balance_behavior_new_test.cpp.
namespace BalanceMath {
constexpr float RAD_PER_DEG = 0.017453292519943295f;
inline float clamp(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
inline float wrapDeg(float angle) {
  float value = fmodf(angle + 180.0f, 360.0f);
  if (value < 0.0f) value += 360.0f;
  return value - 180.0f;
}
inline float fuseDeg(float previous, float rate, float accel, float dt,
                     float timeConstant) {
  const float predicted = wrapDeg(previous + rate * dt);
  const float correction = dt / (timeConstant + dt);
  return wrapDeg(predicted + correction * wrapDeg(accel - predicted));
}
inline float voltage(float pitch, float target, float rate, float kp, float kd,
                     float limit) {
  return clamp((kp * wrapDeg(pitch - target) + kd * rate) * RAD_PER_DEG,
               -limit, limit);
}
// Positive pitch and wheel speed both mean chassis-forward. A forward-moving
// chassis must lean backward to slow down; do not subtract wheel speed directly
// from motor voltage, which can oppose the fast balance loop's recovery.
inline float speedLeanDeg(float speed, float dt, float kp, float ki,
                          float limit, float& integral) {
  const float candidate = clamp(integral + speed * dt, -100.0f, 100.0f);
  const float raw = -(kp * speed + ki * candidate);
  if (fabsf(raw) <= limit || (raw < -limit && speed < 0.0f) ||
      (raw > limit && speed > 0.0f)) integral = candidate;
  return clamp(-(kp * speed + ki * integral), -limit, limit);
}
}  // namespace BalanceMath
