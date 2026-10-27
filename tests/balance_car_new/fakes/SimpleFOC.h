#pragma once
#include "Arduino.h"
#include <stdexcept>
constexpr float _2PI = 6.28318530718f;
class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual float getSensorAngle() = 0;
  void update() { getSensorAngle(); }
 protected:
  virtual void init() { update(); }
};
enum class TorqueControlType { voltage };
enum class MotionControlType { torque };
enum class FOCModulationType { SpaceVectorPWM };
enum class FOCMotorStatus { motor_uncalibrated, motor_init_failed };
enum class Direction { CW = 1, CCW = -1 };
class BLDCDriver3PWM {
 public:
  BLDCDriver3PWM(int, int, int, int enable) : enablePin(enable) {}
  int enablePin;
  bool failInit = false;
  float voltage_power_supply = 0, voltage_limit = 0;
  long pwm_frequency = 0;
  bool init() { return !failInit; }
};
class BLDCMotor {
 public:
  explicit BLDCMotor(int) {}
  TorqueControlType torque_controller{};
  MotionControlType controller{};
  FOCModulationType foc_modulation{};
  FOCMotorStatus motor_status = FOCMotorStatus::motor_uncalibrated;
  Direction sensor_direction = Direction::CW;
  struct { float Tf = 0; } LPF_velocity;
  struct { float q = 0, d = 0; } voltage;
  float target = 0, shaft_velocity = 0, zero_electric_angle = 0;
  float voltage_limit = 0, voltage_sensor_align = 0;
  bool enabled = false;
  void linkSensor(Sensor* s) { sensor_ = s; }
  void linkDriver(BLDCDriver3PWM* d) { driver_ = d; }
  void init() { motor_status = FOCMotorStatus::motor_uncalibrated; enable(); }
  bool initFOC() { sensor_->update(); return true; }
  void disable() {
    if (!driver_) throw std::runtime_error("uninitialized driver dereferenced");
    enabled = false; digitalWrite(driver_->enablePin, LOW);
  }
  void enable() { enabled = true; digitalWrite(driver_->enablePin, HIGH); }
  void loopFOC() { sensor_->update(); }
  void move(float value) { if (enabled) target = voltage.q = value; }
 private:
  Sensor* sensor_ = nullptr;
  BLDCDriver3PWM* driver_ = nullptr;
};
