# Dual-Motor Test Firmware Design

## Goal

Add a standalone ESP32 firmware target for testing both wheel motors without an
MPU6050. After both AS5600 encoders pass initialization and magnet checks, the
firmware aligns both motors and drives both wheels in the vehicle-forward
direction at a fixed 0.5 V q-axis command until the operator sends `X`.

The existing balance firmware and the repository's default PlatformIO
environment remain unchanged.

## Structure

- Add `V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino` as the Arduino IDE
  entry point and complete test implementation.
- Add `src/dual_motor_test_new.cpp` as the PlatformIO entry point that includes
  the Arduino sketch.
- Add a `dual_motor_test_new` environment to `platformio.ini`, selecting only
  `dual_motor_test_new.cpp` and inheriting the existing ESP32, serial, and
  SimpleFOC configuration.
- Add the new target and its commands to `README_new.md`.

The test firmware is independent of `BalanceController_new.h` and contains no
MPU6050 initialization, reads, calibration, or fault conditions.

## Startup and Runtime Flow

1. Hold both driver-enable pins low and start the serial port at 115200 baud.
2. Initialize the two independent AS5600 I2C buses.
3. Check both encoders for I2C communication and a valid magnet-detected state.
4. Initialize both angle sensors. Any failure latches a fault and leaves both
   drivers disabled.
5. Initialize and align the left motor, then the right motor. The wheels must be
   off the ground because FOC alignment moves them.
6. After both alignments succeed, enable both motors and continuously apply a
   0.5 V q-axis command. Use the existing vehicle-forward signs: left `+1`,
   right `-1`.
7. Continuously run both FOC loops and publish left/right shaft velocity and
   state at a low serial telemetry rate.
8. On `X` or `x`, immediately command zero voltage, disable both motors, and
   hold both enable pins low. The firmware remains stopped until the ESP32 is
   reset.

## Safety and Fault Handling

- Any encoder I2C failure, invalid/absent magnet indication, non-finite shaft
  velocity, or absolute shaft speed above 30 rad/s stops and disables both
  motors.
- A fault or operator stop is latched for the remainder of that boot; there is
  no serial restart command.
- Driver supply, motor voltage limits, PWM frequency, pole-pair count, pins,
  and encoder buses match the current balance firmware so the same hardware can
  be tested without rewiring.
- The fixed 0.5 V command is a voltage command, not a current limit.

## Verification

- Add a host test that compiles the actual dual-motor controller with the
  repository's fake Arduino, Wire, and SimpleFOC interfaces.
- Verify that encoder or magnet failure prevents motion, successful startup
  commands both wheels in the configured forward direction, `X` disables both
  outputs, and runtime encoder/overspeed faults stop both motors.
- Run the host test and a clean PlatformIO build of `dual_motor_test_new`.
- Hardware upload and wheel-direction confirmation remain manual because they
  require the connected robot. The first hardware run must use supported,
  off-ground wheels.
