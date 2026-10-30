# Dual-Motor Weak-Magnet Bypass Design

## Goal

Allow the standalone `dual_motor_test_new` firmware to continue motor testing
when an AS5600 reports `MAGNET_TOO_WEAK`. The balance firmware remains
unchanged.

## Behavior

- An AS5600 status with the `ML` bit set and the `MH` bit clear is accepted for
  this test even when `MD` is clear, matching the observed right-encoder status
  `0x13`.
- The serial log prints a clear `WARNING: MAGNET_TOO_WEAK; continuing test`
  message instead of reporting the encoder as healthy.
- I2C failures, short reads, `MH` (magnet too strong), no `MD/ML/MH` indication,
  and invalid simultaneous `ML`/`MH` indications remain latched faults that
  disable both motors.
- The relaxed check applies at startup and during the periodic runtime magnet
  check.
- Angle-read failures, non-finite speed, overspeed, and serial `X` behavior are
  unchanged.

## Scope

- Modify only the new dual-motor test controller, its host test, and its README
  section.
- Do not modify `BalanceController_new.h` or the `balance_car_new` environment.
- Do not add a general configuration system or bypass all magnet diagnostics.

## Verification

- Add a failing host scenario showing that startup status `0x10`/`0x13`
  currently prevents motion, then make it pass with a warning.
- Keep or add scenarios proving missing magnet and magnet-too-strong status
  still latch both motors off.
- Run both dual-motor and balance host suites.
- Build `dual_motor_test_new` and `balance_car_new` with PlatformIO.
- Hardware upload and physical wheel testing remain manual and require both
  wheels to be supported off the ground.
