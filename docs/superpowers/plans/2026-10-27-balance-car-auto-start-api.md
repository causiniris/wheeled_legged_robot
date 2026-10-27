# Balance Car Auto-Start and Project API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a small public MPU6050/AS5600/motor API, automatically enter balance after calibration, replace chassis-dependent stop thresholds with debounced fall detection, and make the balance firmware the default ESP32-WROOM build.

**Architecture:** Keep `BalanceController_new.h` as the single controller implementation and add `BalanceCarApi_new.h` as its public contract. The existing SimpleFOC and MPU6050 objects remain private; public functions translate their state into stable project-level data types and the main control loop uses the same motor API exposed to callers. Preserve the existing 200 Hz complementary-filter plus angle-PD controller, while changing only startup, runtime tuning, and stop-state behavior.

**Tech Stack:** C++17 host tests, Arduino-ESP32 2.0.17, PlatformIO `espressif32@7.1.3`, `esp32dev` for ESP32-WROOM-32 DevKit, SimpleFOC 2.3.3, PowerShell.

## Global Constraints

- Keep `V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h` as the main logic file.
- Do not add an MPU6050 dependency; continue using direct I2C register access.
- Keep motor control in SimpleFOC voltage-torque mode and clamp project API commands to ±3 V.
- Keep the posture loop target at 5 ms (200 Hz) and leave the optional speed PI loop disabled by default.
- Preserve I2C, non-finite-data, FOC-initialization, 30 ms control-overrun, and serial `X` hard stops.
- Remove the 8°/20°/s/1 rad/s start gates and the 30 rad/s runtime stop.
- Treat tilt as fallen only after more than 60° from calibrated upright for 300 ms continuously.
- Do not claim real-robot stability from host tests or firmware compilation.
- Do not upload firmware as part of implementation verification.

---

## File Map

- Create `V1_06_TwoWheel_Balance_Car_new/BalanceCarApi_new.h`: public data types and function declarations only.
- Modify `V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h`: implement the public API and use it from startup/control logic.
- Modify `tests/balance_car_new/fakes/SimpleFOC.h`: model `shaft_angle` used by the public encoder interface.
- Modify `tests/balance_car_new/balance_behavior_new_test.cpp`: cover public interfaces, auto-start, live tuning, fall debounce, retained faults, and removed thresholds.
- Modify `platformio.ini`: select `balance_car_new` by default.
- Modify `README_balance_new.md`: describe the API, auto-start state machine, safety boundary, build, upload, and run sequence.
- Modify `README_new.md`: remove stale statements that the default environment is `sensor_test` and that balance waits for `A` at every boot.

### Task 1: Public sensor and motor API

**Files:**
- Create: `V1_06_TwoWheel_Balance_Car_new/BalanceCarApi_new.h`
- Modify: `V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h`
- Modify: `tests/balance_car_new/fakes/SimpleFOC.h`
- Test: `tests/balance_car_new/balance_behavior_new_test.cpp`

**Interfaces:**
- Consumes: existing `readImu(ImuSample&)`, `CheckedAS5600`, `leftMotor`, `rightMotor`, `leftSensor`, `rightSensor`, motor direction signs, `MOTOR_VOLTAGE_LIMIT`, `stopMotors()`, and `fault()`.
- Produces: `BalanceCar::Wheel`, `BalanceCar::Mpu6050Data`, `BalanceCar::EncoderData`, `BalanceCar::readMpu6050()`, `BalanceCar::readEncoder()`, `BalanceCar::setMotorVoltages()`, and `BalanceCar::stopAllMotors()`.

- [ ] **Step 1: Extend the SimpleFOC fake and write failing public-interface assertions**

Add `shaft_angle` beside `shaft_velocity` in the fake:

```cpp
float target = 0, shaft_angle = 0, shaft_velocity = 0, zero_electric_angle = 0;
```

Add a fresh `healthySetup()`, arm the current controller with `command('A')`, and then add:

```cpp
healthySetup();
command('A');
assert(armed);

BalanceCar::Mpu6050Data imu{};
assert(BalanceCar::readMpu6050(imu));
assert(imu.rawAz == 16384);
assert(fabsf(imu.accelZG - 1.0f) < 0.0001f);
assert(fabsf(imu.pitchDeg - pitchAngleDeg) < 0.0001f);

leftMotor.shaft_angle = 1.25f;
leftMotor.shaft_velocity = 2.5f;
BalanceCar::EncoderData encoder{};
assert(BalanceCar::readEncoder(BalanceCar::Wheel::Left, encoder));
assert(fabsf(encoder.angleRad - LEFT_MOTOR_SIGN * 1.25f) < 0.0001f);
assert(fabsf(encoder.velocityRadPerSec - LEFT_MOTOR_SIGN * 2.5f) < 0.0001f);
assert(encoder.magnetHealthy);

assert(BalanceCar::setMotorVoltages(0.4f, 0.5f));
assert(fabsf(leftMotor.target - LEFT_MOTOR_SIGN * 0.4f) < 0.0001f);
assert(fabsf(rightMotor.target - RIGHT_MOTOR_SIGN * 0.5f) < 0.0001f);
BalanceCar::stopAllMotors();
assert(!driving() && !hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
```

- [ ] **Step 2: Run the host test and confirm the missing API failure**

Run:

```powershell
.\tests\balance_car_new\run_new.ps1
```

Expected: compilation fails because `BalanceCar` and its public types/functions do not exist.

- [ ] **Step 3: Create the public header**

Create `BalanceCarApi_new.h` with this complete contract:

```cpp
#pragma once
#include <stdint.h>

namespace BalanceCar {

enum class Wheel { Left, Right };

struct Mpu6050Data {
  int16_t rawAx = 0, rawAy = 0, rawAz = 0;
  int16_t rawGx = 0, rawGy = 0, rawGz = 0;
  float accelXG = 0, accelYG = 0, accelZG = 0;
  float gyroXDps = 0, gyroYDps = 0, gyroZDps = 0;
  float pitchDeg = 0, pitchRateDegPerSec = 0;
};

struct EncoderData {
  float angleRad = 0;
  float velocityRadPerSec = 0;
  bool magnetHealthy = false;
};

bool readMpu6050(Mpu6050Data& data);
bool readEncoder(Wheel wheel, EncoderData& data);
bool setMotorVoltages(float leftVoltage, float rightVoltage);
void stopAllMotors();

}  // namespace BalanceCar
```

- [ ] **Step 4: Implement the API against the existing private hardware objects**

Include `BalanceCarApi_new.h` from `BalanceController_new.h`. Immediately after the existing anonymous namespace closes, define the four API functions. Use `16384.0f` for ±2 g conversion and `GYRO_SCALE` for ±250°/s conversion. Apply `LEFT_MOTOR_SIGN`/`RIGHT_MOTOR_SIGN` to encoder angle, encoder velocity, and motor voltage so positive project values always mean chassis-forward.

`setMotorVoltages()` must:

```cpp
if (!motorsReady || faultLatched) return false;
if (!isfinite(leftVoltage) || !isfinite(rightVoltage)) {
  fault("invalid motor voltage request");
  return false;
}
leftMotor.loopFOC();
if (leftSensor.failed) {
  fault("LEFT AS5600 I2C read");
  return false;
}
rightMotor.loopFOC();
if (rightSensor.failed) {
  fault("RIGHT AS5600 I2C read");
  return false;
}
leftMotor.move(LEFT_MOTOR_SIGN * BalanceMath::clamp(
  leftVoltage, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT));
rightMotor.move(RIGHT_MOTOR_SIGN * BalanceMath::clamp(
  rightVoltage, -MOTOR_VOLTAGE_LIMIT, MOTOR_VOLTAGE_LIMIT));
return true;
```

`readEncoder()` must select the requested motor/sensor, call `magnetHealthy()`, fail through `fault("AS5600 magnet status / I2C")` on error, and return the sign-normalized cached `shaft_angle`/`shaft_velocity`. `stopAllMotors()` must delegate to `stopMotors("project API")`.

- [ ] **Step 5: Run the host test and confirm the public API passes**

Run:

```powershell
.\tests\balance_car_new\run_new.ps1
```

Expected: all existing regression assertions and the new public-interface assertions pass.

- [ ] **Step 6: Commit the public API slice**

```powershell
git add -- V1_06_TwoWheel_Balance_Car_new/BalanceCarApi_new.h V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h tests/balance_car_new/fakes/SimpleFOC.h tests/balance_car_new/balance_behavior_new_test.cpp
git commit -m "feat: add balance car hardware API"
```

### Task 2: Auto-start, live tuning, and rewritten stop state

**Files:**
- Modify: `V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h`
- Test: `tests/balance_car_new/balance_behavior_new_test.cpp`

**Interfaces:**
- Consumes: Task 1 `BalanceCar::setMotorVoltages(float, float)` and existing posture-control math.
- Produces: `startBalancing()`, automatic startup after calibration, live PD/setpoint tuning, and 60°/300 ms fall detection.

- [ ] **Step 1: Rewrite host expectations as failing behavior tests**

Change `healthySetup()` to expect automatic arming and enabled drivers:

```cpp
setup();
assert(motorsReady && imuReady && !faultLatched && armed);
assert(hostPins[LEFT_EN] == HIGH && hostPins[RIGHT_EN] == HIGH);
```

Update tests that require a stopped controller by sending `X` immediately after `healthySetup()`. In particular:

```cpp
healthySetup();
command('X');
hostTimeUs += 20000;
command('A');
assert(!armed);  // stale IMU data is still rejected.
assert(updatePitch(0.005f));
lastControlUs = micros();
command('A');
assert(armed);

healthySetup();
command('X');
command('L');
assert(testWheel == 1 && hostPins[LEFT_EN] && !hostPins[RIGHT_EN]);
```

Remove redundant `command('A')` calls from fault, timeout, overspeed-removal, and fall tests because `healthySetup()` now returns with balance already armed.

Add tests proving live tuning while armed:

```cpp
const float oldKp = angleKp, oldKd = angleKd, oldSetpoint = balanceSetpointDeg;
command('1'); command('3'); command('+');
assert(armed);
assert(angleKp == oldKp + 0.5f);
assert(fabsf(angleKd - (oldKd + 0.02f)) < 0.0001f);
assert(fabsf(BalanceMath::wrapDeg(balanceSetpointDeg - oldSetpoint) - 0.1f) < 0.0001f);
```

Replace the old overspeed-stop assertion with:

```cpp
healthySetup();
leftMotor.shaft_velocity = 40.0f;
loop();
assert(armed && !faultLatched);
```

Add a transient-versus-sustained fall test using MPU6050 values representing approximately 70° pitch:

```cpp
healthySetup();
wordAt(Wire, 0x68, 0x3D, 15396);
wordAt(Wire, 0x68, 0x3F, 5603);
pitchAngleDeg = 70.0f;
for (int i = 0; i < 40; ++i) { hostTimeUs += 5000; loop(); }
assert(armed);  // 200 ms is shorter than the 300 ms fall hold.
wordAt(Wire, 0x68, 0x3D, 0);
wordAt(Wire, 0x68, 0x3F, 16384);
pitchAngleDeg = 0;
hostTimeUs += 5000; loop();
assert(armed);  // Returning upright clears the pending fall.
wordAt(Wire, 0x68, 0x3D, 15396);
wordAt(Wire, 0x68, 0x3F, 5603);
pitchAngleDeg = 70.0f;
for (int i = 0; i < 62; ++i) { hostTimeUs += 5000; loop(); }
assert(!driving() && !faultLatched);
```

Retain explicit assertions for serial `X`, MPU6050/AS5600 failure, non-finite motor voltage, and control-loop overrun.

- [ ] **Step 2: Run the host test and confirm old behavior fails**

Run:

```powershell
.\tests\balance_car_new\run_new.ps1
```

Expected: failure because startup remains disarmed, tuning commands are ignored while driving, overspeed still stops, and fall debounce is absent.

- [ ] **Step 3: Replace chassis-dependent constants and add fall state**

Remove `ARM_TILT_DEG`, `MAX_TILT_DEG`, and `MAX_WHEEL_SPEED`. Add:

```cpp
constexpr float FALLEN_TILT_DEG = 60.0f;
constexpr uint32_t FALLEN_HOLD_MS = 300;
bool fallPending = false;
uint32_t fallStartMs = 0;
```

Reset `fallPending` and `fallStartMs` in `stopMotors()`.

Add this exact state transition:

```cpp
void updateFallState() {
  if (!armed) {
    fallPending = false;
    fallStartMs = 0;
    return;
  }
  const bool beyond = fabsf(BalanceMath::wrapDeg(
    pitchAngleDeg - calibratedUprightDeg)) > FALLEN_TILT_DEG;
  if (!beyond) {
    fallPending = false;
    fallStartMs = 0;
    return;
  }
  if (!fallPending) {
    fallPending = true;
    fallStartMs = millis();
    return;
  }
  if (millis() - fallStartMs >= FALLEN_HOLD_MS) stopMotors("fallen");
}
```

- [ ] **Step 4: Simplify arming and make startup automatic**

Keep initialization/fresh-data/magnet checks in `canStart()`, but remove angle, angular-rate, and wheel-speed thresholds. Add:

```cpp
bool startBalancing(const char* source) {
  if (!canStart()) return false;
  leftMotor.enable();
  rightMotor.enable();
  armed = true;
  fallPending = false;
  fallStartMs = 0;
  Serial.printf("BALANCE ARMED (%s); X stops.\n", source);
  return true;
}
```

After successful IMU calibration in `setup()`, call:

```cpp
if (!startBalancing("automatic startup")) {
  fault("automatic balance start");
  return;
}
```

Keep `A` as a manual call to `startBalancing("serial A")` after `X` or a non-fault fall stop.

- [ ] **Step 5: Allow safe live tuning and route the loop through the public motor API**

In `processSerial()`, handle `X`, `1`, `0`, `3`, `2`, `+`, `-`, and `?` before rejecting other commands while `driving()`. Continue to allow `L`, `R`, `C`, `V`, and `A` only while stopped.

Replace direct `leftMotor.loopFOC()`, `rightMotor.loopFOC()`, and `move()` calls in `loop()` with:

```cpp
const float leftVoltage = armed ? balanceVoltage : (testWheel == 1 ? TEST_VOLTAGE : 0);
const float rightVoltage = armed ? balanceVoltage : (testWheel == 2 ? TEST_VOLTAGE : 0);
if (!BalanceCar::setMotorVoltages(leftVoltage, rightVoltage)) {
  if (!faultLatched) fault("motor update");
  return;
}
```

Remove the wheel-overspeed stop. After each successful 5 ms `updatePitch(dt)`, call `updateFallState()` and only call `updateBalanceControl(dt)` if `armed` remains true.

- [ ] **Step 6: Run the complete host regression**

Run:

```powershell
.\tests\balance_car_new\run_new.ps1
```

Expected final line begins with `PASS:` and reports auto-start, live tuning, fall debounce, interfaces, and retained hard faults.

- [ ] **Step 7: Commit the controller-state slice**

```powershell
git add -- V1_06_TwoWheel_Balance_Car_new/BalanceController_new.h tests/balance_car_new/balance_behavior_new_test.cpp
git commit -m "feat: auto-start balance commissioning mode"
```

### Task 3: ESP32-WROOM default build and operator documentation

**Files:**
- Modify: `platformio.ini`
- Modify: `README_balance_new.md`
- Modify: `README_new.md`

**Interfaces:**
- Consumes: Task 1 public API and Task 2 startup/stop behavior.
- Produces: a default `balance_car_new` PlatformIO build and accurate compile/upload/run instructions.

- [ ] **Step 1: Write a failing configuration check**

Run:

```powershell
$default = Select-String -LiteralPath platformio.ini -Pattern '^default_envs\s*=\s*balance_car_new$'
if (-not $default) { throw 'balance_car_new is not the default environment' }
```

Expected: throws because the current default is `sensor_test`.

- [ ] **Step 2: Make the balance firmware the default without changing the board target**

Change only this setting:

```ini
[platformio]
default_envs = balance_car_new
```

Retain `board = esp32dev`, the short `${sysenv.USERPROFILE}/.pio/...` build paths, SimpleFOC 2.3.3, and the `balance_car_new` platform pin `espressif32@7.1.3`.

- [ ] **Step 3: Update both operator documents**

In `README_balance_new.md`, document:

- `BalanceController_new.h` is the main logic file; `.ino` and `src/balance_car_new.cpp` are thin Arduino/PlatformIO entries.
- `BalanceCarApi_new.h` public types and functions, with units and the requirement to call `setMotorVoltages()` continuously.
- startup now automatically arms immediately after successful calibration;
- `A` is for rearming after `X` or a detected fall, not required on every boot;
- Kp/Kd/setpoint commands work during balance;
- the 60°/300 ms fall rule and the exact retained hard faults;
- `board = esp32dev` targets common ESP32-WROOM-32 DevKit boards;
- current-workspace PowerShell commands:

```powershell
Set-Location 'C:\Users\29316\Desktop\wheeled_legged_robot'
pio device list
pio run
pio run -t upload --upload-port COM5
pio device monitor --port COM5 -b 115200
```

State that `COM5` is only an example and must be replaced with the detected port. In `README_new.md`, change the default-environment description and balance startup summary to match.

- [ ] **Step 4: Verify configuration and stale-document text**

Run:

```powershell
$default = Select-String -LiteralPath platformio.ini -Pattern '^default_envs\s*=\s*balance_car_new$'
if (-not $default) { throw 'default environment mismatch' }
rg -n "默认环境仍是旧版|等待串口命令|仍需显式选择.*balance_car_new" README_new.md README_balance_new.md
```

Expected: the default check passes and `rg` returns no stale behavior statements.

- [ ] **Step 5: Commit configuration and documentation**

```powershell
git add -- platformio.ini README_balance_new.md README_new.md
git commit -m "docs: make balance firmware the default"
```

### Task 4: Final host and firmware verification

**Files:**
- Modify only if verification exposes a defect directly caused by Tasks 1-3.
- Verify: all files changed in Tasks 1-3.

**Interfaces:**
- Consumes: completed public API, controller state machine, configuration, and documentation.
- Produces: reproducible host-test and ESP32 build evidence.

- [ ] **Step 1: Run formatting and diff checks**

Run:

```powershell
git diff --check HEAD~3
git status --short
```

Expected: no whitespace errors; only intended changes are present.

- [ ] **Step 2: Run the host controller regression from a clean invocation**

Run:

```powershell
.\tests\balance_car_new\run_new.ps1
```

Expected: exit code 0 and a `PASS:` summary covering initialization, auto-start, public interfaces, tuning, fall debounce, sensor faults, timing, and control math.

- [ ] **Step 3: Resolve the local PlatformIO executable and compile the explicit environment**

Run:

```powershell
$pioCommand = Get-Command pio -ErrorAction SilentlyContinue
$pioExe = if ($pioCommand) { $pioCommand.Source } else { Join-Path $env:USERPROFILE '.platformio\penv\Scripts\pio.exe' }
if (-not (Test-Path -LiteralPath $pioExe)) { throw "PlatformIO not found: $pioExe" }
& $pioExe run -e balance_car_new
if ($LASTEXITCODE -ne 0) { throw 'Explicit balance_car_new build failed' }
```

Expected: `SUCCESS` for `balance_car_new`.

- [ ] **Step 4: Compile through the new default environment**

Run:

```powershell
& $pioExe run
if ($LASTEXITCODE -ne 0) { throw 'Default PlatformIO build failed' }
```

Expected: `SUCCESS`, with `balance_car_new` selected as the default environment.

- [ ] **Step 5: Inspect the final diff and report physical-test boundaries**

Run:

```powershell
git diff HEAD~3 --stat
git log -4 --oneline --decorate
git status --short
```

Expected: three implementation commits after the design/plan checkpoint and a clean worktree. Report explicitly that no upload or real-wheel actuation was performed, and that first power-on requires the chassis supported with wheels clear during FOC alignment and human restraint during auto-balance tuning.
