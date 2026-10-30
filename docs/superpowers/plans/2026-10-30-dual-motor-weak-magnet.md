# Dual-Motor Weak-Magnet Bypass Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `dual_motor_test_new` continue with an explicit warning when an AS5600 reports a weak magnet, while preserving all other encoder and motor shutdowns.

**Architecture:** Change only the checked-AS5600 policy inside the standalone dual-motor sketch. Treat `ML=1, MH=0` as an accepted warning state, derive the warning from the raw status, and keep I2C, no-magnet, strong-magnet, invalid-status, angle-read, speed, and stop behavior unchanged.

**Tech Stack:** C++17 host tests, Arduino-ESP32 2.0.17, PlatformIO `espressif32@7.1.3`, SimpleFOC 2.3.3, PowerShell.

## Global Constraints

- Modify only `dual_motor_test_new`, its host scenarios, and its README section.
- Do not modify `BalanceController_new.h` or the `balance_car_new` environment.
- Accept raw AS5600 status `0x13` because it has `ML=1` and `MH=0`, even though `MD=0`.
- Keep `MH=1`, no `MD/ML/MH` bits, simultaneous `ML/MH`, I2C errors, and short reads as latched faults.
- Print `WARNING: MAGNET_TOO_WEAK; continuing test` once when weak status becomes active, without 10 Hz warning spam.
- Do not upload or operate physical hardware.

---

### Task 1: Specify weak-magnet behavior with host scenarios

**Files:**
- Modify: `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`
- Modify: `tests/dual_motor_test_new/run_new.ps1`

**Interfaces:**
- Consumes: actual sketch globals and faked `Wire`, `Wire1`, `Serial`, and motor drivers.
- Produces: scenarios `weak-magnet`, `runtime-weak-magnet`, and `strong-magnet`.

- [ ] **Step 1: Add startup and runtime weak-magnet scenarios**

```cpp
} else if (scenario == "weak-magnet") {
  Wire1.registers[0x36][0x0B] = 0x13;
  setup();
  assert(running && !faultLatched);
  assert(Serial.output.find("WARNING: MAGNET_TOO_WEAK; continuing test") != std::string::npos);
} else if (scenario == "runtime-weak-magnet") {
  setup();
  Serial.output.clear();
  Wire1.registers[0x36][0x0B] = 0x13;
  hostTimeUs += 100000;
  loop();
  assert(running && !faultLatched);
  assert(Serial.output.find("WARNING: MAGNET_TOO_WEAK; continuing test") != std::string::npos);
```

- [ ] **Step 2: Add the strong-magnet rejection scenario**

```cpp
} else if (scenario == "strong-magnet") {
  Wire1.registers[0x36][0x0B] = 0x28;
  setup();
  assert(faultLatched && !running);
  assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
```

Add all three names to the PowerShell scenario array.

- [ ] **Step 3: Run the red test**

Run: `& '.\tests\dual_motor_test_new\run_new.ps1'`

Expected: `weak-magnet` fails because current firmware latches `MAGNET_TOO_WEAK` and does not run.

### Task 2: Implement the narrow warning policy

**Files:**
- Modify: `V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino`
- Test: `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`

**Interfaces:**
- Consumes: AS5600 status bits `MD=0x20`, `ML=0x10`, `MH=0x08`.
- Produces: `CheckedAS5600::weakMagnet` and `weakWarningPending`, read by `checkEncoderMagnets()` and diagnostic output.

- [ ] **Step 1: Accept only weak-without-strong status**

In `magnetHealthy()`, after a successful status read:

```cpp
const uint8_t flags = magnetStatus & 0x38;
const bool nowWeak = (flags & 0x10) != 0 && (flags & 0x08) == 0;
weakWarningPending = nowWeak && !weakMagnet;
weakMagnet = nowWeak;
if (nowWeak) {
  failure = Failure::none;
  return true;
}
if (flags != 0x20) return latchFailure(Failure::magnet);
failure = Failure::none;
return true;
```

Add public booleans initialized false:

```cpp
bool weakMagnet = false;
bool weakWarningPending = false;
```

- [ ] **Step 2: Print the warning once per transition**

In the encoder diagnostic, select the warning label when `weakMagnet` is true:

```cpp
if (sensor.weakMagnet)
  reason = "WARNING: MAGNET_TOO_WEAK; continuing test";
else if (sensor.failure == CheckedAS5600::Failure::magnet)
  // retain existing fault reason selection
```

In `checkEncoderMagnets()`, print each side when `logHealthy` or its warning is pending, then clear both pending flags after printing.

- [ ] **Step 3: Run all host tests**

Run:

```powershell
& '.\tests\dual_motor_test_new\run_new.ps1'
$env:PATH = 'C:\C++_Env\mingw64\bin;' + $env:PATH
& '.\tests\balance_car_new\run_new.ps1'
```

Expected: all dual-motor scenarios and both balance test binaries pass.

### Task 3: Document and compile the delivered policy

**Files:**
- Modify: `README_new.md`

**Interfaces:**
- Consumes: weak-magnet warning behavior from Task 2.
- Produces: operator-visible warning and risk boundary.

- [ ] **Step 1: Update the dual-motor test warning**

Add a bullet stating that this test continues on `MAGNET_TOO_WEAK`, but angle feedback may be noisy or unreliable; it still stops for I2C failure, missing magnet, too-strong magnet, and invalid status.

- [ ] **Step 2: Run source and build verification**

Run:

```powershell
git diff --check
& '.\tests\dual_motor_test_new\run_new.ps1'
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
& $pio run -e dual_motor_test_new
& $pio run -e balance_car_new
```

Expected: no whitespace errors, every host scenario passes, and both PlatformIO environments report `SUCCESS`.

- [ ] **Step 3: Commit the implementation**

```powershell
git add -- V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino `
  tests/dual_motor_test_new/dual_motor_test_new_test.cpp `
  tests/dual_motor_test_new/run_new.ps1 README_new.md
git commit -m "feat: allow weak magnet in dual motor test"
```
