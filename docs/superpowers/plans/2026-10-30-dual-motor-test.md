# Dual-Motor Test Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an independent ESP32 firmware target that validates both AS5600 encoders, aligns both motors, and drives both wheels forward at 0.5 V until serial `X` stops them.

**Architecture:** Keep the new test isolated from the balance controller. The Arduino sketch owns the complete two-bus AS5600 diagnostics, SimpleFOC motor setup, latched stop/fault behavior, and telemetry; a one-line PlatformIO wrapper includes that sketch, matching the repository's existing `_new` targets.

**Tech Stack:** C++17 host tests, Arduino-ESP32 2.0.17, PlatformIO `espressif32@7.1.3`, SimpleFOC 2.3.3, PowerShell test runner.

## Global Constraints

- Do not modify the behavior or default selection of `balance_car_new` or `sensor_test`.
- Do not initialize, read, calibrate, or depend on an MPU6050.
- Use left PWM/EN/I2C pins `32/33/25`, `12`, `21/22`; use right pins `14/27/26`, `13`, `19/18`.
- Use 7 pole pairs, 12 V supply, 6 V driver limit, 3 V motor limit, 10 kHz PWM, and fixed 0.5 V test command.
- Vehicle-forward command signs are left `+1` and right `-1`.
- `X`/`x` and all defined faults latch both motors off until reset.
- Build and test only; do not upload or operate physical hardware.

---

## File Structure

- Create `V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino`: complete Arduino firmware and testable control state.
- Create `src/dual_motor_test_new.cpp`: PlatformIO entry that includes the sketch.
- Create `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`: host scenarios against the actual sketch.
- Create `tests/dual_motor_test_new/run_new.ps1`: compile once and execute each isolated host scenario.
- Modify `platformio.ini`: add only the `dual_motor_test_new` environment.
- Modify `README_new.md`: list the target, commands, startup sequence, stop behavior, and safety boundary.

### Task 1: Add failing host behavior tests

**Files:**
- Create: `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`
- Create: `tests/dual_motor_test_new/run_new.ps1`
- Test: `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`

**Interfaces:**
- Consumes: fake `Arduino.h`, `Wire.h`, and `SimpleFOC.h` from `tests/balance_car_new/fakes`.
- Produces: scenario contract for firmware globals `running`, `faultLatched`, `leftMotor`, `rightMotor`, constants `LEFT_EN`, `RIGHT_EN`, `TEST_VOLTAGE`, and functions `setup()` and `loop()`.

- [ ] **Step 1: Write the scenario-based failing test**

Create one test executable that includes the actual sketch and selects exactly one fresh-process scenario from `argv[1]`:

```cpp
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include "../../V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino"

void healthyEncoders() {
  Wire.registers[0x36][0x0B] = 0x20;
  Wire1.registers[0x36][0x0B] = 0x20;
}

int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string scenario = argv[1];
  healthyEncoders();

  if (scenario == "healthy") {
    setup();
    assert(running && !faultLatched);
    assert(hostPins[LEFT_EN] == HIGH && hostPins[RIGHT_EN] == HIGH);
    assert(std::fabs(leftMotor.voltage.q - TEST_VOLTAGE) < 0.0001f);
    assert(std::fabs(rightMotor.voltage.q + TEST_VOLTAGE) < 0.0001f);
  } else if (scenario == "encoder-failure") {
    Wire.nackAddress = 0x36;
    setup();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "magnet-failure") {
    Wire1.registers[0x36][0x0B] = 0x00;
    setup();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "serial-stop") {
    setup();
    Serial.input.push_back('X');
    loop();
    assert(!running && !leftMotor.enabled && !rightMotor.enabled);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "runtime-encoder-failure") {
    setup();
    Wire.shortReadAddress = 0x36;
    loop();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else if (scenario == "overspeed") {
    setup();
    leftMotor.shaft_velocity = MAX_WHEEL_SPEED + 1.0f;
    loop();
    assert(faultLatched && !running);
    assert(hostPins[LEFT_EN] == LOW && hostPins[RIGHT_EN] == LOW);
  } else {
    return 2;
  }
  std::cout << "PASS: " << scenario << '\n';
}
```

- [ ] **Step 2: Write the PowerShell runner**

Resolve `g++`, prepend its own directory to process `PATH` so the matching MinGW runtime DLLs load, compile with warnings as errors, and run all six scenarios:

```powershell
param([string]$Compiler = 'g++')
$ErrorActionPreference = 'Stop'
$compilerPath = (Get-Command $Compiler -ErrorAction Stop).Source
$env:PATH = (Split-Path -Parent $compilerPath) + ';' + $env:PATH
$outputFolder = Join-Path $env:USERPROFILE '.pio\esp32-dual-motor-new\host-tests'
New-Item -ItemType Directory -Force $outputFolder | Out-Null
$binary = Join-Path $outputFolder 'dual_motor_test_new_test.exe'
$fakeIncludes = Join-Path $PSScriptRoot '..\balance_car_new\fakes'
& $compilerPath '-std=c++17' '-Wall' '-Wextra' '-Werror' '-I' $fakeIncludes `
  (Join-Path $PSScriptRoot 'dual_motor_test_new_test.cpp') '-o' $binary
if ($LASTEXITCODE -ne 0) { throw 'Host test compilation failed' }
foreach ($scenario in @('healthy','encoder-failure','magnet-failure','serial-stop',
                         'runtime-encoder-failure','overspeed')) {
  & $binary $scenario
  if ($LASTEXITCODE -ne 0) { throw "Host test failed: $scenario" }
}
```

- [ ] **Step 3: Run the test to verify the red state**

Run: `& '.\tests\dual_motor_test_new\run_new.ps1'`

Expected: compilation fails because `V1_07_DualMotor_Test_new.ino` does not exist.

- [ ] **Step 4: Commit the failing test**

```powershell
git add -- tests/dual_motor_test_new
git commit -m "test: specify dual motor test behavior"
```

### Task 2: Implement the isolated firmware and PlatformIO target

**Files:**
- Create: `V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino`
- Create: `src/dual_motor_test_new.cpp`
- Modify: `platformio.ini`
- Test: `tests/dual_motor_test_new/dual_motor_test_new_test.cpp`

**Interfaces:**
- Consumes: Arduino `Serial`, `Wire`, `Wire1`; SimpleFOC `Sensor`, `BLDCMotor`, and `BLDCDriver3PWM`.
- Produces: `setup()` and `loop()` firmware entry points; `dual_motor_test_new` PlatformIO environment; host-visible state defined in Task 1.

- [ ] **Step 1: Implement constants, checked encoders, and fail-safe state**

In the sketch, define the exact pins and limits from Global Constraints. Implement `readRegisters()`, `CheckedAS5600::magnetHealthy()`, and `CheckedAS5600::getSensorAngle()` using the same short-read and magnet-status checks as the balance firmware. Implement these state transitions:

```cpp
bool leftLinked = false, rightLinked = false;
bool running = false, faultLatched = false;

void stopMotors(const char* reason) {
  const bool wasRunning = running;
  digitalWrite(LEFT_EN, LOW);
  digitalWrite(RIGHT_EN, LOW);
  if (leftLinked) { leftMotor.disable(); leftMotor.target = 0; leftMotor.voltage.q = leftMotor.voltage.d = 0; }
  if (rightLinked) { rightMotor.disable(); rightMotor.target = 0; rightMotor.voltage.q = rightMotor.voltage.d = 0; }
  running = false;
  if (wasRunning) Serial.printf("STOP: %s\n", reason);
}

void fault(const char* reason) {
  stopMotors(reason);
  if (!faultLatched) Serial.printf("FAULT: %s. Reset after inspection.\n", reason);
  faultLatched = true;
}
```

- [ ] **Step 2: Implement initialization and automatic drive**

`setup()` must disable EN first, initialize both hardware I2C buses, evaluate both magnet checks before motor initialization, initialize both sensors, align the left then right motor, and only then enable both motors:

```cpp
if (!checkEncoderMagnets(true)) return;
leftSensor.init();
rightSensor.init();
if (leftSensor.failed || rightSensor.failed) { fault("AS5600 angle initialization"); return; }
if (!initializeMotor(leftMotor, leftDriver, leftSensor, leftLinked, "LEFT") ||
    !initializeMotor(rightMotor, rightDriver, rightSensor, rightLinked, "RIGHT")) {
  fault("motor alignment; check phases, magnets, supply and pole pairs");
  return;
}
leftMotor.enable();
rightMotor.enable();
running = true;
leftMotor.move(LEFT_MOTOR_SIGN * TEST_VOLTAGE);
rightMotor.move(RIGHT_MOTOR_SIGN * TEST_VOLTAGE);
Serial.println("BOTH MOTORS RUNNING FORWARD AT 0.5 V; X stops until reset.");
```

- [ ] **Step 3: Implement the continuous runtime checks**

`loop()` must check serial first, remain idle after a stop/fault, run both FOC updates, reject encoder errors and non-finite/overspeed values, refresh both 0.5 V commands, check magnet state every 100 ms, and print telemetry every 200 ms. `processSerial()` accepts only whitespace and `X`/`x`; after `X`, no command can restart the motors.

- [ ] **Step 4: Add the PlatformIO wrapper and environment**

Create the wrapper:

```cpp
#include <Arduino.h>
#include "../V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino"
```

Add an environment inheriting `common`, using the same pinned ESP32 platform as the balance target, no `send_on_enter` monitor filter, and selecting only the wrapper:

```ini
[env:dual_motor_test_new]
extends = common
platform = espressif32@7.1.3
monitor_filters =
build_src_filter =
    -<*>
    +<dual_motor_test_new.cpp>
```

- [ ] **Step 5: Run the host test to verify green**

Run: `& '.\tests\dual_motor_test_new\run_new.ps1'`

Expected: six `PASS:` lines and exit code 0.

- [ ] **Step 6: Run a clean PlatformIO build**

Run:

```powershell
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
& $pio run -e dual_motor_test_new -t clean
& $pio run -e dual_motor_test_new
```

Expected: `dual_motor_test_new SUCCESS` with a generated `firmware.bin`.

- [ ] **Step 7: Commit the firmware**

```powershell
git add -- V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino `
  src/dual_motor_test_new.cpp platformio.ini
git commit -m "feat: add dual motor test firmware"
```

### Task 3: Document and regress the delivered target

**Files:**
- Modify: `README_new.md`
- Test: `tests/dual_motor_test_new/run_new.ps1`

**Interfaces:**
- Consumes: PlatformIO environment `dual_motor_test_new` from Task 2.
- Produces: exact operator commands and safety instructions for the new target.

- [ ] **Step 1: Add the README target and commands**

Add the table row and commands:

```markdown
| 双电机同时转动测试（双 AS5600，无 MPU6050） | `V1_07_DualMotor_Test_new/V1_07_DualMotor_Test_new.ino` | `dual_motor_test_new` |

pio run -e dual_motor_test_new
pio run -e dual_motor_test_new -t upload
pio device monitor -e dual_motor_test_new -b 115200
```

Add a focused section stating: wheels must be supported off the ground; both encoder magnet checks run before FOC alignment; alignment itself moves each wheel; after both align, both run at 0.5 V in vehicle-forward direction; `X` disables both until reset; any encoder/magnet/overspeed fault also latches off; the program does not read MPU6050; 0.5 V is not a current limit.

- [ ] **Step 2: Run source-boundary checks**

Run:

```powershell
rg -n "MPU_I2C|SoftwareI2C|initializeImu|readImu|calibrateImu" `
  V1_07_DualMotor_Test_new src/dual_motor_test_new.cpp
git diff --check
```

Expected: `rg` returns no matches; `git diff --check` returns no errors.

- [ ] **Step 3: Run complete verification**

Run:

```powershell
& '.\tests\dual_motor_test_new\run_new.ps1'
$pio = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"
& $pio run -e dual_motor_test_new
& $pio run -e balance_car_new
```

Expected: all six host scenarios pass and both PlatformIO environments report `SUCCESS`.

- [ ] **Step 4: Commit documentation and test runner**

```powershell
git add -- README_new.md tests/dual_motor_test_new
git commit -m "docs: add dual motor test instructions"
```

- [ ] **Step 5: Inspect the final change set**

Run: `git status --short --branch; git log -4 --oneline; git diff HEAD~3 --stat`

Expected: clean worktree; commits limited to the design, plan, tests, firmware target, configuration, and documentation described above.
