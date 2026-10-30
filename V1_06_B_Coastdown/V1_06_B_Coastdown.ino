/*
  Homework 4.3: estimate viscous damping from an unpowered coast-down.

  Hardware: ESP32 Dev Module + AS5600 + SimpleFOC Mini + 2804 BLDC.
  AS5600: SDA=21, SCL=22. Driver: IN1=32, IN2=33, IN3=25, EN=12.

  Send S to run three trials. Each trial applies a low fixed q-axis voltage,
  spins up for a few seconds, disables the driver, then logs the free coast.
  Send X to abort. The wheel/load must be removed to measure the bare motor.

  CSV fields: run,phase,t_us,uq_cmd_V,angle_rad,omega_rad_s,dt_us
  phase=0: powered spin-up; phase=1: driver disabled and coasting.
  t_us resets at the start of each phase. During coasting, fit ln(|omega|)
  against t; its negative slope is B/J for a viscous-friction model.
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr int POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE = 12.0f;  // Set to the measured DC bus voltage.
constexpr float DRIVER_VOLTAGE_LIMIT = 3.0f;
constexpr float MOTOR_VOLTAGE_LIMIT = 3.0f;
constexpr float ALIGN_VOLTAGE = 1.0f;
constexpr float SPINUP_UQ_V = 2.4f;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t SAMPLE_INTERVAL_US = 10000;
constexpr uint32_t SPINUP_DURATION_US = 3000000;
constexpr uint32_t MIN_COAST_DURATION_US = 1500000;
constexpr uint32_t MAX_COAST_DURATION_US = 30000000;
constexpr uint32_t STOP_HOLD_US = 500000;
constexpr uint32_t BETWEEN_RUNS_US = 1000000;
constexpr float STOP_SPEED_RAD_S = 15.0f;
constexpr size_t RUN_COUNT = 3;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

enum class TestState : uint8_t {
  waiting,
  spinup,
  coast,
  pause,
  done,
  aborted
};

TestState state = TestState::waiting;
bool motorReady = false;
uint8_t runNumber = 0;
uint32_t phaseStartUs = 0;
uint32_t nextSampleUs = 0;
uint32_t previousSampleUs = 0;
uint32_t belowStopSpeedSinceUs = 0;
uint32_t pauseStartUs = 0;
uint32_t droppedSamples = 0;

void startRun(uint32_t nowUs) {
  runNumber++;
  motor.enable();
  motor.target = SPINUP_UQ_V;
  state = TestState::spinup;
  phaseStartUs = nowUs;
  nextSampleUs = nowUs;
  previousSampleUs = nowUs;
  belowStopSpeedSinceUs = 0;
  Serial.printf("# RUN_START,%u\n", runNumber);
}

void beginCoast(uint32_t nowUs) {
  motor.disable();
  motor.target = 0.0f;
  state = TestState::coast;
  phaseStartUs = nowUs;
  nextSampleUs = nowUs;
  previousSampleUs = nowUs;
  belowStopSpeedSinceUs = 0;
  Serial.printf("# COAST_BEGIN,%u\n", runNumber);
}

void finishCoast(uint32_t nowUs, bool timedOut) {
  motor.disable();
  state = TestState::pause;
  pauseStartUs = nowUs;
  Serial.printf("# COAST_END,%u,%s\n", runNumber,
                timedOut ? "timeout" : "low_speed");
}

void abortTest() {
  motor.disable();
  state = TestState::aborted;
  Serial.printf("# ABORTED,dropped_samples=%lu\n",
                static_cast<unsigned long>(droppedSamples));
}

void startTest() {
  if (!motorReady || state != TestState::waiting) return;
  runNumber = 0;
  droppedSamples = 0;
  Serial.println("# START");
  Serial.println("# Remove the wheel/load; keep motor fixed and shaft clear.");
  Serial.println("# CSV: run,phase,t_us,uq_cmd_V,angle_rad,omega_rad_s,dt_us");
  Serial.println("run,phase,t_us,uq_cmd_V,angle_rad,omega_rad_s,dt_us");
  startRun(micros());
}

void logSample(uint32_t nowUs, uint8_t phase, float uqCommand) {
  if (static_cast<int32_t>(nowUs - nextSampleUs) < 0) return;

  const uint32_t lateness = nowUs - nextSampleUs;
  if (lateness >= SAMPLE_INTERVAL_US) {
    const uint32_t missed = lateness / SAMPLE_INTERVAL_US;
    droppedSamples += missed;
    nextSampleUs += missed * SAMPLE_INTERVAL_US;
  }
  nextSampleUs += SAMPLE_INTERVAL_US;

  const uint32_t dtUs = nowUs - previousSampleUs;
  previousSampleUs = nowUs;
  const uint32_t tUs = nowUs - phaseStartUs;
  char line[144];
  const int length = snprintf(
      line, sizeof(line), "%u,%u,%lu,%.4f,%.6f,%.6f,%lu\n",
      static_cast<unsigned int>(runNumber),
      static_cast<unsigned int>(phase),
      static_cast<unsigned long>(tUs), uqCommand, motor.shaft_angle,
      motor.shaft_velocity, static_cast<unsigned long>(dtUs));

  if (length <= 0 || length >= static_cast<int>(sizeof(line)) ||
      Serial.availableForWrite() < length) {
    droppedSamples++;
    return;
  }
  Serial.write(reinterpret_cast<const uint8_t*>(line), length);
}

void setup() {
  Serial.setTxBufferSize(8192);
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println("V1_06_B_Coastdown");
  Serial.println("# Motor alignment during initFOC can move the rotor.");
  Serial.printf("# Udc=%.2f V, spin-up Uq=%.2f V, samples every %.1f ms.\n",
                SUPPLY_VOLTAGE, SPINUP_UQ_V,
                SAMPLE_INTERVAL_US / 1000.0f);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(0x36);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not found at 0x36. Check wiring and reset.");
    return;
  }
  sensor.init(&Wire);
  sensor.update();
  if (sensor.currWireError != 0) {
    Serial.println("AS5600 I2C read failed. Check wiring and reset.");
    return;
  }
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) {
    Serial.println("Driver init failed. Check GPIO configuration and reset.");
    return;
  }
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.voltage_sensor_align = ALIGN_VOLTAGE;
  motor.LPF_velocity.Tf = 0.01f;
  motor.target = 0.0f;

  motor.init();
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
    motor.disable();
    Serial.println("Motor init failed. Check configuration and reset.");
    return;
  }
  Serial.println("# Running initFOC; rotor alignment can move the motor.");
  if (!motor.initFOC()) {
    motor.disable();
    Serial.println("FOC init failed. Check phase wiring, sensor and pole pairs.");
    return;
  }

  motor.disable();
  motorReady = true;
  Serial.println("# READY at 921600 baud. Open log capture, then send S.");
  Serial.println("# Three runs; send X to abort. Driver is disabled between runs.");
}

void loop() {
  if (!motorReady) {
    delay(1);
    return;
  }

  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    if ((input == 'S' || input == 's') && state == TestState::waiting) {
      startTest();
    } else if ((input == 'X' || input == 'x') &&
               (state == TestState::spinup || state == TestState::coast)) {
      abortTest();
      return;
    }
  }

  uint32_t nowUs = micros();
  if (state == TestState::spinup) {
    motor.loopFOC();
    motor.move(SPINUP_UQ_V);
    nowUs = micros();
    logSample(nowUs, 0, SPINUP_UQ_V);
    if (nowUs - phaseStartUs >= SPINUP_DURATION_US) beginCoast(nowUs);
    if (sensor.currWireError != 0) {
      Serial.println("# AS5600_I2C_ERROR");
      abortTest();
      return;
    }
  } else if (state == TestState::coast) {
    motor.loopFOC();
    motor.move(0.0f);  // Updates sensor velocity; disabled driver stays off.
    nowUs = micros();
    logSample(nowUs, 1, 0.0f);
    if (sensor.currWireError != 0) {
      Serial.println("# AS5600_I2C_ERROR");
      abortTest();
      return;
    }

    const uint32_t coastElapsed = nowUs - phaseStartUs;
    if (coastElapsed >= MIN_COAST_DURATION_US &&
        fabsf(motor.shaft_velocity) <= STOP_SPEED_RAD_S) {
      if (belowStopSpeedSinceUs == 0) belowStopSpeedSinceUs = nowUs;
      if (nowUs - belowStopSpeedSinceUs >= STOP_HOLD_US) {
        finishCoast(nowUs, false);
      }
    } else {
      belowStopSpeedSinceUs = 0;
    }
    if (state == TestState::coast && coastElapsed >= MAX_COAST_DURATION_US) {
      finishCoast(nowUs, true);
    }
  } else if (state == TestState::pause) {
    if (nowUs - pauseStartUs >= BETWEEN_RUNS_US) {
      if (runNumber < RUN_COUNT) {
        startRun(nowUs);
      } else {
        state = TestState::done;
        Serial.printf("# DONE,dropped_samples=%lu\n",
                      static_cast<unsigned long>(droppedSamples));
        Serial.println("# Reset the board to run another set.");
      }
    }
  } else {
    delay(1);
  }
}
