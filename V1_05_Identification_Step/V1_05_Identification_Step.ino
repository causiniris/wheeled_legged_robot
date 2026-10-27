/*
  作业 4.2/4.3：2804 无刷电机 q 轴电压阶跃辨识数据采集。

  硬件：ESP32 Dev Module + AS5600 + SimpleFOC Mini v1.0 + 2804 BLDC。
  AS5600：SDA=21，SCL=22；驱动：IN1=32，IN2=33，IN3=25，EN=12。

  与直流有刷电机模板的区别：本程序不闭合机械速度环，而是在传感器定向的
  FOC 电压转矩模式下直接给 q 轴电压 Uq，采集 Uq -> omega 的开环机械响应。
  速度 PID 不能用于辨识，否则得到的是“闭环控制器+电机”的模型。

  串口：921600 baud。复位并完成 initFOC 后，先打开串口日志保存，再发送 S。
  程序自动执行 D=0.1/0.2/0.4/0.6 四档；Ua=D*Udc。
  每档包含基线、阶跃和恢复段。
  发送 X 可中止。完成后复位才能重跑。

  CSV：t_us,run,phase,duty_D,Udc_V,Ua_V,omega_rad_s,dt_us
  phase: 0=阶跃前基线，1=阶跃，2=回零恢复。
  为适配作业表格，Ua_V 表示 BLDC 的等效电枢电压，即 FOC q 轴命令电压；
  duty_D=Ua_V/Udc_V，是等效命令占空比，不是 SVPWM 某一相的瞬时占空比。
  Udc_V 来自 SUPPLY_VOLTAGE，实验前必须改成万用表实测母线电压。
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr int POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE = 12.0f;       // 运行前改成实测母线电压
constexpr float MAX_EXPERIMENT_DUTY = 0.6f;
constexpr float DRIVER_VOLTAGE_LIMIT = SUPPLY_VOLTAGE * MAX_EXPERIMENT_DUTY;
constexpr float MOTOR_VOLTAGE_LIMIT = SUPPLY_VOLTAGE * MAX_EXPERIMENT_DUTY;
constexpr float ALIGN_VOLTAGE = 1.0f;
constexpr long PWM_FREQUENCY_HZ = 10000;
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t SAMPLE_INTERVAL_US = 1000;
constexpr uint32_t BASELINE_US = 1000000;
constexpr uint32_t STEP_US = 4000000;
constexpr uint32_t RECOVERY_US = 1500000;
constexpr size_t STEP_COUNT = 4;
constexpr float DUTY_LEVELS_D[STEP_COUNT] = {0.1f, 0.2f, 0.4f, 0.6f};

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

enum class ExperimentPhase : uint8_t {
  idle = 255,
  baseline = 0,
  step = 1,
  recovery = 2,
  done = 3,
  aborted = 4
};

ExperimentPhase phase = ExperimentPhase::idle;
bool motorReady = false;
bool experimentRunning = false;
size_t runIndex = 0;
float dutyCommand = 0.0f;
float uqCommand = 0.0f;
uint32_t experimentStartUs = 0;
uint32_t phaseStartUs = 0;
uint32_t nextSampleUs = 0;
uint32_t previousSampleUs = 0;
uint32_t droppedSamples = 0;

void setPhase(ExperimentPhase next, uint32_t nowUs) {
  phase = next;
  phaseStartUs = nowUs;
  dutyCommand =
      (phase == ExperimentPhase::step) ? DUTY_LEVELS_D[runIndex] : 0.0f;
  uqCommand = dutyCommand * SUPPLY_VOLTAGE;
}

void stopExperiment(bool aborted) {
  experimentRunning = false;
  dutyCommand = 0.0f;
  uqCommand = 0.0f;
  motor.target = 0.0f;
  motor.move();
  motor.loopFOC();
  motor.disable();
  phase = aborted ? ExperimentPhase::aborted : ExperimentPhase::done;
  Serial.printf("# %s,dropped_samples=%lu\n",
                aborted ? "ABORTED" : "DONE",
                static_cast<unsigned long>(droppedSamples));
  Serial.println("# Any dropped sample or dt_us > 1200 means the run must be repeated.");
}

void startExperiment() {
  if (!motorReady || experimentRunning || phase == ExperimentPhase::done ||
      phase == ExperimentPhase::aborted) return;

  runIndex = 0;
  droppedSamples = 0;
  const uint32_t nowUs = micros();
  experimentStartUs = nowUs;
  nextSampleUs = nowUs;
  previousSampleUs = nowUs;
  setPhase(ExperimentPhase::baseline, nowUs);
  experimentRunning = true;

  Serial.println("# START");
  Serial.println("# D=Ua/Udc; Ua is the BLDC q-axis command voltage.");
  Serial.println("t_us,run,phase,duty_D,Udc_V,Ua_V,omega_rad_s,dt_us");
}

void updateExperimentState(uint32_t nowUs) {
  const uint32_t elapsed = nowUs - phaseStartUs;
  if (phase == ExperimentPhase::baseline && elapsed >= BASELINE_US) {
    setPhase(ExperimentPhase::step, nowUs);
  } else if (phase == ExperimentPhase::step && elapsed >= STEP_US) {
    setPhase(ExperimentPhase::recovery, nowUs);
  } else if (phase == ExperimentPhase::recovery && elapsed >= RECOVERY_US) {
    runIndex++;
    if (runIndex >= STEP_COUNT) {
      stopExperiment(false);
    } else {
      setPhase(ExperimentPhase::baseline, nowUs);
    }
  }
}

void logSample(uint32_t nowUs) {
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
  const uint32_t tUs = nowUs - experimentStartUs;
  char line[160];
  const int length = snprintf(
      line, sizeof(line), "%lu,%u,%u,%.6f,%.4f,%.4f,%.6f,%lu\n",
      static_cast<unsigned long>(tUs),
      static_cast<unsigned int>(runIndex + 1),
      static_cast<unsigned int>(phase),
      dutyCommand, SUPPLY_VOLTAGE, uqCommand, motor.shaft_velocity,
      static_cast<unsigned long>(dtUs));

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
  Serial.println("V1_05_Identification_Step");
  Serial.println("Fix motor securely; run unloaded first; keep clear of the shaft.");
  Serial.printf("Configured Udc=%.4f V. Replace SUPPLY_VOLTAGE with the measured value.\n",
                SUPPLY_VOLTAGE);
  SimpleFOCDebug::enable(&Serial);

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
  motor.LPF_velocity.Tf = 0.005f;
  motor.target = 0.0f;

  motor.init();
  if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
    motor.disable();
    Serial.println("Motor init failed. Check configuration and reset.");
    return;
  }
  Serial.println("Running initFOC: rotor alignment will move the motor.");
  if (!motor.initFOC()) {
    motor.disable();
    Serial.println("FOC init failed. Check phase wiring, sensor and pole pairs.");
    return;
  }

  motorReady = true;
  Serial.println("# READY: start serial logging at 921600 baud, then send S.");
  Serial.println("# CSV fields: t_us, run, phase, duty_D, Udc_V, Ua_V, omega_rad_s, dt_us.");
  Serial.println("# Four runs: D = 0.1, 0.2, 0.4, 0.6; Ua = D * Udc.");
  Serial.printf("# At Udc=%.4f V, maximum Ua=%.4f V. Send X to abort.\n",
                SUPPLY_VOLTAGE, MOTOR_VOLTAGE_LIMIT);
}

void loop() {
  if (!motorReady) {
    delay(1);
    return;
  }

  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    if ((input == 'S' || input == 's') && phase == ExperimentPhase::idle) {
      startExperiment();
    } else if ((input == 'X' || input == 'x') && experimentRunning) {
      stopExperiment(true);
      return;
    }
  }

  const uint32_t nowUs = micros();
  if (experimentRunning) updateExperimentState(nowUs);
  if (!motorReady || phase == ExperimentPhase::done || phase == ExperimentPhase::aborted) return;

  // SimpleFOC 推荐高频执行 loopFOC()，move() 不做速度闭环，只写入 Uq 目标。
  motor.loopFOC();
  if (sensor.currWireError != 0) {
    Serial.println("# AS5600 I2C ERROR");
    stopExperiment(true);
    return;
  }
  motor.move(uqCommand);

  if (experimentRunning) logSample(micros());
}
