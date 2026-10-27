/*
  作业 4.1：SimpleFOC Mini 三相驱动板占空比/线电压标定。

  重要安全条件：
  1. 标定时必须断开 2804 电机；不要用电机绕组充当静态负载。
  2. 万用表/示波器测量 M1-M2，M3 悬空；建议 M1-M2 并接 100 ohm / 5 W
     电阻作为轻载（没有合适电阻时只做空载测量并在报告中注明）。
  3. SimpleFOC Mini 输入电源与 ESP32 必须共地。

  接线：IN1=GPIO32，IN2=GPIO33，IN3=GPIO25，EN=GPIO12。
  串口 115200，换行结束。命令示例：D0.1、D0.2、D0.4、D0.6、D0.8、D0。

  本程序让 M1 以 D 占空比 PWM，M2/M3 保持低电平。因此 M1-M2 的平均
  线电压可用于拟合 Ua = k * D * Udc + b。它只是驱动板台架标定，不等于
  电机运行时 SVPWM 的任一相占空比，也不等于 q 轴电压。
*/

#include <Arduino.h>
#include <SimpleFOC.h>

constexpr float SUPPLY_VOLTAGE_CONFIG = 12.0f;
constexpr long PWM_FREQUENCY_HZ = 10000;

BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);
Commander command = Commander(Serial);
float dutyCommand = 0.0f;
bool driverReady = false;

void applyDuty(float duty) {
  dutyCommand = constrain(duty, 0.0f, 0.8f);
  if (dutyCommand <= 0.0f) {
    driver.setPwm(0.0f, 0.0f, 0.0f);
    driver.disable();
  } else {
    driver.enable();
    driver.setPwm(dutyCommand * SUPPLY_VOLTAGE_CONFIG, 0.0f, 0.0f);
  }
  Serial.printf("duty_cmd=%.4f, expected_ideal_average_V=%.4f\n",
                dutyCommand, dutyCommand * SUPPLY_VOLTAGE_CONFIG);
}

void onDuty(char* cmd) {
  if (!driverReady) return;
  command.scalar(&dutyCommand, cmd);
  applyDuty(dutyCommand);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("V1_04_Driver_Calibration");
  Serial.println("MOTOR MUST BE DISCONNECTED. Measure M1-M2 average voltage.");
  Serial.println("Commands: D0.1 D0.2 D0.4 D0.6 D0.8 D0");

  driver.voltage_power_supply = SUPPLY_VOLTAGE_CONFIG;
  driver.voltage_limit = SUPPLY_VOLTAGE_CONFIG;
  driver.pwm_frequency = PWM_FREQUENCY_HZ;
  if (!driver.init()) {
    Serial.println("Driver init failed. Check GPIO configuration and reset.");
    return;
  }
  driver.disable();
  command.add('D', onDuty, "phase-A duty ratio, 0.0 to 0.8");
  driverReady = true;
  Serial.println("Ready. Record actual Udc and measured Ua for at least 5 points.");
}

void loop() {
  command.run();
}
