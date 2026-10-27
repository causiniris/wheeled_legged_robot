/*
  AS5600 手转测试（new）
  ESP32 + AS5600：VCC=3.3V，GND=GND，SDA=21，SCL=22。
  串口：115200。上电自动清零；发送 Z 或 z 可在当前位置重新清零。
  输出：相对角度(rad)、净累计计数、净圈数、速度(rad/s)、RPM、
        相对 +4 圈的计数偏差、相对 -4 圈的计数偏差。
  4096 计数/机械圈；+4 圈=+16384，-4 圈=-16384。
  这是 I2C 角度换算的计数，不是 ABZ 硬件脉冲计数。
  本程序不驱动电机。左右轮依次接入，并在各次测试前清零。
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;
constexpr uint8_t AS5600_ADDRESS = 0x36;
constexpr long COUNTS_PER_TURN = 4096;
constexpr long FOUR_TURN_COUNTS = 4 * COUNTS_PER_TURN;
constexpr uint32_t PRINT_INTERVAL_MS = 100;

// 若约定的正转方向读出负数，将此值改为 -1；不要对累计计数取绝对值。
constexpr int SENSOR_DIRECTION = 1;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
float startAngle = 0.0f;
uint32_t lastPrint = 0;
bool sensorReady = false;

void zeroPosition() {
  // loop() 持续更新传感器；这里只设置相对零点，不清除库的多圈状态。
  startAngle = sensor.getAngle();
  sensor.getVelocity();  // 建立新的测速时间基准。
  lastPrint = millis();
  Serial.println("Zero set: counts=0, turns=0. Send Z to zero again.");
  Serial.println("angle_rad\tcounts\tturns\tvelocity_rad_s\trpm\terr_pos4_counts\terr_neg4_counts");
}

void setup() {
  Serial.setTxBufferSize(256);
  Serial.begin(115200);
  delay(1000);
  Serial.println("V1_01_AS5600_Sensor_Test_new");
  Serial.println("SDA=21, SCL=22. Rotate by hand; motor driver power OFF.");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(AS5600_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("AS5600 not found at 0x36. Check wiring, then reset ESP32.");
    return;
  }

  sensor.init(&Wire);
  sensor.update();
  if (sensor.currWireError != 0) {
    Serial.println("AS5600 I2C read failed. Check wiring, then reset ESP32.");
    return;
  }
  sensorReady = true;
  zeroPosition();
}

void loop() {
  if (!sensorReady) {
    delay(1);
    return;
  }

  // 每次循环采样、跟踪跨零点和多圈；只限制打印频率，不用 delay(100)。
  sensor.update();
  if (sensor.currWireError != 0) {
    sensorReady = false;
    Serial.println("AS5600 I2C error. Test stopped; reconnect and reset to zero.");
    return;
  }

  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    if (input == 'Z' || input == 'z') zeroPosition();
  }

  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    const float deltaAngle = SENSOR_DIRECTION * (sensor.getAngle() - startAngle);
    const float turns = deltaAngle / _2PI;
    const long counts = lroundf(turns * static_cast<float>(COUNTS_PER_TURN));
    const float velocity = SENSOR_DIRECTION * sensor.getVelocity();
    const float rpm = velocity * 60.0f / _2PI;
    // 偏差统一定义为“实测净累计计数 - 理论计数”。
    const long errorPositive4 = counts - FOUR_TURN_COUNTS;
    const long errorNegative4 = counts + FOUR_TURN_COUNTS;

    Serial.printf("%.4f\t%ld\t%.4f\t%.4f\t%.2f\t%ld\t%ld\n",
                  deltaAngle, counts, turns, velocity, rpm,
                  errorPositive4, errorNegative4);
  }
}
