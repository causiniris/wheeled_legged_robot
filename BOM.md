# Bill Of Materials

This BOM is based on the hardware used in this ESP32 + SimpleFOC BLDC motor-control project.

## Core Hardware

| Item | Qty | Recommended Description For GitHub | Notes |
|---|---:|---|---|
| MCU | 1 | DOIT ESP32 DevKit V1 / ESP32 Dev Module | Main controller |
| Motor Model | 1 | 2804 BLDC gimbal motor, 7 pole pairs | Used by `BLDCMotor(7)` |
| Power Supply | 1 | 12V DC power supply, at least 2A recommended | Powers the motor driver |
| Encoder Module | 1 | AS5600 I2C magnetic encoder module | Connected via SDA/SCL |
| Motor Driver | 1 | SimpleFOC mini v1.0 3PWM BLDC driver board | Controlled by ESP32 PWM pins |
| USB Cable | 1 | USB cable for ESP32 programming and serial monitor | Also powers ESP32 during testing |

## Wiring And Assembly

| Item | Qty | Notes |
|---|---:|---|
| Dupont / jumper wires | Several | Used for ESP32, driver, and encoder wiring |
| Motor phase wires | 3 | Connect motor phases to the driver output |
| AS5600 cable | 1 | 4-wire I2C connection: VCC, GND, SDA, SCL |
| Common ground wiring | 1 set | ESP32, SimpleFOC mini, and 12V supply ground must be connected |
| Mounting hardware | As needed | Depends on the motor and encoder mount |

## Pin Mapping

| Signal | ESP32 Pin |
|---|---|
| AS5600 SDA | GPIO21 |
| AS5600 SCL | GPIO22 |
| Driver IN1 | GPIO32 |
| Driver IN2 | GPIO33 |
| Driver IN3 | GPIO25 |
| Driver EN | GPIO12 |

## Notes

- The motor is configured as a 7-pole-pair BLDC motor in the sketches.
- A 12V DC power supply rated at 2A or higher is recommended.
- If the hardware is purchased as a kit, this BOM can be used as a verification checklist.
