# Dependencies

This project can be built with Arduino IDE or PlatformIO.

## Required Libraries And Frameworks

| Dependency | Purpose | Version |
|---|---|---|
| ESP32 Arduino core | ESP32 board support | Installed through Arduino IDE or PlatformIO `espressif32` |
| SimpleFOC | BLDC motor, driver, sensor, and control logic | `^2.2` in PlatformIO |
| Wire | I2C communication for AS5600 | Built into Arduino framework |

## Arduino IDE Setup

1. Install Arduino IDE 2.x.
2. Install the ESP32 board package.
3. Install the SimpleFOC library from Library Manager.
4. Open one of the `.ino` sketches.
5. Select the ESP32 board and upload.
6. Open Serial Monitor at `115200` baud.

## PlatformIO Setup

Install PlatformIO, then build one of the environments:

```bash
pio run -e sensor_test
pio run -e openloop_test
pio run -e closedloop_velocity_test
```

Upload example:

```bash
pio run -e closedloop_velocity_test -t upload
```

Serial monitor:

```bash
pio device monitor -b 115200
```

## Sketch Dependencies

| Sketch | SimpleFOC | Wire / I2C | AS5600 Required |
|---|---|---|---|
| `V1_01_AS5600_Sensor_Test` | Yes | Yes | Yes |
| `V1_02_2804_OpenLoop_Test` | Yes | No | No |
| `V1_03_ClosedLoop_Velocity_Test` | Yes | Yes | Yes |

## Version Notes

The PlatformIO configuration uses the `espressif32` platform and the SimpleFOC dependency declared in `platformio.ini`.  
When using Arduino IDE instead of PlatformIO, install the ESP32 board package and SimpleFOC library from the IDE package/library managers.
