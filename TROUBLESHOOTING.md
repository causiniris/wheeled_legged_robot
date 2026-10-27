# Troubleshooting

This page collects common checks for bringing up the ESP32 + SimpleFOC + AS5600 + 2804 BLDC setup.

## AS5600 Does Not Show Angle Changes

- Check that AS5600 VCC is connected to ESP32 `3.3V`.
- Check that AS5600 GND is connected to ESP32 GND.
- Check that SDA is connected to GPIO21.
- Check that SCL is connected to GPIO22.
- Make sure the magnet is centered above the AS5600 sensor.
- Upload `V1_01_AS5600_Sensor_Test` before testing motor control.

## Motor Does Not Move In Open Loop

- Confirm the SimpleFOC mini board has external 12V power.
- Confirm ESP32 GND, driver GND, and 12V supply negative are connected together.
- Confirm motor phases are connected to the driver output.
- Use `T6.28` instead of very small target values such as `T1`.
- Try increasing the voltage limit carefully with `L4` or `L5`.

## Motor Vibrates Or Rotates Poorly

- Check that the 2804 motor pole-pair count is correct. This project uses `BLDCMotor(7)`.
- Check motor phase wiring.
- Confirm the encoder is mechanically aligned with the motor shaft.
- Reduce the target velocity and voltage limit during debugging.

## Closed-Loop Init Fails Or Behaves Incorrectly

- Run the AS5600 sensor test first.
- Run the open-loop motor test before closed-loop testing.
- Make sure the motor can rotate freely during `initFOC()`.
- Follow the recommended power-up sequence in `V1_03_ClosedLoop_Velocity_Test`.
- If the motor direction is unexpected, check phase order and encoder orientation.
- `ERR-MOT: No current sense.` is expected because this project uses voltage-mode torque control without current sensing.

## Serial Commands Do Not Work

- Set Serial Monitor baud rate to `115200`.
- Use newline line ending if the command does not trigger.
- Commands are case-sensitive.
- Example commands: `T3`, `T6.28`, `T0`, `T-6.28`, `L4`, `L5`.
