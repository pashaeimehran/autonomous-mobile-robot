# Hardware and wiring

## Components

- Raspberry Pi 4B running ROS 1 Noetic
- YDLIDAR X2 2D laser scanner
- L298N dual H-bridge motor driver
- Two DC gear motors in a differential-drive arrangement
- Passive caster wheel
- Motor power supply and regulated logic supply

## Raspberry Pi to L298N

The GPIO configuration uses **BCM numbering** for the C++ `pigpio` interface. Pin assignments match the physical header wiring of the Python controller.

| Signal | Raspberry Pi physical pin | BCM GPIO | L298N channel |
|---|---:|---:|---|
| Right motor IN1 | 11 | 17 | A |
| Right motor IN2 | 13 | 27 | A |
| Right motor ENA / PWM | 8 | 14 | A |
| Left motor IN1 | 16 | 23 | B |
| Left motor IN2 | 18 | 24 | B |
| Left motor ENB / PWM | 12 | 18 | B |

Direction polarity can be adjusted through the `invert_right` and `invert_left` private ROS parameters. Both motor-enable signals use `pigpio` software PWM with a configured range of 0–100 and a requested frequency of 1 kHz.

## Electrical integration

- Remove the L298N ENA/ENB jumpers when controlling motor speed through GPIO PWM.
- Use a shared ground reference between the Raspberry Pi and the motor driver.
- Supply motors from a suitable motor power source. Keep the Pi GPIO inputs within 3.3 V logic limits.
- GPIO14 is also used for UART transmit on some Pi configurations; disable conflicting serial-console assignments before using it for motor PWM.
- Set both PWM outputs to zero before switching H-bridge direction, during normal stop, and during shutdown.

## LiDAR placement

The URDF locates the `laser` frame relative to `base_link` at:

```text
translation: x = 0.1007 m, y = 0 m, z = 0.0558 m
rotation:    roll = 0, pitch = 0, yaw = 0
```

The sensor driver must publish scans with a frame consistent with `laser`.
