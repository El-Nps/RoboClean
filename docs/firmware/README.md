
# Firmware

[← Docs](../README.md)


# Firmware (Arduino)

_**Note**: The firmware includes a built-in PID controller to maintain target wheel speeds based on encoder ticks. This is necessary because the motors require a lot of PWM to get the robot moving from a standstill. The firmware sends raw encoder tick deltas that the Python `base_controller` node interprets for odometry computations._

## Command Reference

| Command | Direction | Description | Example | Ack |
| --- | ---: | --- | --- | --- |
| `M,targetTicksL,targetTicksR` | host→device | Set target wheel encoder ticks per interval | `M,12,-12` | none |
| `V,state` | host→device | Vacuum on/off (`state` 0 or 1) | `V,1` | `V,1` |
| `B,state` | host→device | Brush on/off (`state` 0 or 1) | `B,1` | `B,1` |
| `R,a,pin` | host→device | Read analog `pin` (A0 as 0) | `R,a,0` | `SA,0,<value>` |
| `R,d,pin` | host→device | Read digital `pin` | `R,d,4` | `SD,4,<value>` |
| `E,deltaLeft,deltaRight` | device→host | Encoder tick deltas, sent ~20Hz | `E,12,15` | n/a |
| `S,left,middle,right` | device→host | Down-facing proximity analog readings, sent ~10Hz | `S,123,456,789` | n/a |
| `?` | device→host | Unknown command response | `?` | n/a |

## Firmware Tests

These tests require a working python installation and the `pyserial` package.

```bash
pip install pyserial
```

_**Note**: Some tests might require additional dependencies._

## Flash Arduino

The firmware on the Arduino can be reloaded using the arduino-cli:

```bash
arduino-cli compile --upload -p /dev/... --fqbn arduino:avr:nano:cpu=atmega328old firmware
```

_**Note**: Replace `/dev/...` with the actual serial port._

[Hardware →](../hardware/README.md)
