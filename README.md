# Arduino Smartphone Gimbal

An Arduino-based two-axis smartphone gimbal using an MPU-6050 motion sensor, two MG996R servo motors, and a KY-023 joystick.

## Interactive 3D Model

Explore the step-by-step 3D model of the gimbal:

[**Open the Interactive 3D Model**](https://ayyan839.github.io/Arduino-Gimbal/3d-model/)

## Demonstration Video

Watch the gimbal demonstration:

[**Watch the Demonstration**](video/)

## Source Code

The Arduino source code is available in `ArduinoGimbal.ino`.

## Hardware Connections

| Component | Arduino Pin |
|---|---|
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |
| Joystick VRx | A0 |
| Joystick VRy | A1 |
| Joystick SW | D2 |
| Pitch servo signal | D9 |
| Roll servo signal | D10 |

The Arduino and servos use separate power sources. The external servo supply ground must be connected to Arduino GND.

Refer to the circuit diagram in the `images/` folder for the wiring layout.

## Repository Contents

- `ArduinoGimbal.ino` — Arduino source code
- `images/` — Circuit diagram and project images
- `video/` — Demonstration video
- `3d-model/` — Interactive step-by-step 3D model

## License

This project is licensed under the MIT License.
