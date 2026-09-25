# ROVERBOT

ROS 2 C++ rover stack for:

- NVIDIA Jetson Orin Nano
- ROS 2 Humble
- Orbbec Astra Pro
- YOLO + TensorRT
- RTAB-Map
- ESP32
- TB6612FNG motor driver

## Motor protocol

Jetson -> ESP32:

F = Forward
B = Backward
L = Left
R = Right
S = Stop

A### = Left motor PWM
C### = Right motor PWM

Examples:

A120
C115
F

## Build

Install dependencies:

```bash
chmod +x install.sh
./install.sh
