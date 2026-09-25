# ROVERBOT

ROS 2 C++ rover stack for NVIDIA Jetson Orin Nano, ROS 2 Humble, Orbbec Astra Pro, YOLO/TensorRT, RTAB-Map, ESP32, and TB6612FNG motor control.

## Structure

- `firmware/esp32_motor/` - ESP32 motor firmware
- `src/rover_motor/` - manual and ROS motor control
- `src/rover_camera/` - RGB and depth camera nodes
- `src/rover_yolo/` - TensorRT YOLO node
- `src/rover_bringup/` - ROS 2 launch files
- `models/` - local YOLO model/engine location (model binaries are ignored by Git)

## Build

```bash
cd ~/ROVERBOT
source /opt/ros/humble/setup.bash
chmod +x install.sh
./install.sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Manual control

```bash
ros2 run rover_motor rover_control
```

Controls: W forward, S backward, A left, D right, SPACE stop, `[`/`]` left PWM down/up, `-`/`=` right PWM down/up, P show PWM, Q quit.

## Camera

```bash
ros2 run rover_camera rgb_node
ros2 run rover_camera depth_node
```

## Sensors launch

```bash
ros2 launch rover_bringup sensors.launch.xml
```

## YOLO

Place a TensorRT engine at `~/ROVERBOT/models/yolo11n.engine`, then run:

```bash
ros2 run rover_yolo yolo_node --ros-args -p engine:=$HOME/ROVERBOT/models/yolo11n.engine
```

## RTAB-Map

```bash
ros2 launch rover_bringup mapping.launch.xml
```

## Full rover

```bash
ros2 launch rover_bringup rover.launch.xml
```

Test motors, RGB, depth, YOLO, and RTAB-Map individually before running the full stack.
