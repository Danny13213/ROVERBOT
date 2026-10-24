# Rover Startup with iPad

How to start ROVERBOT and drive it from the iPad. Written after testing on October 23, 2026.

## Quick reference

| What | Value |
| --- | --- |
| Robot computer | Jetson Nano (Ubuntu 22.04, ROS 2 Humble) |
| Network | UniFi Wi-Fi only (for now) |
| SSH login | `ssh cad281@10.0.200.126` |
| ROS code folder | `~/ROVERBOT` (motor, cameras, YOLO, navigation) |
| Old Python folder (with a dash) | `~/ROVER-BOT` (`camera_stream.py`) |
| Camera Python script | `/home/cad281/ROVER-BOT/camera_stream.py` |
| Start command | `rover` |
| Rosbridge port (app connection) | `9090` |
| Video feed port | `5000` (path `/stream`) |

## Start the rover (one command)

1. Join UniFi Wi-Fi on both the laptop and the iPad.
2. SSH into the Jetson: `ssh cad281@10.0.200.126`
3. Run: `rover`

   This starts everything: Rosbridge (`9090`), both cameras, YOLO, obstacle avoidance, the motor node and the video feed (`5000`). Press Ctrl+C to stop it all.
4. On the iPad: open the ROVERBOT app, set Rosbridge `9090` and Video feed `5000`, then tap Connect.
5. Manual uses the arrow buttons. Autonomous drives by itself, so give it clear floor space. STOP always works.

> **Warning:** Before running `rover`, close any other terminal running `camera_stream.py`, `motor_node` or Rosbridge. Two copies fight over the camera or the motor port.

## Start each part by hand (if `rover` doesn't work)

Every new terminal must load ROS first:

```bash
cd ~/ROVERBOT && source /opt/ros/humble/setup.bash && source install/setup.bash
```

Then run each part in its own terminal:

| Part | Command | Port |
| --- | --- | --- |
| Rosbridge (app connection) | `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` | 9090 |
| Robot (cameras, YOLO, autonomous, motors) | `ros2 launch rover_bringup rover.launch.xml` | - |
| Video feed | `ros2 run rover_camera web_stream_node` | 5000 |
| Motors only (manual driving, no autonomous) | `ros2 run rover_motor motor_node` | - |

Manual-only option with the old Python camera: `cd ~/ROVER-BOT && python3 camera_stream.py` (port `5000`). Run it from inside `~/ROVER-BOT`, because it needs `astra_camera.py` and `nav_logic.py` from that folder. Don't use it together with the ROS cameras or autonomous mode.

## Problems we hit and the fixes

| What you see | Cause | Fix |
| --- | --- | --- |
| App can't connect / `nc -zv localhost 9090` refused | Rosbridge not running | Start Rosbridge. The firewall (ufw) was not the problem. |
| "Could not set PWM: Service /rover_motor/set_parameters does not exist" | Motor node not running | Run `motor_node` from `~/ROVERBOT` (not `~/ROVER-BOT`). |
| "Package 'rover_motor' not found" | Forgot to load the workspace | Also run `source install/setup.bash`. |
| "Video: Could not connect" | Nothing on port 5000 | Start the video feed. The app asks for `/stream`. |
| "Camera can't open" / "Resource busy" | Only one program can use the camera; `camera_stream.py` was still holding it | `pkill -f camera_stream.py`, then start again. |
| "Could not enable Autonomous: obstacle_avoidance is not running" | Autonomous needs the full ROS stack, and the code was missing the `enabled` switch the app uses | Run `rover.launch.xml`. Code fix is in the pull request below. |

## Code changes

- Autonomous on/off switch added to `src/rover_navigation/src/obstacle_avoidance.cpp`.
- One-command launch file: `src/rover_bringup/launch/full.launch.xml` (used by `rover`).
- GitHub: [pull request #1 into the prof branch](https://github.com/Danny13213/ROVERBOT/pull/1). Waiting for Danny to merge. Not fully tested on the rover yet.

## Issues to address

- The rover only works on the UniFi network. Switch to Bluetooth or a USB dongle.
- The app's left and right controls are swapped.
- Improve obstacle detection for autonomous navigation.
- The camera is too zoomed in, making objects appear closer than they are.
- Add the ability to create paths for the rover to follow, instead of only traveling forward.
