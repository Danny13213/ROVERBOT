#!/usr/bin/env bash
set -euo pipefail

if [ ! -d /opt/ros/humble ]; then
  echo "ERROR: ROS 2 Humble is not installed at /opt/ros/humble"
  exit 1
fi

sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libopencv-dev \
  libopenni2-dev \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-camera-info-manager \
  ros-humble-image-transport \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-rtabmap-ros \
  ros-humble-rviz2

sudo usermod -aG dialout "$USER"

echo "Dependencies installed. Log out/in once for dialout group changes to take effect."
echo "Orbbec OpenNI2 runtime is expected at /opt/openni2-orbbec."
