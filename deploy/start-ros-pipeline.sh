#!/bin/bash
# RoboMaster Sniper: ROS pipeline auto-start launcher
# Wrapper: source ROS 2 → start hik_camera + video_encoder

set -e

source /opt/ros/humble/setup.bash
source /home/rmnuc1/RM-customization/install/setup.bash

echo "[sniper] Starting ROS pipeline..."
exec ros2 launch bringup sniper.launch.py
