#!/bin/bash
# RoboMaster Sniper: ROS pipeline auto-start launcher
# Wrapper: source ROS 2 → start hik_camera + video_encoder

set -e

source /opt/ros/kilted/setup.bash
source /home/hyc/002/Pacific_doorlock_sniper/install/setup.bash

echo "[sniper] Starting ROS pipeline..."
exec ros2 launch bringup sniper.launch.py
