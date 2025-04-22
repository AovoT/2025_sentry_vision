#! /bin/bash

cd /home/sentry/Downloads/2025_sentry/right/auto_aim_lasted
export ROS_DOMAIN_ID=1
colcon build
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
ROS2_PID=$!

wait $ROS2_PID
