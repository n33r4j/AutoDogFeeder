#!/usr/bin/bash

source install/local_setup.bash
colcon build
ros2 run adf_prototype camera
#ros2 launch adf_demo_launch.py