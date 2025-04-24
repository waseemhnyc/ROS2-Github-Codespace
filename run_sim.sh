#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/drone_ws/install/setup.bash

# Start the drone simulator
ros2 launch delft_drone_sim sim.launch.py 