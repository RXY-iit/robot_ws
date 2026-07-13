#!/usr/bin/env bash
set -eo pipefail

cd /home/ruan-x/robot_ws

# Avoid library conflicts from Conda, Isaac Sim, or another ROS overlay.
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset PYTHONHOME
unset LD_PRELOAD
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

exec ros2 launch robot_agent agent_dashboard.launch.py mode:=sim
