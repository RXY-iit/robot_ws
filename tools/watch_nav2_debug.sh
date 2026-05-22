#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_cli_logs}"

set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

exec "$WS/tools/nav2_debug_monitor.py"
