#!/usr/bin/env bash
# setup_fast_lio.sh — Clone and build FAST-LIO2 for ROS2 Humble + Livox MID360.
#
# Run once from the workspace root:
#   tools/setup_fast_lio.sh
#
# What this does:
#   1. Install system dependencies (Eigen3, PCL, etc.)
#   2. Clone hku-mars/FAST_LIO (ROS2 branch) into src/fast_lio
#   3. Build with colcon
#
# MID360 xfer_format note:
#   FAST-LIO2 (lidar_type: 1) expects livox_ros_driver2 CustomMsg.
#   The default in msg_MID360_launch.py is xfer_format=0 (PointCloud2), which
#   is used by gicp_localizer_node.py and pointcloud_to_laserscan.
#
#   When launching FAST-LIO2, temporarily switch to xfer_format=1 by running:
#     ros2 launch localization_pkg fast_lio_localization.launch.py with_fast_lio:=true
#   This launch file does NOT change the Livox driver config automatically.
#   You must edit msg_MID360_launch.py → xfer_format=1, rebuild, and revert
#   after FAST-LIO2 testing, OR use two simultaneous driver instances on
#   different namespaces (advanced, not done here).
#
#   Simpler alternative (Phase 2 upgrade path):
#     Use a FAST-LIO2 fork that natively supports MID360 PointCloud2 output.
#     Known fork: https://github.com/PiusLim373/FAST_LIO_ROS2  (check README)
#
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"

echo "[setup_fast_lio] Installing system dependencies..."
sudo apt-get update -q
sudo apt-get install -y \
  libeigen3-dev \
  libpcl-dev \
  libboost-dev \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions

echo "[setup_fast_lio] Cloning FAST-LIO2 ROS2 branch..."
cd "$WS/src"
if [[ -d fast_lio ]]; then
  echo "  src/fast_lio already exists, skipping clone."
else
  git clone https://github.com/hku-mars/FAST_LIO --recurse-submodules -b ROS2 fast_lio
fi

echo "[setup_fast_lio] Building fast_lio..."
cd "$WS"
conda deactivate 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build \
  --packages-select fast_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_cohesion+

echo ""
echo "[setup_fast_lio] Done."
echo ""
echo "Next steps:"
echo "  1. Edit src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py:"
echo "       xfer_format = 1   (CustomMsg, required by FAST-LIO2)"
echo "  2. Rebuild livox_ros_driver2:"
echo "       colcon build --packages-select livox_ros_driver2"
echo "  3. Source and launch with FAST-LIO2:"
echo "       cd /home/matsunaga-h/robot_ws"
echo "       source /opt/ros/humble/setup.bash"
echo "       source install/setup.bash"
echo "       ros2 pkg prefix fast_lio"
echo "       ros2 launch localization_pkg fast_lio_localization_live.launch.py with_fast_lio:=true use_fast_lio_hint:=true"
echo ""
echo "  Important: if your terminal was already open before building fast_lio,"
echo "  source install/setup.bash again; otherwise ROS 2 cannot find package 'fast_lio'."
echo ""
echo "  After FAST-LIO2 testing, revert xfer_format=0 for normal operation"
echo "  (GICP localizer and pointcloud_to_laserscan require PointCloud2)."
