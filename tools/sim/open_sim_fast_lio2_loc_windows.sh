#!/usr/bin/env bash
# Open Isaac Sim FAST-LIO2 + GICP localization test windows.
#
# This is the sim counterpart of tools/open_fast_lio2_loc_terminals.sh.
# It keeps the real-robot localization chain:
#   /point_cloud -> /livox/lidar(CustomMsg) -> FAST-LIO2
#   /livox/lidar(CustomMsg) -> /livox/lidar_pc2(PointCloud2) -> GICP + /scan
#   GICP publishes map -> odom
#
# Use this before Nav2: move the robot manually with Joy-Con and verify
# FAST-LIO2 odometry, GICP score, and map -> odom.

set -euo pipefail

WS="${WS:-/home/ruan-x/robot_ws}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
MAP="${MAP:-$WS/maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml}"
PCD_MAP="${PCD_MAP:-$WS/maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/sim_gicp_nav2.rviz}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/debug-output/sim_fast_lio2_loc_$(date +%Y%m%d_%H%M%S)/ros-log}"
CLEANUP="${CLEANUP:-false}"
START_JOY="${START_JOY:-true}"
START_RVIZ="${START_RVIZ:-true}"
START_OVERLAY="${START_OVERLAY:-true}"
PUBLISH_INITIALPOSE="${PUBLISH_INITIALPOSE:-true}"
INITIAL_X="${INITIAL_X:-0.0}"
INITIAL_Y="${INITIAL_Y:-0.0}"
INITIAL_YAW="${INITIAL_YAW:-0.0}"
SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:--0.8}"
SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:--0.3}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Open Isaac Sim FAST-LIO2 + GICP localization test windows.

Options:
  --map PATH          2D Nav2 map yaml (default: $MAP)
  --pcd-map PATH      3D PLY map for GICP (default: $PCD_MAP)
  --rviz PATH         RViz config (default: $RVIZ_CONFIG)
  --joy-dev PATH      Joy-Con/gamepad device (default: $JOY_DEV)
  --initial-x X       initial robot x in map frame (default: $INITIAL_X)
  --initial-y Y       initial robot y in map frame (default: $INITIAL_Y)
  --initial-yaw YAW   initial robot yaw rad in map frame (default: $INITIAL_YAW)
  --scan-min-height Z pointcloud_to_laserscan min height (default: $SCAN_MIN_HEIGHT)
  --scan-max-height Z pointcloud_to_laserscan max height (default: $SCAN_MAX_HEIGHT)
  --cleanup           kill old sim localization/RViz processes first
  --no-joy            do not start joy_node/teleop_twist_joy
  --no-rviz           do not start RViz
  --no-overlay        do not publish the 3D PLY overlay
  --no-initialpose    do not publish the initial /initialpose automatically
  -h, --help          show this help

Before running:
  1. Isaac Sim is open and in Play.
  2. The sim MID360/ROS Script Editor setup is active.
  3. /clock, /point_cloud, /livox/imu, /wheel_odom, and /tf are visible.

Joy-Con manual drive:
  - B sets MANUAL mode.
  - Hold L1/LB dead-man and move sticks to publish /teleop/cmd_vel.
  - The full chain is /teleop/cmd_vel -> /cmd_vel_raw -> /cmd_vel -> sim drive.

Initial pose calibration:
  - This script publishes one /initialpose from --initial-x/y/yaw.
  - In RViz, use "2D Pose Estimate" to refine GICP's map -> odom at any time.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="$2"; shift 2 ;;
    --pcd-map|--ply) PCD_MAP="$2"; shift 2 ;;
    --rviz) RVIZ_CONFIG="$2"; shift 2 ;;
    --joy-dev) JOY_DEV="$2"; shift 2 ;;
    --initial-x) INITIAL_X="$2"; shift 2 ;;
    --initial-y) INITIAL_Y="$2"; shift 2 ;;
    --initial-yaw) INITIAL_YAW="$2"; shift 2 ;;
    --scan-min-height) SCAN_MIN_HEIGHT="$2"; shift 2 ;;
    --scan-max-height) SCAN_MAX_HEIGHT="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    --no-joy) START_JOY=false; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    --no-overlay) START_OVERLAY=false; shift ;;
    --no-initialpose) PUBLISH_INITIALPOSE=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

require_path() {
  if [[ ! -e "$1" ]]; then
    echo "Missing required path: $1" >&2
    exit 1
  fi
}

require_path "$WS/install/setup.bash"
require_path "$MAP"
require_path "$PCD_MAP"
require_path "$RVIZ_CONFIG"
command -v gnome-terminal >/dev/null 2>&1 || { echo "gnome-terminal not found" >&2; exit 1; }

read -r INITIAL_QZ INITIAL_QW < <(
  python3 -c 'import math, sys; yaw=float(sys.argv[1]); print(math.sin(yaw/2.0), math.cos(yaw/2.0))' "$INITIAL_YAW"
)

mkdir -p "$ROS_LOG_DIR"

set +u
conda deactivate >/dev/null 2>&1 || true
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID
export ROS_LOG_DIR
set -u

RELAY_EXE="$WS/install/localization_pkg/lib/localization_pkg/livox_custom_to_pc2_node"
if [[ ! -x "$RELAY_EXE" ]] || head -c 2 "$RELAY_EXE" | grep -q '#!'; then
  cat >&2 <<EOF
[sim_fast_lio2_loc] ERROR: FAST-LIO2 mode needs the C++ livox_custom_to_pc2_node relay.
[sim_fast_lio2_loc] Missing or Python relay:
  $RELAY_EXE

Rebuild:
  cd $WS
  source /opt/ros/humble/setup.bash
  colcon build --packages-select localization_pkg --symlink-install
  source install/setup.bash
EOF
  exit 1
fi

if [[ "$CLEANUP" == "true" ]]; then
  echo "[sim_fast_lio2_loc] Cleaning old processes..."
  pkill -f "sim_manual_bringup.launch.py" || true
  pkill -f "fast_lio_localization.launch.py" || true
  pkill -f "fastlio_mapping" || true
  pkill -f "gicp_localizer_node" || true
  pkill -f "livox_custom_to_pc2_node" || true
  pkill -f "pointcloud2_to_livox_custom_msg_node" || true
  pkill -f "pointcloud_to_laserscan_node" || true
  pkill -f "nav2_map_server" || true
  pkill -f "lifecycle_manager" || true
  pkill -f "sim_omni_drive_command_node" || true
  pkill -f "robot_state_publisher" || true
  pkill -f "interactive_ply_initialpose_publisher.py" || true
  pkill -f "publish_ply_pointcloud.py" || true
  pkill -f "rviz2" || true
  ros2 daemon stop || true
  ros2 daemon start || true
  sleep 1
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
export ROS_DOMAIN_ID="'"$ROS_DOMAIN_ID"'"
export ROS_LOG_DIR="'"$ROS_LOG_DIR"'"
set -u
cd "'"$WS"'"
'

open_window() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$common_setup
echo '===== $title ====='
echo 'map: $MAP'
echo 'pcd_map: $PCD_MAP'
echo 'initial: x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW'
echo 'ros_log_dir: $ROS_LOG_DIR'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

cat <<EOF
[sim_fast_lio2_loc] map             : $MAP
[sim_fast_lio2_loc] pcd_map         : $PCD_MAP
[sim_fast_lio2_loc] rviz            : $RVIZ_CONFIG
[sim_fast_lio2_loc] joy_dev         : $JOY_DEV
[sim_fast_lio2_loc] start_joy       : $START_JOY
[sim_fast_lio2_loc] initial         : x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW
[sim_fast_lio2_loc] ROS_DOMAIN_ID   : $ROS_DOMAIN_ID
[sim_fast_lio2_loc] ROS_LOG_DIR     : $ROS_LOG_DIR
EOF

open_window "sim loc 1 bringup joy adapter" '
echo "Checking Isaac topics..."
ros2 topic list | sort
echo
echo "Starting sim bringup: robot TF, Joy/teleop/mode/safety, sim drive, and PointCloud2 -> Livox CustomMsg adapter..."
ros2 launch robot_sim sim_manual_bringup.launch.py \
  joy_dev:="'"$JOY_DEV"'" \
  start_joy:='"$START_JOY"' \
  start_pc2_adapter:=true \
  direct_cmd_vel:=false \
  use_sim_time:=true'

sleep 2

open_window "sim loc 2 fast-lio2 gicp" '
echo "Waiting for sim /livox/lidar CustomMsg and /livox/imu..."
sleep 4
ros2 launch localization_pkg fast_lio_localization.launch.py \
  map:="'"$MAP"'" \
  pcd_map:="'"$PCD_MAP"'" \
  initial_x:="'"$INITIAL_X"'" \
  initial_y:="'"$INITIAL_Y"'" \
  initial_yaw:="'"$INITIAL_YAW"'" \
  with_fast_lio:=true \
  use_fast_lio_hint:=true \
  use_sim_time:=true \
  scan_min_height:="'"$SCAN_MIN_HEIGHT"'" \
  scan_max_height:="'"$SCAN_MAX_HEIGHT"'"'

sleep 1

open_window "sim loc 3 initialpose" '
echo "Waiting for GICP and odom TF..."
sleep 9
if [[ "'"$PUBLISH_INITIALPOSE"'" == "true" ]]; then
  echo "Publishing initial /initialpose for GICP..."
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: '"$INITIAL_X"', y: '"$INITIAL_Y"', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: '"$INITIAL_QZ"', w: '"$INITIAL_QW"'}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}" || true
fi
echo
echo "Initial pose calibration:"
echo "  - RViz 2D Pose Estimate publishes /initialpose."
echo "  - gicp_localizer resets map->odom from that pose."
echo "  - For Joy-Con movement, keep robot_mode MANUAL: press B if needed."
echo "  - Hold L1/LB dead-man and move the sticks slowly."
echo
echo "Listening for future /initialpose clicks..."
ros2 topic echo /initialpose'

sleep 1

if [[ "$START_RVIZ" == "true" ]]; then
  open_window "sim loc 4 rviz overlay" '
sleep 6
if [[ "'"$START_OVERLAY"'" == "true" ]]; then
  python3 tools/publish_ply_pointcloud.py "'"$PCD_MAP"'" --topic /sim_glim_points --frame-id map --use-sim-time &
fi
rviz2 -d "'"$RVIZ_CONFIG"'" --ros-args -p use_sim_time:=true'
  sleep 1
fi

open_window "sim loc 5 status" '
echo "Waiting for localization graph..."
sleep 8
while true; do
  echo
  date
  echo "--- graph snapshot ---"
  ros2 node list 2>/dev/null | sort || echo "no nodes"
  ros2 topic list -t 2>/dev/null | grep -E "^/(point_cloud|livox/lidar|livox/lidar_pc2|livox/imu|fast_lio|gicp_loc|scan|tf|tf_static|map|wheel_odom|cmd_vel|robot_mode)" || true

  echo "--- sim inputs ---"
  timeout 3s ros2 topic echo /clock --once 2>/dev/null || echo "no /clock"
  ros2 topic info -v /point_cloud 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /point_cloud"
  timeout 5s ros2 topic hz /point_cloud --window 5 2>/dev/null || echo "no /point_cloud hz"
  ros2 topic info -v /livox/lidar 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /livox/lidar"
  timeout 5s ros2 topic hz /livox/lidar --window 5 2>/dev/null || echo "no /livox/lidar CustomMsg hz"
  ros2 topic info -v /livox/lidar_pc2 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /livox/lidar_pc2"
  timeout 5s ros2 topic hz /livox/lidar_pc2 --window 5 2>/dev/null || echo "no /livox/lidar_pc2 relay hz"
  ros2 topic info -v /livox/imu 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /livox/imu"
  timeout 5s ros2 topic hz /livox/imu --window 20 2>/dev/null || echo "no /livox/imu hz"

  echo "--- FAST-LIO2 ---"
  ros2 topic info -v /fast_lio/odometry 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /fast_lio/odometry topic"
  timeout 3s ros2 topic echo /fast_lio/odometry --once --field header 2>/dev/null || echo "no /fast_lio/odometry"
  ros2 topic info -v /fast_lio/cloud_registered 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /fast_lio/cloud_registered topic"
  timeout 5s ros2 topic hz /fast_lio/cloud_registered --window 5 2>/dev/null || echo "no /fast_lio/cloud_registered"

  echo "--- GICP ---"
  ros2 topic info -v /gicp_loc/score 2>/dev/null | sed -n "/Type:/p;/Publisher count:/,/Subscription count:/p" || echo "no /gicp_loc/score topic"
  timeout 3s ros2 topic echo /gicp_loc/score --once 2>/dev/null || echo "no /gicp_loc/score yet"
  timeout 3s ros2 topic echo /gicp_loc/pose --once --field header 2>/dev/null || echo "no /gicp_loc/pose yet"
  timeout 3s ros2 topic echo /scan --once --field header 2>/dev/null || echo "no /scan"

  echo "--- TF ---"
  timeout 4s ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true 2>/dev/null || echo "no map->odom"
  timeout 4s ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true 2>/dev/null || echo "no odom->base_footprint"
  timeout 4s ros2 run tf2_ros tf2_echo base_footprint base_link --ros-args -p use_sim_time:=true 2>/dev/null || echo "no base_footprint->base_link"

  echo "--- manual command chain ---"
  timeout 3s ros2 topic echo /robot_mode --once 2>/dev/null || echo "no /robot_mode"
  ros2 topic info -v /teleop/cmd_vel 2>/dev/null | sed -n "/Publisher count/,\$p" || echo "no /teleop/cmd_vel"
  ros2 topic info -v /cmd_vel 2>/dev/null | sed -n "/Publisher count/,\$p" || echo "no /cmd_vel"
  timeout 3s ros2 topic echo /wheel_odom --once --field header 2>/dev/null || echo "no /wheel_odom"

  echo "sleeping 8 sec..."
  sleep 8
done'

echo "[sim_fast_lio2_loc] All windows requested."
