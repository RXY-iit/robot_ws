#!/usr/bin/env bash
# open_fast_lio2_loc_terminals.sh
# Quick launcher for Phase 2: FAST-LIO2 + GICP localization.
#
# This is an experimental Phase 2 entrypoint. FAST-LIO2 needs Livox CustomMsg
# input (livox_ros_driver2 xfer_format=1) unless you use a FAST-LIO2 fork or
# relay that supports PointCloud2. GICP still needs PointCloud2 on /livox/lidar.

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
MODE="${MODE:-real}"
BAG="${BAG:-$WS/slam_bag/l402_map_0503}"
BAG_RATE="${BAG_RATE:-1.0}"
LOOP_BAG="${LOOP_BAG:-false}"
MAP="${MAP:-$WS/maps/l402_2d_map_0503.yaml}"
PCD_MAP="${PCD_MAP:-$WS/maps/saved-map/map-l402-0503/l402_points_0503}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/gicp_nav2.rviz}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_fast_lio2_loc_logs}"
CLEANUP="${CLEANUP:-false}"
INITIAL_X="${INITIAL_X:-0.0}"
INITIAL_Y="${INITIAL_Y:-0.0}"
INITIAL_YAW="${INITIAL_YAW:-0.0}"
START_RVIZ="${START_RVIZ:-true}"
START_OVERLAY="${START_OVERLAY:-true}"
START_BAG="${START_BAG:-true}"
START_BRINGUP="${START_BRINGUP:-true}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Launch Phase 2 FAST-LIO2 + GICP localization.

Modes:
  --mode real     Real robot mode. Starts robot_bringup, then localization with:
                    with_fast_lio:=true use_fast_lio_hint:=true

  --mode bag      Rosbag replay mode. Starts robot_state_publisher,
                  localization with FAST-LIO2 enabled, and ros2 bag play --clock.
                  The bag must contain the FAST-LIO2-compatible Livox format
                  or your FAST-LIO2 build must support the bag's PointCloud2.

Options:
  --mode MODE        real|live|bag|rosbag (default: $MODE)
  --bag PATH         bag directory for bag mode (default: $BAG)
  --rate N           bag playback rate (default: $BAG_RATE)
  --loop             loop rosbag playback
  --map PATH         2D Nav2 map yaml (default: $MAP)
  --pcd-map PATH     3D PLY map for GICP (default: $PCD_MAP)
  --rviz PATH        RViz config (default: $RVIZ_CONFIG)
  --initial-x X      initial x in map frame (default: $INITIAL_X)
  --initial-y Y      initial y in map frame (default: $INITIAL_Y)
  --initial-yaw YAW  initial yaw rad in map frame (default: $INITIAL_YAW)
  --cleanup          kill old bag/GICP/FAST-LIO/RViz/bringup processes first
  --no-rviz          do not start RViz
  --no-overlay       do not start /l402_glim_points overlay publisher
  --no-bag           bag mode: do not start rosbag play
  --no-bringup       real mode: do not start robot_bringup
  -h, --help         show this help

Before real mode:
  1. Run tools/setup_fast_lio.sh once.
  2. Ensure fast_lio is built and sourced.
  3. Ensure Livox input format matches your FAST-LIO2 build.

EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --bag) BAG="$2"; shift 2 ;;
    --rate) BAG_RATE="$2"; shift 2 ;;
    --loop) LOOP_BAG=true; shift ;;
    --map) MAP="$2"; shift 2 ;;
    --pcd-map) PCD_MAP="$2"; shift 2 ;;
    --rviz) RVIZ_CONFIG="$2"; shift 2 ;;
    --initial-x) INITIAL_X="$2"; shift 2 ;;
    --initial-y) INITIAL_Y="$2"; shift 2 ;;
    --initial-yaw) INITIAL_YAW="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    --no-overlay) START_OVERLAY=false; shift ;;
    --no-bag) START_BAG=false; shift ;;
    --no-bringup) START_BRINGUP=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

case "$MODE" in
  bag|rosbag) MODE="bag"; USE_SIM_TIME=true; LOC_LAUNCH="fast_lio_localization_bag.launch.py" ;;
  real|live) MODE="real"; USE_SIM_TIME=false; LOC_LAUNCH="fast_lio_localization_live.launch.py" ;;
  *) echo "Invalid --mode: $MODE (use real or bag)" >&2; exit 2 ;;
esac

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
if [[ "$MODE" == "bag" && "$START_BAG" == "true" ]]; then
  require_path "$BAG"
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Run the printed commands manually." >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR"

if [[ "$CLEANUP" == "true" ]]; then
  echo "[fast_lio2_loc] Cleaning old localization processes..."
  pkill -f "ros2 bag play" || true
  pkill -f "fastlio_mapping" || true
  pkill -f "fast_lio_localization" || true
  pkill -f "gicp_localizer_node" || true
  pkill -f "pointcloud_to_laserscan_node" || true
  pkill -f "nav2_map_server" || true
  pkill -f "lifecycle_manager" || true
  pkill -f "test_all.launch.py" || true
  pkill -f "robot_state_publisher" || true
  pkill -f "interactive_ply_initialpose_publisher.py" || true
  pkill -f "publish_l402_overlay.sh" || true
  pkill -f "rviz2" || true
  ros2 daemon stop || true
  ros2 daemon start || true
  sleep 1
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
set -u
export ROS_LOG_DIR="'"$ROS_LOG_DIR"'"
cd "'"$WS"'"
'

open_terminal() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$common_setup
echo '===== $title ====='
echo 'mode: $MODE   use_sim_time: $USE_SIM_TIME   FAST-LIO2: enabled'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[fast_lio2_loc] mode        : $MODE"
echo "[fast_lio2_loc] use_sim_time: $USE_SIM_TIME"
echo "[fast_lio2_loc] map         : $MAP"
echo "[fast_lio2_loc] pcd_map     : $PCD_MAP"
echo "[fast_lio2_loc] rviz        : $RVIZ_CONFIG"
echo "[fast_lio2_loc] initial     : x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW"
echo "[fast_lio2_loc] WARNING     : FAST-LIO2 input format must match /livox/lidar."
if [[ "$MODE" == "bag" ]]; then
  echo "[fast_lio2_loc] bag         : $BAG"
  echo "[fast_lio2_loc] bag rate    : $BAG_RATE"
fi

if [[ "$MODE" == "real" ]]; then
  OVERLAY_MODE="live"
else
  OVERLAY_MODE="bag"
fi

if [[ "$MODE" == "bag" ]]; then
  open_terminal "fast_lio2 bag 1 robot_state_publisher" \
    'ros2 launch robot_description rsp.launch.py use_sim_time:=true'
  sleep 2
fi

if [[ "$MODE" == "real" && "$START_BRINGUP" == "true" ]]; then
  open_terminal "fast_lio2 real 1 robot_bringup" \
    'ros2 launch robot_bringup test_all.launch.py \
      camera:=false \
      rviz:=false \
      use_glim_loc:=true'
  sleep 6
fi

open_terminal "fast_lio2 $MODE 2 localization" \
  'ros2 launch localization_pkg '"$LOC_LAUNCH"' \
    map:="'"$MAP"'" \
    pcd_map:="'"$PCD_MAP"'" \
    initial_x:="'"$INITIAL_X"'" \
    initial_y:="'"$INITIAL_Y"'" \
    initial_yaw:="'"$INITIAL_YAW"'" \
    with_fast_lio:=true \
    use_fast_lio_hint:=true'

sleep 2

if [[ "$MODE" == "bag" && "$START_BAG" == "true" ]]; then
  bag_cmd='ros2 bag play "'"$BAG"'" --clock --rate "'"$BAG_RATE"'" --remap /tf_static:=/bag_tf_static_ignored'
  if [[ "$LOOP_BAG" == "true" ]]; then
    bag_cmd+=' --loop'
  fi
  open_terminal "fast_lio2 bag 3 rosbag --clock" "$bag_cmd"
  sleep 1
fi

if [[ "$START_OVERLAY" == "true" ]]; then
  open_terminal "fast_lio2 $MODE 4 ply overlay" \
    'tools/publish_l402_overlay.sh '"$OVERLAY_MODE"
  sleep 1
fi

if [[ "$START_RVIZ" == "true" ]]; then
  open_terminal "fast_lio2 $MODE 5 rviz" \
    'rviz2 -d "'"$RVIZ_CONFIG"'" --ros-args -p use_sim_time:='"$USE_SIM_TIME"
  sleep 1
fi

open_terminal "fast_lio2 $MODE 6 status" \
  'echo "Waiting for FAST-LIO2 + GICP topics..."
sleep 5
while true; do
  echo
  date
  echo "--- mode/time ---"
  echo "mode='"$MODE"' use_sim_time='"$USE_SIM_TIME"'"
  if [[ "'"$MODE"'" == "bag" ]]; then
    timeout 3s ros2 topic echo /clock --once 2>/dev/null || echo "no /clock"
  else
    ros2 topic list 2>/dev/null | grep -x /clock || echo "no /clock OK"
  fi

  echo "--- FAST-LIO2 output ---"
  timeout 3s ros2 topic echo /fast_lio/odometry --once --field header 2>/dev/null || echo "no /fast_lio/odometry"
  timeout 5s ros2 topic hz /fast_lio/cloud_registered --window 5 2>/dev/null || echo "no /fast_lio/cloud_registered"

  echo "--- GICP output ---"
  timeout 3s ros2 topic echo /gicp_loc/score --once 2>/dev/null || echo "no /gicp_loc/score yet"
  timeout 3s ros2 topic echo /gicp_loc/pose --once --field header 2>/dev/null || echo "no /gicp_loc/pose yet"

  echo "--- TF chain ---"
  timeout 3s ros2 run tf2_ros tf2_echo map odom 2>/dev/null || echo "no map->odom"
  timeout 3s ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null || echo "no odom->base_footprint"
  timeout 3s ros2 run tf2_ros tf2_echo base_footprint base_link 2>/dev/null || echo "no base_footprint->base_link"
  timeout 3s ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null || echo "no map->base_footprint"

  echo "--- input topics ---"
  ros2 topic info /livox/lidar 2>/dev/null || echo "no /livox/lidar"
  timeout 5s ros2 topic hz /livox/imu --window 20 2>/dev/null || echo "no /livox/imu"
  timeout 3s ros2 topic echo /scan --once --field header 2>/dev/null || echo "no /scan"

  echo "--- graph sanity ---"
  ros2 node list 2>/dev/null | sort | uniq -d || true
  echo "sleeping 8 sec..."
  sleep 8
done'

echo "[fast_lio2_loc] All requested terminals opened."
echo
echo "Phase 2 reminder:"
echo "  FAST-LIO2 must receive the LiDAR message type it was built/configured for."
echo "  Current hku-mars FAST_LIO config expects Livox CustomMsg on /livox/lidar."
echo "  GICP and /scan conversion still need PointCloud2, so a relay/fork may be required."
