#!/usr/bin/env bash
# open_gicp_loc_terminals.sh
# Launch the latest GICP localization stack in clearly separated modes.
#
# Modes:
#   bag  / rosbag : rosbag replay with --clock, use_sim_time=true
#   real / live   : real robot/sensors, wall-clock time, use_sim_time=false

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
MODE="${MODE:-bag}"
BAG="${BAG:-$WS/slam_bag/l402_map_0503}"
BAG_RATE="${BAG_RATE:-1.0}"
LOOP_BAG="${LOOP_BAG:-false}"
MAP="${MAP:-$WS/maps/l402_2d_map_0503.yaml}"
PCD_MAP="${PCD_MAP:-$WS/maps/saved-map/map-l402-0503/l402_points_0503}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/gicp_nav2.rviz}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_gicp_loc_logs}"
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

Launch all GICP localization terminals with explicit mode separation.

Modes:
  --mode bag      Rosbag replay mode. Starts:
                    robot_description rsp.launch.py with use_sim_time=true
                    localization_pkg fast_lio_localization_bag.launch.py
                    ros2 bag play --clock with /tf_static remapped away
                    PLY overlay (/l402_glim_points) with use_sim_time=true
                    RViz and status checks

  --mode real     Real robot mode. Starts:
                    robot_bringup test_all.launch.py
                    localization_pkg fast_lio_localization_live.launch.py
                    PLY overlay (/l402_glim_points) with use_sim_time=false
                    RViz and status checks

Options:
  --mode MODE        bag|rosbag|real|live (default: $MODE)
  --bag PATH         bag directory for bag mode (default: $BAG)
  --rate N           bag playback rate (default: $BAG_RATE)
  --loop             loop rosbag playback
  --map PATH         2D Nav2 map yaml (default: $MAP)
  --pcd-map PATH     3D PLY map for GICP (default: $PCD_MAP)
  --rviz PATH        RViz config (default: $RVIZ_CONFIG)
  --initial-x X      initial x in map frame (default: $INITIAL_X)
  --initial-y Y      initial y in map frame (default: $INITIAL_Y)
  --initial-yaw YAW  initial yaw rad in map frame (default: $INITIAL_YAW)
  --cleanup          kill old bag/GLIM/GICP/RViz/bringup processes first
  --no-rviz          do not start RViz
  --no-overlay       do not start /l402_glim_points overlay publisher
  --no-bag           bag mode: do not start rosbag play
  --no-bringup       real mode: do not start robot_bringup
  -h, --help         show this help

Environment overrides:
  WS, MODE, BAG, BAG_RATE, LOOP_BAG, MAP, PCD_MAP, RVIZ_CONFIG, ROS_LOG_DIR,
  CLEANUP, INITIAL_X, INITIAL_Y, INITIAL_YAW, START_RVIZ, START_OVERLAY,
  START_BAG, START_BRINGUP
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
  *) echo "Invalid --mode: $MODE (use bag or real)" >&2; exit 2 ;;
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
  echo "[gicp_loc] Cleaning old localization processes..."
  pkill -f "ros2 bag play" || true
  pkill -f "glim_rosnode" || true
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
echo 'mode: $MODE   use_sim_time: $USE_SIM_TIME'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[gicp_loc] mode        : $MODE"
echo "[gicp_loc] use_sim_time: $USE_SIM_TIME"
echo "[gicp_loc] map         : $MAP"
echo "[gicp_loc] pcd_map     : $PCD_MAP"
echo "[gicp_loc] rviz        : $RVIZ_CONFIG"
echo "[gicp_loc] initial     : x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW"
if [[ "$MODE" == "bag" ]]; then
  echo "[gicp_loc] bag         : $BAG"
  echo "[gicp_loc] bag rate    : $BAG_RATE"
fi

if [[ "$MODE" == "real" ]]; then
  OVERLAY_MODE="live"
else
  OVERLAY_MODE="bag"
fi

if [[ "$MODE" == "bag" ]]; then
  open_terminal "gicp bag 1 robot_state_publisher" \
    'ros2 launch robot_description rsp.launch.py use_sim_time:=true'
  sleep 2
fi

if [[ "$MODE" == "real" && "$START_BRINGUP" == "true" ]]; then
  open_terminal "gicp real 1 robot_bringup" \
    'ros2 launch robot_bringup test_all.launch.py \
      camera:=false \
      rviz:=false \
      use_glim_loc:=true'
  sleep 6
fi

open_terminal "gicp $MODE 2 localization" \
  'ros2 launch localization_pkg '"$LOC_LAUNCH"' \
    map:="'"$MAP"'" \
    pcd_map:="'"$PCD_MAP"'" \
    initial_x:="'"$INITIAL_X"'" \
    initial_y:="'"$INITIAL_Y"'" \
    initial_yaw:="'"$INITIAL_YAW"'"'

sleep 2

if [[ "$MODE" == "bag" && "$START_BAG" == "true" ]]; then
  bag_cmd='ros2 bag play "'"$BAG"'" --clock --rate "'"$BAG_RATE"'" --remap /tf_static:=/bag_tf_static_ignored'
  if [[ "$LOOP_BAG" == "true" ]]; then
    bag_cmd+=' --loop'
  fi
  open_terminal "gicp bag 3 rosbag --clock" "$bag_cmd"
  sleep 1
fi

if [[ "$START_OVERLAY" == "true" ]]; then
  open_terminal "gicp $MODE 4 ply overlay" \
    'tools/publish_l402_overlay.sh '"$OVERLAY_MODE"
  sleep 1
fi

if [[ "$START_RVIZ" == "true" ]]; then
  open_terminal "gicp $MODE 5 rviz" \
    'rviz2 -d "'"$RVIZ_CONFIG"'" --ros-args -p use_sim_time:='"$USE_SIM_TIME"
  sleep 1
fi

open_terminal "gicp $MODE 6 status" \
  'echo "Waiting for localization topics..."
sleep 5
while true; do
  echo
  date
  echo "--- mode/time ---"
  echo "mode='"$MODE"' use_sim_time='"$USE_SIM_TIME"'"
  if [[ "'"$MODE"'" == "bag" ]]; then
    echo "/clock is required:"
    timeout 3s ros2 topic echo /clock --once 2>/dev/null || echo "no /clock"
  else
    echo "/clock should usually be absent in real mode:"
    ros2 topic list 2>/dev/null | grep -x /clock || echo "no /clock OK"
  fi

  echo "--- GICP score ---"
  timeout 3s ros2 topic echo /gicp_loc/score --once 2>/dev/null || echo "no /gicp_loc/score yet"

  echo "--- TF chain ---"
  timeout 3s ros2 run tf2_ros tf2_echo map odom 2>/dev/null || echo "no map->odom"
  timeout 3s ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null || echo "no odom->base_footprint"
  timeout 3s ros2 run tf2_ros tf2_echo base_footprint base_link 2>/dev/null || echo "no base_footprint->base_link"
  timeout 3s ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null || echo "no map->base_footprint"

  echo "--- robot model source ---"
  timeout 3s ros2 topic echo /robot_description --once 2>/dev/null | head -c 200 || echo "no /robot_description"
  echo

  echo "--- point clouds ---"
  timeout 5s ros2 topic hz /livox/lidar --window 5 2>/dev/null || echo "no /livox/lidar"
  timeout 3s ros2 topic echo /l402_glim_points --once --field header 2>/dev/null || echo "no /l402_glim_points"
  timeout 3s ros2 topic echo /scan --once --field header 2>/dev/null || echo "no /scan"

  echo "--- graph sanity ---"
  echo "duplicate node names:"
  ros2 node list 2>/dev/null | sort | uniq -d || true
  echo "/tf publishers:"
  ros2 topic info -v /tf 2>/dev/null | sed -n "1,80p" || echo "no /tf"

  echo "sleeping 8 sec..."
  sleep 8
done'

echo "[gicp_loc] All requested terminals opened."
echo
if [[ "$MODE" == "bag" ]]; then
  echo "Bag mode rule: use robot_state_publisher + fast_lio_localization_bag.launch.py + ros2 bag play --clock."
  echo "Do not run robot_bringup/test_all at the same time."
else
  echo "Real mode rule: use fast_lio_localization_live.launch.py + robot_bringup."
  echo "Do not run ros2 bag play at the same time."
fi
