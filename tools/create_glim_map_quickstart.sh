#!/usr/bin/env bash
# Open terminals for live GLIM mapping and rosbag recording.
#
# Typical usage:
#   tools/create_glim_map_quickstart.sh --bag l402_2026_05_03
#
# Terminals opened:
#   1. full hardware bringup: sensors + motors + current URDF TF
#   2. Joy-Con teleop
#   3. GLIM mapping
#   4. rosbag record
#   5. status monitor
#
# The rosbag topic set follows tools/record_semantic_bag.sh so mapping bags are
# also useful for later semantic map construction and GICP replay tests.
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
GLIM_CONFIG="${GLIM_CONFIG:-$WS/glim_config}"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_glim_mapping_logs}"
BAG_NAME="${BAG_NAME:-}"
BAG_DIR="${BAG_DIR:-$WS/slam_bag}"
CLEANUP="${CLEANUP:-false}"
RVIZ="${RVIZ:-false}"
CAMERA="${CAMERA:-true}"
SERIAL_MOTORS="${SERIAL_MOTORS:-true}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag NAME [options]

Open terminals for live GLIM mapping and rosbag recording.

Options:
  --bag NAME          Bag output name under $BAG_DIR.
                      Example: --bag l402_new_30deg
  --bag-dir DIR       Bag output directory parent (default: $BAG_DIR)
  --config DIR        GLIM config directory (default: $GLIM_CONFIG)
  --rviz              Open RViz from test_all.launch.py
  --no-camera         Do not start RealSense camera
  --no-serial-motors  Do not start serial linear/camera-swing motors
  --cleanup           Kill old mapping/teleop/record/rviz processes first
  -h, --help          Show this help

Environment overrides:
  WS, GLIM_CONFIG, ROS_LOG_DIR, BAG_NAME, BAG_DIR, CLEANUP, RVIZ,
  CAMERA, SERIAL_MOTORS, EXTRA_TOPICS

Recorded topics:
  Geometry:     /livox/lidar /livox/imu /wheel_odom
  TF:           /tf /tf_static /robot_description
  Camera RGB:   /camera/camera/color/image_raw /camera/camera/color/camera_info
  Camera depth: /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info
  Camera state: /chokudomotor/angle /cameraswingmotor/angle
  plus EXTRA_TOPICS when set
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_NAME="$2"
      shift 2
      ;;
    --bag-dir)
      BAG_DIR="$2"
      shift 2
      ;;
    --config)
      GLIM_CONFIG="$2"
      shift 2
      ;;
    --rviz)
      RVIZ=true
      shift
      ;;
    --no-camera)
      CAMERA=false
      shift
      ;;
    --no-serial-motors)
      SERIAL_MOTORS=false
      shift
      ;;
    --cleanup)
      CLEANUP=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "$BAG_NAME" ]]; then
        BAG_NAME="$1"
        shift
      else
        echo "Unknown argument: $1" >&2
        usage
        exit 2
      fi
      ;;
  esac
done

require_path() {
  if [[ ! -e "$1" ]]; then
    echo "Missing required path: $1" >&2
    exit 1
  fi
}

if [[ -z "$BAG_NAME" ]]; then
  echo "Missing bag name." >&2
  usage
  exit 2
fi

if [[ "$BAG_NAME" == */* ]]; then
  echo "Bag name should be a simple name, not a path: $BAG_NAME" >&2
  echo "Use --bag-dir to choose the parent directory." >&2
  exit 2
fi

require_path "$WS/install/setup.bash"
require_path "$GLIM_CONFIG/config.json"

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Run the commands manually." >&2
  exit 1
fi

mkdir -p "$ROS_LOG_DIR"
mkdir -p "$BAG_DIR"

BAG_PATH="$BAG_DIR/$BAG_NAME"
if [[ -e "$BAG_PATH" ]]; then
  echo "Bag output already exists: $BAG_PATH" >&2
  echo "Choose another --bag name." >&2
  exit 1
fi

if [[ "$CLEANUP" == "true" ]]; then
  echo "[glim_mapping] Cleaning old processes."
  pkill -f "glim_rosnode" || true
  pkill -f "ros2 bag record" || true
  pkill -f "test_all.launch.py" || true
  pkill -f "teleop.launch.py" || true
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
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[glim_mapping] Workspace : $WS"
echo "[glim_mapping] Config    : $GLIM_CONFIG"
echo "[glim_mapping] Bag       : $BAG_PATH"
echo "[glim_mapping] RViz      : $RVIZ"
echo "[glim_mapping] Camera    : $CAMERA"
echo "[glim_mapping] Serial    : $SERIAL_MOTORS"

open_terminal "glim_map 1 full hardware" \
  'ros2 launch robot_bringup test_all.launch.py \
    rviz:='"$RVIZ"' \
    camera:='"$CAMERA"' \
    serial_motors:='"$SERIAL_MOTORS"

sleep 4

open_terminal "glim_map 2 joycon teleop" \
  'ros2 launch robot_bringup teleop.launch.py'

sleep 2

open_terminal "glim_map 3 glim_rosnode" \
  'ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:="'"$GLIM_CONFIG"'"'

sleep 2

record_topics="
/livox/lidar
/livox/imu
/wheel_odom
/tf
/tf_static
/robot_description
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
/chokudomotor/angle
/cameraswingmotor/angle
"
if [[ -n "$EXTRA_TOPICS" ]]; then
  record_topics="$record_topics $EXTRA_TOPICS"
fi

open_terminal "glim_map 4 rosbag record" \
  'echo "Recording bag to: '"$BAG_PATH"'"
echo "Topics:"
printf "%s\n" '"$record_topics"'
ros2 bag record \
  --output "'"$BAG_PATH"'" \
  --compression-mode file \
  --compression-format zstd \
  '"$record_topics"

sleep 1

open_terminal "glim_map 5 status" \
  'echo "Waiting for topics..."
sleep 5
while true; do
  echo
  date
  echo "--- LiDAR input ---"
  timeout 5s ros2 topic hz /livox/lidar --window 5 2>/dev/null || echo "no /livox/lidar"
  echo "--- IMU input ---"
  timeout 5s ros2 topic hz /livox/imu --window 20 2>/dev/null || echo "no /livox/imu"
  echo "--- wheel odom ---"
  timeout 5s ros2 topic hz /wheel_odom --window 5 2>/dev/null || echo "no /wheel_odom"
  echo "--- current URDF LiDAR TF ---"
  timeout 3s ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null || echo "no base_link->livox_frame"
  echo "--- GLIM output ---"
  timeout 5s ros2 topic hz /glim_ros/aligned_points --window 5 2>/dev/null || echo "no /glim_ros/aligned_points"
  echo "--- recording output ---"
  du -sh "'"$BAG_PATH"'" 2>/dev/null || echo "bag not created yet"
  echo "sleeping 8 sec..."
  sleep 8
done'

echo "[glim_mapping] All terminals opened."
echo
echo "Drive slowly with the Joy-Con."
echo "When finished, stop GLIM and rosbag record with Ctrl-C in their terminals."
echo "Bag output: $BAG_PATH"
