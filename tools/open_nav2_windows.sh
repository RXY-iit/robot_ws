#!/usr/bin/env bash
# Simple gnome-terminal launcher for Nav2 Phase 1/real checks.
# This avoids Terminator layout/keybinding issues.

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
MAP="${MAP:-$WS/maps/l402_2d_map_clean_0509.yaml}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/nav2_navigation.rviz}"
PLY_MAP="${PLY_MAP:-$WS/maps/saved-map/map-l402-0503/l402_points_0503}"
CLEANUP="${CLEANUP:-false}"
PHASE1_SAFE="${PHASE1_SAFE:-false}"
DEBUG_OUTPUT_ROOT="${DEBUG_OUTPUT_ROOT:-$WS/debug-output}"
DEBUG_SESSION_DIR="${DEBUG_SESSION_DIR:-$DEBUG_OUTPUT_ROOT/nav2_$(date +%Y%m%d_%H%M%S)}"
WITH_FAST_LIO="${WITH_FAST_LIO:-true}"
USE_FAST_LIO_HINT="${USE_FAST_LIO_HINT:-true}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Open Nav2 windows using gnome-terminal.

Options:
  --phase1-safe    Start sensors/localization/Nav2/RViz, but do not start
                   movement motors. Publishes fake stationary /wheel_odom.
  --fast-lio       Enable FAST-LIO2 mode (default):
                     - bringup: Livox xfer_format=1 (CustomMsg)
                     - localization: launches FAST-LIO + relay + GICP hint
  --no-fast-lio    Disable FAST-LIO2 and use GICP-only localization with
                   Livox PointCloud2 on /livox/lidar.
  --map PATH       2D Nav2 map yaml (default: $MAP)
  --cleanup        Kill old Nav2/localization/RViz/debug recorder processes first
  -h, --help       Show this help

Debug output:
  $DEBUG_SESSION_DIR

Environment overrides:
  WITH_FAST_LIO=false      Disable FAST-LIO mode without passing --no-fast-lio
  USE_FAST_LIO_HINT=false  Disable /fast_lio/odometry hint
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --phase1-safe|--safe|--no-motors) PHASE1_SAFE=true; shift ;;
    --fast-lio) WITH_FAST_LIO=true; USE_FAST_LIO_HINT=true; shift ;;
    --no-fast-lio) WITH_FAST_LIO=false; USE_FAST_LIO_HINT=false; shift ;;
    --map) MAP="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -f "$WS/install/setup.bash" ]] || { echo "Missing $WS/install/setup.bash" >&2; exit 1; }
[[ -f "$MAP" ]] || { echo "Missing map: $MAP" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "Missing RViz config: $RVIZ_CONFIG" >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "gnome-terminal not found" >&2; exit 1; }

mkdir -p "$DEBUG_SESSION_DIR"
mkdir -p "$DEBUG_SESSION_DIR/ros-log"

if [[ "$PHASE1_SAFE" == "true" ]]; then
  BRINGUP_ARGS="rviz:=false drive_motors:=false steer_motors:=false serial_motors:=false lift:=false static_odom:=false use_glim_loc:=true"
else
  BRINGUP_ARGS="rviz:=false"
fi

if [[ "$WITH_FAST_LIO" == "true" ]]; then
  BRINGUP_ARGS="$BRINGUP_ARGS fast_lio_mode:=true"
fi

if [[ "$CLEANUP" == "true" ]]; then
  echo "[nav2-windows] Cleaning old processes..."
  pkill -f "test_all.launch.py" || true
  pkill -f "navigation.launch.py" || true
  pkill -f "fast_lio_localization" || true
  pkill -f "gicp_localizer_node" || true
  pkill -f "pointcloud_to_laserscan_node" || true
  pkill -f "nav2_debug_recorder.py" || true
  pkill -f "nav2_debug_monitor.py" || true
  pkill -f "fake_wheel_odom.py" || true
  pkill -f "rviz2" || true
  pkill -f "publish_l402_overlay.sh" || true
  ros2 daemon stop || true
  ros2 daemon start || true
  sleep 1
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
export ROS_LOG_DIR="'"$DEBUG_SESSION_DIR"'/ros-log"
set -u
cd "'"$WS"'"
'

open_window() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$common_setup
echo '===== $title ====='
echo 'phase1_safe: $PHASE1_SAFE'
echo 'debug_output: $DEBUG_SESSION_DIR'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[nav2-windows] phase1_safe : $PHASE1_SAFE"
echo "[nav2-windows] map         : $MAP"
echo "[nav2-windows] rviz        : $RVIZ_CONFIG"
echo "[nav2-windows] debug_output: $DEBUG_SESSION_DIR"
echo "[nav2-windows] with_fast_lio: $WITH_FAST_LIO"
echo "[nav2-windows] fast_lio_hint: $USE_FAST_LIO_HINT"

if [[ "$WITH_FAST_LIO" == "true" ]]; then
  RELAY_EXE="$WS/install/localization_pkg/lib/localization_pkg/livox_custom_to_pc2_node"
  if [[ ! -x "$RELAY_EXE" ]] || head -c 2 "$RELAY_EXE" | grep -q '#!'; then
    cat >&2 <<EOF
[nav2-windows] ERROR: --fast-lio needs the C++ livox_custom_to_pc2_node relay.
[nav2-windows] Current installed relay is missing or still the old Python script:
  $RELAY_EXE

Rebuild before starting FAST-LIO mode:
  cd $WS
  conda deactivate  # if conda is active
  source /opt/ros/humble/setup.bash
  colcon build --packages-select localization_pkg --symlink-install
  source install/setup.bash

Why:
  The Python relay imports livox_ros_driver2/msg/CustomMsg and can fail when
  livox_ros_driver2 was built under a different Python version. The C++ relay
  uses C++ type support and avoids that Python type-support mismatch.
EOF
    exit 1
  fi
fi

open_window "nav2 0 debug" '
echo "Starting debug recorder immediately..."
tools/nav2_debug_recorder.py --output-dir "'"$DEBUG_SESSION_DIR"'" &
echo $! > "'"$DEBUG_SESSION_DIR"'/recorder.pid"
echo
echo "Starting live debug monitor early..."
tools/watch_nav2_debug.sh'

sleep 1

open_window "nav2 1 bringup" '
if [[ "'"$PHASE1_SAFE"'" == "true" ]]; then
  echo "Starting fake stationary /wheel_odom..."
  tools/fake_wheel_odom.py &
fi
ros2 launch robot_bringup test_all.launch.py '"$BRINGUP_ARGS"

sleep 1

open_window "nav2 2 localization" '
echo "Waiting 3 sec for bringup..."
sleep 3
ros2 launch localization_pkg fast_lio_localization_live.launch.py \
  map:="'"$MAP"'" \
  pcd_map:="'"$PLY_MAP"'" \
  with_fast_lio:="'"$WITH_FAST_LIO"'" \
  use_fast_lio_hint:="'"$USE_FAST_LIO_HINT"'"'

sleep 1

open_window "nav2 3 navigation" '
echo "Waiting 5 sec for localization..."
sleep 5
ros2 launch nav_pkg navigation.launch.py'

sleep 1

open_window "nav2 4 check" '
echo "Waiting 8 sec for Nav2 graph..."
sleep 8
tools/check_nav2_phase1.sh
echo "Debug recorder is already running in nav2 0 debug window."'

sleep 1

open_window "nav2 5 rviz" '
echo "Waiting 5 sec..."
sleep 5
if [[ -f "'"$PLY_MAP"'" ]]; then
  tools/publish_l402_overlay.sh live &
fi
rviz2 -d "'"$RVIZ_CONFIG"'"'

echo "[nav2-windows] All windows requested."
echo "[nav2-windows] Debug session: $DEBUG_SESSION_DIR"
