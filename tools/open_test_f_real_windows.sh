#!/usr/bin/env bash
# Open the real-robot Test-F calibration stack in separate gnome-terminal windows.

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/src/robot_description/rviz/robot.rviz}"
CLEANUP="${CLEANUP:-false}"
TEST_SCRIPT="$WS/todo-my/task-sim-real-dev/sim-real-calibration/test_f_calib_motion.py"
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/test_f_ros_logs}"
PROFILE="${PROFILE:-baseline}"
TRIAL="${TRIAL:-01}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Open the real-robot Test-F calibration stack.

JoyCon:
  A: AUTO, start/resume Test-F
  B: MANUAL, stop Test-F output and enable dead-man teleop
  L1: manual teleop dead-man
  Y: emergency-stop toggle

Options:
  --joy-dev PATH  joystick device (default: $JOY_DEV)
  --profile NAME  baseline|straight-forward|straight-reverse|lateral-left|
                  lateral-right|spin-ccw|spin-cw (default: $PROFILE)
  --trial ID      measurement trial id (default: $TRIAL)
  --cleanup       stop old Test-F/mode/safety/RViz processes first
  -h, --help      show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --joy-dev) JOY_DEV="$2"; shift 2 ;;
    --profile) PROFILE="$2"; shift 2 ;;
    --trial) TRIAL="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -f "$WS/install/setup.bash" ]] || { echo "Missing $WS/install/setup.bash" >&2; exit 1; }
[[ -f "$TEST_SCRIPT" ]] || { echo "Missing $TEST_SCRIPT" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "Missing $RVIZ_CONFIG" >&2; exit 1; }
[[ -e "$JOY_DEV" ]] || { echo "Missing joystick: $JOY_DEV" >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "gnome-terminal not found" >&2; exit 1; }
mkdir -p "$ROS_LOG_DIR"

if [[ "$CLEANUP" == "true" ]]; then
  echo "[test-f] Cleaning old calibration processes..."
  pkill -f "test_f_calib_motion.py" || true
  pkill -f "nav_mode_switch_node.py" || true
  pkill -f "cmd_vel_safety_node" || true
  pkill -f "test_all.launch.py" || true
  pkill -f "teleop_node" || true
  pkill -f "joy_node" || true
  pkill -f "rviz2" || true
  sleep 1
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
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
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[test-f] ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "[test-f] joy device   : $JOY_DEV"
echo "[test-f] bag output   : $WS/calib_bags"
echo "[test-f] ROS log      : $ROS_LOG_DIR"
echo "[test-f] profile/trial: $PROFILE / $TRIAL"

open_window "test-f 1 bringup" '
ros2 launch robot_bringup test_all.launch.py \
  lidar:=false camera:=false serial_motors:=false lift:=false \
  camera_motor_joy:=false rviz:=false use_glim_loc:=false \
  joy_dev:="'"$JOY_DEV"'" cmd_vel_pipeline:=false'

sleep 2

open_window "test-f 2 mode safety" '
echo "Starting AUTO/MANUAL arbitration and safety layer..."
ros2 run nav_pkg nav_mode_switch_node.py &
MODE_PID=$!
ros2 run safety_layer cmd_vel_safety_node &
SAFETY_PID=$!
trap "kill \$MODE_PID \$SAFETY_PID 2>/dev/null || true" EXIT INT TERM
wait'

sleep 1

open_window "test-f 3 motion bag" '
echo "Running preflight: zero steer/drive and check fresh feedback topics..."
tools/test_f_preflight.py
echo
echo "Test-F waits in MANUAL. Press A when the area is clear."
python3 "'"$TEST_SCRIPT"'" --label real --profile "'"$PROFILE"'" --trial "'"$TRIAL"'"'

sleep 1

open_window "test-f 4 rviz" '
rviz2 -d "'"$RVIZ_CONFIG"'"'

sleep 1

open_window "test-f 5 monitor" '
echo "A=AUTO/start/resume, B=MANUAL/pause, hold L1 for manual teleop, Y=ESTOP"
echo
ros2 topic echo /robot_mode &
MODE_ECHO_PID=$!
ros2 topic echo /safety_status &
SAFETY_ECHO_PID=$!
trap "kill \$MODE_ECHO_PID \$SAFETY_ECHO_PID 2>/dev/null || true" EXIT INT TERM
wait'

echo "[test-f] All windows requested. Robot starts in MANUAL."
