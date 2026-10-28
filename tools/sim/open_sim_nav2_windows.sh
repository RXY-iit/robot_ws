#!/usr/bin/env bash
# Open the final Isaac Sim Nav2 test windows.

set -euo pipefail

WS="${WS:-/home/ruan-x/robot_ws}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
MAP="${MAP:-$WS/maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml}"
PLY_MAP="${PLY_MAP:-$WS/maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/nav2_navigation.rviz}"
SIM_JOY_CONFIG="${SIM_JOY_CONFIG:-$WS/src/robot_sim/config/sim_joy_teleop.yaml}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
DEBUG_OUTPUT_ROOT="${DEBUG_OUTPUT_ROOT:-$WS/debug-output}"
DEBUG_SESSION_DIR="${DEBUG_SESSION_DIR:-$DEBUG_OUTPUT_ROOT/sim_nav2_$(date +%Y%m%d_%H%M%S)}"
CLEANUP="${CLEANUP:-false}"
AUTO="${AUTO:-false}"
ENABLE_MOTION_POLICY="${ENABLE_MOTION_POLICY:-true}"
INITIAL_X="${INITIAL_X:-0.0}"
INITIAL_Y="${INITIAL_Y:-0.0}"
INITIAL_YAW="${INITIAL_YAW:-0.0}"
SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:--0.8}"
GICP_SCAN_MIN_HEIGHT="${GICP_SCAN_MIN_HEIGHT:--1000000000.0}"
GICP_SCAN_MAX_HEIGHT="${GICP_SCAN_MAX_HEIGHT:-1000000000.0}"
SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:--0.3}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Open Isaac Sim Nav2 test windows.

Options:
  --map PATH       2D Nav2 map yaml (default: $MAP)
  --ply PATH       3D PLY map overlay / GICP map (default: $PLY_MAP)
  --rviz PATH      RViz config (default: $RVIZ_CONFIG)
  --joy-dev PATH   Joy-Con/gamepad device (default: $JOY_DEV)
  --initial-x X    initial robot x in map frame (default: $INITIAL_X)
  --initial-y Y    initial robot y in map frame (default: $INITIAL_Y)
  --initial-yaw R  initial robot yaw rad in map frame (default: $INITIAL_YAW)
  --cleanup        Disabled; stop the owning ROS terminals explicitly
  --auto           Run startup checks, but keep nav_mode_switch in MANUAL
                   (safe sim behavior; no automatic AUTO command is sent)
  --no-auto        Keep nav_mode_switch in MANUAL after startup
  --motion-policy  Enable nav_mode_switch motion policy (default)
  --no-motion-policy
                   Disable nav_mode_switch motion policy and pass Nav2 MPPI cmd_vel through
  -h, --help       Show this help

Before running:
  1. Isaac Sim is open and in Play.
  2. Mapping/sensor Script Editor setup has been run.
  3. /clock, /point_cloud, /livox/imu, /wheel_odom, /tf, camera topics are visible.

This script uses FAST-LIO2 + GICP localization, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, and sim time.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --map) MAP="$2"; shift 2 ;;
    --ply|--pcd-map) PLY_MAP="$2"; shift 2 ;;
    --rviz) RVIZ_CONFIG="$2"; shift 2 ;;
    --joy-dev) JOY_DEV="$2"; shift 2 ;;
    --initial-x) INITIAL_X="$2"; shift 2 ;;
    --initial-y) INITIAL_Y="$2"; shift 2 ;;
    --initial-yaw) INITIAL_YAW="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    --auto) AUTO=true; shift ;;
    --no-auto) AUTO=false; shift ;;
    --motion-policy) ENABLE_MOTION_POLICY=true; shift ;;
    --no-motion-policy) ENABLE_MOTION_POLICY=false; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -f "$WS/install/setup.bash" ]] || { echo "Missing $WS/install/setup.bash" >&2; exit 1; }
[[ -f "$MAP" ]] || { echo "Missing map: $MAP" >&2; exit 1; }
[[ -f "$PLY_MAP" ]] || { echo "Missing PLY map: $PLY_MAP" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "Missing RViz config: $RVIZ_CONFIG" >&2; exit 1; }
[[ -f "$SIM_JOY_CONFIG" ]] || { echo "Missing sim joy config: $SIM_JOY_CONFIG" >&2; exit 1; }
[[ -f "$WS/src/robot_sim/config/livox_pc2_adapter.yaml" ]] || { echo "Missing sim Livox adapter config" >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "gnome-terminal not found" >&2; exit 1; }

set +u
conda deactivate >/dev/null 2>&1 || true
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
set -u

missing_nav2_plugins=()
ros2 pkg prefix nav2_mppi_controller >/dev/null 2>&1 || missing_nav2_plugins+=(ros-humble-nav2-mppi-controller)
ros2 pkg prefix nav2_regulated_pure_pursuit_controller >/dev/null 2>&1 || missing_nav2_plugins+=(ros-humble-nav2-regulated-pure-pursuit-controller)
ros2 pkg prefix nav2_navfn_planner >/dev/null 2>&1 || missing_nav2_plugins+=(ros-humble-nav2-navfn-planner)
if (( ${#missing_nav2_plugins[@]} > 0 )); then
  cat >&2 <<EOF
[sim-nav2] ERROR: missing Nav2 plugin package(s):
  ${missing_nav2_plugins[*]}

Current nav2_params.yaml uses:
  nav2_mppi_controller::MPPIController
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
  nav2_navfn_planner/NavfnPlanner

Install them before running final Nav2:
  sudo apt-get update
  sudo apt-get install -y ${missing_nav2_plugins[*]}
EOF
  exit 1
fi

mkdir -p "$DEBUG_SESSION_DIR/ros-log"

read -r INITIAL_QZ INITIAL_QW < <(
  python3 -c 'import math, sys; yaw=float(sys.argv[1]); print(math.sin(yaw/2.0), math.cos(yaw/2.0))' "$INITIAL_YAW"
)

export ROS_LOG_DIR="$DEBUG_SESSION_DIR/ros-log"

if [[ "$CLEANUP" == "true" ]]; then
  echo "[sim-nav2] --cleanup is disabled: stop the owning ROS terminals explicitly; no processes were killed." >&2
  exit 2
fi
/usr/bin/python3 "$WS/tools/sim_startup_preflight.py" --workspace "$WS" --domain "$ROS_DOMAIN_ID" --starting ros

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
export ROS_DOMAIN_ID="'"$ROS_DOMAIN_ID"'"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR="'"$DEBUG_SESSION_DIR"'/ros-log"
export PYTHONNOUSERSITE=1
set -u
cd "'"$WS"'"
'

open_window() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$common_setup
echo '===== $title ====='
echo 'map: $MAP'
echo 'ply: $PLY_MAP'
echo 'debug_output: $DEBUG_SESSION_DIR'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

cat <<EOF
[sim-nav2] map          : $MAP
[sim-nav2] ply          : $PLY_MAP
[sim-nav2] rviz         : $RVIZ_CONFIG
[sim-nav2] joy_config   : $SIM_JOY_CONFIG
[sim-nav2] joy_dev      : $JOY_DEV
[sim-nav2] debug_output : $DEBUG_SESSION_DIR
[sim-nav2] auto checks  : $AUTO
[sim-nav2] motion policy: $ENABLE_MOTION_POLICY
[sim-nav2] initial      : x=$INITIAL_X y=$INITIAL_Y yaw=$INITIAL_YAW

Mapping-era reminders:
  - Use a clean ROS terminal; avoid Isaac/conda library pollution.
  - Isaac must already be in Play.
  - FAST-LIO2 consumes /livox/lidar CustomMsg.
  - GICP and /scan consume /livox/lidar_pc2 PointCloud2.
EOF

B1_ROUTE_ARGS=""
if [[ "$(basename "$(dirname "$MAP")")" == "sim_factory_mid360_like_gpu_trueimu_011" ]]; then
  B1_ROUTE_ARGS="--waypoint-route $WS/src/robot_skills/config/waypoint_list_a.yaml"
fi

open_window "sim nav2 0 debug" '
tools/nav2_debug_recorder.py --output-dir "'"$DEBUG_SESSION_DIR"'" '"$B1_ROUTE_ARGS"' &
sleep 1
tools/watch_nav2_debug.sh'

sleep 1

open_window "sim nav2 1 inputs drive joy" '
echo "Checking Isaac topics..."
ros2 topic list | sort
echo
echo "Starting Joy-Con -> /teleop/cmd_vel..."
ros2 run joy joy_node --ros-args -p dev:="'"$JOY_DEV"'" -r joy:=/joy/physical &
ros2 run teleop_twist_joy teleop_node \
  --ros-args \
  --params-file "'"$SIM_JOY_CONFIG"'" \
  -r cmd_vel:=/teleop/cmd_vel &
echo "Starting Joy-Con -> sim lift/camera holder control..."
echo "  Y/A: arm lift base up/down"
echo "  left D-pad/stick left/right: camera pan yaw; up/down: camera swing pitch"
ros2 run serial_transciever lift_joy_node \
  --ros-args \
  -p enable_button:=4 \
  -p up_button:=3 \
  -p down_button:=0 &
ros2 run serial_transciever camera_motor_joy_node \
  --ros-args \
  -p enable_button:=2 \
  -p pan_axis:=6 \
  -p tilt_axis:=7 \
  -p pan_sign:=-1.0 \
  -p tilt_sign:=-1.0 \
  -p pan_step_deg:=5.0 \
  -p tilt_step_deg:=5.0 \
  -p command_period_sec:=0.05 \
  -p initial_tilt_angle:=101.8 \
  -p initial_pan_angle:=267.1 &
ros2 run robot_sim sim_lift_command_node \
  --ros-args \
  -p publish_rate:=30.0 \
  -p lift_speed_m_s:=0.06 \
  -p lift_lower_m:=0.0 \
  -p lift_upper_m:=0.2 \
  -p camera_pan_neutral_deg:=267.1 \
  -p camera_pan_lower_rad:=-1.5708 \
  -p camera_pan_upper_rad:=1.5708 \
  -p camera_neutral_deg:=101.8 \
  -p camera_down_offset_deg:=42.0 \
  -p camera_lower_rad:=-1.57080 \
  -p camera_upper_rad:=2.30383 \
  -p robot_cmd_vel_topic:=/cmd_vel \
  -p robot_mode_topic:=/robot_mode \
  -p stop_linear_threshold:=0.01 \
  -p stop_angular_threshold:=0.02 \
  -p cmd_vel_timeout_sec:=0.5 \
  -p use_sim_time:=true &
ros2 run serial_transciever camera_pan_tilt_joint_state_node --ros-args -p use_sim_time:=true &
echo "Starting /point_cloud -> /livox/lidar Livox CustomMsg adapter..."
ros2 run robot_sim pointcloud2_to_livox_custom_msg_node \
  --ros-args \
  --params-file "'"$WS"'/src/robot_sim/config/livox_pc2_adapter.yaml" \
  -p input_topic:="'"${B1_LIDAR_INPUT_TOPIC:-/point_cloud}"'" \
  -p use_sim_time:=true &
echo "Starting /cmd_vel -> /sim/target_joint_states bridge..."
ros2 run robot_sim sim_omni_drive_command_node'

sleep 1

open_window "sim nav2 2 robot tf" '
echo "Starting robot_state_publisher for base_footprint -> base_link -> livox_frame..."
ros2 launch robot_sim export_robot_sim_urdf.launch.py use_sim_time:=true'

sleep 1

open_window "sim nav2 3 fast-lio2 gicp" '
echo "Starting FAST-LIO2 + GICP localization..."
ros2 launch localization_pkg fast_lio_localization.launch.py \
  map:="'"$MAP"'" \
  pcd_map:="'"$PLY_MAP"'" \
  initial_x:="'"$INITIAL_X"'" \
  initial_y:="'"$INITIAL_Y"'" \
  initial_yaw:="'"$INITIAL_YAW"'" \
  with_fast_lio:=true \
  gicp_input_topic:="'"${B1_GICP_INPUT_TOPIC:-/livox/lidar_pc2}"'" \
  use_fast_lio_hint:=true \
  use_sim_time:=true \
  scan_min_height:="'"$GICP_SCAN_MIN_HEIGHT"'" \
  scan_max_height:="'"$GICP_SCAN_MAX_HEIGHT"'"'

sleep 1

open_window "sim nav2 3b arm touch mock" '
echo "Starting SO101 sim mock backend + guarded gateway + mounted actual/candidate TF + Touch..."
unset PYTHONNOUSERSITE
ros2 launch robot_sim sim_arm_touch.launch.py use_sim_time:=true start_touch:=true
'

sleep 1

open_window "sim nav2 4 navigation" '
sleep 5
ros2 launch nav_pkg navigation.launch.py \
  use_sim_time:=true \
  joy_estop_button:=-1 \
  enable_motion_policy:="'"$ENABLE_MOTION_POLICY"'"'

sleep 1

open_window "sim nav2 5 init auto check" '
echo "Waiting for Nav2 graph..."
sleep 10
echo "Requesting Nav2 lifecycle startup..."
timeout 8 ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" || true
for node in /controller_server /planner_server /behavior_server /bt_navigator; do
  state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
  if echo "$state" | grep -q "active"; then
    continue
  fi
  if echo "$state" | grep -q "unconfigured"; then
    echo "Fallback configure: $node"
    timeout 10 ros2 lifecycle set "$node" configure || true
    sleep 0.5
  fi
  state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
  if echo "$state" | grep -q "inactive"; then
    echo "Fallback activate: $node"
    timeout 10 ros2 lifecycle set "$node" activate || true
    sleep 0.5
  fi
done
sleep 3
echo "Nav2 lifecycle states:"
timeout 5 ros2 lifecycle get /controller_server || true
timeout 5 ros2 lifecycle get /planner_server || true
timeout 5 ros2 lifecycle get /bt_navigator || true
timeout 5 ros2 lifecycle get /behavior_server || true
sleep 1
echo "Publishing initial pose for GICP at the mapping start pose..."
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: '"$INITIAL_X"', y: '"$INITIAL_Y"', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: '"$INITIAL_QZ"', w: '"$INITIAL_QW"'}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]}}" || true
sleep 2
if [[ "'"$AUTO"'" == "true" ]]; then
  echo "Safe AUTO note: startup no longer publishes /nav_mode/command=AUTO."
  echo "Robot remains MANUAL until the Dashboard/Joy mode control explicitly requests AUTO."
  echo "Clearing sim E-Stop only..."
  ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}" || true
fi
WS="'"$WS"'" USE_SIM_TIME=true tools/check_nav2_phase1.sh
echo
echo "Quick command-chain checks:"
timeout 5 ros2 topic info -v /fast_lio/odometry || true
timeout 5 ros2 topic echo /gicp_loc/score --once || true
timeout 5 ros2 topic info -v /cmd_vel || true
timeout 5 ros2 topic info -v /sim/target_joint_states || true'

sleep 1

open_window "sim nav2 6 rviz" '
sleep 8
python3 tools/publish_ply_pointcloud.py "'"$PLY_MAP"'" --topic /sim_glim_points --frame-id map &
rviz2 -d "'"$RVIZ_CONFIG"'" --ros-args -p use_sim_time:=true'

echo "[sim-nav2] All windows requested."
echo "[sim-nav2] Debug session: $DEBUG_SESSION_DIR"
