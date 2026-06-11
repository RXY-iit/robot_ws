#!/usr/bin/env bash
# Open Terminator windows for Isaac Sim GLIM mapping and rosbag recording.
#
# Typical usage:
#   tools/sim/create_sim_glim_map_quickstart.sh --bag sim_warehouse_mid360_trueimu_001 --gpu --direct-cmd-vel --cleanup
#
# Before running this script:
#   1. Open src/robot_sim/usd/scenes/full_warehouse_r.usd in Isaac Sim.
#   2. Run the BEFORE Play cleanup/config + true IMU commands printed by terminal 1.
#   3. Press Play, then run only the AFTER Play MID360 LiDAR commands printed by terminal 1.
set -euo pipefail

WS="${WS:-/home/ruan-x/robot_ws}"
GLIM_CONFIG="${GLIM_CONFIG:-$WS/glim_config}"
GLIM_GPU="${GLIM_GPU:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
SIM_RMW_IMPLEMENTATION="${SIM_RMW_IMPLEMENTATION:-}"
SIM_CYCLONEDDS_URI="${SIM_CYCLONEDDS_URI:-}"
LOG_ROOT="${LOG_ROOT:-$WS/log}"
SESSION_LOG_DIR="${SESSION_LOG_DIR:-}"
ROS_LOG_DIR="${ROS_LOG_DIR:-}"
BAG_NAME="${BAG_NAME:-}"
BAG_DIR="${BAG_DIR:-$WS/sim_bag}"
CLEANUP="${CLEANUP:-false}"
RVIZ="${RVIZ:-true}"
JOY="${JOY:-true}"
DIRECT_CMD_VEL="${DIRECT_CMD_VEL:-false}"
FAKE_IMU="${FAKE_IMU:-false}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"
ALLOW_MULTI_POINT_CLOUD="${ALLOW_MULTI_POINT_CLOUD:-false}"
RECORD_D455="${RECORD_D455:-false}"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag NAME [options]

Open one split-pane Terminator window for Isaac Sim GLIM mapping with Isaac true IMU.

Options:
  --bag NAME       Bag output name under $BAG_DIR.
  --bag-dir DIR    Bag output parent directory (default: $BAG_DIR)
  --config DIR     GLIM config directory (default: $GLIM_CONFIG)
  --gpu            Use $WS/glim_config/sim GPU config root
  --no-rviz        Do not open RViz from sim_sensor_check.launch.py
  --no-joy         Do not start Joy-Con teleop chain
  --direct-cmd-vel Bypass mode/safety chain and drive sim from /sim/cmd_vel
  --no-fake-imu    Do not start robot_sim sim_livox_imu_node (default)
  --fake-imu       Start corrected ROS-side fake IMU fallback
  --allow-multi-point-cloud
                  Do not fail when /point_cloud has multiple Isaac publishers
  --record-d455    Wait for and record D455 color/depth topics (not used by GLIM)
  --cleanup        Kill old sim mapping/record/rviz processes first
  -h, --help       Show this help

Environment overrides:
  WS, GLIM_CONFIG, GLIM_GPU, ROS_DOMAIN_ID, SIM_RMW_IMPLEMENTATION,
  SIM_CYCLONEDDS_URI, LOG_ROOT, SESSION_LOG_DIR, ROS_LOG_DIR, BAG_NAME,
  BAG_DIR, CLEANUP, RVIZ, JOY, DIRECT_CMD_VEL, FAKE_IMU, EXTRA_TOPICS,
  ALLOW_MULTI_POINT_CLOUD, RECORD_D455, SIM_GLIM_DRY_RUN

Default DDS policy:
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  export ROS_DOMAIN_ID=20

Isaac Script Editor commands for the current mainline:

  [BEFORE Play]
  exec(open("$WS/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/list_articulations.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
  exec(open("$WS/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())

  [Press Play]

  [AFTER Play]
  exec(open("$WS/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())

For --record-d455, replace cleanup_d455_for_mapping.py before Play with:
  exec(open("$WS/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())

Then, after Play and MID360 setup, run once:
  exec(open("$WS/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
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
    --gpu)
      GLIM_GPU=true
      GLIM_CONFIG="$WS/glim_config/sim"
      shift
      ;;
    --no-rviz)
      RVIZ=false
      shift
      ;;
    --no-joy)
      JOY=false
      shift
      ;;
    --direct-cmd-vel)
      DIRECT_CMD_VEL=true
      shift
      ;;
    --no-fake-imu)
      FAKE_IMU=false
      shift
      ;;
    --fake-imu)
      FAKE_IMU=true
      shift
      ;;
    --allow-multi-point-cloud)
      ALLOW_MULTI_POINT_CLOUD=true
      shift
      ;;
    --record-d455)
      RECORD_D455=true
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
require_path "$WS/src/robot_sim/usd/scenes/full_warehouse_r.usd"

if [[ "$GLIM_GPU" == "true" ]]; then
  if ! dpkg-query -W 'ros-humble-glim-ros-cuda*' >/dev/null 2>&1; then
    cat >&2 <<EOF
GPU mode requested, but no ros-humble-glim-ros-cuda* package is installed.

Install one CUDA GLIM package that matches your NVIDIA driver/CUDA runtime, e.g.:
  sudo apt install -y ros-humble-glim-ros-cuda12.6

Then verify:
  nvidia-smi
  ros2 pkg executables glim_ros
EOF
    exit 1
  fi

  if ! nvidia-smi >/dev/null 2>&1; then
    cat >&2 <<EOF
GPU mode requested, but nvidia-smi cannot access the NVIDIA driver.

Check on this PC:
  nvidia-smi

If this runs inside a container/session, make sure GPU passthrough is enabled.
EOF
    exit 1
  fi
fi

if ! command -v terminator >/dev/null 2>&1; then
  echo "terminator not found. Install Terminator or start the listed commands manually." >&2
  exit 1
fi

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
if [[ -z "$SESSION_LOG_DIR" ]]; then
  SESSION_LOG_DIR="$LOG_ROOT/sim_glim_mapping_${BAG_NAME}_${RUN_STAMP}"
fi
if [[ -z "$ROS_LOG_DIR" ]]; then
  ROS_LOG_DIR="$SESSION_LOG_DIR/ros"
fi

mkdir -p "$SESSION_LOG_DIR"
mkdir -p "$ROS_LOG_DIR"
mkdir -p "$BAG_DIR"
mkdir -p "$WS/maps"

if [[ "${SIM_GLIM_DRY_RUN:-false}" != "true" && -e /tmp/dump ]]; then
  echo "[sim_glim_mapping] Archiving pre-run /tmp/dump."
  mv /tmp/dump "$SESSION_LOG_DIR/pre_run_dump"
fi

BAG_PATH="$BAG_DIR/$BAG_NAME"
if [[ -e "$BAG_PATH" ]]; then
  echo "Bag output already exists: $BAG_PATH" >&2
  echo "Choose another --bag name." >&2
  exit 1
fi

graceful_stop_matching() {
  local pattern="$1"
  local label="$2"
  local wait_sec="${3:-12}"

  if ! pgrep -f "$pattern" >/dev/null 2>&1; then
    return
  fi

  echo "[sim_glim_mapping] Requesting graceful stop: $label"
  pkill -INT -f "$pattern" || true
  local deadline=$((SECONDS + wait_sec))
  while (( SECONDS < deadline )); do
    pgrep -f "$pattern" >/dev/null 2>&1 || return
    sleep 1
  done

  echo "[sim_glim_mapping] Graceful stop timed out; terminating: $label" >&2
  pkill -TERM -f "$pattern" || true
}

if [[ "${SIM_GLIM_DRY_RUN:-false}" != "true" && "$CLEANUP" == "true" ]]; then
  echo "[sim_glim_mapping] Cleaning old processes."
  graceful_stop_matching "glim_rosnode" "old GLIM (allow map save)" 15
  graceful_stop_matching "ros2 bag record" "old rosbag recorder (flush metadata)" 8
  pkill -f "sim_sensor_check.launch.py" || true
  pkill -f "sim_livox_imu_node" || true
  pkill -f "rviz2" || true
  (
    unset RMW_IMPLEMENTATION CYCLONEDDS_URI
    export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
    ros2 daemon stop || true
    ros2 daemon start || true
  )
  sleep 1
fi

echo "[sim_glim_mapping] Ensuring no stale GLIM/fake-IMU processes remain."
if [[ "${SIM_GLIM_DRY_RUN:-false}" != "true" ]]; then
  graceful_stop_matching "glim_rosnode" "stale GLIM (allow map save)" 15
  pkill -f "sim_livox_imu_node" || true
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
set -u
export ROS_DOMAIN_ID="'"$ROS_DOMAIN_ID"'"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
if [[ -n "'"$SIM_RMW_IMPLEMENTATION"'" ]]; then
  export RMW_IMPLEMENTATION="'"$SIM_RMW_IMPLEMENTATION"'"
fi
if [[ -n "'"$SIM_CYCLONEDDS_URI"'" ]]; then
  export CYCLONEDDS_URI="'"$SIM_CYCLONEDDS_URI"'"
fi
export ROS_LOG_DIR="'"$ROS_LOG_DIR"'"
cd "'"$WS"'"
set +u
'

if ! bash -lc "$common_setup
ros2 pkg executables glim_ros >/dev/null 2>&1
"; then
  cat >&2 <<EOF
Missing ROS package: glim_ros

This script uses the same GLIM config as the real robot:
  $GLIM_CONFIG

No sim-specific GLIM config was created. Install or overlay the real robot GLIM
packages first, then rerun this command.

Check after installation:
  source /opt/ros/humble/setup.bash
  source $WS/install/setup.bash
  ros2 pkg list | grep -E '^glim$|^glim_ros$'
  ros2 pkg executables glim_ros
EOF
  exit 1
fi

PANE_SCRIPTS=()

add_pane() {
  local title="$1"
  local body="$2"
  local pane_number=$((${#PANE_SCRIPTS[@]} + 1))
  local pane_script="$SESSION_LOG_DIR/pane_${pane_number}.sh"

  cat > "$pane_script" <<EOF
# This file is loaded as an interactive Bash rcfile by Terminator.
$common_setup
echo '===== $title ====='
echo 'ROS/debug log directory: $ROS_LOG_DIR'
echo "ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=\${RMW_IMPLEMENTATION:-unset/default}"
echo "CYCLONEDDS_URI=\${CYCLONEDDS_URI:-unset/default}"
echo "ROS_LOG_DIR=\${ROS_LOG_DIR:-unset}"
echo "ros2=\$(command -v ros2)"
echo "robot_sim prefix=\$(ros2 pkg prefix robot_sim 2>/dev/null || echo missing)"
echo "glim_ros prefix=\$(ros2 pkg prefix glim_ros 2>/dev/null || echo missing)"
$body
echo
echo '[command finished] this interactive pane will remain open'
EOF
  chmod +x "$pane_script"
  PANE_SCRIPTS+=("$pane_script")
}

launch_terminator_layout() {
  local pane_count="${#PANE_SCRIPTS[@]}"
  local config_path="$SESSION_LOG_DIR/terminator-config"
  local layout_name="sim_glim_mapping"

  if (( pane_count != 5 && pane_count != 6 )); then
    echo "Unsupported Terminator pane count: $pane_count" >&2
    exit 1
  fi

  local pane_script
  for pane_script in "${PANE_SCRIPTS[@]}"; do
    if ! bash -n "$pane_script"; then
      echo "Generated pane script has invalid shell syntax: $pane_script" >&2
      echo "Terminator was not started. Inspect the script above and rerun." >&2
      exit 1
    fi
  done

  if [[ "${SIM_GLIM_DRY_RUN:-false}" == "true" ]]; then
    echo "[sim_glim_mapping] Dry run: all $pane_count generated pane scripts passed bash -n."
    return
  fi

  cat > "$config_path" <<EOF
[global_config]
  suppress_multiple_term_dialog = True
[keybindings]
[profiles]
  [[default]]
    scrollback_infinite = True
    exit_action = hold
[layouts]
  [[$layout_name]]
    [[[window0]]]
      type = Window
      parent = ""
      order = 0
      size = 1800, 1000
      maximised = True
    [[[main_split]]]
      type = HPaned
      parent = window0
      order = 0
      ratio = 0.5
    [[[left_top_split]]]
      type = VPaned
      parent = main_split
      order = 0
      ratio = 0.34
    [[[pane1]]]
      type = Terminal
      parent = left_top_split
      order = 0
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[0]} -i
      directory = $WS
    [[[left_bottom_split]]]
      type = VPaned
      parent = left_top_split
      order = 1
      ratio = 0.5
    [[[pane2]]]
      type = Terminal
      parent = left_bottom_split
      order = 0
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[1]} -i
      directory = $WS
EOF

  if (( pane_count == 5 )); then
    cat >> "$config_path" <<EOF
    [[[pane3]]]
      type = Terminal
      parent = left_bottom_split
      order = 1
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[2]} -i
      directory = $WS
    [[[right_split]]]
      type = VPaned
      parent = main_split
      order = 1
      ratio = 0.5
    [[[pane4]]]
      type = Terminal
      parent = right_split
      order = 0
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[3]} -i
      directory = $WS
    [[[pane5]]]
      type = Terminal
      parent = right_split
      order = 1
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[4]} -i
      directory = $WS
EOF
  else
    cat >> "$config_path" <<EOF
    [[[pane3]]]
      type = Terminal
      parent = left_bottom_split
      order = 1
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[2]} -i
      directory = $WS
    [[[right_top_split]]]
      type = VPaned
      parent = main_split
      order = 1
      ratio = 0.34
    [[[pane4]]]
      type = Terminal
      parent = right_top_split
      order = 0
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[3]} -i
      directory = $WS
    [[[right_bottom_split]]]
      type = VPaned
      parent = right_top_split
      order = 1
      ratio = 0.5
    [[[pane5]]]
      type = Terminal
      parent = right_bottom_split
      order = 0
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[4]} -i
      directory = $WS
    [[[pane6]]]
      type = Terminal
      parent = right_bottom_split
      order = 1
      profile = default
      command = bash --noprofile --rcfile ${PANE_SCRIPTS[5]} -i
      directory = $WS
EOF
  fi

  cat >> "$config_path" <<EOF
[plugins]
EOF

  echo "[sim_glim_mapping] Opening one Terminator window with $pane_count panes."
  terminator --no-dbus -g "$config_path" -l "$layout_name" &
}

wait_for_topic_cmd() {
  local topic="$1"
  local timeout_sec="$2"
  cat <<EOF
echo "Waiting up to $timeout_sec sec for $topic ..."
deadline=\$((SECONDS + $timeout_sec))
while (( SECONDS < deadline )); do
  if timeout 3s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "OK: $topic"
    break
  fi
  sleep 1
done
if (( SECONDS >= deadline )); then
  echo "ERROR: timed out waiting for $topic" >&2
  echo "Available topics:" >&2
  ros2 topic list >&2 || true
  exit 1
fi
EOF
}

wait_for_tf_cmd() {
  local target_frame="$1"
  local source_frame="$2"
  local timeout_sec="$3"
  cat <<EOF
echo "Waiting up to $timeout_sec sec for TF $target_frame -> $source_frame ..."
deadline=\$((SECONDS + $timeout_sec))
while (( SECONDS < deadline )); do
  if timeout 3s ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null | grep -q "Translation:"; then
    echo "OK: TF $target_frame -> $source_frame"
    break
  fi
  sleep 1
done
if (( SECONDS >= deadline )); then
  echo "ERROR: timed out waiting for TF $target_frame -> $source_frame" >&2
  echo "Current /tf_static sample:" >&2
  timeout 5s ros2 topic echo /tf_static --once >&2 || true
  exit 1
fi
EOF
}

assert_single_point_cloud_publisher_cmd() {
  cat <<'EOF'
if [[ "$ALLOW_MULTI_POINT_CLOUD" != "true" ]]; then
  echo "Checking /point_cloud publisher count ..."
  pc_pub_count=$(
    timeout 8s ros2 topic info -v /point_cloud 2>/dev/null \
      | awk '/Publisher count:/ {print $3; exit}'
  )
  if [[ -z "$pc_pub_count" ]]; then
    echo "ERROR: could not read /point_cloud publisher count." >&2
    ros2 topic info -v /point_cloud >&2 || true
    exit 1
  fi
  echo "Current /point_cloud Publisher count: $pc_pub_count"
  if (( pc_pub_count != 1 )); then
    cat >&2 <<'MSG'
ERROR: /point_cloud must have exactly 1 publisher for mapping.

Multiple publishers mean the Isaac MID360 setup script was run more than once
in the same Isaac session. ROS-side cleanup cannot remove these Replicator
writers because they live inside the Isaac Sim process.

Fix:
  1. Stop this quickstart.
  2. In Isaac Sim, stop simulation and restart Isaac Sim completely.
  3. Open full_warehouse_r.usd.
  4. Before Play, clean foreign articulations and remove Isaac runtime IMU:
       exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
       exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
       exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
  5. Press Play and run the MID360 setup once:
       exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
  6. Confirm:
       ros2 topic info -v /point_cloud
     It must show: Publisher count: 1

For an emergency test only, rerun quickstart with:
  --allow-multi-point-cloud
MSG
    ros2 topic info -v /point_cloud >&2 || true
    exit 1
  fi
fi
EOF
}

echo "[sim_glim_mapping] Workspace     : $WS"
echo "[sim_glim_mapping] Domain        : $ROS_DOMAIN_ID"
echo "[sim_glim_mapping] RMW           : ${SIM_RMW_IMPLEMENTATION:-unset/default}"
echo "[sim_glim_mapping] CycloneDDS    : ${SIM_CYCLONEDDS_URI:-unset/default}"
echo "[sim_glim_mapping] Scene         : $WS/src/robot_sim/usd/scenes/full_warehouse_r.usd"
echo "[sim_glim_mapping] LiDAR mainline: mid360_like"
echo "[sim_glim_mapping] IMU mainline  : Isaac true IMU"
echo "[sim_glim_mapping] Config        : $GLIM_CONFIG"
echo "[sim_glim_mapping] GLIM GPU      : $GLIM_GPU"
echo "[sim_glim_mapping] Bag           : $BAG_PATH"
echo "[sim_glim_mapping] RViz          : $RVIZ"
echo "[sim_glim_mapping] Joy-Con       : $JOY"
echo "[sim_glim_mapping] Direct cmd_vel: $DIRECT_CMD_VEL"
echo "[sim_glim_mapping] Fake IMU      : $FAKE_IMU"
echo "[sim_glim_mapping] Allow multi /point_cloud: $ALLOW_MULTI_POINT_CLOUD"
echo "[sim_glim_mapping] Record D455    : $RECORD_D455"
echo "[sim_glim_mapping] Session logs  : $SESSION_LOG_DIR"
echo "[sim_glim_mapping] ROS logs      : $ROS_LOG_DIR"

cat > "$SESSION_LOG_DIR/run-info.txt" <<EOF
started_at=$RUN_STAMP
workspace=$WS
ros_domain_id=$ROS_DOMAIN_ID
rmw_implementation=${SIM_RMW_IMPLEMENTATION:-unset/default}
cyclonedds_uri=${SIM_CYCLONEDDS_URI:-unset/default}
scene=$WS/src/robot_sim/usd/scenes/full_warehouse_r.usd
glim_config=$GLIM_CONFIG
glim_gpu=$GLIM_GPU
bag_path=$BAG_PATH
rviz=$RVIZ
joy=$JOY
direct_cmd_vel=$DIRECT_CMD_VEL
fake_imu=$FAKE_IMU
allow_multi_point_cloud=$ALLOW_MULTI_POINT_CLOUD
record_d455=$RECORD_D455
session_log_dir=$SESSION_LOG_DIR
ros_log_dir=$ROS_LOG_DIR
EOF

add_pane "sim_map 1 isaac sensor commands" \
  'cat <<'"'"'EOF'"'"'
Open this scene in Isaac Sim:
  /home/ruan-x/robot_ws/src/robot_sim/usd/scenes/full_warehouse_r.usd

=== BEFORE Play: run these in Isaac Script Editor ===
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())

  D455 mode selected by quickstart: '"$RECORD_D455"'
  If false:
    exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
  If true (--record-d455):
    exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())

  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())

=== Press Play ===

=== AFTER Play: run these in Isaac Script Editor ===
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())

  If D455 mode is true, run once after MID360:
    exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())

Expected topics after Play + MID360 setup:
  /point_cloud
  /livox/imu              (published by Isaac true IMU graph)
  /livox/lidar
  /livox/lidar_pc2_accumulated
  /camera/camera/color/image_raw       (only with --record-d455)
  /camera/camera/depth/image_rect_raw  (only with --record-d455)

Important:
  setup_isaac_livox_imu_ros.py must run BEFORE Play.
  It reads Isaac true IMU through Python ImuSensorInterface and timestamps ROS
  messages with Isaac simulation time.
  cleanup_isaac_mid360_ros.py must run BEFORE Play. Never detach/remove an RTX
  LiDAR writer while Play is running. Run setup_isaac_mid360_like_ros.py exactly
  once AFTER Play.
  If /point_cloud Publisher count becomes 2 or 3, restart Isaac Sim before mapping:
    ros2 topic info -v /point_cloud
  D455 is recording-only and not used by GLIM. Start with 640x360 @ 10 Hz.
EOF'

sleep 1

add_pane "sim_map 2 sim bringup" \
  'ros2 launch robot_sim sim_sensor_check.launch.py \
    start_joy:='"$JOY"' \
    direct_cmd_vel:='"$DIRECT_CMD_VEL"' \
    start_rviz:='"$RVIZ"

sleep 3

if [[ "$FAKE_IMU" == "true" ]]; then
  add_pane "sim_map 3 fake livox imu" \
    'ros2 run robot_sim sim_livox_imu_node --ros-args \
      -p use_sim_time:=true \
      -p topic:=/livox/imu \
      -p frame_id:=livox_frame \
      -p publish_rate:=400.0 \
      -p accel_z:=9.80665 \
      -p use_clock_interpolation:=true \
      -p cmd_vel_topic:=/sim/cmd_vel \
      -p livox_pitch_rad:=0.5236'
  sleep 1
fi

make_glim_pane_body() {
  cat <<EOF
$(wait_for_topic_cmd /point_cloud 45)
$(wait_for_topic_cmd /livox/lidar 45)
$(wait_for_topic_cmd /livox/lidar_pc2_accumulated 45)
$(wait_for_topic_cmd /livox/imu 45)
$(wait_for_tf_cmd base_footprint livox_frame 45)
ALLOW_MULTI_POINT_CLOUD="$ALLOW_MULTI_POINT_CLOUD"
$(assert_single_point_cloud_publisher_cmd)
echo
echo "IMPORTANT: Ctrl-C this GLIM pane to save the map."
echo "Keep this Terminator window open until GLIM prints 'saved' and MAP SAVE OK."
echo "GLIM is running in the foreground so Ctrl-C reaches it directly."
ros2 run glim_ros glim_rosnode --ros-args \
    -p use_sim_time:=true \
    -p config_path:="$GLIM_CONFIG" || true

echo "[sim_glim_mapping] GLIM stopped. Waiting for dump files ..."
sleep 3
if [[ -s /tmp/dump/graph.txt ]]; then
  cat /tmp/dump/graph.txt
  num_submaps=\$(awk '/num_submaps:/ {print \$2; exit}' /tmp/dump/graph.txt)
  traj_lines=\$(wc -l < /tmp/dump/traj_lidar.txt 2>/dev/null || echo 0)
  if [[ "\${num_submaps:-0}" =~ ^[0-9]+$ ]] && (( num_submaps > 0 )) && (( traj_lines > 0 )); then
    saved_map="$WS/maps/sim_glim_dump_${BAG_NAME}_${RUN_STAMP}"
    cp -a /tmp/dump "\$saved_map"
    echo "MAP SAVE OK: \$saved_map"
  else
    echo "MAP SAVE INVALID: num_submaps=\${num_submaps:-missing}, traj_lidar_lines=\$traj_lines" >&2
  fi
else
  echo "MAP SAVE FAILED: /tmp/dump/graph.txt is missing or empty." >&2
  find /tmp/dump -maxdepth 2 -type f -printf "%p %s bytes\\n" 2>/dev/null || true
fi
EOF
}

add_pane "sim_map 4 glim_rosnode" "$(make_glim_pane_body)"

sleep 2

record_topics="
/livox/lidar
/livox/lidar_pc2_accumulated
/point_cloud
/livox/imu
/tf
/tf_static
/robot_description
/clock
/joint_states
/cmd_vel
/sim/target_joint_states
/glim_ros/aligned_points
/glim_ros/odom
/glim_ros/pose
/glim_ros/map
"
if [[ -n "$EXTRA_TOPICS" ]]; then
  record_topics="$record_topics $EXTRA_TOPICS"
fi
if [[ "$RECORD_D455" == "true" ]]; then
  record_topics="$record_topics
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
"
fi
record_topics_args="$(printf '%s' "$record_topics" | tr '\n' ' ')"

make_rosbag_pane_body() {
  cat <<EOF
$(wait_for_topic_cmd /point_cloud 45)
$(wait_for_topic_cmd /livox/lidar 45)
$(wait_for_topic_cmd /livox/lidar_pc2_accumulated 45)
$(wait_for_topic_cmd /livox/imu 45)
$(if [[ "$RECORD_D455" == "true" ]]; then
    wait_for_topic_cmd /camera/camera/color/image_raw 60
    wait_for_topic_cmd /camera/camera/depth/image_rect_raw 60
  fi)
$(wait_for_tf_cmd base_footprint livox_frame 45)
ALLOW_MULTI_POINT_CLOUD="$ALLOW_MULTI_POINT_CLOUD"
$(assert_single_point_cloud_publisher_cmd)
echo "Recording bag to: $BAG_PATH"
echo "Topics:"
printf "%s\n" "$record_topics"
echo "rosbag is running in the foreground so Ctrl-C reaches it directly."
ros2 bag record \
  --output "$BAG_PATH" \
  --compression-mode file \
  --compression-format zstd \
  $record_topics_args || true
echo "[sim_glim_mapping] rosbag stopped. Validating metadata ..."
ros2 bag info "$BAG_PATH" || echo "ROSBAG VALIDATION FAILED: $BAG_PATH" >&2
EOF
}

add_pane "sim_map 5 rosbag record" "$(make_rosbag_pane_body)"

sleep 1

add_pane "sim_map 6 status" \
  'echo "Waiting for topics..."
sleep 5
while true; do
  echo
  date
  echo "--- raw Isaac LiDAR ---"
  timeout 5s ros2 topic info -v /point_cloud 2>/dev/null | sed -n "/Publisher count:/p" || true
  timeout 5s ros2 topic hz /point_cloud --window 10 2>/dev/null || echo "no /point_cloud"
  echo "--- Livox CustomMsg adapter ---"
  timeout 5s ros2 topic hz /livox/lidar --window 10 2>/dev/null || echo "no /livox/lidar"
  echo "--- accumulated RViz PointCloud2 ---"
  timeout 5s ros2 topic hz /livox/lidar_pc2_accumulated --window 5 2>/dev/null || echo "no /livox/lidar_pc2_accumulated"
  echo "--- IMU input ---"
  timeout 5s ros2 topic hz /livox/imu --window 20 2>/dev/null || echo "no /livox/imu"
  echo "--- GLIM output ---"
  timeout 5s ros2 topic hz /glim_ros/aligned_points --window 5 2>/dev/null || echo "no /glim_ros/aligned_points"
  if [[ "'"$RECORD_D455"'" == "true" ]]; then
    echo "--- D455 recording-only side channel ---"
    timeout 5s ros2 topic hz /camera/camera/color/image_raw --window 5 2>/dev/null || echo "no D455 color"
    timeout 5s ros2 topic hz /camera/camera/depth/image_rect_raw --window 5 2>/dev/null || echo "no D455 depth"
  fi
  echo "--- current TF ---"
  timeout 3s ros2 run tf2_ros tf2_echo base_footprint livox_frame 2>/dev/null || echo "no base_footprint->livox_frame"
  echo "--- /tf_static sample ---"
  timeout 3s ros2 topic echo /tf_static --once 2>/dev/null | sed -n "1,35p" || echo "no /tf_static"
  echo "--- recording output ---"
  du -sh "'"$BAG_PATH"'" 2>/dev/null || echo "bag not created yet"
  echo "sleeping 8 sec..."
  sleep 8
done'

launch_terminator_layout

if [[ "${SIM_GLIM_DRY_RUN:-false}" == "true" ]]; then
  echo "[sim_glim_mapping] Dry run complete; Terminator was not opened."
else
  echo "[sim_glim_mapping] Terminator layout opened."
fi
echo
echo "Keep the robot stationary until GLIM aligned_points is stable for 5 seconds."
echo "Then drive slowly with Joy-Con. Keep turns gentle for the first map."
echo "When finished, stop GLIM and rosbag record with Ctrl-C in their terminals."
echo "Bag output: $BAG_PATH"
echo "Debug logs: $SESSION_LOG_DIR"
echo "GLIM dump, if generated: /tmp/dump"
echo "Suggested save path:"
echo "  ls -lah /tmp/dump"
echo "  mkdir -p $WS/maps"
echo "  cp -a /tmp/dump $WS/maps/sim_glim_dump_\$(date +%Y%m%d_%H%M%S)"
echo
echo "If /tmp/dump does not exist, GLIM has not produced a map dump yet."
echo "Check: ros2 topic hz /glim_ros/aligned_points"
