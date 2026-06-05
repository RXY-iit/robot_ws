#!/usr/bin/env bash
# Open terminals for Isaac Sim GLIM mapping and rosbag recording.
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
ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_sim_glim_mapping_logs}"
BAG_NAME="${BAG_NAME:-}"
BAG_DIR="${BAG_DIR:-$WS/sim_bag}"
CLEANUP="${CLEANUP:-false}"
RVIZ="${RVIZ:-true}"
JOY="${JOY:-true}"
DIRECT_CMD_VEL="${DIRECT_CMD_VEL:-false}"
FAKE_IMU="${FAKE_IMU:-true}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"
ALLOW_MULTI_POINT_CLOUD="${ALLOW_MULTI_POINT_CLOUD:-false}"

usage() {
  cat <<EOF
Usage: $(basename "$0") --bag NAME [options]

Open terminals for Isaac Sim GLIM mapping in full_warehouse_r.usd.

Options:
  --bag NAME       Bag output name under $BAG_DIR.
  --bag-dir DIR    Bag output parent directory (default: $BAG_DIR)
  --config DIR     GLIM config directory (default: $GLIM_CONFIG)
  --gpu            Use $WS/glim_config/sim GPU config root
  --no-rviz        Do not open RViz from sim_sensor_check.launch.py
  --no-joy         Do not start Joy-Con teleop chain
  --direct-cmd-vel Bypass mode/safety chain and drive sim from /sim/cmd_vel
  --no-fake-imu    Do not start robot_sim sim_livox_imu_node
  --fake-imu       Start corrected ROS-side fake IMU fallback (default)
  --allow-multi-point-cloud
                  Do not fail when /point_cloud has multiple Isaac publishers
  --cleanup        Kill old sim mapping/record/rviz processes first
  -h, --help       Show this help

Environment overrides:
  WS, GLIM_CONFIG, GLIM_GPU, ROS_DOMAIN_ID, SIM_RMW_IMPLEMENTATION,
  SIM_CYCLONEDDS_URI, ROS_LOG_DIR, BAG_NAME, BAG_DIR, CLEANUP, RVIZ, JOY,
  DIRECT_CMD_VEL, FAKE_IMU, EXTRA_TOPICS, ALLOW_MULTI_POINT_CLOUD

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

  [Press Play]

  [AFTER Play]
  exec(open("$WS/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
  exec(open("$WS/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
  # Optional later, after GLIM is stable:
  # exec(open("$WS/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
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

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal not found. Run with --help and start the listed commands manually." >&2
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
  echo "[sim_glim_mapping] Cleaning old processes."
  pkill -f "glim_rosnode" || true
  pkill -f "ros2 bag record" || true
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
pkill -f "glim_rosnode" || true
pkill -f "sim_livox_imu_node" || true

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
echo "[sim_glim_mapping] IMU mainline  : corrected ROS fake IMU (Isaac true IMU is experimental)"
echo "[sim_glim_mapping] Config        : $GLIM_CONFIG"
echo "[sim_glim_mapping] GLIM GPU      : $GLIM_GPU"
echo "[sim_glim_mapping] Bag           : $BAG_PATH"
echo "[sim_glim_mapping] RViz          : $RVIZ"
echo "[sim_glim_mapping] Joy-Con       : $JOY"
echo "[sim_glim_mapping] Direct cmd_vel: $DIRECT_CMD_VEL"
echo "[sim_glim_mapping] Fake IMU      : $FAKE_IMU"
echo "[sim_glim_mapping] Allow multi /point_cloud: $ALLOW_MULTI_POINT_CLOUD"

open_terminal "sim_map 1 isaac sensor commands" \
  'cat <<'"'"'EOF'"'"'
Open this scene in Isaac Sim:
  /home/ruan-x/robot_ws/src/robot_sim/usd/scenes/full_warehouse_r.usd

=== BEFORE Play: run these in Isaac Script Editor ===
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())

=== Press Play ===

=== AFTER Play: run these in Isaac Script Editor ===
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())

  # Optional later, after LiDAR mapping is stable:
  # exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())

Expected topics after Play + MID360 setup:
  /point_cloud
  /livox/imu              (published by corrected ROS fake IMU terminal)
  /livox/lidar
  /livox/lidar_pc2_accumulated

Important:
  Do not run setup_isaac_livox_imu_ros.py in the default mapping flow.
  Isaac physics IMU currently triggers PhysX getVelocities (2,6) in this scene.
  The corrected ROS fake IMU keeps GLIM bringup stable while LiDAR mapping is debugged.
  If /point_cloud Publisher count becomes 2 or 3, restart Isaac Sim before mapping:
    ros2 topic info -v /point_cloud
EOF'

sleep 1

open_terminal "sim_map 2 sim bringup" \
  'ros2 launch robot_sim sim_sensor_check.launch.py \
    start_joy:='"$JOY"' \
    direct_cmd_vel:='"$DIRECT_CMD_VEL"' \
    start_rviz:='"$RVIZ"

sleep 3

if [[ "$FAKE_IMU" == "true" ]]; then
  open_terminal "sim_map 3 fake livox imu" \
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

open_terminal "sim_map 4 glim_rosnode" \
  "$(wait_for_topic_cmd /point_cloud 45)
$(wait_for_topic_cmd /livox/lidar 45)
$(wait_for_topic_cmd /livox/lidar_pc2_accumulated 45)
$(wait_for_topic_cmd /livox/imu 45)
$(wait_for_tf_cmd base_footprint livox_frame 45)
ALLOW_MULTI_POINT_CLOUD=\"$ALLOW_MULTI_POINT_CLOUD\"
$(assert_single_point_cloud_publisher_cmd)
ros2 run glim_ros glim_rosnode --ros-args \
    -p use_sim_time:=true \
    -p config_path:=\"$GLIM_CONFIG\" 2>&1 | tee \"$ROS_LOG_DIR/glim_rosnode_${BAG_NAME}.log\""

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
"
if [[ -n "$EXTRA_TOPICS" ]]; then
  record_topics="$record_topics $EXTRA_TOPICS"
fi

open_terminal "sim_map 5 rosbag record" \
  "$(wait_for_topic_cmd /point_cloud 45)
$(wait_for_topic_cmd /livox/lidar 45)
$(wait_for_topic_cmd /livox/lidar_pc2_accumulated 45)
$(wait_for_topic_cmd /livox/imu 45)
$(wait_for_tf_cmd base_footprint livox_frame 45)
ALLOW_MULTI_POINT_CLOUD=\"$ALLOW_MULTI_POINT_CLOUD\"
$(assert_single_point_cloud_publisher_cmd)
echo \"Recording bag to: $BAG_PATH\"
echo "Topics:"
printf \"%s\n\" \"$record_topics\"
ros2 bag record \
  --output \"$BAG_PATH\" \
  --compression-mode file \
  --compression-format zstd \
  $record_topics 2>&1 | tee \"$ROS_LOG_DIR/rosbag_record_${BAG_NAME}.log\""

sleep 1

open_terminal "sim_map 6 status" \
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
  echo "--- current TF ---"
  timeout 3s ros2 run tf2_ros tf2_echo base_footprint livox_frame 2>/dev/null || echo "no base_footprint->livox_frame"
  echo "--- /tf_static sample ---"
  timeout 3s ros2 topic echo /tf_static --once 2>/dev/null | sed -n "1,35p" || echo "no /tf_static"
  echo "--- recording output ---"
  du -sh "'"$BAG_PATH"'" 2>/dev/null || echo "bag not created yet"
  echo "sleeping 8 sec..."
  sleep 8
done'

echo "[sim_glim_mapping] All terminals opened."
echo
echo "Drive slowly with Joy-Con. Keep turns gentle for the first map."
echo "When finished, stop GLIM and rosbag record with Ctrl-C in their terminals."
echo "Bag output: $BAG_PATH"
echo "GLIM dump, if generated: /tmp/dump"
echo "Suggested save path:"
echo "  ls -lah /tmp/dump"
echo "  mkdir -p $WS/maps"
echo "  cp -a /tmp/dump $WS/maps/sim_glim_dump_\$(date +%Y%m%d_%H%M%S)"
echo
echo "If /tmp/dump does not exist, GLIM has not produced a map dump yet."
echo "Check: ros2 topic hz /glim_ros/aligned_points"
