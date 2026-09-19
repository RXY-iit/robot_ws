#!/usr/bin/env bash
set -euo pipefail

workspace_dir="/home/matsunaga-h/robot_ws"
offline_domain_id="${TOUCH_OFFLINE_DOMAIN_ID:-113}"
rate="1.0"
drop_tf="false"
drop_arm_camera="false"

usage() {
  echo "Usage: $0 BAG_DIR [--rate N] [--drop-tf] [--drop-arm-camera]"
  echo "Runs on an isolated ROS domain and remaps every actuator-facing output."
}

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi
bag_path="$1"
shift
while [[ $# -gt 0 ]]; do
  case "$1" in
    --rate) rate="$2"; shift 2 ;;
    --drop-tf) drop_tf="true"; shift ;;
    --drop-arm-camera) drop_arm_camera="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done
if [[ ! -f "${bag_path}/metadata.yaml" ]]; then
  echo "Not a rosbag2 directory: ${bag_path}" >&2
  exit 2
fi

cd "${workspace_dir}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${offline_domain_id}"
export ROS_LOG_DIR="/tmp/touch_anything_replay_logs"

node_pid=""
cleanup() {
  if [[ -n "${node_pid}" ]] && kill -0 "${node_pid}" 2>/dev/null; then
    kill -INT "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Offline replay domain: ${ROS_DOMAIN_ID}"
echo "All Arm, Pan/Tilt and navigation outputs are remapped below /offline_sink."
ros2 run touch_anything touch_observation_node.py --ros-args \
  --params-file src/touch_anything/config/touch_anything.yaml \
  -r /arm/dashboard_command:=/offline_sink/arm/dashboard_command \
  -r /nav_mode/command:=/offline_sink/nav_mode/command \
  -r /chokudomotor/target_angle:=/offline_sink/chokudomotor/target_angle \
  -r /cameraswingmotor/target_angle:=/offline_sink/cameraswingmotor/target_angle \
  -r /touch_anything/base_lock:=/offline_sink/touch_anything/base_lock \
  >"/tmp/touch_anything_replay_node.log" 2>&1 &
node_pid=$!

topics=(
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /camera/camera/color/camera_info
  /arm_camera/color/image_raw
  /arm_camera/color/camera_info
  /joint_states
  /arm/visual_joint_states
  /arm/dashboard_state
  /arm/hardware_status
  /touch_anything/command
  /chokudomotor/angle
  /cameraswingmotor/angle
  /robot_mode
  /cmd_vel
  /lift/position
  /lift/state
  /tf
  /tf_static
  /arm_controller/controller_state
)
if [[ "${drop_tf}" == "true" ]]; then
  filtered=()
  for topic in "${topics[@]}"; do
    [[ "${topic}" == "/tf" || "${topic}" == "/tf_static" ]] || filtered+=("${topic}")
  done
  topics=("${filtered[@]}")
fi
if [[ "${drop_arm_camera}" == "true" ]]; then
  filtered=()
  for topic in "${topics[@]}"; do
    [[ "${topic}" == /arm_camera/* ]] || filtered+=("${topic}")
  done
  topics=("${filtered[@]}")
fi

sleep 1
ros2 bag play "${bag_path}" --clock 50 --rate "${rate}" \
  --disable-keyboard-controls --topics "${topics[@]}"

echo "Replay completed. Touch-node log: /tmp/touch_anything_replay_node.log"
echo "This validates recorded perception/state timing only; it does not reproduce real mechanics."
