#!/usr/bin/env bash
set -euo pipefail

workspace_dir="/home/matsunaga-h/robot_ws"
output_root="${workspace_dir}/debug-output/touch-anything-bags"
duration_sec="0"
skip_preflight="false"

usage() {
  echo "Usage: $0 [--duration SEC] [--output-root DIR] [--skip-preflight]"
  echo "Records evidence only. It does not publish a motion command."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      duration_sec="$2"
      shift 2
      ;;
    --output-root)
      output_root="$2"
      shift 2
      ;;
    --skip-preflight)
      skip_preflight="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cd "${workspace_dir}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

timestamp="$(date +%Y%m%d_%H%M%S)"
bag_path="${output_root}/touch_regression_${timestamp}"
mkdir -p "${output_root}"

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
  /arm/dashboard_command
  /arm/control_mode
  /arm/hold_command
  /arm/visual_servo_command
  /arm/visual_servo_state
  /arm/candidate_joint_states
  /arm/mount_transform
  /display_planned_path
  /touch_anything/state
  /touch_anything/command
  /touch_anything/tracker_debug
  /touch_anything/control_debug
  /touch_anything/base_lock
  /touch_anything/overlay_image
  /touch_anything/target_point
  /touch_anything/target_pose
  /touch_anything/markers
  /chokudomotor/angle
  /chokudomotor/target_angle
  /cameraswingmotor/angle
  /cameraswingmotor/target_angle
  /robot_mode
  /nav_mode/command
  /cmd_vel
  /lift/position
  /lift/state
  /tf
  /tf_static
  /arm_controller/controller_state
  /arm_controller/follow_joint_trajectory/_action/goal
  /arm_controller/follow_joint_trajectory/_action/result
  /arm_controller/follow_joint_trajectory/_action/feedback
  /arm_controller/follow_joint_trajectory/_action/status
  /diagnostics
  /parameter_events
  /rosout
)

critical_topics=(
  /arm_camera/color/image_raw
  /arm_camera/color/camera_info
  /joint_states
  /arm/dashboard_state
  /arm/hardware_status
  /arm/visual_servo_command
  /arm/visual_servo_state
  /arm/mount_transform
  /arm_controller/controller_state
  /touch_anything/state
  /touch_anything/tracker_debug
  /touch_anything/control_debug
  /tf
  /tf_static
)

if [[ "${skip_preflight}" != "true" ]]; then
  declare -A available_topics=()
  while IFS= read -r topic; do
    available_topics["${topic}"]=1
  done < <(ros2 topic list)
  missing_topics=()
  for topic in "${critical_topics[@]}"; do
    if [[ -z "${available_topics[${topic}]:-}" ]]; then
      missing_topics+=("${topic}")
    fi
  done
  if [[ ${#missing_topics[@]} -gt 0 ]]; then
    echo "Recording preflight failed; required publishers are missing:" >&2
    for topic in "${missing_topics[@]}"; do
      echo "  ${topic}" >&2
    done
    echo "Start the complete Touch/Arm stack, or use --skip-preflight only for a non-motion diagnostic bag." >&2
    exit 3
  fi
fi

echo "Bag output: ${bag_path}"
echo "This command is read-only and sends no Arm/base command."
echo "Stop with Ctrl-C after the Touch run finishes."

record_cmd=(ros2 bag record -o "${bag_path}" --max-cache-size 1073741824 --include-hidden-topics)
record_cmd+=("${topics[@]}")
if [[ "${duration_sec}" == "0" ]]; then
  "${record_cmd[@]}"
else
  timeout --signal=INT --kill-after=10 "${duration_sec}" "${record_cmd[@]}" || status=$?
  if [[ "${status:-0}" -ne 0 && "${status:-0}" -ne 124 ]]; then
    exit "${status}"
  fi
fi

for node in touch_observation_node so101_arm_gateway so101_real_trajectory_server; do
  ros2 param dump "/${node}" >"${bag_path}/${node}_parameters.yaml" 2>"${bag_path}/${node}_parameters.error.log" || true
done

audit_path="${bag_path}/recording_audit.json"
if ! /usr/bin/python3 tools/touch_anything/audit_touch_regression_bag.py \
  "${bag_path}" --json-output "${audit_path}"; then
  echo "Recorded bag was preserved, but its evidence audit failed: ${audit_path}" >&2
  exit 4
fi

echo "Recorded: ${bag_path}"
echo "Audit: ${audit_path}"
