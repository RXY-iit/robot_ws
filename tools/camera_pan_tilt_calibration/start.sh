#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${tool_dir}/../.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-13}"
export ROS_LOG_DIR="/tmp/camera_pan_tilt_calibration_logs"
mkdir -p "${ROS_LOG_DIR}" "${tool_dir}/output"

if ! ros2 pkg prefix camera_pan_tilt_calibration >/dev/null 2>&1; then
  echo "camera_pan_tilt_calibration is not built. Run:" >&2
  echo "  cd ${workspace_dir}" >&2
  echo "  colcon build --packages-select camera_pan_tilt_calibration --symlink-install" >&2
  exit 2
fi

exec ros2 launch camera_pan_tilt_calibration calibration_capture.launch.py \
  output_root:="${tool_dir}/output" "$@"
