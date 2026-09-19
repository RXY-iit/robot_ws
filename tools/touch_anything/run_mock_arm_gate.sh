#!/usr/bin/env bash
set -euo pipefail

workspace_dir="/home/matsunaga-h/robot_ws"
offline_domain_id="${TOUCH_OFFLINE_DOMAIN_ID:-113}"
log_root="${workspace_dir}/debug-output/touch-anything-regression/mock_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${log_root}"

cd "${workspace_dir}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${offline_domain_id}"
export ROS_LOG_DIR="${log_root}/ros"
export TOUCH_MOCK_REPORT_PATH="${log_root}/readiness.json"

launch_pid=""
cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "${launch_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${launch_pid}" 2>/dev/null; then
      kill -TERM "${launch_pid}" 2>/dev/null || true
    fi
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting MOCK-only Arm gateway on isolated ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
echo "The mock server contains no USB or real-hardware path."
ros2 launch so101_arm_control arm_mock_dashboard.launch.py rviz:=false dashboard:=false \
  >"${log_root}/launch.log" 2>&1 &
launch_pid=$!

sleep 2
ros2 run so101_arm_control mock_smoke_test | tee "${log_root}/run.log"
echo "Mock Arm gate passed. Report: ${TOUCH_MOCK_REPORT_PATH}"
