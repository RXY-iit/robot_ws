#!/usr/bin/env bash
set -euo pipefail

workspace_dir="/home/matsunaga-h/robot_ws"
report_dir="${workspace_dir}/debug-output/touch-anything-regression"
timestamp="$(date +%Y%m%d_%H%M%S)"
report_path="${report_dir}/offline_gate_${timestamp}.json"

mkdir -p "${report_dir}"
cd "${workspace_dir}"

echo "[1/2] Running pure algorithm and Arm safety tests (no ROS hardware commands)..."
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/touch_anything/test src/so101_arm_control/test

echo "[2/2] Auditing the representative real evidence manifest..."
/usr/bin/python3 tools/touch_anything/offline_regression.py \
  --manifest tools/touch_anything/regression_manifest.yaml \
  --json-output "${report_path}"

echo "Offline gate passed. Report: ${report_path}"
