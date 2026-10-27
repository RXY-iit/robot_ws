#!/usr/bin/env bash
set -eo pipefail

cd /home/ruan-x/robot_ws

# Avoid library conflicts from Conda, Isaac Sim, or another ROS overlay.
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset PYTHONHOME
unset LD_PRELOAD
unset CYCLONEDDS_URI

source /opt/ros/humble/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
# Match the Isaac/Quick Start stack even when the parent shell uses CycloneDDS.
# Mixed-RMW parameter/action requests are not validated for this sim deployment.
if [[ -n "${RMW_IMPLEMENTATION:-}" && "$RMW_IMPLEMENTATION" != "rmw_fastrtps_cpp" ]]; then
  printf '[launcher] Sim stack requires FastDDS; replacing inherited RMW_IMPLEMENTATION=%s\n' "$RMW_IMPLEMENTATION"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROBOT_MAP_ID="${ROBOT_MAP_ID:-sim_real_lab_001}"
export JEV_API_KEY="${JEV_API_KEY:-}"
export JEV_API_URL="${JEV_API_URL:-https://api.typesafe.ai/v1/systemone}"
export JEV_MODEL="${JEV_MODEL:-jev-latest}"
export JEV_TIMEOUT_SEC="${JEV_TIMEOUT_SEC:-3.0}"
# dashboard_node currently relies on the user-site aiohttp package on this host.
# Do not hide user site here; the Nav2/Touch launch scripts still isolate
# PYTHONNOUSERSITE where cv_bridge is involved.
unset PYTHONNOUSERSITE

# Set by tools/sim_robot_quick_start.py from its ROS mode (nav2 | arm_test) so
# the Recovery Agent ignores components that mode does not start.
export RECOVERY_PROFILE="${RECOVERY_PROFILE:-}"
# Map PLY used by the Recovery Agent's auto_initial_align (localization recovery).
export RECOVERY_MAP_PLY="${RECOVERY_MAP_PLY:-}"

launch_args=(
  mode:=sim
  map_id:="$ROBOT_MAP_ID"
  robot_ws:=/home/ruan-x/robot_ws
)
if [[ -n "$RECOVERY_PROFILE" ]]; then
  launch_args+=(recovery_profile:="$RECOVERY_PROFILE")
fi
if [[ -n "$RECOVERY_MAP_PLY" ]]; then
  launch_args+=(recovery_map_ply:="$RECOVERY_MAP_PLY")
fi

/usr/bin/python3 tools/sim_startup_preflight.py --workspace "$PWD" --domain "$ROS_DOMAIN_ID" --starting agent
exec /usr/bin/python3 tools/agent_stack_guard.py --run ros2 launch robot_agent agent_dashboard.launch.py "${launch_args[@]}"
