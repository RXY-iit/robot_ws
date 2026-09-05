#!/usr/bin/env bash

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
MAP_ID="${ROBOT_MAP_ID:-l402_2d_map_clean_0509}"
DOMAIN_ID="${ROS_DOMAIN_ID:-13}"

if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Missing $WS/install/setup.bash" >&2
  echo "Build robot_agent and robot_skills first; see todo-my/sim2real-agent-img/migration-and-test-guide.md" >&2
  exit 1
fi

set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID="$DOMAIN_ID"
export ROBOT_WS="$WS"
export ROBOT_MAP_ID="$MAP_ID"

cd "$WS"
echo "Starting real agent/dashboard: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROBOT_MAP_ID=$ROBOT_MAP_ID"
exec ros2 launch robot_agent agent_dashboard.launch.py \
  mode:=real \
  map_id:="$ROBOT_MAP_ID" \
  robot_ws:="$ROBOT_WS" \
  enable_foxglove:=false \
  enable_foxglove_ui:=false \
  enable_camera_compressed_relay:=false
