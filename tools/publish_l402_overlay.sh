#!/usr/bin/env bash
# Publish the saved L402 PLY map overlay for RViz.
#
# Usage:
#   tools/publish_l402_overlay.sh live
#   tools/publish_l402_overlay.sh real
#   tools/publish_l402_overlay.sh bag

set -euo pipefail

MODE="${1:-live}"
WS="${WS:-/home/matsunaga-h/robot_ws}"
PLY_MAP="${PLY_MAP:-$WS/maps/saved-map/map-l402-0503/l402_points_0503}"

case "$MODE" in
  live|real) USE_SIM_TIME=false ;;
  bag)  USE_SIM_TIME=true ;;
  *)
    echo "Usage: $(basename "$0") [live|real|bag]" >&2
    exit 2
    ;;
esac

cd "$WS"
conda deactivate 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

cmd=(/usr/bin/python3 tools/publish_ply_pointcloud.py
  "$PLY_MAP" \
  --topic /l402_glim_points \
  --frame-id map)

if [[ "$USE_SIM_TIME" == "true" ]]; then
  cmd+=(--use-sim-time)
fi

exec "${cmd[@]}"
