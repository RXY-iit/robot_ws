#!/usr/bin/env bash
# Check Phase 1 Nav2 graph health before sending a real robot goal.

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros2_cli_logs}"

set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

echo "=== Nav2 Phase 1 graph check ==="
echo "WS: $WS"
echo

echo "--- nodes ---"
nodes="$(ros2 node list 2>/dev/null || true)"
if [[ -z "$nodes" ]]; then
  echo "NO ROS nodes visible. Check ROS_DOMAIN_ID, launch status, or crashed terminals."
else
  echo "$nodes" | sort
fi
echo

need_node() {
  local n="$1"
  if echo "$nodes" | grep -qx "$n"; then
    echo "OK node: $n"
  else
    echo "MISSING node: $n"
  fi
}

need_node /map_server
need_node /planner_server
need_node /controller_server
need_node /behavior_server
need_node /bt_navigator
need_node /nav_mode_switch_node
need_node /cmd_vel_safety_node
echo

echo "--- lifecycle ---"
for n in /map_server /planner_server /controller_server /behavior_server /bt_navigator; do
  printf "%-22s " "$n"
  ros2 lifecycle get "$n" 2>/dev/null || echo "not found"
done
echo

echo "--- critical topics ---"
for t in /map /scan /goal_pose /plan /local_costmap/costmap /global_costmap/costmap /nav2/cmd_vel /teleop/cmd_vel /cmd_vel_raw /cmd_vel /safety_status /robot_mode /wheel_odom; do
  printf "%-30s " "$t"
  ros2 topic info "$t" 2>/dev/null || echo "missing"
done
for t in /tf /tf_static; do
  printf "%-30s " "$t"
  ros2 topic info "$t" 2>/dev/null || echo "missing"
done
echo

echo "--- /cmd_vel authority ---"
ros2 topic info -v /cmd_vel 2>/dev/null | sed -n '/Publisher count/,$p' || echo "/cmd_vel missing"
echo

echo "--- /cmd_vel_raw authority ---"
ros2 topic info -v /cmd_vel_raw 2>/dev/null | sed -n '/Publisher count/,$p' || echo "/cmd_vel_raw missing"
echo

echo "--- TF quick check ---"
timeout 8s ros2 run tf2_ros tf2_echo map odom >/tmp/nav2_tf_map_odom.txt 2>&1 || true
grep -q "At time" /tmp/nav2_tf_map_odom.txt \
  && echo "OK TF: map -> odom" \
  || echo "MISSING TF: map -> odom"
timeout 8s ros2 run tf2_ros tf2_echo odom base_footprint >/tmp/nav2_tf_odom_base.txt 2>&1 || true
grep -q "At time" /tmp/nav2_tf_odom_base.txt \
  && echo "OK TF: odom -> base_footprint" \
  || echo "MISSING TF: odom -> base_footprint"
timeout 8s ros2 run tf2_ros tf2_echo base_footprint base_link >/tmp/nav2_tf_base_link.txt 2>&1 || true
grep -q "At time" /tmp/nav2_tf_base_link.txt \
  && echo "OK TF: base_footprint -> base_link" \
  || echo "MISSING TF: base_footprint -> base_link"
echo

echo "Expected before RViz 2D Goal Pose:"
echo "  - map_server/planner_server/controller_server/behavior_server/bt_navigator are active"
echo "  - /map exists"
echo "  - /goal_pose exists after bt_navigator starts"
echo "  - /plan appears after a valid goal"
echo "  - /nav2/cmd_vel may be silent until a valid goal is actively followed"
echo "  - /cmd_vel_raw should list only nav_mode_switch_node as publisher"
echo "  - /cmd_vel should list only cmd_vel_safety_node as publisher"
echo "  - /teleop/cmd_vel should list teleop_twist_joy_node as publisher"
echo "  - /safety_status should come from cmd_vel_safety_node"
echo
echo "If bt_navigator is inactive:"
echo "  ros2 lifecycle set /bt_navigator activate"
echo
echo "If TF is missing in phase1-safe:"
echo "  /tf_static should contain base_footprint -> base_link from robot_state_publisher"
echo "  /tf_static should contain odom -> base_footprint from odom_to_base_static"
echo "  map -> odom must come from gicp_localizer or a temporary static TF"
