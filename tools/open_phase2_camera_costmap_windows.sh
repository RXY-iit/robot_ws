#!/usr/bin/env bash
# Launch only the camera -> PointCloud2 -> local costmap verification stack.
#
# Default mode is sensor-only: no drive motors, no steer motors, no lift, no
# Nav2 planner/BT/controller motion authority.  Add --camera-motors if you want
# Joy-Con control of the pan/tilt camera base while tuning the view angle.

set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
RVIZ_CONFIG="${RVIZ_CONFIG:-$WS/rviz/phase2_camera_costmap.rviz}"
CLEANUP="${CLEANUP:-false}"
CAMERA_MOTORS="${CAMERA_MOTORS:-false}"
LIDAR="${LIDAR:-true}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --lidar           Start Livox MID360 for camera/LiDAR TF alignment (default).
  --no-lidar        Do not start Livox MID360.
  --camera-motors   Also start serial camera pan/tilt motors and Joy input.
  --joy-dev PATH    Joystick device for --camera-motors (default: $JOY_DEV).
  --rviz PATH       RViz config (default: $RVIZ_CONFIG).
  --cleanup         Kill old phase2/nav2/rviz/camera-costmap related processes.
  -h, --help        Show this help.

What this starts:
  1. robot_bringup sensor-only camera/TF stack
  2. depth_image_proc::PointCloudXyzNode -> /camera/depth/points
  3. controller_server only, for /local_costmap VoxelLayer visualization
  4. optional /livox/lidar for camera/LiDAR TF alignment
  5. RViz focused on LiDAR, RGB, depth points, local costmap, TF, robot model

No drive/steer/lift motors are started unless --camera-motors is used, and
even then only the camera pan/tilt serial stack is enabled.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lidar) LIDAR=true; shift ;;
    --no-lidar) LIDAR=false; shift ;;
    --camera-motors) CAMERA_MOTORS=true; shift ;;
    --joy-dev) JOY_DEV="$2"; shift 2 ;;
    --rviz) RVIZ_CONFIG="$2"; shift 2 ;;
    --cleanup) CLEANUP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

[[ -f "$WS/install/setup.bash" ]] || { echo "Missing $WS/install/setup.bash" >&2; exit 1; }
[[ -f "$RVIZ_CONFIG" ]] || { echo "Missing RViz config: $RVIZ_CONFIG" >&2; exit 1; }
command -v gnome-terminal >/dev/null 2>&1 || { echo "gnome-terminal not found" >&2; exit 1; }

if [[ "$CAMERA_MOTORS" == "true" ]]; then
  JOY_ARG="true"
  CAMERA_MOTOR_JOY_ARG="true"
else
  JOY_ARG="false"
  CAMERA_MOTOR_JOY_ARG="false"
fi

if [[ "$CLEANUP" == "true" ]]; then
  echo "[phase2-camera] Cleaning old processes..."
  pkill -f "test_all.launch.py" || true
  pkill -f "phase2_camera_costmap.launch.py" || true
  pkill -f "navigation.launch.py" || true
  pkill -f "depth_proc_container" || true
  pkill -f "livox_lidar_publisher" || true
  pkill -f "livox_ros_driver2" || true
  pkill -f "point_cloud_xyz_node" || true
  pkill -f "controller_server" || true
  pkill -f "lifecycle_manager_phase2_camera_costmap" || true
  pkill -f "rviz2" || true
  ros2 daemon stop || true
  ros2 daemon start || true
  sleep 1
fi

common_setup='
set +u
conda deactivate >/dev/null 2>&1 || true
source /opt/ros/humble/setup.bash
source "'"$WS"'/install/setup.bash"
set -u
cd "'"$WS"'"
'

open_window() {
  local title="$1"
  local body="$2"
  gnome-terminal --title="$title" -- bash -lc "$common_setup
echo '===== $title ====='
echo 'camera_motors: $CAMERA_MOTORS'
echo 'lidar        : $LIDAR'
echo 'rviz_config  : $RVIZ_CONFIG'
$body
echo
echo '[window finished] press Enter to close'
read -r
"
}

echo "[phase2-camera] camera_motors: $CAMERA_MOTORS"
echo "[phase2-camera] lidar        : $LIDAR"
echo "[phase2-camera] rviz_config  : $RVIZ_CONFIG"

open_window "phase2 1 camera tf" '
ros2 launch robot_bringup test_all.launch.py \
  lidar:='"$LIDAR"' \
  camera:=true \
  drive_motors:=false \
  steer_motors:=false \
  serial_motors:='"$CAMERA_MOTORS"' \
  joy:='"$JOY_ARG"' \
  joy_dev:="'"$JOY_DEV"'" \
  camera_motor_joy:='"$CAMERA_MOTOR_JOY_ARG"' \
  lift:=false \
  rviz:=false \
  static_odom:=true \
  use_glim_loc:=false'

sleep 2

open_window "phase2 2 local costmap" '
echo "Waiting 4 sec for camera topics and TF..."
sleep 4
ros2 launch nav_pkg phase2_camera_costmap.launch.py'

sleep 1

open_window "phase2 3 monitor" '
echo "Waiting 8 sec for local costmap..."
sleep 8
while true; do
  clear
  date
  echo
  echo "--- nodes ---"
  ros2 node list | grep -E "camera|livox|lidar|robot_state_publisher|point_cloud_xyz|depth_proc|controller_server|local_costmap|lifecycle_manager_phase2" || true
  echo
  echo "--- /camera/depth/points ---"
  ros2 topic info -v /camera/depth/points 2>/dev/null | grep -E "Publisher count|Subscription count|Node name|Reliability|Durability" || true
  echo
  echo "--- /local_costmap/costmap ---"
  ros2 topic info -v /local_costmap/costmap 2>/dev/null | grep -E "Publisher count|Subscription count|Node name" || true
  echo
  echo "--- lifecycle ---"
  ros2 lifecycle get /controller_server 2>/dev/null || true
  echo
  echo "--- latest point cloud frame ---"
  timeout 2 ros2 topic echo /camera/depth/points --once --field header 2>/dev/null || echo "no point cloud yet"
  echo
  echo "--- /livox/lidar ---"
  ros2 topic info -v /livox/lidar 2>/dev/null | grep -E "Publisher count|Subscription count|Node name|Reliability|Durability" || true
  timeout 2 ros2 topic echo /livox/lidar --once --field header 2>/dev/null || echo "no lidar point cloud yet"
  echo
  echo "--- sensor TF quick check ---"
  timeout 2 ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame 2>/dev/null | head -n 8 || true
  timeout 2 ros2 run tf2_ros tf2_echo base_link livox_frame 2>/dev/null | head -n 8 || true
  sleep 2
done'

sleep 1

open_window "phase2 4 rviz" '
echo "Waiting 5 sec..."
sleep 5
rviz2 -d "'"$RVIZ_CONFIG"'"'

echo "[phase2-camera] All windows requested."
