#!/usr/bin/env bash
# record_semantic_bag.sh — Record a rosbag suitable for future semantic map construction.
#
# tools/record_semantic_bag.sh --start-phase2-loc
# Prerequisites before recording:
#   - Full robot bringup running (test_all.launch.py)
#   - Camera at INITIAL POSITION: pan=275.0°, tilt=67.0° (do NOT pan/tilt during recording)
#   - Phase 2 FAST-LIO2 + GICP localization running
#     (for /fast_lio/odometry, /gicp_loc/pose, /gicp_loc/score and map→odom TF)
#   - Robot moves through all target areas for semantic labelling
#
# Recorded topics (beyond geometry):
#   /camera/camera/color/image_raw          RGB 640×480 @ 30 Hz  — object detection
#   /camera/camera/color/camera_info        intrinsics (constant)
#   /camera/camera/depth/image_rect_raw     depth image @ 30 Hz
#   /camera/camera/depth/camera_info        depth intrinsics
#   /chokudomotor/angle                     camera pan state (fixed during recording)
#   /cameraswingmotor/angle                 camera tilt state (fixed during recording)
#
# Camera pose assumption:
#   Pan=275.0° tilt=67.0° → camera_link pose relative to base_link is FIXED.
#   The static transform in URDF (base_link → camera_link) must match this angle.
#   If camera is truly stationary, it can be treated as a fixed extrinsic for
#   LiDAR-camera fusion (semantic 3D projection).
#
# Output:
#   slam_bag/semantic_YYYYMMDD_HHMMSS/
#
set -euo pipefail

WS="${WS:-/home/matsunaga-h/robot_ws}"
BAG_DIR="${BAG_DIR:-$WS/slam_bag}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT="$BAG_DIR/semantic_$TIMESTAMP"
START_PHASE2_LOC="${START_PHASE2_LOC:-false}"
LOC_NO_BRINGUP="${LOC_NO_BRINGUP:-false}"
LOC_CLEANUP="${LOC_CLEANUP:-false}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Record a full-sensor rosbag for semantic map construction.
Camera must be at initial position (pan=275°, tilt=67°) and stationary.

Options:
  --output DIR   Output bag directory (default: slam_bag/semantic_TIMESTAMP)
  --start-phase2-loc
                 Start Phase 2 FAST-LIO2 + GICP localization before recording
  --loc-no-bringup
                 With --start-phase2-loc, do not start robot_bringup again
  --loc-cleanup  With --start-phase2-loc, cleanup old localization processes first
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) OUTPUT="$2"; shift 2 ;;
    --start-phase2-loc) START_PHASE2_LOC=true; shift ;;
    --loc-no-bringup) LOC_NO_BRINGUP=true; shift ;;
    --loc-cleanup) LOC_CLEANUP=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "$BAG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

if [[ "$START_PHASE2_LOC" == "true" ]]; then
  loc_cmd=(tools/open_fast_lio2_loc_terminals.sh --mode real --no-rviz --no-overlay)
  if [[ "$LOC_NO_BRINGUP" == "true" ]]; then
    loc_cmd+=(--no-bringup)
  fi
  if [[ "$LOC_CLEANUP" == "true" ]]; then
    loc_cmd+=(--cleanup)
  fi
  echo "[record_semantic_bag] Starting Phase 2 localization:"
  echo "  ${loc_cmd[*]}"
  (cd "$WS" && "${loc_cmd[@]}")
  echo "[record_semantic_bag] Waiting 12 sec for localization startup..."
  sleep 12
fi

echo ""
echo "[record_semantic_bag] Output: $OUTPUT"
echo ""
echo "  BEFORE STARTING: verify camera is at initial position"
echo "    pan=275.0°  (/chokudomotor/angle)"
echo "    tilt=67.0°  (/cameraswingmotor/angle)"
echo ""
echo "  Topics to be recorded:"
echo "    Geometry:    /livox/lidar(CustomMsg)  /livox/lidar_pc2(PointCloud2)  /livox/imu  /wheel_odom"
echo "    TF:          /tf  /tf_static  /robot_description"
echo "    RGB:         /camera/camera/color/image_raw  /camera/camera/color/camera_info"
echo "    Depth:       /camera/camera/depth/image_rect_raw  /camera/camera/depth/camera_info"
echo "    Camera state:/chokudomotor/angle  /cameraswingmotor/angle"
echo "    Localization:/gicp_loc/pose  /gicp_loc/score  /fast_lio/odometry"
echo "    FAST-LIO2:   /fast_lio/cloud_registered  /fast_lio/path"
echo ""
echo "Press ENTER to start recording, Ctrl+C to stop."
read -r

ros2 bag record \
  --output "$OUTPUT" \
  --compression-mode file \
  --compression-format zstd \
  \
  /livox/lidar \
  /livox/lidar_pc2 \
  /livox/imu \
  /wheel_odom \
  /tf \
  /tf_static \
  /robot_description \
  \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  \
  /chokudomotor/angle \
  /cameraswingmotor/angle \
  \
  /gicp_loc/pose \
  /gicp_loc/score \
  /fast_lio/odometry \
  /fast_lio/cloud_registered \
  /fast_lio/path \
  /scan \
  /map

echo ""
echo "[record_semantic_bag] Bag saved to: $OUTPUT"
echo ""
echo "For semantic mapping, this bag provides:"
echo "  - 3D geometry:   /livox/lidar (CustomMsg for FAST-LIO) and /livox/lidar_pc2 (PointCloud2 for RViz/GICP)"
echo "  - IMU:           /livox/imu    (for FAST-LIO2 offline reprocessing)"
echo "  - Robot pose:    /gicp_loc/pose or TF map→base_footprint"
echo "  - Loc quality:   /gicp_loc/score and /fast_lio/odometry"
echo "  - RGB frames:    /camera/camera/color/image_raw"
echo "    → run YOLO/SAM offline on these to get 2D bounding boxes"
echo "    → back-project using depth + camera pose → 3D object positions"
echo "  - Camera pose:   fixed (pan=275° tilt=67°) → use URDF extrinsic"
