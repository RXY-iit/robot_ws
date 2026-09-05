# Camera pan/tilt + SO101 wrist-camera ChArUco capture tool

This tool starts only:

- Intel RealSense D435 color, raw depth and aligned-depth streams;
- the deployed SO101 Sonix wrist camera at 640 x 480 / 15 FPS;
- the deployed OpenRB-150 pan/tilt serial bridge;
- the SO101 follower reader in torque-off `real2sim` mode, publishing measured
  `/joint_states` (the Leader arm is not started);
- `joy_node` and the existing camera-only Joy controller;
- the read-only capture node;
- RViz with a dedicated `ChArUco Capture` panel.

It does **not** start drive/steer motors, teleop twist, `/cmd_vel`, Nav2, LiDAR,
Agent or Lift. At launch, pan/tilt are intentionally commanded to the reference
pose `267 deg / 102 deg`; keep the mechanism clear. The SO101 reader requires
all follower torques to be OFF at startup and does not intentionally move it.
Because this launch must exclusively own the D435, it requests the RealSense
driver's startup device reset to clear the intermittently observed stale depth
stream state. Do not run it beside another RealSense process.

## 1. Board contract

The default detector is exactly:

| Field | Value |
|---|---:|
| Type | ChArUco |
| Squares | 7 x 5 |
| Square length | 33.8 mm measured |
| Marker length | 16.5 mm measured |
| Dictionary | `DICT_5X5_100` |

Generate the matching image:

```bash
cd /home/matsunaga-h/robot_ws
python3 tools/camera_pan_tilt_calibration/generate_charuco_board.py
```

The deployed print was physically measured as 33.8 mm per complete cell and
16.5 mm per coded marker. These measured dimensions, rather than the PNG's
nominal scale, are the capture defaults. A replacement print must be measured
again and its values passed explicitly at launch.

OpenCV uses `square_length_m` for one complete chessboard cell and
`marker_length_m` for the smaller coded ArUco marker inside a white cell. The
marker must be smaller than the cell. Measure both physical lengths; do not
substitute the black chessboard-cell width for the inner marker width.

## 2. Build

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select camera_pan_tilt_calibration --symlink-install
source install/setup.bash
```

## 3. Start

Do not run this beside the normal full bringup; both would compete for the same
RealSense, wrist camera, Joy device and OpenRB serial port.

```bash
cd /home/matsunaga-h/robot_ws
export ROS_DOMAIN_ID=13
tools/camera_pan_tilt_calibration/start.sh
```

For a different Joy device or physically measured board dimensions:

```bash
tools/camera_pan_tilt_calibration/start.sh \
  joy_dev:=/dev/input/js1 \
  square_length_m:=0.0338 \
  marker_length_m:=0.0165
```

The wrist camera uses this verified identity-stable default path:

```text
/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0
```

Override it only after verifying the replacement device:

```bash
tools/camera_pan_tilt_calibration/start.sh \
  arm_camera_device:=/dev/v4l/by-id/VERIFIED_CAPTURE_DEVICE
```

If the board pose relative to `base_link` has been independently measured,
record it explicitly instead of letting later calibration code guess it:

```bash
tools/camera_pan_tilt_calibration/start.sh \
  board_pose_reference:="laser/ruler survey 2026-09-02" \
  board_base_x_m:=1.500 board_base_y_m:=0.000 board_base_z_m:=0.800 \
  board_base_roll_rad:=0.000 board_base_pitch_rad:=0.000 board_base_yaw_rad:=3.141593
```

The numbers above only demonstrate argument syntax; do **not** reuse them as
real measurements. If omitted, output truthfully records `NOT_PROVIDED`.

## 4. Capture

1. Fix the robot base and ChArUco board so neither can move during a sample.
   Arrange the board so both the D435 and wrist camera can see it.
2. Hold `L1/LB` and use the D-pad to control pan/tilt, as in the existing system.
3. Confirm the Lift is fixed at its physical lower limit. Release all controls
   and wait until the RViz panel reports `READY`.
4. Optionally enter a pose label such as `middle_center`.
5. Press **Capture stable sample / 安定姿勢を保存**.
6. Accept the sample only when the panel reports `SAVED`. The status reports
   RealSense/wrist corner counts. `REJECTED` means no
   sample was committed; correct the stated evidence problem and retry.
7. Approach important poses from both directions. Use labels such as
   `middle_center_from_pan_inc` and `middle_center_from_pan_dec`.

The capture gate currently requires fresh D435 color/raw-depth/aligned-depth,
fresh wrist RGB and all four CameraInfo sources, fresh pan/tilt feedback, at
least 0.8 s of numerically stable feedback with no more than 0.15 raw-unit
span, fresh complete real SO101 `/joint_states` stable over the same window
(default maximum per-joint span `0.01 rad`), a complete direct
`/arm/hardware_status` sample no more than 1.5 s old, and at least 8 detected
ChArUco corners in **each** RGB camera. These are
explicit data-quality rules, not claims that the mechanism has physically
settled.

The cameras do not have a verified shared hardware clock or trigger, so their
ROS header stamps are not used as a cross-camera acceptance gate. Capture is
allowed only while every stream is freshly observed on the host and the rig is
stationary. Exact header-stamp and host receive-time differences are saved for
audit and are not presented as proof of identical exposure time.

## 5. Output

Every run creates:

```text
output/session_YYYYMMDD_HHMMSS_microseconds/
  session.json
  manifest.jsonl
  sample_0001_optional_label/
    metadata.json
    charuco.json
    color.png
    color_charuco.png
    arm_color.png
    arm_color_charuco.png
    arm_charuco.json
    depth_raw.png
    depth_raw.npy
    depth_aligned_to_color.png
    depth_aligned_to_color.npy
```

`metadata.json` records raw pan/tilt feedback, the latest five-second raw
feedback trace, latest targets, raw arrival direction, stability window/result,
ROS image stamps/encodings/frame IDs, explicit inter-camera stamp differences,
full color/depth/aligned/wrist CameraInfo, the RealSense driver's latest `/tf` and
`/tf_static` transforms (including its internal stream-frame geometry), current
Joy state, required real `/joint_states`, the operator-confirmed fixed Lift
reference, and the complete `/arm/hardware_status` JSON. The six direct servo
`raw_position` values are extracted separately from converted joint radians.
UTC capture time and source age/skew are retained for audit.
`charuco.json` and `arm_charuco.json` record every detected corner ID, pixel
coordinate and its known board coordinate in metres for the two cameras.

The RViz capture panel displays live Pan and Tilt raw feedback values. These
device-reported values are useful for selecting repeatable poses, but are not
labeled as calibrated physical angles until the motion model is solved.

Arrival direction is deliberately named `INCREASING`/`DECREASING`: it describes
the raw feedback change and does not guess physical left/right/up/down. Depth
PNG/NPY values retain the source image encoding and units. This capture tool
does not infer a calibrated angle, URDF transform or metric base-frame pose.

## 6. Wrist-camera intrinsics

The default wrist CameraInfo is now the measured
`robot_bringup/config/arm_camera_calibrated.yaml`. It was solved from 130 valid
ChArUco views captured on 2026-09-03 (OpenCV RMS reprojection error
`0.314985 px`). The all-zero `arm_camera_uncalibrated.yaml` remains only as an
explicit fallback placeholder.

To recalibrate after changing the camera/lens or image resolution, collect at
least 10-20 wrist-camera views with the board at varied image positions,
distances and tilts; repeated nearly identical views do not provide a
well-conditioned intrinsic calibration. Then run:

```bash
cd /home/matsunaga-h/robot_ws
python3 tools/camera_pan_tilt_calibration/solve_arm_camera_intrinsics.py \
  tools/camera_pan_tilt_calibration/output/session_YYYYMMDD_HHMMSS_microseconds
```

This writes `arm_camera_calibrated.yaml` and a per-view reprojection-error
report inside the session. Review the errors before replacing the deployed
measured file; the solver never overwrites an existing result unless `--force`
is explicitly passed.

This solves **intrinsics only**. Refining `base_link -> arm/world` separately
requires multiple arm configurations with fresh real `/joint_states`, plus the
wrist-camera-to-gripper hand-eye transform. Simultaneous images alone cannot
separate a mount error from a wrist-camera mounting error. Samples without
`/joint_states` remain valid for wrist-camera intrinsics, and metadata marks
them `NOT_AVAILABLE` rather than inventing an arm pose.

Stop with `Ctrl-C` and wait for all processes to exit before starting the normal
robot bringup again.
