# Touch Anything V1

ROS 2 Humble package for the real-tested SO101 elevator-button Press demo. It combines a human point prompt, on-demand SAM 2.1, D435 aligned depth, continuous Arm Camera tracking, guarded SO101 IK/execution and auditable evidence.

## Current state machine

```text
D435 freeze/prompt/SAM/depth -> human confirms 3-D preview
-> automatic FAR preview -> reference pan/tilt -> D435 reacquire
-> guarded FAR execution -> automatic Arm Camera SAM handoff
-> human confirms target once
-> initialize immutable identity keyframe + Pyramid KLT tracker
-> FAR: continuously tracked mask-centre alignment + 25 mm +X steps
-> measured +X >= 35 mm: latch NEAR
-> NEAR: hold X and align fixed lower tip to tracked-mask centroid in Y/Z
-> centroid-aligned + human confirmation: guarded 25 mm +X PRESS
-> repeated colour/appearance change inside tracked button region: guarded Home
```

NEAR is a one-way transition. Measured X drift during Y/Z correction cannot return the loop to FAR. The configured `pre_touch_near_distance_m` remains a marker/reference only.

SAM is not run after every arm waypoint. During normal FAR-to-PRESS motion,
Pyramid KLT and partial-affine RANSAC propagate one target state at the Arm
Camera rate from a latest-frame, depth-1 RGB input. This image-space loop does
not wait for a timestamp-paired CameraInfo or TF: CameraInfo is cached as fixed
calibration metadata, and the newest common TF is sampled only at each motion
planning boundary. A temporary geometry gap therefore delays motion without
starving the tracker. If KLT degenerates, ORB/RANSAC first attempts geometric recovery
against the immutable human-confirmed keyframe. Only if both fail is SAM run
again. A SAM recovery candidate is never accepted automatically: RViz shows
`SAM RECOVERY: CONFIRM`, motion remains paused, and the operator must confirm
that it is the same elevator button (or click the correct button) before the
interrupted approach resumes.

## Evidence sources

- D435 RGB, aligned Z16 depth and CameraInfo;
- Arm Camera RGB and CameraInfo;
- image-stamped TF plus bounded stable fallback;
- live `/joint_states`, `/arm/dashboard_state`, `/arm/hardware_status`;
- Pan/Tilt, Lift and base-motion state.

Every motion goes through the existing Arm gateway Preview/Validate/Execute path. This package never publishes a hardware trajectory directly. RViz candidates are not collision proofs; V1 has no complete environment collision model.

## Important deployed parameters

- initial Touch view: pan 267°, tilt 40° (one-shot request, not an arrival gate);
- D435 FAR: 80 mm before the retained target plane;
- FAR step: 25 mm +X, up to 60 mm Y/Z correction;
- NEAR latch: 35 mm measured +X advance from FAR;
- fixed lower-tip pixel: `(0.496 W, 0.735 H)`, approximately `(317,352)` at 640×480;
- PRESS entry alignment: fixed tip within 35 px of the tracked-mask centroid; binary-mask
  containment is diagnostic only because jaw occlusion can shrink the mask;
- PRESS: 25 mm +X per accepted step, bounded to 100 mm beyond the D435 plane;
- gripper: 1.549 rad / 88.8°;
- SAM: Hiera Tiny, two CPU threads, idle unload after 180 s;
- continuous tracker: 25 Hz timer, up to 160 grid-distributed features,
  forward/backward KLT check, RANSAC and event-triggered ORB recovery;
- rolling control: every healthy 10 Hz control tick replaces a 10-point,
  50 ms/point action tail; the 20 Hz guarded server consumes it continuously,
  skips obsolete points using transport age, and retains its independent
  350 ms watchdog;
- delay compensation: measured joint velocity predicts the execution-start
  state using camera processing age plus an 80 ms scheduling/transport lead,
  capped at 250 ms and 1.0 rad/s;
- diagnostic target point/pose/markers use best-effort, volatile, depth-1 QoS
  and their periodic refresh is suspended while automatic/active Arm motion
  owns the latency budget; this prevents RViz/rosbag backpressure from starving
  the tracker timer;
- SAM mask score (recovery/manual segmentation only): 0.40 initial, 0.20 FAR,
  0.10 NEAR and 0.05 PRESS;
- automatic loop: at most 24 steps / 180 s.

The exact source of truth is [config/touch_anything.yaml](config/touch_anything.yaml). Do not create different real and simulation copies of these values.

## Target handoff and visual loop

The retained D435 3-D target is projected into Arm Camera only once as the initial SAM prompt. The operator confirms or corrects that Arm Camera mask. Later Y/Z control uses only the current Arm Camera mask, CameraInfo, TF and fixed-tip reference; the D435 projection must not re-enter this loop.

FAR centres the object in the Arm Camera image. NEAR aligns the propagated mask centroid with the rigid lower-jaw tip while keeping X fixed. `solve_near_tip_alignment` preserves the measured camera/tool rotation, frees wrist flex for a physically reachable solution, and locks wrist roll. `solve_near_forward_press` locks shoulder pan/wrist roll and primarily uses shoulder lift/elbow, with wrist-flex compensation.

The tracker rejects weak KLT geometry using feature count, forward/backward error, RANSAC inlier ratio and feature coverage. ORB recovery must also pass descriptor and RANSAC consistency. The primary success evidence compares the continuously tracked button region with the first human-confirmed Arm Camera keyframe: CIE-Lab delta ≥45 over ≥50% of tracked pixels, or a sufficiently large HSV-histogram change, for three consecutive camera frames after an authorized PRESS was attempted. Current/following error, pixel geometry and D435 X remain recorded diagnostics; they do not veto this strong repeated appearance evidence.

## Saved artifacts

Each session is saved below `debug-output/touch-anything/observation_<uuid>/`:

- D435 RGB/depth/mask, CameraInfo, TF and geometry;
- Arm Camera RGB/mask, KLT/ORB metrics, semantic-recovery state and success evidence;
- FAR six-motor current median/MAD baseline;
- per-step motor distribution, waypoint, feedback, Cartesian residual and gateway result.

Runtime artifacts and model weights are intentionally Git-ignored.

For pre-hardware regression, representative saved-session auditing, MOCK-only
Arm execution and isolated rosbag replay, see
`/tools/touch_anything/OFFLINE-REGRESSION.md`. Sparse saved event frames are not
treated as proof of continuous 25 Hz tracking; the dedicated bag recorder is
the source for timing regressions.

## SAM installation

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/setup_sam21.sh
```

## Build and test

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build --packages-select touch_anything so101_arm_control \
  --symlink-install --allow-overriding touch_anything so101_arm_control

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/touch_anything/test src/so101_arm_control/test
```

For workspace migration and the real/sim freeze rules, read `/REAL-SIM-WORKSPACE-HANDOFF-20260906.md` first.
