# Touch Anything V1

ROS 2 Humble package for the real-tested SO101 Press demo. It combines a human point prompt, on-demand SAM 2.1, D435 aligned depth, an Arm Camera visual loop, guarded SO101 IK/execution and auditable evidence.

## Current state machine

```text
D435 freeze/prompt/SAM/depth -> human confirms 3-D preview
-> automatic FAR preview -> reference pan/tilt -> D435 reacquire
-> guarded FAR execution -> automatic Arm Camera SAM handoff
-> human confirms target once
-> FAR: mask-centre alignment + 25 mm +X steps
-> measured +X >= 35 mm: latch NEAR
-> NEAR: hold X and align fixed lower tip to SAM mask in Y/Z
-> aligned + fixed tip inside mask: guarded 25 mm +X PRESS
-> repeated appearance change: guarded Home
```

NEAR is a one-way transition. Measured X drift during Y/Z correction cannot return the loop to FAR. The configured `pre_touch_near_distance_m` remains a marker/reference only.

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
- NEAR alignment: ≤8 px and fixed tip inside mask;
- PRESS: 25 mm +X per accepted step, bounded to 100 mm beyond the D435 plane;
- gripper: 1.045 rad / 59.9°;
- SAM: Hiera Tiny, two CPU threads, idle unload after 180 s;
- mask score: 0.40 normally, 0.15 for ≥25% image occupancy, 0.05 during close PRESS occlusion;
- automatic loop: at most 24 steps / 180 s.

The exact source of truth is [config/touch_anything.yaml](config/touch_anything.yaml). Do not create different real and simulation copies of these values.

## Target handoff and visual loop

The retained D435 3-D target is projected into Arm Camera only once as the initial SAM prompt. The operator confirms or corrects that Arm Camera mask. Later Y/Z control uses only the current Arm Camera mask, CameraInfo, TF and fixed-tip reference; the D435 projection must not re-enter this loop.

FAR centres the object in the Arm Camera image. NEAR aligns the mask centroid with the rigid lower-jaw tip while keeping X fixed. `solve_near_tip_alignment` locks wrist flex/roll and primarily uses pan/lift/elbow. `solve_near_forward_press` locks shoulder pan/wrist roll and primarily uses shoulder lift/elbow, with wrist-flex compensation.

SAM continuity rejects abrupt target switches while allowing perspective growth, crop and tip occlusion. The primary success evidence compares the current target region with the human-confirmed Arm Camera CIE-Lab baseline: Lab delta ≥60 over ≥65% of the region for two stable observations. Current/following error and D435 X are recorded diagnostics, not independent success proof.

## Saved artifacts

Each session is saved below `debug-output/touch-anything/observation_<uuid>/`:

- D435 RGB/depth/mask, CameraInfo, TF and geometry;
- every Arm Camera RGB/mask/continuity/success observation;
- FAR six-motor current median/MAD baseline;
- per-step motor distribution, waypoint, feedback, Cartesian residual and gateway result.

Runtime artifacts and model weights are intentionally Git-ignored.

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
