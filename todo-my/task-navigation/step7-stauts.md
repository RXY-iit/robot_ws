# Step 7 Navigation Status

Last updated: 2026-05-22

## Current Summary

Step 7 has reached a useful real-robot baseline:

```text
FAST-LIO2 + GICP localization
RealSense depth local costmap
Nav2 MPPI controller
AUTO/MANUAL mode switch
cmd_vel safety layer
debug recorder
```

Latest useful run:

```text
debug-output/nav2_20260522_110151
debug-output/nav2_20260522_150715
```

Result:

- FAST-LIO2 started and did not crash.
- Livox was running in `CustomMsg` mode.
- `livox_custom_to_pc2_node` C++ relay published `/livox/lidar_pc2`.
- GICP localization started with `/fast_lio/odometry` hint enabled.
- GICP is still the owner of the final `map -> odom` localization TF.
- Nav2 completed a small real goal.
- `bt_navigator`: `Goal succeeded`.
- `controller_server`: `Reached the goal!`.
- `number_of_recoveries: 0`.
- No TF extrapolation failure.
- No controller patience abort.
- No collision / recovery loop.
- MPPI produced non-zero `linear_y`, so holonomic control is being used.

This is a meaningful Step 7 pass. The system is now good enough for repeated controlled navigation tests and longer-goal validation.

Current interpretation:

```text
GICP-only:
  /livox/lidar_pc2 -> prior-map matching -> map -> odom

FAST-LIO2 + GICP:
  /livox/lidar CustomMsg + /livox/imu -> /fast_lio/odometry
  /fast_lio/odometry gives GICP a motion hint
  GICP still corrects against the saved map and publishes map -> odom
```

## Current Launch Baseline

Use this as the normal real-robot startup:

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
tools/open_nav2_windows.sh --cleanup
```

`tools/open_nav2_windows.sh` now defaults to FAST-LIO mode:

```text
WITH_FAST_LIO=true
USE_FAST_LIO_HINT=true
```

Fallback GICP-only mode is still available:

```bash
tools/open_nav2_windows.sh --no-fast-lio --cleanup
```

## Current Architecture

```text
Livox MID360
  -> /livox/lidar        (CustomMsg, FAST-LIO input)
  -> /livox/imu

livox_custom_to_pc2_node
  -> /livox/lidar_pc2    (PointCloud2 for GICP / scan / RViz)

FAST-LIO2
  -> /fast_lio/odometry
  -> /fast_lio/cloud_registered

GICP localizer
  -> map -> odom TF
  -> /gicp_loc/pose
  -> /gicp_loc/score

Nav2
  -> planner_server
  -> MPPI FollowPath controller
  -> bt_navigator
  -> /nav2/cmd_vel

nav_mode_switch_node
  -> /cmd_vel_raw

cmd_vel_safety_node
  -> /cmd_vel

omni_base_driver
  -> real motor commands
```

RViz note:

- `/livox/lidar` is `livox_ros_driver2/msg/CustomMsg`; RViz cannot display it directly.
- Use `/livox/lidar_pc2` for raw Livox PointCloud2 visualization.
- Use `/fast_lio/cloud_registered` for FAST-LIO output visualization.
- `camera_init` and `body` are FAST-LIO internal frames. They are not part of the Nav2 TF chain, so RViz TF warnings for those frames are not a Nav2 localization failure.
- `rviz/nav2_navigation.rviz` keeps the TF display disabled by default to avoid confusing FAST-LIO internal-frame warnings during normal navigation.

## Important Fixes Already Applied

### Debug / Startup

- `nav2_debug_recorder.py` starts immediately in `nav2 0 debug`.
- Live monitor starts early and can show missing topics while nodes come up.
- Debug output goes to:

```text
debug-output/nav2_YYYYMMDD_HHMMSS
```

### TF / Localization

- GICP correction stays at 2 Hz.
- `map -> odom` TF rebroadcast is 30 Hz.
- `gicp_localizer_node.py` uses `MultiThreadedExecutor`.
- GICP correction and TF broadcast use separate callback groups.
- `transform_tolerance` was relaxed to 0.3 s.
- GICP can use FAST-LIO odometry delta as an initial-pose hint.

### FAST-LIO Integration

Problems encountered and fixed:

- `test_all.launch.py` originally did not switch Livox to `CustomMsg`; fixed with `fast_lio_mode`.
- Python relay failed because `livox_ros_driver2` Python type support was built under a different Python version.
- Replaced default relay with C++ `livox_custom_to_pc2_node`.
- `fast_lio_mid360.yaml` was not a valid ROS2 parameter file; fixed to `/**: ros__parameters:`.
- FAST-LIO absolute topics were remapped to `/fast_lio/...`.

Current expected topic checks:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/lidar_pc2
ros2 topic info -v /fast_lio/odometry
```

Expected:

```text
/livox/lidar       Type: livox_ros_driver2/msg/CustomMsg
/livox/lidar_pc2   Publisher count: 1
/fast_lio/odometry Publisher count: 1
```

### Nav2 / Controller

- MPPI is the primary controller.
- RPP is kept as fallback.
- MPPI `motion_model` is Omni.
- `PositionGoalChecker` is used.
- Spin / BackUp / Wait recoveries are enabled for Phase 3+.
- Local/global inflation radius was increased so it is larger than the robot inscribed radius.
- Polygon footprint is configured for footprint collision checking.
- RealSense depth PointCloud2 feeds the local costmap voxel layer.
- `mark_threshold: 1` fixed most obstacle clearing/marking issues during camera costmap tests.

### Safety / Mode

- `A`: switch MANUAL -> AUTO.
- `B`: switch AUTO -> MANUAL, cancel active Nav2 goal, publish zero.
- `Y`: toggle emergency stop.
- `nav_mode_switch_node` publishes to `/cmd_vel_raw`.
- `cmd_vel_safety_node` is the intended only publisher to `/cmd_vel`.
- Watchdog publishes zero if upstream command disappears.

## Latest Run Interpretation

Latest run:

```text
debug-output/nav2_20260522_110151
debug-output/nav2_20260522_150715
```

Key timeline:

```text
11:02:01 FAST-LIO started
11:02:01 C++ relay ready: /livox/lidar -> /livox/lidar_pc2
11:02:03 GICP localizer started
11:02:05 Nav2 managed nodes active
11:03:30 2D Goal Pose sent
11:03:31 A pressed: MANUAL -> AUTO
11:03:43 controller_server: Reached the goal!
11:03:43 bt_navigator: Goal succeeded
```

Useful metrics from recorder:

```text
goal count: 1
result: SUCCEEDED
recoveries: 0
navigation_time_sec: 13
initial distance_remaining: 1.20 m
final distance_remaining: 0.0 m
max /nav2_cmd_vel:
  linear_x: 0.120 m/s
  linear_y: 0.061 m/s
  angular_z: 0.294 rad/s
```

Interpretation:

- This was not just "RViz looked OK"; the action server reported success.
- MPPI produced lateral velocity, so the robot is no longer forward-only like RPP.
- The safety layer did not block the successful AUTO interval.
- Watchdog warnings before/after active navigation are expected zero-output safety behavior.

## Issues To Watch

### 1. Base Driver `diff_pos` Resets

The latest run still has warnings like:

```text
diff_pos[...] exceeds threshold. Resetting drive_vel.
```

This is below Nav2 and may indicate steering/motor feedback discontinuities. It did not prevent the latest goal, but if the robot feels jerky, pauses, or does not physically match `/cmd_vel`, debug the base driver next.

### 2. Dynamic Obstacle Avoidance Still Needs Real Validation

Camera local costmap marking/clearing is working in basic tests, but the latest successful run was a small goal. It does not yet prove robust avoidance of moving people or crowded indoor obstacles.

Continue testing with:

- low speed,
- short goals,
- clear manual override,
- one obstacle at a time,
- debug recorder running.

### 3. RViz Display Limits

`/livox/lidar` is not visible in RViz because it is `CustomMsg`. This is expected. Use:

```text
/livox/lidar_pc2
/fast_lio/cloud_registered
```

### 4. Old GLIM / AMCL Docs Are Legacy

Navigation baseline is no longer AMCL or GLIM localization. GLIM is only historical map creation / reference material. Current online navigation uses:

```text
FAST-LIO2 odometry hint + GICP prior-map localization
```

## Next Steps

1. Repeat 2-3 longer navigation goals with the same startup command.
2. Record topic rates during those tests:

```bash
ros2 topic hz /fast_lio/odometry
ros2 topic hz /livox/lidar_pc2
ros2 topic hz /cmd_vel
```

3. Test obstacle behavior in a controlled way:
   - static obstacle appears,
   - static obstacle removed,
   - moving person crosses,
   - narrow passage with enough clearance.
4. If motion is jerky, inspect `cmd_vel_to_motor_node`, steering feedback, and `diff_pos` reset cause.
5. After repeated short/medium goals pass, start Phase 4 work:
   - Mission Manager,
   - waypoint map,
   - operation commands,
   - blocked/observe/retry behavior.
6. Phase 5 should wait until navigation + obstacle behavior is repeatable:
   - semantic detection,
   - object/zone layer,
   - LLM task planner.
