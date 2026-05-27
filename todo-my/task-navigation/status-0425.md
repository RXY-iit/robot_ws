# Status Record

Original record date: 2026-04-25  
Latest update: 2026-05-22

This file keeps the main progress thread from basic hardware bringup to the current Navigation baseline.

## Current Result - 2026-05-22

The current real-robot Navigation baseline is:

```text
FAST-LIO2 + GICP localization
RealSense local costmap
Nav2 MPPI controller
AUTO/MANUAL mode switch
cmd_vel safety layer
debug-output recorder
```

Latest useful debug run:

```text
debug-output/nav2_20260522_110151
debug-output/nav2_20260522_150715
```

Result:

- FAST-LIO2 starts and stays alive.
- `/livox/lidar` is `livox_ros_driver2/msg/CustomMsg`.
- `/livox/lidar_pc2` is published by the C++ relay.
- GICP uses `/fast_lio/odometry` as a hint and publishes `map -> odom`.
- FAST-LIO2 improves short-term LiDAR-IMU odometry, but GICP remains the final map-correction source for Nav2.
- Nav2 MPPI completed a small goal.
- `bt_navigator`: `Goal succeeded`.
- `controller_server`: `Reached the goal!`.
- `number_of_recoveries: 0`.
- MPPI produced non-zero `linear_y`, confirming holonomic control is active.

Normal startup:

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
tools/open_nav2_windows.sh --cleanup
```

`tools/open_nav2_windows.sh` now defaults to FAST-LIO mode. Use this only when the robot is ready for the current Nav2 stack. For fallback testing:

```bash
tools/open_nav2_windows.sh --no-fast-lio --cleanup
```

## Current Topic Policy

```text
/livox/lidar       CustomMsg     FAST-LIO input, not directly visible in RViz
/livox/lidar_pc2   PointCloud2   GICP / pointcloud_to_laserscan / RViz
/fast_lio/odometry Odometry      GICP hint
/fast_lio/cloud_registered PointCloud2 FAST-LIO output
/gicp_loc/pose     PoseStamped   localization debug
/gicp_loc/score    Float32       localization score
/scan              LaserScan     Nav2/global obstacle support
/camera/depth/points PointCloud2 RealSense local costmap source
```

RViz should display `/livox/lidar_pc2`, not `/livox/lidar`.

`camera_init` and `body` are FAST-LIO internal frames. They do not need to connect to `map` for Nav2; the important TF chain remains `map -> odom -> base_footprint -> base_link -> livox_frame`.

## Important Fixes Since Original 2026-04-25 Bringup

### TF / Odom

- `robot_odom_node` publishes odometry on `/wheel_odom`, not `/odom`.
- TF chain is:

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

- `odom -> base_footprint` is dynamic from wheel odom during real operation.
- `static_odom` is only for sensor-only tests.

### Python / Conda

ROS 2 Humble should use Python 3.10.

Avoid building ROS packages while conda is active. A real failure happened when `livox_ros_driver2` Python type support was generated under Python 3.13; the Python CustomMsg relay could not import it from ROS Humble Python 3.10.

Expected:

```bash
which python3
python3 --version
```

```text
/usr/bin/python3
Python 3.10.x
```

### FAST-LIO Integration

Problems found and fixed:

- `test_all.launch.py` needed `fast_lio_mode` so Livox can switch to `xfer_format=1`.
- FAST-LIO needs `/livox/lidar` as `CustomMsg`.
- GICP and RViz need PointCloud2, so `livox_custom_to_pc2_node` converts:

```text
/livox/lidar -> /livox/lidar_pc2
```

- Python relay failed due Python type-support mismatch.
- Default relay is now C++.
- `fast_lio_mid360.yaml` was converted to ROS2 parameter format.
- FAST-LIO absolute topics are remapped to `/fast_lio/...`.
- Standard navigation now uses FAST-LIO2 + GICP by default; GICP-only is a fallback/debug mode.

### Navigation

- MPPI is primary controller.
- RPP remains available as fallback.
- `PositionGoalChecker` is used.
- Spin / BackUp / Wait recoveries are enabled.
- Local/global costmaps use polygon footprint.
- Inflation radius was increased above the robot inscribed radius.
- RealSense depth PointCloud2 feeds the local voxel costmap.
- `/nav2/cmd_vel -> /cmd_vel_raw -> cmd_vel_safety_node -> /cmd_vel`.

## Historical Milestones

### 2026-04-25 Basic Hardware Bringup

Confirmed:

- `/livox/imu` publishes.
- `/livox/lidar` publishes.
- `/steer_odom` publishes.
- `/om_response0` publishes around 10 Hz.
- `/drive_odom` issue was traced to `drive_motor.py`.
- Joy-Con teleop and `/cmd_vel` were checked.
- TF tree was corrected.

### 2026-05-03 / 2026-05-05 Mapping And GLIM

GLIM was used for 3D SLAM / map creation and historical localization tests.

Current map assets:

```text
maps/l402_glim_map_0503
maps/saved-map/map-l402-0503/l402_points_0503
maps/l402_2d_map_clean_0509.yaml
```

GLIM is no longer the online Navigation localization baseline.

### 2026-05-20 Phase 1 Nav2

Phase 1 proved basic Nav2 closed-loop movement with GICP localization after fixing TF timing:

- GICP correction 2 Hz.
- `map -> odom` rebroadcast 30 Hz.
- Nav2 controller 10 Hz.
- `transform_tolerance: 0.3`.
- debug recorder starts early.

### 2026-05-21 Phase 2 / Phase 3

Added:

- RealSense local costmap via depth PointCloud2 / voxel layer.
- MPPI controller with Omni motion.
- safety layer.
- Mission Manager package skeleton / OperationLib direction.
- Spin/BackUp recoveries.

Key costmap tuning:

- `mark_threshold: 1` fixed most camera obstacle marking/clearing behavior.
- Local costmap should be judged by `/local_costmap/costmap`, not only raw clearing endpoints.

## Remaining Watch Items

- `cmd_vel_to_motor_node` still reports `diff_pos[...] exceeds threshold`; if motion feels jerky, debug motor/steer feedback next.
- Dynamic obstacle avoidance is not fully validated yet.
- Long-distance navigation should be tested in small increments.
- Semantic / LLM integration should wait until repeated navigation and obstacle tests are stable.

## Next Step

Continue Step 7 real-robot tests:

1. Repeat 2-3 medium goals with `tools/open_nav2_windows.sh --cleanup`.
2. Record:

```bash
ros2 topic hz /fast_lio/odometry
ros2 topic hz /livox/lidar_pc2
ros2 topic hz /cmd_vel
```

3. Test obstacle response with one controlled obstacle.
4. If stable, proceed toward Phase 4 Mission Manager and waypoint workflow.
