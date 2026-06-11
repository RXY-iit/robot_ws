# Sim Navigation Guidance

> Created: 2026-06-11
> Scope: Isaac Sim, ROS 2 Humble, `ROS_DOMAIN_ID=20`
> Policy: localization / map / Nav2 parameters should match the real robot unless there is a clear sim-only reason.

## Status

- Mapping: basically complete. Use the GLIM sim mapping flow in `todo-my/task-sim-real-dev/mapping/sim/isaac-sim-glim-map-creation-guide.md`.
- Sim environment: basically complete. Isaac publishes `/clock`, `/joint_states`, `/point_cloud`, camera topics, `/livox/imu`, `/tf`, and `/wheel_odom`.
- Calibration: basically complete for navigation bringup. Current valid sim baseline bag:

```text
/home/ruan-x/robot_ws/calib_bags/calib_baseline_sim_02_20260611_154944
```

- Navigation: first FAST-LIO2 + GICP + Nav2 sim closed-loop test succeeded on 2026-06-11. Latest valid debug session:

```text
/home/ruan-x/robot_ws/debug-output/sim_nav2_20260611_174946
```

Latest check summary:

- All lifecycle nodes active: controller, planner, behavior, BT navigator, map server, global costmap, local costmap.
- FAST-LIO2 + GICP localization active; `/gicp_loc/score` was about `0.116`.
- `/scan` published in `base_footprint`; `map -> odom` TF was available.
- RViz goal succeeded with `number_of_recoveries: 0`.
- Goal: `x=1.365`, `y=0.106`, `yaw=0.403` in `map`.
- Plan: 5 poses in `map`.
- Final odom was about `x=1.49`, `y=0.14`, `yaw=0.45`.
- `/nav2/cmd_vel` and `/cmd_vel` were zero after success, which is expected.

Required Nav2 plugin packages on the sim PC:

```bash
sudo apt-get install -y \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-navfn-planner
```

The sim Nav2 launcher checks these before starting.

Important calibration result:

- `/cmd_vel -> sim_omni_drive_command_node -> /sim/target_joint_states -> Isaac -> /wheel_odom/tf` has been verified.
- Keep sim `motion_scale = 1.0` for navigation. The real-robot coarse `0.71` scale is only a current-real response profile, not the default sim navigation model.
- Before recording or navigating, `sim_omni_drive_command_node` must be running; otherwise `/cmd_vel` has no subscriber and the robot will not move.

## Goal

Bring up localization and Nav2 in Isaac Sim using the same behavioral assumptions as the real robot:

- Same frames where possible: `map -> odom -> base_footprint -> base_link`.
- Same Nav2 controller/planner/costmap parameters where possible: `src/nav_pkg/config/nav2_params.yaml`.
- Same command chain semantics where possible: Nav2 produces `/nav2/cmd_vel`; final sim motion consumes `/cmd_vel`.
- Use existing mapping and map-conversion tools instead of creating a separate sim-only navigation stack.

First success criterion:

```text
RViz goal in map frame -> Nav2 plan -> /nav2/cmd_vel -> /cmd_vel -> sim robot moves -> /wheel_odom updates
```

Status: achieved in `/home/ruan-x/robot_ws/debug-output/sim_nav2_20260611_174946`.

## Current Localization Test Entry

Final sim localization should match the real robot:

```text
FAST-LIO2 + GICP localization
```

Use the sim-specific localization launcher first:

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_fast_lio2_loc_windows.sh --cleanup
```

This script uses:

```text
3D PLY: /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply
2D map: /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml
```

It starts:

- `robot_sim sim_manual_bringup.launch.py` with Joy-Con/manual command chain, sim drive, robot TF, and PointCloud2 -> Livox CustomMsg adapter.
- `localization_pkg fast_lio_localization.launch.py with_fast_lio:=true use_fast_lio_hint:=true use_sim_time:=true`.
- `/initialpose` publisher for the initial GICP `map -> odom` guess.
- RViz and 3D PLY overlay.
- A status window checking `/livox/lidar`, `/livox/lidar_pc2`, `/fast_lio/odometry`, `/gicp_loc/score`, `/scan`, and TF.

RViz config:

```text
/home/ruan-x/robot_ws/rviz/sim_gicp_nav2.rviz
```

The sim 3D PLY overlay topic is:

```text
/sim_glim_points
```

Do not use `/l402_glim_points` for sim. That topic name is reserved for the real-robot L402 map overlay.

Manual test flow:

1. Start Isaac Sim and press Play.
2. Run `tools/sim/open_sim_fast_lio2_loc_windows.sh --cleanup`.
3. In RViz, use `2D Pose Estimate` if the initial pose needs correction.
4. Keep robot mode MANUAL: press B if needed.
5. Hold L1/LB dead-man and move slowly with Joy-Con.
6. Watch `/gicp_loc/score` and `map -> odom`.

The older AMCL debug path has been replaced. Use FAST-LIO2 + GICP for both sim localization and the final sim Nav2 launcher.

## Current Nav2 Test Entry

Use the sim-specific launcher:

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_nav2_windows.sh --cleanup
```

This script uses:

```text
3D PLY: /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply
2D map: /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml
```

This launcher now uses the same FAST-LIO2 + GICP localization chain as `tools/sim/open_sim_fast_lio2_loc_windows.sh`, then starts Nav2 on top of that localization.

Notes:

- `tools/open_nav2_windows.sh` is the real-robot/reference launcher; use `tools/sim/open_sim_nav2_windows.sh` for sim.
- `src/nav_pkg/config/nav2_params.yaml` BT XML paths have been updated to `/home/ruan-x/robot_ws/...`.
- `src/localization_pkg/launch/localization.launch.py` default map now points to the generated sim Nav2 map.
- Sim mapping may publish `/livox/lidar` as `livox_ros_driver2/msg/CustomMsg`; this final sim launcher uses `/point_cloud` as the PointCloud2 source for `/scan`.
- The sim localization launcher starts robot static TF through `robot_sim sim_manual_bringup.launch.py use_sim_time:=true`; this is required for `/tf_static` frames such as `base_footprint -> base_link -> livox_frame`.

## Current Debug Note

Debug session:

```text
/home/ruan-x/robot_ws/debug-output/sim_nav2_20260611_163706
```

Observed:

- RViz displayed the 2D map correctly, so the generated Nav2 map and `map_server` were usable.
- The debug recorder reached `AUTO`, but recorded no Nav2 goal yet. A 2D Goal Pose still needs to be sent from RViz for the first real navigation test.
- `pointcloud_to_laserscan_node` repeatedly dropped `livox_frame` messages because the final sim launcher did not start robot static TF.
- TF checks reported missing `map -> odom`, `odom -> base_footprint`, and `base_footprint -> base_link`; live graph later showed Isaac still had `/wheel_odom` and `/tf`, but `/tf_static` was missing.

Action taken:

- Added robot static TF startup to the final sim launcher.
- Replaced the earlier AMCL debug path in `tools/sim/open_sim_nav2_windows.sh` with FAST-LIO2 + GICP localization.
- Updated `tools/check_nav2_phase1.sh` so TF checks can run with `USE_SIM_TIME=true`.
- Added `tools/sim/open_sim_fast_lio2_loc_windows.sh` as the real-aligned sim localization test path. This supersedes AMCL for localization evaluation.

## Step 1: Start Isaac Sim Sensors And Motion Chain

Use a clean ROS terminal:

```bash
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset RMW_IMPLEMENTATION
unset CYCLONEDDS_URI

source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_DISTRO=humble
```

Isaac Sim:

1. Open the current navigation/mapping scene.
2. Run the mapping guide's required Script Editor setup commands.
3. Press Play.
4. Confirm core topics:

```bash
ros2 topic list
ros2 topic hz /wheel_odom --window 20
ros2 topic hz /joint_states --window 20
ros2 topic info -v /wheel_odom
```

Start the sim command bridge when navigation should move the robot:

```bash
ros2 run robot_sim sim_omni_drive_command_node
```

Expected:

```bash
ros2 topic info -v /cmd_vel
# subscriber: sim_omni_drive_command_node

ros2 topic info -v /sim/target_joint_states
# publisher: sim_omni_drive_command_node
# subscriber: Isaac ROS2_ActionGraph_subscribe_joint_state
```

For localization sensors, robot static TF must also be present:

```bash
ros2 launch robot_sim export_robot_sim_urdf.launch.py use_sim_time:=true
```

Expected:

```bash
ros2 topic info /tf_static
ros2 run tf2_ros tf2_echo base_footprint base_link --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo base_link livox_frame --ros-args -p use_sim_time:=true
```

## Step 2: Create Or Reuse Map

Current selected source:

```text
/home/ruan-x/robot_ws/maps/sim_glim_dump_sim_factory_mid360_like_gpu_trueimu_011_20260607_193707
```

Generated outputs:

```text
/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply
/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml
/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.pgm
```

To recreate them:

```bash
python3 tools/sim/export_glim_dump_to_ply.py \
  /home/ruan-x/robot_ws/maps/sim_glim_dump_sim_factory_mid360_like_gpu_trueimu_011_20260607_193707 \
  --output /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply \
  --voxel 0.05

python3 tools/ply_to_nav2_map.py \
  /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply \
  --output /home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2 \
  --resolution 0.05 \
  --z-min -0.3 \
  --z-max 1.5 \
  --min-hits 2 \
  --inflate-radius 0.12
```

For a future new map, use the existing GLIM sim mapping flow:

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_nav_mapping_001 \
  --gpu \
  --direct-cmd-vel \
  --cleanup
```

Follow the detailed Script Editor sequence in:

```text
todo-my/task-sim-real-dev/mapping/sim/isaac-sim-glim-map-creation-guide.md
```

After GLIM saves a valid dump/map, create a Nav2 2D occupancy map using existing tools. For live aligned points:

```bash
python3 tools/pointcloud_to_nav2_map.py \
  --topic /glim_ros/aligned_points \
  --target-frame map \
  --output maps/sim_nav2_map_001 \
  --duration 60 \
  --resolution 0.05
```

If using an exported PLY map instead:

```bash
python3 tools/ply_to_nav2_map.py \
  <map_points.ply> \
  --output maps/sim_nav2_map_001
```

Output expected:

```text
maps/sim_nav2_map_001.yaml
maps/sim_nav2_map_001.pgm
```

## Step 3: Localization And Navigation Bringup

Use the same real-robot localization/Nav2 packages first.

FAST-LIO2 + GICP sim localization manual equivalent:

```bash
ros2 launch robot_sim sim_manual_bringup.launch.py \
  start_joy:=true \
  start_pc2_adapter:=true \
  direct_cmd_vel:=false \
  use_sim_time:=true
```

Then:

```bash
ros2 launch localization_pkg fast_lio_localization.launch.py \
  map:=/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml \
  pcd_map:=/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply \
  with_fast_lio:=true \
  use_fast_lio_hint:=true \
  use_sim_time:=true \
  initial_x:=0.0 \
  initial_y:=0.0 \
  initial_yaw:=0.0
```

Topic chain:

```text
/point_cloud -> pointcloud2_to_livox_custom_msg_node -> /livox/lidar
/livox/lidar -> FAST-LIO2 -> /fast_lio/odometry
/livox/lidar -> livox_custom_to_pc2_node -> /livox/lidar_pc2
/livox/lidar_pc2 -> GICP + pointcloud_to_laserscan -> map->odom + /scan
```

If running manually, publish an initial pose after GICP is up:

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

RViz `2D Pose Estimate` also publishes `/initialpose`; use it to refine the GICP initial `map -> odom`.

Navigation:

```bash
ros2 launch nav_pkg navigation.launch.py use_sim_time:=true
```

For first sim test, keep the real Nav2 parameters:

```text
src/nav_pkg/config/nav2_params.yaml
```

Only change a parameter for sim if a specific check proves the real setting is incompatible with Isaac timing, topic type, or sensor availability.

## Step 4: Command Chain For Sim

Real robot normal chain:

```text
Nav2 -> /nav2/cmd_vel -> nav_mode_switch -> /cmd_vel_raw -> safety -> /cmd_vel -> motors
```

Sim first-pass chain can be either:

```text
Nav2 -> /nav2/cmd_vel -> nav_mode_switch -> /cmd_vel_raw -> safety -> /cmd_vel -> sim_omni_drive_command_node -> Isaac
```

or, for a minimal debug run:

```text
Nav2/controller debug -> /cmd_vel -> sim_omni_drive_command_node -> Isaac
```

Prefer the full chain unless debugging a specific Nav2/localization issue. Keeping the full chain makes sim behavior closer to the real robot.

## Step 5: Test Script Guide

### Graph Health

Existing script:

```bash
WS=/home/ruan-x/robot_ws tools/check_nav2_phase1.sh
```

Expected important checks:

- `/map_server`, `/planner_server`, `/controller_server`, `/behavior_server`, `/bt_navigator` exist and are active.
- `/map`, `/scan`, `/global_costmap/costmap`, `/local_costmap/costmap` exist.
- TF exists: `map -> odom`, `odom -> base_footprint`, `base_footprint -> base_link`.
- `/cmd_vel_raw` publisher is `nav_mode_switch_node`.
- `/cmd_vel` publisher is `cmd_vel_safety_node`.

### Live Debug

Existing tools:

```bash
tools/nav2_debug_monitor.py
tools/nav2_debug_recorder.py --output-dir debug-output/sim_nav2_<timestamp>
tools/watch_nav2_debug.sh
```

Use them when RViz sends a goal but the robot does not move. They distinguish:

- no plan
- plan exists but `/nav2/cmd_vel` is zero
- `/nav2/cmd_vel` nonzero but `/cmd_vel_raw` or `/cmd_vel` blocked
- `/cmd_vel` nonzero but sim bridge missing

### Minimal Motion Check

Before testing Nav2 goals:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic echo /wheel_odom --once
```

If `/wheel_odom` does not change after a nonzero command, check:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /sim/target_joint_states
```

### Navigation Goal Check

In RViz:

1. Use `2D Pose Estimate` to refine GICP `/initialpose` if localization is offset.
2. Send a nearby `2D Goal Pose` first, about `0.5-1.0 m`.
3. Watch `/plan`, `/nav2/cmd_vel`, `/cmd_vel_raw`, `/cmd_vel`, and `/wheel_odom`.

Useful commands:

```bash
ros2 topic echo /nav2/cmd_vel --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /plan --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

## First-Pass Acceptance Criteria

- Nav2 lifecycle nodes are active.
- Map is loaded in RViz and global costmap aligns with it.
- Local costmap updates from sim depth or the selected obstacle source.
- A nearby goal creates a valid `/plan`.
- `/nav2/cmd_vel` becomes nonzero while following the plan.
- `/cmd_vel` reaches `sim_omni_drive_command_node`.
- `/wheel_odom` changes consistently with the command.
- Robot stops when goal completes or when safety/mode switch blocks motion.

## Parameter Policy

Use real-robot parameters by default:

- Nav2: `src/nav_pkg/config/nav2_params.yaml`
- FAST-LIO2: `src/localization_pkg/config/fast_lio_mid360.yaml`
- GICP localization: `src/localization_pkg/config/gicp_localizer.yaml`

Allowed sim-specific differences:

- Map files should use `maps/sim_*` names.
- Bags/logs should use `sim_*` names.
- A PointCloud2 source may differ if sim `/livox/lidar` is `CustomMsg`.
- `use_sim_time` may need to be set consistently if TF timestamps prove that Isaac `/clock` is required.

Do not create a separate sim tuning fork until a concrete failure is observed and recorded.
