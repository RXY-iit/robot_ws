# Sim Navigation Change Log - 2026-06-11

## Map Outputs

- 3D PLY exported from GLIM dump:
  `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply`
- 2D Nav2 map:
  `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml`
- 2D map image:
  `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.pgm`

Source GLIM dump:

```text
/home/ruan-x/robot_ws/maps/sim_glim_dump_sim_factory_mid360_like_gpu_trueimu_011_20260607_193707
```

## Files Modified

- `tools/sim/export_glim_dump_to_ply.py`
  - Added exporter for GLIM dump `points_compact.bin` submaps.
  - Applies each submap `T_world_origin` from `data.txt`.
  - Writes binary little-endian PLY.

- `src/nav_pkg/launch/navigation.launch.py`
  - Added `use_sim_time` launch argument.
  - Nav2 lifecycle and servers can now run with `use_sim_time:=true` for Isaac Sim.

- `src/nav_pkg/config/nav2_params.yaml`
  - Replaced BT XML absolute paths from `/home/matsunaga-h/...` to `/home/ruan-x/...`.

- `src/localization_pkg/launch/localization.launch.py`
  - Updated default map path to the generated sim Nav2 map.

- `src/localization_pkg/launch/fast_lio_localization.launch.py`
- `src/localization_pkg/launch/fast_lio_localization_live.launch.py`
- `src/localization_pkg/launch/fast_lio_localization_bag.launch.py`
- `src/localization_pkg/config/gicp_localizer.yaml`
  - Updated old `/home/matsunaga-h/...` map defaults to the current sim 2D map and 3D PLY.

- `tools/sim/open_sim_nav2_windows.sh`
  - Added final sim Nav2 test launcher.
  - Uses `ROS_DOMAIN_ID=20`.
  - Starts Joy-Con/teleop, sim drive, PointCloud2 -> Livox CustomMsg adapter, robot static TF, FAST-LIO2 + GICP localization, Nav2, debug recorder/monitor, RViz, and PLY overlay.
  - Uses `localization_pkg fast_lio_localization.launch.py` with `with_fast_lio:=true`, `use_fast_lio_hint:=true`, and `use_sim_time:=true`.
  - Publishes `/initialpose` once before the automatic check phase for GICP `map -> odom`.
  - Publishes Joy A once by default to switch `nav_mode_switch_node` to AUTO.
  - Default RViz config changed to `rviz/sim_nav2_navigation.rviz`.
  - Added preflight check for required controller plugins:
    `ros-humble-nav2-mppi-controller` and
    `ros-humble-nav2-regulated-pure-pursuit-controller`.
  - Added preflight check for required planner plugin:
    `ros-humble-nav2-navfn-planner`.

- `tools/sim/open_sim_fast_lio2_loc_windows.sh`
  - Added sim counterpart of `tools/open_fast_lio2_loc_terminals.sh`.
  - Default RViz config changed to `rviz/sim_gicp_nav2.rviz`.
  - Starts `robot_sim sim_manual_bringup.launch.py` with Joy-Con/manual command chain, sim drive, robot TF, and PointCloud2 -> Livox CustomMsg adapter.
  - Starts `localization_pkg fast_lio_localization.launch.py` with `with_fast_lio:=true`, `use_fast_lio_hint:=true`, and `use_sim_time:=true`.
  - Publishes one initial `/initialpose` from `--initial-x/--initial-y/--initial-yaw`.
  - Leaves RViz `2D Pose Estimate` available for GICP initial pose refinement.
  - Adds a status window for ROS graph snapshots, `/livox/lidar`, `/livox/lidar_pc2`, `/fast_lio/odometry`, `/gicp_loc/score`, `/scan`, TF, and manual command chain checks.

- `rviz/sim_gicp_nav2.rviz`
  - Added sim-specific GICP/PLY RViz config copied from `rviz/gicp_nav2.rviz`.
  - Replaced real-robot overlay topic `/l402_glim_points` with `/sim_glim_points`.

- `rviz/sim_nav2_navigation.rviz`
  - Added sim-specific Nav2 RViz config copied from `rviz/nav2_navigation.rviz`.
  - Replaced real-robot overlay topic `/l402_glim_points` with `/sim_glim_points`.

- `src/robot_sim/launch/sim_manual_bringup.launch.py`
  - Added `use_sim_time` launch argument.
  - Passes sim time to `robot_state_publisher` and the PointCloud2 -> Livox CustomMsg adapter.

- `src/robot_sim/launch/export_robot_sim_urdf.launch.py`
  - Added `use_sim_time` launch argument for sim navigation runs.

- `tools/check_nav2_phase1.sh`
  - Added `USE_SIM_TIME=true` support for TF quick checks.
  - Updated TF notes to match the sim localization chain: `odom -> base_footprint` from wheel odometry, `map -> odom` from GICP localization.

## Debug Session 20260611_163706

Session:

```text
/home/ruan-x/robot_ws/debug-output/sim_nav2_20260611_163706
```

Findings:

- RViz 2D map display was OK, so the selected 2D map loaded correctly.
- Debug state reached AUTO but had no goal yet, so no `/nav2/cmd_vel` was expected.
- `/point_cloud -> /scan` was blocked by missing static TF for `livox_frame`.
- `/tf_static` was absent in the live sim graph; the final launcher now starts `robot_state_publisher` to provide the robot frame chain.

## Debug Session 20260611_173214

Findings:

- FAST-LIO2 + GICP localization was healthy: `/fast_lio/odometry`, `/gicp_loc/score`, `/scan`, `map -> odom`, and robot TF were present.
- Nav2 `controller_server` crashed during lifecycle configure with exit code `-6`.
- Root cause: the sim PC did not have the controller/planner plugin packages used by `src/nav_pkg/config/nav2_params.yaml`:
  - `ros-humble-nav2-mppi-controller`
  - `ros-humble-nav2-regulated-pure-pursuit-controller`
  - `ros-humble-nav2-navfn-planner`
- The sim Nav2 launcher now checks for these packages before opening windows.

## Debug Session 20260611_174053

Findings:

- After installing the controller plugins, `controller_server` no longer failed on configure.
- `planner_server` still crashed during lifecycle configure.
- Root cause: `nav2_navfn_planner/NavfnPlanner` was referenced by `src/nav_pkg/config/nav2_params.yaml`, but `ros-humble-nav2-navfn-planner` was not installed yet.
- `tools/sim/open_sim_nav2_windows.sh` was updated to include `nav2_navfn_planner` in the preflight plugin check.

## Debug Session 20260611_174946

Session:

```text
/home/ruan-x/robot_ws/debug-output/sim_nav2_20260611_174946
```

Result:

- First successful sim navigation closed loop with FAST-LIO2 + GICP + Nav2.
- Required plugin packages were installed:
  - `ros-humble-nav2-mppi-controller`
  - `ros-humble-nav2-regulated-pure-pursuit-controller`
  - `ros-humble-nav2-navfn-planner`
- Nav2 lifecycle nodes were all active:
  - `/controller_server`
  - `/planner_server`
  - `/behavior_server`
  - `/bt_navigator`
  - `/map_server`
  - `/global_costmap/global_costmap`
  - `/local_costmap/local_costmap`
- Localization checks were healthy:
  - `/fast_lio/odometry` was published.
  - `/gicp_loc/score` was about `0.116`.
  - `/scan` was published in `base_footprint`.
  - `map -> odom` TF was available.
  - `odom -> base_footprint` was available from wheel odometry.
- RViz navigation goal succeeded:
  - `action_status: SUCCEEDED`
  - `number_of_recoveries: 0`
  - `navigation_time_sec: 12`
  - goal: `x=1.365`, `y=0.106`, `yaw=0.403` in `map`
  - plan: 5 poses in `map`
  - final odom was about `x=1.49`, `y=0.14`, `yaw=0.45`
- `/nav2/cmd_vel`, `/cmd_vel`, and teleop command topics were zero after success; this is expected because the robot had reached the goal.

Problems solved by this run:

- Replaced the earlier AMCL path with the real-aligned FAST-LIO2 + GICP localization chain.
- Fixed missing robot static TF for scan conversion and RViz frame display.
- Replaced real-map RViz overlay topic `/l402_glim_points` with sim overlay `/sim_glim_points`.
- Added sim-time support to the relevant robot sim and Nav2 launch paths.
- Added preflight checks for the Nav2 plugins required by the real robot parameter file.

Recommended next checks:

- Run two or three longer RViz goals across the mapped area.
- Watch `/gicp_loc/score`, local/global costmaps, and recovery count.
- Confirm the robot does not clip shelf edges or walls when turning near obstacles.
- Keep Nav2/localization parameters aligned with the real robot unless a sim-only deviation is clearly needed.

## Notes From Mapping Issues

- Use a clean ROS shell to avoid `spdlog/fmt` dynamic library conflicts.
- Isaac Sim must already be in Play before the sim Nav2 launcher is useful.
- If `sim_omni_drive_command_node` is not running, `/cmd_vel` has no subscriber and the robot will not move.
- For this sim run, `/point_cloud` is used as the PointCloud2 source for `/scan`; do not assume `/livox/lidar` is PointCloud2.
