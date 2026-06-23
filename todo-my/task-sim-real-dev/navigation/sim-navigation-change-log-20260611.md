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

### note rxy
大致可以分成这几类：

**环境/运行环境相关**
- `ROS_DOMAIN_ID=20`
- Isaac Sim 必须先 Play
- `use_sim_time:=true` 支持
- clean ROS shell，避免 `spdlog/fmt` 动态库冲突
- `.launch.py` 里给 sim / Nav2 / robot_state_publisher / adapter 传 sim time

这一类本质是：让 ROS2 节点在 Isaac Sim 时间和 sim ROS graph 里正常工作。

**路径/地图/folder 相关**
- 新 3D map：
  `maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply`
- 新 2D Nav2 map：
  `maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml`
- 新 2D image：
  `maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.pgm`
- GLIM dump source：
  `maps/sim_glim_dump_sim_factory_mid360_like_gpu_trueimu_011_20260607_193707`
- `/home/matsunaga-h/...` 改成 `/home/ruan-x/...`
- `localization_pkg` 默认 map / PLY 指向 sim map

这一类是路径和资源对齐：把 real/旧 PC 路径换成当前 sim PC 的地图资源。

**依赖安装相关**
- 缺少 Nav2 controller/planner plugins：
  - `ros-humble-nav2-mppi-controller`
  - `ros-humble-nav2-regulated-pure-pursuit-controller`
  - `ros-humble-nav2-navfn-planner`
- `open_sim_nav2_windows.sh` 加了 preflight check

这一类是运行依赖：不是逻辑 bug，而是 sim PC 没装真机参数文件需要的 Nav2 plugin。

**topic / frame / TF 对齐相关**
- `/l402_glim_points` 改成 `/sim_glim_points`
- `/point_cloud -> /livox/lidar -> /livox/lidar_pc2 -> /scan`
- `/scan` frame 对齐到 `base_footprint`
- 补 `/tf_static`，包括 `base_footprint -> base_link -> livox_frame`
- `map -> odom` 来自 GICP
- `odom -> base_footprint` 来自 wheel odom

这一类是 sim-real 对齐的核心之一：让 sim 发布的话题和 frame 能喂给真机同款 localization/Nav2 chain。

**localization 逻辑相关**
- 放弃 AMCL debug path
- 改成真机一致：FAST-LIO2 + GICP localization
- 加 `/initialpose` 发布
- 支持 RViz `2D Pose Estimate`
- 检查 `/fast_lio/odometry`、`/gicp_loc/score`、`map -> odom`

这一类属于定位方案对齐：sim 不另起 AMCL，而是尽量复用真机定位结构。

**控制链/导航逻辑相关**
- `tools/sim/open_sim_nav2_windows.sh`
- Joy-Con / teleop
- `nav_mode_switch_node` 切 AUTO
- Nav2 输出 `/nav2/cmd_vel`
- 最终 sim motion 消费 `/cmd_vel`
- `sim_omni_drive_command_node` 必须运行，否则 `/cmd_vel` 没 subscriber
- goal succeeded 后 `/cmd_vel` 归零是正常现象

这一类是控制闭环：Nav2 goal 能真正经过 mode switch / cmd_vel / sim drive 让 robot 动起来。

**工具/测试脚本相关**
- `tools/sim/export_glim_dump_to_ply.py`
- `tools/sim/open_sim_fast_lio2_loc_windows.sh`
- `tools/sim/open_sim_nav2_windows.sh`
- `tools/check_nav2_phase1.sh`
- sim-specific RViz configs


核心控制逻辑是尽量沿用真机同一套。

具体来说：

- **Localization**：沿用真机方向的 `FAST-LIO2 + GICP localization`
  - sim launcher 调的是 `localization_pkg fast_lio_localization.launch.py`
  - 不是另写一套 AMCL sim-only localization
  - 只是把输入 topic、map path、`use_sim_time`、initial pose 这些适配到 sim

- **Navigation / Nav2**：沿用真机的 `nav_pkg`
  - `src/nav_pkg/config/nav2_params.yaml`
  - `src/nav_pkg/launch/navigation.launch.py`
  - controller/planner plugin 也是按真机参数文件来装和跑
  - sim 只是加了 `use_sim_time:=true` 和路径修正

- **控制链语义**：也按真机方式靠近
  - Nav2 输出 `/nav2/cmd_vel`
  - mode switch 控制 AUTO/MANUAL
  - 最终输出 `/cmd_vel`
  - sim 端只是在最后把 `/cmd_vel` 转成 Isaac 能吃的 `/sim/target_joint_states`

所以可以理解为：

```text
真机逻辑：Localization + Nav2 + mode switch + cmd_vel chain
sim 复用：同一套 Localization/Nav2/mode 逻辑
sim 替换：sensor source、map path、sim time、最终 motor/Isaac actuator adapter
```

真正 sim-only 的主要是外围适配，不是核心定位/导航策略。