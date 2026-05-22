# Current System Frequency Map

Date: 2026-05-22

Purpose: summarize the current runtime frequencies, timeouts, and launch delays that affect real Nav2 debugging.

## Navigation

| Part | Current value | Meaning | Reference |
| --- | ---: | --- | --- |
| Nav2 controller loop | 10 Hz | `controller_server` computes `FollowPath` commands at this rate. | `src/nav_pkg/config/nav2_params.yaml:4` |
| Progress required movement | 0.03 m | Robot must move at least this far to count as progress. | `src/nav_pkg/config/nav2_params.yaml:15` |
| Progress timeout | 20.0 s | If robot does not make the required movement within this time, progress checker can fail the goal. | `src/nav_pkg/config/nav2_params.yaml:16` |
| Controller failure tolerance | 0.3 s | Controller exceptions, such as missing/stale TF, are tolerated only briefly before aborting. | `src/nav_pkg/config/nav2_params.yaml:8` |
| RPP transform tolerance | 0.3 s | Regulated Pure Pursuit transform lookup tolerance. | `src/nav_pkg/config/nav2_params.yaml:31` |
| Desired linear velocity | about 0.10-0.12 m/s | Low real-robot navigation speed observed in the latest MPPI run. | `src/nav_pkg/config/nav2_params.yaml` |
| Min approach velocity | 0.05 m/s | Minimum commanded linear speed near the goal. | `src/nav_pkg/config/nav2_params.yaml:33` |
| Planner expected frequency | 1 Hz | Expected planner server rate. | `src/nav_pkg/config/nav2_params.yaml:52` |
| BT loop duration | 10 ms | Behavior tree tick period, roughly 100 Hz. | `src/nav_pkg/config/nav2_params.yaml:92` |
| Behavior server cycle | 10 Hz | Behavior server update rate. Spin / BackUp / Wait recoveries are enabled in the current Phase 3+ setup. | `src/nav_pkg/config/nav2_params.yaml` |
| Behavior transform tolerance | 0.3 s | TF tolerance used by behavior server. | `src/nav_pkg/config/nav2_params.yaml:78` |

Notes:
- If the robot is intentionally kept still in MANUAL mode, the progress checker timeout is normally about 20 seconds.
- If TF is stale or unavailable, abort can happen much faster because `failure_tolerance` is 0.3 seconds.
- For observation in MANUAL, `movement_time_allowance` can be longer, for example 20-30 seconds, but it should not hide real stuck behavior in AUTO.

## Costmaps And Sensors

| Part | Current value | Meaning | Reference |
| --- | ---: | --- | --- |
| Local costmap update | 5 Hz | Local obstacle/costmap update loop. | `src/nav_pkg/config/nav2_params.yaml:105` |
| Local costmap publish | 2 Hz | Published local costmap visualization/output rate. | `src/nav_pkg/config/nav2_params.yaml:106` |
| Local costmap transform tolerance | 0.3 s | TF tolerance for local costmap pose/lookups. | `src/nav_pkg/config/nav2_params.yaml` |
| Global costmap update | 1 Hz | Global costmap update loop. | `src/nav_pkg/config/nav2_params.yaml:139` |
| Global costmap publish | 1 Hz | Published global costmap visualization/output rate. | `src/nav_pkg/config/nav2_params.yaml:140` |
| Global costmap transform tolerance | 0.3 s | TF tolerance for global costmap pose/lookups. | `src/nav_pkg/config/nav2_params.yaml` |
| Livox MID360 publish | 10 Hz | `/livox/lidar` driver publish frequency. In the current default FAST-LIO mode this topic is `CustomMsg`. | `src/robot_bringup/launch/test_all.launch.py` |
| Livox CustomMsg -> PC2 relay | input 10 Hz | Publishes `/livox/lidar_pc2` as PointCloud2 for GICP, scan conversion, and RViz. | `src/localization_pkg/src/livox_custom_to_pc2_node.cpp` |
| pointcloud_to_laserscan scan time | 0.1 s | LaserScan timing value, roughly 10 Hz if input cloud arrives at 10 Hz. | `src/localization_pkg/launch/fast_lio_localization.launch.py:180` |
| pointcloud_to_laserscan TF tolerance | 0.05 s | Transform tolerance for converting `/livox/lidar_pc2` to `/scan` in FAST-LIO mode. | `src/localization_pkg/launch/fast_lio_localization.launch.py` |
| RealSense color profile | 640x480x30 | Color stream configured for 30 fps. Depth is enabled, but exact depth FPS is launch-default unless explicitly set. | `src/robot_bringup/launch/bringup.launch.py` |

## Localization And TF

| Part | Current value | Meaning | Reference |
| --- | ---: | --- | --- |
| GICP localization period | 0.5 s / 2 Hz | Expensive GICP correction timer. | `src/localization_pkg/config/gicp_localizer.yaml:36` |
| `map->odom` TF broadcast period | 0.033333 s / 30 Hz | Cheap rebroadcast of the latest valid GICP correction. | `src/localization_pkg/config/gicp_localizer.yaml` |
| GICP map voxel size | 0.25 m | Static map downsampling. Lower values cost more CPU. | `src/localization_pkg/config/gicp_localizer.yaml:13` |
| GICP scan voxel size | 0.20 m | Incoming scan downsampling before registration. Lower values cost more CPU. | `src/localization_pkg/config/gicp_localizer.yaml:17` |
| FAST-LIO launch option | default on in Nav2 window launcher | `WITH_FAST_LIO=false` or `--no-fast-lio` disables FAST-LIO explicitly. | `tools/open_nav2_windows.sh` |
| FAST-LIO hint option | default on in Nav2 window launcher | `USE_FAST_LIO_HINT=true` lets GICP use `/fast_lio/odometry` as a hint. | `tools/open_nav2_windows.sh` |
| FAST-LIO time sync | false | External LiDAR/IMU time sync is disabled in current FAST-LIO config. | `src/localization_pkg/config/fast_lio_mid360.yaml` |

Notes:
- Current shape: FAST-LIO provides local odometry; GICP matching stays at 2 Hz and publishes the global `map->odom` correction.
- The latest valid `map->odom` transform is rebroadcast at 30 Hz for Nav2.
- Nav2 window startup now defaults to FAST-LIO mode. GICP-only mode is still available with `tools/open_nav2_windows.sh --no-fast-lio`.

## Base And Motor Pipeline

| Part | Current value | Meaning | Reference |
| --- | ---: | --- | --- |
| `cmd_vel_to_motor_node` loop | 100 Hz | Converts `/cmd_vel` to steer/drive motor command topics. | `src/omni_base_driver/src/cmd_vel_to_motor_node.cpp` |
| Motor command timeout | 0.3 s | BLV-R drive command is forced to zero if no recent `drive_vel` command arrives. | `src/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py:74` |
| BLV-R command timer | 0.01 s / 100 Hz | Publishes Modbus query/command loop in `drive_motor.py`. Actual serial response rate can be lower. | `src/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py:314` |
| BLV-R polling timer | 0.08 s / 12.5 Hz | Polls drive feedback. This influences `/drive_odom` update rate. | `src/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py:616` |
| Modbus updateRate launch arg | 1000 | Passed to `om_modbus_master_launch.py` from robot bringup. Exact effect depends on driver internals. | `src/robot_bringup/launch/test_all.launch.py:209` |
| Steer motor node loop | 60 Hz | Reads/commands Dynamixel steering motors. | `src/omni_base_driver/src/steer_motor_node.cpp` |
| Wheel odom publish/TF loop | 20 Hz | `/wheel_odom` and `odom->base_footprint` TF timer. | `src/omni_base_driver/src/robot_odom_node.cpp` |
| Fake wheel odom loop | 20 Hz | Phase1-safe fake stationary odom/TF publisher. | `tools/fake_wheel_odom.py` |

## Teleop, Mode, And Debug

| Part | Current value | Meaning | Reference |
| --- | ---: | --- | --- |
| `nav_mode_switch_node` mode heartbeat | 0.5 Hz | Publishes `/robot_mode` every 2 seconds. | `src/nav_pkg/scripts/nav_mode_switch_node.py` |
| Teleop max normal speed | 0.3 m/s | Manual x/y max speed. | `src/robot_bringup/config/joy_teleop.yaml` |
| Teleop max yaw | 0.5 rad/s | Manual yaw max speed. | `src/robot_bringup/config/joy_teleop.yaml` |
| Debug recorder snapshot | 1 Hz | Writes compact state snapshots to `events.jsonl` and `latest_state.md`. | `tools/nav2_debug_recorder.py` |
| Debug monitor print | 1 Hz | Prints compact live Nav2 status. | `tools/nav2_debug_monitor.py` |

Important behavior:
- MANUAL mode should not block `/nav2/cmd_vel`; it only blocks relay from `/nav2/cmd_vel` to real `/cmd_vel`.
- If the robot is kept in MANUAL while a Nav2 goal is active, `/nav2/cmd_vel` can be observed briefly, but after the progress timeout Nav2 may abort because the robot did not move.

## Current Window Launch Delays

| Step | Delay | Reference |
| --- | ---: | --- |
| Cleanup daemon wait | 1 s | `tools/open_nav2_windows.sh` |
| Debug recorder/monitor start | immediate | `tools/open_nav2_windows.sh` |
| Parent delay before bringup window | 1 s | `tools/open_nav2_windows.sh` |
| Parent delay before localization window | 1 s | `tools/open_nav2_windows.sh` |
| Localization internal wait for bringup | 3 s | `tools/open_nav2_windows.sh` |
| Parent delay before navigation window | 1 s | `tools/open_nav2_windows.sh` |
| Navigation internal wait for localization | 5 s | `tools/open_nav2_windows.sh` |
| Parent delay before check window | 1 s | `tools/open_nav2_windows.sh` |
| Check internal wait for Nav2 graph | 8 s | `tools/open_nav2_windows.sh` |
| Parent delay before RViz window | 1 s | `tools/open_nav2_windows.sh` |
| RViz internal wait | 5 s | `tools/open_nav2_windows.sh` |

Current direction:
- `nav2_debug_recorder.py` starts immediately in the first debug window.
- The live monitor starts early and can show missing topics until nodes come up.
- Launch gaps are shortened so the next test records startup and first-goal behavior.
