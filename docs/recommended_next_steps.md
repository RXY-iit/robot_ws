# Recommended Next Steps

Generated: 2026-07-13

## Final Conclusion

1. 当前 Simulation workspace 是否适合直接迁移到真机？

Answer: **整理后适合**.

Reason:

- The core architecture is already suitable: `robot_agent -> robot_skills -> Nav2 -> nav_mode_switch -> safety_layer -> base driver`.
- The user context says the real OS has already run bottom navigation with the same logic, so this is not a from-scratch migration.
- Many packages are hardware-neutral or real-intended.
- But the workspace is not ready for direct copy/deploy because sim maps, Isaac/USD assets, `/clock` assumptions, large bags, hardcoded absolute paths, and sim controllers are still mixed into the same tree.

## Execution Order From Simulation to Real Robot

### 1. Freeze the current sim checkpoint

- Commit this documentation audit.
- Tag the current known-good sim state.
- Do not continue mixing sim asset edits with real runtime cleanup on the same unlabelled branch.

### 2. Define real runtime package set

Carry forward:

- `src/robot_agent`
- `src/robot_skills`
- `src/nav_pkg`
- `src/localization_pkg`
- `src/safety_layer`
- `src/robot_bringup`
- `src/robot_description`
- `src/omni_base_driver`
- `src/my_messages`
- required third-party drivers

Exclude from real runtime:

- `src/robot_sim`
- `src/robot_sim/usd/**`
- `models/omniverse_assets/**`
- `sim_bag/**`
- sim-generated map dumps except as offline references

### 3. Create real profiles

Create or derive:

- `src/nav_pkg/config/nav2_params_real.yaml`
- `src/localization_pkg/config/gicp_localizer_real.yaml`
- a real map bundle, for example `maps/real_lab_001/`
- a real dashboard/agent launch or script, for example `tools/run_agent_dashboard_real.sh`

Keep sim profiles separate:

- `nav2_params_sim.yaml`
- `gicp_localizer_sim.yaml`
- `ROBOT_MAP_ID=sim_real_lab_001`

### 4. Rebuild the real interface contract on real OS

On real robot, record:

- `ros2 topic list -t`
- `ros2 topic info -v /livox/lidar`
- `ros2 topic info -v /camera/depth/points`
- `ros2 topic info -v /cmd_vel`
- `ros2 topic hz /livox/lidar`
- `ros2 topic hz /livox/imu`
- `ros2 topic hz /camera/camera/depth/image_rect_raw`
- `ros2 topic hz /wheel_odom`
- `ros2 run tf2_ros tf2_echo map base_footprint`
- `ros2 run tf2_ros tf2_echo base_footprint livox_frame`

Update `docs/sim_real_interface_contract.md` with measured QoS and frequency.

### 5. Real map and localization

- Build a real `map_3d.ply` from real Livox data.
- Build a real `nav2.yaml` + `nav2.pgm`.
- Calibrate GICP threshold, voxel sizes, correspondence distance, and initial pose workflow.
- Only then enable autonomous Nav2 goals.

### 6. Real navigation bringup with conservative limits

Start with:

- `use_sim_time:=false`
- low `max_vx`, `max_vy`, `max_wz`
- reverse motion disabled or very limited
- motion policy disabled for A/B test, then enabled only after baseline works
- operator holding hardware estop

Test order:

1. TF and sensor freshness only, no motor power.
2. Manual `/teleop/cmd_vel -> /cmd_vel_raw -> /cmd_vel`.
3. Safety watchdog and estop.
4. Wheel odometry straight/rotate/lateral calibration.
5. Nav2 local costmap obstacle marking.
6. Single short `NavigateToPose` in open space.
7. Semantic/agent-driven `goto_pose`.

### 7. Re-enable agent and skills

Only allow skills marked real-supported:

- `get_robot_status`
- `await_robot_auto_mode`
- `await_robot_stationary`
- `check_localization_health`
- `resolve_semantic_target`
- `list_known_targets`
- `goto_pose`
- `move_relative`
- `move_relative_xy`
- `rotate_relative`
- `cancel_motion`
- `capture_observation` if camera provider is confirmed

Keep disabled on real until implemented/validated:

- `recover_localization_spin`
- `detect_known_object`
- `set_camera_pose`
- `move_lift_bounded`

## Specific Answers

### 2. 哪些内容可以直接复用？

- `src/safety_layer/safety_layer/cmd_vel_safety_node.py`
- `src/my_messages/**`
- `src/robot_description/urdf/robot.urdf.xacro`
- `src/robot_description/launch/rsp.launch.py`
- `src/omni_base_driver/**`
- `src/nav_pkg/scripts/nav_mode_switch_node.py`, after real button/limit review

### 3. 哪些内容需要参数化后复用？

- `src/nav_pkg/launch/navigation.launch.py`
  - hardcoded safety limits and motion policy parameters
  - suggested params: `robot_profile`, `safety_limit_profile`, `enable_motion_policy`, `base_frame`, `plan_topic`
- `src/nav_pkg/config/nav2_params.yaml`
  - hardcoded controller/costmap tuning and BT XML absolute paths
  - suggested params/files: `nav2_params_real.yaml`, package-share BT paths
- `src/localization_pkg/launch/fast_lio_localization.launch.py`
  - hardcoded default sim map/PLY paths
  - suggested params: `map_bundle`, `map`, `pcd_map`, `lidar_mode`, `with_fast_lio`
- `src/localization_pkg/config/gicp_localizer.yaml`
  - hardcoded sim map path and sim threshold
  - suggested params/files: `gicp_localizer_real.yaml`, `fitness_score_threshold_real`, `voxel_profile`
- `src/robot_agent/config/*.yaml`
  - mode/model/parser/map policy
  - suggested params: `mode=real`, `ROBOT_MAP_ID=real_lab_001`, real semantic map path
- `src/robot_skills/config/*.yaml`
  - semantic map and skill support
  - suggested params: real map bundle, real perception provider, hardware capability flags

### 4. 哪些内容不能或者不需要迁移？

- Isaac Sim API scripts in `src/robot_sim/scripts/**`
- USD files in `src/robot_sim/usd/**`
- Simulation sensor setup for RTX LiDAR, Isaac D455/D435, Isaac true IMU
- Ground truth pose/time and simulator-authored TF
- `/clock` dependency and `use_sim_time:=true` defaults
- Simulation controller nodes:
  - `sim_omni_drive_command_node.py`
  - `sim_lift_command_node.py`
  - `sim_livox_imu_node.py`
- 仿真资产:
  - `models/omniverse_assets/**`
  - `sim_bag/**`
  - sim-generated map dumps as final real maps

### 5. Navigation 参数如何处理？

可以直接保持一致:

- command chain `/nav2/cmd_vel -> /cmd_vel_raw -> /cmd_vel`
- frame contract: `map`, `odom`, `base_footprint`, `livox_frame`, camera frames
- Nav2 lifecycle and action interface
- safety watchdog/estop architecture
- map static layer transient-local expectation

可以作为真机初始值:

- MPPI controller at 10 Hz
- conservative `vx_max=0.24`, `vy_max=0.12`, `wz_max=0.40`
- local costmap `4m x 4m`, `0.05m` resolution
- local inflation `0.45m`
- global inflation `0.20m`
- safety limits, but start lower if uncertain

必须重新标定:

- real 2D/3D map and origin
- GICP score threshold, voxel sizes, correspondence distance
- LiDAR height/pitch and scan slice
- RealSense depth obstacle ranges/heights
- robot footprint and base origin
- wheel radius, steering zero, steering limits, steering rate
- odometry covariance and TF latency

真机中必须增加:

- real profile files
- hardware estop and motor-driver disable checks
- battery/motor/sensor health gates
- sensor freshness watchdogs
- QoS audit per topic
- initial pose workflow
- real semantic map bundle
