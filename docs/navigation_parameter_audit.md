# Navigation Parameter Audit

Generated: 2026-07-13

Source files audited:

- `src/nav_pkg/config/nav2_params.yaml`
- `src/nav_pkg/launch/navigation.launch.py`
- `src/nav_pkg/scripts/nav_mode_switch_node.py`
- `src/safety_layer/safety_layer/cmd_vel_safety_node.py`
- `src/localization_pkg/launch/fast_lio_localization.launch.py`
- `src/localization_pkg/config/gicp_localizer.yaml`
- `maps/sim_real_lab_001/nav2.yaml`
- `src/robot_sim/config/sim_omni_drive.yaml`

## Navigation Verdict

The current Nav2 stack is a good real-robot starting point, but it must not be treated as final real tuning. Parameters should be split into four groups:

- 可以直接保持一致
- 可以作为真机初始值
- 必须重新标定
- 真机中必须增加

## 可以直接保持一致

| Parameter / behavior | Current value | Why keep | Verification |
|---|---:|---|---|
| Command chain | `/nav2/cmd_vel -> /cmd_vel_raw -> /cmd_vel` | Correct safety architecture. | Confirm only safety layer publishes `/cmd_vel`. |
| `robot_base_frame` | `base_footprint` | Consistent across Nav2, skills, localization. | `ros2 run tf2_ros tf2_echo map base_footprint`. |
| `global_frame` | `map` for BT/global costmap | Standard global navigation frame. | Check `/map` frame and localization TF. |
| Local costmap rolling window | `true` | Required for real moving robot. | Confirm local costmap tracks base smoothly. |
| Map static layer transient subscription | `map_subscribe_transient_local: true` | Correct for Nav2 map server. | Restart Nav2 after map server and verify map arrives. |
| Safety watchdog concept | `watchdog_timeout: 0.5` | Correct fail-safe pattern. | Stop `/cmd_vel_raw`; verify zero command within 0.5 s. |
| AUTO/MANUAL cancel behavior | B/manual cancels Nav2 and publishes zero | Required for real operation. | Press B during active goal and check motion stops. |

## 可以作为真机初始值

| Parameter | Current value | Risk | Real test |
|---|---:|---|---|
| MPPI `controller_frequency` | `10.0` Hz | Too low if odom/TF is noisy, acceptable as start. | Measure CPU and command smoothness. |
| MPPI `time_steps` / `model_dt` | `56` / `0.10` | Long horizon may overfit sim and cost CPU. | Compare path tracking at 0.1/0.2/0.3 m/s. |
| MPPI `vx_max` | `0.24` m/s | Conservative; good first real autonomous value. | Straight-line stop distance and operator comfort. |
| MPPI `vx_min` | `-0.08` m/s | Reverse motion can be risky with front-only depth. | Test in open space only; consider disabling initially. |
| MPPI `vy_max/min` | `+/-0.12` m/s | Lateral command depends on real omni kinematics. | Lateral step tests and wheel slip check. |
| MPPI `wz_max/min` | `+/-0.40` rad/s | Reasonable but must match steering response. | Rotate-in-place and curved path tests. |
| Safety `max_vx` | `0.30` m/s | Higher than MPPI max; ok as ceiling. | Use staged speed escalation. |
| Safety `max_vy` | `0.20` m/s | May be high for real lateral slip. | Start lower if floor is slippery. |
| Safety `max_wz` | `1.0` rad/s | Above Nav2 command; ok as ceiling, but high for manual. | Confirm estop and steering response. |
| Local costmap width/height | `4 x 4` m | Good indoor start. | Verify CPU and obstacle persistence. |
| Local costmap resolution | `0.05` m | Good indoor start. | Verify with real depth pointcloud density. |
| Local inflation radius | `0.45` m | Reasonable with ~0.67 x 0.60 m chassis. | Narrow passage trial with spotter. |
| Global inflation radius | `0.20` m | Helps narrow passages; may plan too close. | Compare against real footprint margins. |
| Planner `allow_unknown` | `true` | Useful in partial maps, risky near unknown hazards. | Use only with local obstacle layer verified. |
| GICP `localize_period` | `0.5` s | 2 Hz may be enough at low speed. | Check drift during turns and near repeated geometry. |
| GICP `tf_broadcast_period` | `0.033333` s | Good for Nav2 TF freshness. | Confirm no stale TF warnings. |

## 必须重新标定

| Parameter / area | Current value | Why recalibrate | Method |
|---|---:|---|---|
| 2D map origin/resolution | `maps/sim_real_lab_001/nav2.yaml`: origin `[-5.598335, -10.376734, 0.0]`, resolution `0.05` | Sim map origin is not real map origin. | Build real 2D/3D map bundle and verify known landmarks. |
| 3D GICP map | sim PLY defaults | Sim point cloud geometry and real LiDAR noise differ. | Create real PLY from real bag/live mapping. |
| GICP `fitness_score_threshold` | `0.50` | Comment says sim repeated geometry uses wider threshold; real should be stricter. | Log score distribution while stationary/moving, choose reject threshold. |
| GICP voxel sizes | map `0.25`, scan `0.20` | Real density and motion distortion differ. | Sweep voxel sizes and compare localization stability/CPU. |
| GICP correspondence distance | `1.5` m | Too permissive can snap to wrong structure. | Test with offset initial poses and repeated corridors. |
| `scan_min_height/max_height` | `-0.8` / `-0.3` in base frame | Based on LiDAR height/tilt assumptions. | Measure real Livox height/pitch; visualize `/scan`. |
| Camera obstacle ranges | marking max `1.8`, clearing max `2.5` | Real depth noise, mounting angle, exposure differ. | RViz inspect `/camera/depth/points` and local costmap. |
| Camera obstacle height gates | `0.10` to `1.20` m | Real floor, table legs, reflective objects differ. | Place known obstacles and confirm marking/clearing. |
| Robot footprint | `[[0.34,0.32],[0.34,-0.32],[-0.52,-0.32],[-0.52,0.32]]` | Must match real chassis, wheel protrusions, sensor/lift protrusions, base origin. | Physical measurement and footprint clearance test. |
| Wheel radius | sim `0.1125` | Real wheel effective radius changes with tire/floor/load. | Drive fixed distance and calibrate odometry. |
| Steering rate/limits | sim `2.23053` rad limit, `0.9` rad/s rate | Real steering motor limits and latency differ. | Step steering command and log response. |
| Motion policy params | many launch literals in `navigation.launch.py` | Designed around observed sim/real-lab behavior, but still heuristic. | A/B with policy disabled/enabled in open real area. |
| Safety acceleration/jerk | launch: vx `0.35`, vy `0.20`, wz `0.80`; jerk vx `0.80`, vy `0.50`, wz `1.80` | Real load, tire grip, motor response differ. | Incremental step tests while measuring slip and stop distance. |

## 真机中必须增加

| Missing item | Why needed | Suggested implementation |
|---|---|---|
| Real profile files | Avoid sim defaults and absolute paths. | `nav2_params_real.yaml`, `gicp_localizer_real.yaml`, `real_lab_real.yaml` map bundle config. |
| Hardware health gates | Real robot must check battery, motor driver, serial, estop, sensor freshness. | Add `robot_health` topics or extend `agent` health checks. |
| Sensor freshness watchdog | Nav2 should not move with stale LiDAR/camera/odom. | Add topic frequency checks before AUTO. |
| Real obstacle emergency layer | Depth camera alone may miss rear/side/low obstacles. | Add bumper, lidar obstacle layer, safety scanner, or conservative forbidden zones. |
| Odometry covariance and diagnostics | Nav2/localization need confidence data. | Publish covariance in `/wheel_odom`; add diagnostics topic. |
| Real map bundle selector | Agent/skills need real semantic map, not sim map. | Environment variable or launch arg: `ROBOT_MAP_ID=real_lab_001`. |
| QoS audit | Sensor QoS mismatch can silently break subscribers. | Capture `ros2 topic info -v` for contract topics. |
| Initial pose protocol | Real localization needs repeatable startup pose or operator init. | Define `/initialpose` workflow and success criteria. |

## Hardcoded Positions to Parameterize

| File | Hardcoded value | Suggested parameter |
|---|---|---|
| `src/localization_pkg/launch/fast_lio_localization.launch.py` | default sim map and PLY absolute paths | `map`, `pcd_map`, `map_bundle` launch args with real defaults outside code. |
| `src/localization_pkg/config/gicp_localizer.yaml` | `map_path: /home/ruan-x/...sim_factory.../map_3d.ply` | `map_path` launch override or separate real YAML. |
| `src/nav_pkg/config/nav2_params.yaml` | installed BT XML absolute paths | package-share substitution in launch, or profile-specific BT path. |
| `src/nav_pkg/launch/navigation.launch.py` | safety limits, motion policy params, frame names | real/sim parameter YAML or launch arguments. |
| `src/robot_sim/launch/sim_manual_bringup.launch.py` | sim-only direct command topics and arm limits | keep sim profile only; do not share as real launch. |
| `tools/run_agent_dashboard_sim_clean.sh` | sim map and env setup | create `tools/run_agent_dashboard_real.sh`. |

