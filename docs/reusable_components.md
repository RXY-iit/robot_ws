# Reusable Components

Generated: 2026-07-13

## Directly Reusable

These components can migrate to the real robot OS with little or no code change.

| Component | Paths | Why reusable | Real validation |
|---|---|---|---|
| Safety velocity gate | `src/safety_layer/safety_layer/cmd_vel_safety_node.py`, `src/safety_layer/package.xml` | Hardware-neutral watchdog, clamp, estop, `/cmd_vel_raw -> /cmd_vel`. | Confirm real max velocity/accel/jerk and estop behavior on blocks. |
| Base interface messages | `src/my_messages` | Shared `DriveMotor` and `SteerMotor` message definitions used by sim and real base driver. | Build on real OS and verify message compatibility. |
| Real robot description | `src/robot_description/urdf/robot.urdf.xacro`, `src/robot_description/launch/rsp.launch.py` | Defines real TF tree and sensor frames. | Compare measured Livox/RealSense extrinsics with URDF. |
| Real base driver | `src/omni_base_driver` | Consumes `/cmd_vel`, commands drive/steer hardware, publishes `/wheel_odom`. | Validate wheel radius, steering zero, signs, timeout, odometry covariance/frame. |
| Nav mode switch concept | `src/nav_pkg/scripts/nav_mode_switch_node.py` | Clean AUTO/MANUAL arbitration and Nav2 cancel behavior. | Use with real controller and safety layer; validate button mapping. |

## Reusable After Parameterization

| Component | Paths | Current issue | Required parameterization |
|---|---|---|---|
| Nav2 launch/config | `src/nav_pkg/launch/navigation.launch.py`, `src/nav_pkg/config/nav2_params.yaml` | Mixed sim/real-lab tuning and hardcoded BT XML install paths. | Split sim/real profile, make map/BT/safety/motion-policy params launch-configurable. |
| GICP/FAST-LIO localization | `src/localization_pkg/launch/fast_lio_localization.launch.py`, `src/localization_pkg/config/gicp_localizer.yaml` | Defaults point to sim map bundle and sim threshold. | Map path, PLY path, thresholds, initial pose, LiDAR type, `use_fast_lio_hint`. |
| Agent node | `src/robot_agent/robot_agent/nodes/agent_node.py`, `src/robot_agent/config/*.yaml` | Mode parameter exists, but real launch/profile is not clearly separated. | Real profile for model policy, parser, map id, semantic map, dashboard host/port. |
| Skill layer | `src/robot_skills/config/skill_registry.yaml`, `src/robot_skills/robot_skills/executors/*.py` | Some skills are sim-only or not real-supported. | Enforce `real_supported`, real hardware capability matrix, real perception provider. |
| Semantic map/navigation annotations | `maps/sim_real_lab_001/*.yaml`, `src/robot_skills/config/semantic_map.yaml` | Sim map coordinates are not valid real map coordinates. | Generate real map bundle and point skill config to real map id. |
| RealSense depth pipeline | `src/nav_pkg/launch/navigation.launch.py`, `hardware/realsense_d435.md` | Topics assume `/camera/camera/...`; real profile must match driver namespace. | Camera model, namespace, depth FPS, alignment, clipping ranges. |
| Arm/lift/camera ROS commands | `src/serial_transciever`, `src/robot_sim/robot_sim/sim_lift_command_node.py` | Sim bridge mirrors some real topics but real support is not declared for skills. | Real feedback topics, limits, serial safety, timeout and homing policy. |

## Reusable Only as Simulation Fixtures

| Component | Paths | Use |
|---|---|---|
| Isaac sim drive bridge | `src/robot_sim/robot_sim/sim_omni_drive_command_node.py`, `src/robot_sim/config/sim_omni_drive.yaml` | Keep for sim validation of `/cmd_vel` contract. Do not deploy to real robot. |
| Sim lift/camera bridge | `src/robot_sim/robot_sim/sim_lift_command_node.py` | Keep for sim UI/mission tests. |
| Sim sensor bridge/setup scripts | `src/robot_sim/scripts/**/*.py` | Isaac scene instrumentation only. |
| Real-lab USD scene | `src/robot_sim/usd/scenes/**` | Digital twin and simulation tests only. |
| Collected Omniverse assets | `models/omniverse_assets/**` | Simulation content only. |
| Sim maps and bags | `maps/sim_*`, `sim_bag/**` | Regression fixtures, not real deployment data. |

## Direct Reuse Answer

Specific packages/files that can be directly reused on real:

- `src/safety_layer/safety_layer/cmd_vel_safety_node.py`
- `src/safety_layer/package.xml`
- `src/my_messages/**`
- `src/robot_description/urdf/robot.urdf.xacro`
- `src/robot_description/launch/rsp.launch.py`
- `src/omni_base_driver/include/omni_base_driver/cmd_vel_to_motor_node.hpp`
- `src/omni_base_driver/src/cmd_vel_to_motor_node.cpp`
- `src/omni_base_driver/src/robot_odom_node.cpp`
- `src/omni_base_driver/src/steer_motor_node.cpp`
- `src/nav_pkg/scripts/nav_mode_switch_node.py`, after real parameter review

Specific packages/files that should be reused with profile cleanup:

- `src/nav_pkg/launch/navigation.launch.py`
- `src/nav_pkg/config/nav2_params.yaml`
- `src/localization_pkg/launch/fast_lio_localization.launch.py`
- `src/localization_pkg/config/gicp_localizer.yaml`
- `src/robot_agent/**`
- `src/robot_skills/**`
- `src/robot_bringup/launch/bringup.launch.py`

