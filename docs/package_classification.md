# Package Classification

Generated: 2026-07-13

Classification labels:

- 可直接复用: should move to real OS mostly unchanged.
- 调整后复用: architecture is reusable, but parameters, launch defaults, paths, or hardware profiles must be separated.
- Simulation 专用: do not migrate to real runtime, except as simulation test tooling.
- 第三方: vendor/upstream package; keep as dependency or submodule/vendor import.
- 待确认: role is unclear or old/experimental.

| Package / path | Classification | Reason | Migration action |
|---|---|---|---|
| `src/robot_agent` | 调整后复用 | Agent node is mode-aware and mostly ROS-topic based. It monitors sim/real-neutral topics, but runtime mode/config paths need real profile. | Keep package. Add real launch/profile and avoid sim map defaults. |
| `src/robot_skills` | 调整后复用 | Skill registry already has `sim_supported`/`real_supported`. Navigation and semantic skills are reusable; perception/lift/camera skills have real gaps. | Keep package. Split skill support matrix by real hardware readiness. |
| `src/nav_pkg` | 调整后复用 | Nav2 launch and mode switch are central to both sim and real. Parameters are partly real-lab tuned, but still hardcoded in launch/config. | Keep package. Create separate `nav2_params_sim.yaml` and `nav2_params_real.yaml`, or profile overlay. |
| `src/safety_layer` | 可直接复用 | Hardware-neutral `/cmd_vel_raw` -> `/cmd_vel` safety gate. | Use in real robot. Real limits must be reviewed, but package design is directly reusable. |
| `src/localization_pkg` | 调整后复用 | GICP/FAST-LIO bridge is relevant to real robot. Current defaults point at sim maps and sim-specific tuning. | Keep package. Parameterize map bundle, initial pose, thresholds, LiDAR mode. |
| `src/robot_bringup` | 调整后复用 | Real hardware bringup package. Contains Livox/RealSense launch and real state publisher. | Use as real entry point, but align params with sim contract and real hardware IDs. |
| `src/robot_description` | 可直接复用 | Real URDF/TF description is the right source for real robot frames. | Use on real OS. Confirm frame names match sim contract. |
| `src/omni_base_driver` | 可直接复用 | Real base driver consuming `/cmd_vel` and publishing `/wheel_odom`. | Use on real OS. Validate kinematics, wheel radius, steering limits, odometry frame. |
| `src/omni_mppi_critics` | 调整后复用 | Custom MPPI critic exists but is disabled due to observed controller crash. | Do not enable on real until isolated stability tests pass. |
| `src/mission_manager` | 待确认 | Older Phase 4 BasicNavigator manager. Agent/skills seem to supersede it. | Decide whether to archive, keep for tests, or fold into `robot_agent` docs. |
| `src/my_messages` | 可直接复用 | Custom drive/steer messages are required by real base driver and sim bridge. | Keep as shared interface package. |
| `src/tf_tools` | 待确认 | Static TF test helpers. Some test-only transforms can conflict with real localization. | Keep for diagnostics only. Do not launch in real autonomous stack unless explicitly needed. |
| `src/serial_transciever` | 调整后复用 | Real lift/manipulator/camera-swing serial nodes plus some ROS-only joy command nodes reused by sim. | Separate real serial launch from sim command launch. Confirm package spelling before long-term maintenance. |
| `src/robot_sim` | Simulation 专用 | Isaac Sim integration, sim controllers, sim URDF, USD scripts, sim sensor adapters. | Do not install on real robot runtime except optional offline test tools. |
| `src/fast_lio` | 第三方 | FAST-LIO/Livox mapping/localization algorithm package. | Keep as third-party dependency; pin upstream/version and document patches. |
| `src/livox_ros_driver2` | 第三方 | Livox device driver. | Keep as real dependency; align xfer format with localization launch. |
| `src/realsense-ros/realsense2_camera` | 第三方 | Intel RealSense driver. | Keep as real dependency; verify D435/D455 profile. |
| `src/realsense-ros/realsense2_camera_msgs` | 第三方 | RealSense message definitions. | Keep as dependency. |
| `src/realsense-ros/realsense2_description` | 第三方 | RealSense xacro/meshes. | Keep if needed by `robot_description`; large meshes may belong in vendor subtree. |
| `src/DynamixelSDK/dynamixel_sdk` | 第三方 | ROBOTIS SDK. | Keep as dependency or external source. |
| `src/DynamixelSDK/dynamixel_sdk_custom_interfaces` | 第三方 | SDK example interfaces used by driver. | Keep if real steer driver depends on it. |
| `src/DynamixelSDK/dynamixel_sdk_examples` | 第三方 / 调整后复用 | Vendor examples are also used by `omni_base_driver`. | Keep minimal required nodes/interfaces; avoid relying on sample code long term. |
| `src/om_modbus_master_V201/om_modbus_master` | 第三方 | Oriental Motor Modbus package. | Keep only on real OS/hardware profile. |
| `src/om_modbus_master_V201/om_msgs` | 第三方 | Oriental Motor message/action interfaces. | Keep with Modbus package. |
| `models/omniverse_assets` | Simulation 专用 | Collected USD/texture assets for Isaac Sim. | Move out of main runtime repo or manage with Git LFS/artifact storage. |
| `sim_bag` | Simulation 专用 | Large sim rosbag data. | Do not migrate to real runtime repo; store externally. |
| `maps/sim_*` | Simulation 专用 / 调整后复用 | Simulation-generated maps are not valid real maps, but map bundle structure is reusable. | Keep as sim fixtures; create real map bundle separately. |

## Directly Reusable Packages

Strong candidates for direct reuse:

- `src/safety_layer`
- `src/robot_description`
- `src/omni_base_driver`
- `src/my_messages`

Reusable after profile cleanup:

- `src/robot_agent`
- `src/robot_skills`
- `src/nav_pkg`
- `src/localization_pkg`
- `src/robot_bringup`
- `src/serial_transciever`

