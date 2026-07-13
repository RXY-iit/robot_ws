# Sim/Real Interface Contract

Generated: 2026-07-13

This file defines the interfaces that Simulation and Real Robot should keep compatible. The goal is that L3 agent, L2 skills, Nav2, safety, and hardware/sim base drivers can be swapped without changing upstream behavior.

## Frames

| Frame | Required in sim | Required in real | Owner | Notes |
|---|---:|---:|---|---|
| `map` | yes | yes | map server + localization | Global planning frame. |
| `odom` | yes | yes | localization/odom stack | Continuous local frame. `map -> odom` should be published by localization. |
| `base_footprint` | yes | yes | robot description + odom | Main 2D base frame used by Nav2 and skills. |
| `base_link` | optional/legacy | yes if URDF uses it | robot description | Avoid ambiguous use between `base_link` and `base_footprint`. |
| `livox_frame` | yes | yes | robot description / sensor driver | LiDAR frame for Livox MID360. Extrinsics must match real mount. |
| `camera_link` | yes | yes | robot description / RealSense driver | Base camera frame. |
| `camera_*` optical frames | yes | yes | RealSense/Isaac camera bridge | Must match RealSense driver conventions if reused by perception. |

Required TF timing:

- `map -> odom`: localization publishes latest valid transform; current GICP rebroadcast target is 30 Hz.
- `odom -> base_footprint`: wheel odometry or fused odometry should be fresh enough for Nav2 controller at 10 Hz.
- Static sensor frames: latched/static TF is acceptable.

## Command Topics

| Topic | Type | Direction | QoS | Expected frequency | Timing expectation | Contract |
|---|---|---|---|---:|---|---|
| `/teleop/cmd_vel` | `geometry_msgs/Twist` | teleop -> mode switch | default depth 10 | operator driven | dead-man controlled | Manual velocity command input. |
| `/nav2/cmd_vel` | `geometry_msgs/Twist` | Nav2 -> mode switch | default depth 10 | controller 10 Hz | should not go directly to motors | Autonomous velocity proposal. |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | mode switch -> safety | default depth 10 | 10 Hz in AUTO, operator rate in MANUAL | safety watchdog currently 0.5 s | Only `nav_mode_switch_node` should publish in normal mode. |
| `/cmd_vel` | `geometry_msgs/Twist` | safety -> base driver/sim driver | default depth 10 | up to input rate | zero on watchdog/estop | Only safety-approved base command. |
| `/sim/cmd_vel` | `geometry_msgs/Twist` | test -> sim driver | default depth 10 | test only | not part of real contract | Direct Isaac test bypass. |

Real robot must not bypass `/cmd_vel_raw -> safety_layer -> /cmd_vel`.

## Mode and Safety Topics

| Topic | Type | Direction | QoS | Expected frequency | Contract |
|---|---|---|---|---:|---|
| `/robot_mode` | `std_msgs/String` | mode switch -> all | default depth 10 | 0.5 Hz heartbeat + on change | Values: `AUTO`, `MANUAL`. |
| `/nav_mode/command` | `std_msgs/String` | agent/operator -> mode switch | default depth 10 | event based | Values: `AUTO`, `MANUAL`. |
| `/emergency_stop` | `std_msgs/Bool` | operator/mission -> safety | default depth 10 | event based | `true` forces zero command. |
| `/safety_status` | `std_msgs/String` | safety -> all | default depth 10 | 1 Hz | Values currently include `OK`, `WATCHDOG`, `ESTOP`. |
| `/motion_policy/status` | `std_msgs/String` | mode switch -> debug/agent | default depth 10 | when AUTO cmd received | Human/debug status for bounded motion policy. |

## Navigation Actions and Services

| Name | Type | Direction | Contract |
|---|---|---|---|
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | skills -> Nav2 | Primary navigation action. Target frame should be `map`. |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | skills -> Nav2 | Used by skill layer for route checks/preview. |
| `/navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | mode switch/skills -> Nav2 | Manual mode should cancel active Nav2 goals. |
| `/controller_server/get_state` | `lifecycle_msgs/srv/GetState` | agent -> Nav2 | Agent health check for local planner lifecycle. |

## Localization and Map Topics

| Topic | Type | Direction | QoS | Expected frequency | Contract |
|---|---|---|---|---:|---|
| `/map` | `nav_msgs/OccupancyGrid` | map_server -> Nav2/skills | transient local expected | static / on update | Must match `map` frame and map yaml origin/resolution. |
| `/gicp_loc/pose` | `geometry_msgs/PoseStamped` | localization -> agent/skills | default depth 10 | GICP rate, currently ~2 Hz | Agent uses this as trusted pose. |
| `/gicp_loc/score` | `std_msgs/Float32` | localization -> agent/skills | default depth 10 | GICP rate | Health threshold must be real-specific. |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz/operator/agent -> localization | default depth 10 | event based | Used for manual/recovery initialization. |
| `/wheel_odom` | `nav_msgs/Odometry` | base odom -> Nav2/diagnostics | default depth 10 | should be >= 20 Hz real target | Frame `odom`, child `base_footprint` preferred. |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan -> Nav2/debug | default depth 10 | derived from lidar, ~10 Hz target | Optional; current local costmap uses depth points, but scan is useful for diagnostics. |

## Sensor Topics

| Topic | Type | Sim source | Real source | Expected frequency | Contract |
|---|---|---|---|---:|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` or `livox_ros_driver2/msg/CustomMsg` | Isaac bridge | Livox driver | ~10 Hz or driver-specific | Type depends on `xfer_format`; launch must make mode explicit. |
| `/livox/lidar_pc2` | `sensor_msgs/PointCloud2` | relay | relay from CustomMsg | follows lidar | Used when FAST-LIO needs CustomMsg but GICP needs PointCloud2. |
| `/livox/imu` | `sensor_msgs/Imu` | Isaac true IMU bridge | Livox IMU | driver-specific high rate | Time base and frame must match LiDAR. |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | Isaac D455/D435 bridge | RealSense driver | 15-30 Hz | Input to depth pointcloud generation. |
| `/camera/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Isaac camera bridge | RealSense driver | same as depth | Required by `depth_image_proc`. |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | `depth_image_proc` | `depth_image_proc` | 15-30 Hz target | Nav2 local VoxelLayer input. |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | Isaac camera bridge | RealSense driver | 15-30 Hz | Perception input. |

## Sim-Only Topics

| Topic | Type | Reason |
|---|---|---|
| `/sim/target_joint_states` | `sensor_msgs/JointState` | Isaac Articulation Controller command input. |
| `/sim/drive_target` | `my_messages/DriveMotor` | Sim debug mirror of drive command. |
| `/sim/steer_target` | `my_messages/SteerMotor` | Sim debug mirror of steer command. |
| `/sim/arm_joint_states` | likely `sensor_msgs/JointState` | Sim arm/lift bridge. |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time only. Real runtime should use system/ROS time unless explicitly bag replaying. |

## QoS Baseline

The current custom nodes mostly use default ROS 2 QoS depth 10. Exceptions:

- Skill navigation marker publishers use reliable + transient local QoS.
- Nav2 map/static layers expect transient local behavior for `/map`.
- Real sensor drivers may use sensor-data QoS; every subscriber to camera/LiDAR data should be checked for compatibility.

Migration requirement:

- Record `ros2 topic info -v` for every contract topic in both sim and real.
- Any mismatch in reliability/durability for `/livox/lidar`, `/camera/depth/points`, `/map`, and `/tf_static` must be resolved before autonomous real tests.

