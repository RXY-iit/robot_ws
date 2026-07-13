# Simulation System Inventory

Generated: 2026-07-13

## Scope

This inventory describes the current `robot_ws` as seen from the simulation OS. It focuses on the stack that has been run for Isaac Sim navigation, localization, agent execution, and skill execution.

## High-Level System

The workspace is a mixed simulation/real-robot ROS 2 workspace. The main reusable runtime stack is:

- `robot_agent`: L3 mission agent and dashboard-facing command/state layer.
- `robot_skills`: L2 typed skill executors and semantic/navigation/perception logic.
- `nav_pkg`: Nav2 launch, controller/planner configuration, and AUTO/MANUAL command relay.
- `safety_layer`: velocity watchdog, speed/acceleration/jerk clamp, and emergency stop relay.
- `localization_pkg`: GICP prior-map localization, map server, pointcloud-to-laserscan, optional FAST-LIO2 integration.
- `robot_description`: real robot URDF and TF structure.
- `robot_bringup`: real hardware bringup for Livox, RealSense, state publisher, teleop, and lift.
- `omni_base_driver`: real base command and odometry bridge for drive/steer motors.

The simulation-specific layer is:

- `robot_sim`: Isaac Sim helper launch, simulated omni drive command bridge, simulated lift/camera bridge, pointcloud adapter, Isaac setup scripts, sim URDF, and USD assets.
- `src/robot_sim/usd/**`: robot USD, physics USD, real-lab scene USD layers, and flattened USD.
- `models/omniverse_assets/**`: collected Omniverse/Isaac/SimReady assets and textures.
- `sim_bag/**`: large simulation rosbag databases.
- `maps/sim_*` and `maps/sim_real_lab_001`: simulation-generated 2D/3D maps and metadata.

## Main Launch Relationships

### Simulation manual bringup

Entry point:

- `src/robot_sim/launch/sim_manual_bringup.launch.py`

Launches:

- `robot_state_publisher` using `src/robot_sim/urdf/robot_sim.urdf.xacro`
- optional `joy_node`
- `teleop_twist_joy`
- `nav_pkg/nav_mode_switch_node.py`
- `safety_layer/cmd_vel_safety_node`
- `robot_sim/sim_omni_drive_command_node`
- optional `robot_sim/pointcloud2_to_livox_custom_msg_node`
- optional real-stack joy nodes from `serial_transciever`
- `robot_sim/sim_lift_command_node`

Command chain in normal sim mode:

```text
/teleop/cmd_vel or /nav2/cmd_vel
  -> /cmd_vel_raw
  -> /cmd_vel
  -> sim_omni_drive_command_node
  -> /sim/target_joint_states
  -> Isaac Articulation Controller
```

Direct sim mode can bypass `nav_mode_switch_node` and `cmd_vel_safety_node`:

```text
/sim/cmd_vel -> sim_omni_drive_command_node -> /sim/target_joint_states
```

Direct mode is useful for isolated Isaac motion tests, but it should not be used as the real-robot migration contract.

### Navigation

Entry point:

- `src/nav_pkg/launch/navigation.launch.py`

Launches:

- `nav2_controller/controller_server`
- `nav2_planner/planner_server`
- `nav2_behaviors/behavior_server`
- `nav2_bt_navigator/bt_navigator`
- Nav2 lifecycle manager
- `nav_mode_switch_node`
- `cmd_vel_safety_node`
- `depth_image_proc::PointCloudXyzNode`

Nav command chain:

```text
Nav2 controller_server / behavior_server
  -> /nav2/cmd_vel
  -> nav_mode_switch_node
  -> /cmd_vel_raw
  -> cmd_vel_safety_node
  -> /cmd_vel
  -> sim controller or real base driver
```

Local obstacle chain:

```text
/camera/camera/depth/image_rect_raw + /camera/camera/depth/camera_info
  -> depth_image_proc
  -> /camera/depth/points
  -> Nav2 local_costmap VoxelLayer
```

### Localization

Entry point:

- `src/localization_pkg/launch/fast_lio_localization.launch.py`

Default:

```text
/livox/lidar PointCloud2
  -> gicp_localizer_node
  -> map->odom TF
  -> pointcloud_to_laserscan
  -> /scan
```

Optional FAST-LIO mode:

```text
/livox/lidar Livox CustomMsg
  -> fast_lio
  -> /fast_lio/odometry
  -> livox_custom_to_pc2_node
  -> /livox/lidar_pc2
  -> gicp_localizer_node + pointcloud_to_laserscan
```

Map server:

```text
map_server -> /map -> Nav2 global_costmap StaticLayer
```

Current launch defaults still point at:

- `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml`
- `/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply`

For the real-lab sim map, the user has an active map bundle:

- `maps/sim_real_lab_001/nav2.yaml`
- `maps/sim_real_lab_001/map_3d.ply` if present locally

### Agent and skill layer

Agent entry point:

- `src/robot_agent/launch/agent_dashboard.launch.py`
- `src/robot_agent/robot_agent/nodes/agent_node.py`
- `src/robot_agent/robot_agent/nodes/dashboard_node.py`

Agent input/output:

- subscribes `/agent/text_command`, `/agent/cancel`
- publishes `/agent/status`, `/agent/mission_events`, `/agent/mission_state`, `/agent/health_state`, `/agent/human_prompt`
- monitors `/robot_mode`, `/safety_status`, `/gicp_loc/score`, `/cmd_vel`, `/nav2/cmd_vel`, `/gicp_loc/pose`, `/initialpose`
- queries `/controller_server/get_state`

Skill layer:

- `src/robot_skills/config/skill_registry.yaml`
- `src/robot_skills/robot_skills/executors/navigation.py`
- `src/robot_skills/robot_skills/executors/{system,semantic,perception,camera,lift,localization}.py`

The skill registry already distinguishes `sim_supported` and `real_supported`. Some manipulation/perception skills are explicitly sim-only or not yet real-supported.

## Maps and Assets

Active/important sim map bundle:

- `maps/sim_real_lab_001/nav2.yaml`
- `maps/sim_real_lab_001/nav2.pgm`
- `maps/sim_real_lab_001/semantic_map.yaml`
- `maps/sim_real_lab_001/semantic_segments.yaml`
- `maps/sim_real_lab_001/navigation_annotations.yaml`
- `maps/sim_real_lab_001/metadata.yaml`

Simulation scene:

- `src/robot_sim/usd/scenes/robot_sim_real_lab.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_root.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_structure.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_asset_refs.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_lighting.usd`
- `src/robot_sim/usd/scenes/real_lab/real_lab_materials.usd`

## Current Migration Verdict

Current Simulation workspace status: **整理后适合**.

Reason:

- The reusable architecture is already close to real deployment: agent -> skills -> Nav2 -> safety -> base driver is cleanly layered.
- The bottom navigation stack has already been run on the real OS with the same logic, according to user context.
- Several configs intentionally use real-lab-oriented values and real topics.
- However, sim assets, bags, Isaac APIs, hardcoded absolute paths, simulation-generated maps, `/clock` assumptions, and sim controller nodes are mixed into the same repository/workspace and must be separated or parameterized before direct deployment.

