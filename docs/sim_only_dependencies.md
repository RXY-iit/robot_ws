# Simulation-Only Dependencies

Generated: 2026-07-13

The following dependencies are specific to Isaac Sim or simulation-generated data. They should not be required by the real robot runtime.

## Isaac Sim APIs and Scripts

Do not migrate to real robot OS runtime:

- `src/robot_sim/scripts/setup_isaac_lidar_check_box.py`
- `src/robot_sim/scripts/setup_isaac_mid360_ros.py`
- `src/robot_sim/scripts/setup_isaac_mid360_like_ros.py`
- `src/robot_sim/scripts/setup_isaac_sensors_ros.py`
- `src/robot_sim/scripts/setup_isaac_d455_ros.py`
- `src/robot_sim/scripts/sim/*.py`

These scripts use Isaac/Omniverse scene APIs, RTX LiDAR/camera setup, PhysX configuration, or OmniGraph cleanup. They are valid simulation tools only.

## USD and Simulation Assets

Do not migrate to real robot runtime:

- `src/robot_sim/usd/robot_sim.usd`
- `src/robot_sim/usd/robot_sim_physics.usd`
- `src/robot_sim/usd/robot_sim_flattened.usda`
- `src/robot_sim/usd/scenes/robot_sim_real_lab.usd`
- `src/robot_sim/usd/scenes/full_warehouse_r.usd`
- `src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd`
- `src/robot_sim/usd/scenes/real_lab/*.usd`
- `src/robot_sim/usd/assets/**`
- `models/omniverse_assets/**`

These files define the digital twin, physics model, lighting, furniture, collected assets, and textures. They should live in a simulation artifact store or Git LFS-backed simulation repository, not in the real runtime deployment.

## Simulation Controllers

Do not migrate as real control nodes:

- `src/robot_sim/robot_sim/sim_omni_drive_command_node.py`
- `src/robot_sim/robot_sim/sim_lift_command_node.py`
- `src/robot_sim/robot_sim/sim_livox_imu_node.py`
- `src/robot_sim/robot_sim/pointcloud2_to_livox_custom_msg_node.py` if used only to mimic Livox output from Isaac pointclouds

Sim controller topics:

- `/sim/target_joint_states`
- `/sim/drive_target`
- `/sim/steer_target`
- `/sim/arm_joint_states`
- `/sim/cmd_vel`

The real contract is `/cmd_vel` into `omni_base_driver`, not `/sim/target_joint_states`.

## Simulation Sensors and Ground Truth

Do not rely on these in real:

- Isaac RTX LiDAR pointclouds.
- Isaac D455/D435 camera output.
- Isaac true IMU.
- Any ground-truth pose, perfect TF, or simulator-authored robot pose.
- `/clock` from Isaac Sim.

Real replacements:

- Livox MID360 driver for `/livox/lidar` and `/livox/imu`.
- RealSense driver for `/camera/camera/*`.
- Real wheel odometry from `omni_base_driver`.
- Real localization from FAST-LIO/GICP/AMCL or fused estimator.
- System time or robot clock, not sim time.

## Simulation Maps and Bags

Do not use as real deployment maps:

- `maps/sim_factory_mid360_like_gpu_trueimu_011/**`
- `maps/sim_glim_dump_*`
- `maps/sim_real_lab_001/**` as final real map
- `sim_bag/**`

The map bundle structure is reusable, but the map data must be generated from real robot sensor data.

## `/clock` Policy

Simulation:

- `use_sim_time` may be `true` when Isaac publishes `/clock`.
- Bags may use `/clock` for deterministic replay.

Real:

- `use_sim_time` should be `false`.
- Do not require `/clock`.
- Every launch file should make `use_sim_time` explicit and default to `false` for real.

## Summary: Cannot or Need Not Migrate

重点不迁移:

- Isaac Sim API
- USD scene/model files
- Simulation sensor setup
- ground truth pose/time
- `/clock` dependency
- simulation controller nodes
- Omniverse/SimReady assets
- sim rosbag databases

