# Issue & Slove

## DDS / environment
- Problem: quickstart used wrong DDS/env and ROS graph was invisible or bag empty.
- Solve: use fixed sim env: source Humble + workspace, unset `RMW_IMPLEMENTATION CYCLONEDDS_URI`, `ROS_DOMAIN_ID=20`.

## GLIM package / GPU
- Problem: `glim_ros` was missing, then GPU mode needed CUDA GLIM package.
- Solve: install GLIM from Koide PPA and use `glim_config/sim` for GPU config.

## GLIM input topic
- Problem: `/livox/lidar` is `livox_ros_driver2/CustomMsg`, but sim GLIM needs `PointCloud2`.
- Solve: set sim GLIM `points_topic` to `/livox/lidar_pc2_accumulated`.

## rosbag not saved
- Problem: recorder started before required topics, or DDS/session mismatch.
- Solve: quickstart waits for LiDAR/IMU/TF and records logs to `/tmp/ros2_sim_glim_mapping_logs`.

## Joy-Con no motion
- Problem: direct teleop node name did not match `joy_teleop.yaml` params.
- Solve: direct mode uses `teleop_twist_joy_node` and remaps to `/sim/cmd_vel`.

## Multiple `/point_cloud` publishers
- Problem: running Isaac MID360 setup repeatedly creates stale Replicator writers.
- Solve: quickstart fails if publisher count is not 1; use Isaac cleanup script or restart Isaac.

## Fake IMU timestamps
- Problem: IMU timestamps rewound or drifted away from LiDAR time.
- Solve: fake IMU now follows `/clock` interpolation and ignores small clock jitter.

## Fake IMU gravity
- Problem: gravity was all on `livox_frame` Z, ignoring 30 deg LiDAR pitch.
- Solve: project gravity in `livox_frame`: x ~= -0.5 g, z ~= 0.866 g.

## IMU unit mismatch
- Problem: fake IMU used g units, but Isaac IMU publishes m/s^2.
- Solve: sim GLIM uses `acc_scale=1.0`; fake IMU also publishes m/s^2 now.

## Isaac true IMU
- Problem: fake IMU reduced drift but cannot represent real dynamics.
- Solve: use `setup_isaac_livox_imu_ros.py` in the current mainline.
- Note: clean non-robot warehouse articulations before Play, then start Isaac true IMU.

## Isaac IMU target
- Problem: `ReadIMU.inputs:imuPrim` cannot be set through `keys.SET_VALUES`.
- Solve: set the target after graph creation with `og.Controller.set`; fallback to USD relationship targets.
- Note: target path must be `usdrt.Sdf.Path`, not `pxr.Sdf.Path`.

## Isaac IMU parent
- Problem: IMU under `livox_frame` made PhysX return 2 body velocities and no `/livox/imu`.
- Solve: attach IMU to rigid `base_footprint`, with Livox local pose and `frame_id=livox_frame`.

## Remaining PhysX velocity error — ROOT CAUSE FOUND AND FIXED
- Problem: `getVelocities expected size 6, got (2,6)` printed every frame during Play.
- Investigation:
  - `list_articulations.py` confirmed only 1 articulation (`base_footprint`) in USD — before and after cleanup.
  - Full OmniGraph scan confirmed `IsaacArticulationController` was inactive.
  - (2,6) persisted with only `IsaacReadIMU` active → `IsaacReadIMU` is the sole cause.
  - `readGravity=False` did not fix it → the bug is not in the gravity path.
  - Conclusion: `IsaacReadIMU` OmniGraph node creates 2 ArticulationView tensor
    registrations internally in Isaac Sim 5.x (confirmed bug).
- Fix: removed `IsaacReadIMU` from the OmniGraph entirely.
  - `setup_isaac_livox_imu_ros.py` now uses `ImuSensorInterface.get_sensor_readings()`
    (C++ direct binding in `isaacsim.sensors.physics`) via a physics step subscription.
  - No ArticulationView is created → no tensor shape conflict.
  - The OmniGraph retains only `OnPlaybackTick → ROS2PublishImu`; the physics callback
    pushes `linearAcceleration`, `angularVelocity`, `orientation`, `timeStamp` inputs
    each physics step.
- Workflow change: `setup_isaac_livox_imu_ros.py` must run **BEFORE** Play (no timeline restart).

## PhysX CUDA error 700
- Problem: Isaac freezes with `cudaErrorIllegalAddress` / `Simulation cannot continue`.
- Cause: PhysX tensor shape error from multiple articulations can cascade into CUDA/render graph failure.
- Solve: open `full_warehouse_r.usd`, run PhysX CPU config and foreign-articulation cleanup before Play.
- Commands: run `configure_mapping_physx_cpu.py`, `cleanup_foreign_articulations.py`, then Play.

## LiDAR / IMU time split
- Problem: GLIM saw LiDAR at ~681s and IMU at ~139s; rosbag did not save.
- Cause: MID360 was created before the IMU script; IMU cleanup stopped/restarted the Isaac timeline and broke the LiDAR writer/time epoch.
- Solve: pre-clean scene, then after Play run true IMU setup before MID360; quickstart waits for `/point_cloud`, `/livox/lidar`, `/livox/lidar_pc2_accumulated`, and `/livox/imu`.

## offline_viewer empty
- Problem: saved dump had submap point files but empty global graph/traj.
- Solve: check `graph.txt`, `traj_lidar.txt`, `values.bin`; export via GLIM viewer/real map tools when graph is valid.
