# Mapping Status

## Current
- Sim mapping pipeline now targets `full_warehouse_r.usd`.
- Warehouse scene and imported robot root joint must be cleaned so only `/World/robot_sim/base_footprint` remains as an articulation.
- Isaac `ROS2PublishJointState` is disabled for mapping to avoid PhysX `getVelocities (2,6)`.
- Sim mapping pipeline uses MID360-like LiDAR.
- D455/RSD455 is removed before Play for LiDAR-only mapping because its nested
  rigid body can trigger Isaac PhysX tensor errors.
- GLIM starts with GPU config at `glim_config/sim`.
- Bag recording works when topics are present.
- `/point_cloud` must have exactly 1 publisher.
- Corrected ROS fake IMU is the current stable mapping mainline.
- Isaac true IMU is experimental only: it reads correct gravity, but still
  triggers PhysX tensor `getVelocities (2,6)` in this scene.
- PhysX GPU dynamics is disabled for the mapping USD to avoid Isaac CUDA error 700.

## Working commands
- Isaac Script Editor (BEFORE Play):
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())`
- Press Play
- Isaac Script Editor (AFTER Play):
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())`
  `exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())`
- Quickstart:
  `tools/sim/create_sim_glim_map_quickstart.sh --bag NAME --gpu --direct-cmd-vel --cleanup`
- Check:
  `ros2 topic info -v /point_cloud`
  `ros2 topic hz /livox/lidar_pc2_accumulated`
  `ros2 topic hz /livox/imu`

## Map output
- GLIM dump can be saved under `maps/sim_glim_dump_*`.
- A valid viewer map needs non-empty `graph.txt`, `values.bin`, and `traj_lidar.txt`.
- Use existing real-map tools after PLY export:
  `tools/ply_to_nav2_map.py`
  `tools/pointcloud_to_nav2_map.py`
  `tools/publish_ply_pointcloud.py`

## Known limitation
- Current drift is improved but not fully solved.
- `full_warehouse_r.usd` may include non-robot articulations; clean before Play.
- D455/RSD455 is disabled for mapping unless explicitly needed later.
- If Isaac prints CUDA error 700, restart Isaac and confirm PhysX CPU config before Play.

## IMU implementation note
- `setup_isaac_livox_imu_ros.py` no longer uses OmniGraph `IsaacReadIMU`.
- Root cause: `IsaacReadIMU` creates 2 ArticulationView registrations internally in Isaac
  Sim 5.x, causing `getVelocities (2,6)` even with `IsaacArticulationController` disabled.
- Direct `ImuSensorInterface.get_sensor_reading()` produced correct readings:
  `lin_acc ~= (-4.9, 0, 8.5)`, `orientation ~= pitch 30 deg`.
- However, it still triggers PhysX tensor `getVelocities (2,6)`.
- Therefore default mapping uses ROS-side corrected fake IMU and removes Isaac
  IMU runtime prims before Play.

## Next
- Rerun in `full_warehouse_r.usd` without Isaac true IMU.
- Verify no (2,6) error appears after Play.
- Verify corrected ROS fake IMU publishes `/livox/imu`.
- Verify GLIM graph/traj are non-empty.
