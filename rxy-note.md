## Isaac sensor 启动

打开 Isaac:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/home/ruan-x/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib"

cd /home/ruan-x/isaac-sim
./isaac-sim.sh
```

Before pressing Play, run the default cleanup and sensor preparation. This is
the current default flow and includes D455 camera-only preparation:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/inspect_rtx_lidar_pipeline.py").read())
```

Press Play.

Expected preparation output includes:

```text
Disabled D455 non-camera physics sensor: .../RSD455/Imu_Sensor
OK: D455 cameras remain active without participating in robot physics.
Found 1 articulation prim(s):
ACTIVE robot /World/robot_sim/base_footprint
```

After Play, create MID360-like LiDAR and D455 ROS publishers exactly once:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/inspect_rtx_lidar_pipeline.py").read())
```
After Play, pub odom:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_wheel_odom_ros.py").read())
```

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_fast_lio2_loc_windows.sh --cleanup
```

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_nav2_windows.sh --cleanup
```