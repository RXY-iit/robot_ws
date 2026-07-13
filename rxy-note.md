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
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_wheel_odom_ros.py").read())
```
After Play, pub odom:

```python
# exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_wheel_odom_ros.py").read())
```

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_fast_lio2_loc_windows.sh --cleanup
```

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_nav2_windows.sh --cleanup
```



```bash
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset PYTHONHOME
unset LD_PRELOAD
unset CYCLONEDDS_URI
unset RMW_IMPLEMENTATION

cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

```

##  启动 Agent + Dashboard
ros2 launch ...              = 快，但依赖当前 shell 环境必须干净
run_agent_dashboard_sim_clean.sh = 稍微多一步，但更稳定，推荐
```bash
cd /home/ruan-x/robot_ws
# export ROBOT_MAP_ID=warehouse
export ROBOT_MAP_ID=sim_real_lab_001
tools/run_agent_dashboard_sim_clean.sh
```
```bash
cd /home/ruan-x/robot_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch robot_agent agent_dashboard.launch.py mode:=sim
```
```text
http://127.0.0.1:8765
```
### 可选: 启动 llama.cpp 本地 LLM fallback

如果要测试 LLM fallback，先在单独终端启动 `llama-server`。模型文件名后续可以替换；这里用 3B GGUF 作为推荐形态：

```bash
~/local_llm/llama.cpp/build/bin/llama-server \
  -m ~/local_llm/models/qwen2.5-3b-instruct-q4_k_m.gguf \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 4096
```
```bash
curl http://127.0.0.1:8080/v1/models
```



