# Sim PC 环境检查最新摘要

原始检查日期: 2026-05-27  
当前整理目的: 保留最新可用结论，旧的 build 阻塞和早期 package 状态不再作为当前判断依据。

## 当前环境

```text
OS: Ubuntu 22.04
ROS 2: Humble
Isaac Sim: 5.0.0
Workspace: /home/ruan-x/robot_ws
Sim ROS_DOMAIN_ID: 20
```

## 推荐 ROS 环境

一般情况使用:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20
```

只有 ROS2 CLI 出现 CycloneDDS domain 创建失败时，再临时使用:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ruan-x/robot_ws/src/robot_sim/config/cyclonedds_isaac_local.xml
```

## 当前可用状态

```text
robot_sim package: OK
robot_sim_physics.usd: OK
factory mapping scene: OK
Joy-Con / teleop chain: OK
/clock: OK
/joint_states: OK
/sim/target_joint_states: OK
D455 RGB/depth ROS topics: OK
mid360_like /point_cloud: OK
PointCloud2 -> /livox/lidar adapter: OK
RViz sensor check: OK
sim GLIM mapping quickstart: ready, but glim_ros package must be installed
```

## 当前核心检查命令

启动 Isaac 前:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20

cd /home/ruan-x/isaac-sim
./isaac-sim.sh
```

Isaac 打开:

```text
src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd
```

Press Play 后 Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

ROS 检查:

```bash
ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false
```

topic:

```bash
ros2 topic list
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /point_cloud
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --no-arr
ros2 topic echo /camera/camera/color/image_raw --once --no-arr
ros2 topic echo /camera/camera/depth/image_rect_raw --once --no-arr
```

## 当前 mapping 命令

先确认 GLIM package 存在:

```bash
ros2 pkg list | grep -E '^glim$|^glim_ros$'
ros2 pkg executables glim_ros
```

如果 `glim_ros` 不存在，先安装/overlay 真机同一套 GLIM package。sim 继续使用现有 `glim_config`，不单独创建新 config。

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_like_001 \
  --cleanup
```

完成后保存 GLIM dump。先确认 `/tmp/dump` 存在:

```bash
ls -lah /tmp/dump

mkdir -p /home/ruan-x/robot_ws/maps
cp -a /tmp/dump /home/ruan-x/robot_ws/maps/sim_glim_dump_$(date +%Y%m%d_%H%M%S)
```

如果没有 `/tmp/dump`，先检查 GLIM 是否启动并输出:

```bash
ros2 topic hz /glim_ros/aligned_points
ros2 topic list | grep glim
```

## 注意

- 当前 LiDAR 主线是 `mid360_like`。
- OS1 128ch 只作为 fallback / baseline。
- 当前 `/livox/imu` 如果由 `sim_livox_imu_node` 发布，只能用于 GLIM bringup，不用于最终 localization 精度判断。
