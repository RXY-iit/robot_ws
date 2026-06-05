# robot_sim USD 最新说明

## 当前主 USD

```text
src/robot_sim/usd/robot_sim_physics.usd
```

用途:

- 基于 `robot_sim_flattened.usda` 的 physics overlay。
- 可 Play，不再把 RTX LiDAR / camera render-product graph 写死进 USD。
- 机器人物理、关节、ROS2 clock、joint bridge 已可用。

factory mapping 场景:

```text
src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd
```

用途:

- sublayer `robot_sim_physics.usd`。
- 增加本地工厂几何: 地面、墙、柱子、货架、箱子、障碍。
- 用于下一轮 GLIM / localization / mapping 测试。

## USD 里固定保存的内容

```text
/World/robot_sim/base_footprint
  Articulation root
  sim_chassis_collision
  base_link
    camera_bottom_screw_frame/camera_link/d455_sensor
    livox_frame/mid360_mount

/World/ROS2_ActionGraph
  /clock publisher
  /joint_states publisher
  /sim/target_joint_states subscriber
  IsaacArticulationController

/physicsScene
/World/GroundPlane
```

## 不写进 USD 的内容

以下内容运行时由 Script Editor 创建:

```text
D455 RGB/depth ROS publishers
RTX LiDAR sensor
PointCloud2 publisher
```

原因:

```text
Isaac Sim 5.0 中，手写 sensor render-product graph 到 USD 后 Play 可能 crash。
script 启动更稳定，也方便重复调试。
```

## 当前传感器主线

D455:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

LiDAR:

```text
当前主线: mid360_like
raw topic: /point_cloud
adapter output:
  /livox/lidar
  /livox/lidar_pc2_accumulated
```

`mid360_like` 不是官方 MID360 模型，只是当前更适合 localization 测试的近似 profile。OS1 128ch 保留为 fallback。

## Isaac sensor 启动

打开 Isaac:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20

cd /home/ruan-x/isaac-sim
./isaac-sim.sh
```

打开:

```text
src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd
```

Press Play 后 Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

这个脚本当前默认:

```text
D455 + mid360_like
```

## ROS bringup / 检查

传感器 + RViz 检查:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20

ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false
```

Joy-Con 手动驾驶:

```bash
ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=true
```

隔离运动测试:

```bash
ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false direct_cmd_vel:=true
ros2 topic pub -r 20 /sim/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.15, z: 0.0}, angular: {z: 0.0}}"
```

检查:

```bash
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /sim/target_joint_states
ros2 topic hz /point_cloud
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --no-arr
```

## 三轮控制链路

完整链路:

```text
/joy
  -> /teleop/cmd_vel
  -> nav_mode_switch_node
  -> /cmd_vel_raw
  -> cmd_vel_safety_node
  -> /cmd_vel
  -> sim_omni_drive_command_node
  -> /sim/target_joint_states
  -> Isaac Articulation Controller
```

`/sim/cmd_vel` 只用于 direct 测试。真机同等入口是 `/cmd_vel`。

`/sim/target_joint_states` 控制 6 个自由度:

```text
wheel_right_steer  position
wheel_left_steer   position
wheel_back_steer   position
wheel_right_drive  velocity
wheel_left_drive   velocity
wheel_back_drive   velocity
```

unused command 用 `NaN`，避免 Isaac 同时收到 position 和 velocity 命令。

## GLIM map creation

前提:

```bash
ros2 pkg executables glim_ros
```

如果 package 不存在，需要先安装/overlay 真机同一套 GLIM。sim 不维护单独 GLIM config，继续使用 `/home/ruan-x/robot_ws/glim_config`。

当前使用:

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_like_001 \
  --cleanup
```

注意:

- 当前 `/livox/imu` 是 `sim_livox_imu_node` 提供的 fake IMU，只用于 bringup。
- 最终 localization 精度判断前，应替换为 Isaac simulated IMU。
