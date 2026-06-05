# Sim USD / ROS Bridge 问题与最新解法

## 当前可用状态

```text
USD physics: OK
Joy-Con chain: OK
三轮逆运动学: OK
D455 RGB/depth ROS publish: OK
LiDAR raw /point_cloud: OK
Livox CustomMsg /livox/lidar: OK
RViz sensor check: OK
GLIM sim mapping quickstart: 已准备，但当前系统需要先安装/overlay glim_ros
```

## 已解决的问题

### 1. Play 后机器人前倾 / 掉落异常

解法:

- 使用 `robot_sim_physics.usd` 作为 overlay。
- articulation root 放到 `/World/robot_sim/base_footprint`。
- disable imported `root_joint`。
- disable 原始大底盘 collision，增加 `sim_chassis_collision`。
- 调整 mass / COM / friction。

### 2. Isaac Sim Play crash

原因:

```text
把 RTX LiDAR / D455 render-product ROS graph 写死进 USD 后，Isaac Sim 5.0 容易 native segfault。
```

解法:

```text
USD 只保存 robot physics + sensor mount + D455 visual asset。
传感器 publish 用 Script Editor runtime script 创建。
```

### 3. `/joint_states` / articulation 找不到

解法:

```text
ActionGraph targetPrim = /World/robot_sim/base_footprint
ArticulationController.robotPath 留空
```

### 4. 轮子命令抖动 / 0 和非 0 闪动

原因:

```text
手动往 /cmd_vel 发命令，同时 safety node 也往 /cmd_vel 发 0，两个 publisher 抢 topic。
```

解法:

```text
direct 测试用 /sim/cmd_vel
完整链路用 /teleop/cmd_vel -> /cmd_vel_raw -> /cmd_vel
```

### 5. drive/steer 命令互相干扰

解法:

```text
steer joint: position = angle, velocity = NaN
drive joint: position = NaN, velocity = wheel_rad_s
```

### 6. D455 / LiDAR topic 看不到

优先使用:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20
```

只有出现 CycloneDDS domain 创建失败时，再临时指定:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ruan-x/robot_ws/src/robot_sim/config/cyclonedds_isaac_local.xml
```

## 当前推荐流程

Isaac:

```text
Open src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd
Press Play
```

Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

ROS check:

```bash
ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false
```

GLIM mapping:

```bash
ros2 pkg list | grep -E '^glim$|^glim_ros$'
ros2 pkg executables glim_ros
```

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_like_001 \
  --cleanup
```
