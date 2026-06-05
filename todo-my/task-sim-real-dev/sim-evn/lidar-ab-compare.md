# LiDAR A/B 最新结论

## 结论

下一轮 sim localization / GLIM map creation 主线使用:

```text
mid360_like
```

OS1 128ch 保留为稳定 fallback / baseline。

## 为什么选 mid360_like

`mid360_like` 的 raw `/point_cloud` 单帧已经有明显高度分布:

```text
raw_pc2: 167.9 Hz, point_rate=223.8k/s, points=1333, z=[-0.05,0.52]
livox:  10.0 Hz, point_rate=200.2k/s, points=20000
```

OS1 128ch 的 raw 点云更接近平面，主要靠累计后才看起来像 3D:

```text
raw_pc2:   135.7 Hz, point_rate=352.7k/s, points=2599,  z=[-0.04,0.07]
accum_pc2:  10.0 Hz, point_rate=200.8k/s, points=20000, z=[-0.04,0.54]
```

localization 更应该看 raw 点云的空间约束，而不是只看 RViz 里累计后的视觉效果。

## 当前 topic

```text
/point_cloud                    Isaac RTX LiDAR raw PointCloud2
/livox/lidar                    Livox CustomMsg, 10 Hz, 20000 points
/livox/lidar_pc2_accumulated    RViz 检查用累计 PointCloud2
```

## 启动方式

Isaac Script Editor:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_sensors_ros.py").read())
```

当前 `setup_isaac_sensors_ros.py` 默认就是 `mid360_like`。

也可以手动分开启动:

```python
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_d455_ros.py").read())
exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())
```

ROS 检查:

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20

ros2 launch robot_sim sim_sensor_check.launch.py start_joy:=false
```

```bash
ros2 topic hz /point_cloud
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --no-arr
```

## 下一步

使用 factory scene 做 GLIM mapping:

```text
src/robot_sim/usd/scenes/robot_sim_factory_mapping.usd
```

启动:

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_like_001 \
  --cleanup
```
