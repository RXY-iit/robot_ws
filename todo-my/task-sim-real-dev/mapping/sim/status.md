# Mapping 当前状态

## 当前结论

默认建图流程已经定义并验证为：

```text
D455 camera-only recording + Isaac true IMU + MID360-like RTX LiDAR + GLIM GPU
```

D455 只记录彩色图、深度图和 CameraInfo，不参与机器人 PhysX，也不作为 GLIM
输入。LiDAR-only 仅用于性能对比和故障隔离；corrected fake IMU 仅作为备用诊断工具。

最新默认流程成功运行：

```text
bag: sim_bag/sim_factory_mid360_trueimu_d455_002
map: maps/sim_glim_dump_sim_factory_mid360_trueimu_d455_002_20260607_230507
log: log/sim_glim_mapping_sim_factory_mid360_trueimu_d455_002_20260607_230507
```

- 无 CUDA 700、PhysX `(2,6)`、RSD455 missing rigid body 或 nested rigid-body 错误
- rosbag：`1.1 GiB`、`89.35 s`、`23,841` 条消息
- D455 color/depth/CameraInfo 均成功记录，约 `6.3 Hz`
- `/point_cloud` 和 `/livox/lidar` 约 `6.41 Hz`
- GLIM 输出约 `6.29 Hz`
- GLIM 自动保存成功：`38` 个 submap、`561` 帧轨迹

## 当前系统

- 场景：`full_warehouse_r.usd`
- Robot articulation：仅 `/World/robot_sim/base_footprint`
- D455：camera-only，Play 前停用内部 `Imu_Sensor`，再移除 RigidBody/Mass API
- LiDAR：MID360-like RTX，`ROS2RtxLidarHelper + fullScan=True`，`1x1` render product
- IMU：Isaac true IMU，Python `ImuSensorInterface` 读取，时间戳复用 `/clock`
- GLIM：GPU config，路径 `glim_config/sim`
- 控制：Joy-Con 或 `--direct-cmd-vel`

默认数据链：

```text
Isaac /point_cloud
  -> pointcloud2_to_livox_custom_msg_node
  -> /livox/lidar
  -> /livox/lidar_pc2_accumulated
  -> GLIM

Isaac true IMU -> /livox/imu -> GLIM
D455 cameras -> color/depth/CameraInfo -> rosbag only
```

## 默认启动规则

1. 使用指南中的统一 ROS2 Bridge 环境启动 Isaac。
2. Play 前清理 foreign articulation 和旧 runtime prim。
3. Play 前运行 `prepare_d455_camera_only.py`，必须先停用 D455 内部 IMU。
4. Play 前创建 Isaac true IMU。
5. Press Play 后只运行一次 MID360 setup，再运行一次 D455 ROS setup。
6. 确认 `/point_cloud` Publisher count 为 1，且 D455、LiDAR、IMU 时间戳持续前进。
7. 使用 quickstart 的 `--record-d455` 启动默认建图。
8. GLIM pane 中按 `Ctrl-C`，等待 `saved` 和 `MAP SAVE OK` 后再关闭窗口。
9. 出现 CUDA 700 或 `(2,6)` 后立即重启 Isaac，不继续复用当前 session。

默认 quickstart：

```bash
tools/sim/create_sim_glim_map_quickstart.sh \
  --bag sim_factory_mid360_trueimu_d455_XXX \
  --gpu \
  --direct-cmd-vel \
  --record-d455 \
  --cleanup
```

## 已知残余问题

- D455 与 LiDAR 同时运行会使整体仿真实时速度下降约 `12%–13%`。
- 当前 D455 设置目标为 `640x360 @ 10 Hz`，实际约 `6.3 Hz`。
- LiDAR/GLIM 实际输出约 `6.3–6.4 Hz`，低于目标 10 Hz，但当前建图稳定。
- CameraInfo distortion model 当前回退到默认 `plumb_bob`，不能直接视为精确标定值。
- RTX pipeline 仍可能输出少量 DispatchSync/illegal-cycle 警告；没有 CUDA 700、
  数据停止或时间戳停止时可记录并继续观察。
- 仍需更长时间、多次重启后的重复稳定性测试。

## 下一步

1. 使用默认 D455 流程进行 5–10 分钟以上长时间建图。
2. 检查 offline viewer 中地图闭环、重影、地面平整度和 Z 漂移。
3. 优化 LiDAR 与 D455 吞吐，同时保持单 publisher 和统一 `/clock`。
4. 补充可靠的 D455 相机内外参与 distortion 配置。
5. 使用现有实机工具导出 PLY 和 Nav2 2D map。
