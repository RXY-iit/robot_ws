# Mapping 当前调试总结

## 原始目标

在 Isaac Sim 中尽量复用实机流程，使用模拟 LiDAR、Isaac true IMU、机器人 TF、
Joy-Con 控制和实机 GLIM 配置生成可保存、可回放、可继续用于导航处理的地图。

目标包括：

- 保持实机同名 topic：`/livox/lidar`、`/livox/imu`、TF、`/clock`。
- 记录有效 rosbag。
- 使用 GLIM GPU 完成建图并保存 dump。
- 最终使用 Isaac true IMU，而不是长期依赖 fake IMU。

当前结论：建图主线已经跑通，目标可实现。当前默认启动流程已经定义为：

```text
D455 camera-only recording + Isaac true IMU + MID360-like RTX LiDAR + GLIM GPU
```

D455 不参与 GLIM，只作为 rosbag 图像记录旁路。LiDAR-only 与 corrected fake IMU
保留为性能对比和故障隔离方案，不再是默认流程。

## 最新成功运行

```text
bag:
sim_bag/sim_factory_mid360_trueimu_d455_002

map:
maps/sim_glim_dump_sim_factory_mid360_trueimu_d455_002_20260607_230507

runtime log:
log/sim_glim_mapping_sim_factory_mid360_trueimu_d455_002_20260607_230507
```

核验结果：

- Isaac 运行期间未出现 CUDA error 700。
- 未出现 PhysX `getVelocities (2,6)` 错误。
- rosbag 有效：约 `1.1 GiB`、`89.35 s`、`23,841` 条消息。
- `/point_cloud`、Livox adapter 输出、true IMU 和 GLIM 输出均有数据。
- D455 color/depth/CameraInfo 均成功记录，约 `6.3 Hz`。
- GLIM 自动保存成功：`38` 个非空 submap、`561` 帧 LiDAR/IMU 轨迹。
- 地图包含有效 `graph.bin`、`values.bin`、轨迹和 submap 点云。
- 轨迹时间约 `56.0 s`，覆盖约 X `6.6 m`、Y `7.2 m`，Z 变化约 `0.65 m`。
- `/point_cloud`、Livox、GLIM 输出约 `6.3–6.4 Hz`。

这是当前默认 D455 流程的成功验证，但仍需更长时间和多次重启后的重复运行。

后续 A/B 测试：

- LiDAR-only `1x1`：无 CUDA 700、无 `(2,6)`，`/point_cloud ~= 7.33 Hz`，
  GLIM 输出约 `7.05 Hz`，说明 `1x1` render product 有正向效果。
- D455 camera-only `_002`：先停用内部 `Imu_Sensor`，再移除 RigidBody/Mass API；
  无 `(2,6)`、nested RigidBody、RSD455 missing rigid body 或 CUDA 700。
- 加入 D455 后，LiDAR/GLIM 相比 LiDAR-only 下降约 `12%–13%`，但数据链稳定。

## 当前系统设计

### 环境

```bash
source /opt/ros/humble/setup.bash
source /home/ruan-x/robot_ws/install/setup.bash
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
export ROS_DOMAIN_ID=20
```

- OS：Ubuntu 22.04
- ROS：ROS 2 Humble
- Isaac Sim：5.0.0
- NVIDIA Driver：`580.159.03`
- Workspace：`/home/ruan-x/robot_ws`

所有 Isaac、adapter、GLIM、rosbag 和检查终端必须处于相同 ROS domain 和 workspace
环境。

### 场景与机器人

主线场景：

```text
src/robot_sim/usd/scenes/full_warehouse_r.usd
```

机器人唯一允许保留的 active articulation：

```text
/World/robot_sim/base_footprint
```

LiDAR frame：

```text
base_footprint
  base_link
    livox_frame
```

`livox_frame` 相对机器人约有 30 度 pitch，true IMU 重力投影因此约为：

```text
x ~= -4.9 m/s^2
z ~=  8.5 m/s^2
```

### 控制

```text
Joy-Con
  -> /cmd_vel
  -> /sim/target_joint_states
  -> ROS2SubscribeJointState
  -> IsaacArticulationController
  -> /World/robot_sim/base_footprint
```

### LiDAR

当前使用 MID360-like RTX LiDAR，发布链为：

```text
Isaac RTX LiDAR
  -> ROS2RtxLidarHelper(fullScan=True)
  -> /point_cloud
  -> pointcloud2_to_livox_custom_msg_node
  -> /livox/lidar
  -> /livox/lidar_pc2_accumulated
  -> GLIM
```

当前 GLIM 使用 `/livox/lidar_pc2_accumulated`。`_012` bag 平均频率：

- raw `/point_cloud` 约 `6.85 Hz`
- accumulated/Livox 输出约 `5.97 Hz`

频率低于目标 10 Hz，但已经能够支持 GLIM 建图。LiDAR render product 已从
`128x128` 改为 `1x1`，等待下一次 LiDAR-only A/B 测试确认改善幅度。

### Isaac true IMU

当前采用“路径 2：Python `ImuSensorInterface`”：

```text
Isaac physics IMU prim
  -> Python physics-step callback
  -> Isaac C++ ImuSensorInterface
  -> publish-only ROS2PublishImu graph
  -> /livox/imu
```

选择该路径的原因：

- `IsaacReadIMU.inputs:imuPrim` 在 Isaac Sim 5.x 中无法通过普通
  OmniGraph `SET_VALUES` 稳定绑定。
- 旧 `IsaacReadIMU` 路径出现过全零数据、`no valid sensor reading` 和运行时
  tensor 干扰。
- `ImuSensorInterface` 仍然读取 Isaac physics IMU 的真实物理数据，不是 fake IMU。
- 绕过 `IsaacReadIMU` 后，只保留最小 ROS 发布 graph，减少 OmniGraph/PhysX
  tensor 交互。

关键实现：

- IMU prim 和发布 graph 必须在 Play 前创建。
- 不再创建 `IsaacReadIMU` OmniGraph node。
- callback 直接读取加速度、角速度和 orientation。
- orientation 按 ROS 需要的 `x,y,z,w` 顺序发布。
- ROS header timestamp 使用现有 `/clock` simulation time，不使用 IMU sensor local
  time。

这解决了早期 LiDAR 与 IMU 相差数百秒、GLIM 持续等待 IMU 的问题。

### D455

D455 camera-only recording 已进入默认建图流程。D455 不用于 GLIM，只记录：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

`prepare_d455_camera_only.py` 必须在 Play 前运行，并严格按顺序：

1. 停用 D455 内部 `RSD455/Imu_Sensor`；
2. 移除 RSD455 的 RigidBody/Mass API；
3. 禁用碰撞和 joint；
4. 保留 Camera prim。

顺序错误会使内部 IMU 在父 rigid body 被移除后留下错误 tensor reader，触发
`getVelocities (2,6)`。当前记录配置为 `640x360 @ 10 Hz`，实际约 `6.3 Hz`。

## 已完成的主要排查

### 1. ROS 环境与 rosbag

历史问题：

- 不同窗口使用不同 `ROS_DOMAIN_ID`、DDS 或 workspace 环境。
- recorder 在必要 topic 出现前启动，生成空 bag。
- 重复运行 Isaac LiDAR setup 后出现多个 `/point_cloud` publisher。

解决：

- quickstart 与所有手动窗口统一环境。
- 启动前等待必要 topic 和 TF。
- 建图前确认 `/point_cloud` Publisher count 为 1。
- rosbag 结束后使用 `ros2 bag info` 验证。

### 2. Robot USD 与 PhysX `(2,6)`

历史错误：

```text
getVelocities: expected total size 6, received total size 12 with shape (2, 6)
```

做过的检查：

- 扫描 active/inactive articulation。
- 清理 `full_warehouse_r.usd` 中的 forklift、pushcart 等外部 articulation。
- 移除重复 `/World/robot_sim/root_joint` articulation schema。
- 分别禁用 JointState publisher、ArticulationController 和 Isaac IMU graph。
- 建立 LiDAR-only 基线后，将 D455 改为 camera-only：停用内部 IMU，再移除
  RigidBody/Mass API。
- 重启 Isaac 后重新建立干净 Play session。

当前结论：

- 该错误不是单独由某一个 visible graph node 引起。
- 更可能是多个 articulation、nested rigid body、旧 runtime prim 和不干净 Play
  session 共同导致 PhysX tensor view/handle 重复。
- Play 前完成清理并重新启动 Isaac 后，最新成功运行未复现 `(2,6)`。

### 3. LiDAR 选择、TF 与频率

历史问题：

- LiDAR 初始 TF/姿态错误，点云方向异常。
- Isaac 没有官方 MID360 profile。
- `/livox/lidar` 为 Livox `CustomMsg`，不能直接作为当前 GLIM PointCloud2 输入。
- RTX publisher 重复创建或使用不稳定路径。

处理：

- 固定 `base_footprint -> livox_frame` 外参和 30 度安装姿态。
- 测试 OS1、Hesai 和 MID360-like 配置。
- 使用 adapter 生成 `/livox/lidar_pc2_accumulated` 给 GLIM。
- RTX 发布改为 `ROS2RtxLidarHelper + fullScan=True`。
- LiDAR setup 每个 Isaac session 只运行一次。

### 4. IMU 发布源与 `/clock` 对齐

历史问题：

- fake IMU 未投影 30 度安装姿态下的重力。
- IMU sensor local time 与 LiDAR `/clock` 时间域不同。
- GLIM 出现：

```text
large time difference between points and imu
waiting for IMU data
IMU timestamp rewind detected
```

处理：

- corrected fake IMU 用于早期隔离验证。
- 主线改为 true IMU + Python `ImuSensorInterface`。
- IMU timestamp 改为复用 `/clock` simulation time。
- 使用 bag 和 topic header 对比验证 LiDAR、IMU、`/clock` 在同一时间域。

### 5. GLIM 启动、漂移与地图保存

历史问题：

- TF、重力和时间戳错误导致漂移、地图下沉或无地图。
- quickstart 后台进程无法获得与手动窗口相同的 `Ctrl-C` 信号语义。
- GLIM 被自动关闭时，`/tmp/dump` 尚未完成写入。

处理：

- 各进程使用正常前台 Terminator pane。
- 用户在 GLIM pane 中按 `Ctrl-C`。
- 等待 GLIM 打印 `saved` 和脚本验证完成后再关闭窗口。
- 自动检查 dump 内容，再复制到 `maps/`。

### 6. Isaac 卡死与 CUDA error 700

历史错误链：

```text
isaacsim.sensors.rtx.plugin: CUDA error 700
OgnROS2PublishPointCloud.cpp: cudaErrorIllegalAddress
Failed to wait on external semaphore
Rendering failed
```

做过的检查：

- 清理 foreign articulation、D455、旧 RenderProduct 和重复 publisher。
- 在简单场景和 built-in `os1_32` 下测试，仍可复现。
- 干净重装 Isaac Sim 5.0.0，清理旧版本和缓存。
- 核验 NVIDIA Driver `580.159.03`。
- 检查 RTX SDGPipeline、DispatchSync 和 WriterSyncGate 日志。

当前定位：

- CUDA 700 的直接触发点位于 RTX sensor/ROS2 PointCloud 发布链。
- 场景、LiDAR profile 和 driver 可能影响复现概率，但都不能单独解释全部问题。
- 旧 direct Replicator writer/partial-scan 路径风险较高。
- 当前改用 `ROS2RtxLidarHelper + fullScan=True` 后，最新完整建图未出现 CUDA 700。

当前状态是“有效规避，尚未证明 Isaac 底层 RTX bug 已完全解决”。仍可能看到少量
SDGPipeline illegal-cycle 或 `DispatchSync.inputs:enabled` 警告；如果没有伴随
CUDA 700、数据停止或渲染失败，可记录并继续观察。

## 当前固定启动原则

1. 使用统一 ROS 环境启动 Isaac 和所有 ROS 进程。
2. 每次 CUDA 700 后必须完全重启 Isaac，不能继续使用损坏的 CUDA context。
3. Play 前清理 foreign articulation、旧 IMU 和旧 RTX LiDAR runtime。
4. Play 前运行 `prepare_d455_camera_only.py` 并创建 true IMU。
5. Press Play 后只运行一次 LiDAR setup，再运行一次 D455 ROS setup。
6. `/point_cloud` Publisher count 必须为 1。
7. topic 存在不代表数据健康，必须检查频率、header stamp 和 publisher count。
8. GLIM 必须在前台 pane 中运行，并通过 `Ctrl-C` 完成保存。

## 当前残余风险

- RTX pipeline 仍偶尔输出 Isaac 内部 illegal-cycle/DispatchSync 警告。
- CUDA 700 目前只确认在最新完整运行中未复现，需要多次长时间测试。
- raw 和 accumulated LiDAR 输出频率仍低于目标。
- D455 已加入默认主线，但会降低约 `12%–13%` 的整体仿真实时速度。
- D455 CameraInfo distortion 当前使用默认回退值，不等于精确标定参数。
- GLIM 在交互式 TTY 中的详细屏幕 warning 尚未完整保存到日志，调试证据仍可改善。
- 地图需要继续在 offline viewer 中检查闭环、重影、Z 漂移和导航可用性。

## 下一步

1. 使用当前固定流程连续进行多次长时间 mapping，确认 CUDA 700 和 `(2,6)` 不再复现。
2. 检查最新地图的闭环、重影、地面平整度和轨迹漂移。
3. 优化 LiDAR/adapter 输出频率，同时保持 `/clock` 时间戳连续。
4. 优化 D455/LiDAR 并行吞吐，并补充可靠的 D455 相机标定参数。
5. 使用现有实机工具导出 PLY 和 Nav2 2D map。

## 开发反思

- 先建立最小可运行基线：单 articulation、单 LiDAR publisher、关闭非必要 sensor。
- 每次只修改一个变量，并使用日志、bag、频率和时间戳证明结论。
- 不要把连锁报错当作根因；semaphore、RenderGraph 和 asyncio 错误通常是前序
  PhysX/RTX 故障的后果。
- 模拟传感器必须共享同一时间域，不能只比较发布频率。
- 自动化脚本必须保留前台程序的信号语义，尤其是依赖 `Ctrl-C` 保存的程序。
- “成功一次”代表方案可行，不代表底层不稳定因素已经消失。
