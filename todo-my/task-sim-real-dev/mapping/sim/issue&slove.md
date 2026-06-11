# Isaac Sim + GLIM 建图问题与解决记录

## 目标

在 Isaac Sim 中复用实机 GLIM 配置和开发流程，使用模拟 LiDAR、真实物理 IMU、
机器人 TF 与 Joy-Con 控制生成可保存、可回放的 GLIM 地图和 rosbag。

## 1. ROS 环境、话题与 rosbag

### 问题

- 不同终端使用了不同的 `ROS_DOMAIN_ID`、DDS 或 workspace 环境，导致话题不可见。
- recorder 在必要话题出现前启动，生成空 bag。
- Isaac 脚本重复运行后，`/point_cloud` 出现多个 publisher。

### 排查与解决

- 所有 ROS 进程统一 source Humble 和 workspace，并使用 `ROS_DOMAIN_ID=20`。
- quickstart 启动前等待 `/point_cloud`、Livox 输出、IMU 和 TF。
- 建图前强制检查 `/point_cloud` Publisher count 必须为 1。
- rosbag 保存后自动执行 `ros2 bag info` 验证。

## 2. 机器人 USD、Articulation 与 `(2,6)` 错误

### 问题

Isaac Play 后持续出现：

```text
getVelocities: expected total size 6, received total size 12 with shape (2, 6)
```

场景中存在叉车、推车等外部 articulation；机器人本身还有重复 root joint。
D455/RSD455 又在机器人刚体层级中引入了嵌套 RigidBody；其内部
`RSD455/Imu_Sensor` 在父 rigid body 被移除后仍可能注册 tensor reader，直接触发
`(2,6)`。

### 做过的测试

- 扫描 active/inactive articulation，确认场景曾包含多个非机器人 articulation。
- 删除叉车和推车，并移除重复的 `/World/robot_sim/root_joint`。
- 分别禁用 JointState publisher、ArticulationController、Isaac IMU graph，观察错误是否消失。
- 禁用 D455/RSD455，先建立 LiDAR-only mapping 基线。
- 对 D455 camera-only 流程分别测试：仅禁用 rigid body、移除 RigidBody/Mass API、
  停用内部 `Imu_Sensor` 后再移除 API。

### 当前解决

- Play 前执行 `cleanup_foreign_articulations.py`，只保留：

```text
/World/robot_sim/base_footprint
```

- 默认流程使用 `prepare_d455_camera_only.py`：
  1. 首先停用 D455 内部 `Imu_Sensor`；
  2. 然后移除 RSD455 的 RigidBody/Mass API；
  3. 保留 Camera prim，仅用于 ROS 图像记录。
- 最新默认 D455 运行中没有 `(2,6)`、RSD455 missing rigid body 或 nested
  rigid-body 错误。

## 3. LiDAR 选择、频率与 TF

### 问题

- 初始 LiDAR TF/安装姿态错误，点云方向和可视范围异常。
- Isaac 没有官方 Livox MID360 模型。
- 原始 `/point_cloud` 发布频率、积累方式和 GLIM 输入格式不匹配。
- `/livox/lidar` 是 Livox `CustomMsg`，不能直接作为当前 GLIM PointCloud2 输入。

### 排查与解决

- 固定 `base_footprint -> livox_frame` 外参，当前安装 pitch 为 30 度。
- 测试 built-in OS1 32/128、Hesai 与 MID360-like 配置。
- 使用 `/point_cloud -> Livox adapter`，输出：
  - `/livox/lidar`
  - `/livox/lidar_pc2_accumulated`
- GLIM 使用 `/livox/lidar_pc2_accumulated`。
- RTX publisher 改为 `ROS2RtxLidarHelper`，并启用 `fullScan=True`。

默认 D455 成功运行中：

- raw `/point_cloud` 约 `6.4 Hz`
- `/livox/lidar` 和 GLIM 输出约 `6.3–6.4 Hz`
- D455 color/depth 约 `6.3 Hz`
- LiDAR-only `1x1` 对比约 `7.3 Hz`

频率低于目标 10 Hz，但已能稳定支持 GLIM。后续仍需优化。

## 4. IMU 发布源、重力与时间戳

### 问题

- 早期 fake IMU 把重力全部放在 Z 轴，未考虑 LiDAR 30 度倾角。
- LiDAR 与 IMU 使用了不同时间源，曾出现数百秒时间差：

```text
large time difference between points and imu
waiting for IMU data
IMU timestamp rewind detected
```

- `IsaacReadIMU.inputs:imuPrim` 无法通过普通 `SET_VALUES` 稳定绑定。
- OmniGraph `IsaacReadIMU` 曾伴随 PhysX tensor 错误。

### 做过的测试

- fake IMU 修正为 `x ~= -4.9 m/s²`、`z ~= 8.5 m/s²`，验证 GLIM 可运行。
- 比较 `/clock`、`/point_cloud.header.stamp` 与 `/livox/imu.header.stamp`。
- 测试 `IsaacReadIMU`、关闭重力读取、直接 `ImuSensorInterface` 等路径。

### 当前解决

- 主线保留 Isaac true IMU。
- 使用 Python `ImuSensorInterface` 直接读取物理 IMU，绕过 `IsaacReadIMU`。
- ROS IMU 时间戳复用现有 Isaac `/clock` simulation time。
- IMU 创建在 Play 前完成，避免 Stop/Play 重建造成时间 epoch 分裂。
- corrected fake IMU 仅作为故障隔离备用。

## 5. GLIM 漂移、启动与地图保存

### 问题

- 初期 GLIM 无地图、严重漂移、地图异常下沉。
- IMU 数量不足、LiDAR/IMU 时间不一致、错误 TF 都会放大漂移。
- quickstart 在后台或自动关闭终端时，`Ctrl-C` 无法直接送到 GLIM，导致
  `/tmp/dump` 未完成保存。

### 排查与解决

- 使用真实前台 Terminator pane 启动各进程。
- GLIM 直接占用 pane TTY，用户在 GLIM pane 中按 `Ctrl-C`。
- 等待 GLIM 打印 `saved` 后验证 `graph.txt`、轨迹和 submap，再复制到 `maps/`。
- 每次运行记录 ROS native logs、参数和 pre-run dump。

## 6. Isaac Sim 卡死与 CUDA error 700

### 问题

RTX LiDAR 运行一段时间后可能出现：

```text
isaacsim.sensors.rtx.plugin: CUDA error 700
OgnROS2PublishPointCloud.cpp: cudaErrorIllegalAddress
Failed to wait on external semaphore
Rendering failed
```

### 排查过程

- 清理多个 articulation、D455、重复 publisher 与旧 RenderProduct。
- 使用简单场景和 built-in `os1_32`，仍能复现 CUDA 700，排除了仅由仓库场景或
  MID360-like profile 引起的可能。
- 核验并干净重装 NVIDIA 官方 Isaac Sim 5.0.0 包。
- 删除旧 Isaac Sim 4.5、旧安装和 Kit/RTX 缓存。
- 保留并验证 NVIDIA Driver `580.159.03`。
- 重装后旧发布方式仍出现 CUDA 700，说明重装本身不是完整解决方案。
- 日志显示崩溃前存在 `DispatchSync` / `WriterSyncGate` 异常和 illegal-cycle。

### 当前定位与解决

直接 Replicator writer/partial-scan 发布路径是当前最明确的触发风险。已改为：

- `ROS2RtxLidarHelper` ActionGraph
- `fullScan=True`
- setup 只允许运行一次
- cleanup 必须在 Play 前执行
- CUDA 700 后必须重启 Isaac，不在损坏的 CUDA context 中继续测试

最新成功运行持续完成建图，没有 CUDA 700。仍有少量 Isaac 内部 SDGPipeline
illegal-cycle 警告，因此当前状态是“有效规避、尚未证明底层 bug 完全消失”。

## 7. 最新默认流程验证：trueimu_d455_002

`sim_factory_mid360_trueimu_d455_002`：

- Isaac：无 CUDA 700、无 `(2,6)` tensor 错误
- D455：无 nested rigid-body/RSD455 missing rigid body，color/depth 均约 `6.3 Hz`
- rosbag：`1.1 GiB`，约 `89.35 s`，`23,841` 条消息
- GLIM：`561` 帧轨迹，`38` 个有效 submap
- 自动保存：成功
- 地图：`graph.txt`、`graph.bin`、`values.bin`、轨迹文件均非空
- `/point_cloud`、Livox 和 GLIM 输出约 `6.3–6.4 Hz`

本次启动关闭时部分 ROS 节点记录 exit code 1，是统一 `Ctrl-C` 停止后的退出记录，
不是运行期间故障。

## 反思与后续开发原则

1. 先建立最小 LiDAR-only 基线，再逐个加入 D455 等旁路传感器；验证完成后及时明确
   默认主线，避免文档长期保留多个互相冲突的默认流程。
2. 每次只修改一个变量，并用 bag、时间戳、频率和日志证明结论。
3. 话题“存在”不等于数据健康；必须检查 publisher count、频率和 header stamp。
4. 所有模拟传感器必须共享同一 `/clock` 时间域。
5. RTX/Replicator setup 只能运行一次；CUDA 700 后必须重启。
6. 不要把连锁错误误判为根因。CUDA semaphore、RenderGraph、async 错误通常是
   前面 RTX/PhysX 故障的后果。
7. 自动化脚本必须保留前台进程的信号语义，尤其是依赖 `Ctrl-C` 保存的程序。
8. D455 camera-only 已进入默认流程，但“成功一次”仍不够；需要继续长时间和多次
   重启后的回归测试。
