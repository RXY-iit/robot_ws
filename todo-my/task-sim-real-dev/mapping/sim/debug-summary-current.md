# Mapping Debug Summary Current

## Original Goal

在 Isaac Sim 中复用真实机器人开发流程，创建可用于导航的 GLIM 3D map。

目标包括：
- 使用真实机器人同名 topic：`/livox/lidar`、`/livox/imu`、TF、`/clock`。
- 使用真实机器人 GLIM config，sim 只在 GPU backend 等必要处做最小差异。
- 记录 rosbag，保存 GLIM dump，再转成后续导航可用地图。
- 最终希望使用 Isaac true IMU，而不是长期依赖 fake IMU。

结论：目标仍然可实现。当前 true IMU + MID360 基础链路已经能启动，
下一步重点是继续验证 GLIM 建图质量和长时间运动稳定性。

## Tests Done So Far

1. GLIM 安装和 GPU 检查
- `glim_ros` 已安装成功。
- GPU GLIM 能启动。
- GLIM 可以读配置并生成部分 dump。

2. ROS 环境统一
- 固定使用：
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ruan-x/robot_ws/install/setup.bash
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  export ROS_DOMAIN_ID=20
  ```
- quickstart 已按这个环境启动。

3. rosbag 记录
- 早期 bag 没保存，原因是话题未出现或 DDS/session 不一致。
- 后来 rosbag 可以保存。
- 当前 quickstart 会等待 `/point_cloud`、`/livox/lidar`、`/livox/lidar_pc2_accumulated`、`/livox/imu`、TF。

4. fake IMU 测试
- 修过 fake IMU 时间戳 rewind。
- 修过 30 deg LiDAR pitch 下的重力投影。
- fake IMU 能让 GLIM 有一定改善，但漂移仍明显。

5. Isaac true IMU 测试
- 成功发布 `/livox/imu`。
- IMU 数据看起来合理：
  - `frame_id: livox_frame`
  - pitch 约 30 deg
  - 重力约 `x=-4.9, z=8.5 m/s^2`
- 最初使用 OmniGraph `IsaacReadIMU` 时，出现 `inputs:imuPrim` 连接失败、
  `/livox/imu` 全零、以及 PhysX tensor `(2,6)` 相关问题。
- 已改为路径 2：Python `ImuSensorInterface`。
  - 不再创建 `IsaacReadIMU` OmniGraph node。
  - Python physics step callback 直接调用 Isaac C++ IMU sensor interface：
    `isaacsim.sensors.physics._sensor.acquire_imu_sensor_interface()`。
  - 每个 physics step 读取 `get_sensor_reading(..., read_gravity=True)`。
  - 读取结果写入最小 ROS2 publish graph：`OnPlaybackTick -> ROS2PublishImu`。
- 修正过一次 orientation 顺序：
  - Isaac reading 内部为 `x,y,z,w`。
  - `ROS2PublishImu` 输入按 `x,y,z,w` 写入。
  - 目标 ROS echo 应为 `orientation.y ~= 0.2588`、`orientation.w ~= 0.9659`。
- 在清理 foreign articulation、D455/RSD455、旧 IMU runtime prim 后，
  true IMU + MID360 测试中 `(2,6)` 暂未复现。

6. 场景 articulation 清理
- 发现 `full_warehouse_r.usd` 内存在机器人以外的 articulation，例如 forklift、pushcart 等。
- 新增脚本清理外部 articulation。
- 清理后确认只剩：
  ```text
  ACTIVE robot /World/robot_sim/base_footprint
  ```
- 同时删除/禁用 `/World/robot_sim/root_joint` 的 articulation schema。

7. OmniGraph 节点隔离
- 禁用 `ROS2PublishJointState` 后，`getVelocities (2,6)` 仍继续。
- 禁用 `IsaacArticulationController` 后，`getVelocities (2,6)` 仍继续。
- 禁用 Isaac IMU graph 后，`getVelocities (2,6)` 仍继续。

当前结论：早期 `(2,6)` 不是单纯由 joint state publisher、articulation
controller、或某一个 visible articulation 单独触发。它更像是多个 runtime
因素叠加：旧 Isaac IMU graph、D455/RSD455 nested rigid body、foreign
articulation、以及不干净的 Play session。当前流程要求 Play 前清理这些对象。

## Current System Design

## Environment

- OS: Ubuntu 22.04 / ROS 2 Humble
- Isaac Sim: PC 本机运行
- ROS domain: `ROS_DOMAIN_ID=20`
- DDS: 默认 RMW，清除 CycloneDDS 特殊环境变量
- Workspace: `/home/ruan-x/robot_ws`

## Scene

当前主线场景：

```text
src/robot_sim/usd/scenes/full_warehouse_r.usd
```

它包含：
- `/World/robot_sim`
- `/World/full_warehouse`
- `/World/ROS2_ActionGraph`
- `/physicsScene`

重要要求：
- Play 前必须清理非机器人 articulation。
- Play 前只允许 `/World/robot_sim/base_footprint` 是 active articulation。

## Robot Structure

机器人主 articulation：

```text
/World/robot_sim/base_footprint
```

传感器 frame：

```text
base_footprint
  base_link
    livox_frame
```

LiDAR 安装姿态：
- `base_link -> livox_frame`
- pitch 约 30 deg
- 高度约 1.49 m from `base_footprint`

已处理的问题：
- `/World/robot_sim/root_joint` 原本也带 articulation schema，已在 overlay 中删除 schema，并在 cleanup 中移除。

## Control

控制链路当前设计：
- Joy-Con 或 direct cmd_vel
- ROS 节点转换为 `/sim/target_joint_states`
- Isaac OmniGraph 中：
  - `ROS2SubscribeJointState`
  - `IsaacArticulationController`
- controller target:
  ```text
  /World/robot_sim/base_footprint
  ```

注意：
- 禁用 `IsaacArticulationController` 后机器人不能动。
- 但即使禁用 controller，当前 `getVelocities (2,6)` 仍继续。

## Sensors

LiDAR：
- Isaac RTX LiDAR runtime prim：
  ```text
  /World/robot_sim/base_footprint/base_link/livox_frame/localization_lidar
  ```
- Raw topic：
  ```text
  /point_cloud
  ```
- Adapter converts:
  ```text
  /point_cloud -> /livox/lidar -> /livox/lidar_pc2_accumulated
  ```

IMU：
- 当前目标是 Isaac true IMU：
  ```text
  /World/robot_sim/base_footprint/isaac_livox_imu
  ```
- ROS topic：
  ```text
  /livox/imu
  ```
- frame:
  ```text
  livox_frame
  ```
- 当前实现使用路径 2：Python `ImuSensorInterface`。
  - IMU prim 仍是 Isaac physics IMU sensor。
  - 读取逻辑不使用 OmniGraph `IsaacReadIMU`。
  - Python physics callback 直接读 Isaac C++ sensor interface。
  - ROS 发布只使用最小 OmniGraph `ROS2PublishImu`。
  - 这个设计保留真实 Isaac IMU 数据，同时绕开 `IsaacReadIMU` 的 prim
    relationship / tensor view 问题。
- 必须在 Press Play 前运行：
  ```python
  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())
  ```
- Press Play 后应看到前三帧 debug：
  ```text
  [IMU step 1] lin_acc=(-4.9, 0, 8.5) ... orient_xyzw=(0, 0.2588, 0, 0.9659)
  ```

D455：
- 暂不作为 mapping 主线。
- 避免 RSD455 rigid body 和额外 render/physics complexity 干扰。
- 当前 LiDAR-only mapping 前会执行 `cleanup_d455_for_mapping.py`，将
  `/d455_sensor` reference deactivate，避免 nested rigid body 进入 PhysX。

## IMU Logic: Why Path 2

选择路径 2 的原因：

1. 路径 1 `readGravity=False` 只是在数据层绕开重力处理
- 不能解决 `IsaacReadIMU` 的 prim relationship 和 OmniGraph 运行时问题。
- 还需要在 GLIM 或 ROS 侧补偿重力，容易引入新的 frame/sign 错误。

2. 路径 2 保留 Isaac true IMU，又绕开 `IsaacReadIMU`
- 目标不是 fake IMU，而是继续使用 Isaac physics IMU sensor 的真实读数。
- 旧方案问题集中在 OmniGraph `IsaacReadIMU`：
  - `inputs:imuPrim` 在 Isaac Sim 5.x 中不能稳定通过 `SET_VALUES` 设置。
  - node 容易出现 `no valid sensor reading`。
  - 还可能触发 PhysX tensor view / articulation velocity 相关问题。
- 新方案只保留 ROS 发布 graph，不再让 OmniGraph node 读取 IMU。

3. 路径 2 的当前结构
- Play 前创建 IMU prim：
  ```text
  /World/robot_sim/base_footprint/isaac_livox_imu
  ```
- Play 前创建 publish-only graph：
  ```text
  OnPlaybackTick -> ROS2PublishImu
  ```
- Play 后 physics step callback：
  ```python
  imu_iface.get_sensor_reading(IMU_PATH, read_gravity=True)
  ```
- callback 将以下数据写入 `ROS2PublishImu` inputs：
  - `linearAcceleration`
  - `angularVelocity`
  - `orientation` (`x,y,z,w`)
  - `timeStamp`

4. 当前验证结果
- ROS topic 存在：
  ```text
  /livox/imu
  ```
- 频率约 `75-80 Hz`。
- linear acceleration 正确投影到 30 deg `livox_frame`：
  ```text
  x ~= -4.9
  z ~= 8.5
  ```
- 已修正 orientation 输出顺序，下一轮应确认：
  ```text
  orientation.y ~= 0.2588
  orientation.w ~= 0.9659
  ```
- true IMU + MID360 当前能同时出现：
  ```text
  /livox/imu
  /point_cloud
  ```

## Current Blocking Error

历史上 Isaac Play 后持续出现：

```text
Incompatible size of velocity tensor in function getVelocities:
expected total size 6, received total size 12 with shape (2, 6)
```

含义：
- 某个 PhysX tensor reader 期望一个 6-DOF articulation。
- 实际返回了两个 6-DOF group，shape 为 `(2, 6)`。

已排除/处理：
- 外部 forklift / pushcart articulation：已清理。
- `/World/robot_sim/root_joint`：已移除/删除 schema。
- `ROS2PublishJointState`：禁用后错误仍继续。
- `IsaacArticulationController`：禁用后错误仍继续。
- D455/RSD455 nested rigid body：已通过 `cleanup_d455_for_mapping.py` deactivate。
- Isaac true IMU graph：旧 `IsaacReadIMU` 路线废弃，改为路径 2。

当前状态：
- 重启 Isaac 后，按清理顺序执行，再使用路径 2 true IMU，`(2,6)` 暂未复现。
- 如果再次复现，优先检查：
  - 是否未重启 Isaac。
  - 是否还有 active D455/RSD455。
  - 是否误运行旧 `IsaacReadIMU` graph。
  - 是否 `/point_cloud` publisher count 大于 1。

## Possible Causes

1. PhysX tensor view 缓存未清理
- 即使 stage 中 articulation 已删除，Play 后 PhysX tensor backend 可能仍保留旧 view。
- 需要重开 Isaac 或在 Play 前完成全部 cleanup。

2. full_warehouse reference 内仍有隐藏 articulation
- 当前 list 脚本未发现，但可能有 instance/prototype/reference 内部对象参与 PhysX。
- 需要更底层地检查 PhysX scene 或禁用整个 warehouse 中的 physics articulation/collision。

3. Isaac ActionGraph 中还有未列出的 PhysX tensor reader
- 当前 list 只覆盖常见 node type。
- 可能有隐藏 node、extension、Replicator、或者 physics monitor 在读 articulation velocity。

4. Robot articulation 本身被重复注册
- `/World/robot_sim/base_footprint` 可能被两个 graph/view 同时 target。
- 虽然只有一个 articulation prim，但 PhysX view 可能创建了两个 articulation handles。

5. USD layer composition 问题
- `robot_sim_flattened.usda` 和 `robot_sim_physics.usd` overlay 中 articulation root 的 schema 删除可能没有完全覆盖运行时 composition。
- 需要在 Isaac 内确认实际 composed prim schema，而不是只看源码文件。

6. Isaac Sim / PhysX bug
- 如果 stage 确认只有一个 active articulation，且所有 obvious tensor reader 都禁用后仍刷错，可能是 Isaac 5.x 对该 imported URDF articulation 的 tensor bug。

## Possible Modification Directions

1. 做更强的 PhysX/OmniGraph 诊断
- 列出所有 OmniGraph node type。
- 查所有含 `targetPrim=/World/robot_sim/base_footprint` 的节点。
- 查所有 active/inactive prim 的 composed schemas。

2. 创建精简控制版本
- 禁用 USD 内置 `ROS2_ActionGraph`。
- Play 后用一个干净 runtime graph 只创建必要 controller。
- 避免旧 graph 中隐藏 tensor reader。

3. 修改机器人 USD
- 从源头移除 `root_joint` articulation schema，而不是 overlay delete。
- 重新导出/flatten 一个只有一个 articulation root 的 robot USD。

4. 改用简化 warehouse
- 继续用 full warehouse 的视觉/碰撞静态部分。
- 删除所有非静态 dynamic/physics/animation/articulation prim。
- 或生成一个 `full_warehouse_static_for_mapping.usd`。

5. IMU fallback
- 如果 PhysX true IMU 路径长期卡住，可临时回到 corrected fake IMU 继续 GLIM map pipeline。
- 但最终定位质量仍建议解决 true IMU 或使用更真实的 kinematic IMU publisher。

6. LiDAR fallback
- MID360-like RTX profile 如果再次触发 CUDA/render 错误，可临时换官方 OS1 profile 验证 GLIM pipeline。
- 但当前主线仍保留 MID360-like。

## Feasibility

目标可实现，但分两层：

1. 建图 pipeline 可实现
- LiDAR topic、adapter、GLIM、bag、dump 保存都已经局部跑通。
- 如果临时使用 fake IMU 或简化场景，可继续推进 map creation。

2. 高质量 full warehouse + true IMU 建图
- 当前被 PhysX tensor `(2,6)` 阻塞。
- 需要先消除 Isaac scene / graph / articulation 结构问题。
- 一旦 PhysX 不再刷错，true IMU + MID360-like + GLIM 方案是合理的。

## Immediate Next Debug Step

在 Isaac 内运行一个更底层的 graph/target 查询：

```python
import omni
stage = omni.usd.get_context().get_stage()
for prim in stage.TraverseAll():
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if "targetPrim" in name or "robotPath" in name:
            print(prim.GetPath(), name, attr.Get())
```

目标：找出除已知 controller/joint publisher/IMU 外，还有谁在 target `/World/robot_sim/base_footprint`。
