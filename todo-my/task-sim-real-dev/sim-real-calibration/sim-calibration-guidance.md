# 仿真运动学校准指南

> 创建: 2026-06-07
> 范围: Isaac Sim Phase 1 — 在进行建图之前，验证 `omni_drive_controller.py` 和 `/wheel_odom` 的准确性。
> 参考: [how-to-gpt.md](how-to-gpt.md)（通用校准思路），[sim-real-build-guideline.md](../sim-real-build-guideline.md)（Phase 1 通过标准）

---

## 现有 bag 的可用性评估（sim_factory_mid360_like_gpu_trueimu_011）

bag 时长 349.8 s，内容为建图阶段的自由遥控录制。

| 测试 | 需要的 topic | bag 中是否存在 | 可评估性 |
|---|---|---|---|
| A — 关节速度比例 | `/cmd_vel` + `/joint_states` + `/sim/target_joint_states` | ✅ 全部存在 | **可做粗检** — 找稳定 vx 段对比 joint 速度，但非受控采集 |
| B — 直行距离 | `/wheel_odom` | ❌ **缺失** | 不可评估 |
| C — 纯侧移 | `/wheel_odom` + vy 专项指令 | ❌ 缺失 + 无 vy 段 | 不可评估 |
| D — 原地旋转 | `/wheel_odom` | ❌ 缺失 | 不可评估 |
| E — 正方形路径 | 结构化指令序列 | ❌ 无结构化指令 | 不可评估 |

**结论：需要重新录制。**

`/wheel_odom` 缺失是核心问题（Isaac Sim Bridge 未配置或未加入录制列表）。
在录新 bag 前，先确认 `/wheel_odom` 正在发布：

```bash
export ROS_DOMAIN_ID=20
ros2 topic hz /wheel_odom   # 必须有输出，否则先修 Bridge 配置
```

**测试 A 粗检方法（利用现有 bag）：**

找 cmd_vel 中 vx 较稳定的一段（≥ 1 s），对比 `/sim/target_joint_states` 中驱动关节速度：

```bash
# 找 vx≠0 的时间段
ros2 bag play sim_bag/sim_factory_mid360_like_gpu_trueimu_011 \
  --topics /cmd_vel /sim/target_joint_states \
  | grep -E "linear|velocity"
```

期望：drive joint velocity ≈ cmd_vel.linear.x / 0.1125

---

## 机器人几何参数（固定值 — 不得单独修改 URDF 或 controller）

| 参数 | 值 | 来源 |
|---|---|---|
| 车轮半径 | **0.1125 m** | `robot_sim.urdf.xacro:8` |
| 车轮宽度 | 0.060 m | `robot_sim.urdf.xacro:9` |
| 右轮距中心 | 0.32330 m | `picking_robot_matrix.hpp:21` |
| 左轮距中心 | 0.32330 m | `picking_robot_matrix.hpp:21` |
| 后轮距中心 | 0.38500 m | `picking_robot_matrix.hpp:21` |
| 右轮位置角 | -0.883978 rad（≈ -50.6°） | `picking_robot_matrix.hpp:22` |
| 左轮位置角 | +0.883978 rad（≈ +50.6°） | `picking_robot_matrix.hpp:22` |
| 后轮位置角 | π rad（180°） | `picking_robot_matrix.hpp:22` |

**驱动关节单位换算**：`omni_drive_controller.py` 输出的驱动速度单位是 **m/s**（与真机 `cmd_vel_to_motor_node` 一致）。Isaac Sim 的 Articulation 需要 **rad/s**。controller 在设置 joint velocity 前必须做如下换算：

```
joint_vel [rad/s] = drive_vel [m/s] / 0.1125
```

如果这个换算缺失或写错，所有运动测试都会出现固定比例的误差。

---

## 前置检查：运动测试前先确认 topic

Isaac Sim 进入 Play 状态、`robot_state_publisher` 和 ROS2 Bridge 启动后执行。

```bash
export ROS_DOMAIN_ID=20

# 以下三个都应该有数据
ros2 topic hz /wheel_odom           # 期望: ~20 Hz（与真机 robot_odom_node 一致）
ros2 topic hz /livox/imu            # 期望: ~200 Hz
ros2 topic hz /joint_states         # 期望: ~60 Hz

# 确认 frame_id 正确
ros2 topic echo /wheel_odom --once  # header.frame_id="odom", child_frame_id="base_footprint"

# TF 链必须完整
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link livox_frame
```

如果 TF lookup 失败或 `/wheel_odom` 的 frame_id 字段不对，**在这里停下来修复**，不要继续。
frame_id 错误会导致后续 GICP 和 Nav2 全部失败。

**当前仿真补充（2026-06-08）**：

Isaac OmniGraph 方式创建 `ROS2PublishOdometry` 后曾打印成功：
`Created /wheel_odom and /tf odom->base_footprint publishers.`
但 DDS graph 中没有实际出现 `/wheel_odom`。本次校准 bag 采用 ROS 侧辅助节点从 Isaac `/joint_states`
计算并发布 `/wheel_odom`：

```bash
export ROS_DOMAIN_ID=20
ros2 run robot_sim sim_omni_drive_command_node
python3 ~/robot_ws/tools/sim_joint_state_wheel_odom.py
```

`sim_joint_state_wheel_odom.py` 使用 `/joint_states` 中的 steer/drive joint 作为仿真编码器反馈：
- drive joint velocity 单位为 rad/s，先乘 `0.1125` 转成轮线速度 m/s
- 按真机 `robot_odom_node` 同一套三轮全向运动学矩阵积分
- 发布 `/wheel_odom` 与 `/tf: odom -> base_footprint`

正式修复目标仍应是 Isaac/ROS Bridge 原生发布 `/wheel_odom`，但在此之前该 ROS 侧 publisher 可用于受控校准录包。

---

## 测试 A — 关节速度比例（纯 vx）

**目的**：验证 `drive_vel [m/s] / wheel_radius` 换算和纯前进指令下的转向角逻辑。

**`linear.x = 0.1 m/s, angular.z = 0` 时的期望行为**：
- 3 个转向关节 → **0 rad**（朝前；后轮可能因 π-flip 变为 ±π，可接受）
- 3 个驱动关节 → **0.1 / 0.1125 ≈ 0.889 rad/s**

```bash
# 终端 A：持续发布前进指令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10

# 终端 B：观察关节状态
ros2 topic echo /joint_states
```

观察 `velocity[]` 中 3 个驱动关节的值。若驱动关节转速约为 0.889 rad/s、转向关节接近 0 rad，则比例正确。

**常见故障：**

| 现象 | 可能原因 |
|---|---|
| 驱动关节约 0.1 rad/s（而非 0.889） | 缺少 m/s → rad/s 换算 |
| 驱动关节速度正确但机器人向侧面运动 | 转向关节符号错误 |
| 驱动关节快但机器人几乎不动 | URDF 与 controller 中的轮半径不一致 |
| 后轮驱动方向相反 | π-flip 逻辑（`0.71π` 阈值）的符号需要修正 |

---

## 测试 B — 直行距离（vx，用 /wheel_odom 测量）

**目的**：确认 `/wheel_odom` 的位移与指令距离一致。

```bash
# 记录 10 秒前进
ros2 bag record /cmd_vel /wheel_odom /tf /joint_states /clock \
  -o /tmp/calib_straight

# 在另一个终端，精确运行 10 秒：
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10 \
  --times 100   # 100 次 × 0.1 s 间隔 = 10 s
```

期望位移：**0.1 m/s × 10 s = 1.0 m**（+X 方向）。

从 bag 读取最终 odom 位姿：
```bash
ros2 bag play /tmp/calib_straight
ros2 topic echo /wheel_odom | grep -A5 "pose:" | head -20
```

同时在 Isaac Sim GUI 中确认：选中机器人 prim → Translate X ≈ 1.0 m。

**测试组：**

| 指令 vx | 时长 | 期望 Δx | 通过标准 |
|---|---|---|---|
| 0.1 m/s | 10 s | 1.0 m | odom Δx 误差 ≤ 5 cm |
| 0.2 m/s | 5 s | 1.0 m | odom Δx 误差 ≤ 5 cm |
| 0.3 m/s | 5 s | 1.5 m | odom Δx 误差 ≤ 8 cm |

误差随速度线性增长 → 动力学问题（摩擦、惯性），不是运动学问题。
误差在所有速度下保持固定比例 → 轮半径写错了。

---

## 测试 C — 纯侧移（vy，全向机器人专项）

**目的**：验证全向移动能力。这是 GPT 差速轮指南里没有的测试，是本机器人特有的校验。

普通差速轮机器人发送 vy 指令时不会移动。如果机器人在纯 vy 指令下旋转或斜向运动，说明 vy 对应的转向角计算有误。

**`linear.y = 0.1 m/s, angular.z = 0` 时的期望行为**：
- 右轮 → 转向 ≈ -90°（朝左），驱动前转
- 左轮 → 转向 ≈ +90°（朝左），驱动前转
- 后轮 → 转向 ≈ 90°（朝左），驱动前转

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

检查 `/wheel_odom`：10 秒后 Δy ≈ 1.0 m，Δx ≈ 0，Δyaw ≈ 0。

**常见故障：**

| 现象 | 可能原因 |
|---|---|
| 机器人原地旋转而非平移 | ROS cmd_vel 坐标系 x/y 搞反 |
| 机器人斜向移动 | 某个车轮的 vy 符号错误 |
| 移动方向正确但速度偏慢 | 与测试 B 相同的轮半径比例问题 |

---

## 测试 D — 原地旋转（纯 angular.z）

**目的**：确认偏航角速度和有效转弯半径。

`angular.z = 0.5 rad/s` 转一圈需要约 **12.57 s**。

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
```

记录 12.57 s 后检查 `/wheel_odom` 的 yaw：应接近 2π（360°）。

**测试组：**

| 指令 ω | 时长 | 期望 Δyaw | 通过标准 |
|---|---|---|---|
| 0.3 rad/s | 10 s | 3.0 rad ≈ 171.9° | 误差 ≤ 10° |
| 0.5 rad/s | 12.57 s | 2π ≈ 360° | 误差 ≤ 15° |
| -0.5 rad/s | 12.57 s | -2π ≈ -360° | 误差 ≤ 15° |

同时确认：机器人中心几乎不平移（< 5 cm）。如果旋转时机器人中心漂移，说明 `omni_drive_controller.py` 中的 `wheel_dist` 与 `picking_robot_matrix.hpp` 不一致。

---

## 测试 E — 组合运动（正方形路径）

**目的**：综合验证直行 + 转向的组合误差。

路径：前进 0.5 m → 左转 90° → 重复 × 4 → 期望回到起点附近。

逐段执行（每段等待到位后再发下一段）：

```
1. vx=0.2，持续 2.5 s  → 前进 0.5 m
2. ωz=0.5，持续 3.14 s → 左转 90°
3. 重复以上 × 3 次
```

录制整个过程：
```bash
ros2 bag record /cmd_vel /wheel_odom /tf /joint_states /clock \
  -o /tmp/calib_square
```

结束时 `/wheel_odom` 应距起点 **≤ 15 cm、yaw 误差 ≤ 15°**。
这直接对应 Phase 1 通过标准 — 此测试通过即可进入建图阶段。

---

## 测试 F — 仿真与真机对比

使用统一脚本在仿真和真机上执行**完全相同的指令序列**，录制 bag 后对比。

### 真机分阶段校准实验（先于完整测试 F）

不要先用完整组合动作拟合参数。按以下顺序分别录制，每个 profile 至少重复 3 次：

```bash
# 每次实验重新启动；机器人从同一标记线、同一负载、同一地面开始
tools/open_test_f_real_windows.sh --cleanup --profile straight-forward --trial 01
tools/open_test_f_real_windows.sh --cleanup --profile straight-reverse --trial 01
tools/open_test_f_real_windows.sh --cleanup --profile spin-ccw --trial 01
tools/open_test_f_real_windows.sh --cleanup --profile spin-cw --trial 01
tools/open_test_f_real_windows.sh --cleanup --profile lateral-left --trial 01
tools/open_test_f_real_windows.sh --cleanup --profile lateral-right --trial 01
```

Profile 设计：

| profile | 指令 | 目的 |
|---|---|---|
| `straight-forward` | +X 3 m | 有效轮径、前进侧偏 |
| `straight-reverse` | -X 3 m | 区分轮径误差与转向零点/机械偏差 |
| `spin-ccw` | +720° | 降低转向对准与启动损失占比，校准旋转 |
| `spin-cw` | -720° | 检查旋转方向不对称与打滑 |
| `lateral-left` | +Y 2 m | 全向侧移与转向零点 |
| `lateral-right` | -Y 2 m | 检查侧移方向不对称 |

每段开始/结束都会发布 `/calibration/event` 并录入 bag。实验后执行：

```bash
source ~/robot_ws/install/setup.bash
tools/analyze_calibration_bag.py ~/robot_ws/calib_bags/calib_<profile>_real_<trial>_<timestamp>
```

现场记录表：

| profile/trial | 实测主方向距离或角度 | 实测横向偏移 | odom 主方向 | odom 横向/yaw | 备注 |
|---|---:|---:|---:|---:|---|
| straight-forward/01 |281mm | | | | |
| straight-forward/02 | 282mm| | | | |
| straight-forward/03 | 281mm| | | | 左偏15cm|
| straight-reverse/01 | 280mm| | | | |
| straight-reverse/02 | 227mm| | | | 有明显卡顿，结束后手动操作移动到了初始位置|
| straight-reverse/03 |265mm | | | | |
| spin-ccw/01 | | | | | 没有旋转，前进速度变快。手动操作也只能前进后退，速度较快|
| spin-ccw/02 | | | | | 逆时针旋转，停止位置相比于初始位置大概偏移20cm,20度夹角|
| spin-ccw/03 | | | | | 同上|
| spin-cw/01 | | | | |顺时针旋转，偏移较小大概5cm,角度基本正确|
| spin-cw/02 | | | | | 测量两次第二次同上，第一次启动时没反映，猜测是电池问题|
| spin-cw/03 | | | | | 同上|
| lateral-left/01 |2m | | | |上偏移32cm |
| lateral-left/02 |2m | | | | 几乎无偏移|
| lateral-left/03 | |1.98m | | | 几乎无偏移|
| lateral-right/01 |202mm | | | | |
| lateral-right/02 | 153mm| | | | 上偏移30mm，运动有明显卡顿|
| lateral-right/03 | 2m| | | | 上偏35mm|

判定原则：

- 先确认旋转时 wheel odom 平移接近零，再拟合任何尺度参数。
- `/steer_odom` 必须与 `/steer_angle` 使用相同符号约定；旋转时符号相反会把纯旋转错误解释为侧移。
- 正反直行比例一致且重复性好，才调整有效轮径。
- 正反方向侧偏符号相反，优先校准转向零点 `home_pos`。
- 正反方向均向同一物理方向偏，检查轮胎、载荷、地面摩擦和机械安装。
- 旋转正反不对称，不能只用一个 `wheel_dist` 比例掩盖，应先检查单轮反馈和打滑。

**动作序列**（`todo-my/task-sim-real-dev/sim-real-calibration/test_f_calib_motion.py`）：
```
前进 1 m → 原地旋转 360° → 前进 2 m → 原地旋转 180° → 前进 3 m
总时长 ≈ 68 s
```

### 录制

```bash
source ~/robot_ws/install/setup.bash

# 仿真（Isaac Sim 处于 Play 状态，omni_drive_controller.py 已连接）
export ROS_DOMAIN_ID=20
python3 ~/robot_ws/todo-my/task-sim-real-dev/sim-real-calibration/test_f_calib_motion.py \
  --label sim --direct-control
# Bag 保存至 ~/robot_ws/calib_bags/test_f_sim_<时间戳>/

# 真机快捷启动（机器人已上电，JoyCon 已连接）
export ROS_DOMAIN_ID=13
~/robot_ws/tools/open_test_f_real_windows.sh
# Bag 保存至 ~/robot_ws/calib_bags/test_f_real_<时间戳>/
```

真机快捷启动只启用底盘电机、底盘里程计、JoyCon、AUTO/MANUAL 仲裁、安全层、
测试 F 脚本和 RViz；不启动 LiDAR、相机、升降机或其他串口电机。

真机控制链：

```text
AUTO:   test_f_calib_motion.py -> /nav2/cmd_vel -> nav_mode_switch -> /cmd_vel_raw
MANUAL: JoyCon/L1             -> /teleop/cmd_vel -> nav_mode_switch -> /cmd_vel_raw
both:   /cmd_vel_raw -> cmd_vel_safety_node -> /cmd_vel -> motors
```

脚本在 MANUAL 下暂停动作计时且不发布自动命令；按 A 切回 AUTO 后从当前动作的剩余时间继续。
按 B 时 `nav_mode_switch_node` 会立即发布零速，因此不需要等待测试脚本下一次循环。
真机启动和运行期间还要求 `/drive_odom` 在 0.5 秒内持续更新；反馈中断时测试 F 自动暂停，
避免 `/wheel_odom` 使用陈旧速度继续积分。

脚本自动录制以下 topic（不存在的 topic 会被 ros2 bag record 等待/忽略）：

| topic | 仿真 | 真机 | 用途 |
|---|---|---|---|
| `/cmd_vel` | ✅ | ✅ | 指令参考 |
| `/nav2/cmd_vel` | — | ✅ | 测试 F 自动命令源 |
| `/cmd_vel_raw` | — | ✅ | AUTO/MANUAL 仲裁后的指令 |
| `/teleop/cmd_vel` | — | ✅ | MANUAL JoyCon 指令 |
| `/robot_mode`, `/safety_status`, `/joy` | — | ✅ | 模式切换与安全事件 |
| `/wheel_odom` | ✅ | ✅ | 位置/yaw 比对主数据 |
| `/tf` | ✅ | ✅ | odom→base_footprint 备用 |
| `/joint_states` | ✅ | — | 仿真轮速反馈 |
| `/sim/target_joint_states` | ✅ | — | 仿真 controller 指令 |
| `/drive_vel`, `/steer_angle` | — | ✅ | 真机驱动/转向命令 |
| `/drive_odom`, `/steer_odom` | — | ✅ | 真机驱动/转向反馈 |
| `/clock` | ✅ | — | 仿真时间轴 |

### 真机运行前检查

```bash
# 1. 启动后必须处于 MANUAL，且安全层与底盘里程计存在
ros2 topic echo /robot_mode --once
ros2 topic info /cmd_vel
ros2 topic hz /wheel_odom
ros2 topic hz /drive_odom  # 必须持续稳定，目标约 10 Hz 以上；0.1 Hz 不可运行测试 F

# 纯前进时，逻辑轮速反馈必须同号且接近命令值
ros2 topic echo /drive_odom --once
# vx=+0.15、转向角约 0 时，vel1/vel2/vel3 应全部约 +0.15 m/s

# 2. 按住 L1 小幅手动移动，确认方向与停车正常；再按 A 开始测试 F

# 3. 确认周围有足够空间（前进 3 m + 转弯余量，约 4 m × 4 m）
```

不要在真机用 `--direct-control`，也不要直接向 `/cmd_vel` 发布测试动作；这会绕过
JoyCon AUTO/MANUAL 仲裁与 safety layer。

### 比对方法

```bash
# 从 wheel_odom 提取 (x, y, yaw) 轨迹
ros2 bag play ~/robot_ws/calib_bags/test_f_sim_<时间戳> \
  --topics /wheel_odom
```

对比指标：

| 指标 | 测量方式 | Phase 1 通过标准 |
|---|---|---|
| 前进 1 m 段位移 | 该段起终点的平面距离与局部前向投影 | 仿真 vs 真机差异 < 15 cm |
| 前进 2 m 段位移 | 同上，必须按该段起点做相对位移 | 差异 < 20 cm |
| 前进 3 m 段位移 | 同上，180° 后不能直接读全局 `pose.position.x` | 差异 < 30 cm |
| 旋转 360° 段 yaw | 对 quaternion yaw 解包（unwrap）后取该段相对变化 | 差异 < 0.3 rad |
| 旋转 180° 段 yaw | 同上 | 差异 < 0.2 rad |

真机与仿真的以下差异属于正常，不需要修正：
- 真机 BLV-R 电机有启动惯性，仿真 PhysX joint 响应接近瞬时
- 真机 motor timeout watchdog 0.3 s 在停止段有轻微制动效果
- 真机地面摩擦系数与 PhysX 默认值不同，旋转时中心漂移可能略大

### 2026-06-08 仿真录制结果

有效 bag：

```text
/home/ruan-x/robot_ws/calib_bags/test_f_sim_20260608_000614
```

录制前置状态：
- Isaac Sim 已进入 Play
- `/sim/target_joint_states` 有 1 个 publisher：`sim_omni_drive_command_node`
- `/sim/target_joint_states` 有 1 个 subscriber：Isaac `ROS2_ActionGraph_subscribe_joint_state`
- `/wheel_odom` 由 `tools/sim_joint_state_wheel_odom.py` 从 `/joint_states` 计算发布
- `/wheel_odom.header.frame_id = "odom"`
- `/wheel_odom.child_frame_id = "base_footprint"`
- TF `odom -> base_footprint` 可 lookup

bag metadata 已通过 `ros2 bag reindex` 重建。`ros2 bag info` 摘要：

| topic | count |
|---|---:|
| `/cmd_vel` | 689 |
| `/wheel_odom` | 7656 |
| `/tf` | 7656 |
| `/joint_states` | 4375 |
| `/sim/target_joint_states` | 4845 |
| `/clock` | 4375 |

`/wheel_odom` 轨迹快速检查：

| 指标 | 值 |
|---|---:|
| `x_range` | -0.093 .. 2.000 m |
| `y_range` | -1.289 .. 0.004 m |
| `yaw_range` | -3.141 .. 3.141 rad |
| `max_abs_twist.vx` | 0.158 m/s |
| `max_abs_twist.vy` | 0.046 m/s |
| `max_abs_twist.wz` | 0.501 rad/s |
| final pose | x=2.000 m, y=-1.172 m, yaw=-0.003 rad |

解释：
- bag 内 topic 完整，且 `/wheel_odom` 有明显运动，可作为本轮仿真 Test F 结果。
- 终点不是理想回到原点，说明仿真运动/odom 仍有待校准；这正是后续与真机 bag 对比要定位的问题。
- 该结果先作为 **sim baseline** 保存，后续补充真机 `test_f_real_<timestamp>` 后再做定量对比。

无效 bag（不要用于校准）：

```text
/home/ruan-x/robot_ws/calib_bags/test_f_sim_20260608_000213
```

原因：第一次录制时 `sim_omni_drive_command_node` 未运行，`/cmd_vel` 没有转换成 `/sim/target_joint_states`，
机器人几乎没动。虽然 topic 完整，但 `/wheel_odom` 轨迹基本静止：

```text
x_range=0.000..0.041
y_range=-0.000..0.000
yaw_range=-0.003..0.000
```

---

## 参数修改位置

| 参数 | 文件 | 修改内容 |
|---|---|---|
| 车轮半径 | `src/robot_sim/urdf/robot_sim.urdf.xacro:8` | `wheel_radius` 属性 |
| 车轮几何（真值） | `src/omni_base_driver/include/omni_base_driver/picking_robot_matrix.hpp:21-22` | **不得修改** — 真机源代码 |
| Controller 车轮几何 | `src/robot_sim/scripts/omni_drive_controller.py`（WHEEL_DIST, WHEEL_PHI） | 必须与 picking_robot_matrix.hpp 完全一致 |
| 驱动关节速度换算 | `omni_drive_controller.py` | `cmd_vel_to_wheels()` 内的 `speed / wheel_radius` |
| π-flip 阈值 | `omni_drive_controller.py` | `0.71 * π` — 必须与 picking_robot_matrix.hpp:89,94 一致 |

**原则**：如果发现 `omni_drive_controller.py` 与 `picking_robot_matrix.hpp` 有差异，修 Python 文件。
不允许为了让仿真通过而修改 C++ 头文件 — 那是真机的源代码。

---

## 快速校准检查清单

```
[ ] 前置检查：所有 topic 以预期频率发布，TF 链完整
[ ] 测试 A：vx=0.1 时驱动关节转速 ≈ 0.889 rad/s，转向关节 ≈ 0 rad
[ ] 测试 B：vx=0.1 运行 10 s，/wheel_odom Δx = 1.0 m（误差 ≤ 5 cm）
[ ] 测试 C：vy=0.1 运行 10 s，/wheel_odom Δy = 1.0 m，Δx ≈ 0，Δyaw ≈ 0
[ ] 测试 D：ω=0.5 运行 12.57 s，yaw ≈ 360°，机器人中心平移 < 5 cm
[ ] 测试 E：正方形路径终点距起点 ≤ 15 cm / 15°
[x] 测试 F：仿真 bag 已录制（test_f_sim_20260608_000614）
[ ] 测试 F：真机 bag 待补充
[ ] 测试 F：仿真与真机轨迹形状定性相似
```

以上全部通过后，进入 Phase 2 建图。
