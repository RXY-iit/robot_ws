# Motion Control Discussion Notes

目的：记录 real lab 场景里底层运动控制/规划层的问题和候选方案。这里先不直接改 controller。

## 当前观察

- Nav2 已经能收到 agent 发出的目标，但多个目标在 lab 狭窄空间里 timeout。
- 有些目标其实已经很接近，例如 `tool_box` 最后 xy error 约 0.12m、yaw error 约 0.22rad，但仍被 Nav2 strict completion 判失败。
  - 注意：这不等于 Nav2 goal tolerance 已经动态放宽。
  - 之前 warehouse 做过的 tolerance 调整主要在 agent / skill 层，用来判断“任务是否可以接受为成功”。
  - 它不是 Nav2 controller_server 内部 goal checker 的动态参数调整。
  - 当前主要任务先不依赖 agent / skill 兜底，而是优化基础运动能力。
- real lab 场景比 warehouse 更密集，桌椅、推车、书架、沙发之间有很多窄通道。
- 机器人是全向移动底盘，但当前控制链没有完全发挥全向能力。
- 最新 `debug-output/sim_nav2_20260706_191801` 显示：
  - Nav2 收到 goal 并持续发布 `/nav2/cmd_vel`。
  - `/nav2/cmd_vel` 有非零 linear command。
  - 进入 `/cmd_vel` 后，motion policy 早期把输出变成近似 `linear_x=0, linear_y=0, angular_z>0`。
  - controller log 没有明确 `no valid trajectory` / collision failure，最后是 goal 被 cancel。
  - 这说明当前主要嫌疑不是 planner 完全失败，而是 command relay / motion policy 与 omni controller 不一致。

## 代码现状

- `nav2_params.yaml` 里已经配置了 polygon footprint。
- MPPI 使用 `motion_model: Omni`，并且 `CostCritic.consider_footprint: true`。
- local/global costmap 都有 footprint 和 inflation。
- Nav2 `SimpleGoalChecker` 仍是固定值：
  - `xy_goal_tolerance: 0.10`
  - `yaw_goal_tolerance: 0.20`
- 之前的 dynamic timeout / task tolerance 在 `NavigationSkills.goto_pose()` 和 `MissionAgent` 里实现：
  - 通过 TF 判断当前 map pose 是否进入 task tolerance。
  - 如果进入，就 cancel Nav2 action 并把 skill 标记为成功。
  - 当前 tolerance policy 的显式规则主要匹配 warehouse shelf / 棚 / 数字棚号，`tool_box`、桌子、沙发、门附近目标不一定触发。
- 但是 `nav_mode_switch_node.py` 会对 Nav2 输出做后处理：
  - `lateral_scale` 默认较低。
  - lateral motion 会被削弱，并部分转成 yaw。
  - 这会让机器人更像“前进优先”，不完全像真正 omni。
- 即使关闭 motion policy，MPPI 参数本身也偏保守：
  - `vy_std: 0.02`
  - `vy_max: 0.05`
  - `vy_min: -0.05`
  - `PathAngleCritic.forward_preference: true`
  - 这会让局部控制偏向前进/转向，而不是像人工 Joy-Con 那样充分使用左上、右上、侧移微调。
- `tools/sim/open_sim_nav2_windows.sh` 已加入：
  - `--no-motion-policy`
  - `--motion-policy`
- sim 默认先用 `--no-motion-policy`，用于 A/B 验证 MPPI 原始 omni command。

## 主要问题

1. 狭窄通道可通过性不明确  
   现在缺少明确的 “robot footprint + corridor clearance” 判断。

2. 目标点可能不可达或不适合进入  
   semantic target 生成的 approach pose 不一定在最安全/最可达的一侧。

3. Final approach 策略太依赖 Nav2 strict goal  
   对 agent task 来说，接近目标并能观察，可能已经足够；不一定需要严格完成 Nav2 yaw。

4. 全向能力没有充分利用  
   MPPI 是 Omni，但后处理策略会压低横向运动。

5. 失败来源需要分层记录  
   现在 “agent timeout / skill cancel / Nav2 cancel / controller failure” 容易混在一起，需要日志里明确来源。

6. 目标可达性和目标完成条件需要分开  
   对 agent 来说，靠近目标并能观察可能已经成功；对 Nav2 strict goal 来说，yaw 或最后几厘米不满足仍可能失败。

7. 底盘运动没有充分利用全向轮  
   人工 Joy-Con 操作时经常通过左上 / 右上 / 后退小幅调整穿过窄空间，但自动导航更像 “转向 + 前进”。

8. 路径贴障碍物  
   测试中经常贴着桌脚走，说明 global path 或 MPPI cost 对 “走空白区域中心” 的偏好不够强。

9. 轮子和车身不应频繁大幅旋转  
   除了需要改变朝向的场景，底盘应尽量保持平滑姿态，用小幅侧移 / 斜移修正路径，而不是不断大角度转向。

## 候选方案

### A. 保守方案

- 保留当前 controller。
- 调整 agent/skill 层：
  - dynamic timeout。
  - task tolerance success。
  - multiple approach candidates。
- 优点：风险小，容易验证。
- 缺点：底层窄通道能力不会根本改善。

### B. Controller profile 方案（早期备选，当前不优先）

增加可切换运动 profile：

- `default`: 当前策略。
- `omni_full`: 直接保留 MPPI `linear_x/linear_y/angular_z`，只经过 safety clamp。
- `narrow_passage`: 低速、高 clearance、允许横移微调。
- `final_approach`: 小范围精调，不强求大旋转。

优点：可以 A/B 测试，不会一次性推翻现有策略。  
缺点：需要更多参数实验，也会增加运行和维护复杂度。

当前讨论倾向：

- 不做需要人工选择的多套 profile。
- 先收敛一套统一默认策略。
- 如果必须根据环境变化调整，也应做自动 context-aware scaling，而不是让用户或 launch 手动选择。

### C. Planner feasibility 方案

在发 goal 前，对多个 approach candidates 做可达性检查：

- 是否能生成 global plan。
- path 是否穿过高 cost 区。
- candidate 附近是否有足够 footprint clearance。

优点：能减少“目标点理论上有，但实际进不去”的失败。  
缺点：需要接入 planner / costmap 信息，工程量更大。

### D. 运动能力分层优化方案

把问题拆成四层，避免把所有现象都归因于 “Nav2 不好”。

#### Planner / global path 的职责

- 尽量走空白区域中心，而不是贴桌脚。
- 避开明显窄通道，除非目标必须经过。
- 对同一个 semantic target 生成多个 approach pose，并选择更开阔的一侧。
- 如果目标附近空间很窄，global path 可以先到开阔 staging pose，再进入 final approach。

可调方向：

- global costmap inflation radius / cost scaling。
- NavFn / A* / Smac planner 对 cost 的使用。
- approach candidate 的 cost-aware selection。
- segment map 中给桌子、柜子、通道增加 preferred approach side。

#### MPPI / local controller 的职责

- 在局部障碍和 global path 之间平衡，不贴障碍物。
- 在狭窄空间里使用 `linear_y` 做横移微调。
- 允许微小后退，不要一味转大角度。
- 保持车身姿态平滑，减少不必要 yaw oscillation。

可调方向：

- 增大 `vy_max` / `vy_std`，让 MPPI 真的采样 lateral motion。
- 降低过强的 forward preference。
- 增强 obstacle / inflation / footprint cost 权重，让轨迹离桌脚更远。
- 调整 PathAlign / PathFollow / PathAngleCritic 权重，避免“贴路径但贴障碍”。
- 在窄通道条件下自动降低速度、提高 obstacle cost、允许更高 lateral ratio。

#### Motion relay / safety layer 的职责

- 不应该把 MPPI 的 holonomic command 改成非 holonomic。
- safety layer 只做速度、加速度、急停、watchdog clamp。
- 如果要加 motion policy，应作为可选调试机制，而不是默认覆盖 MPPI。

可调方向：

- sim lab 默认 `--no-motion-policy`。
- 如果保留调试入口，可以保留：
  - `disabled`
  - `omni_full`
  - `narrow_passage`
  - `final_approach`
- 但正式策略不应依赖用户手动选择这些 profile。
- 对每个调试入口记录 `/nav2/cmd_vel -> /cmd_vel` 的差异。

#### 低层底盘 / 轮子控制的职责

- 把 `/cmd_vel` 转成稳定的全向轮目标。
- 避免因为速度分解、加速度限制或关节状态反馈导致抖动。
- 轮子方向/速度变化应平滑，但不能牺牲全向移动能力。

可调方向：

- 检查 `cmd_vel` 到 wheel command 的 Jacobian / sign / saturation。
- 检查 sim 和真机 wheel odom 是否与 commanded velocity 一致。
- 检查 `linear_y` 命令是否真的产生侧移，而不是被驱动层削弱。

### E. 统一自适应策略，而不是多套人工 profile

用户倾向：不要维护三套需要人工选择的 profile。理想状态是同一套默认导航参数在不同场景下都能自然应对。

关键约束：

- local costmap 主要依赖前视 depth camera。
- camera 只能稳定看到前方地面和前方障碍。
- 因此机器人应尽量保持 “朝前移动 / 前方可见”。
- 但三轮全向底盘仍应允许左上 / 右上斜移，以及小幅侧移 / 后退修正。

这意味着目标不是 “全方向随便横移”，而是：

```text
forward dominant omni motion
  = 主要向前
  + 必要时叠加 lateral 微调
  + 近距离允许小后退
  + 避免大角度原地旋转
```

更具体：

- 宽阔区域：尽量直线、平滑、少转向。
- 桌椅附近：仍以前进为主，但允许斜向绕开桌脚。
- 狭窄通道：低速、提高避障成本、允许更明显的 lateral 修正。
- 近目标：不强求完美 yaw，允许小范围后退 / 侧移完成观察位。

这个可以做成一套连续调节逻辑，而不是多套离散 profile：

- MPPI 采样空间统一放宽 `vy`，但通过 critic 权重让它只在有收益时使用。
- costmap inflation 建成平滑 potential field，让 planner 自然远离桌脚。
- relay 不再把 lateral command 压掉。
- 如果要限制 “纯侧移”，应在 MPPI / critic 层用代价约束，而不是在 relay 层硬改命令。

## 外部资料和调查结论

### Nav2 MPPI: holonomic 需要足够 `vy` 采样

Nav2 MPPI 官方文档里，`vy_std` 是 `Vy` 的采样标准差，默认值是 `0.2`；`vy_max` 是 Omni motion model 下的横向最大速度，默认值是 `0.5`。官方 example 里也给了 `vy_std: 0.2`、`vy_max: 0.5`。  
参考：

- https://docs.nav2.org/configuration/packages/configuring-mppic.html

对当前系统的含义：

- 当前 `vy_std: 0.02`、`vy_max: 0.05` 非常保守。
- 即使 `motion_model: Omni`，MPPI 也很难采样出有意义的左上 / 右上微调。
- 如果我们希望 “尽量向前，但必要时斜移”，应该提高 `vy_std / vy_max`，再用 critic 把无意义 lateral 压回去。

### Nav2 Tuning Guide: inflation 应形成平滑 potential field

Nav2 tuning guide 说明，inflation layer 的价值不只是障碍物外圈防撞，而是在整张地图上形成平滑 potential field，让 planner 倾向走空间中间、尽早远离障碍。文档明确建议在走廊、货架通道、类似 aisle 的空间里创建平滑 potential，以获得更好的路径。  
参考：

- https://docs.nav2.org/tuning/index.html

对当前系统的含义：

- “贴着桌脚走” 不一定是 MPPI 单独的问题，也可能是 global/local cost field 对桌脚周边惩罚太弱。
- 需要优先检查 global/local inflation radius、cost scaling，以及 CostCritic 权重。
- 目标不是简单把 inflation 变大到堵死通道，而是让路径从 0-cost 空白区变成有梯度的 “远离障碍更舒服”。

### Nav2 Tuning Guide: 非圆形机器人应使用真实 footprint

Nav2 tuning guide 建议非圆形机器人使用真实 geometric footprint，因为 Nav2 的多个 planner/controller 可以使用完整 SE2 footprint 做碰撞检查，避免机器人卡在本来可以提前规避的位置。  
参考：

- https://docs.nav2.org/tuning/index.html

对当前系统的含义：

- 现在已经配置 polygon footprint，这是正确方向。
- 但需要继续确认 footprint 与真实/仿真底盘一致，尤其是前后偏心、轮子外扩、base_footprint 原点位置。
- 如果 footprint 过小，会贴桌脚；如果过大，会误判窄通道不可过。

### Nav2 Smac Planner: 可作为后续 cost-aware global planner 对比

Nav2 Smac Planner 文档说明，`SmacPlanner2D` 支持 circular differential-drive 和 omni-directional robots，并实现 cost-aware A*；`SmacPlannerLattice` 支持任意形状和自定义 motion model。  
参考：

- https://docs.nav2.org/configuration/packages/configuring-smac-planner.html

对当前系统的含义：

- 不建议第一步就切 planner。
- 但如果 NavFn 在 lab 里持续贴障碍，可以用 SmacPlanner2D 做 A/B。
- 如果将来需要更严格利用非圆形 footprint 或特殊三轮底盘模型，可以研究 SmacPlannerLattice。

## 建议顺序

1. 固定一组短距离 navigation test cases：
   - open area
   - near table
   - narrow passage
   - sofa / cabinet 附近
   - door 附近
2. 每个 case 同时记录：
   - target id / approach pose
   - global plan 是否生成
   - `/nav2/cmd_vel`
   - `/cmd_vel`
   - `/motion_policy/status`
   - final pose error
   - cancel 来源
   - `/nav2/cmd_vel` 的 `linear_y` 使用比例
   - `/cmd_vel` 是否保留 `linear_y`
   - 轨迹到障碍物的最小距离
3. 先做 A/B：
   - `--no-motion-policy`
   - `--motion-policy`
4. 如果 `--no-motion-policy` 明显改善：
   - 不再把 “align then forward” 作为 sim lab 默认策略。
   - 重新设计 motion profile，而不是继续调低 lateral。
5. 如果 `--no-motion-policy` 仍失败：
   - 查 MPPI lateral sampling 是否太弱。
   - 查 footprint / costmap / inflation / local obstacle layer。
   - 查 approach pose 是否落在高 cost 或窄通道不可达位置。
6. 不做人工切换的多 profile，改为逐项收敛一套默认参数：
   - 先提高 `vy_std / vy_max`，确认左上 / 右上微调出现。
   - 再调整 inflation / CostCritic，让路径离桌脚更远。
   - 再降低过强 forward/yaw preference，减少不必要大转向。
   - 最后确认前视 depth camera 的可见区域仍覆盖主要运动方向。
7. 如果单一默认策略仍无法覆盖所有场景，再考虑自动化 context-aware 调节，而不是人工 profile。
8. 最后再做 planner feasibility check。

## 调查清单

### 必查日志

```bash
cd /home/ruan-x/robot_ws
ls -td debug-output/sim_nav2_* | head
sed -n '1,220p' debug-output/<RUN>/latest_state.md
rg -n 'action_status|motion_policy|nav2_cmd_vel|cmd_vel|plan' debug-output/<RUN>/events.jsonl
rg -n 'Received a goal|Goal was canceled|No valid|collision|costmap|Failed|Aborted|Timed out' debug-output/<RUN>/ros-log
```

### 运行时 topic

```bash
ros2 topic echo /motion_policy/status --once
ros2 topic echo /nav2/cmd_vel --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /gicp_loc/score --once
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
```

### 判断规则

- `/nav2/cmd_vel` 合理，但 `/cmd_vel` 被改坏：查 `nav_mode_switch_node.py` / safety layer。
- `/nav2/cmd_vel` 本身为零：查 controller / costmap / goal tolerance。
- `/nav2/cmd_vel.linear_y` 长期接近 0：查 MPPI `vy_std` / `vy_max` / PathAngleCritic。
- `/nav2/cmd_vel.linear_y` 有值，但 `/cmd_vel.linear_y` 消失：查 motion policy / safety clamp。
- global plan 没生成：查 map / start-goal cost / TF。
- 已接近目标但 timeout：查 task tolerance / yaw tolerance / final approach。
- 只在窄通道失败：查 footprint、inflation、local costmap、approach pose。
- 轨迹贴桌脚：查 global inflation / path cost / CostCritic 权重 / approach candidate。

## 修正方向

### 短期

- real lab sim 默认使用 `--no-motion-policy`。
- 保留 safety layer clamp，不绕过安全链。
- agent/skill 层继续保留 dynamic timeout 和 task tolerance，但当前阶段不以它作为主要修正。
- debug recorder 记录 inferred goal，避免 agent action 没有 `/goal_pose` 时误判。
- 记录 MPPI 是否真正输出 lateral velocity。
- 建立 5-10 个固定导航场景，优先用同一组目标重复 A/B。

### 第一轮已应用参数

文件：

- `src/nav_pkg/config/nav2_params.yaml`

MPPI lateral:

```yaml
vy_std: 0.08
vy_max: 0.12
vy_min: -0.12
```

Local costmap inflation:

```yaml
inflation_radius: 0.45
cost_scaling_factor: 2.5
```

Global costmap inflation:

```yaml
inflation_radius: 0.60
cost_scaling_factor: 2.5
```

预期：

- 宽阔区域仍以前进为主。
- 桌椅附近出现合理左上 / 右上微调。
- global path 稍微远离桌脚、柜脚。
- local obstacle 不应把窄通道直接堵死。

第一轮不修改：

- footprint。
- planner plugin。
- Nav2 goal checker。
- 通用 `navigation.launch.py` 默认 motion policy。

sim 测试仍建议使用 `tools/sim/open_sim_nav2_windows.sh` 默认的 `--no-motion-policy`。

## 2026-07-07 第二轮修改

### Agent / Skill timeout

问题：

- `NavigationSkills.goto_pose()` 之前的 `dynamic_timeout` 本质仍是总耗时 timeout。
- sofa 这类远目标在机器人仍正常移动时被 agent cancel，违背了 “异常卡住才求助” 的设计目的。

修改：

- `dynamic_timeout=True` 时改为 progress watchdog。
- 每 10 秒检查一次：
  - 距离目标至少接近 `0.10m`，或
  - 机器人自身移动至少 `0.08m`，
  - 就重置 watchdog。
- 只有长期没有进展时才 cancel Nav2。
- 固定 timeout 的旧行为不变。

预期：

- 正常移动中的长路线不再被 agent 干扰。
- 真正卡住、原地抖动、无法推进时仍能主动取消并向 human 求助。

### MPPI / Critic

修改：

```yaml
batch_size: 1200
vy_std: 0.10
vy_max: 0.14
vy_min: -0.14
ay_max: 0.18
wz_std: 0.16
PathAlignCritic.cost_weight: 12.0
PathAngleCritic.cost_weight: 1.0
```

意图：

- 保持前向主导，适配前方 depth camera。
- 增强左前 / 右前斜向微调。
- 降低过度贴合 global path 的倾向。
- 减少不必要的大角速度修正。

测试观察点：

- `/cmd_vel.linear_y` 是否稳定存在，但不应长期接近上限。
- 窄桌腿附近是否更愿意斜向绕行。
- RViz 里 global path 是否仍可通，local trajectory 是否不再贴桌脚。
- 如果出现左右晃动，优先回退 `vy_std` 到 `0.08` 或 `vy_max` 到 `0.12`。

## Camera / LiDAR TF 动态校准思路

现象：

- RViz 中 static global map 和 local costmap/depth obstacle 有轻微偏移。
- 如果同一个桌脚/柜脚在 global map 和 camera depth 点云里稳定错开，优先怀疑 sensor TF，而不是先缩窄 costmap 边界。

注意：

- 不要直接再发布一条同名 `base_link -> camera_link` static TF。
- 当前 camera / LiDAR TF 来自 URDF + `robot_state_publisher`，重复发布同名 TF 会造成 TF 冲突。

先看当前 TF：

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo map base_footprint
```

sim 使用的参数位置：

```text
src/robot_sim/urdf/robot_sim.urdf.xacro
```

真实机器人使用的参数位置：

```text
src/robot_description/urdf/robot.urdf.xacro
```

优先微调：

- `camera_x`: 前后偏移，建议每次 `0.01-0.02m`。
- `camera_y`: 左右偏移，建议每次 `0.01-0.02m`。
- `camera_pitch`: 地面/近距离障碍高度投影不一致时调整，建议每次 `0.01-0.02rad`。
- `livox_x/y/yaw`: 如果 GLIM map 本身相对机器人姿态有系统偏差，再调 LiDAR。

快速迭代流程：

```bash
cd /home/ruan-x/robot_ws

# 修改 xacro 里的 camera_x / camera_y / camera_pitch 后，重新 build 或至少重新 source。
colcon build --symlink-install --packages-select robot_sim robot_description
source install/setup.bash

# 重启 sim/nav2 窗口，让 robot_state_publisher 重新发布 TF。
tools/sim/open_sim_nav2_windows.sh \
  --map maps/sim_real_lab_001/nav2.yaml \
  --ply maps/sim_real_lab_001/map_3d.ply \
  --initial-x 0.0 \
  --initial-y 0.0 \
  --initial-yaw 0.0 \
  --cleanup
```

RViz 判断方法：

- 固定一个清晰垂直物体，例如桌脚、柜边、cart 立柱。
- robot 停住，不要同时移动。
- 看 global static obstacle 和 local depth obstacle 是否稳定同方向错开。
- 一次只改一个参数，记录截图。
- 如果偏移随机器人朝向变化，优先查 yaw / pitch。
- 如果偏移方向基本固定，优先查 x / y 平移。

## 2026-07-07 倒退问题复盘与第三轮修改

最新 log：

```text
debug-output/sim_nav2_20260707_141242
```

结论：

- TF 校准后 local/global 对齐改善。
- Nav2 多段目标可以 `SUCCEEDED`，说明基础链路有效。
- 但第 1 段和第 3 段出现明显倒退：
  - 第 1 段 `cmd_vel.linear_x < -0.01` 约 `83.6%`。
  - 第 3 段 `cmd_vel.linear_x < -0.01` 约 `46.7%`。
- `/nav2/cmd_vel` 本身已经是负 `linear_x`，所以不是 relay 或底盘翻译错误，而是 MPPI 选择了后退轨迹。

原因判断：

- 第二轮把 `PathAngleCritic.cost_weight` 降到 `1.0`，forward preference 约束变弱。
- `vx_min=-0.24` 允许和前进同等幅度的长距离后退。
- 在某些目标姿态 / global path 组合下，MPPI 认为倒着贴 path 的代价更低。

第三轮决定：

```yaml
vx_min: -0.08
vy_std: 0.08
vy_max: 0.12
vy_min: -0.12
PathAlignCritic.cost_weight: 18.0
PathAngleCritic.cost_weight: 1.5
```

意图：

- 采用第一次修正后表现较好的 lateral 强度。
- 保留小幅后退，用于脱困或最后姿态微调。
- 禁止长距离倒退成为主要移动策略。
- 避免第二轮中过多横向移动 / 倒退策略。

下一次测试期待：

- 开阔路径主要 `linear_x > 0`。
- 窄空间允许短暂负 `linear_x`，但不应连续数十秒倒退。
- `/cmd_vel.linear_y` 仍然存在，用于左前 / 右前斜向绕行。
- 如果仍然倒退，下一步考虑把 `vx_min` 收到 `-0.03`。

## 2026-07-07 Relative Motion 改为直接微调

问题：

- `move_relative` / `move_relative_xy` / `rotate_relative` 之前会转换成 Nav2 `goto_pose`。
- 这导致 `前进 1m`、`turn right 5 degree`、`turn right 10 degree` 都受 global planner、local costmap、goal checker 影响。
- 小角度旋转还会被 `yaw_goal_tolerance=0.20rad` 吃掉，5-10 度可能被 Nav2 认为已经到达。

决定：

- relative motion 定位为 “human micro adjustment skill”。
- 行为应接近 joycon：直接发布速度，不创建 Nav2 goal。
- 发送入口使用 `/cmd_vel_raw`，仍经过 `cmd_vel_safety_node` 的 watchdog / speed clamp，再到 `/cmd_vel`。

当前实现：

- `move_relative`: 按 robot base frame 方向发布 `linear.x / linear.y`。
- `move_relative_xy`: 按 robot base frame 的 `dx / dy` 发布斜向速度。
- `rotate_relative`: 直接发布 `angular.z`。
- 仍用 TF 估计移动距离 / 旋转角度，用于停止和结果记录。

注意：

- 这些指令不再受 Nav2 costmap 保护。
- 适合小步调整，例如 `前進少し`、`右前方20cm`、`turn right 10 degree`。
- 不适合长距离导航；去 sofa / trash bin 仍应走 semantic target + Nav2。

## 2026-07-07 Global Inflation 窄通道测试

目的：

- 测试 “global planner 给出较大胆路径，local costmap + MPPI 再判断是否能通过”。
- 用于十分狭窄的 lab 场景，不代表最终安全参数。

修改：

```yaml
global_costmap.inflation_layer.inflation_radius: 0.20
global_costmap.inflation_layer.cost_scaling_factor: 5.0
```

预期：

- global path 更容易穿过窄缝。
- 如果 local costmap 判断太近，MPPI 仍可能不通过或慢速绕行。
- 如果出现贴家具/碰撞风险，需要回调 global inflation 或增加 approach candidate 选择逻辑。

## 2026-07-07 Long Distance 预转向策略

问题：

- 当目标距离较远、目标方向相对车头角度较大时，MPPI 仍可能选择大量横向移动 / 斜退，而不是先把车身朝向行进方向。
- 由于当前 robot 只有前方 depth camera 能看到近处障碍，长距离移动应尽量让 camera 朝向主要行进方向。
- 斜向 / 横向移动仍然重要，但更适合窄通道穿越和小范围微调，不应该成为长距离导航的主要姿态策略。

当前实现：

- 在 `goto_pose` 发送 Nav2 goal 前，根据当前 map pose 和目标 pose 计算目标相对车头方向。
- 满足以下条件时先直接旋转车身：
  - 目标距离 `>= 0.8m`
  - 目标方向相对车头 `>= 45deg`
- 预转向发布到 `/cmd_vel_raw`，仍经过 safety layer。
- 预转向不是 hard gate；即使没有完全达到角度，也继续发送 Nav2 goal。

预期：

- 长距离目标：更倾向先把前方 camera 朝向行进方向，再交给 Nav2 / MPPI。
- 近距离目标：不触发，保留左前 / 右前 / 横向的小范围微调能力。
- 如果目标几乎正前方，即使距离较远也不额外预转身。

未完成：

- 已加入第一版 local costmap clearance 判断。
- 如果 local costmap 明确显示旋转区域不够宽，预转身会跳过，并在 result metrics 里记录：
  - `local_clearance`
  - `pre_align_skip_reason: rotation_clearance_blocked`
- 如果 local costmap 不可用或 stale，暂时不阻止预转身，避免因为观测缺失让导航完全停住。

## 2026-07-07 静态家具穿模判断

现象：

- 桌子会真实卡住 / 撞上，说明桌子资产本身有 collider 或 collision proxy。
- 柜子附近可能发生穿模，说明视觉 mesh 存在，但 PhysX collision 可能没有正确启用或没有覆盖到外形。

判断：

- 对静态家具来说，关键不是 `Rigid Body`，而是 `Collider / Collision API`。
- `Rigid Body` 决定物体是否参与动力学、会不会被重力和碰撞推动。
- `Collider` 决定它是否能阻挡机器人。
- 大型柜子、书架、墙体、固定桌子通常应是 static collider，不建议加 dynamic rigidbody，否则可能抖动、下坠或被机器人推走。

后续处理：

- 对柜子 / 书架 / 固定家具检查是否有 collider。
- 如果资产自带 collider 不稳定，给外形添加简单 box collision proxy。
- proxy 可以不可见，但要保留 collision；这样 RGB 不应看到，depth 是否看到取决于渲染可见性，但 PhysX 会用于碰撞。

## 2026-07-07 Local Passage Policy 第一版

问题：

- 两侧都有障碍、前方清空时，机器人应该先朝能通过的方向穿过窄通道，然后再追远距离目标。
- 目前 global planner + MPPI 不会显式生成 “先穿过眼前通道” 的短期子目标。

当前实现：

- `goto_pose` 发送 Nav2 goal 前读取最近的 local costmap。
- 如果满足：
  - 长距离目标 `>= 0.8m`
  - front sector clear
  - left/right sector 都 close
- 则先发布一次 direct `/cmd_vel_raw` 前进 `0.35m`。
- 前进后重新读取当前位置、目标方向和 local clearance，再决定是否预转身 / 发送 Nav2 goal。
- result metrics 里记录：
  - `local_clearance`
  - `local_passage_result`

注意：

- 这不是替换 MPPI。
- 更像是在 MPPI 前面生成一个很短的局部阶段动作。
- 当前规则只用扇区统计，是第一版启发式；后续可以替换成基于 footprint swept volume / local costmap polygon check 的更严格版本。

## 2026-07-07 Map-Level Navigation Annotation

测试观察：

- 新策略下方向变化能力已经明显改善。
- 狭窄通道可以通过，但还不是每次稳定通过。
- 单靠运行时 `front_clear + sides_congested` 这种 reactive 判断，会在通道中段 / 通道出口附近出现不稳定。

新增文件：

- `maps/sim_real_lab_001/navigation_annotations.yaml`
- `maps/sim_real_lab_001/metadata.yaml` 已把它登记为 map bundle 文件。

设计意图：

- 把 “狭窄通道应该先走到出口” 当作地图结构知识，而不是完全交给 MPPI 或 local reactive policy。
- 当 global plan 穿过已知 narrow passage 时，先插入 passage `exit` 作为 temporary goal。
- 到达 temp goal 附近后，再恢复原始 semantic target。

当前 annotation 内容：

- `sofa_bookshelf_passage`
- `left_cabinet_toolbox_passage`
- `center_right_table_passage`

待接入：

- `SemanticMap` / navigation planner 还没有读取这个文件。
- 下一步需要在发 `goto_pose` 前：
  - 读取 map bundle annotation。
  - 判断 global plan 或 start/goal 是否穿过 passage。
  - 插入 `temp_goal.exit`。
  - temp goal 完成后继续原始目标。

注意：

- 这类似 waypoint，但不是用户任务层 waypoint。
- 它应该属于 map bundle，和 `nav2.yaml`、`semantic_map.yaml`、`semantic_segments.yaml` 一样，是地图基础信息。
- 坐标目前是 first-pass estimate，需要你在 RViz 中结合实际通过位置微调。

### 中期

- 收敛一套统一默认策略：
  - forward-dominant。
  - lateral-assisted。
  - camera-aware。
  - clearance-aware。
- motion relay 不再做强规则覆盖，只保留可解释的 safety clamp。
- 如果必须根据场景调整，优先做自动 context-aware scaling：
  - 根据 local cost / path curvature / obstacle proximity 连续调整速度和 lateral allowance。
  - 不让用户或 launch 手动选择多套 profile。

### 长期

- approach candidate 进入 planner feasibility check。
- 基于 robot footprint 做 clearance 评估。
- 对 segment / object region 生成多个候选接近点，并选择 path cost 最低而不是距离最近。
- 建立 “地图结构 / local cost -> 连续调节参数” 的机制：
  - open area: 直线、平滑、速度优先。
  - narrow area: clearance、侧移、低速优先。
  - near target: final approach / observation pose 优先。
  - 这些是自动调节维度，不是人工 profile。

## 关于之前 tolerance 调整为什么没有解决当前问题

结论：之前的修改是必要的，但它解决的是 “任务是否可以接受为成功”，不是 “底盘能不能更好地移动”。

原因：

1. 它在 agent / skill 层  
   `NavigationSkills.goto_pose()` 用 TF 判断当前 pose 是否进入 task tolerance。进入后会 cancel Nav2 并返回 success。

2. 它不是 Nav2 goal checker  
   `nav2_params.yaml` 里的 `SimpleGoalChecker` 仍然使用固定 `0.10m / 0.20rad`。

3. 当前 policy 主要为 warehouse shelf 写的  
   `navigation_tolerance_profiles.yaml` 里的 rule 匹配 `shelf / 棚 / 01-09` 等模式。`tool_box`、桌子、沙发、门附近目标未必会触发同样 profile。

4. 它通常在 timeout / retry 后才介入  
   这意味着机器人前面一段运动仍然由 Nav2 + MPPI + relay 决定。如果底盘在窄通道里走得很差，task tolerance 只能兜底，不能让路径变好。

5. 当前优先级应转到底层运动能力  
   real lab 的核心问题是 “怎么走得更像全向底盘”，包括侧移、斜移、后退微调、远离桌脚、减少不必要大转向。

## 2026-07-08 recovery 更新

已接入的策略：

- `goto_pose` 的 semantic recovery policy 会从 `semantic_map.yaml` 传到 Navigation Skill。
- `front_door` / `back_door` 是 `fixed_point`：
  - 不允许 relaxed tolerance。
  - 不允许 alternate goal。
  - 只能到固定门中心点。
- 区域目标默认允许：
  - near-goal direct micro-adjust。
  - relaxed region tolerance。
  - local-costmap-aware alternate goal。

local-costmap-aware alternate goal：

- 只使用机器人当前前方可信扇区：
  - front `+/-80 deg`
  - 距离 `0.3m ~ 1.8m`
- 候选点必须在 local costmap 中是 low-cost/free。
- 候选点会再用 footprint 近似半径检查 blocked / inflated / unknown ratio。
- 用于区域目标 blocked / timeout 后重新选择可达附近点。

测试重点：

- `go to sofa`：blocked 后是否能选到前方可见的可行 alternate goal。
- `go to rxy desk`：近目标 timeout 后是否先进行小范围横向/斜向 micro-adjust。
- `go to front door` / `go to back door`：确认不会 relaxed，也不会换目标点。
- RViz markers：确认 original goal 与 sampled alternate waypoint 能显示。
