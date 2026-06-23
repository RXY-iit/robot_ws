# Agent 自然语言导航与移动操作 Guidance

> 创建日期: 2026-06-13
> 范围: 自然语言机器人任务 agent、语义导航、未来移动机械臂操作
> 当前基础: Isaac Sim + ROS 2 Humble + FAST-LIO2/GICP + Nav2
> 核心原则: agent 只负责选择和编排已验证的 skill；真实行为由 ROS2/Nav2/MoveIt/感知节点执行。

## 当前起点

当前 sim navigation 已经可以作为 agent 的第一版执行后端：

```text
FAST-LIO2 + GICP localization
Nav2 NavigateToPose
Isaac Sim motion chain
/nav2/cmd_vel -> /cmd_vel -> sim robot motion
```

参考文档：

```text
todo-my/task-sim-real-dev/navigation/sim-navigation-guidance.md
todo-my/task-sim-real-dev/navigation/sim-navigation-change-log-20260611.md
```

agent 项目应该构建在这个稳定导航层之上。不要让 LLM 直接发布速度、TF、关节命令或任意 ROS message。安全结构应该是：

```text
自然语言
  -> 结构化任务
  -> 语义地图 / WorldState 查询
  -> 类型严格的 skill 调用
  -> ROS2 action/service/topic wrapper
  -> Nav2 / perception / MoveIt / controller
```

## 推荐系统结构

保留 `Agent-Frame-choose.md` 里的方向，但把边界定义得更严格：

```text
Voice/Text UI
  -> Intent Parser
  -> LangGraph Mission Graph
  -> Pydantic AI typed tools
  -> Robot Skill API
  -> ROS2 actions/services/topics
  -> Nav2 / perception / MoveIt / ros2_control
```

各层职责：

- LangGraph：长任务状态机、重试、恢复、人类确认、任务日志。
- Pydantic AI：工具输入输出 schema、结构化 LLM 输出、参数校验。
- ROS2 Skill API：封装 Nav2、感知、语义地图、机械臂、安全检查等能力。
- Nav2：移动底盘导航执行层。
- MoveIt 2 或专用 arm controller：未来机械臂运动执行层。
- Isaac Sim：导航和未来移动操作的第一回归测试环境。

## Milestone 0: Agent 项目骨架

### Demo 目标

暂时不让机器人移动。系统能接收自然语言命令，并输出通过校验的结构化任务。

示例：

```text
输入: 去叉车旁边
输出:
  task_type: navigate_to_semantic_target
  target_name: forklift
  relation: near
  require_confirmation: false
```

### 要做的内容

- 新建 agent package 或 tools 区域，先不要改稳定的 Nav2 栈。
- 定义共享数据结构：
  - `RobotState`
  - `WorldState`
  - `TaskState`
  - `SkillResult`
  - `SafetyDecision`
- 规划配置文件位置：
  - 语义地图 YAML/JSON
  - skill registry YAML/JSON
  - agent runtime config
  - debug/log 输出目录

### 在本 workspace 开发时注意

- 初期不要混入 `src/nav_pkg` 和 `src/localization_pkg`。
- 可以新建 `src/robot_agent`，也可以先用 `tools/agent/` 做轻量原型。
- agent 能可靠调用 mock skill 之前，不要修改 navigation launch 文件。
- 日志建议放到 `debug-output/agent_*`，风格对齐现有 Nav2 debug session。

## Milestone 1: Semantic Map v0

### Demo 目标

agent 可以把这些自然语言目标解析到 `map` frame 下的导航目标：

```text
去叉车旁边
去6号架子
到电梯前
```

第一版不需要视觉，只依赖事前标注的语义地图。

预期 demo：

```text
User: 去叉车旁边
Agent:
  解析 "叉车" -> semantic object "forklift_01"
  选择 approach pose
  打印或发送 Nav2 goal candidate
```

### 要做的内容

- 定义简单的 semantic map 文件。
- 标注静态物体、区域、approach poses。
- 可选：发布 RViz marker，方便检查语义物体和 approach pose。
- 增加语义查询 skill：

```text
resolve_semantic_target(name, relation) -> object_id, pose, confidence
```

建议 schema：

```yaml
objects:
  - id: forklift_01
    names: ["forklift", "叉车"]
    category: vehicle
    frame_id: map
    pose:
      x: 1.0
      y: 2.0
      yaw: 0.0
    footprint:
      type: polygon
      points: []
    approach_poses:
      - id: front
        x: 0.5
        y: 1.5
        yaw: 0.0
    affordances: ["navigate_near", "inspect"]
    safety_zone_m: 0.5
    confidence: 1.0
```

### 在本 workspace 开发时注意

- 所有可导航目标优先用 `map` frame，对齐 Nav2 和 localization。
- 物体 alias 可以同时写英文、中文、日文，方便语音/文本输入。
- 第一版先用人工验证过的 pose。open-vocabulary 3D scene graph 放到后面。
- semantic map 独立于 GLIM/Nav2 map 生成流程，不要自动改导航地图。
- 如果做 RViz marker publisher，让它只读 semantic map，不负责改文件。

## Milestone 2: Nav Skill API

### Demo 目标

自然语言可以触发真实 sim 导航，到达某个语义目标附近。

预期 demo：

```text
User: 去叉车旁边
Agent:
  解析命令
  查询 forklift_01 approach pose
  调用 Nav2 NavigateToPose
  监控执行结果
  报告成功或失败
```

### 要做的内容

- 实现类型明确的导航 skill wrapper：
  - `goto_pose(x, y, yaw, frame_id="map")`
  - `goto_named_place(name)`
  - `goto_object_nearby(object_name, relation="near")`
  - `cancel_navigation()`
  - `get_robot_pose()`
  - `check_navigation_health()`
- 通过 Nav2 `NavigateToPose` action 执行，不要直接发布 `/goal_pose` 当作正式接口。
- 返回结构化结果：

```text
success
status
final_pose
navigation_time_sec
recoveries
message
```

### 能完成的 demo

这个阶段应该支持：

```text
去叉车旁边
去6号架子前
移动到电梯门口
回到起点
```

此阶段不要求支持“箱子里面找工具”这种细粒度搜索。

### 在本 workspace 开发时注意

- sim 测试使用 `tools/sim/open_sim_nav2_windows.sh --cleanup`。
- sim 保持 `ROS_DOMAIN_ID=20`。
- 发送任务前检查 Nav2 lifecycle nodes 是否 active。
- 不要绕过 `nav_mode_switch_node` 或 safety command chain。
- 如果缺 `/clock`、缺 `map -> odom`、`/gicp_loc/score` 不健康，agent 应该 precheck fail。
- 复用现有 debug 习惯，例如 `tools/nav2_debug_recorder.py` 和 `tools/nav2_debug_monitor.py`。

## Milestone 3: Pydantic AI Tool Contract

### Demo 目标

LLM 或 parser 可以把命令转成严格类型的任务；无效或不安全命令会在机器人动作前被拒绝。

预期 demo：

```text
User: 尽量快一点贴着墙走过去
Agent:
  识别高速/贴墙风险
  限制速度或拒绝目标
  要求确认或返回安全错误
```

### 要做的内容

- 用 Pydantic 定义 agent 输入输出模型。
- 校验以下内容：
  - frame id
  - speed limit
  - target confidence
  - 是否需要人类确认
  - 最大移动距离
  - allowed skill list
- 明确任务类型：

```text
navigate_to_place
navigate_to_object
inspect_object
search_near_place
ask_human
```

- 增加 schema 级拒绝规则：
  - 禁止直接电机命令
  - 未知目标必须澄清或安全失败
  - 没有安全状态时禁止 manipulation

### 能完成的 demo

这个阶段应能稳定处理：

```text
去叉车旁边
到6号架子前面
在电梯前停下
找一下红色箱子在哪个区域
```

此阶段的“找红色箱子”可以先是 semantic lookup 或视觉 stub，不要求完整开放词汇检测。

### 在本 workspace 开发时注意

- 第一版可以不用 cloud LLM，用 deterministic parser/mock 先跑通结构。
- 即使用简单 parser，所有 tool call 也要走 typed schema。
- 给 schema validation 写小测试，比测试 LLM 文案更有价值。
- 不要把地图坐标写进 prompt；LLM 应该调用 semantic map skill 查询。

## Milestone 4: LangGraph Mission Flow

### Demo 目标

agent 可以执行多步骤任务，并具备状态保存、观察、验证、重试、人类确认路径。

预期 demo：

```text
User: 去6号架子旁边，找红色箱子
Agent:
  解析任务
  查询 shelf_06
  导航到 shelf_06 approach pose
  运行视觉/语义搜索 stub
  报告候选结果
  低置信度时询问人类
```

### 要做的内容

- 建立 mission graph：

```text
parse_instruction
  -> resolve_targets
  -> safety_precheck
  -> execute_step
  -> observe
  -> verify
  -> done / retry / ask_human / fail_safe
```

- 将 task state 持久化到磁盘。
- 给每一步加 timeout 和 retry policy。
- 用 JSONL 记录 task event log。
- 增加 dry-run 模式：

```text
agent --dry-run "去叉车旁边"
```

### 能完成的 demo

这个阶段应支持：

```text
去叉车旁边
去6号架子找红色箱子
到电梯前并确认是否看到按钮
```

此阶段不应该自动按按钮或抓取物体。

### 在本 workspace 开发时注意

- graph 先保持小。不要过早拆出很多 specialized agents。
- 只有普通函数变得难以维护时，再增加 graph node。
- 每个 node 都应该消费和返回 typed state。
- 日志放在 Nav2 debug output 附近，方便对比 sim session。
- 明确使用 `FAILED_PRECHECK`、`FAILED_NAVIGATION`、`FAILED_PERCEPTION`、`NEEDS_HUMAN` 等状态。

## Milestone 5: Perception v0

### Demo 目标

机器人可以导航到已知区域，并验证或搜索一个视觉目标。

预期 demo：

```text
User: 去6号架子找红色箱子
Agent:
  导航到 shelf_06
  获取 camera frame
  检测 red box candidate
  报告相对方向 / 图像证据 / confidence
```

### 要做的内容

- 先从简单感知开始：
  - 已知 demo 物体的颜色阈值
  - 必要时用 fiducial marker
  - 固定类别 detector，例如 boxes/buttons
- 后续再接开放词汇感知：
  - Grounding DINO 风格检测
  - SAM/SAM2 风格分割
  - VLM 二次确认
- 将检测结果转成 WorldState observation：

```text
object_id
label
confidence
camera_frame_bbox
estimated_3d_position
map_pose_if_available
timestamp
```

### 能完成的 demo

这个阶段应支持：

```text
找红色箱子
检查电梯按钮是否在视野里
确认叉车附近是否有目标物
```

不要求可靠 grasp pose。

### 在本 workspace 开发时注意

- semantic/perception 数据采集可以复用 `tools/record_semantic_bag.sh`。
- 感知输出和 semantic map 分开。动态检测结果更新 `WorldState`，不要自动改静态地图。
- 每个检测结果必须带 confidence 和 timestamp。
- sim 中先确认 camera topics 和 frame IDs，再围绕它们写 agent 逻辑。
- 重视觉推理不要阻塞导航栈，最好独立进程/节点。

## Milestone 6: 移动机械臂预集成

### Demo 目标

机器人导航到目标附近，未来机械臂只移动到安全 named pose 或预操作位。

预期 demo：

```text
User: 到电梯前，准备按按钮
Agent:
  导航到 elevator approach pose
  检测 button panel candidate
  机械臂移动到 safe pre-press pose
  最终动作前询问人类
```

### 要做的内容

- 在真实 arm 执行前先定义 manipulation skill contract：
  - `move_arm_named_pose(name)`
  - `move_arm_to_pregrasp(target_id)`
  - `move_arm_to_prepress(button_id)`
  - `stop_arm()`
  - `check_arm_ready()`
- 在这些 skill 后面接 MoveIt 2 或选定的 arm controller。
- 定义 workspace limit、collision objects、force/velocity limits。
- 增加 manipulation safety gates：

```text
arm_enabled
base_stopped
target_confirmed
collision_scene_ready
human_confirmed_for_contact
```

### 能完成的 demo

这个阶段应支持：

```text
找到电梯按钮并移动到预按压姿态
识别开关并把末端移动到安全观察位
到架子前准备抓取红色箱子
```

最终接触动作仍应要求人类确认。

### 在本 workspace 开发时注意

- 不要把 arm controller 实验混进稳定 navigation package。
- 机械臂靠近架子、电梯、叉车前，先加入 collision objects。
- 底盘必须停止、Nav2 goal 完成后，才允许 arm motion。
- LLM 永远不能直接生成 joint trajectory 或 Cartesian path。
- 任何新 contact 行为必须 sim-first。

## Milestone 7: Contact Skills

### Demo 目标

系统可以在 sim 中执行有限、可验证的接触动作，之后再迁移到真机。

预期 demo：

```text
User: 找到电梯按钮，确认后按下
Agent:
  导航到 elevator
  检测 button
  移动到 pre-press pose
  询问人类确认
  在 force/position limit 内按下
  后退
  验证结果
```

### 要做的内容

- 实现有边界的 contact skills：
  - `press_button(button_id, force_limit, travel_limit)`
  - `toggle_switch(switch_id, direction)`
  - `grasp_object(object_id, grasp_id)`
  - `place_object(location_id)`
- 增加动作后验证：
  - 视觉变化
  - 力/位置结果
  - 任务相关状态
- 增加立即 abort 路径。

### 能完成的 demo

这个阶段应支持：

```text
按下已识别按钮
拨动已识别开关
抓取简单已知物体
把工具/箱子移动到指定位置
```

### 在本 workspace 开发时注意

- 所有 contact skill 都要窄而明确。
- 每个 contact skill 都必须有 force/velocity/travel limits。
- 真机执行前必须有物理急停方案。
- 每次尝试都记录图像、pose、动作结果。
- 验证失败时应该退到安全位，不要盲目重试。

## Milestone 8: Memory And Skills

### Demo 目标

agent 可以记住过去任务结果，并复用稳定的操作流程。

预期 demo：

```text
User: 像上次一样去工具箱那里
Agent:
  回忆上次成功的 tool_box 位置
  检查 semantic map/object observation 是否仍有效
  如果记忆过期，则请求确认后导航
```

### 要做的内容

- 增加 memory layers：
  - episodic memory：任务日志和结果
  - semantic memory：alias、物体位置、稳定事实
  - skill memory：成功参数和失败记录
- 增加 memory freshness 规则。
- 增加“多次观测后提升为 semantic map”的流程。
- 给 skill registry 加 versioning。

### 能完成的 demo

这个阶段应支持：

```text
记住上次红色箱子的位置
记住某个按钮/开关的操作方式
根据过去失败记录选择更保守 approach pose
生成任务报告
```

### 在本 workspace 开发时注意

- memory 不能静默覆盖 static semantic map。
- 学到的数据默认低置信度，除非人工确认。
- 保存足够上下文，方便在 sim 里 replay failure。
- memory 应该可检查，例如 JSONL/YAML/SQLite，不要只藏在 prompt 里。

## Skill Registry 要求

每个 skill 都应该有 registry 条目：

```text
name
description
input_schema
output_schema
preconditions
side_effects
timeout_sec
retry_policy
safety_level
requires_human_confirm
sim_supported
real_supported
```

示例：

```yaml
skills:
  - name: goto_object_nearby
    safety_level: navigation
    requires_human_confirm: false
    timeout_sec: 120
    preconditions:
      - localization_healthy
      - nav2_active
      - target_pose_known
    sim_supported: true
    real_supported: true
```

## 安全规则

第一版就应该加入这些硬规则：

- agent 不能直接发布 raw `/cmd_vel`。
- agent 不能直接发布 raw joint commands。
- agent 不能根据 LLM 输出动态创建任意 ROS topic 名。
- 未知语义目标必须澄清，或安全失败。
- 低置信度感知结果不能触发接触动作。
- 底盘导航和机械臂运动不能在无控制状态下同时运行。
- contact actions 必须有明确的 skill limits，初期必须人类确认。
- 每个 mission 都必须有 cancel/abort 行为。

## 评测计划

每次 demo run 记录：

```text
task_parse_success
semantic_resolution_success
precheck_success
nav_goal_success
nav_recovery_count
perception_confidence
human_confirmation_count
task_success
failure_state
total_time_sec
```

建议第一组评测任务：

```text
1. 去叉车旁边
2. 去6号架子前
3. 去电梯前
4. 去6号架子找红色箱子
5. 到电梯前确认按钮是否可见
```

每个任务按这个顺序跑：

```text
dry-run
sim execution
debug log review
换一个稍微不同的初始位姿再跑一次
```

## 近期推荐实现顺序

建议按这个顺序开发：

```text
1. Semantic map v0
2. Nav2 NavigateToPose skill wrapper
3. Deterministic parser 或 Pydantic schema parser
4. Dry-run CLI
5. Sim execution CLI
6. LangGraph mission flow
7. Perception v0
8. Manipulation pre-integration
```

第一版最小可用 demo：

```text
"去叉车旁边"
自然语言 -> semantic map -> Nav2 goal -> success report
```

第二阶段扩展为：

```text
"去6号架子找红色箱子"
navigation -> perception -> verification -> report / ask human
```

未来移动操作 demo：

```text
"找到电梯，按下按钮"
navigation -> perception -> pre-press -> human confirm -> bounded contact -> verify
```

