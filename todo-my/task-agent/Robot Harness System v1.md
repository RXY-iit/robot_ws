我觉得你现在到了一个很关键的节点。

其实你已经完成了很多机器人项目最难的部分：

```text
Real Robot
    ↓
ROS2
    ↓
FAST-LIO2
    ↓
GLIM
    ↓
Nav2
    ↓
Isaac Sim
    ↓
Real2Sim
```

很多团队会卡在这里。

而你下一阶段真正的问题已经不是：

```text
如何导航？
如何建图？
如何定位？
```

而是：

```text
机器人如何完成任务？
```

这实际上就是从：

```text
Robot Platform
```

进入：

```text
Robot Harness System
```

阶段。

---

# 我建议先把整个计划重构

你原来的：

```text
Ph0
Ph1
Ph2
Ph3
Ph4
Ph5
Ph6
Ph7
Ph8
Ph9
```

其实是：

```text
技术导向
```

规划。

例如：

```text
Agent Core
Memory
VLM
RL
Digital Twin
```

看起来很合理。

但有个问题：

你很难判断：

```text
做到什么程度算完成？
```

---

我更建议改成：

# 第一层：机器人能力树

```text
L1 Platform Layer
L2 Skill Layer
L3 Agent Layer
L4 Fleet/Digital Twin Layer
```

---

# L1 Platform Layer（已基本完成）

对应你现在的成果

```text
Real Robot
Isaac Sim
ROS2
Localization
Navigation
```

目标：

```text
机器人能稳定移动
```

---

完成标准：

```text
Navigation Success >90%
```

---

# L2 Skill Layer

这里不要直接做Agent。

先做：

```text
Skill
```

---

例如：

### MoveTo

```text
去A点
```

---

### LookAt

```text
转头观察
```

---

### SearchObject

```text
搜索红色箱子
```

---

### PickObject

```text
抓取
```

---

### Dock

```text
自动回充
```

---

### AskHuman

```text
请求人工帮助
```

---

本质：

```python
skill.execute()
```

---

未来所有Agent都调用这些。

---

# L3 Agent Layer

这里才开始：

```text
Mission Agent
```

---

你的第一版我建议：

# Agent v1

只有三个Agent

```text
Mission Agent

   ↓

Navigation Agent

   ↓

Vision Agent
```

---

不要一开始就：

```text
Mission
Navigation
Vision
Manipulation
Recovery
Memory
Planner
WorldModel
```

太重。

---

# 第一版任务

例如：

```text
去实验室找红色箱子
```

---

Mission Agent

负责：

```text
拆任务
```

---

Navigation Agent

负责：

```text
去目标区域
```

---

Vision Agent

负责：

```text
识别目标
```

---

完成。

---

这是最小闭环。

---

# Agent v2

加入：

```text
Recovery Agent
```

---

例如：

导航失败

↓

Agent发现

↓

重新规划

↓

继续执行

---

这是Agent开始有价值的时候。

---

# Agent v3

加入：

```text
Manipulation Agent
```

---

任务：

```text
找到箱子
抓取箱子
放到指定位置
```

---

这时候才需要机械臂。

---

# Agent v4

加入：

```text
Memory Agent
```

---

记住：

```text
红箱子通常在A区

充电桩在B区

某个门打不开
```

---

长期运行。

---

# Mission Agent应该拆成什么

我建议：

```text
Mission Agent
```

下面拆：

```text
Task Planner

State Manager

Skill Dispatcher

Mission Memory
```

---

Task Planner

负责：

```text
用户任务
→
子任务
```

---

例如：

```text
找红箱子
```

变成：

```text
导航到A区

搜索

确认

返回
```

---

State Manager

负责：

```text
当前任务进行到哪里
```

---

Skill Dispatcher

负责：

```text
调用哪个Skill
```

---

Mission Memory

负责：

```text
记录执行历史
```

---

# Navigation Agent应该解决什么

输入：

```text
目标位置
```

---

调用：

```text
Nav2
```

---

输出：

```text
成功
失败
剩余距离
```

---

需要解决：

```text
Goal生成

路径监控

超时判断
```

---

# Vision Agent应该解决什么

输入：

```text
找红色箱子
```

---

调用：

```text
Realsense

VLM

YOLO

SAM2
```

---

输出：

```text
发现目标

目标位置

置信度
```

---

# Recovery Agent

这是我最推荐你做的。

因为你之前研究方向就有：

```text
Generative AI Recovery
```

---

输入：

```text
任务卡住
```

---

例如：

```text
3分钟没移动

Nav2失败

检测不到目标
```

---

输出：

```text
换路线

重新搜索

调用人工
```

---

这部分最容易发论文。

---

# 关于你的年度计划

我建议完全重写。

不要：

```text
Ph2 Agent Core

Ph4 Memory

Ph5 VLA
```

这种技术导向。

改成：

# Phase 2

Skill Layer

```text
MoveTo
SearchObject
LookAt
```

---

# Phase 3

Agent v1

```text
Mission Agent

Navigation Agent

Vision Agent
```

---

# Phase 4

Recovery Agent

```text
Auto Recovery

Replanning
```

---

# Phase 5

Manipulation Agent

```text
Arm

Pick&Place
```

---

# Phase 6

Memory + World Model

```text
长期运行
```

---

# Phase 7

Digital Twin

这里不要单独做。

因为你已经在做：

```text
Isaac Sim
=
Digital Twin的一部分
```

---

真正的Digital Twin应该定义为：

```text
Real Robot

↕

Isaac Sim

↕

Mission State

↕

Environment State
```

---

不是单独一个Phase。

---

我建议下一步我们专门花一次讨论，把：

```text
Robot Harness System v1
```

画成一个真正能实现的架构图：

```text
Mission Agent
      ↓
Task Planner
      ↓
Skill Dispatcher
      ↓
Navigation Skill
Vision Skill
Recovery Skill
      ↓
ROS2
      ↓
Nav2
FAST-LIO2
GLIM
Isaac Sim
```

然后再反推：

```text
2026.06~2027.03
每个月做什么
每个月验收什么
```

这样会比现在的计划清晰很多，而且更符合你当前已经完成的进度。

下面这版可以作为你后面年度计划里 **L2 Skill Layer / L3 Agent Layer** 的核心说明。

---

# 总体关系

你的系统可以理解成 4 层：

```text
L4 User / LLM / Mission Interface
        ↓
L3 Agent Layer
        ↓
L2 Skill Layer
        ↓
L1 ROS2 / Nav2 / Sensors / Isaac Sim / Real Robot
```

最重要的是：

```text
LLM 不直接控制机器人
Agent 不直接发 /cmd_vel
Skill 不负责思考任务意义
ROS 只负责真实执行
```

也就是说：

```text
LLM 负责理解
Agent 负责决策
Skill 负责封装动作
ROS/Nav2 负责执行
```

---

# L2 Skill Layer 是什么

L2 Skill Layer 是把底层 ROS 功能包装成“可调用能力”。

不是直接让上层去操作：

```text
/cmd_vel
/nav2_goal
/camera/image
/tf
```

而是包装成：

```text
MoveTo(location)
SearchObject(object)
LookAt(target)
DetectObject(query)
PickObject(object)
RecoverNavigation()
```

这样上层 Agent 只需要调用 Skill，不需要理解 ROS topic 的细节。

---

# L2 Skill Layer 的作用

它解决的问题是：

```text
ROS系统太底层
LLM/Agent不应该直接碰topic
每个动作需要统一输入输出
每个动作需要知道成功/失败/超时
```

例如 Navigation 不是简单发一个 goal。

真正需要包括：

```text
发送目标点
监控 Nav2 状态
判断是否卡住
判断是否到达
失败时返回原因
必要时取消任务
```

所以 Skill 是一个“动作封装器”。

---

# L2 Skill Layer 推荐结构

```text
robot_harness/
├── skills/
│   ├── base_skill.py
│   ├── navigation/
│   │   ├── move_to_skill.py
│   │   ├── patrol_skill.py
│   │   └── explore_area_skill.py
│   ├── vision/
│   │   ├── detect_object_skill.py
│   │   ├── search_object_skill.py
│   │   └── estimate_object_pose_skill.py
│   ├── manipulation/
│   │   ├── pick_skill.py
│   │   ├── place_skill.py
│   │   └── handover_skill.py
│   ├── recovery/
│   │   ├── nav_recovery_skill.py
│   │   ├── reobserve_skill.py
│   │   └── ask_human_skill.py
│   └── registry.py
```

---

# Base Skill 应该统一什么

每个 Skill 最好有统一接口：

```python
class BaseSkill:
    name: str
    description: str
    input_schema: dict
    output_schema: dict

    def execute(self, params) -> SkillResult:
        pass
```

输出统一成：

```python
SkillResult(
    success=True,
    status="completed",
    data={...},
    error=None,
    confidence=0.92
)
```

这样 Agent 可以稳定判断：

```text
成功
失败
失败原因
是否需要 Recovery
```

---

# Navigation Skill 怎么实现

## MoveToSkill

上层调用：

```python
MoveToSkill.execute({
    "target": "lab_table_area"
})
```

Skill 内部做：

```text
target name
 ↓
查 waypoint database
 ↓
转换成 map 坐标
 ↓
发送 Nav2 NavigateToPose action
 ↓
监控 feedback
 ↓
判断成功/失败/超时
 ↓
返回结果
```

底层连接：

```text
MoveToSkill
    ↓
ROS2 Action Client
    ↓
/navigate_to_pose
    ↓
Nav2 BT Navigator
    ↓
Controller Server
    ↓
/cmd_vel
    ↓
Real Robot or Isaac Sim
```

相关技术名词：

```text
ROS2 Action
Nav2 NavigateToPose
Behavior Tree
Lifecycle Node
TF2
Costmap
Waypoint
PoseStamped
Action Feedback
Goal Status
```

---

# Vision Skill 怎么实现

## DetectObjectSkill

上层调用：

```python
DetectObjectSkill.execute({
    "query": "red box"
})
```

内部做：

```text
获取 RGB / Depth
 ↓
调用 detector / VLM / YOLO / SAM
 ↓
得到 bbox / mask
 ↓
用 depth 估计 3D 位置
 ↓
通过 TF 转到 map 或 base_link
 ↓
返回 object pose
```

底层连接：

```text
DetectObjectSkill
    ↓
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera_info
/tf
    ↓
YOLO / SAM / VLM
    ↓
object_pose
```

输出例子：

```json
{
  "found": true,
  "object": "red box",
  "bbox": [320, 180, 120, 90],
  "pose_frame": "map",
  "position": [2.1, -0.4, 0.6],
  "confidence": 0.86
}
```

相关技术名词：

```text
YOLO
SAM2
GroundingDINO
Open-Vocabulary Detection
VLM
RGB-D
Point Cloud
CameraInfo
TF transform
Object Pose Estimation
```

---

# SearchObjectSkill 是 Vision + Navigation 的组合 Skill

这个很适合你的第一版 Agent Task。

上层调用：

```python
SearchObjectSkill.execute({
    "object": "red box",
    "area": "lab"
})
```

内部流程：

```text
去搜索区域
 ↓
旋转观察
 ↓
调用 DetectObject
 ↓
如果没找到，移动到下一个观察点
 ↓
重复
 ↓
找到后返回位置
```

它实际上会调用：

```text
MoveToSkill
LookAroundSkill
DetectObjectSkill
```

所以 Skill Layer 内部也可以组合。

---

# Manipulation Skill 怎么实现

第一版不要太复杂。

先定义接口：

```python
PickObjectSkill.execute({
    "object_pose": [...],
    "grasp_type": "top_grasp"
})
```

内部连接：

```text
PickObjectSkill
    ↓
MoveIt2 / cuMotion / Isaac Motion Generation
    ↓
joint trajectory
    ↓
ros2_control
    ↓
arm controller
```

相关技术名词：

```text
MoveIt2
Motion Planning
Inverse Kinematics
Grasp Pose
End Effector
Trajectory Planning
ros2_control
JointTrajectoryController
```

第一阶段可以先做 Sim Only：

```text
Isaac Sim 中完成 pick demo
真实机械臂以后再接
```

---

# Recovery Skill 怎么实现

Recovery 是你项目很有价值的部分。

常见输入：

```text
Nav2 failed
robot stuck
object not found
low confidence
localization unstable
```

RecoverySkill 做：

```text
分析失败原因
 ↓
选择恢复动作
 ↓
执行恢复
 ↓
重新返回上层
```

例子：

```text
Navigation failed
    ↓
clear costmap
    ↓
rotate in place
    ↓
retry from nearby waypoint
```

底层连接：

```text
/clear_entire_costmap
/cmd_vel
/nav2 lifecycle service
/tf
/diagnostics
```

相关技术名词：

```text
Failure Detection
Timeout
Retry Policy
Fallback
Costmap Clearing
Behavior Tree Recovery
Stuck Detection
Watchdog
Health Monitor
```

---

# L3 Agent Layer 是什么

L3 Agent Layer 是“会思考和决策”的层。

它不直接操作 ROS，而是调用 L2 Skill。

关系是：

```text
Mission Agent
    ↓
decide next skill
    ↓
Skill Layer
    ↓
ROS/Nav2
```

---

# L3 推荐第一版结构

```text
robot_harness/
├── agents/
│   ├── mission_agent.py
│   ├── navigation_agent.py
│   ├── vision_agent.py
│   ├── recovery_agent.py
│   └── agent_state.py
├── planner/
│   ├── task_planner.py
│   ├── skill_planner.py
│   └── policy.py
├── memory/
│   ├── short_term_memory.py
│   ├── world_state.py
│   └── event_log.py
└── llm/
    ├── llm_client.py
    ├── prompt_templates.py
    └── tool_schema.py
```

---

# Mission Agent 负责什么

Mission Agent 是最高层机器人任务管理器。

它负责：

```text
理解用户目标
拆成子任务
选择下一个 Skill
监控任务状态
失败时调用 Recovery
决定任务是否完成
```

例子：

用户输入：

```text
去实验室找红色箱子，然后回到起点
```

Mission Agent 拆成：

```text
1. MoveTo(lab_area)
2. SearchObject(red box)
3. ConfirmObject(red box)
4. MoveTo(start_point)
5. ReportResult()
```

---

# Navigation Agent 负责什么

Navigation Agent 不等于 Nav2。

Nav2 是底层导航系统。

Navigation Agent 是使用 Nav2 的决策层。

它负责：

```text
目标点选择
waypoint管理
路径执行监控
导航失败判断
导航恢复策略选择
```

例如：

```text
Mission Agent: 去 A 区
Navigation Agent:
    - A区入口在哪？
    - 应该走哪个 waypoint？
    - Nav2 是否成功？
    - 如果失败是否换路线？
```

---

# Vision Agent 负责什么

Vision Agent 不只是调用 YOLO。

它负责：

```text
决定看哪里
决定用哪个识别模型
管理识别置信度
把视觉结果转成世界状态
```

例如：

```text
找红色箱子
```

Vision Agent 决定：

```text
先转头扫描
再调用 YOLO/SAM
置信度低时换角度
用 depth 估计位置
写入 World State
```

---

# Recovery Agent 负责什么

Recovery Agent 是专门处理失败的 Agent。

它接收：

```text
failure_type
failure_context
current_state
```

然后决定：

```text
retry
replan
backtrack
ask human
abort mission
```

例子：

```text
SearchObject 失败
    ↓
Recovery Agent 判断：
    可能是视角不够
    ↓
执行 ReobserveSkill
    ↓
再 SearchObject
```

---

# LLM 和 Agent 的关系

LLM 不应该一直控制机器人。

建议关系是：

```text
LLM = reasoning / planning helper
Agent = runtime controller
Skill = executable tools
ROS = robot execution
```

也就是说：

```text
LLM 给计划
Agent 执行和检查
失败时再问 LLM
```

不要让 LLM 每 0.1 秒控制机器人。

---

# 推荐运行方式

## 不推荐

```text
LLM
 ↓
直接发布 /cmd_vel
```

危险，不稳定，不可复现。

---

## 推荐

```text
LLM
 ↓
生成结构化 plan
 ↓
Mission Agent 验证 plan
 ↓
Skill Dispatcher 调用 Skill
 ↓
ROS2 执行
```

例如 LLM 输出：

```json
{
  "plan": [
    {"skill": "MoveTo", "args": {"target": "lab_area"}},
    {"skill": "SearchObject", "args": {"object": "red box"}},
    {"skill": "MoveTo", "args": {"target": "home"}}
  ]
}
```

Mission Agent 检查：

```text
Skill是否存在
参数是否合理
目标点是否存在
是否安全
```

然后执行。

---

# 和 ROS2 的连接方式

建议 L2 Skill Layer 本身就是 ROS2 node，或者内部持有 ROS2 client。

例如：

```text
robot_harness_node
    ├── Nav2 Action Client
    ├── TF Buffer
    ├── Image Subscriber
    ├── Depth Subscriber
    ├── Service Client
    └── Skill Server
```

外部 Agent 可以通过：

```text
Python function call
ROS2 service
ROS2 action
HTTP/gRPC
```

调用 Skill。

第一版我建议：

```text
Agent 和 Skill 都写 Python
同一个进程内 function call
底层用 rclpy 接 ROS2
```

简单、快、好调试。

后面再拆成 ROS2 service/action。

---

# 第一版推荐架构

```text
User Command
    ↓
LLM Planner
    ↓
Mission Agent
    ↓
Skill Dispatcher
    ↓
MoveToSkill / SearchObjectSkill / RecoverSkill
    ↓
ROS2 Interface
    ↓
Nav2 / TF / Camera / Isaac Sim
```

---

# 第一版最小任务

我建议你第一版就做这个：

```text
Task:
Go to area A, search for red box, report result.
```

需要的 Skill：

```text
MoveToSkill
LookAroundSkill
DetectObjectSkill
SearchObjectSkill
ReportSkill
NavRecoverySkill
```

需要的 Agent：

```text
Mission Agent
Navigation Agent
Vision Agent
Recovery Agent
```

但第一版实现时可以先把 Navigation/Vision/Recovery Agent 写成 Mission Agent 内部模块。

---

# 具体实现优先级

## Step 1：Skill Registry

先建立：

```python
skill_registry = {
    "MoveTo": MoveToSkill(),
    "DetectObject": DetectObjectSkill(),
    "SearchObject": SearchObjectSkill(),
    "RecoverNavigation": RecoverNavigationSkill()
}
```

---

## Step 2：统一 SkillResult

所有 Skill 都返回同一种结果。

---

## Step 3：MoveToSkill 接 Nav2

这是最重要的第一个 Skill。

---

## Step 4：DetectObjectSkill 接相机/VLM

一开始可以先用 mock detection。

例如先假装检测到：

```json
{"found": true, "object": "red box"}
```

等 Agent 流程跑通后再接真实 YOLO/VLM。

---

## Step 5：Mission Agent 执行 plan

输入结构化 plan，逐步执行 Skill。

---

## Step 6：加入 Recovery

只处理两个失败：

```text
MoveTo failed
SearchObject failed
```

---

# 你应该避免的复杂化

第一版不要做：

```text
多机器人
全自动学习
复杂Memory
VLA端到端控制
强化学习
完整Digital Twin数据库
```

这些可以放后面。

第一版目标应该是：

```text
LLM 能把自然语言任务变成 plan
Agent 能按 plan 调用 Skill
Skill 能驱动 ROS/Nav2/Camera
失败时能做一个简单 Recovery
```

---

# 最终理解

可以这样定义：

```text
L2 Skill Layer:
把 ROS2 / Nav2 / Vision / Manipulation 包装成标准化、可调用、可监控的机器人能力。

L3 Agent Layer:
根据任务目标和环境状态，选择和组合这些 Skill，处理失败，维持任务状态。

LLM:
负责自然语言理解、任务拆解、必要时辅助推理，但不直接控制机器人底层动作。
```

这一套就是你的第一版：

```text
Robot Harness System v1
```
