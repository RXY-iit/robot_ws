这段内容本质上是在讲：

> **Skill ≠ Prompt**
> 
> **Skill = 可复用、可触发、可评测、可迭代的能力模块（Capability）**

对于你现在在做的 **Isaac Sim + ROS2 + Navigation + Future Agent System** 来说，这个观点非常重要。

---

# 一句话总结

一个工业级 Skill 应该包含：

```text
Trigger
↓
Planning
↓
Tool Use
↓
Validation
↓
Output
↓
Evaluation
```

而不是：

```text
User
↓
巨大Prompt
↓
LLM
↓
结果
```

---

# Skill 的本质

很多人以为：

```python
def goto(location):
    ...
```

或者：

```text
你是导航专家...
请完成导航...
```

就是 Skill。

其实都不是。

真正的 Skill 更像：

```text
Capability
(能力封装)
```

例如：

```text
NavigateToGoal
```

里面包含：

- 什么时候触发

- 怎么规划

- 调什么工具

- 怎么检查成功

- 怎么失败恢复

---

# 7 个步骤总结

## 1. 先定义 Use Case

不要先写 Prompt。

先问：

```text
用户什么时候会需要这个Skill？
```

例如：

### 不好的定义

```text
Navigation Skill
```

太大了。

---

### 好的定义

场景1：

```text
去厨房
```

场景2：

```text
把物品送到会议室
```

场景3：

```text
巡逻工厂A区域
```

此时共性出现：

```text
目标地点
↓
导航
↓
到达确认
```

于是抽象出：

```text
NavigateToGoal
```

Skill

---

# 2. Description 最重要

很多人写：

```text
This skill helps robot navigation.
```

没用。

Agent根本不知道什么时候用。

---

应该写：

```text
Use this skill when:

- User asks robot to move to a known location
- User gives a destination
- Task requires autonomous navigation

Output:
- navigation result
- success/failure
```

相当于：

```text
Agent Router
```

靠 Description 工作。

---

# 3. Tool Boundary

这是工业Agent非常重要的概念。

---

不要：

```text
Navigation Skill

权限：
✓ SSH
✓ 删除文件
✓ 执行命令
✓ 修改代码
✓ 发邮件
✓ 控机器人
```

过大。

---

应该：

```text
Navigation Skill

Allowed:
✓ Nav2 Action

Forbidden:
✗ 修改代码
✗ Git
✗ 文件删除
```

最小权限原则：

```text
Least Privilege
```

---

对于你的机器人：

### Skill

NavigateToGoal

只允许：

```text
Nav2 Action
Localization Query
TF Query
```

---

### Skill

MapBuilding

只允许：

```text
FAST-LIO
GLIM
Map Save
```

---

### Skill

CodeFix

只允许：

```text
Git
ROS Build
Test
```

---

# 4. Progressive Disclosure

不要把所有东西塞进 Prompt。

很多新人：

```text
10000行Prompt
```

Agent还没干活：

```text
Context已经爆了
```

---

应该拆：

## Main

```text
触发条件
核心流程
工具权限
验证方法
```

---

## References

```text
Nav2文档

GLIM文档

FAST-LIO文档
```

---

## Assets

```text
模板

地图格式

yaml示例
```

---

## Scripts

```bash
start_nav.sh

save_map.sh
```

---

Agent需要的时候再加载。

这就是：

```text
Progressive Disclosure
```

渐进式披露。

---

# 5. 模型分层

很多人：

```text
所有事情 GPT-5
```

浪费。

---

应该：

| 任务   | 模型   |
| ---- | ---- |
| 路线规划 | 强模型  |
| 日志分类 | 小模型  |
| 数据提取 | 小模型  |
| 代码生成 | 强模型  |
| 状态检查 | 规则系统 |

---

对于你的机器人：

### L3 Agent

```text
GPT-5
Claude Opus
```

负责：

```text
理解任务
规划任务
决策
```

---

### L2 Skill

```text
小模型
规则系统
```

负责：

```text
执行
检查
```

---

# 6. Trigger Test

很多人测试：

```text
Run NavigateToGoal
```

通过。

没有意义。

---

真实测试：

```text
去会议室
```

---

```text
帮我送水到办公室
```

---

```text
巡逻一下A区
```

---

看看：

```text
Skill会不会自动触发
```

这才是真测试。

---

# 7. Evaluation Loop（最关键）

这是很多 Agent 项目失败的原因。

---

大家停留在：

```text
我觉得不错
```

结束。

---

工业级做法：

准备：

```text
100个测试任务
```

例如：

```text
导航
搬运
巡逻
异常恢复
```

---

跑：

```text
Skill V1
```

---

统计：

```text
成功率

时间

成本

失败原因
```

---

例如：

```text
成功率

V1:
72%

V2:
84%

V3:
91%
```

---

形成：

```text
Evaluate
↓
Improve
↓
Evaluate
↓
Improve
```

闭环。

---

# 套到你的机器人架构

你最近一直在问：

> Skill Layer 和 Agent Layer 为什么不合并？

这篇内容其实给出了答案。

## Agent Layer（L3）

负责：

```text
理解用户

任务分解

选择Skill

任务编排
```

例如：

```text
帮我去会议室找人
```

Agent拆成：

```text
1 导航
2 感知
3 找人
4 对话
```

---

## Skill Layer（L2）

负责：

```text
稳定执行
```

例如：

```text
NavigateToGoal

DetectPerson

PickObject

BuildMap
```

---

Agent：

```text
会思考
会决策
```

Skill：

```text
会执行
会验证
```

---

# 对你未来 Isaac Sim 系统最推荐的结构

```text
User
↓
GPT-5 Agent
(L3)
↓
Task Planner
↓
Skill Manager
(L2)
├─ NavigateToGoal
├─ BuildMap
├─ DetectPerson
├─ PickObject
├─ Docking
├─ Recovery
└─ Explore
↓
ROS2 Actions
↓
Nav2
FAST-LIO
GLIM
MoveIt
↓
Robot
```

这也是目前很多机器人 Agent 系统（包括 NVIDIA Cosmos/GR00T、Google Robotics、OpenAI Robotics 方向、Physical AI 系统）逐渐采用的思路：

> **LLM 不直接控制机器人。**
> 
> **LLM 负责决策，Skill 负责执行。**

这样才能获得可维护、可评测、可扩展的工业级系统。
