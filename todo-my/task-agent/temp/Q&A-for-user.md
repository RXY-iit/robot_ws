你的理解大方向是对的，但有一个关键边界可以再压清楚：

**Skill 是“机器人能力接口”，Agent 是“任务决策与编排”。**

Skill 不一定用 LLM。事实上，**绝大多数 Skill 不应该用 LLM**，应该主要用 Python / ROS2 / C++ / Pydantic / OpenCV / Nav2 action client 这类确定性代码。

比如：

```text
goto_pose
resolve_semantic_target
capture_observation
detect_red_box
cancel_motion
await_robot_auto_mode
move_lift_bounded
```

这些 Skill 的核心应该是：

```text
输入 schema
前置条件检查
调用 ROS action/service/topic
监控执行
超时/失败处理
返回结构化 SkillResult
```

LLM 最多在少数 Skill 里做“辅助解释/视觉确认”，但不负责直接控制机器人。

---

**Agent 不一定每次都调用 Skill。**

Agent 可以只做分析、澄清、计划、总结、风险判断。

例如用户说：

```text
你现在知道哪些地方？
```

Agent 可能只读 semantic map，回答即可，不需要机器人动。

用户说：

```text
去06架子前找红色箱子
```

Agent 才会拆成：

```text
resolve target
await AUTO
goto pose
capture image
detect object
summarize result
```

也就是说 Agent 是“是否调用 Skill、调用哪个 Skill、按什么顺序调用、失败后怎么办”的决策层。

---

**Agent layer 是否接 LLM？**

是的，L3 很适合接 LLM，但不是所有节点都必须接。

比较好的结构是 hybrid：

```text
确定性代码：
  schema validation
  safety policy
  mission state machine
  retry/timeout
  Skill execution

LLM：
  自然语言理解
  多步骤任务拆分
  模糊目标澄清
  任务报告
  非接触视觉解释
```

安全相关的判断不能只靠 LLM。LLM 可以建议，但 policy 和 Skill precheck 才能放行。

---

**L4 要不要一起做？**

我不建议现在正式做 L4，但建议现在预留接口。

L4 更像：

```text
Fleet / Digital Twin / Evaluation / Dataset / Replay / Long-term Memory
```

它负责：

```text
多个机器人
大量任务回放
仿真批量测试
长期性能指标
数字孪生场景管理
```

现在如果同时构建完整 L4，会拖慢主线。但你可以从 L3 日志格式开始，为 L4 留好数据：

```text
mission.json
skill_events.jsonl
robot_status timeline
screenshots
rosbag
metrics.json
```

这样以后 L4 可以直接消费这些数据做统一测试。

---

**为什么 Navigation Agent / Perception Agent 要单独拎出来？**

你觉得它们和 Skill 重合，这个感觉很正常。区别在于：

```text
Navigation Skill = 执行动作
Navigation Agent = 决定导航策略
```

举例：

`goto_pose` Skill 只管：

```text
给我一个 map pose，我调用 Nav2 去那里，返回成功/失败。
```

Navigation Agent 负责：

```text
“去叉车旁边”里的“旁边”是什么意思？
选哪个 approach pose？
目标太近/太窄要不要 cautious？
失败后换另一个 approach pose 还是 ask human？
当前 localization 不健康时要不要拒绝？
```

同理：

```text
detect_known_object Skill = 检测图像里有没有目标
Perception Agent = 决定看哪里、拍几次、用哪个 detector、低置信度怎么问人
```

所以边界是：

```text
Skill: how to execute
Agent: what/why/when/what next
```

你也可以第一版不单独做 Navigation Agent / Perception Agent，而是让 Mission Agent 直接调用 Skill。等 Mission Agent 变复杂后，再拆出来。这可能更适合你现在的阶段。

我建议第一版：

```text
Mission Agent only
  + Navigation helper functions
  + Perception helper functions
```

第二版再升级为：

```text
Mission Agent
Navigation Agent
Perception Agent
Recovery Agent
Memory Agent
```

---

**状态可视化怎么做？**

你需要展示：

```text
任务拆分
当前执行步骤
Skill 调用历史
机器人健康状态
Nav2 状态
感知结果
人工确认请求
日志/错误
```

常见选择有四种。

**1. RViz + ROS topic**

优点：机器人开发最常见，能看 TF、地图、costmap、pose、marker。
缺点：不适合展示 Agent 思考、任务拆分、自然语言状态。

适合底层调试，不适合作为最终 user dashboard。

**2. rqt / Foxglove Studio**

Foxglove 很适合 ROS 系统状态可视化。

优点：ROS topic、TF、image、plot、diagnostics 都好看；实时性强。
缺点：Agent 的任务树/思考过程需要你额外发布成 topic 或 JSON。

如果你想快，Foxglove 是很好的中间方案。

**3. 本地 Python App**

比如 PySide6 / Qt / DearPyGui / Textual。

优点：本地运行，和 ROS2 Python 集成方便。
缺点：UI 设计和实时图像/地图展示会比较麻烦，长期维护不如网页灵活。

适合工程调试工具，不一定适合最终界面。

**4. Web dashboard**

我最推荐这个作为长期方向。

结构：

```text
ROS2 agent_node
  -> publishes mission/status/events
  -> FastAPI/WebSocket bridge
  -> React/Vue/Svelte dashboard
```

页面可以展示：

```text
左侧：任务输入 / 人工确认按钮
中间：任务步骤 timeline / agent plan
右侧：robot health / Nav2 / safety / localization
下方：event log / SkillResult / perception images
```

优点：展示 Agent 状态最自然，远程访问方便，UI 可扩展。
缺点：要多写一个 bridge 和前端。

---

我的建议路线：

```text
现在：
  ROS topics + JSONL logs + 简单 CLI/status output

近期：
  Foxglove 看 robot 状态
  agent 发布 /agent/status /agent/mission_events /agent/plan

下一步：
  做一个本地 web dashboard
  FastAPI + WebSocket + React
```

关键是 Agent 一开始就要发布结构化状态：

```text
/agent/mission_state
/agent/current_step
/agent/skill_events
/agent/health_summary
/agent/human_prompt
```

这样不管你之后用 Foxglove、网页、Qt，本质数据源都统一。

一句话总结：

```text
Skill = 可靠执行能力
Agent = 任务理解、规划、监督、恢复
Dashboard = 把 Agent 的计划和 Robot 的状态透明地显示给人
```

你现在最应该先做的是：

```text
Mission Agent v0
robot_skills v0
结构化 mission event
Foxglove/Web dashboard 数据接口
```

Navigation Agent / Perception Agent 可以先作为 Mission Agent 里的 helper，不急着拆成独立 agent。

1. 和 Claude Code 的 Agent Skill 有什么区别？
概念相似，机制完全不同。

Claude Code 的 Skill（/code-review、/verify 等）本质是：

一段 prompt 模板 + 工具调用配置
LLM 读到这段指令后自己决定怎么做，可以处理开放式、模糊的任务
这个机器人的 Skill 是：

确定性 Python 函数，输入 payload → 输出 SkillResult
没有任何推理，只是结构化的代码调用
二者的共同点是抽象层次上的——都在试图把复杂行为分解成有名字、可复用、有明确输入输出的单元。区别在于单元内部是否有智能：Claude Skill 靠 LLM 执行，机器人 Skill 靠固定代码执行。

2. NavigationAgent.build_plan() 和 Waypoint 是同一类问题吗？
是的，是同一问题的不同抽象层次。

传统 Waypoint：


[(x=1.0, y=2.0, yaw=0.0), (x=3.0, y=5.0, yaw=1.57), ...]
纯几何序列，没有语义，没有条件，Nav2 的 FollowWaypoints action 就是这个。

NavigationAgent.build_plan() 在此基础上加了三层：

语义解析（名字 → 地图坐标，resolve_semantic_target）
安全前置条件（必须是 AUTO 模式）
结构化错误处理（每步有 code，失败有类型）
类比：waypoint 像直接写 SQL，skill plan 像 ORM——底层做的是同一件事，但上层加了验证、语义、错误处理。更大的视角下，skill plan 属于 Task and Motion Planning（TAMP） 里的 task layer，waypoint 是 motion layer。

3. 这种 Skill 模式是新的吗？起源于哪里？
不新，但 LLM 时代让它变得流行。演化路径大致是：

1988 — BDI 模型（Belief-Desire-Intention）

学术 AI 理论：agent 有信念（世界状态）、欲望（目标）、意图（当前计划）。这个机器人的 MissionState + intent + plan 结构就是 BDI 的简化实现。

~2005 — Behavior Tree（行为树）

从游戏 AI（Halo 系列）借鉴到机器人。BT 的叶节点（Action Node）就是这里的 Skill。ROS 里的 BehaviorTree.CPP 就是这个思路。现在这个项目里的 plan 是一个退化的平铺行为树（只有序列节点，没有选择/并行）。

2009 — Option Framework（Konidaris & Barto）

学术机器人里把可复用的子策略叫 "option"，和 skill 是同一概念。

2023 — OpenAI Function Calling / LangChain Tools

LLM API 加入 tool use 后，"agent + tools" 模式爆炸式普及。这里的 SkillCall 结构和 OpenAI tool call 的 JSON schema 几乎是同构的。这个项目的架构就是这个模式的 deterministic 版本——只是把 LLM 换成了正则表达式。

4. Stub 是什么意思？为什么永远返回固定值？
Stub（桩） 是软件工程里的占位实现：接口完整，但内部什么都不做，只返回硬编码的值。

这里的逻辑是：


def capture_observation(self, context):
    return make_result(..., True, codes.OK, "stub completed")
    # 不拍照，不访问摄像头，直接说"成功了"

def detect_known_object(self, context, query, ...):
    return make_result(..., False, codes.NEEDS_HUMAN, "not implemented yet")
    # 不做任何识别，直接说"需要人工"
为什么这样做： 在实际摄像头和模型接入之前，先让整个 plan 流程能跑通——导航、状态机、日志、dashboard 都可以正常测试，只是感知那一步永远走人工确认分支。这是合理的开发策略，但现在的问题是没有任何提示告诉用户"这是 stub 还没实现"，他们会以为是自己的指令有问题。

5. BERT/SentenceTransformer 是什么？和 Llama 的区别？
BERT/SentenceTransformer（Encoder 模型）：

只能做分类/相似度，不能生成文字
极小（100MB~400MB），推理 < 10ms
输入一句话 → 输出"这是 navigate_to_object，概率 0.95"
缺点：task_type 是固定的，加新类型要重新训练，需要标注数据
Llama/Qwen 等（Decoder 生成模型）：

能生成结构化 JSON，能处理没见过的表达
较重（1B 模型约 2GB，8B 约 8GB），推理几百毫秒到数秒
零样本（zero-shot）：不需要训练数据，写好 prompt 就能用
你问的：用本地模型跑大部分简单任务——完全可以。

对于这个项目（intent 只有 ~7 种 task_type），一个 Qwen2.5-1.5B 或 Llama3.2-1B 在 CPU 上约 1-2 秒就能出结果，在 GPU 上 < 0.5 秒。Prompt 大概这样写：


你是机器人命令解析器。把用户输入转成 JSON。
可用 task_type: robot_status / list_known_targets / navigate_to_object /
               inspect_place / search_object_near_place / cancel / ask_human

用户: "去06架子前找红色箱子"
输出: {"task_type":"search_object_near_place","target":"06架子","relation":"front","object_query":"红色箱子"}
这比正则强得多，也比调用云端 API 更适合机器人实时场景。

6. PerceptionAgent.build_plan() 的问题，以及真正的检测方案
当前的根本问题：


set_camera_pose(-20°) → 发 topic（不等到位）
capture_observation  → 返回假 OK（不拍照）
... × 3
detect_known_object  → 永远 NEEDS_HUMAN（stub）
这整条链路在感知上是空的。想检测"有没有红色箱子"，目前是完全无法实现的。

真正的通用检测方案，按难度从低到高：

方案一：YOLO v8/v11（推荐作为第一步）

速度极快（GPU 上可到实时 30fps）
适合仓库场景（可下载预训练模型，支持 80 种常见物体）
红色箱子这类自定义类别需要少量标注数据微调（100~500 张图就够）
输出：bounding box + 类别 + 置信度
接入方式：在 detect_known_object 里调用 ultralytics 库
方案二：Grounding DINO（零样本，更灵活）

用文字描述检测任意物体，不需要训练
输入："red box"、"shelf label 06" → 输出 bounding box
比 YOLO 慢但适合新目标
可和 SAM 结合做精确分割
方案三：CLIP + 分类

对已拍摄图像问"这张图里有没有红色箱子？"
速度快，适合 yes/no 判断，精度不如检测模型
实用建议：

先用 YOLO v8（ultralytics 包，几行代码），处理已知类别；Grounding DINO 作为 fallback 处理新类别。两个都可以本地 GPU 运行，不需要云端。

7. 是否应该加入 LLM 辅助？
应该加，但要选对位置。

当前最大的痛点是 Parser 层，加 LLM 的最高性价比是：


用户输入 → LLM（本地小模型）→ MissionIntent JSON
                                    ↓
                          规则 Plan Builder（保持确定性）
                                    ↓
                          Skill 执行（保持确定性）
LLM 只在解释输入和解释错误两个地方介入，机器人的实际控制回路（导航、相机、lift）全部保持确定性。这是机器人系统里最安全的做法——LLM 的不确定性不会直接传递到硬件动作上。

不推荐的做法： 让 LLM 直接生成 SkillCall 序列。听起来很智能，但 LLM 可能幻觉出一个不存在的 skill 名字，或者生成一个语义上说得通但物理上危险的执行序列（比如先 goto_pose 再解析目标）。

更进一步的结构（如果你有时间）：

可以考虑加一个"Plan Validator"层——LLM 生成 plan 草案，固定规则验证 skill 名称合法性、参数类型、前置条件，不通过则打回给 LLM 重新生成。这样兼顾了灵活性和安全性。