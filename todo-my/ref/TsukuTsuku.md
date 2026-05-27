我看了这个报告。结论是：**它的核心仍然是“传统 Navigation 架构”，但工程上做了很多比赛/实环境适配，所以比普通 Nav2 教程式系统更接近“实机可跑系统”。** 

## 1. 和通常 Navigation 思路是否一样？

**大框架是一样的：**

```text
Localization → Global Planner → Local Controller → Obstacle Avoidance → Recovery
```

他们的系统大致是：

```text
3D LiDAR + IMU + Wheel Odom
        ↓
Autoware Localization / NDT + EKF
        ↓
Nav2
  - Global Planner: Smac Hybrid-A*
  - Controller: DWB Controller
  - Costmap: static / obstacle / inflation layer
  - Behavior Tree: planner失败、controller失败、recovery管理
        ↓
YP-Spur / Motor Driver
```

所以从 Navigation 的基本思想看，和你现在理解的 **SLAM / Localization + Nav2 + Costmap + Planner + Controller** 是同一类。

但它和普通 Navigation 不一样的地方在于：
**它不是只让机器人“从 A 到 B”，而是围绕つくばチャレンジ任务，把 Navigation 包装成了一个任务执行系统。**

也就是说，它的上层不是单纯的 Nav2 Goal，而是：

```text
State Manager
  ├─ 通常走行
  ├─ 停止
  ├─ 選択課題A
  ├─ 選択課題B
  └─ 選択課題D1
        ↓
Waypoint Publisher
        ↓
Nav2
```

这一点很值得借鉴。

---

## 2. 最值得借鉴的地方

### ① Localization 和 obstacle detection 分离

他们没有只用一个 LiDAR 解决全部问题，而是分开了：

```text
上部 VLP-16：
  用于 localization / mapping

前方2个 + 后方1个 Livox Mid-360：
  用于近距离障碍物检测

鱼眼相机：
  用于信号识别 / 标签识别

超声波：
  用于玻璃自动门检测
```

这个设计很现实。因为上部 3D LiDAR 视野好，适合定位；但看不到机器人脚下和近距离低矮障碍物，所以他们又加了 Mid-360 做 obstacle detection。报告里也明确说明，把自己位置推定用传感器和障碍物检测用传感器分开的原因，是上部 3D LiDAR 看不到机器人足元障害物。

对你现在的 Mid360 + GLIM / Nav2 系统来说，值得参考的是：

```text
Localization sensor ≠ Obstacle detection sensor
```

也就是说，未来你可以考虑：

```text
Mid360 / 3D LiDAR：
  负责 localization / 3D map

RealSense / 低位 LiDAR / 近距传感器：
  负责脚下障碍物、语义识别、局部安全判断
```

不要强行让一个传感器解决所有问题。

---

### ② 使用 Autoware Localization + Nav2，而不是纯 Nav2

他们的定位不是 AMCL，而是：

```text
3D LiDAR point cloud + PCD map
        ↓
NDT scan matching
        ↓
Gyro Odometer: wheel odom + IMU
        ↓
EKF
        ↓
map → base_link
```

这比普通 2D AMCL 更适合 outdoor / wide area / 3D environment。

对你来说，这一点很重要。你现在在研究：

```text
Mid360 + GLIM 3D map
↓
2D map
↓
AMCL / Nav2
```

但这个报告的思路更像：

```text
3D LiDAR localization
↓
Nav2 navigation
```

也就是：
**定位尽量保持 3D LiDAR / NDT / SLAM 系统，Nav2 只负责路径规划和控制。**

这和你之前问的 “GLIM 自己 localization 后能不能接 Nav2” 是同一方向。报告中的 Autoware Localization 就是一个类似答案：
**可以把 3D localization 的结果作为 Nav2 的 TF source。**

---

### ③ Behavior Tree 的恢复动作设计很实用

他们的 BT 不是复杂 AI，而是很工程化：

```text
Global Path Planning 失败
  → clear global costmap
  → retry

Follow Path 失败
  → clear local costmap
  → retry

多次失败后
  → clear both costmaps
  → wait 3 sec
  → backward 0.6 m
  → retry
```

这很值得借鉴。因为实机 Navigation 里，经常不是算法完全错，而是 costmap 被临时障碍物、点云噪声、动态物体污染。
所以 recovery 行为很重要。

你可以借鉴成自己的 Nav2 BT：

```text
Plan failed:
  clear global costmap

Control failed:
  clear local costmap

Still failed:
  stop
  wait
  backup
  replan

Still failed:
  ask human / AI diagnosis
```

如果你未来加入 AI，这个地方也很好扩展：

```text
传统 BT recovery 失败
        ↓
AI 判断：
  - 是不是地图问题？
  - 是不是障碍物长期堵塞？
  - 是不是 localization 偏移？
  - 是否需要人工介入？
```

---

### ④ 用 map filter 控制“避让 or 等待”，这个非常值得学

报告里最有价值的工程点之一是 **binary filter + laserscan switcher**。

他们不是所有障碍物都避让，而是根据区域决定：

```text
普通区域：
  障碍物进入 costmap
  → Nav2 规划避让路线

特定区域，例如横断歩道前的等待队列：
  给 Nav2 空点云 / dummy scan
  → 不生成避让路线
  → 等待障碍物消失
```

这个思想很关键。

普通 Nav2 的问题是：只要看到障碍物，就会尝试绕开。但现实中有些地方不应该绕，比如：

```text
横断歩道前的人群
狭窄通路
排队区域
自动门前
电梯前
任务等待点
```

这些地方应该是：

```text
不要乱绕
停下来等
```

他们用 binary filter 做了一个区域开关：

```text
区域 A：允许避让
区域 B：禁止避让，只等待
```

这个对你的机器人非常有参考价值。比如你未来可以做：

```text
工厂通路：
  可以绕障碍物

机械设备附近：
  禁止绕行，低速靠近

电梯/门口：
  等待，不绕行

窄通路：
  限速 + 不进行大幅绕行
```

这比单纯调 costmap inflation 更高级。

---

### ⑤ Speed filter 的使用也很值得借鉴

他们根据地图区域限制速度：

```text
危险区域：低速
狭窄区域：低速
横断歩道：快速通过
未探索区域：低速
```

这和你未来的移动机械臂很相关。

普通 Nav2 通常只设置一个最大速度，但实机应该根据场景动态变化：

```text
宽阔走廊：
  normal speed

人多区域：
  low speed

机械臂展开状态：
  very low speed

狭窄空间：
  slow + high safety margin

靠近目标物体：
  precision mode
```

所以你可以借鉴他们的思路，把 navigation mode 分成：

```text
normal navigation
safe navigation
precision approach
waiting mode
recovery mode
```

---

### ⑥ TF Switcher：切换定位源的做法非常值得参考

在選択課題A里，他们进入市役所内部，无法使用事前 3D map localization，于是临时启动 LIO-SAM 做实时 SLAM。

这里最有价值的是：
他们没有让整个 Nav2 系统大改，而是通过 **TF Switcher** 切换 localization source。

普通走行时：

```text
Autoware ekf_localizer
map → base_link
```

課題A时：

```text
LIO-SAM
slam_odom → slam_base_link
```

然后通过一个 switched frame 给 Nav2 使用：

```text
map → switched_base_link
```

也就是说，Nav2 不关心后面到底是 Autoware 还是 LIO-SAM，只看统一的 TF。

这个思想非常适合你未来系统：

```text
通常：
  GLIM localization

局部未知环境：
  Real-time SLAM

失效时：
  wheel odom + IMU fallback

视觉定位成功时：
  visual relocalization correction
```

上层 Nav2 / AI 不应该直接绑定某一个定位算法，而应该通过统一 TF 或 localization interface 获取位姿。

---

### ⑦ State Manager 比直接发 waypoint 更工程化

普通 Nav2 教程通常是：

```text
send goal
robot goes there
```

但他们是：

```text
Waypoint Publisher
        ↓
Nav2
        ↓
到达后反馈
        ↓
State Manager 判断下一状态
```

这非常适合比赛任务，也适合你未来的移动机械臂任务。

例如你的系统可以变成：

```text
State Manager
  ├─ idle
  ├─ navigate_to_machine
  ├─ recognize_target
  ├─ approach_precisely
  ├─ manipulate
  ├─ verify_result
  ├─ recover
  └─ ask_human
```

这样 AI 不需要直接控制电机，而是控制状态和技能：

```text
AI → choose next skill/state
State Manager → execute safe robotic behavior
```

这和你之前提到的 **Skill API / AI robot nervous system** 很一致。

---

## 3. 它的问题也很值得反面借鉴

报告里也暴露了几个典型实机问题。

### ① DWB Controller 在“等待但不避让”时会前后抖动

他们提到，在障碍物前等待、不进行避让时，机器人会出现前后“煽る”一样的动作。这个问题你也很可能遇到。

原因大概是：

```text
local controller 仍然想追踪 path
但前方被 obstacle / cost 限制
于是速度评价在前进、停止、后退之间反复变化
```

解决方向：

```text
等待模式下不要让 DWB 继续正常控制
进入 explicit stop state
或者切换 controller
或者设置 zero velocity hold
或者让 BT/State Manager 接管等待
```

他们最后也说，未来需要根据情况切换 Planner / Controller。

这点对你很重要：
**不要期待一个 controller 解决所有场景。**

---

### ② 狭窄区域 Global Path 计算慢 / 生成不可通过路径

他们在課題A中失败的主要原因之一是：
动态障碍物回避过程中，规划出了一条机器人无法通过的狭窄路径。

这说明即使 costmap / planner 正常，也可能出现：

```text
地图上看起来能过
实际车体过不去

局部 planner 觉得可以尝试
但机器人卡住

global path 合理
但 execution 不合理
```

你的机器人如果加机械臂，问题会更明显。因为 footprint 不是固定的：

```text
机械臂收起：footprint 小
机械臂伸出：实际占用空间大
携带物体：更大
```

所以未来你应该考虑：

```text
dynamic footprint
task-based safety margin
narrow passage detection
path feasibility check
```

---

### ③ 定位失效有时候不可复现

他们本走行中在停车场入口附近发生自己位置破綻，但实验中没有出现，之后也难以复现。

这是 outdoor robot 的典型问题：

```text
光照
人流
车辆
点云结构
反射
地图变化
初始化误差
偶发 TF / time delay
```

所以系统需要：

```text
localization confidence monitoring
自动重定位
失败检测
人工恢复点
日志记录
rosbag replay
```

对你来说，这说明只做“能跑一次”不够，必须设计诊断机制。

---

## 4. 对你现在系统最直接的借鉴建议

你的当前方向是：

```text
Mid360 / GLIM / 3D map
↓
localization
↓
Nav2
↓
未来加入 RealSense / AI / 移动机械臂
```

可以从这个报告里直接借鉴下面这些设计：

| 借鉴点                    | 对你的意义                                             |
| ---------------------- | ------------------------------------------------- |
| 3D localization + Nav2 | 不一定要坚持 AMCL，可以让 GLIM/3D localization 输出 TF 给 Nav2 |
| State Manager          | 把 Navigation 从“去某点”升级成“任务状态机”                     |
| Waypoint Publisher     | 不直接一次性发所有目标，而是按任务阶段发 waypoint                     |
| Binary filter          | 某些区域避让，某些区域等待                                     |
| Speed filter           | 按地图区域/任务状态限制速度                                    |
| Collision Monitor      | 在 controller 后面再加安全刹车层                            |
| TF Switcher            | 未来可以在 GLIM / SLAM / visual localization 之间切换      |
| sensor separation      | 定位传感器和障碍物/语义传感器分工                                 |
| recovery BT            | costmap clear、wait、backup、replan 是实机必须品           |

---

## 5. 和你的 AI Navigation 方向的关系

这个系统本身不是 AI Navigation，而是 **规则式任务状态机 + 传统 Navigation**。

但它很适合作为 AI 系统的底座。

你未来可以这样升级：

```text
LLM / VLM / Memory
        ↓
Task Planner
        ↓
State Manager
        ↓
Skill API
  - goto()
  - wait_until_clear()
  - detect_signal()
  - approach_object()
  - relocalize()
  - ask_human()
        ↓
Nav2 / GLIM / Controller
```

也就是说，AI 不应该直接替代 Nav2。更合理的是：

```text
Nav2 负责低层安全移动
AI 负责高层判断、模式切换、异常解释、任务规划
```

例如：

```text
AI 不直接算 local trajectory
AI 判断：
  “这里是横断歩道前，应该等待，不应该绕行”
  “这里是狭窄区域，切换低速模式”
  “定位置信度下降，暂停并请求重新定位”
  “目标箱子检测到了，在它前方生成 waypoint”
```

这和报告里的 State Manager / binary filter / speed filter / TF switcher 很契合。

---

## 总结

这个 robot system 的 Navigation 主体是传统架构：

```text
3D localization + Nav2 + costmap + BT + controller
```

但值得借鉴的是它的 **实机工程化设计**：

```text
状态机管理任务
不同区域切换避让策略
不同区域限速
定位源可切换
传感器分工明确
BT recovery 机制完整
感知结果动态生成 waypoint
```

对你来说，最应该学的不是某一个具体算法，而是它的系统思想：

**把 Nav2 当成底层移动能力，把 State Manager / Skill API / AI 放在上层，负责决定什么时候导航、什么时候等待、什么时候切换定位、什么时候进入任务模式。**
