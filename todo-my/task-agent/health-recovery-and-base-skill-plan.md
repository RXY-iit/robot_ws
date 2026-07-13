# Health / Recovery Agent 与基础 Skill 完善计划

> Date: 2026-06-30  
> Workspace: `/home/ruan-x/robot_ws`  
> 状态: Phase 1/2 完成；Phase 3 初版已落地；Phase 4 仍是设计方案  
> 目标: 补齐机械臂之前的移动、观察、健康监控、恢复能力，让上层 Agent 更稳定地调用底层能力。

## 1. 当前问题总结

现在系统已经有 L2 Skill 和 L3 Agent 的基本骨架：

```text
User Command
  -> Parser
  -> Mission Agent
  -> Navigation / Perception / Localization Agent
  -> typed Skill
  -> ROS2 / Nav2 / camera / detector
```

实际测试曾暴露以下底层能力缺口：

```text
1. move_relative 只支持明确方向 + 明确距离，不能处理模糊移动。已补第一版。
2. look left / 左を見て 这类指令不应该走语义目标导航，只需要动相机。已补第一版。
3. 现在 heartbeat 主要是记录和展示，不会主动介入 mission。已补 health_state 展示，并在 BAD 时请求 cancel 当前 motion。
4. Localization check 只在导航前或导航失败后触发，不是持续 watchdog。已补 mission step 边界 recovery；连续异步 watchdog 仍需实测完善。
5. Dashboard 的健康状态还不够像 operator 面板，细节太 raw。已补 Robot Health 第一版。
```

这几个问题本质上不是 LLM 能不能解析，而是底层 Skill ontology 还不够清晰。

## 2. Skill 分类原则

同质化 Skill 应该归类到一起，便于上层 Agent 理解和组合。

建议 L2 Skill 分为以下几组。

### 2.1 Health / Safety Skills

不直接移动机器人，只负责状态读取、健康判断、安全门控。

```text
get_robot_status
get_health_summary
check_safety_status
check_localization_health
check_nav2_health
check_sensor_health
check_path_clear
```

特点：

```text
read-only 或 safety-gate
可被 heartbeat/watchdog 定期调用
可被 mission preflight 调用
结果应该是结构化 health level，而不是只靠 message
```

建议统一输出：

```yaml
health_level: OK | WARN | BAD | UNKNOWN
component: localization | safety | nav2 | sensor | battery | perception
summary: short human-readable text
details:
  ...
recommended_action: continue | pause | recover | ask_human | abort
```

### 2.2 Base Motion Skills

负责底盘移动。上层不能直接发布 `/cmd_vel`。

现有：

```text
goto_pose
move_relative
cancel_motion
```

建议补充：

```text
move_relative_xy
rotate_relative
turn_to_direction
move_until_blocked_or_clear
stop_base
```

语义区别：

```text
goto_pose:
  已知 map frame 目标位姿，交给 Nav2。

move_relative:
  当前 base frame 下的前/后/左/右位移，内部转换成 map frame goal，再交给 Nav2。

move_relative_xy:
  当前 base frame 下的 dx/dy 组合位移，例如左前方。

rotate_relative:
  原地相对旋转，例如 turn around / 左を向いて。

move_until_blocked_or_clear:
  最大移动距离内前进，但遇到近距离障碍就停止。
```

当前 `move_relative` 已经不是直接输出运动指令，它是：

```text
read map->base_footprint TF
compute target map pose
call goto_pose / Nav2 NavigateToPose
```

所以基础避障目前依赖：

```text
Nav2 costmap
local planner / controller
LiDAR / obstacle layer
```

它不是 skill 自己检测“眼前是否有障碍”。如果障碍物没有进入 costmap，skill 不会知道。

### 2.3 View / Camera Skills

负责“看哪里”，不移动底盘。

现有：

```text
set_camera_pose
capture_observation
detect_known_object
```

建议补充：

```text
look_direction
scan_direction
capture_directional_observation
scan_3x3
return_camera_navigation_pose
```

语义区别：

```text
look_direction:
  单次方向观察，只动相机，不移动底盘。

scan_direction:
  对某个方向做上/中/下扫描，例如左侧三张图。

scan_3x3:
  当前已有的上中下 x 左中右九宫格扫描。

return_camera_navigation_pose:
  回到 navigation mode 的相机姿态。
```

例子：

```text
look left
左を見て
右上を見て
前方を確認
```

应该生成：

```text
set_camera_pose(...)
capture_observation(...)
optionally detect_known_object(...)
return_camera_navigation_pose
```

不应该生成：

```text
resolve_semantic_target("左")
goto_pose(...)
```

### 2.4 Perception / Detection Skills

负责“看到了什么”。

现有：

```text
detect_known_object
```

建议进一步拆分 detector intent：

```text
detect_object_class
detect_text_label
detect_shelf_number
detect_color_region
summarize_observation
```

原因：

```text
YOLO 适合 object class
OCR 适合 shelf number / sign / label
color rule 适合 red box 初期验证
VLM/API 未来只适合解释和报告，不应该直接控制机器人
```

### 2.5 Manipulation-Lite Skills

本阶段先跳过。

原因：

```text
1. 未来加入机械臂后，Manipulation-Lite 与 Arm/Gripper Skill 需要一起重新定义。
2. 当前相机 pan/tilt 更适合归入 View / Camera Skills，而不是作为 manipulation 能力暴露给上层。
3. 当前 lift 只保留现有 move_lift_bounded，不在本轮扩展。
```

后续机械臂阶段再补：

```text
arm_precheck
move_arm_joint_safe
move_arm_cartesian_safe
open_gripper / close_gripper
lift / camera / arm 的互锁策略
contact action human confirmation
```

## 3. 模糊移动指令的参数化方案

现在 `move_relative` 可以解析明确距离：

```text
move forward 1m
左へ50センチ
後退30センチ
```

但以下还不稳定：

```text
前進少し
左前方へ少し
turn around
左を向いて
```

建议统一加入 `motion_phrase_policy`。

### 3.1 距离模糊词

```yaml
distance_aliases:
  little:
    jp: ["少し", "ちょっと"]
    en: ["a little", "slightly"]
    distance_m: 0.20
  normal:
    jp: ["少し前へ", "前へ"]
    en: ["forward", "move forward"]
    distance_m: 0.50
  large:
    jp: ["大きく"]
    en: ["a lot"]
    distance_m: 1.00
```

建议默认值：

```text
少し / a little -> 0.20m
normal without distance -> 0.50m
大きく / a lot -> 1.00m
```

实际距离要保守，因为这是移动机器人，不是 lift。

### 3.2 方向词

```yaml
directions:
  forward:
    dx: 1.0
    dy: 0.0
    aliases: ["forward", "前進", "前へ"]
  back:
    dx: -1.0
    dy: 0.0
    aliases: ["back", "backward", "後退", "後ろへ"]
  left:
    dx: 0.0
    dy: 1.0
    aliases: ["left", "左へ"]
  right:
    dx: 0.0
    dy: -1.0
    aliases: ["right", "右へ"]
  front_left:
    dx: 0.707
    dy: 0.707
    aliases: ["front left", "左前方"]
  front_right:
    dx: 0.707
    dy: -0.707
    aliases: ["front right", "右前方"]
```

### 3.3 旋转词

```yaml
turn_aliases:
  turn_left:
    yaw_deg: 90
    aliases: ["turn left", "左を向いて"]
  turn_right:
    yaw_deg: -90
    aliases: ["turn right", "右を向いて"]
  turn_around:
    yaw_deg: 180
    aliases: ["turn around", "振り向いて"]
```

建议新增 task type：

```text
move_relative_xy
rotate_relative
look_direction
```

不要把所有模糊动作都塞进 `move_relative`，否则 Skill 语义会变混。

当前已落地的第一版：

```text
前進少し -> move_relative(direction=forward, distance_m=0.2)
左前方へ少し -> move_relative_xy(dx_m=0.141, dy_m=0.141)
右前方へ50センチ -> move_relative_xy(dx_m=0.354, dy_m=-0.354)
turn around -> rotate_relative(yaw_deg=180)
```

## 4. 观察指令方案

### 4.1 当前问题

`look left` / `左を見て` 本质是观察，不是导航。

当前系统可能会：

```text
左を見て -> inspect_place(target="左") -> resolve_semantic_target("左") -> NEEDS_HUMAN
```

这是错误的任务归类。

当前已落地的第一版：

```text
左を見て -> look_direction(direction=left, band=middle)
look left -> look_direction(direction=left, band=middle)
```

### 4.2 建议设计

新增 intent：

```text
look_direction
```

Payload：

```yaml
direction: front | left | right | up | down | front_left | front_right
band: ground | middle | upper | all
object_query: optional
```

Skill plan：

```text
look_direction without object:
  set_camera_pose(direction, band)
  capture_observation
  return_camera_navigation_pose

look_direction with object:
  set_camera_pose(direction, band)
  capture_observation
  detect_known_object(query)
  return_camera_navigation_pose
```

例子：

```text
左を見て
look left
右上を見て
前方に車がありますか
```

解析结果：

```json
{
  "task_type": "look_direction",
  "target": null,
  "relation": null,
  "object_query": null,
  "constraints": {
    "direction": "left",
    "band": "middle"
  }
}
```

### 4.3 与 9 点扫描的关系

当前 9 点扫描适合：

```text
检查架子号码
搜索附近是否有目标
需要最大覆盖率的观察
```

方向观察适合：

```text
看看左边
看看右上
前方有没有车
```

所以两者都需要，但不是同一个 Skill。

## 5. Heartbeat 与 Localization Agent 的重新分配

### 5.1 当前状态

当前 `AgentNode` 有 2 秒 heartbeat：

```text
robot_mode
safety_status
localization_warning
stationary_drift_m
gicp_score
```

但它主要是：

```text
publish event
dashboard display
log record
```

它不会主动：

```text
cancel navigation
pause mission
run recovery
ask human
```

当前 localization recovery 只在以下情况触发：

```text
check_localization_health 返回 FAILED_LOCALIZATION
goto_pose 返回 FAILED_NAVIGATION / FAILED_TIMEOUT
```

这意味着移动过程中 localization 变坏时，系统可能发现得太晚。

### 5.2 建议: Health / Recovery Agent

建议把 heartbeat 和 localization agent 统合成一个更高层的：

```text
Health / Recovery Agent
```

它包含两个子职责：

```text
Health Monitor:
  定期检查基础健康状态。

Recovery Manager:
  当健康状态 BAD 时，决定 pause / stop / recover / ask human。
```

不要让 Localization Agent 单独只管 localization，因为实际稳定运行需要同时看：

```text
safety
robot mode
localization
Nav2
sensors
camera
perception model readiness
mission execution state
```

### 5.3 推荐职责分配

```text
Mission Agent:
  负责用户任务、计划、执行、暂停/继续。

Navigation Agent:
  负责目标解析、路径计划、goto/move_relative。

Perception Agent:
  负责观察、扫描、检测、视觉确认。

Health / Recovery Agent:
  持续监控 robot health。
  在 mission 运行中也工作。
  发现 BAD 时通知 Mission Agent pause。
  必要时 cancel motion。
  执行 localization recovery。
  恢复后允许 Mission Agent resume。
```

### 5.4 Health Loop

建议独立 timer，例如 2Hz 或 1Hz：

```text
every 0.5s or 1.0s:
  read robot_mode
  read safety_status
  read cmd_vel age
  read odom / TF health
  read GICP score + age
  read localization drift
  read Nav2 lifecycle / action server
  read sensor freshness
  publish /agent/health_state
```

健康等级：

```text
OK:
  可以继续执行。

WARN:
  可以继续，但 dashboard 显示黄色。
  下一步 skill 前应重新 precheck。

BAD:
  应暂停 mission。
  如果正在导航，cancel Nav2 goal。
  尝试 recovery 或请求 human。

UNKNOWN:
  关键数据缺失。
  real mode 下应按 BAD 处理，sim mode 也按照 WARN 处理。
```

### 5.5 Recovery Policy

Localization BAD 时：

```text
1. publish health_state=BAD
2. Mission Agent state -> PAUSED_HEALTH
3. cancel_motion
4. stop_base
5. recover_localization_spin
6. check_localization_health
7. OK -> resume mission from last safe step
8. timeout -> NEEDS_HUMAN
```

Safety BAD 时：

```text
1. cancel_motion
2. stop_base
3. Mission Agent -> NEEDS_HUMAN
4. 不自动恢复
```

Sensor BAD 时：

```text
camera unavailable:
  perception task -> NEEDS_HUMAN or retry

LiDAR unavailable:
  navigation task -> pause/abort

GICP score stale:
  localization health -> WARN/BAD depending on age
```

## 6. Dashboard 健康状态设计

Dashboard 应该有一个独立区域：

```text
Robot Health
```

它不应该展示太多 raw JSON。用户只需要第一眼知道能不能继续。

### 6.1 推荐布局

```text
Robot Health
------------------------------------------------
Overall        OK / WARN / BAD / UNKNOWN
Safety         OK
Mode           AUTO
Localization   OK
Navigation     READY
Sensors         OK
Perception      READY
Battery         UNKNOWN
------------------------------------------------
Current action  Moving to shelf_06
Recommendation  Continue
```

颜色：

```text
OK: green
WARN: yellow
BAD: red
UNKNOWN: gray
```

### 6.2 展示层次

默认只显示简洁结果：

```text
Localization: OK
```

有问题时展开细节：

```text
Localization: BAD
Reason: GICP_SCORE_BAD
GICP score: 0.62
Stationary drift: 0.12m
Last trusted pose: 8.4s ago
Action: Recovery required
```

### 6.3 Mission 与 Health 的关系显示

当 Health Agent 介入时，Dashboard 应明确显示：

```text
Mission: PAUSED_HEALTH
Health / Recovery Agent: running recover_localization_spin
Navigation Agent: canceled
Next action: resume goto_pose after localization OK
```

这比单纯显示一堆 `mission_events` 更容易理解。

## 7. 推荐新增 Topic / State

### `/agent/health_state`

```json
{
  "overall": "OK",
  "components": {
    "safety": {"level": "OK", "summary": "Safety status OK"},
    "localization": {"level": "WARN", "summary": "GICP score stale"},
    "navigation": {"level": "OK", "summary": "Nav2 active"},
    "sensors": {"level": "OK", "summary": "Camera and lidar fresh"},
    "perception": {"level": "OK", "summary": "YOLO model loaded"}
  },
  "recommended_action": "continue",
  "active_recovery": null,
  "timestamp": "..."
}
```

### `/agent/recovery_events`

```json
{
  "type": "recovery_started",
  "reason": "FAILED_LOCALIZATION",
  "action": "recover_localization_spin",
  "mission_id": "..."
}
```

### MissionState 增加状态

```text
PAUSED_HEALTH
RECOVERING
RESUMING
```

## 8. Parser / Intent 更新建议

新增 task types：

```text
look_direction
move_relative_xy
rotate_relative
health_status
```

扩展 constraints：

```json
{
  "direction": "left",
  "distance_m": 0.2,
  "dx_m": 0.5,
  "dy_m": 0.5,
  "yaw_deg": 90,
  "band": "middle"
}
```

DeterministicParser 应该优先处理：

```text
look left
左を見て
前進少し
左前方へ少し
turn around
左を向いて
health
ロボット状態
```

Embedding / llama.cpp fallback 只处理更复杂表达。

## 9. 实施顺序建议

### Phase 1: Skill taxonomy 与 parser 修正

```text
1. 在 skill_registry.yaml 中按 group 标记 Skills。DONE
2. 新增 look_direction task type。DONE
3. 新增 rotate_relative / move_relative_xy dry-run。DONE
4. 给 move_relative 增加模糊距离解析。DONE
5. 修正 左を見て / look left 不再走 semantic target。DONE
```

### Phase 2: Health state 独立发布

```text
1. 新增 HealthState schema。DONE
2. AgentNode heartbeat 改成 health_state publisher。DONE, 保留旧 heartbeat event
3. Dashboard 新增 Robot Health 区域。DONE
4. health_state 保持只展示，不主动干预。DONE for Phase 2；Phase 3 已增加 BAD 时介入
```

### Phase 3: Health / Recovery Agent 介入 mission

```text
1. MissionAgent 支持 PAUSED_HEALTH。DONE
2. Health Agent BAD 时 cancel motion。DONE, heartbeat BAD 时请求 cancel 当前 motion
3. Localization BAD 时自动 recover_localization_spin。DONE, mission step 边界触发
4. recovery OK 后 resume mission。DONE, 返回 RESUMING 后继续当前 step
5. recovery timeout 后 NEEDS_HUMAN。DONE
```

Phase 3 当前限制：

```text
1. 当前是初版 Health / Recovery Agent 介入。
2. mission step 边界可以 pause/recover/resume。
3. heartbeat 检测 BAD 时会请求 cancel 当前 motion。
4. 如果 Nav2 action 正在长时间阻塞，取消后的 resume 行为需要在 sim 中继续实测。
5. 还没有完整的异步 PAUSED_HEALTH 状态机队列；这是下一轮稳定化工作。
```

### Phase 4: Path clear / obstacle-aware relative motion

```text
1. 新增 check_path_clear。
2. move_relative 前可选 path precheck。
3. move_until_blocked_or_clear 使用 costmap / scan 判断。
4. Dashboard 显示 path blocked reason。
```

## 10. 当前系统是否覆盖机械臂以外的基础能力

结论：还没有完全覆盖。

已覆盖：

```text
状态读取
AUTO gate
语义目标导航
相对平移基础版
取消
相机九宫格扫描
图像捕获
YOLO/OCR/rule 检测
lift bounded motion
localization check/recovery 初版
dashboard 初版
```

未覆盖或不足：

```text
相对旋转
斜向移动
模糊距离移动
方向观察
路径是否可通行
障碍物前停止
持续 health watchdog
mission pause/resume
健康状态 dashboard
Nav2 / sensor freshness health
perception model readiness health
```

机械臂之前，建议优先补齐：

```text
1. Health / Recovery Agent
2. look_direction
3. rotate_relative
4. move_relative_xy
5. check_path_clear
6. Dashboard Robot Health panel
```

这几项会显著提高系统的可测试性、可解释性和长时间运行稳定性。
