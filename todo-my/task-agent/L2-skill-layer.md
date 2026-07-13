# L2 Skill Layer 当前设计总结

更新日: 2026-07-11

范围: `/home/ruan-x/robot_ws/src/robot_skills`

这份文档用于后续 review，记录当前 real lab sim2real 环境下 L2 skill layer 的职责、设计理由、主要逻辑和仍需继续观察的问题。旧版内容已经过时，这里以当前实现为准。

## 1. L2 的定位

L2 Skill Layer 是 L3 Agent 可以安全调用的 typed robot API。

```text
L3 Agent / MissionAgent
  -> SkillCall
  -> L2 Skill executor
  -> ROS2 action / topic / service
  -> Nav2 / safety layer / camera / perception / localization / sim bridge
  -> SkillResult
```

L2 的边界:

- Agent 不直接 publish raw `/cmd_vel`。
- Agent 不直接调用 Nav2 action。
- Agent 只表达 intent 和 SkillCall，低层执行由 Skill 负责。
- Skill 负责参数校验、precheck、timeout、recovery metrics、debug topic。
- 所有结果用 `SkillResult` 返回，L3 根据 `success/code/metrics` 决策。

这样设计的原因:

- 同一套 Agent 要能在 warehouse / real lab / sim / real 间复用。
- 低层失败必须可复盘，不能只靠 dashboard 现场观察。
- parser/LLM 不能直接驱动底盘。
- navigation、perception、health recovery 的失败语义不同，需要统一结果格式但保留细节。

## 2. 主要文件

核心数据:

- `src/robot_skills/robot_skills/schemas.py`
- `src/robot_skills/robot_skills/result_codes.py`
- `src/robot_skills/robot_skills/semantic_map.py`

Skill executors:

- `src/robot_skills/robot_skills/executors/navigation.py`
- `src/robot_skills/robot_skills/executors/system.py`
- `src/robot_skills/robot_skills/executors/localization.py`
- `src/robot_skills/robot_skills/executors/perception.py`
- `src/robot_skills/robot_skills/executors/camera.py`
- `src/robot_skills/robot_skills/executors/lift.py`
- `src/robot_skills/robot_skills/executors/semantic.py`

Map / marker tools:

- `src/robot_skills/robot_skills/nodes/semantic_map_marker_node.py`
- `src/robot_skills/robot_skills/nodes/navigation_annotation_tool.py`
- `src/robot_skills/robot_skills/nodes/segmentation_tool.py`

当前主要 map bundle:

- `maps/sim_real_lab_001/metadata.yaml`
- `maps/sim_real_lab_001/nav2.yaml`
- `maps/sim_real_lab_001/nav2.pgm`
- `maps/sim_real_lab_001/map_3d.ply`
- `maps/sim_real_lab_001/semantic_map.yaml`
- `maps/sim_real_lab_001/semantic_segments.yaml`
- `maps/sim_real_lab_001/navigation_annotations.yaml`
- `maps/sim_real_lab_001/dashboard_examples.yaml`

## 3. Map Bundle 设计

map bundle 是当前系统的重要边界。Nav2 map、3D map、semantic map、segment map、navigation annotation、dashboard examples 都跟随同一个 `map_id`。

目标:

- 不再覆盖同一个 `semantic_map.yaml` 来切换地图。
- warehouse 和 real lab 可以并存。
- 代码通过 `ROBOT_MAP_ID` 加载对应地图数据。
- Agent/Skill 逻辑不写死房间结构。

关键思想:

- `semantic_map.yaml` 是地图基础信息，不属于 agent package。
- `navigation_annotations.yaml` 是导航辅助信息，不属于 segmentation object。
- `dashboard_examples.yaml` 是 UI 示例来源，不应该写死在 dashboard code。

## 4. SkillResult 契约

代表性类型:

```text
Pose2D:
  frame_id
  x
  y
  yaw

SkillContext:
  run_id
  mode
  robot_mode
  max_duration_sec

SkillResult:
  skill_name
  success
  code
  message
  started_at
  ended_at
  metrics
  observations
  artifacts
```

常用 result code:

- `OK`
- `DRY_RUN_OK`
- `FAILED_SCHEMA`
- `FAILED_PRECHECK`
- `FAILED_LOCALIZATION`
- `FAILED_NAV2_INACTIVE`
- `FAILED_SAFETY`
- `FAILED_TIMEOUT`
- `FAILED_NAVIGATION`
- `FAILED_PERCEPTION`
- `FAILED_UNSUPPORTED`
- `NEEDS_HUMAN`
- `CANCELLED`

设计理由:

- L3 不需要 parse 自然语言错误消息。
- dashboard、JSONL、dry-run test、mission recovery 都能读同一份结构。
- `CANCELLED`、`FAILED_TIMEOUT`、`FAILED_NAVIGATION`、`NEEDS_HUMAN` 语义必须分开。

## 5. Semantic Skills

主要文件:

- `src/robot_skills/robot_skills/semantic_map.py`
- `src/robot_skills/robot_skills/executors/semantic.py`

职责:

- 加载当前 `ROBOT_MAP_ID` 的 map bundle。
- resolve `sofa`, `tool_box`, `front_door`, `back_door`, desks 等 semantic target。
- 返回 final pose、approach candidates、approach policy。

当前原则:

- parser 不再用 warehouse target 白名单决定 `go to **` 是否可执行。
- target 是否存在交给 semantic map。
- region target 和 fixed point target 的 recovery policy 不同。

重要区别:

- region target: `sofa`, table area 等，可以 relaxed tolerance，也可以重新选择 alternate approach。
- fixed point target: `front_door`, `back_door` 等，final goal 不应被替换；但可以插入临时避障 waypoint 后再回到原 fixed goal。

## 6. Navigation Skills 总览

主要文件:

- `src/robot_skills/robot_skills/executors/navigation.py`

主要 skill:

- `goto_pose`
- `move_relative`
- `move_relative_xy`
- `rotate_relative`
- `cancel_navigation`

`goto_pose` 不是简单 Nav2 wrapper。当前它承担 real lab 狭窄环境中的执行级导航策略:

```text
validate pose
  -> wait/check Nav2 action server
  -> get current map pose
  -> compute global path
  -> apply map-level navigation annotations if needed
  -> publish runtime route state
  -> run alignment / waypoint / final goal
  -> progress watchdog
  -> near-goal micro adjust
  -> relaxed tolerance if allowed
  -> local-costmap-aware alternate / escape waypoint if allowed
  -> structured SkillResult
```

这样设计的原因:

- real lab 里桌脚、椅子、柜子、临时物品很多，单纯依赖 Nav2 有时会贴边或卡住。
- 机器人是全向移动，但 camera/depth 主要看前方，因此长距离移动更希望车头朝前。
- 人用 joycon 时会自然做斜前、横向、微退、微转，这些需要逐步抽象成 skill-level recovery。
- 这些策略属于执行层，不能放到 parser 或 MissionAgent 里硬写。

## 7. Navigation Annotation

定义文件:

- `maps/sim_real_lab_001/navigation_annotations.yaml`

当前结构:

- `narrow_passages`
- `entry`
- `exit`
- `centerline`
- `alignment_zones`
- `waypoint`
- `trigger_policy`
- `traversal_policy`

作用:

- 如果 global plan 穿过某个 narrow passage，则插入 alignment 和 passage waypoint。
- 如果 plan 不穿过 passage，则跳过，不强行绕路。
- entry / exit 附近标注 open area，用于转身对齐。

最新约束:

- narrow path 通过前必须正对 passage。
- alignment yaw tolerance 运行时强制收紧到 `±5 deg`。
- waypoint yaw tolerance 运行时也强制收紧到 `±5 deg`。
- 即使 robot 已经在入口附近，只要 yaw 不满足，也不能跳过 alignment，会执行短旋转。
- xy tolerance 可以较宽，yaw 必须严格，因为通过狭窄通道时朝向很关键。

为什么不用 segmentation object 表示这些区域:

- segmentation map 表示 user/agent 可引用的语义对象或区域。
- navigation annotation 是功能性参考区域，不应该被 parser 当成可导航对象。
- 例如 alignment zone 是“通过 passage 的辅助区域”，不是用户说 “go to alignment zone” 的目标。

## 8. Runtime Route State / RViz Marker

当前 publish:

- `/agent/navigation_route_state` (`std_msgs/String`, JSON)
- `/agent/navigation_progress` (`std_msgs/String`, JSON)
- `/robot_skills/navigation_route_markers` (`visualization_msgs/MarkerArray`)

`/agent/navigation_route_state` 是 runtime route 的 source of truth。

它表达:

- alignment
- waypoint
- final_goal
- obstacle_escape waypoint
- local-costmap alternate goal
- reached / active / failed / accepted_relaxed / canceled 状态

为什么不用 CSV 作为控制源:

- route 是运行时动态生成的。
- 控制源如果变成 CSV，会和 ROS runtime state 二重管理。
- CSV 更适合作为 recorder/debug mirror。
- dashboard 和 RViz 应该订阅 runtime topic，而不是读静态文件。

RViz marker 目标:

- 所有 runtime route item 都能显示。
- 通过后的非 final waypoint 应清除或变为 reached。
- final goal 如果 relaxed tolerance 成功，应保留并显示 `accepted_relaxed`。
- marker 不应使用过粗线条遮挡 costmap，主要用小箭头和 tolerance circle。

## 9. Progress Watchdog / Timeout

当前原则:

- timeout 不是惩罚“任务时间长”，而是检测“卡住不动”。
- 正常移动或取得 progress 时应 reset stuck counter。
- progress check interval 当前为较短周期，用于更快发现 stuck。

当前恢复阶段:

```text
timeout 1/3:
  near-goal micro adjust if close enough

timeout 2/3:
  relaxed tolerance check if target policy allows

timeout full:
  alternate goal / escape waypoint / fail
```

重要点:

- 微调开始后，如果仍没有超过 reset 条件，计时不会因为“尝试了微调”而清零。
- 三档逻辑互不冲突，后续阶段仍可触发。
- dashboard 通过 `/agent/navigation_progress` 显示 watchdog / stuck / timeout 信息。

## 10. Local Costmap Aware Recovery

当前思路:

- 重点参考 robot 前方可信区域。
- 典型扇区: front `±80 deg`，距离约 `0.3-1.8m`。
- 不过度相信背后未 clear 的 local costmap。
- 在 blocked 或 near-goal timeout 时，可以采样 front open area 作为 alternate/escape waypoint。

使用场景:

- region target 无法到达时，找附近可通行 approach。
- fixed point target 被障碍物挡住时，先插入 escape waypoint，再回到原 fixed goal。
- sofa area 等目标接近但被局部障碍挡住时，重新选择合理 approach。

设计理由:

- camera/depth 主要看前方。
- local costmap 中背后或旧障碍可能误导路径。
- 全向移动能力有用，但不应该长期背对未知区域运动。

仍需观察:

- MPPI 是否足够早地避开障碍。
- local planner 和 skill-level waypoint recovery 的边界是否清晰。
- local costmap 与 USD collider/collision proxy 是否一致。

## 11. Local Planner Health

主要文件:

- `src/robot_skills/robot_skills/executors/system.py`
- `src/robot_agent/robot_agent/nodes/agent_node.py`
- `src/robot_agent/robot_agent/agents/mission_agent.py`

最新能力:

- AgentNode 订阅 `/nav2/cmd_vel`。
- AgentNode 非阻塞查询 `/controller_server/get_state`。
- SystemSkills 在 `/agent/health_state` 中新增 `components.local_planner`。
- Dashboard Robot Health 显示 `Local Planner`。
- 只有当前 active skill 是 `goto_pose` 时，local planner 不可用才判为 BAD。
- idle 或 perception 阶段不会因为 local planner 没有输出速度而误报。

health metrics:

- `required`
- `required_reason`
- `controller_state`
- `controller_state_age_sec`
- `nav2_cmd_age_sec`
- `nav2_cmd_speed`
- `summary`

恢复策略:

- mission 中 local planner BAD 会 cancel active motion。
- MissionAgent 进入 `PAUSED_HEALTH`。
- 等待 controller 恢复，恢复后 retry 当前 navigation step。
- 如果超时仍未恢复，进入 `NEEDS_HUMAN`。

注意:

- 当前实现是监视和暂停等待恢复。
- 如果 `controller_server` 进程级 crash，是否自动 respawn 取决于 launch/lifecycle manager。
- 后续若确认需要进程级重启，再补专门 restart mechanism。

## 12. System / Localization Skills

主要 skill:

- `get_robot_status`
- `check_localization_health`
- `recover_localization_spin`
- `await_robot_auto_mode`
- `await_robot_stationary`

SystemSkills 追踪:

- robot mode
- safety status
- `/cmd_vel`
- `/nav2/cmd_vel`
- odom/localization pose
- GICP score
- stationary drift
- local planner lifecycle state

设计理由:

- navigation 前需要确认 AUTO 和 localization。
- perception 前需要确认 stationary。
- localization BAD 和 local planner BAD 是不同 health failure，恢复策略不同。
- health cancel 不应被误认为“目标点不可达”。

## 13. Perception Skills

主要文件:

- `src/robot_skills/robot_skills/executors/perception.py`
- `src/robot_skills/robot_skills/executors/camera.py`
- `src/robot_skills/config/perception_models.yaml`
- `src/robot_skills/config/perception_targets.yaml`

当前原则:

- parser 只负责把 query 分解为 color/object/text 等约束。
- 能检测什么由 L2 capability matrix 和模型决定。
- `red bird` 这类 composite query 需要分别报告:
  - color found
  - object found
  - both found
- partial known query 也要尝试可检测部分，然后按需要 `NEEDS_HUMAN`。
- 完全 unknown query 不应浪费 9 pose scan。

Dashboard 显示目标:

- 优先显示有 bounding box / match 的结果图。
- 减少重复无效图。
- 图片每行更大，便于检查。

## 14. Relative Motion Skills

目标:

- 支持人类 joycon 类似的短距离微调。
- 用于前进、横移、斜向移动、短角度旋转。
- 不替代长距离 `goto_pose`。

重要原则:

- Agent 不直接发 raw motion。
- Skill 执行 bounded motion。
- 仍需经过 robot mode / health / safety。

当前观察:

- 狭窄空间里斜前 / 横向移动有价值。
- 长距离移动仍应优先朝前，避免 camera 看不到障碍。
- 前轮同步策略曾降低避障能力，暂时回退，后续单独研究底盘控制。

## 15. 当前已知问题

Navigation:

- MPPI/local planner 有时不会足够早避开明显障碍。
- local costmap 与 global path 的关系仍需调优。
- route marker 可视化仍需继续清理粗线/旧 marker。
- narrow path 策略已可用，但入口/出口标注精度会明显影响效果。

Map / USD:

- USD collider / invisible proxy / visual mesh 的边界不一致会影响 costmap 和碰撞。
- 地面物品应尽量有合理 collider。
- 手动移动 USD prim 后要确认 override 保存到工作 layer。

Perception:

- capability matrix 需要持续和实际模型 class list 对齐。
- composite query 的 dry-run case 需要继续扩充。

Review 重点:

1. `goto_pose` 是否承担过多策略。
2. local planner health 和 localization health 是否分离清楚。
3. fixed point / region target recovery policy 是否正确。
4. runtime route state 是否成为 dashboard/RViz/log 的统一来源。
5. narrow path `±5 deg` 是否过严或正好。
6. perception capability 是否还残留 parser-side hardcode。
