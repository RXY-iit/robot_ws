# L3 Agent Layer 当前设计总结

更新日: 2026-07-11

范围: `/home/ruan-x/robot_ws/src/robot_agent`

这份文档用于后续 review，记录当前 L3 Agent Layer 的职责、agent 划分、parser/mission/recovery/dashboard 逻辑，以及为什么这样设计。

## 1. L3 的定位

L3 是 task executive，不是底盘控制器。

```text
user command
  -> HybridParser
  -> MissionIntent
  -> MissionAgent builds SkillCall plan
  -> SkillClient executes L2 skills
  -> SkillResult / health / route state
  -> retry / recover / ask human / done
```

L3 不做:

- 不直接 publish `/cmd_vel`。
- 不直接调用 Nav2 action。
- 不直接读取 local costmap 来输出底盘速度。
- 不把 LLM 输出直接变成机器人动作。
- 不写死 warehouse 或 lab 的目标列表。

L3 做:

- 将用户命令解析成结构化 intent。
- 基于 intent 生成 SkillCall 序列。
- 根据 SkillResult、health、policy 做恢复或结束判断。
- 将 mission state / events 输出给 dashboard 和 log。
- 保持 agent 逻辑尽量 map-independent。

设计原因:

- parser/LLM 可能不稳定，必须与低层执行隔离。
- map 从 warehouse 切到 lab 时，理想状态是只换 map bundle。
- navigation/perception/health 的错误语义不同，需要 MissionAgent 横向协调。

## 2. 主要文件

核心:

- `src/robot_agent/robot_agent/schemas.py`
- `src/robot_agent/robot_agent/skill_client.py`
- `src/robot_agent/robot_agent/agents/mission_agent.py`
- `src/robot_agent/robot_agent/agents/navigation_agent.py`
- `src/robot_agent/robot_agent/agents/perception_agent.py`

Parser:

- `src/robot_agent/robot_agent/parsers/deterministic_parser.py`
- `src/robot_agent/robot_agent/parsers/embedding_parser.py`
- `src/robot_agent/robot_agent/parsers/llama_cpp_parser.py`
- `src/robot_agent/robot_agent/parsers/hybrid_parser.py`

Nodes / UI:

- `src/robot_agent/robot_agent/nodes/agent_node.py`
- `src/robot_agent/robot_agent/nodes/dashboard_node.py`

Dry-run / review:

- `todo-my/task-agent/dry-run-test/`
- `todo-my/task-agent/dry-run-test/reports/`

## 3. MissionIntent / Task Types

主要 task type:

- `navigate_to_object`
- `navigate_to_place`
- `search_object_near_place`
- `search_object_here`
- `inspect_place`
- `look_direction`
- `move_relative`
- `move_relative_xy`
- `rotate_relative`
- `move_lift_bounded`
- `robot_status`
- `list_known_targets`
- `cancel`
- `ask_human`

设计原则:

- parser 输出少量 typed task。
- MissionAgent 根据 task type deterministic 地生成 SkillCall plan。
- LLM fallback 只能生成受限 schema，不能直接调用任意 skill。

## 4. Parser Architecture

当前结构:

```text
DeterministicParser
  -> EmbeddingParser
  -> LlamaCppParser fallback
  -> ambiguity / semantic gate
```

### 4.1 DeterministicParser

职责:

- 快速处理明确 command。
- 例如 `go to sofa`, `go to tool box`, `turn right 10 degree`, `go ahead 1m`, `look left`。
- 不再使用 warehouse-only target whitelist。

设计理由:

- 常用命令应稳定、可 dry-run。
- target 是否存在应由 semantic map 判断。
- parser 不应该因为地图换了就要改代码。

### 4.2 EmbeddingParser

职责:

- 处理 deterministic parser 难以覆盖的言い換え。
- 通过 template similarity 生成 task_type 和 constraints。

设计理由:

- 纯 regex 容易脆。
- embedding 层比 LLM 更轻，也更容易 dry-run。

### 4.3 LlamaCppParser

职责:

- deterministic / embedding 都失败时 fallback。
- 输出结构化 JSON intent。
- task_type 必须在 allowlist 中。

设计理由:

- 可以利用 local LLM 的语言理解，但不让它直接控制机器人。
- model 替换不应影响 MissionAgent 的 skill planning。

### 4.4 Perception Query 解析原则

当前方针:

- parser 可以拆出 color / object / OCR-like text。
- 是否能检测，由 L2 PerceptionSkills capability matrix 判断。
- `green door` 应被理解为 color + object，而不是 OCR。
- `red bird` 应分别报告 color/object/both。
- known part + unknown part 时，尝试 known part，同时在结果里说明 partial / needs human。

设计理由:

- YOLO、OCR、HSV/color region 的能力边界由模型决定。
- parser 不应该写死所有物体。
- partial evidence 对实际调试很重要。

## 5. MissionAgent

主要文件:

- `src/robot_agent/robot_agent/agents/mission_agent.py`

职责:

- 调用 parser。
- 建立 SkillCall plan。
- 按 step 执行 skill。
- 记录 skill results。
- 处理 health cancel / recovery / resume。
- 处理 navigation timeout / relaxed tolerance / alternate candidate。
- 发布 mission state / mission events。

主要状态:

- `RECEIVED`
- `PARSE`
- `PLAN`
- `EXECUTE_STEP`
- `PAUSED_HEALTH`
- `RECOVERING`
- `RESUMING`
- `NEEDS_HUMAN`
- `FAILED_SKILL`
- `CANCELLED`
- `DONE`

为什么集中在 MissionAgent:

- mission 级决策需要同时看 parser、navigation、perception、health。
- 如果分散到各 agent，dashboard 和 event log 会更难统一。
- 当前阶段优先保证可观测、可恢复，而不是过早拆分。

## 6. Skill Planning Rules

主要规则:

```text
cancel
  -> cancel_motion

robot_status
  -> get_robot_status

list_known_targets
  -> list_known_targets

move_relative / move_relative_xy / rotate_relative
  -> await_robot_auto_mode
  -> check_localization_health
  -> movement skill

navigate_to_place / navigate_to_object
  -> resolve_semantic_target
  -> await_robot_auto_mode
  -> check_localization_health
  -> goto_pose
  -> optional arrival confirmation perception

inspect_place
  -> navigation plan
  -> perception plan

search_object_near_place
  -> navigation plan
  -> perception search plan

search_object_here
  -> perception preflight
  -> perception search plan
```

设计理由:

- navigation 前必须确认 AUTO 和 localization。
- perception 前必须等待 stationary。
- search near place 是 navigation + perception 的 composite task。
- L3 只组装 SkillCall，不实现底层动作。

## 7. NavigationAgent

主要文件:

- `src/robot_agent/robot_agent/agents/navigation_agent.py`

职责:

- 根据 navigation intent 生成 SkillCall plan。
- 包含 semantic resolve、AUTO 等待、localization check、goto_pose。

它不做:

- 不直接插入 narrow passage waypoint。
- 不直接读 local costmap。
- 不直接调用 Nav2。

设计理由:

- NavigationAgent 是 L3 plan builder。
- 实际 route annotation、local-costmap escape、progress watchdog 都在 L2 `goto_pose` 执行时处理。
- 这样 agent code 不绑定具体地图结构。

## 8. L3 Navigation Recovery Policy

MissionAgent 当前处理:

- `FAILED_NAVIGATION` / `FAILED_TIMEOUT` 可以触发 alternate candidate。
- `CANCELLED` 不再当作 navigation failure 直接切换 alternate goal。
- health cancel 后优先 recovery/resume 同一个 step。
- region target 可以 relaxed tolerance / alternate semantic approach。
- fixed point target 不替换 final goal。
- fixed point blocked 时，允许 L2 插入 temporary escape waypoint，再返回原 final goal。

为什么这样分:

- `sofa area` 是区域，到附近即可合理。
- `front_door/back_door` 是固定点，不应该自动换终点。
- health cancel 说明系统健康问题，不说明目标点错。
- obstacle escape waypoint 是路径策略，不是 final goal 替换。

## 9. Health / Recovery

相关文件:

- `src/robot_agent/robot_agent/nodes/agent_node.py`
- `src/robot_agent/robot_agent/agents/mission_agent.py`
- `src/robot_skills/robot_skills/executors/system.py`
- `src/robot_skills/robot_skills/executors/localization.py`

AgentNode 监控:

- `/robot_mode`
- `/safety_status`
- `/gicp_loc/score`
- `/cmd_vel`
- `/nav2/cmd_vel`
- `/gicp_loc/pose`
- `/initialpose`
- `/controller_server/get_state`

Health components:

- `mode`
- `safety`
- `localization`
- `local_planner`

### 9.1 Localization Recovery

触发:

- localization BAD。

行为:

- cancel active motion。
- run `recover_localization_spin`。
- 成功后 resume same mission step。

原因:

- localization bad 不是目标点错误。
- 不应因 localization recovery cancel 而换 alternate goal。

### 9.2 Local Planner Recovery

最新追加:

- AgentNode 非阻塞查询 `/controller_server/get_state`。
- AgentNode 订阅 `/nav2/cmd_vel`。
- 只有 active skill 是 `goto_pose` 时，local planner required。
- dashboard Robot Health 显示 `Local Planner`。

触发:

- 当前执行 `goto_pose`，但 local planner/controller state BAD 或 stale。

行为:

- health watchdog cancel active motion。
- MissionAgent 进入 `PAUSED_HEALTH`。
- 等待 local planner 恢复。
- 恢复后 retry 当前 navigation step。
- 超时仍未恢复则 `NEEDS_HUMAN`。

注意:

- 当前实现负责监控、暂停、等待恢复。
- 如果 Nav2 controller 进程级崩溃，是否自动重启依赖 launch/lifecycle manager。
- 未来如果确认需要 process respawn，再加专门 restart 逻辑。

## 10. PerceptionAgent

主要文件:

- `src/robot_agent/robot_agent/agents/perception_agent.py`

职责:

- 生成 camera pose sequence。
- 生成 observation capture。
- 调用 detect skill。
- 对 search / inspect / look task 组装计划。

当前 plan:

- `look_direction`: set camera pose -> capture observation -> optional detect -> return home。
- `search_object_here`: scan poses -> detect。
- unknown query: 短路到 detect，交给 L2 capability 判断，避免无意义 scan。
- `search_object_near_place`: navigation plan -> perception search plan。

设计理由:

- camera movement / capture / detection 是 perception domain。
- parser 不应该决定 YOLO/OCR/HSV 能力。
- `red bird`, `green cart`, `blue door` 等 composite task 需要由 perception capability 做细粒度判断。

## 11. Runtime Route State

相关 topics:

- `/agent/navigation_route_state`
- `/agent/navigation_progress`
- `/robot_skills/navigation_route_markers`

L3 的角色:

- MissionAgent 不直接管理 route item。
- Dashboard 订阅 route state 并显示 phase。
- mission events 记录 navigation recovery / route 相关事件。

为什么不用 temp waypoint CSV 做 source of truth:

- route 是 mission runtime 结果。
- CSV 作为控制源会和 ROS topic 状态二重管理。
- topic 更适合实时更新、清除、恢复、显示。
- 如果需要 CSV，可由 recorder 从 `/agent/navigation_route_state` 导出。

Dashboard/RViz 目标:

- command 发出后显示 runtime route。
- alignment / waypoint / final goal 状态可见。
- 已通过 waypoint 消失或转为 reached。
- final goal relaxed success 后保留。
- timeout counter 在 Robot Health / navigation progress 区域可见。

## 12. Dashboard Node

主要文件:

- `src/robot_agent/robot_agent/nodes/dashboard_node.py`

订阅:

- `/agent/status`
- `/agent/mission_events`
- `/agent/mission_state`
- `/agent/health_state`
- `/agent/navigation_progress`
- `/agent/navigation_route_state`
- `/agent/human_prompt`
- `/robot_mode`
- `/safety_status`

当前显示:

- command input。
- dynamic examples。
- mission summary。
- parser routing。
- agent block。
- skill execution plan。
- health cards: mode / safety / localization / local planner / nav timeout。
- navigation phase / timeout / route state。
- perception result images。

设计理由:

- UI 上看到的失败必须能对应到 state/events。
- parser、agent、skill、Nav2、perception 的问题要能分层定位。
- real lab 调参需要 dashboard + RViz + JSONL 同时看。

## 13. Dry-run Test

位置:

- `todo-my/task-agent/dry-run-test/`
- `src/robot_agent/test/test_real_lab_parser_semantic.py`
- `src/robot_skills/test/test_perception_composite_planner.py`

目的:

- 不启动 Isaac Sim / Nav2，也能验证 parser 和 plan。
- 确认 real lab target 不被 parser 错判为 `ask_human`。
- 验证 perception composite query 的分解与 capability routing。

最近验证:

- `test_real_lab_parser_semantic.py`
- `test_perception_composite_planner.py`

运行示例:

```bash
cd /home/ruan-x/robot_ws
bash -lc 'source /opt/ros/humble/setup.bash && source /home/ruan-x/robot_ws/install/setup.bash && PYTHONPATH=/home/ruan-x/robot_ws/src/robot_agent:/home/ruan-x/robot_ws/src/robot_skills:$PYTHONPATH python3 -m pytest -q src/robot_agent/test/test_real_lab_parser_semantic.py src/robot_skills/test/test_perception_composite_planner.py'
```

## 14. Map 依赖信息的放置

当前方针:

- Map data 放在 `maps/<map_id>/`。
- Agent package 不保存 room-specific target。
- Agent code 通过 map bundle data-driven 行动。

主要文件:

- `maps/sim_real_lab_001/semantic_map.yaml`
- `maps/sim_real_lab_001/semantic_segments.yaml`
- `maps/sim_real_lab_001/navigation_annotations.yaml`
- `maps/sim_real_lab_001/dashboard_examples.yaml`

这样做的原因:

- warehouse/lab 切换不应该修改 agent code。
- semantic target 是地图事实，不是 agent hardcode。
- navigation annotation 是地图级导航辅助，不是 mission logic。

## 15. 当前已知问题和后续 review 点

Navigation:

- MPPI/local planner 在部分障碍物前仍可能不够早避障。
- runtime route marker 的显示样式还需继续优化。
- narrow path `±5 deg` 可能需要根据测试调整是否过严。
- local-costmap-aware escape/alternate waypoint 仍需更多测试。

Health:

- local planner monitor 已加入，但 process-level restart 还未实现。
- health BAD 取消任务后，dashboard 文案和状态刷新仍需观察是否足够及时。

Parser / Perception:

- composite detection dry-run case 还要继续扩充。
- object-like unknown query 如何进入 embedding/LLM/capability 判断仍需完善。
- capability matrix 要持续跟实际模型同步。

Architecture:

- MissionAgent 逐渐变大，未来可考虑抽出:
  - navigation recovery policy
  - perception planning policy
  - health recovery policy
- 当前阶段优先保证行为可观测、可测试、可恢复。

Review 优先级:

1. MissionAgent 是否仍保持 task executive，不侵入 L2 控制。
2. health cancel / navigation failure / user cancel 是否完全区分。
3. local planner health 是否减少 timeout 假象。
4. fixed point 与 region target policy 是否清楚。
5. runtime route state 是否足够支撑 dashboard/RViz/log。
6. parser 是否仍保持 map-independent。
