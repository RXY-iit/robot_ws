接下来我建议改成这一套，先不动代码，等你 review。

**目标**

保持 global path 稳定，不把前向 camera 的临时障碍直接写进 global costmap；让 MPPI 负责连续局部避障，让 Navigation Skill/Agent 在卡住或接近目标失败时做明确的 recovery 决策。

**1. Stuck Timeout 逻辑**

把 timeout 从“总运行时间”改成“无有效进展时间”。

- `max_timeout_sec = 60`
- 机器人正常移动、有进展时 reset stuck timer
- 只有位移/距离目标改善低于阈值时开始累计
- 例如：
  - `progress_reset_distance_m`: 0.05 或 0.08
  - `progress_reset_goal_improve_m`: 0.08
  - `stuck_timeout_sec`: 动态值，但 cap 到 60
- 这样正常绕路不会被误 cancel，真正卡住才触发 recovery。

**2. Recovery 顺序**

每个 navigation goal 进入统一状态机：

1. normal Nav2 goto
2. 如果 half-time/stuck 且距离目标 `< 1.0m`：进入 near-goal local micro-adjust
3. micro-adjust 仍失败：
   - 区域目标：检查 relaxed tolerance 是否已经合理到达
   - 固定点目标：不允许 relaxed，继续 strict 或失败
4. 区域目标仍未到达：
   - 用 local-costmap-aware open-area sampling 重新选 nearby goal
   - 重发 navigation
5. 没有合理候选点：失败，并报告 blocked / no reachable alternate

**3. 固定点 vs 区域目标**

通过 semantic map / approach policy 控制：

- `fixed_point`
  - 例：`front_door`, `back_door`
  - 不允许 relaxed tolerance
  - 不允许 alternate goal
  - 不允许 sofa 那种“附近也算到达”
- `region_area`
  - 例：`sofa`, `table`, `desk area`
  - 允许 relaxed tolerance
  - 允许 local-costmap-aware 重新采样目标点
  - 允许选择同一区域边缘/附近可达点

前面已经把 front/back door 改成固定门中心点，这部分后续 recovery 要读取并遵守。

**4. Local-Costmap-Aware Sampling**

采样只使用机器人“当前可信可见区域”，避免被身后 stale costmap 干扰。

默认可信区域：

- front sector: `+/- 80 deg`
- distance: `0.3m ~ 1.8m`
- frame: robot/base 前方扇区，投影到 map
- 候选点必须满足：
  - local costmap cell free / low cost
  - footprint 周围 clearance 足够
  - 在目标区域附近，或能作为通向目标区域的中间 waypoint
  - 不在 unknown / lethal / inflated 太高区域
  - 优先靠近原始目标，其次靠近 global path，再其次选择更开阔位置

如果目标在身后，先转身让 camera 看向目标方向，再采样。

**5. 行进中实时调整 vs 出问题后调整**

两层分工：

- 行进中实时调整：交给 MPPI
  - 继续用 local costmap
  - 优化 critic 权重，让它敢于轻微横移/斜前移动，不死贴 global path
- 出问题后调整：Navigation Skill recovery
  - stuck / blocked / near goal failure 时，才重新选 waypoint 或区域 goal
  - 不直接让 global planner 因前向 camera 噪声频繁改路

**6. Path Valid / Blocked 判断**

增加一个内部判断：

- 当前 global path 在 robot 前方可信扇区内是否被 local costmap 阻塞
- 如果 path 前方一段连续高 cost，则认为 local blocked
- blocked 后不要立刻 fail：
  - 先让 MPPI 尝试
  - 持续 stuck 后再进入 recovery
- 💡 未来可以参考 Nav2 `IsPathValid` BT 条件，但先在 skill 层做可控实现。

**7. Narrow Path Annotation**

保留现有有效逻辑，但清理旧的强插入逻辑：

- 先计算 global path
- 只有 global path 穿过 annotated narrow passage 时才插入 waypoint
- entry alignment 使用宽松 xy、严格 yaw：
  - xy 宽松，例如 `0.4~0.6m`
  - yaw 严格，例如 `20 deg`
- waypoint / alignment marker 全部发布到 RViz，方便看调试结果

**8. 需要改的主要文件**

预计修改：

- `src/robot_skills/robot_skills/executors/navigation.py`
  - stuck timer
  - near-goal micro-adjust
  - region/fixed recovery gate
  - local-costmap-aware sampling
  - path blocked 判断
  - marker/debug output
- `src/robot_agent/robot_agent/agents/navigation_agent.py`
  - timeout payload 保持 60
  - 传递 semantic recovery policy，如果目前 skill metrics 不够就补字段
- `src/robot_skills/robot_skills/semantic_map.py`
  - resolve target 时带出 `approach_policy` / `recovery_mode`
- `maps/sim_real_lab_001/semantic_map.yaml`
  - 确认 sofa/table 等区域目标允许 alternate goal
  - door 等固定点禁止 alternate/relaxed
- `todo-my/task-sim2real/motion-control-discussion.md`
  - 记录这套策略和测试 checklist

**9. 测试项**

建议你后续测：

- `go to sofa`
  - blocked 后是否选区域附近可达点
- `go to rxy desk`
  - near goal 是否先 micro-adjust，不要直接跳远目标
- `go to front door`
  - 必须 fixed point，不允许 relaxed/alternate
- narrow passage task
  - 只有 path 穿过 passage 才插入 waypoint
- RViz
  - 显示 original goal、annotation waypoint、sampled alternate goal、blocked path segment

这套方案的核心是：**前方可见区域用于短期决策，semantic map 决定 recovery 权限，Nav2 继续负责基础运动。**