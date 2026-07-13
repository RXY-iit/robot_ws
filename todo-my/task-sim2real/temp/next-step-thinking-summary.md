# Real Lab Sim2Real Next Step Thinking Summary

目的：整理 real lab sim2real 当前状态、已完成修改、剩余问题和下一步方向。  
原则：优先长期维护和系统稳定性，不追求单纯快速跑通。

## 1. 当前已完成

### Real Lab USD / Assets

- 已创建 `robot_sim_real_lab.usd`。
- 已加入真实 lab 的基础房间结构、桌椅、书架、沙发、柜子、工具车、杂物、灯、门窗等。
- 外部 Omniverse / SimReady assets 已 collect 到本地：
  - `models/omniverse_assets/Collected_robot_sim_real_lab/`
- 工作 USD 已改为引用本地 collected assets。

### Mapping / Localization / Navigation

- 已创建 real lab GLIM dump、3D point cloud、2D Nav2 map。
- 已整理 real lab map bundle 初版：
  - `maps/sim_real_lab_001/nav2.yaml`
  - `maps/sim_real_lab_001/nav2.pgm`
  - `maps/sim_real_lab_001/map_3d.ply`
  - `maps/sim_real_lab_001/glim_dump/`
  - `maps/sim_real_lab_001/semantic_map.yaml`
  - `maps/sim_real_lab_001/semantic_segments.yaml`
  - `maps/sim_real_lab_001/metadata.yaml`
  - `maps/sim_real_lab_001/dashboard_examples.yaml`
- 使用该 map 测试 localization / navigation 基础管线，基础设施可运行。

### Semantic / Segment Map

- 已创建 real lab semantic map / segment map 初版。
- 已用 `segmentation_tool` 调整 real lab 区域。
- 已支持 runtime map selection：
  - `ROBOT_MAP_ID=sim_real_lab_001`
  - `ROBOT_MAP_ID=warehouse`
  - `ROBOT_SEMANTIC_MAP=/path/to/map.yaml`
- 已恢复/保留 warehouse 独立 semantic fixture：
  - `src/robot_skills/config/warehouse_semantic_map.yaml`

### Parser / Dashboard

- `go to <target>` / `inspect <target>` 不再被 warehouse shelf/door/corridor 白名单限制。
- real lab target 可以进入 semantic resolver：
  - `go to sofa`
  - `go to meeting table`
  - `go to rxy table`
  - `desk_rxy に行って`
- Dashboard quick commands 已从当前 semantic map 动态生成。
- Dashboard 默认 command 也会随当前 map 改变。
- agent-side mission events 已写入：
  - `debug-output/agent/.../skill_events.jsonl`

### Agent / Skill Navigation

- semantic navigation 已加入 task tolerance：
  - `xy_goal_tolerance_m: 0.25`
  - `yaw_goal_tolerance_rad: 0.45`
- dynamic timeout 已改为按距离缩短，近距离任务不会继续硬等 90/120s。
- semantic target region 已能生成多个 approach candidates。
- MissionAgent 会根据最近 localization pose 从 candidates 中选择更近的 approach pose。

### Detection / Perception

- `green_door`、`blue_cart` 等复合 perception query 已拆成 primitive plan：
  - color step
  - object-like step
  - composite overlap step
- `cart` 不再因为 substring 被误识别成 `car`。
- 除非 query 明确包含 text / label / number / letter，不再为普通 color+object 复合任务触发 OCR。
- 未预定义但像物体的词，例如 `door` / `cart`，可以作为 open object-like target 进入 detector。
- 不像物体的普通词，例如 `love`，不会强行进入 detector，会 ask human / unsupported。
- perception result metrics 增加 primitive summary：
  - `color_found`
  - `object_found`
  - `composite_found`
  - `matched_colors`
  - `matched_objects`
- 非 object-like query，例如 `love`，保持 unsupported / ask human。

## 2. Log 结论

### 已确认

- `list_known_targets` 能读取 real lab semantic targets。
- `go to sofa`、`go to meeting table`、`go to RXY`、`go to tool box` 的 parser 和 semantic resolver 都成功。
- 失败主要发生在 Nav2/controller 或目标 approach 可达性层。
- `find green door` / `find blue cart` 的旧问题主要来自 perception planner：
  - door 不在 object registry 时被当作 unsupported color-only。
  - cart 被 substring 误匹配成 car。
  - OCR 参与了不该参与的 ordinary object query。

### 仍需观察

- 新 dynamic timeout / task tolerance / approach candidates 是否能改善 real lab 中的实际 Nav2 timeout。
- 生成的多个 approach candidates 是否真的更可达。
- perception 的 open object-like query 如果 YOLO 模型本身没有该 class，仍可能检测不到；但 planner 不应再误路由成 car/OCR。
- 复合 detection 应分别呈现 primitive 结果，例如：
  - `color_found`
  - `object_found`
  - `both_found / composite_found`

## 3. 当前架构方向

### Map Bundle 是最终方向

用户确认：要完成完整 map bundle。

目标结构可以类似：

```text
maps/sim_real_lab_001/
  nav2.yaml
  nav2.pgm
  map_3d.ply
  glim_dump/
  semantic_segments.yaml
  semantic_map.yaml
  metadata.yaml
  dashboard_examples.yaml
```

原则：

- map data belongs to map。
- agent / skill code 读取当前 selected map。
- parser 不写死环境 target。
- dashboard 不写死环境 examples。

### Semantic Schema

用户确认：

- `RXY`, `TKD`, `MIN` 等更倾向建模为 region alias。
- dashboard 只显示当前 map 中真实存在的 targets。
- agent-side parser / semantic event log 写到 `debug-output/agent/...`。

需要后续正式清理类别：

- `places`
- `objects`
- `regions`
- `region aliases`
- `desks / workstation aliases`
- `doors`

当前仍有 `front_door` / `back_door` 在 places 和 objects/region-like entry 中重复出现的风险，需要 schema 层定规则。

## 4. 剩余问题

### A. 完整 map bundle 尚未完成

当前已完成 real lab bundle 文件夹整理，并支持 `ROBOT_MAP_ID` 从 bundle 读取 semantic map。

还没有完全完成：

- launch file 统一注入 map id。
- segmentation tool 按 map bundle 读写。
- Nav2 map / semantic map / segment map / dashboard examples 的统一绑定。
- warehouse bundle 尚未整理到同样结构。

### B. Navigation controller 还需要精细讨论

已单独创建讨论文件：

- `todo-my/task-sim2real/motion-control-discussion.md`

当前判断：

- Nav2 已有 polygon footprint 和 MPPI `motion_model: Omni`。
- 但 `nav_mode_switch_node.py` 会削弱 lateral motion，让全向能力没有充分发挥。
- 狭窄通道需要单独 profile 和可通过性判断。

### C. Approach candidate 还不是 planner feasibility

现在只是从多个候选里选离当前 pose 最近的点。

后续更理想：

- 调 planner service 检查每个 candidate 是否能生成 path。
- 检查 path cost / footprint clearance。
- 不可达时返回 `needs_human` 或选择次优 candidate。

### D. Detection 仍缺少真正的 open-vocabulary detector

现在 planner 能把 `door` / `cart` 当 object-like target 交给 YOLO。

但是如果当前 YOLO 模型不支持该 class：

- planner 不会误触发 OCR。
- planner 不会误把 cart 当 car。
- 但实际 detector 仍可能找不到。

后续可选：

- 引入 YOLO-World / open-vocabulary detector。
- 或用 VLM 作为低频 fallback。
- 低置信度 / 多候选时 ask human。

### E. Detection 的解析边界还要继续收敛

理想行为：

- 用户没有显式定义特殊 task 时，agent/skill 应自动解析普通视觉任务。
- object-like 词不在当前 object registry 时，不应直接失败。
- object-like 词应进入下一层 detector / embedding / LLM 判断。
- 判断结果置信度高时直接执行检测。
- 置信度低或有多个候选时 ask human。
- 复合 query 应由 agent/skill 规划成 primitive steps，而不是靠枚举所有组合。

例子：

- `green door` = color `green` + object-like `door`。
- 执行顺序倾向：color > object > composite check，避免只依赖单一检测器漏检。
- 结果展示应能分别说明：
  - color ok / failed
  - object ok / failed
  - both ok / failed
- `find love` 不应被当成 object-like target。

## 5. 当前优先级

### Done

- Parser target handling data-driven。
- Warehouse / lab semantic map 物理分离。
- Runtime semantic map selection。
- Real lab map bundle 初版。
- Parser / semantic lightweight tests。
- Dashboard examples dynamic。
- Agent-side event log 可用于复盘。
- Dynamic timeout / task tolerance。
- Multiple approach candidates。
- Detection composite primitive planner。

### Next

1. 实测新的 navigation timeout / tolerance / candidate behavior。
2. 把 launch / tools 全部切到 map bundle contract。
3. 把 segmentation tool 改为显式读写 selected map bundle。
4. 清理 semantic schema，尤其是 region alias 和重复 door entries。
5. 继续讨论底层 motion controller profile：
   - `default`
   - `omni_full`
   - `narrow_passage`
   - `final_approach`
6. 评估 open-vocabulary perception fallback。

## 6. 已完成的本轮修改

### Detection 3-4

- 放开 object-like composite query：
  - `green_door`
  - `blue_cart`
- 修复 `cart` 被 substring 匹配成 `car` 的问题。
- 普通 color+object query 不再触发 OCR。
- 增加 primitive summary，便于 dashboard / agent 展示分步结果。
- 增加 perception planner regression tests。
- 增加 perception dry-run cases。

## 7. 待确认问题与用户回答

1. Parser data-driven 后继续完成完整 map bundle：yes。
2. Semantic map 选择机制优先使用 `ROBOT_MAP_ID`：yes。
3. Warehouse semantic map 需要恢复成独立 fixture：yes。
4. `RXY`, `TKD`, `MIN` 等建模为 region alias：yes。
5. Dashboard 只显示当前 map 中真实存在的 targets：yes。
6. Agent-side parser / semantic event log 写到 `debug-output/agent/...`：yes。

这些回答已经转成当前方向：

- 不再以覆盖 `semantic_map.yaml` 的方式切换地图。
- map/semantic/segment/dashboard examples 后续统一进入 map bundle。
- parser 和 dashboard 都应从 selected map 读取结构。
- detection 不靠枚举全部复合 task，而是解析成 primitive steps。
