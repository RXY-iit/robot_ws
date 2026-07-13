# L2/L3 Agent 与 Dashboard 测试指南

> 当前环境: sim first
> 当前策略: 人手按 A / 确认 AUTO；agent 不自动切 AUTO
> Dashboard: `http://127.0.0.1:8765`

## 1. 重新 build

```bash
cd /home/ruan-x/robot_ws
colcon build --symlink-install --packages-select robot_skills robot_agent
source install/setup.bash
```

注意：`agent_node` / dashboard 已经在运行时，build 后需要重启 agent 才会加载新的 Python 代码。

## 2. 启动 sim/nav 栈

使用现有 sim 导航启动方式：

```bash
cd /home/ruan-x/robot_ws
tools/sim/open_sim_nav2_windows.sh --cleanup
```

确认这些状态：

```text
/robot_mode 有输出
/safety_status 有输出
Nav2 NavigateToPose action server 存在
```

注意：agent 不会自动切 AUTO。需要导航时，请人工按 A 或通过你认可的方式切到 AUTO。

## 3. 启动 Agent + Dashboard

### 可选: 启动 llama.cpp 本地 LLM fallback

如果要测试 LLM fallback，先在单独终端启动 `llama-server`。模型文件名后续可以替换；这里用 3B GGUF 作为推荐形态：

```bash
~/local_llm/llama.cpp/build/bin/llama-server \
  -m ~/local_llm/models/qwen2.5-3b-instruct-q4_k_m.gguf \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 4096
```

如果暂时只有 1.5B 模型，也可以先用：

```bash
~/local_llm/llama.cpp/build/bin/llama-server \
  -m ~/local_llm/models/qwen2.5-1.5b-instruct-q4_k_m.gguf \
  --host 127.0.0.1 \
  --port 8080 \
  --ctx-size 4096
```

确认服务在线：

```bash
curl http://127.0.0.1:8080/v1/models
```

当前 agent 配置读取：

```text
src/robot_agent/config/model_policy.yaml
endpoint: http://127.0.0.1:8080/v1/chat/completions
```

新开终端：

```bash
cd /home/ruan-x/robot_ws
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch robot_agent agent_dashboard.launch.py mode:=sim
```

浏览器打开：

```text
http://127.0.0.1:8765
```

Dashboard 会显示：

```text
Command 输入框 + 常用命令按钮（日语优先）
Robot Health：Overall / Mode / Safety / Localization，绿色/黄色/红色状态卡
Mission 摘要：status / command / intent / result
Agent Flow：Mission Agent / Navigation Agent / Perception Agent
Skill Plan：每个 skill 的执行状态、所属 agent、payload、结果；“ステップ x / y” 表示当前执行到第 x 步 / 总 y 步
检测结果图片：detect_known_object 成功时显示所有 matched observation 图片
Mission Events：实时事件流
旧版 Robot 状态：robot_mode / safety_status / localization_warning / stationary_drift_m / human_prompt
Raw mission state：折叠显示，用于调试底层 JSON
```

## 4. Dry-run 测试

不需要完整 Nav2，也可以先测解析和 plan：

```bash
cd /home/ruan-x/robot_ws
source install/setup.bash
ros2 run robot_agent agent_dry_run "06棚に行って"
```

预期：

```text
MissionIntent: navigate_to_object
plan: resolve_semantic_target -> await_robot_auto_mode -> check_localization_health -> goto_pose -> arrival visual confirmation
goto_pose 返回 DRY_RUN_OK
```

新增基础移动 / 观察 dry-run：

```bash
ros2 run robot_agent agent_dry_run "前進少し"
ros2 run robot_agent agent_dry_run "左前方へ少し"
ros2 run robot_agent agent_dry_run "右前方へ50センチ"
ros2 run robot_agent agent_dry_run "turn around"
ros2 run robot_agent agent_dry_run "左を見て"
ros2 run robot_agent agent_dry_run "look left"
```

预期：

```text
前進少し -> move_relative distance_m=0.2
左前方へ少し -> move_relative_xy dx/dy around 0.141m
右前方へ50センチ -> move_relative_xy dx/dy around 0.354m
turn around -> rotate_relative yaw_deg=180
左を見て / look left -> set_camera_pose -> capture_observation -> return camera pose
```

## 5. ROS topic 命令测试

查看机器人状态：

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String "{data: '查看机器人状态'}"
```

列出语义目标：

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String "{data: '你知道哪些地方'}"
```

导航到架子 A 前：

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String "{data: '去06架子前'}"
```

如果当前不是 AUTO，预期：

```text
状态: NEEDS_HUMAN
human_prompt: Navigation requires AUTO mode. Press A / confirm AUTO...
```

按 A / 确认 AUTO 后，再发送同一条命令。

## 6. Dashboard 操作测试

在网页输入：

```text
去06架子前
```

点击 Send。

观察：

```text
Mission 摘要是否显示当前 command / intent / result
Agent Flow 是否高亮当前工作的 agent
Skill Plan 是否逐步显示 resolve / goto / camera / detect 的执行结果
Mission Events 是否出现 mission_received / intent_parsed / plan_created / skill_started / skill_finished
Robot Health 是否显示 robot_mode / safety_status
如果不是 AUTO，human_prompt 是否提示人工确认
```

点击 Cancel 可以发布 `/agent/cancel`。

## 7. 当前已实现能力

L2 Skills:

```text
get_robot_status
await_robot_auto_mode
check_localization_health
recover_localization_spin
resolve_semantic_target
list_known_targets
goto_pose
move_relative
move_relative_xy
rotate_relative
cancel_motion
capture_observation image artifact capture
detect_known_object with YOLO11s + OCR/rule fallback
set_camera_pose
move_lift_bounded
```

`move_lift_bounded` 使用距离语义，不推荐用户直接输入秒数：

```text
升降台上升10cm -> direction=UP, distance_m=0.10
升降台下降5cm -> direction=DOWN, distance_m=0.05
升降台上升一点 / リフトを少し上げて -> distance_m=0.05
升降台大幅下降 / リフトを大きく下げて -> distance_m=0.15
```

当前 sim bridge 仍只接受 `/lift/command` 的 `UP/DOWN/STOP`，所以 Skill 会按 `distance_m / 0.06 m/s` 换算成 `duration_sec`，再用 timer 发布 STOP。

L3 Agents:

```text
Mission Agent: 解析、计划、执行、状态管理
Navigation Agent: 语义目标 -> 导航 Skill plan
Health / Recovery Agent: /agent/health_state 定期发布健康状态；BAD 时请求 cancel motion；mission step 边界可 PAUSED_HEALTH -> RECOVERING -> RESUMING
Perception Agent: observation / detection plan
```

Health / Recovery 当前策略：

```text
1. /agent/health_state 持续发布 Overall / Mode / Safety / Localization。
2. heartbeat 检测到 overall=BAD 且 mission 正在运行时，会请求 cancel 当前 motion。
3. MissionAgent 在 base motion step 前检查 health。
4. safety BAD -> NEEDS_HUMAN。
5. localization BAD -> cancel_motion -> recover_localization_spin。
6. recovery OK -> RESUMING，继续当前 mission step。
7. recovery timeout -> NEEDS_HUMAN。
```

注意：当前是初版 Health / Recovery 介入。长时间 Nav2 action 被 heartbeat cancel 后的 resume 行为需要在 sim 中继续实测。

Dashboard:

```text
本地 HTTP + Server-Sent Events
无 FastAPI/React 依赖
第一阶段用于实时可视化
```

## 8. 当前限制

```text
部分 semantic_map 目标仍需要继续标注 polygon / 标签侧
detect_known_object 已接 YOLO11s + OCR/rule；06 数字确认已在 sim 跑通，其他物体/标签仍需要逐项实测和调阈值
capture_observation 会保存 RGB 图像和 metadata 到 debug-output/perception/<mission_id>/
agent 不会自动切 AUTO，这是当前安全策略
contact / grasp / press button 未启用
```

注意：`set_camera_pose` 在 sim 中依赖 `sim_lift_command_node`。
如果相机扫描时图像一直不变，通常是旧的 `sim_lift_command_node` 仍在运行。
需要重启 sim/nav 栈，或至少重启 `sim_lift_command_node`，让以下修复生效：

```text
AUTO 但机器人停止时允许相机转动
发布 /chokudomotor/angle 和 /cameraswingmotor/angle feedback
```

## 9. 如何知道“06架子”对应地图哪里

当前 agent 使用的是 L2 semantic map：

```text
src/robot_skills/config/semantic_map.yaml
```

`shelf_06` 当前使用 polygon 区域生成 approach pose，不再使用固定占位点：

```yaml
objects:
  - id: shelf_06
    names: ["shelf_06", "06架子", "架子06", "6号架子", "06 shelf", "shelf 06"]
    map_region:
      type: polygon
      polygon: [...]
      front_side: west
    approach_policy:
      generator: region_side_offset
      preferred_side: west
      anchor_mode: label_end
      label_end: south
      label_end_offset_m: 1.2
      distance_m: 0.75
      face_region_center: true
```

当前测试语义：

```text
“去06架子” = 去 06 数字标签可见的一侧附近
不是去整条长架子的几何中心
```

原因：当前 map 在架子中间部分比较稀疏，测试阶段优先靠近有数字标签的一端，方便相机停下后确认 `06`。

当前所有 shelf / shelf_region 都使用同一类 approach 规则：

```yaml
approach_policy:
  preferred_side: west
  anchor_mode: label_end
  label_end: south
  label_end_offset_m: 2.55
  distance_m: 0.45
```

以当前 06 架子为例，`去06架子` 会生成大约：

```text
x=2.662, y=5.840, yaw=1.44
```

含义：停靠点靠近数字标签所在的一端；朝向则看向架子主体区域，而不是看向临时 anchor 点。

### 推荐标定方法

1. 启动 sim/nav 和 RViz。
2. 在 RViz 里确认 `shelf_06.map_region.polygon` 覆盖真实 06 架子的范围。
3. 确认数字标签端对应 `approach_policy.label_end`。当前配置为 `south`。
4. 如果机器人应靠另一端看数字，只改 `label_end`，不要记一次手动停车点。

可观察：

```bash
ros2 topic echo /plan --once
```

或者看最新 debug：

```text
debug-output/sim_nav2_*/latest_state.md
```

里面的：

```text
plan.end.x
plan.end.y
plan.end.yaw
```

就是可以写回 semantic map 的候选值。

然后修改：

```text
src/robot_skills/config/semantic_map.yaml
```

推荐修改区域/侧向参数，而不是固定 approach pose：

```yaml
  - id: shelf_06
    ...
    approach_policy:
      preferred_side: west
      anchor_mode: label_end
      label_end: south      # north/south/east/west
      label_end_offset_m: 2.55
      distance_m: 0.45
```

修改后重新 build 或至少重新 source/重启 agent：

```bash
colcon build --symlink-install --packages-select robot_skills robot_agent
source install/setup.bash
ros2 launch robot_agent agent_dashboard.launch.py mode:=sim
```

### 可视化 semantic map marker

可以启动 marker 节点：

```bash
ros2 run robot_skills semantic_map_marker_node
```

在 RViz 添加：

```text
MarkerArray: /semantic_map/markers
```

这样可以看到 semantic map 里的目标点是否落在正确位置。注意：对于用 `map_region.polygon` 定义的目标，后续应扩展 marker 节点显示 polygon 和动态生成的 approach pose。

## 10. Segmentation map / Semantic map / Nav2 map 区别

现在系统里有三类“地图”，不要混用：

```text
Nav2 occupancy map:
  maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml
  maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.pgm
  用于 Nav2 路径规划和定位。

3D/GLIM map:
  maps/sim_factory_mid360_like_gpu_trueimu_011_3d.ply
  maps/sim_glim_dump_*/
  用于建图/点云调试，不直接给 agent 语义目标。

Semantic map:
  src/robot_skills/config/semantic_map.yaml
  用于告诉 agent “06架子/叉车/电梯按钮”这些名字对应哪个语义目标。优先存目标区域/范围，再由 resolver 生成 approach pose；不要把一次手动停靠位置当成永久目标点。
```

当前还没有真正的 segmentation map 文件。第一阶段 agent 使用的是 `semantic_map.yaml`。

现在已经新增人工标注 segmentation map：

```text
src/robot_skills/config/semantic_segments.yaml
```

当前只包含你提供的三个标注，不额外添加其他区域：

```yaml
segments:
  - id: initial_position      # 绿色圆圈
  - id: shelf_06              # 红色方框
  - id: shelf_05_06_region    # 绿色方框
```

这些 segment 目前是人工截图标注，`map_pose/map_polygon/approach_pose` 仍为空，不会被 agent 当作可导航目标。

`semantic_map.yaml` 里也已经加入：

```text
shelf_06
shelf_05_06_region
```

`shelf_06` 当前不是固定点标定，而是区域驱动标定入口：

```text
navigation_ready: false
map_region.polygon: []
approach_policy.generator: region_side_offset
```

你刚才手动停在 06 附近并确认 camera 可以看到 06 数字，这个信息已经记录为 calibration sample：

```text
x=3.50656
y=7.44818
yaw=1.41214
result=camera_can_see_06_label
use_as_fixed_goal=false
```

来源：

```text
debug-output/sim_nav2_20260623_214906/latest_state.md
用户手动停在 06 附近并确认 camera 可以看到 06 数字
```

这条记录只说明“这个附近能看到 06”，不会被直接当成永久导航点。

要让 `去06架子` 自动导航，需要先给 `shelf_06.map_region.polygon` 至少 3 个 map-frame 点，例如：

```yaml
map_region:
  type: polygon
  polygon:
    - {x: 0.0, y: 0.0}
    - {x: 0.0, y: 1.0}
    - {x: 0.4, y: 1.0}
    - {x: 0.4, y: 0.0}
  front_side: west
approach_policy:
  generator: region_side_offset
  preferred_side: west
  distance_m: 0.75
  face_region_center: true
```

填好 polygon 后，`resolve_semantic_target` 会根据区域中心、靠近侧和距离动态生成 approach pose。之后 `去06架子` 会走：

```text
resolve_semantic_target
await_robot_auto_mode
goto_pose
capture_observation
detect_known_object with YOLO11s + OCR/rule fallback
```

最后两步用于到达后的相机确认。相机角度不写进 semantic map；Perception Agent 会在停止后使用 `set_camera_pose -> capture_observation` 扫描，并用 `detect_known_object` 的 YOLO11s + OCR/rule fallback 检测是否看到目标。

当前 06 数字确认已经在 sim live test 中跑通：

```text
run_id: live_perception_06_after_fix_173645
query: 06数字
result: OK / Found "06数字".
artifacts: debug-output/perception/live_perception_06_after_fix_173645/
```

当前 perception scan 不假定机器人停在完全一致的位置。机器人停下后会拍 9 张图，覆盖地面/中层/上层 × 左/中/右：

```text
ground_left    pan=-35, tilt=80
ground_center  pan=0,   tilt=80
ground_right   pan=35,  tilt=80
middle_left    pan=-35, tilt=25
middle_center  pan=0,   tilt=25
middle_right   pan=35,  tilt=25
upper_left     pan=-35, tilt=-20
upper_center   pan=0,   tilt=-20
upper_right    pan=35,  tilt=-20
```

9 张图采集结束后，Perception Agent 会先执行：

```text
set_camera_pose pan=0 tilt=67 scan_band=return_home
```

让相机回到 navigation mode / sim lock camera 使用的 neutral 姿态，然后再执行 `detect_known_object`。

Dashboard 日语快捷命令示例：

```text
06棚に行って
06棚を確認して
06棚に赤い箱がありますか
ロボット状態
知っている場所を教えて
キャンセル
```

## 10.1 本地 Segmentation 标注工具

第一版工具已经可以加载 Nav2 map，并用 map yaml 里的原点和分辨率自动换算坐标：

```text
maps/sim_factory_mid360_like_gpu_trueimu_011_nav2.yaml
origin: [-29.168488, -26.009678, 0.0]
resolution: 0.05
```

启动：

```bash
cd /home/ruan-x/robot_ws
source install/setup.bash
ros2 run robot_skills segmentation_tool
```

浏览器打开：

```text
http://127.0.0.1:8770
```

操作方式：

```text
Polygon 模式：在地图上依次点击 shelf body 的 3-4 个角
Point 模式：只记录一个点
Segment 下拉框：切换已有 segment，并把 polygon / tag / policy 载入当前编辑区
已有 Segments 列表：点击某个 segment 也可以切换编辑
New：清空表单，用于追加新的 segment
右侧输入 id / category / aliases / tags / front_side / approach distance / yaw rule
点击 Save to YAML
```

保存后会更新：

```text
src/robot_skills/config/semantic_segments.yaml
src/robot_skills/config/semantic_map.yaml
```

并自动生成备份：

```text
semantic_segments.yaml.bak
semantic_map.yaml.bak
```

对于 `shelf_06`，推荐：

```text
ID: shelf_06
Category: shelf
Names: shelf_06,06架子,架子06,6号架子,06 shelf,shelf 06
Tags: rack, number_label, shelf_06
Front side: 选择机器人应该从哪一侧靠近
Distance m: 0.75
Yaw mode: face region center
Yaw offset deg: 0
Fixed yaw deg: 只在 Yaw mode=fixed yaw 时使用
Expected visual query: 06 shelf label
```

Yaw 规则说明：

```text
yaw=0.0 在 ROS map 里表示朝 +X 方向。
如果 front_side=west，机器人会站在 polygon 的 west 侧，并朝向区域中心；当架子在机器人东侧时，yaw 自然就是 0.0。
如果数字牌需要斜着看，可以调整 Yaw offset deg，例如 10、-15。
如果你已经知道绝对朝向，可以把 Yaw mode 改为 fixed yaw，并填写 Fixed yaw deg。
如果靠近侧选错了，先改 Front side，而不是手改 yaw。
```

保存后再测：

```bash
ros2 run robot_agent agent_dry_run "去06架子"
```

如果 polygon 至少 3 个点，`resolve_semantic_target` 应该返回 `generated_from_region`，并生成一个临时 approach pose。

## 11. Nav2 status 说明

`goto_pose` 的 Nav2 action status：

```text
4 = SUCCEEDED
5 = CANCELED
6 = ABORTED
```

如果看到 status 5，通常代表目标被取消，比如按了 B、发了 cancel、被新的 goal 抢占，或 Nav2/BT 取消了旧 goal。它不等同于路径规划失败。

## 12. 常见问题

如果 Dashboard 没有更新：

```bash
ros2 topic echo /agent/mission_events
ros2 topic echo /agent/status
```

如果导航不执行：

```bash
ros2 topic echo /robot_mode
ros2 action list | grep navigate_to_pose
```

如果 Dashboard 显示定位漂移：

```text
localization_warning = POSSIBLE_STATIONARY_DRIFT
```

含义：

```text
agent 观察到 /cmd_vel 近似为 0，但 /wheel_odom 在静止窗口内仍然移动超过阈值。
当前阈值: drift > 0.05 m 或 yaw drift > 0.10 rad，持续约 3 秒。
```

这不一定代表 localization 真的坏了，但应该暂停自动任务，检查：

当前 L3 流程会在每个 `goto_pose` 前插入：

```text
check_localization_health
```

如果健康检查失败，或 `goto_pose` 出现一次 `FAILED_NAVIGATION / FAILED_TIMEOUT`，Mission Agent 会先触发：

```text
recover_localization_spin
```

恢复策略：

```text
1. 发布 0 速度，停止机器人
2. 使用最近一次健康静止时记录的 last_trusted_pose 发布 /initialpose
3. 原地慢速旋转，给 GICP / localizer 重新匹配环境
4. localization 恢复 OK 后继续原任务
5. timeout 后仍未恢复，则 NEEDS_HUMAN，请人工确认位姿
```

如果 Dashboard 中出现 `Localization Agent` block 变亮，说明定位恢复流程正在介入。

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /wheel_odom --once
ros2 topic echo /robot_mode --once
ros2 topic echo /safety_status --once
```

如果提示 unknown target：

```bash
ros2 run robot_agent agent_dry_run "你知道哪些地方"
```

然后检查：

```text
src/robot_skills/config/semantic_map.yaml
```
