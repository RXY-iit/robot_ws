# Step 7 Extension - AI-ready Navigation, RealSense Costmap, Simulation Test Plan

作成: 2026-05-19 / 更新: 2026-05-20

目的: 現有 Navigation 已经具备 Nav2 + FAST-LIO2/GICP localization + manual/AUTO switch 的基础。本文件整理下一阶段要做的事情: 把 RealSense depth 加入 local costmap、明确 Phase 1-5 的目标、补齐未来 LLM/AI 分层缺口，并在实际运行前用 simulation / rosbag / fake node 验证逻辑。

> **最新 Phase 計画は `ai-navigation-guide.md` も参照すること。** 本文件は実装詳細・検証手順寄り、`ai-navigation-guide.md` はアーキテクチャ設計・議論点寄りで役割分担している。

---

## 0. 当前系统基线（Phase 1 完了時点）

**Phase 1 完了: 2026-05-20** — 実機で Nav2 goal SUCCEEDED（recoveries=0）確認済み。

当前推荐 Navigation 基线:

```text
map -> odom             : gicp_localizer_node
odom -> base_footprint  : wheel odom / robot_odom_node
base_footprint -> base_link -> livox_frame / camera_link : URDF
```

当前 Nav2 链路:

```text
Nav2 Goal
  -> planner_server (NavFn)
  -> controller_server (RPP, 0.10 m/s)
  -> /nav2/cmd_vel
  -> nav_mode_switch_node
     ├─ AUTO: relay -> /cmd_vel
     └─ MANUAL: zero /cmd_vel + cancel goal
  -> omni_base_driver
```

現在の BT: `navigate_to_pose_phase1_wait_only.xml`
- Spin / BackUp 無効（Phase 1 安全重視）
- plan fail → clear global → retry
- control fail → clear local → retry
- both fail → clear both → wait 2s

现有主要文件:

| 文件 | 作用 |
|---|---|
| `src/nav_pkg/launch/navigation.launch.py` | Nav2 planner/controller/behavior/bt + mode switch |
| `src/nav_pkg/config/nav2_params.yaml` | RPP controller, costmap, planner 参数 |
| `src/nav_pkg/scripts/nav_mode_switch_node.py` | A=AUTO, B=MANUAL + cancel Nav2 goal |
| `src/localization_pkg/launch/fast_lio_localization_live.launch.py` | 实机 localization 入口 |
| `src/robot_description/urdf/robot.urdf.xacro` | base, Livox, RealSense TF |
| `hardware/realsense_d435.md` | RealSense topics / TF / depth 说明 |

重要注意:

- URDF 注释和实际传感器参数有不一致风险。实际安装前必须重新测量 `livox_x/y/z/rpy` 和 `camera_x/y/z/rpy`。
- 现在 Nav2 costmap 主要依赖 `/scan`，而 `/scan` 来自 Livox pointcloud 切片。若 Livox 对地面近距离 1.7m 内不可见，则它适合定位和中远距离感知，但不适合作为唯一近距离避障来源。

---

## 1. RealSense depth を local costmap に追加（Phase 2 実装詳細）

### センサー分離アーキテクチャ決定（2026-05-20）

**Livox Mid360 → localization 専用**（FAST-LIO2 + GICP のみ。costmap には使わない）

**RealSense D435 → 全障害物検出担当**（local costmap + semantic 認識）

根拠:
- Livox は近地面 1.7m 以内の低障害物が見えにくい → 障害物検出には不向き
- Livox の主な価値は 3D point cloud による高精度 SLAM / localization
- RealSense は前方近距離（0.3-2.5m）に特化し、RGB + depth で semantic 認識にも対応

変更済み（URDF + nav2_params.yaml）:
- `local_costmap`: `/scan` → `/camera/scan`（depthimage_to_laserscan 出力）
- `global_costmap`: obstacle_layer を削除（static map のみ）

### カメラ TF 実測値（2026-05-20 計測）

| パラメータ | 値 | 換算 |
|---|---|---|
| 高さ（地面から） | 750 mm | camera_z = 0.750 - 0.1125 = **0.6375 m** |
| 前方オフセット（front face から） | +60 mm | camera_x = 0.260 + 0.060 = **0.320 m** |
| Y オフセット | 0 mm | camera_y = **0.0 m** |
| tilt angle at Nav pose | bracket 30° + motor 12° = **42°** | camera_pitch = **0.7330 rad** |

可視範囲（42°, height=0.75m）:
- 近端: **~0.26m** ahead, 幅 ~0.5m
- 中心: **~0.83m** ahead（カメラ正面が地面に当たる距離）
- 遠端: ~3.25m ahead（range_max で 2.5m にクリップ）

> **要確認**: URDF 更新後に RViz で depth image と camera_link TF を確認すること。
> depthimage_to_laserscan の `scan_height` を調整して適切な高さの scan を得ること。

### 目标

用 RealSense D435 depth 补 Livox 近距离盲区，让 local costmap 能看到机器人前方近距离低矮障碍物。

第一阶段只把 RealSense depth 用于 local costmap，不用于主 localization，不让 AI vision 直接控制机器人。

推荐数据流:

```text
RealSense depth image
  -> depthimage_to_laserscan 或 PointCloud2
  -> Nav2 local_costmap obstacle/voxel layer
  -> controller_server 避障
```

### 推荐实现路线 A: depthimage_to_laserscan

优点: 简单、调试快、接近当前 `/scan` 工作方式。

适合第一版验证:

```text
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
  -> /camera/scan
  -> local_costmap obstacle_layer
```

需要新增一个 depth 转 scan 节点。示例参数:

```yaml
depthimage_to_laserscan:
  ros__parameters:
    output_frame: camera_depth_frame
    range_min: 0.30
    range_max: 3.00
    scan_height: 10
```

然后 local costmap 增加 observation source:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      obstacle_layer:
        observation_sources: scan camera_scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_max_range: 2.5
          raytrace_max_range: 3.0
        camera_scan:
          topic: /camera/scan
          data_type: "LaserScan"
          marking: true
          clearing: true
          obstacle_max_range: 2.5
          raytrace_max_range: 3.0
```

注意:

- `camera_scan` 只加到 local costmap，先不要加到 global costmap。
- 如果 camera 有 pan/tilt，必须保证 TF 是当前真实角度，不然 costmap 会把障碍物投到错误位置。
- 如果 camera 是固定朝前，第一版更简单。

### 推荐实现路线 B: PointCloud2 + VoxelLayer

优点: 比 LaserScan 更适合 3D depth，可以处理高度信息。

适合第二版:

```text
RealSense depth
  -> /camera/camera/depth/color/points 或类似 PointCloud2 topic
  -> local_costmap voxel_layer
```

local costmap 示例:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: true
        observation_sources: camera_points
        camera_points:
          topic: /camera/camera/depth/color/points
          data_type: "PointCloud2"
          marking: true
          clearing: true
          min_obstacle_height: 0.05
          max_obstacle_height: 1.20
          obstacle_max_range: 2.5
          raytrace_max_range: 3.0
```

第一步建议先用路线 A，因为它更快发现 TF、topic、QoS、range、盲区问题。等确认有效后，再升级路线 B。

### 验证标准

- RViz 中 `/camera/scan` 或 camera pointcloud 与真实障碍位置一致。
- local costmap 中近距离障碍能出现，位置不漂移、不镜像、不旋转错误。
- 障碍物移走后 costmap 能清除。
- Nav2 AUTO 状态下，机器人在 0.10 m/s 低速接近障碍时会减速、绕行或停止。
- 按 B 键时无论 Nav2 处于什么状态，都能取消 goal 并发布 zero `/cmd_vel`。

### Pan/Tilt camera policy during Navigation

当前 RealSense D435 底座由两个互相垂直的 motor 控制:

| 方向 | Topic | 当前默认 |
|---|---|---:|
| Pan / 水平旋转 | `/chokudomotor/target_angle`, `/chokudomotor/angle` | `267.0 deg` |
| Tilt / 上下俯仰 | `/cameraswingmotor/target_angle`, `/cameraswingmotor/angle` | `102.0 deg` |

Navigation 行驶中，建议把相机固定在一个已校准的 **Nav pose**，不要连续扫描周围。

理由:

- Nav2 local costmap 需要稳定的 sensor frame。如果 camera 在移动中 pan/tilt，但 TF 没有实时反映真实角度，depth obstacle 会被投影到错误位置。
- 即使未来加入动态 TF，移动中的扫描也会让局部 costmap 的 marking/clearing 变复杂，容易出现“刚看到障碍又清掉”或“障碍残留”的问题。
- 移动中的安全避障应优先依赖稳定视野: Livox 负责 360° 中远距离，RealSense 固定朝前下方补近距离盲区。

推荐规则:

```text
NAVIGATING:
  camera fixed at Nav pose
  depth data allowed into local costmap

WAITING / BLOCKED / ARRIVED / MANUAL:
  camera may scan
  scan result goes to semantic layer first
  do not directly feed moving-camera depth into local costmap unless TF is correct

OBSERVE_AROUND:
  robot must stop
  publish zero cmd_vel
  rotate camera to several viewpoints
  collect RGB/depth/object detections
  return camera to Nav pose before resuming Nav2
```

可以改变相机角度的时机:

- 机器人已经停下，`/cmd_vel` 为 zero。
- Nav2 goal 被暂停、取消，或 Mission manager 进入 `WAITING` / `OBSERVE_AROUND`。
- 到达目标点后，需要识别货架、门、物体或人。
- navigation blocked，需要停下观察左右/后方来决定 retry、wait 或 ask human。
- 人工 MANUAL 模式下调试相机。

不建议改变相机角度的时机:

- Nav2 controller 正在输出 `/nav2/cmd_vel`。
- local costmap 正在使用 camera depth 做避障，但 pan/tilt TF 没有实时更新。
- 机器人靠近障碍、窄通道、转弯或倒车时。

---

## 1.1 调整相机初始 Nav pose 的 Guide

目标: 找到一个固定的 camera pan/tilt 初始角度，让 RealSense 在导航时稳定看到机器人前方近距离地面区域，特别是 Livox 近地盲区。

当前默认:

```text
camera_initial_pan_angle: 267.0
camera_initial_tilt_angle: 102.0
```

这两个值不是最终答案，只是当前可工作的启动目标。真正的 Nav pose 应该通过实测决定。

### Step A: 只启动相机、LiDAR、RViz，不启动移动电机

```bash
ros2 launch robot_bringup test_all.launch.py \
  camera:=true lidar:=true rviz:=true \
  drive_motors:=false steer_motors:=false \
  serial_motors:=true camera_motor_joy:=true \
  static_odom:=true
```

### Step B: 让相机回到当前默认初始角

```bash
ros2 topic pub --once /chokudomotor/target_angle std_msgs/msg/Float32 "{data: 267.0}"
ros2 topic pub --once /cameraswingmotor/target_angle std_msgs/msg/Float32 "{data: 102.0}"
```

确认反馈:

```bash
ros2 topic echo --once /chokudomotor/angle
ros2 topic echo --once /cameraswingmotor/angle
```

### Step C: 在 RViz / image viewer 中确认视野

要确认:

- depth image 中能看到机器人前方地面。
- 约 `0.3m - 2.0m` 前方区域有有效深度。
- 不要让画面大部分都是机器人自身结构、支柱、轮子或地面过近区域。
- 放一个低矮障碍物在机器人前方 `0.5m`, `1.0m`, `1.5m`，确认 depth 能看到。

推荐 Nav pose 视野:

```text
Pan:
  正前方为主，误差尽量小。

Tilt:
  轻微向下，看见 0.3-2.0m 前方地面和低矮障碍。
  不要过度向下，否则只能看到近处地面。
  不要过度水平，否则低矮障碍和地面盲区仍然看不到。
```

### Step D: 微调并记录最佳角度

用 Joy-Con:

```text
LB / L1 + D-pad 左右: pan
LB / L1 + D-pad 上下: tilt
```

每次调整后记录反馈:

```bash
ros2 topic echo --once /chokudomotor/angle
ros2 topic echo --once /cameraswingmotor/angle
```

建议记录表:

| Test | Pan deg | Tilt deg | 0.5m obstacle | 1.0m obstacle | 1.5m obstacle | Self occlusion | Note |
|---|---:|---:|---|---|---|---|---|
| A | 267.0 | 102.0 | TBD | TBD | TBD | TBD | current default |
| B | | | | | | | |

### Step E: 固化 Nav pose

如果新的最佳值是:

```text
PAN_NAV = xxx.x
TILT_NAV = yyy.y
```

则后续可以更新 `robot_bringup/launch/test_all.launch.py` 的默认值:

```text
camera_initial_pan_angle:=PAN_NAV
camera_initial_tilt_angle:=TILT_NAV
```

同时在 Navigation 启动流程中加入规则:

1. 启动时相机先回到 Nav pose。
2. Nav2 开始移动前确认 pan/tilt feedback 接近 Nav pose。
3. 若相机正在观察扫描，Mission manager 必须先让机器人停止。
4. 恢复导航前，相机必须回到 Nav pose。

### 后续需要实现的节点

为了让这个策略自动化，后续建议新增:

```text
camera_pose_manager
  Sub:
    /robot_mode
    /navigation_state
    /chokudomotor/angle
    /cameraswingmotor/angle
  Pub:
    /chokudomotor/target_angle
    /cameraswingmotor/target_angle
    /camera_pose_state
```

状态:

```text
NAV_POSE
OBSERVING_LEFT
OBSERVING_RIGHT
OBSERVING_FRONT
RETURNING_TO_NAV_POSE
MANUAL
```

Mission manager 恢复导航前必须等待:

```text
/camera_pose_state == NAV_POSE
```

---

## 2. Phase 1-5 应该做什么

### Phase 1: 基础导航闭环稳定 ✅ COMPLETED (2026-05-20)

目标: 不引入新 AI，不追求复杂避障，只证明 Nav2 基础链路可靠。

完了確認:

- [x] FAST-LIO2 + GICP localization 稳定发布 `map -> odom`
- [x] wheel odom 稳定发布 `odom -> base_footprint`
- [x] Nav2 goal SUCCEEDED（recoveries=0、distance_remaining=0.09m）
- [x] `nav_mode_switch_node` AUTO/MANUAL 切换正常
- [x] behavior_server が `/cmd_vel` を直接 publish しないことを確認（wait only BT）
- [x] B 键 cancel goal + zero `/cmd_vel` 正常
- [x] 速度 0.10 m/s 实机运行
- [ ] localization drop 検出の仕組み確認（Phase 2 に持ち越し）

証拠: `debug-output/nav2_20260520_151528/latest_state.md`

### Phase 2: 近距离感知補強 + localization 監視

目标: 用 RealSense depth 补 Livox 近地近距离盲区。Phase 1 持ち越し項目も対応。

范围:

- [ ] fake simulation launch (`navigation_logic_test.launch.py`) 作成 — 実機なし Nav2 logic テスト用
- [ ] localization_monitor_node 実装（GICP score + TF freshness 監視、閾値割れ時 BLOCKED 通知）
- [ ] camera Nav pose 実測・固化（Section 1.1 手順に従う）
- [ ] 添加 `depthimage_to_laserscan` → `/camera/scan` 発行
- [ ] local costmap に `camera_scan` observation source 追加
- [ ] RViz で近距離障害物が local costmap に入ることを確認
- [ ] 障碍物移开后 costmap clearing 確認

완성 기준:

- 0.3-1.7m 前方障碍物能进入 local costmap。
- 障碍物移开后可以清除。
- fake simulation で AUTO/MANUAL/cancel/goal logic が正常動作。
- localization score 低下時に警告が出る。

**注意:** Nav 中はカメラを Nav pose に固定。pan/tilt 中は TF がずれて costmap 障害物位置が乱れる（Section 1 参照）。

### Phase 3: Safety Layer + Footprint 実装

目标: `robot_radius` を実機 polygon footprint に置き換え、safety layer を独立パッケージとして実装する。

新規パッケージ `src/safety_layer/`:

- [ ] `cmd_vel_safety_node.py`: cmd_vel watchdog（一定時間無更新で zero）+ 速度上限 + 急停入力
- [ ] AUTO/MANUAL 仲裁を safety layer に移管（現在は `nav_mode_switch_node` が担当）
- [ ] MANUAL 時 teleop: `/teleop/cmd_vel` → safety layer → `/cmd_vel`（直接 publish 禁止）

Nav2 パラメータ調整:

- [ ] `robot_radius` → 実機寸法の polygon footprint に変更
- [ ] inflation_radius / cost_scaling_factor を実機に合わせて調整
- [ ] RPP controller パラメータ実機チューニング（最大速度・加速度・角速度）
- [ ] recovery behavior の有効化検討: spin / backup を低速・短距離限定で許可するか評価
- [ ] `camera_pose_manager_node` を `serial_transciever` に追加（Nav pose 自動復帰）

完成标准:

- 窄通道不会擦边。
- behavior_server も safety layer を通る（`/cmd_vel` publisher に behavior_server が現れない）。
- recovery が制造新的碰撞风险になっていない。
- teleop の `/cmd_vel` authority が safety layer に一元化されている。

### Phase 4: Mission Manager 実装

目标: Nav2 goal の直接発行から、状態機ベースの Mission Manager に移行。この時点で Nav2 は「Mission Manager のサブシステム」になる。

新規パッケージ `src/mission_manager/`:

- [ ] `mission_manager_node.py`: Nav2 action client + タスク状態機
- [ ] 状態: `IDLE / NAVIGATING / WAITING / OBSERVE_AROUND / APPROACH_PRECISE / MANIPULATE / BLOCKED / FAILED / MANUAL`
- [ ] waypoint_map.yaml: 登録 waypoint 一覧管理
- [ ] retry / cancel / wait policy 実装
- [ ] `/camera_pose_state == NAV_POSE` を確認してから nav 再開するゲート
- [ ] BT 拡張: `wait-for-human`, `blocked-check`, `semantic-condition` ノード追加
- [ ] Nav2 SpeedFilter + KeepoutFilter の zone.yaml 設定

Controller 評価（サブタスク）:

- [ ] 仮想環境で MPPI を RPP と比較評価（同一 goal・同一障害）
- [ ] 全向移動・動的障害物での改善が明確な場合のみ実機評価
- [ ] MPPI パラメータ不安定なら RPP 継続

完成标准:

- RViz から Nav2 goal を直接送るのではなく、Mission Manager 経由でタスクを発行できる。
- BLOCKED / FAILED 状態に遷移し、適切に stop または human 通知できる。
- Lift と Nav の排他制御が実装されている（lift 動作中は Nav 禁止）。

### Phase 5: AI / Semantic / LLM 接続

目标: Semantic Layer と LLM Task Planner を接続。AI は受限 Task API 経由でのみ Mission Manager に指示。

5 層アーキテクチャ（目標）:

```text
Layer 5: LLM / AI Task Planner
  ↓ Task API のみ
Layer 4: Mission Manager
  ↓ Nav2 Action / Semantic Events
Layer 3: Perception / Semantic Layer
  ↓ /scan, /camera/scan, /map, /costmap
Layer 2: Nav2 Navigation
  ↓ /nav2/cmd_vel
Layer 1: Motor Safety Layer
  ↓ /cmd_vel
Layer 0: Hardware (omni_base_driver / lift / camera motor)
```

新規パッケージ `src/perception_semantic/`:

- [ ] RealSense RGB + 物体検出モデル（YOLOv8 等）統合
- [ ] `semantic_object_pub.py`: 検出結果を `SemanticObject[]` msg に正規化
- [ ] `semantic_zone_manager.py`: 棚・扉・禁入区・待機区のゾーン管理

LLM Tool API（制限付き）:

```text
go_to(target_id)          ← 登録済み waypoint のみ
cancel_navigation()
wait(seconds)
get_status()
report_obstacle(description)
ask_human(message)
```

完成标准:

- AI が `/cmd_vel` を直接 publish しない。
- AI の目標は地図範囲・危険区域・可達性チェックを通過した場合のみ実行。
- AI 失効・超時・出力フォーマット错误時は機器停止または現在の安全策略継続。
- Phase 5 前半: Semantic detection のみ（物体認識 + zone 管理）。
- Phase 5 後半: LLM Task Planner 接続。

---

## 3. 未来分层目前缺少什么（Phase 1 完了時点）

目標 5 层アーキテクチャ（詳細は `ai-navigation-guide.md` Section 1 参照）:

```text
Layer 5: LLM / AI Task Planner
Layer 4: Mission Manager
Layer 3: Perception / Semantic Layer
Layer 2: Nav2 Navigation
Layer 1: Motor Safety Layer
Layer 0: Hardware (omni_base_driver / lift / camera motor)
```

### 已有部分（Phase 1 完了時点）

| 層 / コンポーネント | 状態 |
|---|---|
| Nav2（Layer 2）| ✅ `navigation.launch.py`, `nav2_params.yaml`, Phase1 BT |
| AUTO/MANUAL switch | ✅ `nav_mode_switch_node.py`（現在 Layer 1 相当だが未分離） |
| omni_base_driver（Layer 0）| ✅ `/cmd_vel -> motors` + wheel odom |
| localization | ✅ FAST-LIO2 + GICP baseline |
| LiDAR sensing | ✅ Livox `/livox/lidar -> /scan` |
| camera hardware | ✅ RealSense D435 driver / topics |
| debug logging | ✅ `debug-output/nav2_*/events.jsonl` |

### 缺少部分（Phase 別優先度）

| 缺口 | 対応 Phase | 状態 |
|---|---|---|
| localization drop 検出 | Phase 2 | 未着手 |
| fake simulation launch | Phase 2 | 未着手 |
| RealSense local obstacle integration | Phase 2 | 未着手 |
| camera Nav pose 実測・固化 | Phase 2 | 未着手 |
| camera_pose_manager_node | Phase 3 | 未着手 |
| Motor safety layer（独立パッケージ化） | Phase 3 | 未着手 |
| Polygon footprint | Phase 3 | 未着手 |
| Nav2 recovery 有効化評価（spin/backup） | Phase 3 | 未着手 |
| Mission Manager | Phase 4 | 未着手 |
| Behavior Tree 拡張（blocked-check 等） | Phase 4 | 未着手 |
| SpeedFilter + KeepoutFilter zone | Phase 4 | 未着手 |
| Controller 評価（MPPI vs RPP） | Phase 4 | 未着手 |
| Semantic object interface | Phase 5前半 | 未着手 |
| 物体検出モデル統合 | Phase 5前半 | 未着手 |
| Semantic map / zones | Phase 5前半 | 未着手 |
| LLM Tool API | Phase 5後半 | 未着手 |
| Logging / rosbag replay workflow | Phase 2 | 部分的（記録コマンドは Section 4 参照） |

### 建议新增包

```text
src/mission_manager/        ← Phase 4
  mission_manager_node.py   ← タスク状態機 + Nav2 action client
  task_state.py
  waypoint_map.yaml

src/safety_layer/           ← Phase 3
  cmd_vel_safety_node.py    ← watchdog + 速度上限 + 急停
  (nav_mode_switch_node 機能を統合予定)

src/perception_semantic/    ← Phase 5
  semantic_object_pub.py
  semantic_zone_manager.py

serial_transciever/ に追加:
  camera_pose_manager_node.py  ← Phase 3
```

### LLM 接入原则

LLM 允许:

- 解释任务: “去货架 A 附近”
- 选择目标: `target_id=shelf_A_wait_point`
- 请求状态: “当前为什么停下”
- 选择策略: retry / wait / ask human

LLM 不允许:

- 直接发布 `/cmd_vel`
- 修改 Nav2 costmap 参数
- 绕过 B 键/manual override
- 在未知区域随意生成坐标
- 忽略 sensor stale / TF error / localization lost

---

## 4. 实际运行前如何在 simulation 环境测试逻辑

这里的 simulation 不一定第一步就要完整 Gazebo 物理仿真。建议分三层，从轻到重。

### Simulation Level 1: ROS graph / fake odom / fake scan 测试

目标: 不启动真实电机，不启动真实传感器，只验证 Nav2 action、BT、mode switch、cancel、topic remap 是否正确。

思路:

```text
fake map_server
static map -> odom
fake odom -> base_footprint
fake /scan
Nav2
nav_mode_switch_node
```

要验证:

- Nav2 能进入 active。
- 发送 goal 后会规划路径。
- `/nav2/cmd_vel` 会输出。
- 未按 AUTO 时 `/cmd_vel` 不应该被 Nav2 控制。
- 切 AUTO 后 `/nav2/cmd_vel -> /cmd_vel`。
- 切 MANUAL 后 cancel goal + zero `/cmd_vel`。

建议新增一个 launch:

```text
src/nav_pkg/launch/navigation_logic_test.launch.py
```

它启动:

- `map_server`
- `static_transform_publisher map odom`
- fake odom publisher
- fake laser scan publisher
- Nav2 stack
- `nav_mode_switch_node`

完成后可以不接真实机器人也测试 navigation control logic。

### Simulation Level 2: rosbag replay 测试

目标: 用真实传感器数据复现之前 navigation error，避免每次都推实机。

记录建议:

```bash
ros2 bag record \
  /tf /tf_static \
  /wheel_odom \
  /livox/lidar /livox/imu /scan \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/color/image_raw \
  /map \
  /local_costmap/costmap /global_costmap/costmap \
  /plan /nav2/cmd_vel /cmd_vel /robot_mode
```

回放时要注意:

- `use_sim_time:=true`
- 所有 Nav2/localization 节点使用 bag clock
- 不要同时启动真实 sensor driver
- 不要让真实 motor node 接收 `/cmd_vel`

要验证:

- navigation error 是否能复现。
- TF 是否在错误发生前跳变。
- `/scan` 或 camera obstacle 是否 stale。
- local costmap 是否在错误位置出现障碍。
- Nav2 是否因为 progress checker / goal checker / transform timeout 失败。

### Simulation Level 3: Gazebo / Ignition 仿真

目标: 在带机器人模型、地图、虚拟传感器的环境中测试路径规划和避障。

适合测试:

- footprint
- controller 参数
- camera depth obstacle source
- recovery behavior
- dynamic obstacle

需要补齐:

- robot URDF 的 Gazebo plugin
- diff/omni/ackermann 对应的 base controller plugin
- 2D LiDAR 或 depth camera sensor plugin
- world 文件，对应 L402 地图或简化走廊
- clock / use_sim_time 配置

注意:

- Gazebo 中的电机模型和真实三轮转向/全向底盘不一定完全一致。
- Gazebo 适合验证逻辑和参数趋势，不等于真实安全认证。
- 实机前仍必须低速、短距离、人工急停测试。

---

## 5. 结合之前 navigation error 的重点检查项

之前的调查里，Navigation 相关风险集中在 TF ownership、localization 来源、topic remap、costmap freshness、manual/AUTO authority。实机前按下面顺序排查。

### TF ownership

只能有一个节点负责:

```text
map -> odom
```

只能有一个节点负责:

```text
odom -> base_footprint
```

检查:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo base_link camera_link
```

### topic authority

Nav2 不直接写真实 `/cmd_vel`，必须经过 mode/safety layer。

检查:

```bash
ros2 topic info -v /nav2/cmd_vel
ros2 topic info -v /cmd_vel
ros2 topic echo /robot_mode
```

期望:

- AUTO 时: `/nav2/cmd_vel` 被 relay 到 `/cmd_vel`
- MANUAL 时: Nav2 goal 被 cancel，`/cmd_vel` 为 zero 或由 teleop dead-man 控制
- Teleop 只发布 `/teleop/cmd_vel`，不直接发布 `/cmd_vel`。
- `/cmd_vel` 的 Nav/teleop 仲裁只由 `nav_mode_switch_node` 负责。

重要修正记录:

- `controller_server` 必须 remap `cmd_vel -> /nav2/cmd_vel`
- `behavior_server` 也必须 remap `cmd_vel -> /nav2/cmd_vel`
- 如果 `/cmd_vel` publisher 里出现 `behavior_server`，说明 Nav2 recovery behavior 可以绕过 `nav_mode_switch_node` 直接控制真实机器人。此时即使 `/robot_mode` 显示 `MANUAL`，机器人也可能旋转或 backup。
- 修正后，`ros2 topic info -v /cmd_vel` 中不应该再看到 `behavior_server`。
- 修正后，`ros2 topic info -v /cmd_vel` 中也不应该看到 `teleop_twist_joy_node`。
- `teleop_twist_joy_node` 应该发布 `/teleop/cmd_vel`，AUTO 模式下由 `nav_mode_switch_node` 忽略，MANUAL 模式下才 relay 到 `/cmd_vel`。

一键启动 Phase 1 safe check:

```bash
tools/open_nav2_windows.sh --phase1-safe --cleanup
```

这个模式会启动 sensors / localization / Nav2 / RViz，但不会启动 drive/steer/serial/lift motors。适合在真实机器人移动前确认:

- RViz 有 map / robot / costmap / plan。
- 发送近距离 `2D Goal Pose` 后，Nav2 能产生 `/plan` 和 `/nav2/cmd_vel`。
- `/cmd_vel` 即使有输出，也不会进入 `cmd_vel_to_motor_node`，因为移动电机节点没有启动。

实机移动测试:

```bash
tools/open_nav2_windows.sh --cleanup
```

只有在 Phase 1 safe check 通过后再使用。启动后先保持 MANUAL，确认 `/cmd_vel` publisher 中没有 `behavior_server`，再按 A 进入 AUTO。

自动 debug 记录:

`tools/open_nav2_windows.sh` 每次启动都会创建新的 session folder:

```text
debug-output/nav2_YYYYmmdd_HHMMSS/
```

里面包含:

| 文件 | 内容 |
|---|---|
| `run_info.md` | 本次记录说明 |
| `latest_state.md` | 当前最新状态，适合直接贴给 Codex |
| `events.jsonl` | mode/joy/goal/plan/action/cmd_vel/odom 的时间序列事件 |

测试后可以打包最新记录:

```bash
tools/pack_latest_nav2_debug.sh
```

给 Codex debug 时，优先提供:

```text
debug-output/nav2_xxxxx/latest_state.md
debug-output/nav2_xxxxx/events.jsonl 中 goal 之后的几行
```

### costmap freshness

检查:

```bash
ros2 topic hz /scan
ros2 topic hz /camera/scan
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

观察:

- 障碍物出现时 local costmap 是否 marking。
- 障碍物移开时 local costmap 是否 clearing。
- costmap 是否整体偏移或旋转。

### localization stability

检查:

```bash
ros2 topic hz /wheel_odom
ros2 topic echo /gicp_loc/pose --once
ros2 run tf2_ros tf2_echo map base_footprint
```

观察:

- robot pose 是否跳变。
- 停车时 pose 是否漂移。
- 转弯时 map 中姿态是否跟真实一致。

### odom direction sanity

如果手动 `/cmd_vel` 验证中机器人真实前进方向正确，但 `/wheel_odom` 的 x 方向相反，Nav2 会误以为机器人前进时在后退。这会导致:

- 正前方目标时 `/nav2/cmd_vel linear.x` 可能变成负值。
- controller 可能反复失败。
- recovery behavior 可能被触发，表现为原地旋转或后退。

当前修正:

```text
src/omni_base_driver/src/robot_odom_node.cpp
```

在 `KinemaMatrix.calcRobotVelocity(...)` 后只翻转 `vx`:

```cpp
vx = -vx;
```

原因: 实测 `/cmd_vel x` 执行方向正确，`/wheel_odom y` 方向正确，只有 `/wheel_odom x` 方向相反。

验证:

```bash
ros2 topic echo /wheel_odom
```

手动轻微前进时应满足:

```text
twist.twist.linear.x > 0
pose.pose.position.x 增加
```

手动轻微左/右平移时，y 方向保持原先正确符号。

---

## 6. 最小实施顺序（Phase 1 完了後）

Phase 1 が完了したので、次の順序で進める:

1. **[Phase 2]** URDF 校准: Livox 与 RealSense 的真实 xyz/rpy 実測。
2. **[Phase 2]** `navigation_logic_test.launch.py` 作成: fake sensors で AUTO/MANUAL/cancel logic テスト。
3. **[Phase 2]** localization_monitor_node 実装: GICP score 監視 + 閾値割れ警告。
4. **[Phase 2]** camera Nav pose 実測（Section 1.1 手順）。
5. **[Phase 2]** RealSense depthimage_to_laserscan → `/camera/scan` 発行。
6. **[Phase 2]** `/camera/scan` を local costmap に追加、RViz で位置確認。
7. **[Phase 2]** rosbag replay で nav error 復元検証（Section 4 参照）。
8. **[Phase 3]** `safety_layer` パッケージ実装（watchdog + 速度上限）。
9. **[Phase 3]** polygon footprint + inflation radius 実機チューニング。
10. **[Phase 3]** `camera_pose_manager_node` 実装。
11. **[Phase 4]** Mission Manager 実装 + BT 拡張。
12. **[Phase 4]** MPPI vs RPP 仿真比較評価。
13. **[Phase 5前半]** Semantic detection 統合（物体検出 + zone 管理）。
14. **[Phase 5後半]** LLM Tool API 接続。

---

## 7. 完成检查表

### Phase 1 ✅ COMPLETED (2026-05-20)

- [x] FAST-LIO2 + GICP localization 稳定。
- [x] Nav2 RPP + Phase1 BT (wait only) 実機確認。
- [x] AUTO/MANUAL ジョイコン切替正常。
- [x] behavior_server が `/cmd_vel` を直接 publish しない。
- [x] B 键 cancel goal + zero `/cmd_vel`。
- [x] 実機 short-distance goal SUCCEEDED（recoveries=0）。

### Phase 2（次のターゲット）

- [ ] URDF 中 Livox 与 RealSense 位姿和真实安装一致。
- [ ] RealSense depth 能稳定发布。
- [ ] `/camera/scan` 或 camera PointCloud2 能稳定发布。
- [ ] local costmap 同时使用 Livox `/scan` 和 RealSense 近距离数据。
- [ ] 近距离 0.3-1.7m 障碍能进入 local costmap。
- [ ] 障碍移开后 costmap 能清除。
- [ ] fake simulation 中 Nav2 goal / AUTO / MANUAL / cancel 逻辑正常。
- [ ] rosbag replay 可以复现或排除之前 navigation error。
- [ ] localization drop 検出が動作する。

### Phase 3

- [ ] `/cmd_vel` authority 清楚，safety layer に一元化。
- [ ] AI/Nav2 不能绕过 safety layer。
- [ ] polygon footprint 実机チューニング済み。
- [ ] camera_pose_manager_node 実装済み。

### Phase 4

- [ ] Mission manager 和 safety layer 的接口设计完成。
- [ ] Mission Manager 状態機が BLOCKED/FAILED で適切に動作。
- [ ] Lift と Nav の排他制御実装。

### Phase 5

- [ ] Semantic object detection 統合。
- [ ] LLM が `/cmd_vel` を直接触れない。
- [ ] LLM Tool API 受限実装済み。
