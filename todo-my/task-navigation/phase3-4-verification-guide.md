# Phase 3–4 Verification Guide

作成: 2026-05-21

## 実施した変更のまとめ

| 変更 | ファイル |
|---|---|
| inflation_radius 修正 (local 0.10→0.40, global 0.30→0.50) | `nav_pkg/config/nav2_params.yaml` |
| polygon footprint 追加 (MPPI CostCritic の `consider_footprint=true` 用) | 同上 |
| MPPI を primary controller に追加、RPP を fallback に変更 | 同上 |
| Spin / BackUp / Wait を behavior_server に有効化 | 同上 |
| Phase3 BT (Spin + ClearCostmap + Wait、6リトライ) | `nav_pkg/behavior_trees/navigate_to_pose_phase3.xml` |
| safety_layer パッケージ新規作成 | `src/safety_layer/` |
| nav_mode_switch_node の出力を `/cmd_vel_raw` に変更 | `nav_pkg/scripts/nav_mode_switch_node.py` |
| navigation.launch.py に cmd_vel_safety_node 追加 | `nav_pkg/launch/navigation.launch.py` |
| mission_manager パッケージ新規作成 | `src/mission_manager/` |

---

## 検証前の必須手順

```bash
cd ~/robot_ws
colcon build --packages-select safety_layer mission_manager nav_pkg --symlink-install
source install/setup.bash
```

---

## 検証 1: /cmd_vel topic authority (Safety Layer)

### 目的
Safety layer が `/cmd_vel_raw` → `/cmd_vel` に正しく挿入されているか確認。

### 手順

```bash
# terminal 1: navigation launch
ros2 launch nav_pkg navigation.launch.py

# terminal 2: topic publisher confirmation
ros2 topic info -v /cmd_vel_raw
ros2 topic info -v /cmd_vel
ros2 topic info -v /nav2/cmd_vel
```

### 期待値

```
/cmd_vel_raw  publishers: nav_mode_switch_node
/cmd_vel      publishers: cmd_vel_safety_node   ← safety_layer のみ
/nav2/cmd_vel publishers: controller_server, behavior_server
```

**FAIL条件**: `/cmd_vel` に `nav_mode_switch_node` や `behavior_server` が直接 publish している場合。

Note: `/nav2/cmd_vel` は valid goal を controller が追従している間だけ実データが流れる。Goal 未送信の状態では topic/publisher が存在していても `ros2 topic echo /nav2/cmd_vel` が無音なのは正常。

---

## 検証 2: Safety Layer 動作確認

### 2-A: Speed clamp

```bash
# /cmd_vel_raw に制限値を超えた速度を注入
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: 5.0, y: 5.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 10.0}}"

# /cmd_vel の出力を確認 → clamp されているか
ros2 topic echo /cmd_vel --once
```

**期待値**: `vx ≤ 0.15`, `vy ≤ 0.10`, `wz ≤ 0.5`

### 2-B: Watchdog

```bash
# cmd_vel_raw に一度だけ送信して 1 秒待つ
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

sleep 1.5

# /cmd_vel が zero になっているか確認
ros2 topic echo /cmd_vel --once

# safety_status が WATCHDOG になっているか
ros2 topic echo /safety_status --once
```

**期待値**: 0.5秒後に `/cmd_vel` がゼロ, `/safety_status` = `"WATCHDOG"`

### 2-C: Emergency stop (Y button)

Joy-Con の Y ボタン (index 3) を押す:

```bash
ros2 topic echo /safety_status
```

**期待値**: `"ESTOP"` に切り替わり、`/cmd_vel` がゼロになる。再度 Y ボタンで `"OK"` に戻る。

---

## 検証 3: Inflation Radius 修正確認

### 手順

```bash
# Nav2 起動後、ログに ERROR が出ないことを確認
ros2 launch nav_pkg navigation.launch.py 2>&1 | grep -E "inflation|inscribed"
```

### 期待値

**修正前 (FAIL):**
```
[ERROR] inflation_radius (0.100) is smaller than the computed inscribed radius (0.353)
```

**修正後 (PASS):**
エラーなし。または `inflation_radius (0.40) >= inscribed radius (0.353)` 相当のメッセージ。

追加で以下の footprint エラーが出ないこと:

```
Considering footprint in collision checking but no robot footprint provided in the costmap.
```

このエラーが出る場合、MPPI controller が configure に失敗し、`controller_server` が active にならない。その結果 `/nav2/cmd_vel` は出ず、local/global costmap も RViz で正常表示されない。

### RViz での確認

- local costmap を表示
- ロボット前方 1m 以内に障害物を置く
- costmap の膨張領域がロボット半径 (0.35m) より広くなっているか視認

---

## 検証 4: MPPI Controller の holonomic 動作確認

### 目的
`linear_y ≠ 0` が出力されることを確認。これが Phase 1/2 RPP との最大の違い。

### 手順

```bash
# AUTO モードで Nav2 goal を送信中に監視
ros2 topic echo /nav2/cmd_vel
```

**期待値 (MPPI):**
```
linear:
  x: 0.08    ← 前進
  y: -0.03   ← 横移動 (ゼロでないことが重要)
angular:
  z: 0.15
```

**FAIL条件**: `linear.y` が常に `0.0` のまま → MPPI が RPP にフォールバックしている疑い。

### controller ログ確認

```bash
ros2 launch nav_pkg navigation.launch.py 2>&1 | grep -i "controller\|mppi"
```

**期待値**: `MPPIController` が `FollowPath` に割り当てられていることを確認。

---

## 検証 5: Phase3 BT の Spin recovery 確認

### 目的
障害物でブロックされたとき、Spin recovery が実行されることを確認。

### 手順

1. 障害物をロボット正面 0.5m に置く
2. AUTO モードで障害物の先にゴールを送信
3. debug recorder を起動しておく

```bash
# bt_navigator のログを監視
ros2 launch nav_pkg navigation.launch.py 2>&1 | grep -E "BT|Spin|recovery|abort|collision"
```

**期待される動作シーケンス:**
```
[controller] MPPI: costmap obstacle detected (減速・回避試み)
[controller] patience exceeded (回避できなかった場合)
[BT] ClearLocalCostmap
[BT] retry FollowPath
[BT] Spin 90° (1.57 rad)
[BT] retry ComputePathToPose + FollowPath
```

**FAIL条件**: Spin が起動せずに即座に `Goal failed` になる。

---

## 検証 6: Mission Manager 基本動作

### 6-A: 起動確認

```bash
# 別ターミナルで起動
source ~/robot_ws/install/setup.bash
ros2 run mission_manager mission_manager_node

# status を確認
ros2 topic echo /mission_status
```

**期待値**: `"IDLE: "` が 1Hz で出力される。

### 6-B: go_to コマンド (AUTO モード)

```bash
# まず A ボタンで AUTO モードに切り替える (または robot_mode を確認)
ros2 topic echo /robot_mode --once

# waypoint_map.yaml の "home" に移動
ros2 topic pub --once /mission_command std_msgs/msg/String "{data: 'go_to:home'}"

# ステータス監視
ros2 topic echo /mission_status
```

**期待される状態遷移**: `IDLE → NAVIGATING → IDLE (succeeded)`

### 6-C: cancel コマンド

```bash
# ナビゲーション中に cancel
ros2 topic pub --once /mission_command std_msgs/msg/String "{data: 'cancel'}"
```

**期待値**: `NAVIGATING → IDLE (cancelled)`

### 6-D: MANUAL モード中のコマンド拒否

```bash
# B ボタンを押して MANUAL に切り替え
ros2 topic pub --once /mission_command std_msgs/msg/String "{data: 'go_to:home'}"
```

**期待値**: ログに `Cannot start task: robot is in MANUAL mode` と出て、ナビゲーションが開始しない。

### 6-E: retry/BLOCKED 動作

- 到達不可能なゴールを送信
- 3回失敗 → `BLOCKED` 状態に遷移するか確認

```bash
ros2 topic echo /mission_status
```

**期待値**: `"BLOCKED: retries exhausted — waiting for human"` が出力される。

---

## 検証 7: topic authority の総合チェック

```bash
# /cmd_vel publishers
ros2 topic info -v /cmd_vel | grep "Node name"

# /cmd_vel_raw publishers
ros2 topic info -v /cmd_vel_raw | grep "Node name"
```

**期待値:**
```
/cmd_vel      → cmd_vel_safety_node のみ
/cmd_vel_raw  → nav_mode_switch_node のみ
/nav2/cmd_vel → controller_server, behavior_server
```

**FAIL条件**: `behavior_server` や `teleop_twist_joy_node` が `/cmd_vel` に直接 publish している。

---

## 既知の制限事項 (Phase 3 時点)

| 制限 | 理由 | 対応予定 |
|---|---|---|
| `linear_y` は ±0.06 m/s に制限 | 側方センシングなし | RealSense pan カバレッジ拡張後に緩和 |
| BackUp は BT に未追加 | 後方センシングなし | 後方センサー追加後に検討 |
| waypoint_map.yaml の座標が未設定 | 実測が必要 | RViz で実測して記入 |
| Mission Manager は手動コマンドのみ | LLM/AI 未接続 | Phase 5 |

---

## 検証完了チェックリスト

- [ ] `/cmd_vel` の publisher が `cmd_vel_safety_node` のみ
- [ ] `/cmd_vel_raw` の publisher が `nav_mode_switch_node` のみ
- [ ] Speed clamp が正常動作 (vx≤0.15, vy≤0.10, wz≤0.5)
- [ ] Watchdog: 0.5s 後に zero 出力
- [ ] Y button emergency stop 動作
- [ ] inflation ERROR ログが出ない
- [ ] MPPI の `linear_y ≠ 0` を観測
- [ ] 障害物前で Spin recovery が発動
- [ ] Mission Manager IDLE 起動
- [ ] go_to コマンドで NAVIGATING → IDLE 遷移
- [ ] MANUAL モード中のコマンド拒否
- [ ] 失敗 3回で BLOCKED 遷移
