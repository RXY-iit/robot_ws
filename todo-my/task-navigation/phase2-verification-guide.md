# Phase 2 Verification Guide — Camera Depth → Local Costmap

更新: 2026-05-21

---

## アーキテクチャ変更まとめ（旧 → 現在）

| 項目 | 旧（LaserScan 方式） | 現在（PointCloud2 方式） |
|---|---|---|
| 変換ノード | `depthimage_to_laserscan` (手動起動) | `depth_image_proc::PointCloudXyzNode` (navigation.launch.py に内蔵) |
| 出力トピック | `/camera/scan` (LaserScan) | `/camera/depth/points` (PointCloud2) |
| costmap layer | `ObstacleLayer` | `VoxelLayer` |
| 地面除去 | なし → 地面が障害物になる問題 | `min_obstacle_height: 0.05m` で自動除去 |
| 起動方法 | 別ターミナルで手動 | nav2 launch と同時に自動起動 |

変更済みファイル:
- `src/nav_pkg/config/nav2_params.yaml` — VoxelLayer 設定
- `src/nav_pkg/launch/navigation.launch.py` — depth_image_proc ノード追加

---

## 現在の状態

| コンポーネント | 状態 | 備考 |
|---|---|---|
| Nav2 lifecycle | ✅ active | local/global costmap, bt_navigator 正常 |
| RealSense depth | ✅ 発行中 | `/camera/camera/depth/image_rect_raw` |
| camera_link TF | ✅ 正しい | translation [0.336, 0.018, 0.640], pitch 42° |
| `depth_image_proc` | ✅ navigation.launch に内蔵 | nav2 起動と同時に自動起動 |
| `/camera/depth/points` | ⬜ 要確認 | nav2 起動後に発行されるはず |
| local_costmap | ⬜ 要確認 | VoxelLayer で `/camera/depth/points` 待ち |

---

## 進め方

```
Step 1  ✅ 確認済み: RealSense depth 発行 OK
Step 2  ✅ 確認済み: camera_link TF OK
Step 3  ← 今ここ: /camera/depth/points の確認（nav2 起動後に自動発行）
Step 4  Step 3 通過後: VoxelLayer local costmap 反映確認
Step 5  Step 4 通過後: marking / clearing 動作確認
Step 6  Step 5 通過後: 実機 AUTO モード障害物回避確認
```

---

## Step 3: /camera/depth/points を確認

`depth_image_proc::PointCloudXyzNode` は **navigation.launch.py の中に含まれている**。  
Nav2 を起動すれば自動的に立ち上がる。

### 3-A: Nav2 を起動する（motors なし）

```bash
tools/open_nav2_windows.sh --phase1-safe --cleanup
```

### 3-B: /camera/depth/points が出ているか確認

```bash
# 発行されているか
ros2 topic list | grep depth/points
# 期待: /camera/depth/points

# レート確認 (25-30 Hz が正常)
ros2 topic hz /camera/depth/points

# frame_id 確認
ros2 topic echo /camera/depth/points --once --no-arr | grep frame_id
# 期待: frame_id: "camera_depth_optical_frame"

# local_costmap が subscriber になっているか
ros2 topic info -v /camera/depth/points | grep "Node name"
# 期待: Node name: local_costmap
```

**出ていない場合:**

```bash
# depth_proc_container ノードが起動しているか確認
ros2 node list | grep depth_proc
# 期待: /depth_proc_container

# ノードが見えない場合 → colcon build が必要
cd /home/matsunaga-h/robot_ws
colcon build --packages-select nav_pkg --symlink-install
source install/setup.bash
# その後 Nav2 を再起動
```

### 3-C: RViz で /camera/depth/points を視覚確認

```bash
rviz2
```

**RViz の設定（重要: Fixed Frame は `odom` を使う）:**

```
Global Options
  Fixed Frame: odom        ← base_link ではなく odom
                             (VoxelLayer は odom 基準で高さフィルタするため)

Add → By topic → /camera/depth/points → PointCloud2
  Color Transformer: AxisColor
  Axis:              Z        ← 高さで色分け
  Min Color:         青
  Max Color:         赤
  Size (m):          0.02

Add → RobotModel
  Description Topic: /robot_description

Add → TF
```

**Z 軸色分けの見方:**

```
青（z ≈ 0）    : 地面付近の点
緑（z ≈ 0.5m） : 膝・机面の高さ
赤（z ≈ 1.0m+）: 腰・棚の高さ
```

**確認すること（障害物なし）:**

- 点群が前方扇形状に広がること
- 地面の点（青）が多く出ること
- 青の点は min_obstacle_height: 0.05m でフィルタされるので、costmap には入らない

**確認すること（障害物あり）:**

前方 **0.5m** に段ボール箱（高さ 30-50cm 程度）を置く:

```
期待:
- RViz PointCloud2 に箱の形状が緑〜赤の点で現れる
- 地面（青）の点は箱の周囲に残る
- 箱の高さが 0.05m〜1.20m の範囲なら costmap に入る（Step 4 で確認）
```

**点群が出ない / 全て真っ暗な場合:**

```bash
# depth image の値を確認
ros2 topic echo /camera/camera/depth/image_rect_raw --once --no-arr | head -10
# encoding: 16UC1 (正常)
# step: 1280 (640 * 2 bytes = 正常)

# camera_info が来ているか
ros2 topic hz /camera/camera/depth/camera_info
# 期待: 30 Hz
```

---

## Step 4: VoxelLayer local costmap の反映確認

**前提: Step 3 で /camera/depth/points が 25+ Hz で出ていること。**

### 4-A: VoxelLayer が動いているか確認

```bash
# VoxelLayer パラメータ確認
ros2 param get /local_costmap local_costmap.voxel_layer.observation_sources
# 期待: "camera_points"

ros2 param get /local_costmap local_costmap.voxel_layer.camera_points.min_obstacle_height
# 期待: 0.05

ros2 param get /local_costmap local_costmap.voxel_layer.camera_points.max_obstacle_height
# 期待: 1.2

# local costmap のレート
ros2 topic hz /local_costmap/costmap
# 期待: ~5 Hz
```

### 4-B: RViz で local costmap を確認

**RViz の設定（Step 3 の PointCloud2 に追加）:**

```
Add → By topic → /local_costmap/costmap → Map
  Color Scheme: costmap
  Alpha:        0.7
  (灰色=低コスト、赤=高コスト=障害物エリア、黒=未知)

Add → By topic → /local_costmap/published_footprint → Polygon
  (robot footprint の確認用)
```

**Fixed Frame は `odom` のまま維持すること。**  
`base_link` にすると costmap が常にロボットに追従して見えるため、  
障害物が本当に世界座標に固定されているか判別できない。

**障害物検出の確認手順:**

```
1. 障害物なし
   → local costmap がほぼクリア（灰色）であること
   → 地面が障害物として表示されていないこと ← Phase 2 改善の肝

2. 前方 0.5m に段ボール箱（高さ 30cm 以上）を置く
   → 3-5 秒以内に costmap の対応位置に赤いセルが出ること
   → inflation（膨張、ピンク）が赤セル周囲に広がること

3. 1.0m、1.5m でも同様に確認

4. 箱を取り除く
   → 数秒以内にセルが消えること（clearing）
```

**障害物が costmap に出ない場合のデバッグ:**

```bash
# PointCloud2 の点群が costmap height range 内にあるか確認
# → RViz の AxisColor 表示で z=0.05〜1.20m の緑〜赤点が箱部分に出ているか目視

# height filter が厳しすぎる場合は一時的に範囲を広げてテスト
ros2 param set /local_costmap local_costmap.voxel_layer.camera_points.min_obstacle_height 0.02
ros2 param set /local_costmap local_costmap.voxel_layer.camera_points.max_obstacle_height 1.50
# ← テスト後は元の値に戻すこと（yaml は変わらないので再起動で戻る）
```

**costmap の位置が実際の障害物とずれる場合:**

```bash
# TF を確認（VoxelLayer は odom フレームで点を変換する）
ros2 run tf2_ros tf2_echo odom camera_depth_optical_frame
# translation と rotation に大きなエラーがないこと

# RViz で PointCloud2 と costmap を同時表示して位置比較
# → PointCloud2 の箱の形 ≈ costmap の赤セルの位置  であれば TF は正しい
# → 回転方向にずれる場合 → camera の yaw オフセット（URDF camera_y を確認）
```

---

## Step 5: marking / clearing の確認

```bash
# costmap update stream を確認
ros2 topic hz /local_costmap/costmap_updates
# 障害物を置いた直後に Hz が上がること（更新が来ること）

# clearing の確認
# 1. 前方 0.5m に障害物を置く → costmap に赤セルが出るのを確認
# 2. 障害物を素早く取り除く
# 3. 5秒以内に赤セルが消えることを確認
```

**clearing が起きない場合:**

```bash
# raytrace が機能しているか確認
ros2 param get /local_costmap local_costmap.voxel_layer.camera_points.clearing
# 期待: True

ros2 param get /local_costmap local_costmap.voxel_layer.camera_points.raytrace_max_range
# 期待: 2.5
# raytrace_max_range < 障害物距離 の場合、clearing されない
```

---

## Step 6: 実機 AUTO モードで障害物回避確認

**前提: Step 1-5 が全て通っていること。**

```bash
# モーターありでフルスタック起動
tools/open_nav2_windows.sh --cleanup

# 起動後まず確認（safety check）
ros2 topic info -v /cmd_vel | grep "Node name"
# behavior_server が現れないこと
# nav_mode_switch_node のみ publisher であること

ros2 topic echo /robot_mode --once
# MANUAL であること（まだ AUTO にしない）

# PointCloud2 が出ているか
ros2 topic hz /camera/depth/points
# 25+ Hz であること
```

**テスト手順（低速・短距離・人工急停必須）:**

1. RViz (Fixed Frame: map) で前方 1.5m に 2D Goal Pose を送る
2. A ボタンで AUTO
3. ロボットが動き始めたら前方 0.8m に障害物を置く
4. costmap に障害物が出ること、ロボットが減速・回避・停止することを確認
5. **B ボタンはいつでも押せるよう手に持っておく**

---

## 高さフィルタパラメータの調整指針

現在の設定: `min_obstacle_height: 0.05m` / `max_obstacle_height: 1.20m`

```
検出できる障害物の高さ範囲（地面基準）:
  0.05m 〜 1.20m  ← この範囲の点が costmap に入る

除外されるもの:
  z < 0.05m: 地面・床面（今回の問題の根本解決）
  z > 1.20m: Livox マスト・天井・高い棚など
```

調整の目安:

| 調整したいこと | 変更するパラメータ |
|---|---|
| 低い障害物（靴、荷物など）も検出したい | `min_obstacle_height` を 0.03 に下げる |
| 地面ノイズが出る | `min_obstacle_height` を 0.08 に上げる |
| 棚の上部も検出したい | `max_obstacle_height` を 1.50 に上げる |
| Livox マストが映り込む | `max_obstacle_height` を 1.00 に下げる |

**パラメータを一時変更してテスト:**

```bash
ros2 param set /local_costmap \
  local_costmap.voxel_layer.camera_points.min_obstacle_height 0.03
# テスト後は nav2 を再起動して yaml の値に戻す
```

**yaml を永続変更する場合:**

```bash
# nav2_params.yaml を編集してから nav2 を再起動
nano /home/matsunaga-h/robot_ws/src/nav_pkg/config/nav2_params.yaml
# voxel_layer.camera_points.min_obstacle_height の値を変更
tools/open_nav2_windows.sh --phase1-safe --cleanup
```

---

## Phase 5 への引き継ぎ事項

現在の構成:

```
depth image → depth_image_proc::PointCloudXyzNode → /camera/depth/points (XYZ)
```

Phase 5（semantic 認識）では XYZRGB が必要になる。切り替え手順:

1. `robot_bringup/launch/test_all.launch.py` に追加:
   ```python
   'pointcloud.enable': 'true',
   ```
2. `nav2_params.yaml` の topic を変更:
   ```yaml
   topic: /camera/camera/depth/color/points
   ```
3. `navigation.launch.py` の `depth_to_pointcloud` コンテナを削除

詳細: `software-info/ros-network-setup.md` の「Phase 5 以降の camera PointCloud2 設定変更予定」を参照。

---

## チェックリスト（現状反映版）

### Step 1-2: 確認済み ✅
- [x] `/camera/camera/depth/image_rect_raw` 発行中 (30 Hz)
- [x] `camera_link` TF: translation [0.336, 0.018, 0.640], pitch 42°

### Step 3: /camera/depth/points 確認
- [ ] Nav2 起動後に `/camera/depth/points` が 25+ Hz で発行される
- [ ] `ros2 topic info` に `local_costmap` が subscriber として現れる
- [ ] RViz PointCloud2 (AxisColor Z) で前方の点群が高さ色分けで表示される
- [ ] 障害物を置くと緑〜赤の点が箱形状で現れる

### Step 4: VoxelLayer local costmap 反映
- [ ] costmap が起動直後にクリア状態（地面が障害物として出ていない）
- [ ] 前方 0.5m の箱が costmap の正しい位置に赤セルとして出る
- [ ] 1.0m、1.5m でも位置が正しい

### Step 5: clearing
- [ ] 障害物を取り除くと 5 秒以内に costmap からセルが消える

### Step 6: 実機 AUTO
- [ ] `/camera/depth/points` が 25+ Hz で継続発行されている
- [ ] Nav2 goal 送信後 plan が生成される
- [ ] 障害物前で減速・停止または回避する
- [ ] B ボタンで即 MANUAL に戻れる
