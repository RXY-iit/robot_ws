# ROS2 Network Setup — Lab Environment

作成: 2026-05-21

---

## この PC の設定

| 項目 | 値 |
|---|---|
| ユーザー | matsunaga-h |
| `ROS_DOMAIN_ID` | **13** |
| 設定場所 | `~/.bashrc` (末尾) |

```bash
# ~/.bashrc 末尾に設定済み
export ROS_DOMAIN_ID=13
```

設定確認:

```bash
echo $ROS_DOMAIN_ID   # → 13
```

新しいターミナルを開いた時点で自動的に有効になる。  
今開いているターミナルに即時反映する場合:

```bash
source ~/.bashrc
```

---

## ROS_DOMAIN_ID とは

ROS2 は DDS (Data Distribution Service) でノード探索に UDP マルチキャストを使う。  
同一ネットワーク内で `ROS_DOMAIN_ID` が同じ全ノードが互いに見え合う。

```
DOMAIN_ID が同じ → 同じ ROS2 グラフに属する（topic が共有される）
DOMAIN_ID が違う → 完全に分離（互いの topic は見えない）
```

有効範囲: 0-101（デフォルト: 0）  
研究室全員が 0 のままだと全員のデータが混在する。

---

## 研究室での推奨ルール

| ユーザー / ロボット | ROS_DOMAIN_ID |
|---|---|
| matsunaga-h (このPC) | **13** |
| *(他の人は別番号を割り当て)* | 14, 15, 16 … |
| rosbag 単独再生 (throwaway) | **99** |

研究室内で番号を決めて共有しておくこと。

---

## rosbag 再生時の干渉防止

### 問題

同じカメラ型番の rosbag を同じ DOMAIN_ID で play すると、  
ネットワーク上の全 subscriber がハードウェアと rosbag 両方のデータを受信して混在する。

### 解決策 A: 専用 throwaway DOMAIN_ID で play（推奨）

```bash
ROS_DOMAIN_ID=99 ros2 bag play recording.bag
```

この方法なら自分の DOMAIN_ID=13 の環境に一切影響しない。

### 解決策 B: topic に prefix をつけて play

```bash
ros2 bag play recording.bag \
  --remap /camera/camera/depth/image_rect_raw:=/replay/camera/depth/image_rect_raw \
  --remap /camera/camera/color/image_raw:=/replay/camera/color/image_raw
```

replay データを自分のパイプラインに接続しながらハードウェアと分離したい場合に使う。

### 解決策 C: ローカルホストのみ（ロボットと同一PC の場合）

```bash
export ROS_LOCALHOST_ONLY=1
```

**注意: ロボットとPCが別マシンでネットワーク通信している場合はこれを設定しないこと。**  
設定すると相手のマシンのノードが見えなくなる。

---

## Phase 5 以降の camera PointCloud2 設定変更予定

現在（Phase 2）は `depth_image_proc::PointCloudXyzNode` で XYZ のみの PointCloud2 を生成している。

```
現在: depth image → depth_image_proc → /camera/depth/points (XYZ)
```

Phase 5 で semantic 認識（RGB+depth）が必要になった場合、RealSense ドライバ側で  
XYZRGB pointcloud を有効化する方法に切り替える。

```
将来: depth+color → RealSense SDK (pointcloud.enable:=true) → /camera/camera/depth/color/points (XYZRGB)
```

切り替え手順（Phase 5 時):

1. `robot_bringup/launch/test_all.launch.py` の RealSense 起動引数に追加:

```python
'pointcloud.enable': 'true',
```

2. `nav_pkg/config/nav2_params.yaml` の topic を変更:

```yaml
camera_points:
  topic: /camera/camera/depth/color/points   # /camera/depth/points から変更
```

3. `navigation.launch.py` の `depth_to_pointcloud` コンテナを削除（ドライバ側で生成するため不要）

注意: `pointcloud.enable: true` は CPU と帯域を多く使う。  
depth stream の frame rate に影響する可能性があるため、Phase 5 開始時に実測して確認すること。

---

## 確認コマンド早見表

```bash
# 現在の DOMAIN_ID を確認
echo $ROS_DOMAIN_ID

# 同じ DOMAIN_ID のノード一覧
ros2 node list

# /camera/depth/points が発行されているか
ros2 topic hz /camera/depth/points

# rosbag を分離した DOMAIN_ID で再生
ROS_DOMAIN_ID=99 ros2 bag play <bagfile>

# 特定の DOMAIN_ID だけ一時的に使う（現在のシェルのみ）
ROS_DOMAIN_ID=99 ros2 topic list
```
