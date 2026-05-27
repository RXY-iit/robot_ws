# Isaac Sim / Real Robot 環境構築 Guideline

> 作成: 2026-05-22
> 目的: Isaac Sim 仿真環境を構築し、真機と同一の制御システムで動作させる。
>       将来の各種テスト（Nav2, Mission Manager, AI Planner 等）を仮想環境で先行実施する。
> 前提: 真機は Nav2 + FAST-LIO2/GICP で動作確認済み。

---

## 1. 目標

### 最終的に実現したいこと

```
Isaac Sim 上のロボットが、真機とまったく同じ制御ソフトウェアで動く。
起動時のパラメータを切り替えるだけで sim / real を切り替えられる。
```

具体的には:
- Isaac Sim が仮想 LiDAR / Camera / IMU / Odom を発行する
- 制御システム（Nav2 / GICP / safety_layer）は topic を受け取るだけで、接続先が仮想か実機かを知らない
- Isaac Sim 上で Nav2 goal を送り、ロボットが自律移動する
- 将来: Mission Manager, AI Planner, 各種テストシナリオを仮想環境で先行検証する

### 対象外

- Isaac Sim で真機の物理挙動を高精度に再現すること（デジタルツインは目標外）
- 仿真 → 真機への自動パラメータ転送（Sim-to-Real transfer）

---

## 2. 現在の環境

### 2.1 真機システム構成

```
真機 PC（Ubuntu 22.04 / ROS2 Humble）
  ROS_DOMAIN_ID = 13

  起動中のノード:
    robot_state_publisher    URDF → TF
    livox_ros_driver2        Mid360 → /livox/lidar, /livox/imu
    realsense2_camera        → /camera/camera/color/image_raw, depth
    omni_base_driver         /cmd_vel → 逆運動学 → Dynamixel + DC motor
    robot_odom_node          wheel encoder → /wheel_odom + odom→base_footprint TF
    gicp_localizer_node      /livox/lidar_pc2 → map→odom TF
    fast_lio                 /livox/lidar(CustomMsg) + /livox/imu → odometry hint
    nav2（nav_pkg）          /scan, /wheel_odom → /nav2/cmd_vel
    cmd_vel_safety_node      /cmd_vel_raw → /cmd_vel（watchdog + 速度制限）
```

### 2.2 確定済み TF チェーン

```
map → odom                 gicp_localizer_node（FAST-LIO2 hint + GICP）
odom → base_footprint      robot_odom_node（wheel encoder）
base_footprint → base_link URDF 固定（+0.1125 m Z）
base_link → livox_frame    URDF（x=-0.24 m, z=1.38 m, pitch=30°）
base_link → camera_link    URDF（x=+0.32 m, z=0.638 m, pitch=42°）
```

### 2.3 制御システムの核心 Topic

| Topic | 型 | 方向 | 備考 |
|---|---|---|---|
| `/livox/lidar` | CustomMsg | 入力 | FAST-LIO2 入力（real と同じ標準入力） |
| `/livox/lidar_pc2` | PointCloud2 | 内部 | relay 後の GICP / scan / RViz 入力 |
| `/livox/imu` | Imu | 入力 | FAST-LIO2 入力 |
| `/scan` | LaserScan | 内部 | pointcloud_to_laserscan 変換後 |
| `/camera/camera/color/image_raw` | Image | 入力 | RGB |
| `/camera/camera/depth/image_rect_raw` | Image | 入力 | Depth |
| `/wheel_odom` | Odometry | 入力 | Nav2 odom topic |
| `/cmd_vel` | Twist | 出力 | 最終駆動指令 |

### 2.4 パッケージ分類（現在 + Sim 導入後）

| パッケージ | 真機 | Sim | 分類 |
|---|---|---|---|
| `robot_description` | ✅ | ✅ | 共用 |
| `my_messages` | ✅ | ✅ | 共用 |
| `tf_tools` | ✅ | ✅ | 共用 |
| `localization_pkg` | ✅ | ✅ | 制御層 |
| `nav_pkg` | ✅ | ✅ | 制御層 |
| `safety_layer` | ✅ | ✅ | 制御層 |
| `mission_manager` | ✅ | ✅ | 制御層 |
| `fast_lio` | ✅ | ✅ | 制御層 |
| `livox_ros_driver2` | ✅ | ✅ | 真機 driver + sim CustomMsg 型依存 |
| `serial_transciever` | ✅ | ❌ | 真機専用 |
| `omni_base_driver` | ✅ | ❌ | 真機専用 |
| `robot_bringup` | ✅ | 一部✅ | Joy-Con/teleop 設定は流用、hardware driver launch は使わない |
| `robot_sim` | ❌ | ✅ | **新規（Sim 専用）** |

---

## 3. 全体アーキテクチャ方針

### 3.1 基本思想

**Isaac Sim を「仮想ハードウェア層」と捉え、制御ソフトウェアには一切手を加えない。**

```
             ┌─────────────────┐       ┌──────────────────────┐
             │    真機 PC       │       │   Isaac Sim PC        │
             │  DOMAIN_ID=13   │       │   DOMAIN_ID=20        │
             │                 │       │                        │
             │  真機ハードウェア  │       │  Isaac Sim             │
             │  + 制御システム   │       │  + 制御システム (同一)   │
             └─────────────────┘       └──────────────────────┘
                     ↑                           ↑
             DDS discovery が                DDS discovery が
             お互いに見えない                  お互いに見えない
             （ROS_DOMAIN_ID 隔離）           （ROS_DOMAIN_ID 隔離）
```

**制御システムは「/livox/lidar, /livox/imu, /wheel_odom, /cmd_vel という真機と同名 topic」だけを見る。**
その topic がどこから来てどこへ行くかは、ハードウェア層（真機 / Isaac Sim）の責任。

### 3.2 隔離方式の決定

3 種類の隔離方式を比較し、採用する方式を決定する。

| 方式 | 概要 | 利点 | 欠点 |
|---|---|---|---|
| **A: ROS_DOMAIN_ID のみ** | 真機=13, Sim=20 で完全分離 | Topic 名変更不要、実装コスト最小 | 真機と Sim を同時観察できない |
| B: Namespace のみ | `/sim/...` vs `/real/...` | 同一ネットワークで両方見える | 全 launch 改修が必要（30+箇所） |
| C: A + B 組み合わせ | DOMAIN_ID 分離 + Namespace | 最も安全、柔軟 | 実装コスト最大 |

**採用: 方式 A（Phase 1〜5）**

理由:
- 現在の launch ファイルに topic 名のハードコードが多数あり、namespace 変更は工数大
- 真機と Sim を**同時に動かす必要は当面ない**
- ROS_DOMAIN_ID 分離で安全性は十分確保できる

**将来（Phase 6 以降）:** 必要に応じて方式 C に移行する。

```bash
# 真機 PC での設定
export ROS_DOMAIN_ID=13

# Isaac Sim PC での設定
export ROS_DOMAIN_ID=20
```

---

## 4. Docker 方針

### 4.1 なぜ Docker か

| 問題 | Docker なし | Docker あり |
|---|---|---|
| ROS2 バージョン差異 | 手動管理 | image で固定 |
| 依存 apt/pip パッケージ | 手動管理 | Dockerfile で再現 |
| 真機 PC への影響リスク | 環境が混在する | 完全分離 |
| Isaac Sim PC での環境再構築 | 毎回 `rosdep install` 等 | `docker compose up` だけ |

### 4.2 Docker 化の対象と範囲

**Docker 化する: 制御システム（共用層 + 制御層）**

```
robot_description / my_messages / tf_tools
localization_pkg / nav_pkg / safety_layer / mission_manager
fast_lio / livox_ros_driver2（CustomMsg 型依存）
```

**Docker 化しない:**
- Isaac Sim 本体（NVIDIA 公式 image を使用または GPU ドライバとの相性で native 推奨）
- 真機 PC 上の hardware driver（既存環境を保護）

### 4.3 Docker 化のタイミング

```
Phase 1-4: Docker なし（Joy-Con手動、map作成、FAST-LIO2+GICP、Nav2確認を優先）
Phase 5: 動作確認後に Dockerfile / docker-compose.sim.yml を作成・テスト
```

理由: Docker と Isaac Sim の問題が重なると切り分けが困難になる。
まず手動で動かし、問題を整理してから Docker 化する。

### 4.4 ディレクトリ構成（Docker 導入後）

```
robot_ws/
  docker/
    Dockerfile.control          # 制御システム用 image
    docker-compose.sim.yml      # Isaac Sim PC 用
    docker-compose.real.yml     # 将来: 真機 PC 用（現時点は不要）
    entrypoint.sh
  src/
    ... (既存パッケージ)
    robot_sim/                  # 新規: Isaac Sim 専用
      launch/
      scripts/
      usd/
```

### 4.5 Dockerfile.control（制御システム）

```dockerfile
FROM ros:humble

# ROS2 依存パッケージ
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-depth-image-proc \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# workspace をコピー
WORKDIR /robot_ws
COPY src/ src/

# 制御層のみビルド（hardware driver は除外）
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
    --packages-select \
        robot_description my_messages tf_tools \
        localization_pkg nav_pkg safety_layer mission_manager \
        fast_lio livox_ros_driver2 \
    2>&1"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

### 4.6 docker-compose.sim.yml

```yaml
version: "3.8"
services:
  control:
    image: robot_control:sim
    build:
      context: ..
      dockerfile: docker/Dockerfile.control
    network_mode: host          # ROS2 DDS は host network が必要
    environment:
      - ROS_DOMAIN_ID=20
      - RMW_IMPLEMENTATION=rmw_cyclonedx_cpp  # DDS 実装（Isaac Sim と合わせる）
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix         # RViz 用（オプション）
      - ../maps:/robot_ws/maps                # 地図ファイル共有
    command: >
      bash -c "source install/setup.bash &&
               ros2 launch robot_sim sim_bringup.launch.py"
```

---

## 5. Isaac Sim 構成

### 5.1 ロボット USD の作成

真機の URDF をそのまま使用する。

```bash
# URDF を展開
xacro ~/robot_ws/src/robot_description/urdf/robot.urdf.xacro \
    > /tmp/robot_expanded.urdf

# Isaac Sim GUI: File → Import URDF → /tmp/robot_expanded.urdf
# → USD として保存: robot_ws/src/robot_sim/usd/robot_sim.usd
```

Physics Articulation として定義する Joint:

| Joint 名 | 型 | 制御モード |
|---|---|---|
| `wheel_right_steer` | RevoluteJoint | 位置制御（steering 角度） |
| `wheel_left_steer` | RevoluteJoint | 位置制御（steering 角度） |
| `wheel_back_steer` | RevoluteJoint | 位置制御（steering 角度） |
| `wheel_right_drive` | RevoluteJoint | 速度制御（駆動速度） |
| `wheel_left_drive` | RevoluteJoint | 速度制御（駆動速度） |
| `wheel_back_drive` | RevoluteJoint | 速度制御（駆動速度） |

### 5.2 ROS2 Bridge 発行 Topic（真機と同名）

Isaac Sim ROS2 Bridge で以下の topic を設定する。
**topic 名は真機と完全に一致させる**（ROS_DOMAIN_ID で隔離するため）。

| センサー | Topic | 型 | frame_id | 備考 |
|---|---|---|---|---|
| RTX LiDAR | `/livox/lidar` | CustomMsg | `livox_frame` | FAST-LIO2 入力。Isaac Sim が直接出せない場合は adapter で作る |
| RTX LiDAR relay | `/livox/lidar_pc2` | PointCloud2 | `livox_frame` | real と同じく GICP / `/scan` / RViz 用 |
| IMU | `/livox/imu` | Imu | `livox_frame` | FAST-LIO2 入力 |
| RGB Camera | `/camera/camera/color/image_raw` | Image | `camera_color_optical_frame` | |
| RGB Camera | `/camera/camera/color/camera_info` | CameraInfo | `camera_color_optical_frame` | |
| Depth Camera | `/camera/camera/depth/image_rect_raw` | Image | `camera_depth_optical_frame` | |
| Depth Camera | `/camera/camera/depth/camera_info` | CameraInfo | `camera_depth_optical_frame` | |
| Physics Odom | `/wheel_odom` | Odometry | `odom` → child: `base_footprint` | Nav2 odom topic |
| Physics TF | `/tf` | TFMessage | `odom` → `base_footprint` | real の robot_odom_node と同じ TF edge |
| Clock | `/clock` | Clock | — | `use_sim_time` 対応 |

### 5.3 全向底盤 逆運動学スクリプト

Isaac Sim 内で `/cmd_vel` を受信し、3つの操舵角と3つの駆動速度に変換する。
真機の `picking_robot_matrix.hpp` / `motor_param.hpp` と**まったく同じロジック**を Python で実装する。

ファイル: `src/robot_sim/scripts/omni_drive_controller.py`

```python
"""
Isaac Sim 全向底盘 逆運動学コントローラ
真機 picking_robot_matrix.hpp の calcWheelVelAng() を移植。

/cmd_vel (vx, vy, vth) を受信し、
3輪の steering 角度と drive 速度を計算して
Isaac Sim Articulation の各 joint を駆動する。
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── ホイールパラメータ（motor_param.hpp より）────────────────────
# ロボット中心から各車輪までの距離 [m]
WHEEL_DIST = [0.32330, 0.32330, 0.38500]

# ロボット中心から見た各車輪の位置角 [rad]
# wheel[0]: 右前 (-50.6°), wheel[1]: 左前 (+50.6°), wheel[2]: 後 (180°)
WHEEL_PHI  = [-0.883978, 0.883978, np.pi]

# 各車輪の直交座標
r_x = [WHEEL_DIST[i] * np.cos(WHEEL_PHI[i]) for i in range(3)]
r_y = [WHEEL_DIST[i] * np.sin(WHEEL_PHI[i]) for i in range(3)]

# 運動学行列 R (6×3)
R = np.array([
    [1.0, 0.0, -r_y[0]],
    [0.0, 1.0,  r_x[0]],
    [1.0, 0.0, -r_y[1]],
    [0.0, 1.0,  r_x[1]],
    [1.0, 0.0, -r_y[2]],
    [0.0, 1.0,  r_x[2]],
])

# ── 逆運動学計算 ─────────────────────────────────────────────────
def cmd_vel_to_wheels(vx: float, vy: float, vth: float):
    """
    /cmd_vel → (drive_vel[3], steer_phi[3])
    真機 calcWheelVelAng() と同一ロジック。

    Returns:
        drive_vel: 各車輪の駆動速度 [m/s]（符号あり）
        steer_phi: 各車輪の操舵角 [rad]
    """
    V = np.array([[vx], [vy], [vth]])
    V_wheel = R @ V  # shape (6, 1)

    drive_vel = []
    steer_phi = []
    for i in range(3):
        vx_i = V_wheel[i * 2,     0]
        vy_i = V_wheel[i * 2 + 1, 0]
        speed = np.sqrt(vx_i**2 + vy_i**2)
        angle = np.arctan2(vy_i, vx_i)

        # π フリップ（真機と同じ判定: |angle| > 0.71π）
        if angle < -0.71 * np.pi:
            angle += np.pi
            speed  = -speed
        elif angle > 0.71 * np.pi:
            angle -= np.pi
            speed  = -speed

        drive_vel.append(speed)
        steer_phi.append(angle)

    return drive_vel, steer_phi


class OmniDriveController(Node):
    def __init__(self, articulation):
        super().__init__("omni_drive_controller")
        self._art = articulation  # Isaac Sim Articulation オブジェクト
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.get_logger().info("OmniDriveController ready.")

    def _cb(self, msg: Twist):
        drive_vel, steer_phi = cmd_vel_to_wheels(
            msg.linear.x, msg.linear.y, msg.angular.z
        )
        # Isaac Sim Articulation への適用（joint 名は USD に合わせて要調整）
        # self._art.set_joint_velocities(...)
        # self._art.set_joint_positions(...)
        self.get_logger().debug(f"drive={drive_vel}, steer={steer_phi}")
```

### 5.4 Isaac Sim PC 起動手順（手動・Docker なし）

最初の確認は **Nav2 goal ではなく Joy-Con 手動走行**。
真機と同じ操作系を使い、sim を「もう一台の real robot」として扱う。

```bash
# ── Terminal 1: Isaac Sim ──────────────────────────────────────
# Isaac Sim GUI を起動
# File → Open → robot_sim/usd/robot_sim.usd
# Window → Extensions → ROS2 Bridge を有効化
# Play ボタン

# ── Terminal 2: 制御システム ───────────────────────────────────
export ROS_DOMAIN_ID=20
source ~/robot_ws/install/setup.bash

# robot_state_publisher のみ起動（hardware driver は起動しない）
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(xacro src/robot_description/urdf/robot.urdf.xacro)"

# ── Terminal 3: Joy-Con / mode switch / safety ─────────────────
export ROS_DOMAIN_ID=20
source ~/robot_ws/install/setup.bash

# 真機と同じ /joy, /teleop/cmd_vel, /cmd_vel_raw, /cmd_vel chain を使う。
# B/MANUAL, A/AUTO, Y/E-STOP のボタン配置も real と同じにする。
# 実装時は robot_sim 側 launch から teleop + nav_mode_switch + safety を起動する。

# ── Terminal 4: FAST-LIO2 + GICP localization（地図作成後）────
export ROS_DOMAIN_ID=20
source ~/robot_ws/install/setup.bash

ros2 launch localization_pkg fast_lio_localization.launch.py \
    with_fast_lio:=true \
    use_fast_lio_hint:=true \
    use_sim_time:=false

# ── Terminal 5: navigation（localization 確認後）──────────────
export ROS_DOMAIN_ID=20
source ~/robot_ws/install/setup.bash

ros2 launch nav_pkg navigation.launch.py
# ※ use_sim_time 問題が出た場合は下記「注意事項 6」を参照

# ── Terminal 6: RViz ───────────────────────────────────────────
export ROS_DOMAIN_ID=20
rviz2 -d ~/robot_ws/rviz/nav2_navigation.rviz
```

---

## 6. use_sim_time 問題と対処方針

### 方針

まずは変更なし、`use_sim_time:=false` のまま試す。
Isaac Sim 側が PC の realtime stamp で LiDAR / IMU / odom / TF を publish できれば、
制御システムから見ると「もう一台の real robot」と同じ扱いにできる。

確認方法:

```bash
# stamp が現在時刻に近いこと
ros2 topic echo /tf --once
ros2 topic echo /livox/imu --once
ros2 topic echo /wheel_odom --once

# TF が連続して取れること
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link livox_frame
```

### 注意

real 開発時に起きた問題と同じく、timestamp が混ざると Nav2 controller が止まる。
以下が連続して出る場合だけ `use_sim_time:=true` 対応を検討する:

```text
Lookup would require extrapolation into the future
timestamp on the message is earlier than all the data in the transform cache
TF_OLD_DATA
```

その場合も `navigation.launch.py` を丸ごとコピーした wrapper を作るのではなく、
real と同じ node chain（nav_mode_switch_node / cmd_vel_safety_node / depth_image_proc も含む）を保ったまま
`use_sim_time` launch argument を追加する。

---

## 7. 注意事項（実施前に把握すべきこと）

### 7.1 FAST-LIO2 は sim でも標準構成として使う

| | 真機 | Isaac Sim |
|---|---|---|
| LiDAR 出力形式 | CustomMsg (xfer_format=1) | CustomMsg に合わせる |
| FAST-LIO2 入力要求 | CustomMsg | CustomMsg が必要 |
| FAST-LIO2 動作 | ✅ | adapter / custom publisher ができれば ✅ |
| GICP 入力 | `/livox/lidar_pc2` | `/livox/lidar_pc2` |

方針: sim でも real と同じく `with_fast_lio:=true` を標準にする。

重要な前提:

- FAST-LIO2 は `/livox/lidar` の `livox_ros_driver2/msg/CustomMsg` を読む。
- GICP / `/scan` / RViz は `/livox/lidar_pc2` の `PointCloud2` を使う。
- real では C++ relay が `/livox/lidar` CustomMsg → `/livox/lidar_pc2` PointCloud2 を作る。
- Isaac Sim が PointCloud2 しか出せない場合、`robot_sim` 側で PointCloud2 → Livox CustomMsg adapter を用意する。

CustomMsg adapter で特に注意すること:

- `header.stamp` を IMU と同じ時間軸にする。
- `header.frame_id` は `livox_frame`。
- 各点の `offset_time` を可能な範囲で設定する。難しい場合はまず 0 固定で起動確認し、FAST-LIO の安定性を見る。
- `line` / `tag` / `reflectivity` など Livox 固有 field は FAST-LIO が落ちない値で埋める。
- `/livox/imu` と `/livox/lidar` の周波数・timestamp が大きくずれると FAST-LIO は不安定になる。

### 7.2 全向底盘の逆運動学は自前実装が必要

Isaac Sim に 3輪独立転向+3輪独立駆動のコントローラは存在しない。
`omni_drive_controller.py`（Section 5.3）を OmniGraph の Python Script ノードに接続して使う。

動作確認の手順:
1. Joy-Con で real と同じ teleop chain を使い、最終 `/cmd_vel` でロボットが動くことを確認
2. 操舵角の符号・方向が真機と一致するか確認（必要なら sign を反転）
3. 速度スケールを調整（Isaac Sim の joint velocity と m/s の変換係数）
4. `/wheel_odom` と `odom -> base_footprint` TF が実際の移動方向と一致することを確認

### 7.3 GICP に仮想環境用の地図が必要

真機で使っている PCD 地図（`maps/l402_glim_map_0503`）は実際の環境のスキャン結果。
Isaac Sim の仮想環境はこの地図と一致しないため、GICP は正常に動作しない。

方針:

- 最初に Joy-Con で sim robot を走らせ、sim 環境の map を作る。
- map 作成・2D map 生成の具体手順は後続 task で扱う。
- この guideline では real 開発で起きた注意点だけ記録する。

real 開発からの注意:

- real map と sim map を混ぜない。保存先も `maps/sim_*` のように分ける。
- bag / map / TF の時間軸を混ぜない。real bag と sim bag を同じ起動中に replay しない。
- `map -> odom` は最終的に GICP が publish する。static `map -> odom` と GICP を同時に出さない。
- GICP score が悪い場合、まず地図不一致・初期姿勢・LiDAR frame を疑う。
- `/livox/lidar_pc2` と保存 map の高さ・向き・スケールが一致しているか RViz で確認する。

### 7.4 TF の `frame_id` を統一する

Isaac Sim が発行する各 topic の `frame_id` を、真機と完全に一致させること。

確認リスト:
```
/livox/lidar.header.frame_id   = "livox_frame"    ✅
/livox/imu.header.frame_id     = "livox_frame"    ✅
/wheel_odom.header.frame_id    = "odom"           ✅
/wheel_odom.child_frame_id     = "base_footprint" ✅
/camera/*/header.frame_id      = "camera_*_optical_frame" ✅
```

frame_id がずれると TF lookup が失敗し Nav2 / GICP が動かない。

### 7.5 Isaac Sim の ROS2 Humble 互換性

Isaac Sim 5.x は Python 3.11 を使用しており、ROS2 Humble（Python 3.10）との間で
バージョン不一致が生じる場合がある。

対処:
- Isaac Sim 公式の ROS2 インストールガイドに従って環境を構築する
- ROS2 Bridge は Isaac Sim 専用の Python 環境で動作するため、system の ROS2 とは分離される
- Docker で制御システムを動かす場合は ROS2 Python 環境の衝突がなくなる（推奨理由）

### 7.6 真機 PC への影響防止

Isaac Sim PC で作業中も真機 PC は起動したままでよい。

ただし以下を守ること:
- Isaac Sim PC では `export ROS_DOMAIN_ID=20` を**必ず設定してから** ROS2 コマンドを実行する
- Isaac Sim PC から `ros2 topic list` に真機の topic が**見えないことを確認**してから作業開始する
  ```bash
  export ROS_DOMAIN_ID=20
  ros2 topic list
  # /livox/lidar などが出てきたら Isaac Sim が起動していない → 正常（空のはず）
  # 起動後は Isaac Sim の発行 topic のみ見えるはず
  ```
- `.bashrc` に `ROS_DOMAIN_ID=20` を**ハードコードしない**（将来真機 PC と同じ環境にする際に混乱する）

---

## 8. 実施フェーズと確認事項

### Phase 0: パッケージ準備（Isaac Sim 不要）

```
目標: Isaac Sim と接続する前に制御システム側の準備を完了する。
作業環境: Isaac Sim PC（または真機 PC でのテスト）
```

- [ ] `robot_sim` パッケージのディレクトリ骨格を作成
- [ ] `xacro robot.urdf.xacro > /tmp/robot_expanded.urdf` が成功することを確認
- [ ] `robot_state_publisher` 単体で起動し、TF が正しく発行されることを確認
- [ ] Isaac Sim の LiDAR 出力を real と同じ `/livox/lidar` CustomMsg に合わせる方法を決める
      - 直接 CustomMsg publish
      - または PointCloud2 → Livox CustomMsg adapter
- [ ] `/livox/lidar` CustomMsg → `/livox/lidar_pc2` relay が sim でも動くことを確認
- [ ] Isaac Sim PC に ROS2 Humble をインストール（または Docker 環境を準備）
- [ ] `colcon build` が成功することを確認（hardware driver パッケージは除外）

完了条件:
```bash
# Isaac Sim なしで以下が起動できること
export ROS_DOMAIN_ID=20
ros2 run robot_state_publisher robot_state_publisher ...
# localization は map 作成後に with_fast_lio:=true で確認する
```

---

### Phase 1: Joy-Con 手動走行（Isaac Sim 単独、真機 PC 不起動）

```
目標: real と同じ Joy-Con / teleop / mode switch / safety chain で Isaac Sim のロボットを動かす。
成功基準: Joy-Con 操作 → /cmd_vel → Isaac Sim robot が実際に移動する。
```

**Isaac Sim 側タスク:**
- [ ] URDF → USD インポート完了
- [ ] Physics Articulation（6 joint）を正しく定義
- [ ] OmniGraph Python Script: `omni_drive_controller.py` を接続
  - [ ] `/cmd_vel` を受信できることを確認
  - [ ] joint が動くことを確認
- [ ] ROS2 Bridge:
  - [ ] `/wheel_odom` (header=`odom`, child=`base_footprint`) 発行
  - [ ] `/tf` で `odom -> base_footprint` 発行
  - [ ] `/clock` 発行
  - [ ] `/camera/camera/color/image_raw` 発行
  - [ ] `/camera/camera/depth/image_rect_raw` 発行
  - [ ] `/livox/imu` 発行
  - [ ] `/livox/lidar` CustomMsg 発行、または adapter 入力 topic 発行

**制御システム側タスク（ROS_DOMAIN_ID=20）:**
- [ ] `robot_state_publisher` 起動
- [ ] Joy-Con / teleop を real と同じ設定で起動
- [ ] `nav_mode_switch_node` と `cmd_vel_safety_node` を real と同じ chain で起動
- [ ] MANUAL mode で `/teleop/cmd_vel -> /cmd_vel_raw -> /cmd_vel` が流れることを確認
- [ ] RViz 起動

**動作確認:**
```bash
# TF ツリーが完全であること
ros2 run tf2_tools view_frames

# 各 topic の周波数確認
ros2 topic hz /wheel_odom       # 期待: sim physics rate
ros2 topic hz /cmd_vel
ros2 topic echo /clock --once

# Joy-Con 操作中に /cmd_vel が出ることを確認
ros2 topic echo /cmd_vel
```

Phase 1 通過基準:
```
✅ Joy-Con が real と同じボタン配置で使える
✅ MANUAL mode で /cmd_vel が出る
✅ Isaac Sim 上でロボットが前後・左右・旋回する
✅ /wheel_odom と odom -> base_footprint TF が移動方向と一致する
✅ 真機 PC (DOMAIN_ID=13) から /cmd_vel が見えないことを確認
```

---

### Phase 2: Sim 環境 map 作成

```
目標: Joy-Con で sim robot を走らせ、FAST-LIO2 + GICP / Nav2 用の sim map を作る。
```

- [ ] 手動走行で sim 環境を一周する
- [ ] LiDAR / IMU / wheel odom / TF / camera を rosbag 記録する
- [ ] 後続 task で 3D map と Nav2 2D map を生成する
- [ ] real map と混ざらないように `maps/sim_*` に保存する

記録 topic の目安:

```bash
ros2 bag record \
  /livox/lidar \
  /livox/imu \
  /wheel_odom \
  /tf \
  /tf_static \
  /robot_description \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  -o sim_bag/sim_mapping_YYYYMMDD
```

注意:

- 具体的な map 作成手順はここでは固定しない。
- real 開発時と同じく、TF 時間軸・初期姿勢・frame_id のずれが最大のトラブル源。
- sim map 完成前に GICP の良否を判断しない。

---

### Phase 3: FAST-LIO2 + GICP localization

```
目標: sim でも real と同じ FAST-LIO2 + GICP localization を動かす。
```

- [ ] `/livox/lidar` CustomMsg が publish されることを確認
- [ ] `/livox/imu` が publish されることを確認
- [ ] `livox_custom_to_pc2_node` relay で `/livox/lidar_pc2` が publish されることを確認
- [ ] `fast_lio_localization.launch.py with_fast_lio:=true use_fast_lio_hint:=true` を起動
- [ ] sim 用 3D map を `pcd_map:=...` に指定
- [ ] sim 用 2D map を `map:=...` に指定
- [ ] GICP が `map -> odom` TF を publish することを確認
- [ ] `/scan` が publish されることを確認

確認コマンド:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/lidar_pc2
ros2 topic hz /fast_lio/odometry
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

### Phase 4: Nav2 navigation（real と同じシステム）

```
目標: FAST-LIO2 + GICP localization 上で、real と同じ Nav2 MPPI / safety / Joy-Con 操作で自律移動する。
```

- [ ] `navigation.launch.py` を real と同じ設定で起動
- [ ] Joy-Con の A/B/Y 操作が real と同じことを確認
- [ ] MANUAL で goal を置き、A で AUTO にして動作確認
- [ ] `/nav2/cmd_vel -> /cmd_vel_raw -> /cmd_vel` が流れることを確認
- [ ] `/local_costmap/costmap` と `/global_costmap/costmap` が表示されることを確認
- [ ] obstacle / recovery / MPPI trajectory を確認

---

### Phase 5: Docker 化

```
目標: Isaac Sim PC で docker compose up だけで制御システムが起動する。
```

- [ ] `Dockerfile.control` を作成・build
- [ ] `docker-compose.sim.yml` を作成
- [ ] `docker compose up` → Phase 4 と同じ動作になることを確認
- [ ] map ファイルの volume mount が正しく動作することを確認

---

### Phase 6: タスク等価（将来）

```
目標: mission_manager, AI Planner のテストを Isaac Sim 上で実施する。
```

- [ ] mission_manager が Isaac Sim 上で動作
- [ ] MPPI vs RPP の比較評価
- [ ] 障害物回避・recovery BT のテスト
- [ ] （必要に応じて）Namespace 分離 + 真機と Sim の同時動作

---

## 9. 参考ファイル

| ファイル | 内容 |
|---|---|
| `todo-my/task-sim-real-dev/env-div-guidance.md` | GPT との構成議論（原案・参考） |
| `src/omni_base_driver/include/omni_base_driver/picking_robot_matrix.hpp` | 逆運動学（Isaac Sim スクリプトの参照元） |
| `src/omni_base_driver/include/omni_base_driver/motor_param.hpp` | ホイール位置・home_pos パラメータ |
| `src/localization_pkg/config/fast_lio_mid360.yaml` | FAST-LIO2 設定（`lid_topic: /livox/lidar`, `imu_topic: /livox/imu`） |
| `src/nav_pkg/launch/navigation.launch.py` | use_sim_time ハードコード箇所（5箇所 False） |
| `src/robot_description/urdf/robot.urdf.xacro` | Isaac Sim USD インポート元 URDF |

---

## 10. 決定事項まとめ

| 項目 | 決定内容 |
|---|---|
| 隔離方式 | ROS_DOMAIN_ID のみ（真機=13, Sim=20） |
| Topic namespace | Phase 1〜5 は変更なし（真機と同名） |
| `use_sim_time` | まず変更なしで試す。timestamp 問題が連続した場合だけ対応 |
| Docker | Phase 5 以降。Phase 1-4 は手動 colcon build |
| FAST-LIO2 | sim でも標準構成。`with_fast_lio:=true` を目標にする |
| 逆運動学 | `omni_drive_controller.py`（`picking_robot_matrix.hpp` 移植） |
| Joy-Con | real と同じ `/joy` / teleop / A-B-Y 操作を使う |
| 地図 | Phase 2 で Joy-Con 手動走行し、sim 環境 map を作る |
| Isaac Sim 起動 | Phase 1-4 は GUI。Phase 5 以降は Headless または Docker |
| 真機への影響 | `ROS_DOMAIN_ID=20` + `ros2 topic list` で隔離確認してから作業 |
