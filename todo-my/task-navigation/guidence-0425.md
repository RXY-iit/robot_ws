
**開発順序: TF → Odom → Joy-Con操作確認 → IMU確認 → GLIM建图 → 地図 export → FAST-LIO2 + GICP Localization → Nav2導航 → パラメータ調整**

---

## Latest Status Note - 2026-05-22

Current real Navigation baseline:

```bash
tools/open_nav2_windows.sh --cleanup
```

This now starts FAST-LIO mode by default:

```text
/livox/lidar       CustomMsg     FAST-LIO input; not directly visible in RViz
/livox/lidar_pc2   PointCloud2   GICP / pointcloud_to_laserscan / RViz
/fast_lio/odometry Odometry      GICP hint
```

Current localization meaning:

```text
FAST-LIO2 uses Livox LiDAR + IMU for short-term odometry.
GICP uses the saved map for global correction and publishes map -> odom.
Nav2 uses map -> odom -> base_footprint -> base_link, not FAST-LIO camera_init/body.
```

Latest successful debug run:

```text
debug-output/nav2_20260522_110151
```

Result: FAST-LIO2 stayed alive, GICP localization ran, Nav2 MPPI completed a small goal with `Goal succeeded`, and recoveries were `0`.

For the compact current status, read:

```text
todo-my/step7-stauts.md
todo-my/status-0425.md
```

---

## 0. 前提：現在の workspace 構成

| パッケージ | 状態 | 内容 |
| --- | --- | --- |
| `robot_bringup` | ✅ 実装済み | URDF/TF, Livox MID360, RealSense D435 |
| `omni_base_driver` | ✅ 実装済み | cmd_vel→motor, steer, odom, TF: odom→base_footprint |
| `robot_description` | ✅ 実装済み | URDF/xacro |
| `serial_transciever` | ✅ 実装済み | マニピュレータ制御（linear + camera-swing） |
| `localization_pkg` | ✅ 実装済み | map_server + GICP localizer + FAST-LIO2 hint + pointcloud_to_laserscan |
| `nav_pkg` | ✅ 実装済み | Nav2 + mode switch |

---

## Step 1: TF ツリーと基本トピックの確認

全体を起動してTFとトピックが正しく出ているか確認する。

### Quick check commands

問題が出たときは、以下を上から順に確認する。

```bash
# 0. conda が有効なら抜ける。ROS Humble は Python 3.10 前提
conda deactivate
which python3
python3 --version

# 1. workspace setup
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. USB / serial / permission
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
groups

# 3. duplicated launch / stale ROS graph
ros2 node list
ros2 node list | sort | uniq -d

# duplicated node warning が出る場合
ros2 daemon stop
ros2 daemon start

# 4. Modbus topicID check: drive_motor.py expects om_*0
ros2 topic list | grep -E "om_query|om_response|om_state"
ros2 topic hz /om_response0

# 5. drive odom chain
ros2 topic info -v /drive_odom
ros2 topic echo /drive_odom --once
ros2 topic echo /steer_odom --once
ros2 topic echo /wheel_odom --once

# 6. TF chain
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link livox_frame

# 7. sensor topics
ros2 topic echo /livox/imu --once
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
```

期待する状態：

- `python3 --version` が `Python 3.10.x`
- `groups` に `dialout` がある
- `/dev/ttyUSB0` が BLV-R RS485
- `/om_query0`, `/om_response0`, `/om_state0` が出る（`om_*1` ではない）
- `/drive_odom` と `/steer_odom` が出る
- `/wheel_odom` が出る
- TF が `map → odom → base_footprint → base_link → livox_frame` で繋がる

```bash
# prepare
cd /home/matsunaga-h/robot_ws && colcon build --packages-select robot_bringup && source install/setup.bash && ros2 pkg list | grep -x robot_bringup && ros2 launch robot_bringup test_all.launch.py

# センサー + モーター フル起動
ros2 launch robot_bringup test_all.launch.py

# 別端末で確認
ros2 topic list
ros2 run tf2_tools view_frames        # frames_*.pdf が生成される
ros2 run rqt_tf_tree rqt_tf_tree
ros2 topic echo /wheel_odom --once
ros2 topic echo /livox/imu --once
```

必須 TF チェーン：

```
map → odom → base_footprint → base_link → livox_frame
                                      → camera_link → camera_color_frame ...
```

必須トピックチェック：

| トピック | 型 | Publishノード |
| --- | --- | --- |
| `/cmd_vel` | Twist | Nav2 or teleop |
| `/wheel_odom` | Odometry | robot_odom_node |
| `/tf` | TFMessage | 各ノード |
| `/livox/lidar` | PointCloud2 | livox_ros_driver2 |
| `/livox/imu` | Imu | livox_ros_driver2 |
| `/camera/camera/color/image_raw` | Image | realsense2_camera |

Note: `robot_odom_node` は Odometry メッセージを `/wheel_odom` に publish し、TF は `odom → base_footprint` として publish する。URDF 側で `base_footprint → base_link` の固定TFを持つ。`/odom` は現在 `drive_motor.py` 側に publisher 定義だけ残っているが、publish処理はコメントアウトされているため通常は流れない。

RViz で `No transform from [base_footprint] to [base_link]` が出る場合は、`odom → base_link` と `base_footprint → base_link` が競合している可能性がある。TF は `odom → base_footprint → base_link` に統一する。

### TF ownership rule for GLIM / wheel odom

Current policy is **方案 A: wheel odom owns the robot odom TF**.

Do not let GLIM publish TF into the main robot chain. The main chain must stay:

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

Ownership:

| TF edge | Owner |
|---|---|
| `odom -> base_footprint` | `robot_odom_node` from wheel odom |
| `base_footprint -> base_link` | URDF / `robot_state_publisher` |
| `base_link -> livox_frame` | URDF / `robot_state_publisher` |
| `glim_map -> glim_odom -> glim_base` | GLIM, separated from main robot TF |

Important: `livox_frame` must not have two parents. If `/tf` contains `odom -> livox_frame` while `/tf_static` contains `base_link -> livox_frame`, RViz will show the LiDAR frame far from the robot. This usually means GLIM was configured with `base_frame_id: livox_frame`.

For the current policy, `glim_config/config_ros.json` should keep GLIM frame names separate:

```json
"imu_frame_id": "livox_frame",
"lidar_frame_id": "livox_frame",
"base_frame_id": "glim_base",
"odom_frame_id": "glim_odom",
"map_frame_id": "glim_map",
"publish_imu2lidar": false
```

When checking a new run or a new rosbag, confirm that these bad edges do not exist:

```bash
ros2 run tf2_ros tf2_echo odom livox_frame
ros2 run tf2_ros tf2_echo glim_odom livox_frame
```

Expected: `odom -> livox_frame` should not be a direct published edge in the main robot TF tree. The valid path is through `base_footprint` and `base_link`.

もし `/wheel_odom --once` が待ち続ける場合は、先に入力トピックを確認する。

```bash
ros2 topic info /wheel_odom
ros2 topic echo /steer_odom --once
ros2 topic echo /drive_odom --once
```

`/wheel_odom` は `/steer_odom` と `/drive_odom` の両方を受け取った後に publish される。

`/steer_odom` は出るが `/drive_odom` が出ない場合は、BLV-R drive motor 側の Modbus 通信を確認する。

```bash
ros2 topic info -v /drive_odom
ros2 topic hz /om_response0
ros2 node list | grep -E "om|drive|robot_odom"
```

`/drive_odom` は `drive_motor.py` が `/om_response0` のモーター応答を受け取った後に publish する。`/om_response0` が流れない場合は、BLV-R 電源、USB/RS485 接続、`/dev/ttyUSB0`、権限、`drive_motors:=true` を確認する。

重要: `drive_motor.py` は `/om_query0`, `/om_response0`, `/om_state0` を使う。`ros2 topic list` に `/om_response1` が出て `/om_response0` が出ない場合は、`om_modbus_master` の `topicID` が `1` で起動している。`robot_bringup/launch/test_all.launch.py` では `topicID:=0` にする。

`ros2 node list` で `WARNING: nodes in the graph that share an exact name` が出る場合は、同じ launch を複数回起動している可能性がある。一度すべての launch を止めてから、1つだけ起動し直す。

#### drive motor / BLV-R の切り分け手順

重複 launch を消してから、drive motor だけを段階的に確認する。

```bash
# 1. すべての launch 端末で Ctrl+C
# 2. 残っている ROS プロセスがないか確認
ps -eo pid,cmd | grep -E "ros2|launch|om_modbus|drive_motor|robot_odom|dynamixel|livox|realsense|rviz" | grep -v grep

# 3. 残っている場合のみ終了
pkill -f "ros2 launch"
pkill -f "om_modbus_master_launch.py"
pkill -f "drive_motor.py"
pkill -f "robot_odom_node"

# 4. ROS graph cache をクリア
ros2 daemon stop
ros2 daemon start

# 5. USB/RS485 が見えているか確認
ls -l /dev/ttyUSB*
ls -l /dev/serial/by-id/
groups
```

`groups` に `dialout` が無い場合は、串口権限を追加する。

```bash
sudo usermod -aG dialout $USER
# 追加後は logout/login、または PC reboot が必要
```

#### Python version / conda 環境の注意

ROS 2 Humble は Ubuntu 22.04 の Python 3.10 前提。`(base)` conda 環境のまま `colcon build` すると、`om_msgs` や `my_messages` が `python3.13` 用に生成され、`ros2 topic echo/hz` で `UnsupportedTypeSupport` になる。

`robot_bringup/launch/test_all.launch.py` で `drive_motor.py` を直接起動するときも、conda の `python3` を拾わないように `/usr/bin/python3` を指定する。

以下のようなエラーが出た場合は、message package を Python 3.10 で clean rebuild する。

```text
ModuleNotFoundError: No module named 'om_msgs.om_msgs_s__rosidl_typesupport_c'
UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'om_msgs'
```

手順：

```bash
# conda を抜ける。プロンプトから (base) が消えるまで実行
conda deactivate

cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash

# 古い python3.13 生成物を消す
rm -rf build/om_msgs install/om_msgs
rm -rf build/my_messages install/my_messages
rm -rf build/dynamixel_sdk_custom_interfaces install/dynamixel_sdk_custom_interfaces
rm -rf build/om_modbus_master install/om_modbus_master
rm -rf build/omni_base_driver install/omni_base_driver
rm -rf build/robot_bringup install/robot_bringup

# Python が /usr/bin/python3 / Python 3.10 であることを確認
which python3
python3 --version

# message package と依存ノードを作り直す
colcon build --symlink-install --packages-select \
  om_msgs my_messages dynamixel_sdk_custom_interfaces dynamixel_sdk \
  om_modbus_master omni_base_driver robot_bringup

source install/setup.bash
```

確認：

```bash
find install/om_msgs install/my_messages install/dynamixel_sdk_custom_interfaces -maxdepth 4 -type d | grep python3
```

期待値は `python3.10`。`python3.13` が出る場合は、まだ conda 環境が混ざっている。

drive motor だけ起動して確認する。

```bash
cd /home/matsunaga-h/robot_ws
source install/setup.bash
ros2 launch robot_bringup test_all.launch.py \
  lidar:=false camera:=false steer_motors:=false serial_motors:=false rviz:=false \
  drive_motors:=true static_odom:=true
```

別端末で確認する。

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
ros2 node list | grep -E "om|drive"
ros2 topic list | grep -E "om_|drive_odom"
ros2 topic hz /om_response0
ros2 topic echo /drive_odom --once
```

期待値：

- `/om_node` が存在する
- `/om_query0`, `/om_response0`, `/om_state0` が存在する（`om_*1` ではない）
- `/om_response0` が publish される
- `/drive_odom` の Publisher count が 1 になる
- `/drive_odom --once` で `vel1/vel2/vel3` が表示される

`/om_node` はあるが `/om_response0` が出ない場合は、BLV-R 電源、RS485/USB、ポート名 `/dev/ttyUSB0`、dialout 権限を優先して確認する。

`/om_response0` は出るが `/drive_odom` が出ない場合は、`drive_motor.py` が起動しているか確認する。`drive_motor.py` のノード名は `drive_*` ではなく、`my_sub`, `my_pub`, `my_pub_polling`, `my_sub_2`。

```bash
ros2 node list | grep -E "om_node|my_sub|my_pub"
ros2 topic info -v /drive_odom
ros2 topic echo /om_response0 --once
```

期待値：

- `/my_sub` が存在する
- `/my_pub` または `/my_pub_polling` が存在する
- `/drive_odom` の Publisher count が 1
- `/om_response0 --once` の `func_code` が `0` または `3`
- `/om_response0 --once` の `data` が3個以上ある

`/om_response0 --once` の `func_code` が `16` の場合は write 応答だけが見えている状態。`drive_motor.py` の polling node (`/my_pub_polling`) が動いていれば read request を出すため、`func_code: 0` または `func_code: 3` の応答も出る。

`/my_sub` が存在しない場合は、`drive_motor.py` が起動していない、または起動直後に落ちている。launch terminal のエラーを確認する。

```bash
# launch terminal で確認するログの例
ModuleNotFoundError
UnsupportedTypeSupport
ImportError
Permission denied
```

`tf_transformations` import 時に NumPy / transforms3d エラーで落ちる場合は、`drive_motor.py` では `/drive_odom` publish に不要なため、yaw quaternion を直接計算する実装にして `tf_transformations` 依存を外す。

手動で `drive_motor.py` だけ起動してエラーを見る場合：

```bash
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/om_modbus_master_V201/om_modbus_master/sample/BLV_R/drive_motor.py
```

この状態で別端末から確認する。

```bash
source /home/matsunaga-h/robot_ws/install/setup.bash
ros2 topic info -v /drive_odom
ros2 topic echo /drive_odom --once
```

### ✅ Step 1 確認リスト

- [ ] `ros2 topic list` に上記6トピックが全て存在する
- [ ] `view_frames` で `map→odom→base_footprint→base_link→livox_frame` が繋がっている
- [ ] `/wheel_odom` に wheel odom (x/y/theta) が流れている
- [ ] `/livox/imu` に angular_velocity / linear_acceleration が流れている
- [ ] RViz でPointCloudが表示される

---

## Step 2: Joy-Con による base 手動操作

追加したファイルは以下の2つです：

config/joy_teleop.yaml

- teleop_twist_joy_node のパラメータ定義
- L1をenableスイッチ（押している間のみ/cmd_vel送信）
- R1でターボモード（速度2倍）
- turboスケール追加（0.3→0.6 m/s、0.5→1.0 rad/s）
- コメントに軸番号の調べ方を記載

launch/teleop.launch.py

- joy_node + teleop_twist_joy_node を起動
- joy_dev:= 引数でデバイスパス変更可能（Joy-Conがjs1に割り当てられた場合など）


![todo-my/images/joycon.png](images/joycon.png)
この設定の操作対応は以下の通り。

| 役割 | 入力 | 動作 |
|---|---|---|
| `enable_button: 4` | **L1 / LB** | 押している間だけ `/cmd_vel` を送信する安全スイッチ |
| `enable_turbo_button: 5` | **R1 / RB** | ターボ速度で送信する |
| `axis_linear.x: 1` | 左スティック 上下 | 前後移動 |
| `axis_linear.y: 0` | 左スティック 左右 | 横移動（omni base） |
| `axis_angular.yaw: 3` | 右スティック 左右 | 旋回 |

つまり、L1 を押しながら左スティックで移動、右スティック左右で旋回する。R1 を同時押しすると高速モードになる。

### 起動手順

```bash
# Terminal 1
ros2 launch robot_bringup test_all.launch.py

# 別端末で teleop 起動
ros2 launch robot_bringup teleop.launch.py

# Joy-Con が認識されているか確認
ros2 topic echo /joy --once

# cmd_vel が流れているか確認（L1ボタンを押しながらスティック操作）
ros2 topic echo /cmd_vel
```

### ✅ Step 2 確認リスト

- [ ] `teleop.launch.py` と `joy_teleop.yaml` を `robot_bringup` に追加した
- [ ] `/joy` トピックに Joy-Con の入力が流れている
- [ ] L1を押しながらスティック操作で `/cmd_vel` が出力される
- [ ] `/cmd_vel` を受けて実機の車輪が動く（`test_all.launch.py` が起動中の状態で）
- [ ] 緊急停止：L1を離すと即座に速度0になる

---

## Step 3: IMU の確認と TF 整合性

Livox MID360 は **PointCloud2 + IMU を同時に publish** する。
IMU は後の GLIM SLAM や EKF Localization で使うため、早めに確認しておく。

```bash
# IMU データ確認
ros2 topic echo /livox/imu

# IMU の frame_id 確認（livox_frame であること）
ros2 topic echo /livox/imu --field header.frame_id

# 静止状態での IMU 値確認（linear_accel の z ≈ 9.8 m/s²）
ros2 topic hz /livox/imu   # 目標: 200 Hz 前後
ros2 topic hz /livox/lidar # 目標: 10 Hz 前後
```

IMU の TF フレーム確認（URDF の `livox_frame` と一致していること）：

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
```

### ✅ Step 3 確認リスト

- [ ] `/livox/imu` に正常なデータが流れている (hz: ~200Hz)
- [ ] `header.frame_id` が `livox_frame` になっている
- [ ] 静止時に `linear_acceleration.z ≈ 9.8 m/s²`
- [ ] `tf2_echo base_link livox_frame` で変換が取れる（TF繋がっている）
- [ ] `/livox/lidar` の point cloud frame_id も `livox_frame` になっている

---

## Step 4: GLIM による 3D SLAM 建図

### GLIM とは

| 特徴 | 内容 |
|---|---|
| 入力 | 3D LiDAR (PointCloud2) + IMU |
| 方式 | Global-scale LiDAR-IMU Mapping |
| 出力 | 3D点群地図・6DoFオドメトリ |
| ROS 2 | Humble対応 |
| Mid360 との相性 | ◎ (non-repetitive scan対応) |

SLAM方式の比較：

| 方案 | 適合場合 | IMU使用 |
|---|---|---|
| **GLIM** | 3D LiDAR + IMU, 高精度, Mid360向き | ✅ |
| `FAST-LIO2` | 3D LiDAR + IMU, 高速 | ✅ |
| `LIO-SAM` | 3D LiDAR + IMU + GPS | ✅ |
| `slam_toolbox` | 2D LaserScan のみ | ❌ |

### インストール

`ros-humble-glim-ros` は通常の ROS apt repository には無い。以下のように Koide PPA を追加してから install するか、source build する。

Official docs:

- https://koide3.github.io/glim/installation.html
- https://github.com/koide3/glim
- https://github.com/koide3/glim_ros2

#### Option A: PPA からインストール（推奨）

```bash
# conda を抜ける
conda deactivate

source /opt/ros/humble/setup.bash

# Koide PPA を追加
sudo apt install -y curl gpg
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# CPU版
sudo apt update
sudo apt install -y ros-humble-glim-ros

# install 後に共有ライブラリを更新
sudo ldconfig

# 確認
ros2 pkg list | grep -E "^glim$|^glim_ros$"
ros2 pkg executables glim_ros
```

CUDA版を使う場合はPCのCUDAに合わせてどれか1つを選ぶ。

```bash
sudo apt install -y ros-humble-glim-ros-cuda12.2
sudo apt install -y ros-humble-glim-ros-cuda12.6
sudo apt install -y ros-humble-glim-ros-cuda13.1
```

#### Option B: source build

PPA install が使えない場合のみ source build にする。GLIM は `glim_ros2` だけでなく、core package `glim` と依存ライブラリも必要。

```bash
conda deactivate
source /opt/ros/humble/setup.bash

# common dependencies
sudo apt update
sudo apt install -y \
  libomp-dev libboost-all-dev libmetis-dev \
  libfmt-dev libspdlog-dev \
  libglm-dev libglfw3-dev libpng-dev libjpeg-dev

# GLIM source
cd /home/matsunaga-h/robot_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

cd /home/matsunaga-h/robot_ws
colcon build --symlink-install --packages-select glim glim_ros
source install/setup.bash

# 確認
ros2 pkg list | grep -E "^glim$|^glim_ros$"
ros2 pkg executables glim_ros
```

Source build で GTSAM / gtsam_points / iridescence が見つからない場合は、official installation docs の dependency 手順に従って先に入れる。

### 起動（建図モード）

```bash
# Terminal 1: ロボット本体起動
ros2 launch robot_bringup test_all.launch.py

# Terminal 2: GLIM SLAM
ros2 run glim_ros glim_rosnode
```

GLIM に必要なトピック：

| トピック | 型 | 備考 |
|---|---|---|
| `/livox/lidar` | PointCloud2 | remap 不要（デフォルト一致） |
| `/livox/imu` | Imu | remap 不要 |

GLIM の config で `imu_topic: /livox/imu`, `points_topic: /livox/lidar` を確認する。デフォルト config が別トピックの場合は、GLIM config root の `config_ros.json` をコピーして編集し、`config_path` を指定する。

この環境の GLIM default config root:

```bash
/opt/ros/humble/share/glim/config
```

workspace-local config:

```bash
/home/matsunaga-h/robot_ws/glim_config
```

作成手順：

```bash
mkdir -p /home/matsunaga-h/robot_ws/glim_config
cp -a /opt/ros/humble/share/glim/config/. /home/matsunaga-h/robot_ws/glim_config/
```

`/home/matsunaga-h/robot_ws/glim_config/config_ros.json` の変更点：

```json
"imu_topic": "/livox/imu",
"points_topic": "/livox/lidar",
"imu_frame_id": "livox_frame",
"lidar_frame_id": "livox_frame",
"base_frame_id": "livox_frame",
"acc_scale": 9.80665
```

CPU版 `ros-humble-glim-ros` を使う場合は、`/home/matsunaga-h/robot_ws/glim_config/config.json` を CPU config にする。

```json
"config_odometry": "config_odometry_cpu.json",
"config_sub_mapping": "config_sub_mapping_cpu.json",
"config_global_mapping": "config_global_mapping_cpu.json"
```

以下のエラーが出る場合は GPU config を読んでいる。

```text
failed to open libodometry_estimation_gpu.so
failed to load odometry estimation module
```

インストール済み module の確認：

```bash
find /opt/ros/humble -name 'libodometry_estimation*.so' -o -name 'libsub_mapping*.so' -o -name 'libglobal_mapping*.so'
```

```bash
# config_path を指定する例
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/matsunaga-h/robot_ws/glim_config
```

#### Handheld Livox mapping

Livox MID360 を手で持って歩きながら地図作成することは可能。ただし以下を守る。

- `base_frame_id` は `livox_frame` にする
- robot base の `/wheel_odom` や `odom → base_footprint` TF と混ぜない
- 最初の数秒はセンサーを静止させて IMU 初期化を待つ
- 急回転、強い振動、速すぎる移動を避ける
- LiDAR 視野に壁・柱・家具など十分な形状特徴が入るように歩く
- ガラス、鏡、真っ黒な壁、開けすぎた空間だけの場所は避ける

Handheld で起動する場合は、ロボットの odom TF と衝突しないように sensor-only で起動する。

```bash
# Terminal 1: Livox + robot_state_publisher only
ros2 launch robot_bringup test_all.launch.py \
  drive_motors:=false steer_motors:=false serial_motors:=false camera:=false rviz:=true static_odom:=false

# Terminal 2: GLIM
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/matsunaga-h/robot_ws/glim_config
```

### RViz で確認

```bash
# GLIM の map + odometry を RViz で確認
ros2 topic list | grep -E "glim|map|odom"
```

### ✅ Step 4 確認リスト

- [ ] GLIM が正常に起動する（エラーなし）
- [ ] RViz で 3D 点群マップが蓄積されていく様子が見える
- [ ] GLIM は main robot TF を上書きしない（方案 A: wheel odom owns `odom → base_footprint`）
- [ ] ロボットを移動させながら地図が広がっていくことを確認
- [ ] ループクロージャ（同じ場所に戻ると地図が整合する）を確認

---

## Step 5: 地図の保存

建図完了後に地図を保存する。

GLIMの場合（3D点群マップ）：

```bash
# GLIM ROS package に /glim_ros/save_map service が無い場合がある
ros2 service list | grep -E "glim|save|map"

# この環境では GLIM dump は /tmp/dump に出力される
ls -lah /tmp/dump
find /tmp/dump -maxdepth 2 -type f | sort

# /tmp は消えやすいので、建図後すぐ workspace に保存する
mkdir -p /home/matsunaga-h/robot_ws/maps
cp -a /tmp/dump /home/matsunaga-h/robot_ws/maps/glim_dump_$(date +%Y%m%d_%H%M%S)

# または ROS2 bag で保存しておく
ros2 bag record /livox/lidar /livox/imu /tf /tf_static -o slam_bag
```

`ros2 service call /glim_ros/save_map std_srvs/srv/Empty` が `waiting for service to become available...` のままの場合は、その service が存在しない。`ros2 service list` で実在する service を確認する。

rosbag は `ros2 bag record ... -o slam_bag` を実行したカレントディレクトリに保存される。`~/robot_ws` で実行した場合：

```bash
/home/matsunaga-h/robot_ws/slam_bag/
```

確認：

```bash
ros2 bag info /home/matsunaga-h/robot_ws/slam_bag
ls -lh /home/matsunaga-h/robot_ws/slam_bag
```

### 保存した GLIM map の表示

GLIM dump は RViz が直接読める `.pcd` / `.ply` ではない。まずは GLIM の viewer で確認する。

```bash
ros2 run glim_ros offline_viewer \
  --map_path /home/matsunaga-h/robot_ws/maps/l402_glim_map_0503 \
  --config_path /home/matsunaga-h/robot_ws/glim_config_nav2
```

編集・別名保存したい場合：

```bash
ros2 run glim_ros map_editor \
  --map_path /home/matsunaga-h/robot_ws/maps/l402_glim_map_0503
```

RViz で見る場合は、保存済み dump を直接読むのではなく、rosbag を再生して GLIM に再処理させ、GLIM が publish する topic を RViz に追加する。

```bash
# Terminal 1: GLIM
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/matsunaga-h/robot_ws/glim_config

# Terminal 2: bag replay
ros2 bag play /home/matsunaga-h/robot_ws/slam_bag/l402_map_0503

# Terminal 3: RViz
rviz2
```

RViz で追加するもの：

- `TF`: 座標変換ツリーを表示する
- `PointCloud2`: LiDAR点群またはGLIMがpublishする地図/点群topicを表示する

方案 A では GLIM の TF を main robot TF から分離しているため、GLIM map を見るときは RViz の `Fixed Frame` を `glim_map` にする。

```text
Fixed Frame: glim_map
PointCloud2 Topic: /glim_ros/map
```

If `/glim_ros/map` shows `Status: Ok` but the RViz view is empty, first check whether the map topic is actually publishing new messages. `Status: Ok` only means the topic/TF setup is valid; it does not guarantee that the message contains visible points or that a fresh message has arrived.

```bash
ros2 topic hz /glim_ros/map
ros2 topic hz /glim_ros/aligned_points
ros2 topic hz /livox/lidar
```

During live rosbag replay, `/glim_ros/aligned_points` is the better first visualization topic because it is a live GLIM-aligned point cloud. In the current setup it publishes around 10 Hz, while `/glim_ros/map` may publish rarely or not until GLIM has generated/updated a global map.

```text
Fixed Frame: glim_map
PointCloud2 Topic: /glim_ros/aligned_points
```

If still invisible, temporarily make the PointCloud2 display easier to see:

```text
Style: Points
Size (Pixels): 3-5
Color Transformer: FlatColor
Color: bright yellow or white
```

`Fixed Frame` が `map` のままだと、RViz の TF display に以下の warning が出る。

```text
No transform from [glim_map] to [map]
No transform from [glim_odom] to [map]
```

これは GLIM map が消えているという意味ではなく、`map` tree と `glim_map` tree を意図的に分けているため。GLIM map と robot TF を同じ RViz view で重ねたい場合だけ、一時的な visualization 用 bridge を別 terminal で起動する。

```bash
# Visualization only: connect main map tree and GLIM map tree as identity.
# Do not use this as final localization design.
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map glim_map
```

GLIM が publish している topic は以下で確認する。

```bash
ros2 topic list | grep -E "glim|map|odom|cloud|points"
ros2 topic echo /glim_ros/map --once --field header
```

`TF` はセンサーやロボットの座標関係を見る表示。`PointCloud2` は3D点群そのものを見る表示。`TF` だけでは地図は見えない。`PointCloud2` を追加して topic を選ぶ必要がある。

### Nav2 との関係

GLIM dump は GLIM の建図・編集用データ。Nav2 や安定した localization に直接渡す地図ではない。
2026-05-09 の確認で、GLIM は既知地図に対する固定地図 localization ではなく、既存 dump を読みながら local/global mapping を続け、loop closure 的に旧地図へ寄せる挙動だと判断した。そのため GLIM は navigation localization には使わず、3D map 作成・viewer export 用に限定する。

Nav2 localization は **FAST-LIO2 + GICP prior-map localization** を使う。AMCL は使わない。2D map は localization ではなく、Nav2 の global costmap / 経路計画用として使う。

### ✅ Step 5 確認リスト

- [ ] 地図ファイル（3D pcd or 2D pgm/yaml）が保存されている
- [ ] `my_map.yaml` の `resolution`, `origin` が正しい
- [ ] 保存した地図を RViz で読み込んで表示できる

---

## Step 6: Localization（GICP prior-map 方式）

### 設計方針（2026-05-09 更新）

GLIM は **建図専用**。Nav2 localization には prior-map GICP matching を使う。

**GLIM を Navigation localization に使わない理由**:

```text
glim_config_nav2/config_ros.json:
  enable_local_mapping:  true   ← 走りながら新しい submap を作り続ける
  enable_global_mapping: true   ← loop closure で位置が突然ジャンプする
```

この構成では「固定 map への pose 推定（localization）」ではなく「引き続き地図を描く（SLAM）」に近い挙動になる。起動位置・環境条件によっては収束せず、ドリフトも deterministic でない。

**GICP prior-map localization の特徴**:

| | GLIM localization（旧） | GICP localizer（新） |
|---|---|---|
| 動作 | SLAM-like（submap生成あり） | localization-only（map固定） |
| loop closure | あり（突然ジャンプ） | なし |
| 起動収束 | ~0.75m 移動が必要 | 起動時に初期位置パラメータを与えるだけ |
| 計算コスト | 高 | 低（GICP 0.5 Hz） |
| deterministic | ✗ | ✅ |

**TF 分担（変更なし）**:

```text
map → odom             ← gicp_localizer_node (scan-to-map GICP)
odom → base_footprint  ← wheel odom（robot_odom_node、変更なし）
base_footprint → base_link → livox_frame  ← URDF
```

**2フェーズ実装**:

```text
Phase 1（実装済み）: GICP localizer のみ
  /livox/lidar (PointCloud2, xfer_format=0) → PLY map GICP → map→odom TF

Phase 2（実装済み・推奨）: FAST-LIO2 + GICP
  FAST-LIO2 (LiDAR-IMU tight coupling) → 初期推定精度向上
  GICP の収束が速くなり、振動・急旋回でも安定
  setup: tools/setup_fast_lio.sh
  launch: tools/open_fast_lio2_loc_terminals.sh --mode real --cleanup
```

---

### 6A. Python 依存パッケージのインストール（初回のみ）

重要: ROS 2 Humble は Ubuntu 22.04 の **Python 3.10** 前提。`pip3 install open3d` をそのまま実行すると、conda の Python 3.13 や pip の NumPy 2.x が混ざり、Ubuntu apt の SciPy と ABI 不一致になることがある。

以下のエラーが出た場合:

```text
UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required ...
ValueError: numpy.dtype size changed, may indicate binary incompatibility.
Expected 96 from C header, got 88 from PyObject
```

原因はほぼ `numpy 2.x` と `/usr/lib/python3/dist-packages/scipy` の混在。`pip3` ではなく `/usr/bin/python3 -m pip` を使い、NumPy を 1.24 系に固定する。

```bash
# conda を抜ける。プロンプトから (base) が消えるまで実行
conda deactivate
hash -r

# ROS Humble 用 Python であることを確認
which python3
python3 --version
/usr/bin/python3 --version

# pip が無い場合のみ
sudo apt install python3-pip

# open3d: GICP backend（推奨）
# NumPy 2.x は Ubuntu apt の scipy 1.8.0 と衝突するため固定する
/usr/bin/python3 -m pip install --user --force-reinstall \
  "numpy==1.24.4" \
  "scipy==1.10.1" \
  "scikit-learn==1.3.2" \
  "open3d==0.19.0"

# または small_gicp（軽量代替）
# /usr/bin/python3 -m pip install --user small-gicp
```

確認:

```bash
/usr/bin/python3 -c "import numpy; print('numpy', numpy.__version__, numpy.__file__)"
/usr/bin/python3 -c "import scipy; print('scipy', scipy.__version__, scipy.__file__)"
/usr/bin/python3 -c "import open3d; print('open3d', open3d.__version__)"
/usr/bin/python3 -c "import rclpy; print('rclpy ok')"
/usr/bin/python3 -c "import open3d as o3d; p='maps/saved-map/map-l402-0503/l402_points_0503'; c=o3d.io.read_point_cloud(p, format='ply'); print(len(c.points))"
```

期待:

```text
numpy 1.24.4
scipy 1.10.1
open3d 0.19.0
```

`ros2 run localization_pkg gicp_localizer_node` が conda の Python 3.13 を拾うと、以下のように `rclpy` が失敗する:

```text
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
... _rclpy_pybind11.cpython-313-x86_64-linux-gnu.so ...
```

この場合は conda を抜ける。`localization_pkg/scripts/gicp_localizer_node.py` の shebang は `/usr/bin/python3` に固定してあるため、再 build 後は ROS Humble の Python 3.10 で起動する。

もし `/usr/local/lib/python3.10/dist-packages/numpy` の NumPy 2.x がまだ優先される場合は、ユーザー site 側に固定版を入れ直す:

```bash
/usr/bin/python3 -m pip install --user --force-reinstall "numpy==1.24.4"
```

`l402_points_0503` は PLY header を持つが `.ply` 拡張子が無い。Open3D の自動判定は拡張子に依存するため、手動確認では `format='ply'` を付ける。`gicp_localizer_node.py` 側は PLY header を検出して `format='ply'` で読む。

---

### 6B. localization_pkg のビルド

```bash
cd /home/matsunaga-h/robot_ws
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --packages-select localization_pkg
source install/setup.bash
```

ビルド後に確認:

```bash
ros2 pkg executables localization_pkg
# → localization_pkg  gicp_localizer_node
```

---

### 6C. GICP Localization 起動: live / bag replay mode

GICP localization は **live mode** と **bag replay mode** を分けて起動する。  
混ぜると TF の時間軸が壊れ、以下のような warning が出る:

```text
TF_OLD_DATA ignoring data from the past for frame base_footprint
TF_OLD_DATA ignoring data from the past for frame glim_odom
```

これは「bag の古い timestamp」と「現在の wall-clock timestamp」が同じ TF buffer に混ざった状態。

#### 推奨: 自動 terminal launcher

最新の GICP localization は `tools/open_gicp_loc_terminals.sh` から起動する。  
旧 `tools/open_glim_loc_terminals.sh` は互換 wrapper で、新しい script に転送される。

```bash
# rosbag replay mode: use_sim_time=true + ros2 bag play --clock
tools/open_gicp_loc_terminals.sh --mode bag --cleanup

# 実機 mode: use_sim_time=false + robot_bringup
tools/open_gicp_loc_terminals.sh --mode real --cleanup

# bag 再生速度を落とす場合
tools/open_gicp_loc_terminals.sh --mode bag --rate 0.4 --cleanup
```

自動 launcher が開くもの:

```text
bag mode:
  1. robot_description rsp.launch.py (current URDF, use_sim_time=true)
  2. fast_lio_localization_bag.launch.py
  3. ros2 bag play --clock (/tf_static は remap して無視)
  4. /l402_glim_points PLY overlay (use_sim_time=true)
  5. RViz
  6. status checks

real mode:
  1. robot_bringup test_all.launch.py
  2. fast_lio_localization_live.launch.py
  3. /l402_glim_points PLY overlay (use_sim_time=false)
  4. RViz
  5. status checks
```

#### Mode 1: live sensor / real robot

実機センサー・実機 wheel odom を使う通常運用。`use_sim_time=false` 固定。

```bash
# 古い bag / GLIM / localization を止める
pkill -f "ros2 bag play" || true
pkill -f "glim" || true
pkill -f "fast_lio_localization" || true
ros2 daemon stop && ros2 daemon start

cd /home/matsunaga-h/robot_ws
conda deactivate
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1: robot + sensors + wheel odom
ros2 launch robot_bringup test_all.launch.py

# Terminal 2: GICP localization + map_server + pointcloud_to_laserscan
ros2 launch localization_pkg fast_lio_localization_live.launch.py

# Terminal 3: RViz 用 PLY overlay (/l402_glim_points)
tools/publish_l402_overlay.sh live
```

起動位置を指定する場合:

```bash
ros2 launch localization_pkg fast_lio_localization_live.launch.py \
  initial_x:=1.5 initial_y:=0.3 initial_yaw:=1.57
```

#### Mode 2: rosbag replay test

rosbag の `/livox/lidar`, `/tf`, `/tf_static` を使う再生テスト。`use_sim_time=true` 固定。  
bag は必ず `--clock` 付きで再生する。

```bash
# 実機 bringup / live localization を止める
pkill -f "test_all.launch.py" || true
pkill -f "fast_lio_localization" || true
pkill -f "glim" || true
pkill -f "ros2 bag play" || true
ros2 daemon stop && ros2 daemon start

cd /home/matsunaga-h/robot_ws
conda deactivate
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal 1: bag clock/time を使う localization
ros2 launch localization_pkg fast_lio_localization_bag.launch.py

# Terminal 2: bag replay。--clock 必須
ros2 bag play /home/matsunaga-h/robot_ws/slam_bag/l402_map_0503 --clock --rate 1.0

# Terminal 3: RViz 用 PLY overlay (/l402_glim_points)
# bag mode では use_sim_time=true にする
tools/publish_l402_overlay.sh bag
```

bag に GLIM の `glim_map → glim_odom` など余分な TF が入っていても、main robot TF と分離されていれば基本は問題ない。  
ただし bag 内に `map → odom` や `odom → livox_frame` など main TF と競合する edge が入っている場合は、bag をフィルタする:

```bash
/usr/bin/python3 tools/filter_livox_tf_bag.py \
  slam_bag/l402_map_0503 \
  slam_bag/l402_map_0503_gicp_input \
  --drop-edge map:odom \
  --drop-edge odom:livox_frame \
  --child-frame livox_frame \
  --force

ros2 bag play /home/matsunaga-h/robot_ws/slam_bag/l402_map_0503_gicp_input --clock --rate 1.0
```

Mode check:

```bash
# live mode: /clock は不要
ros2 topic list | grep -x /clock || echo "live mode: no /clock OK"

# bag mode: /clock が必要
ros2 topic echo /clock --once

# duplicate / stale node check
ros2 node list | sort | uniq -d
ros2 topic info -v /tf
```

**起動位置の決め方**:
- ロボットは常に同じ位置（例: 充電ステーション・ドア前）からスタートする運用とする
- RViz で `2D Pose Estimate` を使って初期位置を与えることも可能（`/initialpose` topic）。
  クリックした pose は「map 上の robot pose」として扱われ、GICP localizer が `map -> odom` を再計算する。
- GLIM のような「しばらく動いて収束待ち」は不要

**起動後の確認**:

```bash
# GICP score が fitness_score_threshold (0.5) を下回れば localization 成功
ros2 topic echo /gicp_loc/score --once
# → 0.1〜0.3 程度が正常。0.5 以上の場合は初期位置ずれを疑う

# TF が出ていること
ros2 run tf2_ros tf2_echo map odom
```

RViz で確認:

```text
Fixed Frame: map
/gicp_loc/pose → PoseStamped → robot の推定位置が地図上に表示される
/l402_glim_points → PLY overlay と一致するか確認
/gicp_loc/score → score が低い = localization 成功
```

`/l402_glim_points` が表示されない場合:

```bash
# topic が存在するか
ros2 topic list | grep -x /l402_glim_points

# publisher がいるか、RViz subscriber が見えているか
ros2 topic info -v /l402_glim_points

# header.frame_id は map のはず
ros2 topic echo /l402_glim_points --once --field header
```

期待:

```text
frame_id: map
```

RViz 側:

```text
Fixed Frame: map
Add → PointCloud2
Topic: /l402_glim_points
Reliability Policy: Reliable
Durability Policy: Transient Local
Style: Points
Size (m): 0.03〜0.08
Alpha: 1.0
```

注意: `/l402_glim_points` は GICP localizer ではなく `tools/publish_l402_overlay.sh` が publish する可視化専用 topic。
`/initialpose` では動かさない。RViz の `2D Pose Estimate` は robot localization だけを reset する。

**score が高い（localization 失敗）場合**:

```bash
# /initialpose で位置を再設定（RViz の 2D Pose Estimate ボタン）
# または launch 引数で initial_x/y/yaw を正しい値に変更して再起動
```

---

### 6D. Nav2 用 2D map の確認・再生成

Nav2 の global costmap（static_layer）と map_server が `/map`（OccupancyGrid）を使う。

現在の 2D map（生成済み）:

```text
maps/l402_2d_map_0503.yaml / .pgm      ← 部屋+廊下（全体、元 map）
maps/l402_2d_map_lite_0504.yaml / .pgm ← 部屋のみ（軽量版）
maps/l402_2d_map_clean_0509.yaml / .pgm ← Nav2 testing 用 clean map（推奨）
```

再生成が必要な場合（新しい PLY map を建図した後）:

```bash
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 全体マップ
/usr/bin/python3 tools/ply_to_nav2_map.py \
  maps/saved-map/map-l402-0503/l402_points_0503 \
  --output maps/l402_2d_map_0503 \
  --resolution 0.05 \
  --z-min -1.0 --z-max 0.3 \
  --min-hits 2 \
  --inflate-radius 0.10

# Nav2 testing 用 clean map
/usr/bin/python3 tools/ply_to_nav2_map.py \
  maps/saved-map/map-l402-0503/l402_points_0503 \
  --output maps/l402_2d_map_clean_0509 \
  --resolution 0.05 \
  --z-min -1.0 --z-max 0.3 \
  --min-hits 2 \
  --inflate-radius 0.05

# 部屋のみ軽量版
/usr/bin/python3 tools/ply_to_nav2_map.py \
  maps/l402_lite_0504_replay/l402_lite_0504_points_world.ply \
  --output maps/l402_2d_map_lite_0504 \
  --resolution 0.05 \
  --z-min -1.0 --z-max 0.3 \
  --min-hits 2 \
  --inflate-radius 0.10
```

高さフィルタ（LiDAR z≈0、床 z≈-1.5、天井 z≈+1.5）:

| Parameter | 値 | 意味 |
|---|---|---|
| `--z-min` | -1.0 | 床（z≈-1.5）除外、壁下端（床上+0.5m）から取得 |
| `--z-max` | +0.3 | 天井（z≈+1.5）除外。`1.5` にすると天井が黒く残る |

RViz の横視点で robot model が 2D map の白い平面より下に見える場合がある。これは `OccupancyGrid` が `z=0` の可視化平面として描かれ、3D PLY / robot model は実際の TF z 座標で描かれるため。Nav2 は基本的に XY 平面の `map -> odom -> base_footprint` と `/map` grid を使うので、XY が合っていれば Navigation には大きな問題にならない。  
確認すべき点は `base_footprint` が床基準で、`map -> base_footprint` の roll/pitch がほぼ 0、`/scan` が `base_footprint` frame の正しい高さスライスで出ていること。
| `--min-hits` | 2 | 1点ノイズ除去 |
| `--inflate-radius` | 0.10 | 障害物を少し太らせる |

---

### 6E. /scan パラメータ調整

`fast_lio_localization.launch.py` で起動する `pointcloud_to_laserscan` の高さスライス:

```text
LiDAR 位置: 床上 1.3 m、30° 下向きマウント
ロボット高さ: 1.35 m、幅 0.6 m

LiDAR 30° 下向き盲ゾーン: 前方約 1.4 m 以内は検出不可（壁の下部）
→ /scan は 1.4 m 以遠の障害物壁を検出できる

デフォルト高さスライス（base_footprint frame）:
  scan_min_height: -0.8  ← 床+0.5 m に相当（壁の中段を捉える）
  scan_max_height: -0.3  ← 床+1.0 m に相当

将来改善: RealSense depth camera を local costmap の obstacle_layer に追加し
         盲ゾーン（0〜1.4 m）をカバーする
```

---

### 6F. PLY visualization（debug overlay）

RViz で PLY map を重ねて表示する場合（localization の視覚確認用、Nav2 には不要）:

```bash
tools/publish_l402_overlay.sh live

# rosbag replay の場合
tools/publish_l402_overlay.sh bag
```

---

### 6G. FAST-LIO2 セットアップ（Phase 2）

FAST-LIO2 は LiDAR-IMU tight coupling odometry。GICP の初期推定を大幅に改善し、振動・急旋回での精度が向上する。

**前提条件**: livox_ros_driver2 を `xfer_format=1`（CustomMsg）に変更する必要がある（詳細は `tools/setup_fast_lio.sh` 内コメント参照）。

```bash
# FAST-LIO2 のクローン・ビルド（初回のみ）
tools/setup_fast_lio.sh

# build 後に既存 terminal で手動起動する場合は再 source 必須
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix fast_lio

# FAST-LIO2 + GICP Phase 2 quick launcher（実機）
tools/open_fast_lio2_loc_terminals.sh --mode real --cleanup

# FAST-LIO2 + GICP Phase 2 quick launcher（rosbag）
tools/open_fast_lio2_loc_terminals.sh --mode bag --cleanup

# 手動起動する場合
ros2 launch localization_pkg fast_lio_localization_live.launch.py \
  with_fast_lio:=true \
  use_fast_lio_hint:=true
```

注意: hku-mars FAST_LIO の ROS2 branch は Livox `CustomMsg` を期待する。  
一方、GICP localizer と pointcloud_to_laserscan は `/livox/lidar` の `PointCloud2` を使う。Phase 2 では FAST-LIO2 が読める入力形式と、GICP 用 PointCloud2 の両立（relay / fork / driver設定）が必要。

---

### 6H. セマンティック地図用 rosbag の取得

将来のセマンティックマッピング（YOLO + 3D back-projection）のためのデータ収集。  
カメラを **初期位置（pan=275°, tilt=67°）で固定**してから記録する（動かさない）。

```bash
# 全ノード起動後
tools/record_semantic_bag.sh

# Phase 2 localization も同時に起動してから記録する場合
tools/record_semantic_bag.sh --start-phase2-loc

# robot_bringup が既に動いている場合は localization だけ起動
tools/record_semantic_bag.sh --start-phase2-loc --loc-no-bringup

# 出力: slam_bag/semantic_YYYYMMDD_HHMMSS/
```

記録トピック一覧（geometry + RGB + depth + camera state + TF + FAST-LIO2/GICP pose/score）は `tools/record_semantic_bag.sh` 参照。

---

### ✅ Step 6 確認リスト

- [x] GLIM map dump `maps/l402_glim_map_0503` 保存済み（2026-05-03）
- [x] PLY map `maps/saved-map/map-l402-0503/l402_points_0503` 保存済み
- [x] 2D map 生成: `maps/l402_2d_map_0503.yaml`（z-min=-1.0, z-max=0.3）
- [x] 2D map lite: `maps/l402_2d_map_lite_0504.yaml`
- [x] `localization_pkg/scripts/gicp_localizer_node.py` 実装済み
- [x] `localization_pkg/launch/fast_lio_localization.launch.py` 実装済み
- [x] `localization_pkg/config/gicp_localizer.yaml` 設定済み
- [x] `tools/setup_fast_lio.sh` 作成済み（Phase 2 用）
- [x] `tools/record_semantic_bag.sh` 作成済み（セマンティック用）
- [x] `/usr/bin/python3 -m pip install --user --force-reinstall "numpy==1.24.4" ... "open3d==0.19.0"` 実行済み
- [x] `colcon build --packages-select localization_pkg` 成功
- [x] `tools/open_fast_lio2_loc_terminals.sh --mode real --cleanup` 起動確認
- [x] `/gicp_loc/score` が 0.5 以下で安定（確認値: 約 0.32）
- [x] `map → odom` TF が出ている
- [x] RViz で `/gicp_loc/pose` が PLY overlay と一致

---

## Step 7: Nav2 Navigation（FAST-LIO2 + GICP localization 使用）

FAST-LIO2 が LiDAR+IMU odometry hint を出し、GICP localizer が `map → odom` TF を publish する。Nav2 はその TF を受け取って動く。  
起動後は初期位置パラメータ or RViz の `2D Pose Estimate` で初期位置を設定する。

**実装済み（2026-05-04、localization 更新 2026-05-09）**:  
`nav_pkg/launch/navigation.launch.py`, `nav_pkg/config/nav2_params.yaml`,  
`nav_pkg/scripts/nav_mode_switch_node.py`,  
`localization_pkg/launch/fast_lio_localization.launch.py`（新 localization）

### TF 分担

```text
map → odom             ← gicp_localizer_node（GICP prior-map matching）
odom → base_footprint  ← wheel odom (robot_odom_node)
base_footprint → base_link → livox_frame  ← URDF / robot_state_publisher
```

### Nav2 が必要とする入力

| 入力 | ソース | 備考 |
|---|---|---|
| `map → odom` TF | **gicp_localizer_node** | localization（旧: GLIM） |
| `/map` (OccupancyGrid) | `nav2_map_server` + `l402_2d_map_0503.yaml` | global costmap（経路計画用） |
| `/scan` (LaserScan) | `pointcloud_to_laserscan` | local costmap + global costmap センサー |
| `/wheel_odom` | `robot_odom_node` | Nav2 の odom 入力 |

### 実装済みファイル

| ファイル | 内容 |
|---|---|
| `src/nav_pkg/launch/navigation.launch.py` | Nav2 全スタック + nav_mode_switch_node |
| `src/nav_pkg/config/nav2_params.yaml` | RPP controller, costmap, planner 設定 |
| `src/nav_pkg/scripts/nav_mode_switch_node.py` | Button A AUTO / Button B MANUAL 切替 |
| `src/localization_pkg/launch/fast_lio_localization_live.launch.py` | map_server + GICP + pointcloud_to_laserscan + FAST-LIO2 |
| `tools/open_fast_lio2_loc_terminals.sh` | Phase 2 localization quick launcher |

#### `nav_pkg/launch/navigation.launch.py` の概要

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('nav_pkg'), 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ]
            }]
        ),
    ])
```

#### 3. `nav_pkg/config/nav2_params.yaml` 初期値

omni base 向け初期パラメータ（まず低速で確認する）：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.10
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      max_angular_accel: 1.0
      use_rotate_to_heading: false  # omni base は回転不要

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_footprint
      rolling_window: true
      width: 4
      height: 4
      resolution: 0.05
      robot_radius: 0.30          # 実機サイズより少し大きめ
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.40

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      robot_radius: 0.30
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.40

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /wheel_odom
    default_nav_to_pose_bt_xml: ""  # default BT

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    wait:
      plugin: "nav2_behaviors/Wait"
```

### 起動手順

**ワンコマンド（Terminator 分割ペイン）**:

```bash
cd /home/matsunaga-h/robot_ws
tools/open_nav2_terminals.sh

# 部屋のみ軽量版で起動する場合:
tools/open_nav2_terminals.sh --map maps/l402_2d_map_lite_0504.yaml

# clean map を明示する場合:
tools/open_nav2_terminals.sh --map maps/l402_2d_map_clean_0509.yaml

# 古いプロセスをクリーンアップしてから起動:
tools/open_nav2_terminals.sh --cleanup
```

**手動起動（個別端末）**:

```bash
# Terminal 1: robot base / sensors（Joy-Con 自動起動）
ros2 launch robot_bringup test_all.launch.py rviz:=false

# Terminal 2: FAST-LIO2 + GICP + map_server + pointcloud_to_laserscan
ros2 launch localization_pkg fast_lio_localization_live.launch.py \
  map:=/home/matsunaga-h/robot_ws/maps/l402_2d_map_0503.yaml \
  with_fast_lio:=true \
  use_fast_lio_hint:=true

# Terminal 3: Nav2 + nav_mode_switch_node（A=AUTO / B=MANUAL）
ros2 launch nav_pkg navigation.launch.py

# Terminal 4: RViz
rviz2 -d /home/matsunaga-h/robot_ws/rviz/nav2_navigation.rviz
```

起動後の操作順序：

1. RViz の **2D Pose Estimate** で GICP 初期位置を設定する（必要な場合）
2. `ros2 run tf2_ros tf2_echo map odom` で GICP の `map → odom` が出ていることを確認
3. Joy-Con の **A ボタン**を押して AUTO モードへ切替
4. RViz の **2D Goal Pose** でゴールをクリック → 自律走行開始
5. 緊急時は **B ボタン**を押すと即座に MANUAL モードへ戻る

### FAST-LIO2 + GICP localization の収束確認

FAST-LIO2 は LiDAR+IMU odometry を出し、GICP localizer は固定 PLY map に scan-to-map matching して `map → odom` を publish する。  
ロボットが初期位置から大きく離れている場合は、RViz の `2D Pose Estimate` で robot pose を map 上に与える。

```bash
# GICP が map->odom を出しているか確認
ros2 run tf2_ros tf2_echo map odom

# GICP score。0.5 以下なら localization OK の目安
ros2 topic echo /gicp_loc/score --once

# FAST-LIO2 odometry
ros2 topic echo /fast_lio/odometry --once --field header

# 全チェーン確認
ros2 run tf2_ros tf2_echo map base_footprint

# Nav2 lifecycle 確認
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

`map → odom` が安定してから Nav2 ゴールを送る。

### Button A/B: 手動・自動モード切替（安全機能）

Joy-Con の **A ボタン（Joy index 0）** で AUTO、**B ボタン（Joy index 1）** で MANUAL にする。自律走行中でも B で即座に手動に戻れる。

| モード | 動作 |
|---|---|
| **MANUAL**（起動時デフォルト） | teleop の `/cmd_vel` がそのままモーターへ届く。L1 を押しながらスティック操作で手動走行。 |
| **AUTO** | `nav_mode_switch_node` が `/nav2/cmd_vel` を `/cmd_vel` に中継。Nav2 が自律走行。 |

```text
A ボタン → AUTO モードへ。RViz の 2D Goal Pose でゴール送信
B ボタン → MANUAL モードへ（Nav2 ゴール即キャンセル + ゼロ速度発行）
```

`/robot_mode` トピック（std_msgs/String）で現在のモードを確認できる。

```bash
ros2 topic echo /robot_mode
```

**実装ファイル**: `src/nav_pkg/scripts/nav_mode_switch_node.py`

**物理的な緊急停止ボタン（ロボット本体）**: `~/pickup_ws` の実装を確認したところ、Joy-Con レベルの emergeny stop 実装は存在しない。ロボット本体の赤いボタンはハードウェアレベルの停止であり、ROSとは独立している（→ そのままで問題なし）。

### トピックフロー

```
Joy-Con  → /cmd_vel → omni_base_driver (MANUAL 時、L1 dead-man)
Nav2 Goal → planner → controller → /nav2/cmd_vel
             ↓
     nav_mode_switch_node (AUTO 時のみ中継)
             ↓
           /cmd_vel → omni_base_driver

GICP   → map→odom TF  ─┐
wheel  → odom→base_footprint TF  ─┤→ Nav2 knows robot position in map
URDF   → base_footprint→base_link→livox_frame  ─┘

/scan (pointcloud_to_laserscan) → local costmap / global costmap
```

### RViz 操作

```text
Fixed Frame: map
rviz/nav2_navigation.rviz を使う

2D Pose Estimate:
  GICP localization の初期位置設定に使う。
  /l402_glim_points の PLY overlay は固定表示で、2D Pose Estimate では動かさない。

目標指定: 2D Goal Pose でクリック → global path が表示される
```

GICP score と `map → odom` が安定してから Nav2 ゴールを送る。

### Navigation 前の安全確認

```bash
ros2 topic echo /wheel_odom --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /scan
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

### ✅ Step 7 確認リスト

- [x] `nav_pkg/launch/navigation.launch.py` を実装した
- [x] `nav_pkg/config/nav2_params.yaml` を作成した（初期速度: 0.10 m/s、omni base 向け）
- [x] `nav_pkg/scripts/nav_mode_switch_node.py` を実装した（A=AUTO / B=MANUAL）
- [x] `colcon build --packages-select nav_pkg localization_pkg` でビルドが通る
- [x] Nav2 用 2D map 生成: `maps/l402_2d_map_0503.yaml`
- [x] FAST-LIO2 + GICP が起動できる
- [x] GICP が `map → odom` TF を publish している
- [x] `/gicp_loc/score` が低い値で安定する
- [ ] `map_server` が `/map` を publish している（`lifecycle: active`）
- [ ] `/scan` が約 10 Hz で流れている
- [ ] Nav2 の全ノードが `lifecycle: active` 状態になる
- [ ] `map → odom → base_footprint → base_link` が繋がる
- [ ] `ros2 topic echo /robot_mode` で "MANUAL" が出る
- [ ] A ボタンで `/robot_mode` が "AUTO" に切り替わる
- [ ] `2D Nav Goal` で global path が表示される
- [ ] AUTO 時に `/cmd_vel` が publish され、robot base が低速で反応する
- [ ] 自律走行中に B ボタンを押すと即座に停止して MANUAL に戻る
- [ ] 目標点到達後に停止する

---

## Step 8: パラメータ調整

Step 8 は、まず低速・短距離で調整する。最初は安全のため速度をかなり低くする。

### costmap 調整

```bash
# costmap のパラメータ確認
ros2 param list /local_costmap/local_costmap
ros2 param list /global_costmap/global_costmap
```

主要パラメータ：

| パラメータ | 目安 | 説明 |
|---|---|---|
| `inflation_radius` | 0.3〜0.5 m | 障碍物周りのマージン |
| `robot_radius` | 実機寸法 | ロボット半径 |
| `obstacle_range` | 2.5 m | 障碍物検出距離 |
| `raytrace_range` | 3.0 m | 障碍物クリア距離 |
| `resolution` | 0.03〜0.05 m | costmap 解像度 |

### planner / controller 調整

| コントローラー | 調整パラメータ |
|---|---|
| DWB | `max_vel_x`, `max_vel_theta`, `sim_time` |
| Regulated Pure Pursuit (RPP) | `desired_linear_vel`, `lookahead_dist` |

初期速度の目安：

```text
max_vel_x: 0.05 - 0.10 m/s
max_vel_y: 0.05 - 0.10 m/s  # omni base を使う場合
max_vel_theta: 0.2 - 0.4 rad/s
acc_lim_x/y: small enough to avoid wheel slip
```

`l402_2d_map_0503` を使う場合の注意：

- GLIM Viewer export PLY から 2D map に投影すると、床・天井・高い棚が混ざる可能性がある → `--z-min` / `--z-max` で調整する
- robot footprint / radius は実機サイズより少し大きめにする（現在 0.35 m）
- LiDAR の高さと blind area を考慮して、近距離障碍物は別 sensor 追加も検討する

調整中に見る topic：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /wheel_odom --once
ros2 topic hz /livox/lidar
ros2 topic list | grep costmap
```

### ✅ Step 8 確認リスト

- [ ] 低速で `/cmd_vel` と `/wheel_odom` の向き・スケールが一致する
- [ ] costmap に壁・障碍物が正しい位置で出る
- [ ] robot footprint / radius が実機より小さすぎない
- [ ] 狭い通路でも壁に当たらず通過できる
- [ ] 旋回がスムーズ
- [ ] カーブ経路で脱線しない
- [ ] recovery behavior が機能する

---

## 推奨開発順序（まとめ）

```text
Step 1: TF・トピック確認 (test_all.launch.py で全体起動)
Step 2: Joy-Con teleop 追加・手動操作確認
Step 3: IMU データ確認 (/livox/imu frame_id・hz・値)
Step 4: GLIM で 3D SLAM 建図 (LiDAR + IMU)
Step 5: 地図保存 (3D pcd / 2D pgm)
Step 6: localization (FAST-LIO2 + GICP prior-map localization)
Step 7: Nav2 で自律ナビゲーション
Step 8: costmap / planner / controller パラメータ調整
```

---

## 参考：既存ファイル

| ファイル | 内容 |
|---|---|
| `src/robot_bringup/launch/test_all.launch.py` | 全ハードウェア起動（LiDAR, Camera, Drive/Steer motors） |
| `src/robot_bringup/launch/bringup.launch.py` | センサーのみ起動 |
| `src/omni_base_driver/src/cmd_vel_to_motor_node.cpp` | /cmd_vel → steer_ang + drive_vel |
| `src/omni_base_driver/src/robot_odom_node.cpp` | odom TF & /wheel_odom |
| `src/localization_pkg/launch/fast_lio_localization_live.launch.py` | map_server + GICP + pointcloud_to_laserscan + FAST-LIO2 |
| `src/nav_pkg/launch/navigation.launch.py` | Nav2 全スタック + nav_mode_switch_node |

---

## 追加: RealSense Camera Pan/Tilt Base（2026-05-04）

RealSense D435 を新しい可動式カメラ台座に載せ替えた。旧 `chokudo` / リニアモータは、カメラ台座の水平 360 deg pan motor として再利用する。`cameraswingmotor` は従来通りカメラの上下 tilt motor として使う。

### 構成

| 機能 | ROS topic | 役割 |
|---|---|---|
| Camera base pan | `/chokudomotor/target_angle`, `/chokudomotor/angle` | 水平方向、左右旋回 |
| Camera tilt | `/cameraswingmotor/target_angle`, `/cameraswingmotor/angle` | 上下スイング |
| Joy-Con input | `/joy` | LB dead-man + D-pad 操作 |

実装ファイル：

| ファイル | 内容 |
|---|---|
| `src/serial_transciever/serial_transciever/manipulator_control/camera_motor_joy_node.py` | `/joy` を読み、pan/tilt の目標角を publish |
| `src/serial_transciever/setup.py` | `camera_motor_joy_node` entry point 追加 |
| `src/robot_bringup/launch/test_all.launch.py` | Joy-Con 自動起動 + camera motor joy node 起動 |
| `src/robot_bringup/launch/teleop.launch.py` | `joy_node` + `teleop_twist_joy_node`（`test_all` から include） |

### Joy-Con 操作

| 操作 | 動作 |
|---|---|
| LB / L1 を押しながら操作 | camera motor control 有効 |
| D-pad 左右 | camera base pan（旧 chokudo motor） |
| D-pad 上下 | camera tilt（cameraswing motor） |

`camera_motor_joy_node.py` は内部 target angle を連続更新する。feedback angle を毎回基準にしないため、pan/tilt が断続的になりにくい。現在の pan は `0.03 sec x 2.5 deg`、tilt は `0.03 sec x 5 deg` を基本値にしている。

### 初期角度

現在の default startup target：

```text
camera_initial_pan_angle: 267.0
camera_initial_tilt_angle: 102.0
```

無効化したい場合：

```bash
ros2 launch robot_bringup test_all.launch.py \
  camera_initial_pan_angle:=nan camera_initial_tilt_angle:=nan
```

手動 publish 例：

```bash
ros2 topic pub --once /chokudomotor/target_angle std_msgs/msg/Float32 "{data: 267.0}"
ros2 topic pub --once /cameraswingmotor/target_angle std_msgs/msg/Float32 "{data: 102.0}"
```

### 起動方法

camera + LiDAR + RViz + camera pan/tilt のみ起動し、robot movement motor は起動しない：

```bash
ros2 launch robot_bringup test_all.launch.py \
  camera:=true lidar:=true rviz:=true \
  drive_motors:=false steer_motors:=false \
  serial_motors:=true camera_motor_joy:=true \
  static_odom:=true
```

Joy-Con は `test_all.launch.py` から自動起動するため、通常は別 terminal で `teleop.launch.py` を起動しなくてよい。device が `js1` の場合：

```bash
ros2 launch robot_bringup test_all.launch.py joy_dev:=/dev/input/js1
```

Joy-Con を自動起動しない場合：

```bash
ros2 launch robot_bringup test_all.launch.py joy:=false
```

### 確認コマンド

現在角：

```bash
ros2 topic echo --once /chokudomotor/angle
ros2 topic echo --once /cameraswingmotor/angle
```

Joy-Con mapping：

```bash
ros2 topic echo /joy
```

launch argument 確認：

```bash
ros2 launch robot_bringup test_all.launch.py --show-args
```

### Navigation への将来接続

この camera pan/tilt base は、将来 Nav2 Navigation と連携して以下に使う予定：

- 近距離障碍物認識（LiDAR blind area 補完）
- 物体認識、target tracking
- camera tilt による床面 / 低い障碍物 / 前方物体の確認
- 必要に応じて depth pointcloud を costmap または perception node に入力

将来の追加候補：

- pan/tilt angle を TF に反映する joint_state publisher または dynamic TF
- RealSense depth を `pointcloud_to_laserscan` または costmap obstacle layer に接続
- Navigation mode と連動した camera scanning behavior
- object detection 結果を Nav2 goal / behavior tree に渡す
