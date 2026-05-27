# Isaac Sim PC After Clone Guide

Last updated: 2026-05-22

目的: Isaac Sim PC に `robot_ws` を clone したあと、real robot と同じ制御系で sim robot を動かすまでの順序を迷わないようにする。

## 0. 基本方針

```text
Isaac Sim をもう一台の real robot として扱う。
topic 名・Joy-Con 操作・Nav2 chain は real と同じにする。
ROS_DOMAIN_ID=20 で real robot DOMAIN_ID=13 と隔離する。
```

最初の目標は Nav2 ではなく、Joy-Con で Isaac Sim robot を手動走行できること。

## 1. Clone 直後

```bash
cd ~
git clone <YOUR_REPO_URL> robot_ws
cd ~/robot_ws
git checkout main
git pull
git tag --list | grep real-nav-baseline
```

基準 tag に戻したい場合:

```bash
git checkout real-nav-baseline-20260522
```

通常作業は branch を切る:

```bash
git checkout -b sim-dev/isaac-first-run
```

## 2. ROS 環境

```bash
conda deactivate
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
echo $ROS_DOMAIN_ID
```

真機 topic が見えないことを確認:

```bash
ros2 topic list
```

Isaac Sim 起動前ならほぼ空でよい。`/livox/lidar` など real robot の topic が見えたら DOMAIN が混ざっている。

## 3. Build

まず依存:

```bash
rosdep install --from-paths src -y --ignore-src
```

全体 build:

```bash
colcon build --symlink-install
source install/setup.bash
```

hardware driver 依存で詰まる場合は core だけ:

```bash
colcon build --symlink-install --packages-select \
  robot_description my_messages tf_tools \
  localization_pkg nav_pkg safety_layer mission_manager \
  fast_lio livox_ros_driver2
source install/setup.bash
```

## 4. URDF Export

```bash
cd ~/robot_ws
xacro src/robot_description/urdf/robot.urdf.xacro > /tmp/robot_expanded.urdf
```

Isaac Sim GUI:

```text
File -> Import URDF -> /tmp/robot_expanded.urdf
保存先: ~/robot_ws/src/robot_sim/usd/robot_sim.usd
```

確認ポイント:

```text
wheel_right_steer / wheel_left_steer / wheel_back_steer
wheel_right_drive / wheel_left_drive / wheel_back_drive
base_link / base_footprint / livox_frame / camera_link
```

## 5. Isaac Sim ROS2 Bridge Topic

最初に real と同名で出したい topic:

```text
/wheel_odom                         nav_msgs/Odometry
/tf                                 odom -> base_footprint
/livox/imu                          sensor_msgs/Imu
/livox/lidar                        livox_ros_driver2/msg/CustomMsg
/camera/camera/color/image_raw      sensor_msgs/Image
/camera/camera/color/camera_info    sensor_msgs/CameraInfo
/camera/camera/depth/image_rect_raw sensor_msgs/Image
/camera/camera/depth/camera_info    sensor_msgs/CameraInfo
```

もし Isaac Sim が `/livox/lidar` を PointCloud2 でしか出せない場合:

```text
robot_sim 側に PointCloud2 -> Livox CustomMsg adapter を作る。
FAST-LIO2 は CustomMsg が必要。
GICP / scan / RViz は relay 後の /livox/lidar_pc2 を使う。
```

## 6. 最初の実行目標: Joy-Con 手動走行

必要 chain:

```text
Joy-Con -> /joy
teleop_twist_joy -> /teleop/cmd_vel
nav_mode_switch_node MANUAL -> /cmd_vel_raw
cmd_vel_safety_node -> /cmd_vel
Isaac Sim omni_drive_controller -> wheel joints
```

期待操作:

```text
L1 = manual deadman
A  = AUTO
B  = MANUAL / cancel
Y  = E-STOP toggle
```

確認:

```bash
ros2 topic info -v /joy
ros2 topic echo /robot_mode --once
ros2 topic echo /cmd_vel
ros2 topic hz /wheel_odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

通過条件:

```text
MANUAL mode で Joy-Con 操作により /cmd_vel が出る。
Isaac Sim robot が前後・左右・旋回する。
/wheel_odom と odom -> base_footprint が実際の移動方向と一致する。
```

## 7. 次の順序

1. Joy-Con 手動走行を通す。
2. 手動走行しながら sim 環境 map 用 bag を記録する。
3. 後続 task で sim 3D map と Nav2 2D map を作る。
4. `FAST-LIO2 + GICP` を `with_fast_lio:=true` で起動する。
5. `navigation.launch.py` を real と同じ設定で起動する。
6. MANUAL で goal を置き、A で AUTO にして Nav2 を試す。

## 8. Sim Bag 記録

```bash
mkdir -p ~/robot_ws/sim_bag
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
  -o ~/robot_ws/sim_bag/sim_mapping_$(date +%Y%m%d_%H%M%S)
```

`sim_bag/` は git に入れない。

## 9. 注意点

```text
real map と sim map を混ぜない。
real bag と sim bag を同じ起動中に replay しない。
static map -> odom と GICP map -> odom を同時に出さない。
camera_init/body は FAST-LIO 内部 frame。Nav2 主 TF ではない。
use_sim_time は最初 false のまま試す。
TF timestamp error が連続した場合だけ use_sim_time 対応を検討する。
```

主 TF:

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

主 topic:

```text
/livox/lidar      CustomMsg, FAST-LIO2 input
/livox/lidar_pc2  PointCloud2, GICP / scan / RViz
/wheel_odom       Nav2 odom topic
/cmd_vel          final command to Isaac Sim controller
```
