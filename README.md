# robot_ws Quick Note

Last updated: 2026-05-22

当前默认基线:

```text
FAST-LIO2 + GICP localization -> Nav2 MPPI -> mode switch -> safety layer -> omni base
```

定位理解:

```text
GICP-only: Livox PointCloud2 与保存地图匹配，发布 map -> odom。
FAST-LIO2: Livox CustomMsg + IMU 估计短周期 LiDAR-IMU odometry。
当前主线: FAST-LIO2 给 GICP 初始运动 hint，GICP 仍负责地图修正和最终 map -> odom。
```

最常用启动:

```bash
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
tools/open_nav2_windows.sh --cleanup
```

## 0. 基本准备

每个新 terminal 先做:

```bash
conda deactivate
cd /home/matsunaga-h/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Python 必须是 ROS Humble 的 3.10:

```bash
which python3
python3 --version
```

## 1. 常用命令

### 当前真机 Nav2

正常启动，默认 FAST-LIO on:

```bash
tools/open_nav2_windows.sh --cleanup
```

GICP-only fallback:

```bash
tools/open_nav2_windows.sh --no-fast-lio --cleanup
```

不启动真实移动电机的安全检查:

```bash
tools/open_nav2_windows.sh --phase1-safe --cleanup
```

检查 Nav2 graph:

```bash
tools/check_nav2_phase1.sh
```

打包最新 debug:

```bash
tools/pack_latest_nav2_debug.sh
```

### 关键 topic 检查

FAST-LIO + relay:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/lidar_pc2
ros2 topic info -v /fast_lio/odometry
```

频率:

```bash
ros2 topic hz /fast_lio/odometry
ros2 topic hz /livox/lidar_pc2
ros2 topic hz /cmd_vel
```

TF:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

### RViz 显示规则

```text
/livox/lidar       CustomMsg, FAST-LIO input, RViz 不直接显示
/livox/lidar_pc2   PointCloud2, RViz/GICP/scan 用
/fast_lio/cloud_registered  FAST-LIO 输出点云
```

`camera_init` / `body` 是 FAST-LIO 内部 frame，不接入 Nav2 主 TF 树；Nav2 只看 `map -> odom -> base_footprint -> base_link -> livox_frame`。

### Phase 2 camera costmap 调整

只启动 camera + local costmap + RViz:

```bash
tools/open_phase2_camera_costmap_windows.sh --cleanup
```

带 LiDAR 对齐显示:

```bash
tools/open_phase2_camera_costmap_windows.sh --cleanup --lidar
```

### 记录 rosbag

Semantic / 后续 AI 用 rosbag:

```bash
tools/record_semantic_bag.sh
```

启动 localization 后再录:

```bash
tools/record_semantic_bag.sh --start-phase2-loc
```

### Build / restart 规则

改 `tools/*.sh`: 不用 build，重新运行脚本。

改 `nav2_params.yaml`: 通常不用 build，重启 Nav2/navigation 窗口。

改 URDF/xacro:

```bash
colcon build --packages-select robot_description
source install/setup.bash
```

改 package 内 C++/Python/launch/config，稳妥做法:

```bash
colcon build --packages-select nav_pkg localization_pkg robot_bringup robot_description --symlink-install
source install/setup.bash
```

## 2. Step1-7 速查

### Step 1: TF / 基本硬件

目标: 确认 TF、USB、基础 topic 都正常。

```bash
ros2 launch robot_bringup test_all.launch.py
```

常查:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /wheel_odom --once
ros2 topic echo /livox/imu --once
```

### Step 2: Joy-Con / 手动移动

目标: 确认手柄和 `/cmd_vel` 控制链路。

```bash
ros2 launch robot_bringup teleop.launch.py
```

当前 Nav2 中:

```text
A = AUTO
B = MANUAL / cancel goal
Y = E-STOP toggle
L1 = manual deadman
```

### Step 3: Odom / Motor

目标: 确认 `/drive_odom + /steer_odom -> /wheel_odom`。

```bash
ros2 topic echo /drive_odom --once
ros2 topic echo /steer_odom --once
ros2 topic echo /wheel_odom --once
ros2 topic hz /om_response0
```

如果有 `diff_pos[...] exceeds threshold`，优先查 motor/steer feedback。

### Step 4: GLIM 建图

目标: 只用于创建或更新地图，当前在线导航不再用 GLIM localization。

一键建图 + teleop + rosbag record:

```bash
tools/create_glim_map_quickstart.sh --bag l402_new_mapping --cleanup
```

手动启动 GLIM:

```bash
ros2 run glim_ros glim_rosnode --ros-args \
  -p config_path:=/home/matsunaga-h/robot_ws/glim_config
```

建图同时录 rosbag，必要 topic:

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
  /chokudomotor/angle \
  /cameraswingmotor/angle \
  -o slam_bag/l402_new_mapping
```

GLIM 停止后保存 `/tmp/dump`:

```bash
ls -lah /tmp/dump
mkdir -p /home/matsunaga-h/robot_ws/maps
cp -a /tmp/dump /home/matsunaga-h/robot_ws/maps/glim_dump_$(date +%Y%m%d_%H%M%S)
```

用 GLIM viewer 查看保存 map:

```bash
ros2 run glim_ros offline_viewer \
  --map_path /home/matsunaga-h/robot_ws/maps/l402_glim_map_0503 \
  --config_path /home/matsunaga-h/robot_ws/glim_config_nav2
```

### Step 5: 地图保存 / 2D map

目标: 从 PLY 生成 Nav2 2D map。

```bash
/usr/bin/python3 tools/ply_to_nav2_map.py \
  maps/saved-map/map-l402-0503/l402_points_0503 \
  --output maps/l402_2d_map_new \
  --resolution 0.05 \
  --z-min 0.05 \
  --z-max 1.20 \
  --inflate-radius 0.12
```

RViz overlay:

```bash
tools/publish_l402_overlay.sh live
```

### Step 6: Localization

目标: 当前基线是 FAST-LIO2 odom hint + GICP prior-map `map -> odom`。

一句话: FAST-LIO2 用 LiDAR+IMU 让短时间运动估计更顺，GICP 用保存地图把位置拉回全局坐标。

单独测试 localization:

```bash
tools/open_fast_lio2_loc_terminals.sh --mode real --cleanup
```

当前 Nav2 启动会自动包含 localization:

```bash
tools/open_nav2_windows.sh --cleanup
```

### Step 7: Nav2 Navigation

目标: MPPI + local costmap + safety layer 完成真机导航。

```bash
tools/open_nav2_windows.sh --cleanup
```

测试流程:

```text
1. 等 RViz / map / costmap / robot model 正常
2. 保持 MANUAL，设置 2D Goal Pose
3. 按 A 进入 AUTO
4. 观察 /nav2/cmd_vel, /cmd_vel, latest_state.md
5. 必要时按 B 回 MANUAL 或 Y E-STOP
```

最新通过记录:

```text
debug-output/nav2_20260522_110151
debug-output/nav2_20260522_150715
Goal succeeded, recoveries=0
```

## 3. tools/ 文件说明

### 当前常用

| 文件 | 用途 |
| --- | --- |
| `open_nav2_windows.sh` | 当前主启动脚本，默认 FAST-LIO + GICP + Nav2 + RViz + debug。 |
| `check_nav2_phase1.sh` | 启动后检查 Nav2 节点、topic、TF、cmd_vel authority。 |
| `pack_latest_nav2_debug.sh` | 把最新 `debug-output/nav2_*` 打包给调试用。 |
| `open_phase2_camera_costmap_windows.sh` | 只启动 camera/local costmap/RViz，用来调 RealSense obstacle 参数。 |
| `record_semantic_bag.sh` | 记录后续 semantic/AI mapping 用 rosbag。 |
| `check_robot_urdf.sh` | 检查 URDF、robot_state_publisher、sensor TF。 |
| `open_fast_lio2_loc_terminals.sh` | 单独测试 FAST-LIO2 + GICP localization。 |
| `publish_l402_overlay.sh` | 在 RViz 发布保存的 L402 PLY map overlay。 |

### Debug helper

| 文件 | 用途 |
| --- | --- |
| `nav2_debug_recorder.py` | 记录 Nav2 状态到 `events.jsonl` 和 `latest_state.md`。 |
| `nav2_debug_monitor.py` | 终端实时显示 goal/plan/odom/cmd_vel 简要状态。 |
| `watch_nav2_debug.sh` | 启动 `nav2_debug_monitor.py` 的 shell wrapper。 |
| `fake_wheel_odom.py` | phase safe 模式下发布静止 `/wheel_odom`。 |

### Map / pointcloud

| 文件 | 用途 |
| --- | --- |
| `create_glim_map_quickstart.sh` | GLIM 建图和 rosbag 记录一键窗口。 |
| `ply_to_nav2_map.py` | PLY 点云转 Nav2 2D occupancy map。 |
| `pointcloud_to_nav2_map.py` | 从 live PointCloud2 累积生成 2D map。 |
| `publish_ply_pointcloud.py` | 低层 PLY -> PointCloud2 发布器。 |
| `interactive_ply_initialpose_publisher.py` | RViz 里用 `/initialpose` 调整 PLY overlay。 |
| `visualize_glim_anchor_poses.py` | 显示 GLIM/PLY anchor pose marker。 |

### Calibration / bag tools

| 文件 | 用途 |
| --- | --- |
| `lidar_tf_tuner.py` | 离线用 bag + PLY 调 LiDAR TF。 |
| `filter_livox_tf_bag.py` | 清理 rosbag 里冲突 TF，例如错误的 `livox_frame` edge。 |
| `auto_initial_align.py` | 实验性自动估计初始 map->odom。 |

### Legacy / fallback

| 文件 | 用途 |
| --- | --- |
| `open_gicp_loc_terminals.sh` | GICP localization 老入口或 rosbag fallback。 |
| `open_glim_loc_terminals.sh` | GLIM 名称兼容 wrapper，实际转到 GICP launcher。 |
| `open_nav2_terminals.sh` | Terminator 版旧 Nav2 启动；现在优先用 `open_nav2_windows.sh`。 |
| `setup_fast_lio.sh` | 初次安装/构建 FAST-LIO2 用，通常只跑一次。 |

## 4. todo-my/ 文件说明

| 文件 | 内容 |
| --- | --- |
| `step7-stauts.md` | 当前 Step7 最新状态和下一步，优先读这个。 |
| `status-0425.md` | 从 4/25 到现在的主线进度摘要。 |
| `guidence-0425.md` | 最完整的 Step1-7 长文档，细节多。 |
| `phase2-verification-guide.md` | RealSense depth / local costmap 验证步骤。 |
| `phase3-4-verification-guide.md` | MPPI、安全层、Mission Manager 验证步骤。 |
| `step7-navigation-ai-simulation-plan.md` | Phase1-5、simulation、AI/LLM 接入计划。 |
| `ai-navigation-guide.md` | Navigation + Semantic + Mission + LLM 架构讨论。 |

## 5. 常见问题

### `/livox/lidar` 在 RViz 看不到

正常；现在它是 CustomMsg。看 `/livox/lidar_pc2`。

### RViz TF 里 `camera_init/body` warning

正常；这是 FAST-LIO 内部 odometry frame，不要为了变绿而加假的 `map -> camera_init`。

### `/fast_lio/odometry` 没有 publisher

先查:

```bash
ros2 topic info -v /livox/lidar
ros2 topic info -v /livox/lidar_pc2
ros2 topic info -v /fast_lio/odometry
```

期待:

```text
/livox/lidar       CustomMsg
/livox/lidar_pc2   Publisher count: 1
/fast_lio/odometry Publisher count: 1
```

### Nav2 不动或全 0

先看 debug:

```bash
cat debug-output/$(ls -t debug-output | head -1)/latest_state.md
```

再检查:

```bash
tools/check_nav2_phase1.sh
ros2 topic info -v /cmd_vel
ros2 topic echo /robot_mode --once
```

### 改参数后不知道要不要 build

简单规则:

```text
tools/*.sh: 不 build
YAML: 重启相关 launch
URDF/xacro: build robot_description + 重启 bringup
C++: 必须 colcon build
Python package script: 建议 colcon build + source
```
