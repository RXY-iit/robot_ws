# SO101 joint/kinematic + eye-in-hand calibration capture

这是一个只读数据采集工具。它不创建任何 Arm command publisher，也不启动或占用
Arm controller、Arm camera。Arm 必须已由正常 bringup 启动，操作者只通过 Dashboard
的 `teleoperate` mode 移动真实 Arm。

## 它保存什么

每次按 RViz 的 `Capture stable sample` 后，只有下列直接证据全部有效才会保存：

- `/arm_camera/color/image_raw` 原图、检测标注图；
- `/arm_camera/color/camera_info` 完整内参；
- 实际尺寸为 square 33.8 mm、marker 16.5 mm、`DICT_5X5_100` 的 7×5 ChArUco 角点；
- `camera_from_board` 的逐帧 `solvePnP` 观测值（不直接写入 URDF）；
- `/joint_states` 六关节实测弧度及保存前 5 秒轨迹；
- `/arm/hardware_status` 六电机 `raw_position`、完整状态快照及保存前 5 秒 raw 轨迹；
- `/tf`、`/tf_static` 当时已观察到的完整快照；
- 关节稳定窗口、消息时间戳、接收 age、workspace Git revision；
- 操作者填写的 `board_setup_id` 和 pose label。

输出位于 `output/session_*/sample_*`，`manifest.jsonl` 是整个 session 的逐样本索引。

## Terminal commands

先用正常真机 bringup 启动 Arm、Arm camera 和 Dashboard，并在 Dashboard 选择
`teleoperate`。不要同时启动旧的 camera calibration tool，以免相机设备被重复占用。

首次构建或修改后：

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13

colcon build --packages-select so101_kinematic_handeye_calibration --symlink-install
source install/setup.bash
```

启动采集器和 RViz：

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13

ros2 launch so101_kinematic_handeye_calibration capture.launch.py \
  output_root:=/home/matsunaga-h/robot_ws/tools/so101_kinematic_handeye_calibration/output
```

## 采集步骤

1. 把 ChArUco 板刚性固定；一次 `board_setup_id` 内绝对不能移动板或 Arm base。
2. 在 RViz 检查真实 Arm model、TF 和 Arm camera 图像。
3. 用 `teleoperate` 把 Arm 移到一个姿态，然后停止操作至少 1 秒。
4. 保证清晰看到至少 8 个 ChArUco 内角点。
5. 输入 `board_setup_id`（例如 `board_A`）和 pose label，点击保存。
6. 改变 Arm 姿态并重复。若移动了标定板，必须改用新的 `board_setup_id`。

建议每个固定板位采集 20–30 个姿态，总计至少 40–60 个有效样本：

- 覆盖 shoulder、elbow、wrist 的有效工作区，不只采关键极限点；
- wrist/末端绕至少两个不同轴产生明显旋转，避免手眼标定退化；
- 同一空间位置尽量用不同关节构型到达；
- 对主要关节从增加、减少两个方向接近若干相同姿态，用于识别回差；
- 避免所有图像都正对标定板，覆盖画面中心和四周、近中远距离；
- 不采关节仍在移动、画面模糊、板弯曲或严重反光的样本。

## 数据能解决什么、不能直接证明什么

同一固定板位的一组观测可联合估计关节零偏、部分 DH/URDF 运动学参数和
`gripper/wrist -> arm_camera` eye-in-hand 外参。公开模型只可作为先验；求解结果仍需
留出独立验证姿态做交叉验证。仅靠这些图像通常不能可靠区分所有连杆长度误差、轴线
倾斜、齿隙和柔性形变，因此求解时应只放开可辨识参数，并对机械尺寸加入测量先验。

本工具当前只负责采集和可审计记录，不会自动覆盖 URDF 或 motor calibration。
