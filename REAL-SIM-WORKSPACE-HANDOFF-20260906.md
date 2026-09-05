# Real/Sim Workspace Handoff — 2026-09-06

用途：把当前真机验证版本同步到另一台 Isaac Sim 开发机，并以同一份 ROS 2/URDF/控制/Touch 参数复现 real 行为。本文供新环境中的 Codex 和开发者优先阅读。

## 1. Git 同步范围

本次提交包含可复现系统所需的源码、配置、RViz、硬件说明、地图和工具；不包含：

- `todo-my/`：本机调查、运行记录和阶段总结；
- `think-latter/`：后续构想；
- `debug-output/`、ROS bags、标定 `output/`：体积大且包含本机运行证据；
- `models/`、SAM2 third-party/runtime：需在目标机按工具说明安装；
- `build/`、`install/`、`log/`、`node_modules/`、cache。

目标机应从本提交重新构建，不能复制旧 `install/`。

## 2. 关键新增目录

### `hardware/`

真机硬件事实与接口依据：

- `blv_r_drive_motors.md`：3 台 BLVD-KRD/GFS5G30FR、Modbus ID、寄存器健康信息；
- `dynamixel_steer_motors.md`：转向 DYNAMIXEL 电压/电流/温度信息；
- `e-stop.md`：真实红色急停/BLVD 输入证据；
- `livox_mid360.md`、`realsense_d435.md`：设备级健康和相机来源；
- `lift_mechanism/`：Lift 当前控制、ABZO/RS-485 后续接口；
- `so-arm101.md`：SO101、Follower/Leader 与相机硬件；
- `motor-image/`、`real-bot-figure/`：铭牌和真实安装位置证据。

这些是现实硬件约束，不应由仿真中的方便假设覆盖。

### `src/robot_agent/`

Agent runtime 与 Web Dashboard：三层 robot health 汇总、七域 Health Dashboard、SO101 Arm 页面、真实状态 WebSocket、恢复/控制接口。Web 源码位于 `web/src`，生产构建由 `tools/build_dashboard_web.sh` 完成。

### `src/robot_skills/`

技能注册、semantic map、navigation/localization/perception executor、真实 camera poses 和策略配置。修复了同一 Agent 内部多个 executor publisher 被误判为重复 Agent 的逻辑。

### `src/so101_arm_control/`

SO101 mock/real/read-only/teleoperate/code2real&sim 控制系统：

- calibration-bound raw/joint 映射；
- guarded Preview/Execute gateway；
- HOLD、软限位、温度/电流/反馈保护；
-真实 Follower、Leader teleoperation 与 Dashboard 状态；
- Arm mount TF、真实/候选 RViz 模型和近场 IK 接口。

### `src/so_arm101_description/`

SO101 meshes、visual URDF、真实 Arm Camera frame、real/candidate RobotModel。mesh 来自公开模型，但部署 TF、轴向和关节映射以真机测量/标定为准。

### `src/touch_anything/`

Touch Anything V1：D435 人工 prompt、SAM 2.1、深度 3D、FAR、Arm Camera handoff、视觉闭环、NEAR tip alignment、Press、成功判断、RViz Panel 和运行证据。默认策略参数在 `config/touch_anything.yaml`。

### `src/robot_bringup/`、`src/nav_pkg/`、`src/safety_layer/`

整机启动、模式切换、Touch base lock、JoyCon/Keyboard/Dashboard authority、Nav2 lifecycle 与 camera local costmap。修复 command authority 歧义、Touch Mode 下底盘运动阻断和启动依赖。

### Driver/health changes

- `src/om_modbus_master_V201/`：BLVD 通信回调、错误处理和稳定退出；
- `src/omni_base_driver/`：drive/steer feedback 与命令链；
- `src/DynamixelSDK/.../read_write_node`：转向设备健康摘要；
- `src/livox_ros_driver2/`：设备状态码到 `/livox/device_health`；
- `src/serial_transciever/`：Pan/Tilt、Lift feedback/state 与 JoyCon LT/RT；
- `src/localization_pkg/`：GICP 直接状态、启动和点云频率调整。

### `rviz/`

`nav2_navigation.rviz` 增加 SO101 actual/candidate、Arm Camera、D435、Touch Anything Panel、目标/FAR/NEAR/tip/TF markers；同时保留 Nav2/localization 显示。RViz 只是可视化证据，不是碰撞规划器。

### `tools/`

- `open_nav2_windows.sh`：真机导航/运动策略/可选虚拟 Oriental Motor 启动；
- `run_agent_dashboard_real.sh`、`build_dashboard_web.sh`：Dashboard；
- `restart_drive_motor.sh`：带现场检查和日志的 Modbus/drive restart；
- `real-robot-quick-start.desktop`、`real_robot_quick_start.py`：记忆上次选项的 GUI 快捷启动；
- `fake_drive_motor.py`：无 Oriental Motor 测试模式；
- `camera_pan_tilt_calibration/`：D435/Pan/Tilt/Arm Camera ChArUco 数据采集与联合外参审核；
- `so101_kinematic_handeye_calibration/`：SO101 关节/运动学/eye-in-hand 数据采集和求解；
- `touch_anything/setup_sam21.sh`：目标机安装模型/runtime；
- Nav2/GICP/地图诊断和语义导入工具。

## 3. 为什么做这些更新

### Dashboard/Health

旧 Dashboard 会把间接 summary、publisher 数和高频 topic 当成健康事实，产生误报警并增加 CPU。新版区分 DIRECT/DERIVED/DOCUMENTED/NOT_INSTRUMENTED，Health Monitor 只以低频摘要驱动 UI；补充 drive、steer、DYNAMIXEL、Livox、RealSense、Nav2 lifecycle、GICP、Arm、主机 CPU/温度等直接证据。

### Drive/Modbus

处理 BLVD startup no-response、`om_modbusRTU_node` 异常退出、重复 publisher 误判、错误恢复后永久 DEGRADED 等问题；驱动摘要约 2–5 Hz，而不是让 Dashboard 消费数百 Hz raw state。真实 E-stop、每驱动 alarm、电压、电流和 callback deadline 被独立记录。

### Navigation/Perception

明确 Livox 原始设备健康独立于 GICP 定位健康；近障碍物主要来自 D435 local costmap，不虚构 Livox `/scan` 链路。提供 `--no-fast-lio --motion-policy` 等启动选择并保持命令 authority 可解释。

### Arm/Touch

加入真实 SO101 Follower 控制、Leader teleoperation、标定绑定、Arm Camera 和 Touch pipeline。Touch demo 已在多个高度真实执行成功，但仍保留完整保护与可审计证据，不将 RViz 显示或 SAM score 当作硬件到位证明。

## 4. Real/Sim 必须共同使用的基线

以下文件/参数直接来自真实测量或真机闭环，Isaac Sim 接入必须读取它们，不能复制后另改：

1. `src/robot_description/urdf/robot.urdf.xacro`：Pan/Tilt/Lift/Arm mount TF；
2. `src/so_arm101_description/urdf/`：SO101 links/joint axes/Arm Camera frame；
3. `src/so101_arm_control/config/arm_real_dashboard_test.yaml` 及控制代码：关节 calibration、限位、速度、HOLD；
4. `tools/so101_kinematic_handeye_calibration/arm_calibration_candidate.json`：标定审计结果；
5. `src/touch_anything/config/touch_anything.yaml` 和 `scripts/`：FAR/NEAR/Press、tip pixel、SAM 和证据阈值；
6. `src/robot_skills/config/real_camera_poses.yaml` 与 Touch reference seed；
7. `rviz/nav2_navigation.rviz`：统一观察 real/sim frame 和 candidate。

仿真应复现真实误差、反馈延迟和执行边界，再在上层算法改进；不要通过改变 URDF 或 Touch 阈值让 sim 单方面“成功”。如确需改基线，提交同一修改并安排真机回归。

## 5. 目标机恢复步骤

```bash
cd /path/to/robot_ws
git fetch origin
git switch real-test-usage
git pull --ff-only

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Touch Anything 的 SAM 2.1 模型/runtime（未纳入 Git）
tools/touch_anything/setup_sam21.sh

colcon build --symlink-install
source install/setup.bash
```

Dashboard Web 使用项目锁定依赖构建，不复制本机 `node_modules`：

```bash
cd src/robot_agent/web
nvm use 24.19.0
npm ci
npm run build
```

然后重新执行 `colcon build --packages-select robot_agent --symlink-install`。

## 6. 最低验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  src/touch_anything/test src/so101_arm_control/test

tools/check_robot_urdf.sh
```

在 Isaac Sim 侧至少检查：TF tree 无双 owner/断链；actual/candidate topic frame 一致；同一 joint vector 的 touch tip 和 Arm Camera pose 与 ROS RViz 一致；FAR->NEAR 单向状态转换；仿真执行反馈必须走与 real 相同 gateway，而不是绕过保护直接写 joint。

## 7. 本地但未推送的阅读材料

详细 Dashboard V1 和 Touch 演进记录位于本机 `todo-my/`；构想位于 `think-latter/`。它们按要求不进入 Git。需要迁移时应单独人工复制，不混入可执行基线提交。
