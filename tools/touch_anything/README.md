# Touch Anything operator notes

## Dependency setup

The current installation is pinned to Meta SAM 2 commit
`2b90b9f5ceec907a1c18123530e92e794ad901a4b5` and the official 2024-09-28
SAM 2.1 Hiera Tiny checkpoint. Recreate ignored local assets with:

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/setup_sam21.sh
```

The model is not loaded during bringup. The first RViz point prompt loads it on
CPU; it is retained for 180 seconds after the last inference and then released.

## Phase-1 workflow

1. Start the normal real-robot bringup and Dashboard.
2. In Arm, select `code2real&sim`.
3. Select `Touch Anything / 任意対象にタッチ` in Task triggers.
4. Confirm Dashboard reports `base=LOCKED` and the safety source reports
   `TOUCH_LOCK`.
5. In RViz `Touch Anything`, freeze the D435 frame.
6. Left-click the target. Right-click or select `Negative` to exclude a region.
7. Inspect the green mask, green 3-D point and orange surface-normal arrow.
8. Confirm the preview or exit Touch Mode.

Evidence for both accepted and rejected observations is written to
`debug-output/touch-anything/observation_<uuid>/`.

Phase 1 never publishes an Arm trajectory. Base adjustment is also disabled
while Touch Mode is active; move within the operator-approved ±0.5 m area before
entering Touch Mode, then reacquire/freeze the image.

Navigation is not shut down automatically. SAM runs once on a frozen frame, so
automatic Nav2 lifecycle mutation would add recovery risk without measured need.
If profiling later proves contention, add a dedicated observation-only launcher
that keeps localization and the cmd_vel safety layer but omits Nav2/local-costmap.

## 真机前离线回归

历史 evidence 检查、Mock Arm、连续 rosbag 采集与安全回放已经整理到
[`OFFLINE-REGRESSION.md`](OFFLINE-REGRESSION.md)。最常用的两个入口：

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/run_offline_gate.sh
tools/touch_anything/run_mock_arm_gate.sh
```

两者都不连接真实 Arm。下一次真机运行时，用
`record_touch_regression_bag.sh` 取得连续图像、TF、关节和状态机时序，之后可在
隔离的 ROS Domain 中反复回放，无需每次接电测试。

### 连续 tracker 全帧回放

下面的工具从与 bag 对应的人工确认 Arm Camera RGB/SAM mask 开始，把 bag
中的每一帧 Arm Camera 图像交给真机节点所使用的同一个 KLT/ORB tracker。同时它会在
10 Hz 固定时钟上消费最新 tracker 结果与录制 TF，运行真实 image-Jacobian 控制计算，并对
录制的 approach waypoint 重复运行真实 IK；整个过程仍然只读：

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

tools/touch_anything/replay_continuous_tracker_bag.py \
  debug-output/touch-anything-bags/touch_regression_20260918_140539 \
  --initial-arm-observation \
    debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/arm_observation_b03454a2-0641-4f0a-b077-8fcbfdefd26e.json \
  --json-output \
    debug-output/touch-anything-bags/touch_regression_20260918_140539/tracking_control_cycle_replay_b034.json
```

重点检查三个门禁：

```text
gates.perception_ready
gates.tracking_and_control_same_cycle_ready
gates.all_covered_motion_windows_trackable
```

该工具只读 bag、不发布 topic、不发送运动命令。通过门禁代表录制图像、TF、tracker、
image-Jacobian 与 IK 的计算量足以放入指定控制周期；它不代表线上 callback 调度已经复现，
也不代表真实 actuator 正在接受 rolling target、机械绝对精度或接触已经通过。现有在线执行仍是
有限 guarded trajectory 完成后 HOLD，再观察和重规划下一步。

### Bag tracking 可视化调试器

逐帧确认 mask、KLT 特征、光流、centroid、Action 状态、关节/控制器误差、TF 和控制输出：

```bash
cd /home/matsunaga-h/robot_ws
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

tools/touch_anything/bag_tracking_debugger.py \
  debug-output/touch-anything-bags/touch_regression_20260918_140539 \
  --initial-arm-observation \
    debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/arm_observation_b03454a2-0641-4f0a-b077-8fcbfdefd26e.json
```

操作：

- `Space` 播放/暂停，左右方向键逐帧；
- 速度可选 `0.05x .. 8x`；
- 点击时间轴或输入相对秒数跳转；
- 可在 LOST/runtime error、KLT↔ORB/恢复、frame gap/低健康度、Action/Touch 状态切换处自动断点；
- 事件列表可直接跳到对应帧；
- 右侧显示离线逐帧重算、在线 tracker/control tick、Dashboard execution、Touch state、
  joints、trajectory controller error、动态 Arm-camera TF 和已有 shadow-control 报告。

当前 `20260918_140539` bag 没有逐 tick 在线 telemetry，因此界面会显示
`online_tracker_tick=null` / `online_control_tick=null`，但仍可用 raw RGB 对真实 tracker 做逐帧
重算。新 bag 会额外记录：

```text
/touch_anything/tracker_debug   # 每个实际处理的图像、结果、耗时、generation、动作上下文
/touch_anything/control_debug   # 每个实际 10 Hz tick、tracker age、决策前后状态、是否更新 active goal
```

无 GUI 的快速检查：

```bash
tools/touch_anything/bag_tracking_debugger.py BAG_DIR \
  --initial-arm-observation ARM_OBSERVATION.json \
  --analyze-only --summary-json /tmp/tracking_summary.json

tools/touch_anything/audit_touch_regression_bag.py BAG_DIR \
  --json-output /tmp/recording_audit.json
```

`record_touch_regression_bag.sh` 现在会在录制前检查关键 publisher，录制后自动执行 evidence
audit，并把 Touch、gateway 和 trajectory server 参数快照放入 bag 目录。审计分别给出
`motion_tracking_observed` 与 `realtime_motion_correction_observed`；两者不能互相替代。
motion window 使用 rosbag record time，并按 visual-servo session ID 关联异步 control/server
telemetry；相机 header stamp 只用于图像 gap。审计也会把 tracker timeout、rolling correction failure、
Z-drop 等 STOP reason 列为 control failure，不能再被正常 server HOLD 掩盖。

### 目标命令与实际末端运动差值

对包含 rolling visual-servo topic 的 bag，可把每次 START/UPDATE 的关节目标经同一套 FK
转换为 base Cartesian target，并与时间最近的真实 `/joint_states` 对齐：

```bash
tools/touch_anything/analyze_motion_compensation.py \
  debug-output/touch-anything-bags/touch_regression_20260918_155846 \
  --json-output \
  debug-output/touch-anything-bags/touch_regression_20260918_155846/motion_compensation_analysis.json
```

加 `--include-samples` 会保留每个 sequence 的 requested tip、actual tip、三轴瞬时误差和
feedback 时间偏差。默认报告只保留 session 汇总，适合 review。绝对 tip 坐标没有叠加固定 mount
translation，但同一 rotation 下的位移量与 target-minus-actual 误差不受该平移影响。
报告还给出 STOP 前最后 feedback 到 terminal 后第一 feedback 的 Cartesian 位移包络，用来检查
current-pose HOLD 是否还允许停止后的余动；该值受 joint feedback 采样周期限制，不是精确 STOP 瞬间位移。

### Tracker freshness 判定回放

当 rolling session 因 tracker age 停止时，用实际 tracker callback start/end、输入帧接收年龄和
control tick 时间重放旧/新判定：

```bash
tools/touch_anything/replay_tracker_freshness_bag.py \
  debug-output/touch-anything-bags/touch_regression_20260918_163317 \
  --json-output \
  debug-output/touch-anything-bags/touch_regression_20260918_163317/tracker_freshness_replay.json
```

新逻辑有三个结果：

```text
FRESH         使用已完成的新鲜 tracker 结果更新目标
WAIT_INFLIGHT 已完成结果刚过期，但新鲜帧正在处理；不发旧目标，限时等待
STOP          tracker LOST，或 completed/in-flight 均超过 deadline
```

该工具不会发布 ROS topic 或发送机械臂命令。退出码 0 要求 bag 中真实发生的 freshness STOP 被
`WAIT_INFLIGHT` 替代，而且对应 in-flight callback 在配置 deadline 内以健康状态完成。

### 10 步滚动动作块反事实回放

用 bag 中真实 `/joint_states`、Dashboard rolling target 和 tracker age，重建新控制器在每次真实
命令时刻会生成的 10×50 ms 动作块：

```bash
tools/touch_anything/replay_receding_horizon_bag.py \
  debug-output/touch-anything-bags/touch_regression_20260918_163317 \
  --json-output \
  debug-output/touch-anything-bags/touch_regression_20260918_163317/receding_horizon_replay.json
```

门禁同时检查：10 个点完整、终点与原 IK target 精确一致、所有点在关节界限内、每个关节单调接近
终点、记录到的重规划间隔小于 actuator watchdog、0.5 s 动作尾迹长于 0.35 s watchdog，以及真实
速度反馈确实产生非零 latency prediction。该测试验证调度/几何/安全关系，不声称 bag 里已经运行过
新控制器，也不代替下一次真机末端精度验证。
