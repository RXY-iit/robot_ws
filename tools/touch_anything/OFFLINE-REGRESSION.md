# Touch Anything 真机前离线验证

目标是把测试分成四层，并清楚区分“离线能够证明什么”与“仍需真机证明什么”。所有离线脚本都不会打开 SO101 USB，也不会向真实 Arm、Pan/Tilt、底盘发布命令。

## 1. 一键离线门禁

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/run_offline_gate.sh
```

它依次执行：

1. `touch_anything` 与 `so101_arm_control` 的纯算法、安全边界和状态机单元测试；
2. `regression_manifest.yaml` 中真实历史 session 的文件、schema、mask 几何、动作证据与稀疏 KLT/ORB 重放；
3. 将机器可读报告写入 `debug-output/touch-anything-regression/offline_gate_*.json`。

历史 session 是动作事件发生时保存的快照，并非连续 25 Hz 图像。因此稀疏重放中的 `LOST` 只用于发现明显退化，不作为门禁失败。它不能证明实时跟踪不卡顿。

单独检查历史数据：

```bash
cd /home/matsunaga-h/robot_ws

# 固定代表性集合
/usr/bin/python3 tools/touch_anything/offline_regression.py

# 最近 20 个 session
/usr/bin/python3 tools/touch_anything/offline_regression.py --all --max-sessions 20

# 指定一个 session
/usr/bin/python3 tools/touch_anything/offline_regression.py \
  --session debug-output/touch-anything/observation_<uuid>
```

代表性集合由 `tools/touch_anything/regression_manifest.yaml` 管理。加入新案例时只记录已经确认的事实，例如最少帧数和动作步骤数；不要依据肉眼猜测 SUCCESS/FAIL 标签。

## 2. Mock Arm 控制闭环

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/run_mock_arm_gate.sh
```

脚本固定使用隔离的 `ROS_DOMAIN_ID=113`，启动无 USB 的 `mock_trajectory_server` 与 Arm gateway，实际走一遍 Preview → Execute → FollowJointTrajectory feedback → SUCCEEDED。它验证 ROS Action、Dashboard gateway 和 joint feedback 的连接，但不验证相机、真实电机、重力下垂、机构间隙或真实 TF 刷新。

## 3. 下次真机测试时录制连续证据

在启动 Touch 测试后，另开一个 terminal：

```bash
cd /home/matsunaga-h/robot_ws
export ROS_DOMAIN_ID=13
tools/touch_anything/record_touch_regression_bag.sh
```

完成一次 Touch 后按 `Ctrl-C`。也可以限定 120 秒：

```bash
tools/touch_anything/record_touch_regression_bag.sh --duration 120
```

rosbag 保存到 `debug-output/touch-anything-bags/touch_regression_<timestamp>/`。录制脚本只订阅数据，不发布运动命令。它记录：

- D435 RGB、aligned depth、CameraInfo；
- Arm Camera RGB、CameraInfo；
- `/joint_states`、Arm hardware/dashboard/controller feedback；
- `/arm/mount_transform` 与 Arm control mode/HOLD/candidate/path；
- Touch command/state/overlay/markers；
- 每帧 `/touch_anything/tracker_debug` 与每周期 `/touch_anything/control_debug`；
- Pan/Tilt、Lift、robot mode、cmd_vel；
- `/tf`、`/tf_static`、Action feedback/status 与 `/rosout`。

录制前如果关键 publisher 缺失，脚本会拒绝开始；录制后会生成
`recording_audit.json`。参数快照也保存在 bag 目录中。旧 bag 缺少逐 tick telemetry 时仍可利用
raw Arm RGB 离线重算 tracker，但无法还原线上究竟漏掉了哪一次 callback。

逐帧可视化：

```bash
tools/touch_anything/bag_tracking_debugger.py BAG_DIR \
  --initial-arm-observation ARM_OBSERVATION.json
```

该界面支持时间断点、逐帧、变速、事件跳转，并将离线重算与录制的在线 tracker/control tick
并排显示。

这组连续数据将补齐目前历史快照缺少的关键证据：每帧 KLT 连续性、动作前后 TF 新鲜度、Action 完成与关节/末端反馈的时间关系，以及卡顿究竟发生在视觉、状态机还是真实执行侧。

## 4. 安全回放与故障注入

```bash
cd /home/matsunaga-h/robot_ws

# 正常速度回放
tools/touch_anything/replay_touch_regression_bag.sh \
  debug-output/touch-anything-bags/touch_regression_<timestamp>

# 0.5 倍速
tools/touch_anything/replay_touch_regression_bag.sh BAG_DIR --rate 0.5

# 验证 TF 缺失不会错误执行
tools/touch_anything/replay_touch_regression_bag.sh BAG_DIR --drop-tf

# 验证 Arm Camera 掉线状态
tools/touch_anything/replay_touch_regression_bag.sh BAG_DIR --drop-arm-camera
```

回放强制使用 `ROS_DOMAIN_ID=113`，并把 Arm、Pan/Tilt、Nav 输出全部 remap 到 `/offline_sink/...`。因此即使真实机器人仍接电，回放也不会进入 Domain 13 的真实控制链。

回放复现的是同一次真实运行的感知和反馈时序，不是新的机械仿真。由于录制的图像与关节反馈已经固定，它适合检查：状态迁移、stale/missing evidence、目标跟踪、重复命令和异常恢复；不适合评估新轨迹在真实机构上的到达精度。

## 真机前判定标准

每次修改 Touch 逻辑后，至少满足：

1. 一键离线门禁通过；
2. Mock Arm 门禁通过；
3. 有连续 bag 时，正常回放和相关故障注入没有产生 actuator-facing 输出（输出只允许出现在 `/offline_sink`）；
4. 修改触及真实运动学、补偿、HOLD 容差或执行时序时，最后仍需一次受监督真机测试。

只有以下项目必须保留给真机：真实负载下的关节跟随误差、重力/松动/回差、USB 掉线、真实相机掉线、运动完成后的新鲜 TF、物理接触及颜色变化。离线通过不能替代这些证据。
