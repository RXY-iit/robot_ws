> 后续检查补充：17:32 这轮还复现了 `initial_x` INTEGER/DOUBLE 类型错误。Quick Start 填 `0.0`，此前 terminal 示例填 `0`。现 launch 对 initial_x/y/yaw 和 scan 高度显式转换为 float，示例也已改为 `0.0`。此前的依赖检查只验证了导入，没有覆盖节点参数初始化。

> 2026-10-28 启动环境修正：17:26 这轮日志显示 GICP 启动后退出（exit code 1），随后 RViz 报 `Frame [map] does not exist`，costmap 等待 TF。离线复现发现 `PYTHONNOUSERSITE=1` 会屏蔽用户目录中的 Open3D/small_gicp。现已在 `fast_lio_localization.launch.py` 中仅对 GICP 子进程显式启用 user-site，其他 ROS 节点保留原环境。安装目录是源码符号链接，无须重新编译；重新启动 ROS 定位进程即可加载。Isaac 无须重启。下面的终端命令可以继续使用；L 噪声透传仍须启动，未启动时 GICP 会等待 `/b1/gicp_input`，这与后端缺失导致退出不同。

# B1 手动三因素注入

由人判断注入时机。注入器只改变输入、记录证据；停车、PB、重定位、续行由 Recovery / Nav2 自身处理。不运行 `trigger_b1_segment.py`。以下强度为待校准起点，不保证触发 WARN/BAD；COR/TOR 使用相同参数、路线和从 home 出发的初始状态。

## 每个 terminal 的环境

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI
export B1_RUN=debug-output/b1-manual-cor-01
```

本轮所有 terminal 用同一个 `B1_RUN`。下一轮改为 `b1-manual-cor-02` 或 `b1-manual-tor-01` 等新目录，已有记录拒绝覆盖。

## 一次性接线：必须在任务前完成

**L 点云链路：**从 terminal 启动 Full launcher，选 factory profile：

```bash
B1_GICP_INPUT_TOPIC=/b1/gicp_input tools/run_sim_robot_quick_start.sh
```

Isaac 已启动时取消勾选启动 Isaac，保留现有窗口；该命令不负责关闭旧 ROS。如果 Full 已经按默认配置启动，须先关闭其 ROS 终端，再按这个环境启动；只在事后 export 不能改变正在运行的 GICP 订阅。

也可不用 launcher，沿用单独 ROS 启动方式（两种选一种，不能重复启动）：

```bash
B1_GICP_INPUT_TOPIC=/b1/gicp_input tools/sim/open_sim_nav2_windows.sh \
  --map maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml \
  --ply maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply \
  --initial-x 0.0 --initial-y 0.0 --initial-yaw 0.0
```

在 Dashboard 已启动后，新 terminal 执行公共环境，再开 L 透传程序；保持运行：

```bash
python3 tools/sim/inject_gicp_cloud.py \
  --scope gicp --input /livox/lidar_pc2 --output /b1/gicp_input \
  --max-noise-m 0.10 --ramp-sec 10 --hold-sec 10 --release-sec 5 \
  --seed 20261028 --log-dir "$B1_RUN/lidar"
```

启动后是原样透传。`start` 才开始故障。当前 L 只污染 GICP 输入，不称为所有 LiDAR 消费者共同受损；上游传感器故障另用主指南中的 upstream 接线。

**O 里程计链路：**若本轮要测 O/LO，在 Isaac 初始化时、任务前设置 `os.environ["B1_ODOM_RELAY"]="1"` 再运行既有 post-play 初始化。若已经完成相机/LiDAR初始化，不要重跑整套 post-play；仅在 MANUAL、停止且尚未发任务时运行以下 wheel odom graph 脚本一次：

```python
import os, runpy
os.environ["B1_ODOM_RELAY"] = "1"
runpy.run_path("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_wheel_odom_ros.py", run_name="__main__")
```

这会把原始物理 odom/TF 隔离到 `/b1/physics_odom`、`/b1/physics_tf`；正常 wheel odom/TF 要由下面 relay 提供。ROS mode bridge 已启动、保持 MANUAL 时，新 terminal 执行公共环境，再运行并保持：

```bash
python3 tools/sim/inject_odom_error.py \
  --distance-scale 0 --turn-scale 0.10 --turn-bias-rad-per-m 0 \
  --growth-sec 20 --log "$B1_RUN/odom.jsonl"
```

未 start 时为正常积分。不要在 Mission 中临时切换这条接线；O 对照的 baseline 也使用同一正常 relay。L-only 不需要启用 O relay。新的 wheel graph 无论是否开启 O 都发布 `/b1/ground_truth_odom`；若 topic 缺失，任务前只更新上述 wheel graph，L-only 设置 `B1_ODOM_RELAY="0"`。

## 先开完整记录，再发 Mission

新 terminal 执行公共环境，然后保持记录器运行至任务结束（最多 1 小时 wall time，可 Ctrl+C 正常保存）：

```bash
python3 tools/sim/capture_b1_trial.py --duration 3600 \
  --output "$B1_RUN/capture.jsonl"
```

看到 `Recording: ...` 后，正常选择 AUTO、发布 Mission。记录器自动保存：

- wall / monotonic / sim 时间和源消息时间戳；物理、wheel、FAST-LIO、accepted 定位轨迹。
- 原始匹配 RMSE/fitness/拒绝原因、accepted score、score age、健康分级、Summary、Readiness。
- 注入命令及确认状态；LiDAR 实测噪声 RMS/P95、有效点/遮挡比例/平面残差；O 实际位置/yaw误差、增益和增长窗口。
- Mission / waypoint / Nav2 反馈与结果、BT、Recovery/PB事件、初值请求、速度、local costmap 近障碍指标。

`capture.status.json` 每秒显示各 topic 的数量与接收 age。手动 start 会核对记录器的心跳、clock/物理 odom 已被记录以及实际 injector；不会用实验标签或真实位姿控制 Recovery。其他信号若本身未发布仍会缺失，不能把“记录器运行”当成所有证据齐全；分析时检查各 topic counts。记录器不重新定义健康阈值。

物理轨迹保留原始 Isaac 坐标，**不能直接称为 map 坐标真值**。有独立标定时给 capture 加 `--truth-calibration 文件.json`；没有时可事后用干净初始同源时刻对齐，仅比较相对漂移。含 M 的轮次给 capture 和 L relay 均加 `--map-manifest maps/b1-defects/coarse-050.manifest.json`，记录地图哈希、点数、体素尺寸和可见性代理。

## 三种手动命令

每次在控制 terminal 执行公共环境。先分别测 L、O、M，再测组合。start 会等待 DDS 发现和状态确认，实际起点以记录中的注入确认 sim 时间为准，不用敲回车的时间替代。

**1. L：LiDAR 径向随机噪声，运行中执行。** 建议在 06→05 直行时手动执行：

```bash
python3 tools/sim/control_b1_injection.py start --factor lidar --profile noise
```

默认上述 relay 参数：10 sim-s 从 0 增至 σ=0.10m，维持10 sim-s，再5 sim-s降回0；没有统一平移、yaw偏置或额外遮挡。轻/中/强候选可分别使用 `--max-noise-m 0.05/0.10/0.20`，须在下一轮任务前启动 relay 时设定，实际严重程度由结果判断。

**2. O：odom 转角增益误差，运行中执行。** 建议在完成06、开始转向05时手动执行：

```bash
python3 tools/sim/control_b1_injection.py start --factor odom
```

默认20 sim-s内转角增量放大10%，随后停止增长，**保留已累计的 yaw / 位置漂移**。一直直行且 `distance-scale=0` 时可能几乎不产生误差，这是模型预期。需要直行也增长时，下一轮 relay 配置改为例如 `--distance-scale 0.03`；记录为不同条件。不会跳变重置 odom，也不会自动重定位。现有漂移必须在下一轮前清理、重新对齐；不能在移动中重启 relay。

**3. M：定位点云地图精度降低，任务前执行并加载。** 这是静态坏地图，不是运行中突然换图；只改 GICP PLY，不改 Nav2 栅格与 Isaac 场景：

```bash
env -u PYTHONNOUSERSITE python3 tools/sim/make_b1_fault_map.py \
  --source maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply \
  --output maps/b1-defects/coarse-050.ply \
  --whole-map --spacing 0 --fraction 0 --voxel 0.50 --seed 20261028
```

此离线命令临时允许读取已安装在用户目录的 Open3D，不更改 ROS terminal 的环境。生成一次即可，同一 COR/TOR 配对复用生成文件和 manifest；已有文件拒绝覆盖。然后在 Full launcher 将 PLY 选为 `maps/b1-defects/coarse-050.ply`，Nav2 map 仍为原 factory YAML；或在单独 ROS 启动命令中将 `--ply` 改成此路径。**仅生成文件不会改变当前定位，正在运行的 Mission 继续用原地图。** `/gicp_loc/recovery_status` 的 `map_path` 应为新路径。GICP 默认还会以0.25m降采样，故本例用更粗的0.50m作初步候选；不保证一定故障。

双墙另使用主指南中带实测 `--roi/--normal/--spacing/--fraction` 的命令，不能把整张地图复制平移叫作“墙面重影”。M-only不执行前两个start；LO组合手动执行前两个，LM/OM/LOM在对应坏地图轮次再执行所需start。

## 查看、提前停止

```bash
python3 tools/sim/control_b1_injection.py status --factor lidar
python3 tools/sim/control_b1_injection.py status --factor odom
python3 tools/sim/control_b1_injection.py stop --factor lidar
python3 tools/sim/control_b1_injection.py stop --factor odom
```

`stop` 只撤掉该干扰/停止增长，不取消导航、不恢复定位。O残差仍保留。提前stop属于人工干预，应与自动结束区分。离开实验后恢复普通 Full 时取消 `B1_GICP_INPUT_TOPIC` 接线、恢复正常 wheel graph（MANUAL停止时）并关闭 relay。

本次只做离线代码验证，没有启动任务或注入新故障，也未操作用户新开的 Isaac。
