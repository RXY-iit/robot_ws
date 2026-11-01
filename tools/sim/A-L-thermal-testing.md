# A-L：仿真 LiDAR 温升预警试验（2026-10-31）

本试验使用**实验性温度模型**，不表示 MID-360 的真实过热阈值。`/b1/sim_lidar_thermal_state` 提供 core temperature 与 sim 时间；Health H4 的 `lidar_temperature_trend` 在温度/趋势达到 WARN 时启动只读 `PB-LIDAR-TEMP-WARN` 调查，不自动停车、降速或降温。实验设定 WARN=65℃（或 60 sim 秒预测可达）、BAD=78℃。真实设备默认只显示温度趋势，原生诊断决定严重程度；`real_lidar_temp_warn_c` 须现场基线校准后才可启用。

## 准备

Isaac 保持 Play；完整 ROS/Agent stack 必须重启并加载新源码/安装副本。若要观察**输入真的变坏**，启动 ROS stack 时使用 `B1_LIDAR_INPUT_TOPIC=/b1/lidar_input`，不要设置 `B1_GICP_INPUT_TOPIC`，再启动唯一的 upstream LiDAR relay。未这样重映射时只能测试温度告警/PB，不得宣称验证了点云退化。所有终端先执行：

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONNOUSERSITE=1
unset CYCLONEDDS_URI
```

在完整 ROS stack 启动前，在其启动终端设 `export B1_LIDAR_INPUT_TOPIC=/b1/lidar_input`。可参考 [B1 factor testing](B1-factor-testing.md) 的 upstream 路由；当前 Quick Start 界面若没有传递该变量，应使用下列终端方式：

```bash
export B1_LIDAR_INPUT_TOPIC=/b1/lidar_input
unset B1_GICP_INPUT_TOPIC
tools/sim/open_sim_nav2_windows.sh \
  --map maps/sim_factory_mid360_like_gpu_trueimu_011/nav2.yaml \
  --ply maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply \
  --initial-x 0 --initial-y 0 --initial-yaw 0
```

ROS 窗口启动后，先在另一终端启动唯一 upstream relay，并核对 relay 输出确实有 adapter 消费者；再启动 Agent/Dashboard，避免故障试验开始前先因缺少点云留下旧事件：

```bash
python3 tools/sim/inject_gicp_cloud.py \
  --scope upstream --input /point_cloud --output /b1/lidar_input \
  --ramp-sec 6 --hold-sec 120 --release-sec 6 \
  --log-dir "debug-output/a-l-thermal/relay-$(date +%Y%m%d-%H%M%S)"
```

确认 relay 正在透传后，再开终端启动 Agent/Dashboard（不会重复启动 ROS/Nav2）：

```bash
export ROBOT_MAP_ID=sim_factory_mid360_like_gpu_trueimu_011
export RECOVERY_PROFILE=nav2
export RECOVERY_MAP_PLY=/home/ruan-x/robot_ws/maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply
tools/run_agent_dashboard_sim_clean.sh
```

另开终端保持完整记录。每一轮使用新的输出名：

```bash
python3 tools/sim/capture_b1_trial.py --duration 3600 \
  --output "debug-output/a-l-thermal/trial-$(date +%Y%m%d-%H%M%S)/capture.jsonl"
```

另开终端启动温度模型；`--cloud-effect` 会在模型达到 78℃ 时请求 relay 的 `thermal` profile（仅逐渐丢帧），低于 72℃ 时请求停止。没有唯一 upstream relay、没有新鲜 capture，`start` 会拒绝。若只检验 Health/PB，去掉 `--cloud-effect` 即可。

```bash
python3 tools/sim/inject_lidar_thermal.py --cloud-effect \
  --initial-c 40 --peak-c 85 --warm-c-per-sim-sec 0.5 \
  --cool-c-per-sim-sec 1.0 \
  --log "debug-output/a-l-thermal/model-$(date +%Y%m%d-%H%M%S).jsonl"
```

命令需在模型终端保持运行时，从另一终端发送：

```bash
ros2 topic pub --once /b1/lidar_thermal_command std_msgs/msg/String '{data: start}'
```

在 WARN 后、BAD 前执行下面的命令，代表**人为确认并启动一个假想的降温措施**。这条命令不是 Recovery Agent 自动散热，也不是实际物理冷却。无干预对照不发送它，观察是否进入 BAD、是否真正丢帧。

```bash
ros2 topic pub --once /b1/lidar_thermal_command std_msgs/msg/String '{data: cool}'
```

核对：

```bash
ros2 topic echo /b1/sim_lidar_thermal_state std_msgs/msg/String --once --no-daemon --full-length
ros2 topic echo /b1/lidar_injection_status std_msgs/msg/String --once --no-daemon --full-length
ros2 topic echo /b1/capture_status std_msgs/msg/String --once --no-daemon --full-length
```

正式对照需要记录 `t_start/t_WARN/t_PB_probe/t_cool/t_BAD/t_cloud_loss/t_task_request/t_task_done`，以及模型温度、原始/输出点云率、GICP、Health、PB 诊断、Mission/Nav2 状态。`capture_b1_trial.py` 已订阅温度状态、命令及 Health/Recovery；模型和 relay 另有 JSONL。只看到模型温度 WARN 而点云未受扰动，不能说明传感器故障被预防；要比较有/无人工冷却，并加入同样 WARN 阈值的固定提醒对照。实验结束先 `cool` 并确认温度/点云状态，再 Ctrl+C 结束模型与 capture；保留日志，不复用文件名。
