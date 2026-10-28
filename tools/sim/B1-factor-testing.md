# B1 L/O/M 故障实现与启动说明（2026-10-28）

本轮是实现与离线测试更新，尚未部署到正在运行的 Full，也没有新的 COR/TOR 实验结果。下述强度是待校准起点，不保证触发 WARN/BAD；记录未检出和恢复失败，不能按结果调整配对中的另一侧。

## 已修改的恢复逻辑

- sim 的恢复锚点要求 accepted base pose、同一源时刻 odom、score < 0.15、至少3个观测且跨越3 sim-s、map 相对运动与 odom 一致。它比 Health 的 WARN=0.16/BAD=0.20 更严格。Health 的持续时间与回落规则保持原配置；普通 WARN 不启动 PB，也不自动限速。任务层原有极端 score 紧急门槛保留，不把新 WARN 阈值直接变成 task 中断门槛。
- 锚点不能在坏匹配或 initialpose 积累中刷新。odom 时间倒退、>2 sim-s 连续性缺口，或超过 `2*dt+0.15m` / `3*dt+0.15rad` 的运动突变会使恢复参考失效。阈值是当前平台的保守边界，后续需按运行速度校准。
- 传播使用 SE(2)，上限为120 wall-s、累计3m、累计1rad；不是仅看净位移。发布前再核对最新 TF 与健康报告之间的连续性/累计预算。连续 odom 仍可能缓慢漂移，因此这些边界不等于绝对正确保证。
- PB 先独立暂停依赖能力、确认停止，再检查传感器输入稳定可用，最多使用一次有效参考。**不要求 score 先健康才允许重定位**：坏 score 不能单独区分错误初值、噪声或错误地图。输入不可用先等待；持续噪声但数据仍正常到达时，可能仍有一次有界尝试，失败后人工检查，不能宣称已经自动识别/修复噪声或地图。
- 新鲜、稳定的恢复定位须与旧参考传播一致，才重建静止比较参考、通过 PB 验证和发送自动续行信号。低 score、注入结束、清空旧告警均不等价于恢复成功。无有效参考时，确认导航结束且机器人停止后，用 RViz 做明确人工定位，再收集干净稳定证据；AUTO/MANUAL 本身不作为人工授权证据。人工处理单列，不能计为自主恢复。
- 真值与注入标签只进入实验记录/触发器，不输入 Recovery。

### Stop/Play 后人工定位的参考重建

2026-10-28 修复：旧版本仅在 MANUAL 下把 `/initialpose` 当作人工重新建立参考，导致 AUTO 下 RViz 已重新匹配但仍报 `RECOVERY_REFERENCE_UNVERIFIED`。重置后使旧 odom 参考失效是正确保护，缺口在人工重新建立参考的入口。

新 sim 启动脚本把 RViz 的 2D Pose Estimate 发布到 `/operator/initialpose`；GICP 和 Agent 分别订阅，TOR 没有 Agent 时仍可人工定位。PB、启动初值和注入程序继续使用 `/initialpose`，不能因为处于 MANUAL 就获得重新建立参考的权限。这里的 topic 是操作来源约定，不是根据 score 猜测来源。

人工请求需有效 map 位姿、导航目标已结束及新鲜静止 odom；允许建立新参考后仍要等 GICP 积累完成、score <0.15、至少3个一致观测跨越3 sim-s，才解除阻塞。期间再次 odom 跳变或被非人工 seed 替换，会重新要求人工对齐。不要连续点击初值，重复点击会重新启动积累。

部署需重启 ROS 定位、RViz 与 Agent 以加载新代码和 topic remap，Isaac 不需要重启。**尚未重启的旧进程**可临时使用“MANUAL → 静止 → RViz 2D Pose Estimate 一次 → 等待参考验证 → AUTO”。新版事件与健康指标记录 `source_topic`/`initialpose_source`、`operator_alignment_authorized`、`manual_alignment_pending`；两种初值 topic 均加入 B1 记录。

## 固定实验条件

[候选参数](b1_factor_cases.yaml)列出 none、L、O、M、LO、LM、OM、LOM。配置是实验计划清单；下面的命令显式提供参数，日志中的实际配置为最终依据。正式配对冻结相同参数、种子、地图哈希、home起点、waypoint列表、Nav2/速度桥/安全条件。TOR 使用相同 route 文件与 `--health-gate route`，关闭 Mission/Recovery，保留只读记录；不要混用曾额外启用的桥层定位门控条件。

L：零均值径向噪声，默认无统一平移、无 yaw 偏置、无遮挡；旧 shelf_drift 等 profile 仅保留作旧数据复现。O：积分增量误差，默认只改转角增益，转弯时累积、之后保留。M：运行前选定的静态定位地图缺陷，仅改变 GICP PLY；Nav2 栅格、Isaac物理场景不随之改变。双墙与体素降采样独立编号。

每段只运行一个 `trigger_b1_segment.py` 实例。长路线可逐段筛查，但 O 残差、M 缺陷不能自动清零；前段未验证恢复时结束该 episode，不能紧接下一种注入。正式配对每次回 home，人工清理/重新锚定另记。

## 重启时接入 O 与独立物理位姿

1. 完成当前任务后，MANUAL、确认停止；按现有指南停止 ROS/Agent。当前代码修改不要求立刻中断正在跑的任务。
2. 在 Isaac Script Editor 的既有 post-play 初始化之前设置：

```python
import os, runpy
os.environ['B1_ODOM_RELAY'] = '1'
os.environ['ISAAC_CAMERA_LOW_RESOLUTION'] = '1'
os.environ['ISAAC_CAMERA_HZ'] = '5'
runpy.run_path('/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_real_lab_post_play.py', run_name='__main__')
```

此模式把原 `/wheel_odom` 和 odom→base TF 隔离到 `/b1/physics_odom`、`/b1/physics_tf`。同时独立发布 `/b1/ground_truth_odom`（frame=`isaac_physics_odom`）；无 O 条件也保留同一 relay、零误差，使链路一致。不要在运行中的导航任务里重建此 graph。默认不设置 `B1_ODOM_RELAY` 时原 Full 链路保持原样。

3. 每个 ROS 终端使用：

```bash
cd /home/ruan-x/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

先启动基础 sim bridge（提供 MANUAL 模式），再运行 relay，最后启动定位/Nav2与 Agent。relay 启动阶段要求 MANUAL 且原始物理速度接近零；发现第二个 wheel odom/odom TF 发布者会拒绝继续。

```bash
python3 tools/sim/inject_odom_error.py \
  --distance-scale 0 --turn-scale 0.10 --turn-bias-rad-per-m 0 \
  --log debug-output/b1-new/cor/odom.jsonl
```

relay 默认为正常增量积分。O 的 start 才开启误差增长；stop 只停止增长，**不清除已有误差**。它同时输出 `/wheel_odom` 与唯一 `/tf odom→base_footprint`，pose/twist/TF 来自同一积分。GICP、Nav2、PB 实际消费同一条被污染参考；FAST-LIO 的独立提示可能抵消部分影响，须记录 `fast_lio_hint_used`，不能预先声称 O 一定使定位 BAD。恢复原状须在 MANUAL 停止后重启 relay、重新定位并验证，记录为实验清理。

## L：两个注入范围分开测试

GICP分支隔离条件：定位/Nav2启动继续使用 `B1_GICP_INPUT_TOPIC=/b1/gicp_input`，并在发任务前启动：

```bash
python3 tools/sim/inject_gicp_cloud.py \
  --scope gicp --input /livox/lidar_pc2 --output /b1/gicp_input \
  --max-noise-m 0.10 --ramp-sec 10 --hold-sec 10 --release-sec 5 \
  --seed 20261028 --log-dir debug-output/b1-new/cor/lidar
```

脚本启动为字节不变的透传；不再使用含统一平移的 shelf_drift profile。须有 sim Dashboard 的配置接口，正式 TOR 可保留 Dashboard 界面服务但不可保留 Mission/Recovery 执行节点。时间参数均为 sim-s，超时/数据新鲜度为 wall-s。

传感器公共上游条件另开一组：启动定位/Nav2时指定 `B1_LIDAR_INPUT_TOPIC=/b1/lidar_input`，**不设置** `B1_GICP_INPUT_TOPIC`；relay 改为：

```bash
python3 tools/sim/inject_gicp_cloud.py \
  --scope upstream --input /point_cloud --output /b1/lidar_input \
  --max-noise-m 0.10 --ramp-sec 10 --hold-sec 10 --release-sec 5 \
  --seed 20261028 --log-dir debug-output/b1-new/cor/lidar
```

这使 adapter→Livox/LIO/GICP/scan 的消费者收到受扰动点云。两种范围不可混为同一种试验，也不要同时挂两个 command 消费者。输出无消费者或多发布者时 start 会拒绝。

## M：离线地图版本

从干净 PLY 选定实际墙面 ROI（xmin ymin zmin xmax ymax zmax）及法向，先检查是否真的选中了墙；工具不会替你把“货架附近”自动认作墙面。

```text
python3 tools/sim/make_b1_fault_map.py \
  --source maps/sim_factory_mid360_like_gpu_trueimu_011/map_3d.ply \
  --output maps/b1-defects/wall-double.ply \
  --roi XMIN YMIN ZMIN XMAX YMAX ZMAX --normal NX NY NZ \
  --spacing 0.15 --fraction 0.5 --voxel 0 --seed 20261028
```

上面 ROI/法向是待填实测参数，不能原样执行。生成 `.manifest.json` 包含原/新哈希、ROI点数、实际重复比例、间距、法向、点密度、体素尺寸。已有文件拒绝覆盖。降采样单因素使用 `--spacing 0 --fraction 0 --voxel 0.2` 并给不同输出名；不是双墙的替代物。

定位/Nav2启动保持原 `--map .../nav2.yaml`，只改 `--ply maps/b1-defects/wall-double.ply`。地图缺陷整个 episode 都存在，不做运行中突然换图。L relay 加 `--map-manifest maps/b1-defects/wall-double.manifest.json` 可在未启动 L 时也记录扫描点进入 ROI 的比例；这是基于时间对齐的**估计 map TF**的可见性代理，定位漂移时也会偏，不能当真值。M-only 仍开正常透传 relay 以记录点云质量/可见性。

## 物理坐标与 map 对齐

`/b1/ground_truth_odom` 不受 O relay 影响，但 Isaac physics odom 的原点不自动等于建图坐标。必须在无注入时测量同一时刻的两坐标位姿，保存固定 SE(2)：

```text
python3 tools/sim/calibrate_b1_physics_frame.py \
  --physics-pose PX PY PYAW --map-pose MX MY MYAW \
  --basis clean_initial_alignment --evidence '记录文件及源时间戳' \
  --output debug-output/b1-new/physics-map.json
```

若 map pose 来自干净 GICP，只能评估相对于初始对齐的漂移；只有通过独立场景/地图标定得到的 map pose 才使用 `independent_survey`。不要填猜测的全零变换。跨 COR/TOR 共享同一标定且检查 Isaac 重置是否改变 physics 原点。标定不发布 TF，不影响控制。

## 06→05 触发与完整记录

先启动记录、触发器，再发 home→06→05 任务。触发器必须看到06原始目标被接受并成功，以及05目标被接受；它不把“进入06容差范围”当作完成。L 等至少2 sim-s稳定直行且离开转向起点0.3m；O 在05 goal接受时开启，记录06后起步转弯的误差及直行残留。不是在直行起点人为跳变 odom。

```bash
python3 tools/sim/capture_b1_trial.py \
  --output debug-output/b1-new/cor/capture.jsonl --duration 1800 \
  --truth-calibration debug-output/b1-new/physics-map.json
```

```bash
python3 tools/sim/trigger_b1_segment.py \
  --from-xy 1.4 6 --to-xy -3.3 6 --factors LO \
  --stable-sec 2 --distance 0.3 --growth-sec 20 --timeout 900 \
  --truth-calibration debug-output/b1-new/physics-map.json \
  --output debug-output/b1-new/cor/trigger.jsonl
```

`--factors` 可以换为8个条件之一；包含M时，trigger/capture/relay均加相同 `--map-manifest`。触发器会检查M实际定位地图路径与清单一致，start必须得到run递增和active确认，否则标为失败，不盲目重发。目标被取消/失败只记录为结果：已启动的L仍按固定sim-time包络结束，O仍按固定增长窗口结束，不能因为COR先停车就提前移除故障。后续goal接受不会再触发一次。包络结束后再观察10 sim-s，trigger退出，capture继续。尚未达到直行条件就取消的案例单列为转弯阶段结果，不能混入直行对照。信号 Ctrl+C 会尝试停止注入；若进程被强杀则可能无法发送停止，必须检查状态再清理。

每次日志用新目录，工具拒绝覆盖。完整证据包括 injector逐帧日志、trigger事件、capture，不只保留图片。capture保存源时间、monotonic和wall时间、raw/accepted匹配、匹配拒绝原因/阶段、wheel/physics odom、健康与readiness、命令速度、BT/PB/Mission事件和地图/标定清单。

## 对比图与判读

```bash
python3 tools/sim/analysis/plot_b1_factors.py \
  debug-output/b1-new/cor/capture.jsonl debug-output/b1-new/tor/capture.jsonl \
  --output debug-output/b1-new/comparison
```

输出 wall/sim 两版图、轨迹图、范围统计JSON和事件JSON。横轴从各自capture开始计时；精确注入零点取trigger/command确认记录，不使用平均RTF换算。图包含实际径向噪声RMS/P95、在线局部平面残差、odom位置/yaw误差、地图ROI暴露代理、raw/accepted score与健康色带、Robot Summary/readiness、map/odom有符号x/y/yaw变化、对齐物理误差和速度/积累状态。缺证据会明确标记，不跨长缺口连线。map/odom参考为首个记录的accepted样本。

轨迹实线为经过固定标定的独立physics路径，点为accepted定位估计；没有标定就不画“真实路径”。估计/物理误差只比较源时间相差≤0.1 sim-s的样本。LiDAR注入误差是真值已知的实验量，局部平面残差才是在线可观测代理；后者受环境几何/运动影响，当前仅记录，**未拿未经校准的阈值直接触发PB**。

地图缺陷暴露、自然恢复、PB初值重设、人工定位及最后任务成功需分开计数。score缺口要结合积累阶段/拒绝原因/原始源流解释，不能理解成“BAD时主动隐藏score”。


## 本轮验证记录

离线回归：379项通过（Recovery/技能执行/时间参考/故障模型/点云/定位证据），另对最新启动参数位置做了检查，Python编译、YAML解析、shell语法通过。合成数据验证了wall/sim图、50组源时间对齐及PLY双墙生成（16原始点+8重复点=24点）。合成数据仅用于工具测试，未写为实验结果。

Isaac graph与新relay尚未在运行中的Full重载，DDS端到端与真实场景强度仍须下一轮验证。ROI和physics/map标定必须用场景实测值填写；脚本不会凭空假定这两项。
