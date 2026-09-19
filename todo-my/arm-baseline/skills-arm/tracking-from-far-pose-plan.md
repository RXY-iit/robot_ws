结论：你的判断基本正确，而且“从 FAR 到 PRESS 保持同一个目标跟踪状态”比当前反复运行 SAM、停稳、重新识别更适合这台机械臂。

但建议采用的不是“只用一种算法”，而是：

> SAM 负责首次确定“要跟踪哪个物体”；Pyramid KLT 负责连续高速跟踪；ORB 只负责目标丢失后的重新定位。

## 这些算法是什么

### KLT Tracker

KLT（Kanade–Lucas–Tomasi）是稀疏光流跟踪器。

它不会重新识别“这是什么物体”，而是回答：

> 上一帧里的这些角点，在下一帧移动到了哪里？

通常先用 Shi–Tomasi 选出容易追踪的角点，然后使用 Lucas–Kanade 光流逐帧跟踪。它计算量很小，非常适合相邻视频帧之间的连续跟踪。[OpenCV 官方说明](https://docs.opencv.org/4.13.0/d4/dee/tutorial_optical_flow.html)

限制是：

- 假设相邻帧运动不大；
- 容易受到运动模糊、遮挡和无纹理表面的影响；
- 长时间跟踪会漂移；
- 它只能跟踪点，不知道目标的语义身份。

### Pyramid KLT / ピラミッドKLT

Pyramid KLT 不是另一种完全不同的算法，而是 KLT 的实用增强版。

它先建立不同分辨率的图像金字塔：

- 在低分辨率层估计较大的移动；
- 再逐层返回高分辨率精修。

这样即使机械臂每帧移动稍大，也能把“大位移”转换成多层上的“小位移”。OpenCV 的 `calcOpticalFlowPyrLK()` 就是金字塔 KLT。[OpenCV API](https://docs.opencv.org/4.13.0/dc/d6b/group__video__track.html)

### ORB 特征

ORB 是：

- FAST 特征点检测；
- 带方向的 BRIEF 二进制描述符；
- 多尺度图像金字塔。

它不只是记录“这个点去了哪里”，还为每个点生成一个短的二进制身份描述，因此能在隔了多帧、发生一定旋转/缩放后重新匹配。[OpenCV ORB 文档](https://docs.opencv.org/4.13.0/d1/d89/tutorial_py_orb.html)

适合用途：

- KLT 跟踪失败后的重新定位；
- 确认当前跟踪区域仍然是最初的目标；
- 从保存的 FAR keyframe 重新找到目标。

不适合作为每帧主跟踪器：

- 匹配位置可能跳动；
- 平滑性一般不如光流；
- 白色、纯色、反光或低纹理按钮上可能没有足够特征。

## 当前问题的判断

现有节点虽然有 0.1 秒的自动接近定时器，即理论调度频率为 10 Hz，[touch_observation_node.py](/home/matsunaga-h/robot_ws/src/touch_anything/scripts/touch_observation_node.py:608)，但最近一轮保存的 Arm Camera 观测之间实际间隔约为：

```text
6.94, 6.34, 7.38, 6.55, 9.81, 6.37, 13.24 秒
```

这不是 SAM 单独耗时，而是：

```text
取得图像
→ SAM set_image/predict
→ 生成计划
→ 发送轨迹
→ 等待执行
→ 等待稳定
→ 再次观察
```

整个闭环的周期。

当前 Arm Camera 每次重新取得目标时都会重新向图像预测器执行 `set_image(rgb)`，[touch_observation_node.py](/home/matsunaga-h/robot_ws/src/touch_anything/scripts/touch_observation_node.py:268)，并且还存在停稳窗口：[touch_anything.yaml](/home/matsunaga-h/robot_ws/src/touch_anything/config/touch_anything.yaml:92)。

所以你看到的现象合理：

- 机械臂走一步后停下；
- 重量、松动、回差让末端发生掉落或振动；
- 下一次启动时又产生新的执行偏差；
- 补偿模型面对的不是稳定误差，而是随方向、姿态和机械状态变化的误差。

替换逐次 SAM 能明显改善，但如果轨迹接口仍然要求每步执行完成并停稳，运动仍不会真正连续。

## 推荐的总体结构

| 层 | 算法 | 建议频率 | 作用 |
|---|---|---:|---|
| 目标初始化 | SAM 2.1 + human confirm | 一次 | 确认语义目标和初始 mask |
| 连续跟踪 | Pyramid KLT | 20–30 Hz | 连续更新目标位置、尺度和 mask |
| 身份检查 | ORB + RANSAC | 事件触发或低频 | 防止 KLT 漂移到背景/邻近按钮 |
| 语义恢复 | SAM 2.1 | KLT、ORB 均失败时 | 重新分割目标 |
| 视觉控制 | 图像误差闭环 | 5–10 Hz | 连续输出 Y/Z 和 +X 修正 |
| Press 判定 | 跟踪区域颜色/外观变化 | 视觉帧率 | 判断按钮变色或物体被推开 |

Meta 的 SAM 2 Video Predictor 本身也支持带历史记忆的视频目标传播，[Meta SAM 2](https://ai.meta.com/research/sam2/)、[官方实现](https://github.com/facebookresearch/sam2/blob/main/sam2/sam2_video_predictor.py)，但它是否能在当前 NUC、CPU 模式下稳定满足控制周期必须实测。现阶段不建议把它直接放进实时内环。

## 推荐的特征建立方式

在 FAR 位置完成 human-confirmed SAM mask 后：

1. 保存不可变的目标基准：

   - FAR RGB keyframe；
   - 原始 SAM mask；
   - mask 的颜色直方图；
   - ORB 特征和描述符；
   - 初始面积、边框、质心。

2. 在 mask 内选择 KLT 特征：

   - 对 mask 略微腐蚀，避免边界把背景当成目标；
   - 排除固定下夹爪尖端附近；
   - 把 mask 分成网格，每个网格选择少量 Shi–Tomasi 角点；
   - 避免所有点集中在数字、边框或单一区域。

3. 每帧使用 Pyramid KLT：

   - 上一帧 → 当前帧；
   - 进行 forward-backward consistency 检查；
   - 去掉光流失败、误差大、超出画面的点；
   - 用 RANSAC 拟合 similarity transform 或 partial affine transform；
   - 使用拟合变换传播原始 mask，而不是直接把所有特征点求平均作为质心。

4. 保留两套参考：

   - 不可变的 human-confirmed FAR keyframe，用于确认身份；
   - 滚动的短期 keyframe，用于处理逐渐放大的目标。

5. KLT 退化时：

   - 优先使用 ORB 与 FAR/滚动 keyframe 重新匹配；
   - ORB 匹配通过几何一致性检查后重新初始化 KLT；
   - KLT 和 ORB 都失败时才运行 SAM；
   - SAM 仍不可靠时才要求 human 再确认。

尤其重要：不能无限用当前跟踪结果更新“目标身份”，否则跟踪一旦漂移到背景，就会把背景永久学习成目标。

## 与接近控制的结合

“同一个算法”应理解为同一个 tracker state，而不是 FAR、NEAR、PRESS 使用完全相同的目标像素。

建议连续变化控制目标：

### FAR 阶段

- SAM mask 中心优先向画面中心移动；
- Y/Z 修正优先；
- 同时缓慢推进 +X；
- 全程 KLT 跟踪，不重新运行 SAM。

### 接近阶段

随着目标变大：

- 仍由同一个 KLT tracker 输出 mask；
- 目标像素从“画面中心”逐渐过渡到“固定下夹爪尖端像素”；
- 不要突然切换控制目标，否则容易产生跳变。

### PRESS 阶段

- SAM/KLT 目标中心与固定下夹爪尖端保持对齐；
- 继续沿 `base_link +X` 推进；
- Y/Z 仍持续小幅闭环；
- 以 human-confirmed 时保存的目标区域颜色/外观为基准判断变化；
- 目标被推倒、按钮变色或目标区域发生明确变化后退出并回到 home。

这种方法本质上接近 Image-Based Visual Servoing：直接利用图像误差控制末端，而不是要求 TF、IK 和机械模型完全准确。

针对现有机械间隙，还可以在线学习一个局部二维图像 Jacobian：

```text
[像素 Δu]   [a b] [机械臂 ΔY]
[像素 Δv] = [c d] [机械臂 ΔZ]
```

每次命令后利用真实像素变化更新矩阵。这样它可以逐步适应：

- 相机安装角度；
- 不同姿态下的轴耦合；
- 机械臂重量引起的偏差；
- `Y/Z` 指令与图像移动方向并非理想正交的问题。

## 性能可行性

我用当前保存的 640×480 Arm Camera 图像做了一个本机离线基准：

- 150 个 KLT 特征点：约 `0.84 ms/frame pair`
- ORB 500 个特征：约 `4.45 ms/frame`

这只是热缓存下的 OpenCV 算法耗时，不包含 ROS、图像转换、RANSAC、控制和系统负载，因此不能视为最终性能。但它表明：

> 视觉跟踪 20–30 Hz、控制输出 5–10 Hz，在当前硬件上有很大概率可行；真正的瓶颈不再会是 KLT/ORB 本身。

## 仍需解决的另一半：运动接口

即使跟踪已经达到 30 Hz，如果每条命令仍然是：

```text
规划 → 执行完整轨迹 → HOLD → 等待稳定 → 下一条
```

仍然会发生停顿和重启误差。

更合理的是：

```text
Camera 20–30 Hz
      ↓ latest observation
KLT tracker 20–30 Hz
      ↓
Visual controller 5–10 Hz
      ↓
短时 Cartesian velocity / twist / 可连续更新的短轨迹
```

ROS 2 Humble 的 MoveIt Servo 就是类似方向：持续接收末端速度、关节速度或位置命令，适合视觉伺服和闭环控制，并带有关节边界、奇异点和碰撞处理。[MoveIt Servo 官方文档](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)

但是否直接采用 MoveIt Servo，要看当前 SO101 gateway 能否接受连续位置/速度更新；这需要单独验证，不能从现有代码直接假定。

最终建议是：

> SAM 初始化 + Pyramid KLT 主跟踪 + ORB 恢复 + 5–10 Hz 非阻塞视觉伺服。

这比“每步 SAM 重新分割”更符合当前问题，也能把机械误差转化为连续视觉闭环可以反复修正的小误差。当前最值得先验证的不是识别精度，而是：在不发送运动命令的情况下，让 KLT 连续跑完整个 FAR→PRESS 历史录像，确认目标质心、尺度、特征内点率和处理延迟是否稳定。

## 2026-09-10 确认后的连续控制定义

这里的“连续”不是缩短原有 `看图 → 单点轨迹 → 等待 action 完成 → 等待 TF/停稳 → 再看图`
的等待时间，而是取消这组步骤之间的硬边界：

```text
Arm Camera 约 15–30 Hz
  → Pyramid KLT/RANSAC 每帧更新目标
  → latest-value buffer（旧帧可丢弃，目标身份不可重置）
  → visual controller 5–10 Hz 更新滚动目标
  → actuator loop 以自身固定周期连续执行
```

控制器始终保留一个朝目标表面 `+X` 推进的名义目标；视觉误差只修正正在执行的
轨迹，而不是每次生成一个必须执行完毕的新任务。成功条件仍然是 PRESS 后的人机确认
基准区域外观变化；名义 Cartesian goal 不是成功条件。

### 各周期的职责

| 周期 | 输入 | 输出 | 不应做的事 |
|---|---|---|---|
| Camera/tracker 15–30 Hz | 最新 RGB、上一帧特征、不可变身份参考 | mask、质心、有效特征、内点率、变换 | 等待机械臂停止、每帧运行 SAM |
| Visual controller 5–10 Hz | tracker 最新值、固定 tip 像素、执行反馈 | 有界的 +X/Y/Z 滚动修正 | 等待一条完整 trajectory action 返回 |
| Actuator loop 20 Hz 级 | 最新有效关节/Cartesian target | 连续位置更新 | 每个视觉周期 Torque OFF/HOLD/重新启动 |
| SAM/ORB recovery | tracker 明确退化事件 | 自动恢复候选 | 未经确认就替换不可变目标身份 |

控制与相机线程必须解耦：计算来不及时使用最新一份有效观测，不排队执行旧观测；
运动侧使用 watchdog，视觉数据过期或退出 Touch Mode 时才减速并 HOLD。

## 旧 bag 的全帧 shadow replay

验证输入为真实记录：

- bag：`debug-output/touch-anything-bags/touch_regression_20260910_184602`
- 人工确认初始证据：
  `debug-output/touch-anything/observation_e3b1b6b0-60c7-48a8-b7a1-cd5119969a21/arm_observation_e01a874b-11ad-405a-aac1-d0b35b931b2c.json`
- 初始 RGB/SAM mask：同一证据目录内保存的原始文件；初始有效特征 `38`
- 回放范围：人工确认帧以后 `109.904 s`
- 工具：`tools/touch_anything/replay_continuous_tracker_bag.py`
- 完整结果：
  `debug-output/touch-anything-bags/touch_regression_20260910_184602/continuous_tracker_replay.json`

结果：

| 项目 | 实测值 |
|---|---:|
| 输入 Arm Camera 帧 | 1651 |
| 实际输入频率 | 15.013 Hz |
| KLT/RANSAC 接受帧 | 1651 / 1651（100%） |
| ORB/SAM 恢复次数 | 0 |
| 最大接受帧间隔 | 67.5 ms |
| tracker 耗时 median / p95 / max | 2.65 / 3.36 / 7.37 ms |
| 有效特征 min / median | 32 / 38 |

因此该真实录像可以确认：当前 Pyramid KLT/RANSAC 本身能够连续处理这段目标，
并能以明显高于 5–10 Hz 的频率提供控制观测。过去现场只看到 tracker frame index
缓慢增加，并不是算法耗时造成；全帧回放证明视觉算法不是数秒级瓶颈。

该回放不能确认：

- 真机 actuator 是否已经连续接受滚动目标；
- 机械间隙、重力、回差下的实际末端误差；
- 下一次真机运行时 TF 和 USB Camera 是否持续新鲜；
- 最终触碰/按钮变色是否成功。

特别需要保留这条界线：旧 bag 中记录的是旧的单点 action 执行，不能用它宣称新的
连续 actuator 接口已经通过。它目前通过的是“视觉跟踪及 5–10 Hz shadow-control
输入就绪”门禁。

## RViz Arm Camera 的事实显示

Arm Camera overlay 不再只显示汇总数字：

- 绿色实心小点：本帧真正通过 KLT forward/backward 和 RANSAC 的有效特征；
- 青色短线：同一有效特征从上一帧到本帧的光流位移；
- 紫色透明区域：由这些特征的几何变换传播得到的目标 mask；
- 黄色圆及连线：当前 mask 质心及它与固定 lower-tip 像素的误差；
- 顶部状态条：`status / method / valid feature count / inlier ratio / processing ms`。

显示的特征坐标同时进入 `/touch_anything/state` 的
`target_tracker.last_result.feature_points_uv`；光流起点进入
`previous_feature_points_uv`。最多显示/传输 80/160 个点，避免诊断 UI 反过来成为
性能负担。

## 下一次真机连续执行的验收门禁

在允许 PRESS 前，至少同时记录并核对：

1. tracker 输入持续不低于 10 Hz，p95 耗时低于 100 ms；
2. visual-controller 输出约 5–10 Hz，序列号连续，观测年龄可见；
3. actuator target 接收周期、实际关节反馈周期和 watchdog 状态可见；
4. FAR 到 PRESS 中间不出现 action-complete/settling 作为正常控制边界；
5. 目标恢复只有 `KLT rejected → ORB/SAM → human confirm` 才能改变身份；
6. 退出、过期、硬件故障仍能可靠 HOLD，不能用“连续性”绕过安全状态。

## 2026-09-10 21:26:06 两次现场尝试的复盘与修正

本节对应新记录：

```text
debug-output/touch-anything-bags/touch_regression_20260910_212606
```

结论先行：这轮不是同一个问题重复出现，而是两个相互独立的问题连续暴露。

1. 第一次选中的小按钮 mask 只有 9 个可用 Shi–Tomasi 特征，旧门禁硬要求 10 个，
   因而在人工确认之后被拒绝；mask、SAM score 和目标身份本身并没有失败。
2. 第二次使用较大 mask 后成功初始化 tracker，但诊断 overlay 的同步发布占住全局状态锁，
   使 Arm Dashboard、Touch 命令、自动接近和状态发布回调饥饿。自动接近随后看到的是
   本进程尚未来得及消费的旧 Dashboard 状态，错误地进入 `APPROACH_BLOCKED`。

这两个问题都发生在发送第一条 wrist-camera approach 轨迹之前。bag 没有证明机械臂
接近控制或 PRESS 失败；它证明的是初始化门禁和在线调度链在进入接近控制前失败。

### 记录覆盖与输入健康度

bag 总时长为 `167.923 s`，主要输入计数如下：

| Topic | 消息数 | 约计频率/说明 |
|---|---:|---|
| `/arm_camera/color/image_raw` | 2503 | 约 14.9 Hz，腕部相机没有中断 |
| `/arm/dashboard_state` | 835 | 约 5.0 Hz，上游 Dashboard 仍持续发布 |
| `/touch_anything/state` | 276 | 包含状态机、tracker age 和错误原因 |
| `/touch_anything/overlay_image` | 17 | 仅约 0.1 Hz，明显低于预期的 10 Hz |
| `/touch_anything/command` | 25 | 包含确认、重新采集、多次点击和退出请求 |

因此第二个问题不能归因于“相机没帧”或“Dashboard 没发状态”。消息已经进入 DDS 和
rosbag，异常发生在 Touch 节点内部的消费/调度边界。

### 问题 A：9 个有效特征被 10 点硬门禁拒绝

关键时间线（时间为 bag 起点后的相对秒）：

| 时间 | 事件 |
|---:|---|
| `+75.548 s` | FAR 到达后自动启动 Arm Camera SAM |
| `+79.514 s` | 初始 SAM 候选 `15565 px / score 0.767` |
| `+84.957..94.267 s` | 操作者多次点击，将 mask 收敛到实际小按钮 |
| `+94.267 s` | 最终候选 `2698 px / score 0.934` |
| `+98.679 s` | 确认被拒绝：`only 9 usable KLT points; need 10` |

对应证据文件是：

```text
debug-output/touch-anything/observation_9da7f44b-bb66-4d4b-b22f-3fd34d46b775/
  arm_observation_7a7d8efc-234d-4cdd-a4b3-e85e64ba1ac8.json
  arm_rgb_7a7d8efc-234d-4cdd-a4b3-e85e64ba1ac8.png
  arm_sam_mask_7a7d8efc-234d-4cdd-a4b3-e85e64ba1ac8.png
```

#### 为什么不能简单认定目标不可跟踪

`estimateAffinePartial2D` 拟合的是 4 自由度 partial affine/similarity 变换：二维平移、
统一缩放和旋转。理论最小对应点少于 6，但实际不能使用理论极限，因为仍要给
forward/backward 检查和 RANSAC 留出拒绝异常点的余量。

新的门禁取 `6`，其含义不是“有 6 个点就盲目相信”，而是以下条件必须同时成立：

```text
forward_status == valid
AND backward_status == valid
AND forward/backward error <= 1.5 px
AND retained tracks >= 6
AND partial-affine RANSAC succeeds
AND inlier ratio >= 0.50
AND inlier spatial coverage >= 0.10
AND warped mask area ratio remains in [0.35, 3.00]
AND warped centroid remains inside the image
```

也就是说，`6` 只是允许进入几何验证的下限；内点率、空间覆盖、mask 尺度和图像边界
仍是相互独立的拒绝门禁。低于 6 或任一几何门禁失败时，仍按
`KLT → ORB/RANSAC → SAM + human confirm` 恢复，不会沿用猜测位置。

#### 使用失败现场 mask 的修后回放

使用上述 `2698 px` mask、修改后的默认 6 点门禁，直接回放其后 30 秒真实 Arm Camera
帧，结果如下：

| 项目 | 修后实测 |
|---|---:|
| 初始化特征 | 9 |
| 输入帧 | 450 |
| 输入频率 | 15.013 Hz |
| KLT/RANSAC 接受 | 450 / 450（100%） |
| LOST / ORB / SAM 恢复 | 0 / 0 / 0 |
| 有效特征 min / median | 6 / 6 |
| processing median / p95 / max | 2.81 / 3.38 / 4.06 ms |
| 最大接受帧间隔 | 66.8 ms |

这个结果支持“9 点目标在这段真实运动中可稳定跟踪”，不是只把错误阈值改小让 UI
通过。它仍不能替代下一次真机接近测试，因为录像中的机械运动和新代码调度不同。

### 问题 B：overlay 反压导致所有关键回调饥饿

第二次重新采集后，操作者选择了较大的目标区域：

| 时间 | 事件 |
|---:|---|
| `+121.537 s` | SAM 候选 `15502 px / score 0.895` |
| `+123.238 s` | 收到 `confirm_arm_target` |
| `+123.298 s` | rosout 已打印 `ARM_TARGET_CONFIRMED`，tracker 初始化成功 |
| `+131.842 s` | `/touch_anything/state` 才实际记录该状态，迟到约 8.54 s |
| `+132.026 s` | 自动规划看到本地 Dashboard 状态过期，进入 `APPROACH_BLOCKED` |

`+131.842 s` 的状态同时给出：

```text
tracker.frame_index = 1
tracker.processing_ms = 5.98
tracker.last_good_age_sec = 8.54
```

这三个数共同限定了阻塞位置：KLT 本身只用了约 6 ms，而且 tracker 已经写入
`last_good_monotonic`；其后约 8.5 秒才完成状态发布，所以耗时发生在 tracker 计算之后。
旧调用链正好是：

```text
tracker.update()                         # processing_ms 到这里结束
  → 更新 _tracker_last_good_monotonic
  → _republish_overlay()
       → 持有全局 self._lock
       → 绘制并转换 640×480 RGB
       → 同步 DDS publish 921676-byte Image
       → 慢订阅者/rosbag 产生反压
  → 其他回调等待同一个 self._lock
```

后续 state 中 tracker frame 只从 1 增加到 7，frame 间隔约 5–9 秒；但同一段 bag 的
相机仍是 15 Hz。在线 `processing_ms` 偶尔为 40–213 ms，也远小于整个 5–9 秒间隔。
此外，多次 `acquire_arm_target`、点击和 `exit` 命令也延迟处理，这说明受影响的不是
tracker 一个 timer，而是共享锁和 callback group 后面的整条控制链。

对第二个 `15502 px / 20 特征` 初始帧做只读全帧回放：

| 项目 | 实测 |
|---|---:|
| 输入帧 | 848 |
| 覆盖时长 | 56.417 s |
| 输入/接受频率 | 15.013 / 15.013 Hz |
| 接受率 | 100% |
| processing median / p95 / max | 2.36 / 3.25 / 3.92 ms |
| 有效特征 min / median | 20 / 20 |

所以这里修的是 ROS 在线调度和反压隔离，不是继续调 KLT 参数。

## 修后的线程、锁与 latest-value 逻辑

### 六类可独立调度工作

`MultiThreadedExecutor` 现在使用 6 个 worker，并把工作分成以下逻辑 lane：

| Lane | Callback group | 允许阻塞的工作 | 禁止持有/等待的资源 |
|---|---|---|---|
| Touch 控制 | mutually exclusive | 小步规划、状态转移 | 不等待 overlay DDS |
| Arm 状态 | mutually exclusive | 消费 latest Dashboard state、完成 action 状态机 | 不运行 OpenCV/SAM |
| Sensor/telemetry | reentrant | 只替换 depth-1/latest buffer | 不做规划或图像推理 |
| Tracker | mutually exclusive | 单个 KLT/ORB update | OpenCV 运算时不持有全局状态锁 |
| Visualization | mutually exclusive | overlay 绘制和 best-effort publish | 不影响控制、Arm state、tracker |
| TF callbacks | executor 独立可调度 | 更新 TF buffer | 不与 overlay 串行 |

线程数不是实时性保证；真正的保证来自“阻塞资源不跨 lane 传播”。即使 RViz 不读取
overlay，或 rosbag 写盘瞬时变慢，最坏结果应只是 overlay 更新变慢，不能让命令、
Dashboard 状态或 tracker 停止。

### Tracker update 的提交协议

tracker 对象仍然是有状态对象，所以不能让 `initialize/reset/update` 并发修改。新逻辑
使用独立 `_tracker_operation_lock`，并用 `_tracker_generation` 防止旧结果回写：

```text
1. 持有 self._lock：
   - 读取最新 Arm RGB；
   - claim image stamp，防止同一帧重复处理；
   - 快照 tracker object 和 generation；
   - 立即释放 self._lock。

2. 持有 _tracker_operation_lock：
   - cv_bridge RGB snapshot；
   - tracker.update()；
   - 复制 propagated mask；
   - 释放 _tracker_operation_lock。

3. 重新持有 self._lock，提交前验证：
   - generation 未改变；
   - tracker object 仍是当前对象；
   - Touch Mode 仍 active；
   - Arm handoff 仍有效。

4. 任一验证失败：
   - 丢弃这一帧结果；
   - 不恢复旧 mask；
   - 不启动 SAM；
   - 不发送运动。
```

当 `exit/cancel/new confirmation/reset` 与 OpenCV update 重叠时，reset 会增加 generation。
这样旧 session 的计算即使稍后完成，也不能污染新 session。锁顺序也避免了控制回调
长期等待全局锁：耗时 OpenCV 操作只占 tracker 专用锁。

### Overlay 的 latest-value/coalescing 协议

所有产生新显示内容的代码只执行：

```text
_overlay_dirty = true
```

独立 visualization timer 每 100 ms 最多 claim 一次：

```text
if not dirty:
    return
dirty = false
render newest state snapshot
release self._lock
publish overlay
```

关键语义是：

- overlay 是诊断输出，不是控制输入；
- publish 前释放全局锁；
- publish 阻塞时不排队保存每一张旧 overlay；
- 阻塞期间到来的多个更新合并为下一张最新 overlay；
- 允许丢失中间诊断帧，不允许延迟命令或运动安全状态；
- 真实 Arm RGB tracker 输入仍按 depth-1/latest 策略独立消费。

回归测试会在 fake publisher 内检查 `self._lock` 已释放，并检查两个连续 timer tick 在
没有新 dirty 请求时只发布一次。

`/touch_anything/state` 现已直接公开：

```text
target_tracker.generation
target_tracker.last_input_stamp
target_tracker.discarded_generation_count
visualization.overlay_dirty
visualization.refresh_requested_count
visualization.coalesced_update_count
visualization.published_count
visualization.last_publish_duration_ms
visualization.last_error
arm_gate.state_receive_age_sec
```

这些计数是节点进程生命周期内累计值；session 边界可由 `enter/exit` 状态时间戳切分。

### Dashboard 暂时 stale 的自动等待分支

旧逻辑把所有 `_arm_gate()` 失败都视为永久故障：

```text
stale once → _pause_automatic_approach() → automatic=false
```

这对控制模式错误、硬件故障是正确的，对一次调度延迟却过于激进。新逻辑只对精确原因
`Arm Dashboard state is missing or stale` 使用可恢复等待：

```text
ARM_TARGET_CONFIRMED
  → plan_approach_step
  → Dashboard age > 0.75 s
  → AUTO_APPROACH_ARM_STATE_WAIT
  → no preview, no execute, no motion
  → due = now + 0.10 s
  → tracker observation 仍须 fresh
  → 重新采样当前 Arm Camera TF
  → 再次进入 plan_approach_step
```

以下错误仍然立即暂停，不能通过 retry 绕过：

- control mode 不是 `code2real&sim`；
- control mode 未同步；
- Arm health 为 `STALE/STARTING/RECOVERY_REQUIRED/HOLD_FAULT/CODE_TORQUE_INCOMPLETE`；
- tracker/mask/目标身份无效；
- IK、joint limit、Cartesian endpoint 或 gateway validation 失败；
- tracker 和 ORB 均丢失目标；
- Touch Mode 退出。

等待分支仍受 `automatic_approach_timeout_sec` 总超时限制。它不会在 stale 状态下发布
preview/execute，也不会把缓存关节状态解释成新反馈。

## 修正后的状态机不变量

为了 review，下面列出本次修改后必须保持的行为不变量：

1. **身份不变量**：第一次人工确认的 RGB/mask/appearance/ORB reference 在本次 Touch
   session 内不可变；rolling keyframe 只服务短期尺度变化。
2. **帧单调不变量**：`_tracker_last_stamp` claim 后，同一或更旧 RGB 不再处理。
3. **代际不变量**：reset/reinitialize 后，旧 generation 的计算结果一律丢弃。
4. **锁边界不变量**：OpenCV update 和 DDS overlay publish 都不持有 `self._lock`。
5. **显示降级不变量**：overlay 可以降频、合并或丢中间帧；控制输入和命令不能因此延迟。
6. **运动授权不变量**：fresh tracker 只提供观测，不等于 motion authorization；Arm gate、
   IK、gateway preview 和 operator PRESS confirmation 继续独立生效。
7. **stale fail-silent 不变量**：Dashboard/TF 暂时过期时只等待，绝不基于旧数据发送运动。
8. **语义恢复不变量**：KLT、ORB 都失败后，SAM 只能生成候选；没有人工再次确认就不恢复运动。
9. **PRESS 不变量**：没有 `confirm_press` 和实际 PRESS motion attempt，外观变化不构成 Touch 成功。
10. **退出不变量**：`exit/cancel` 使 active/automatic 失效；即使旧 tracker update 稍后完成，
    generation 检查也禁止其恢复目标或动作。

## 本次已完成验证

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/touch_anything/test
```

结果为 `66 passed`。完整无硬件离线门禁（Touch + SO101 safety tests + 三个代表性真实
稀疏证据 session）结果为 `127 passed`，三个 evidence audit 全部 `PASS`；报告保存在：

```text
debug-output/touch-anything-regression/offline_gate_20260910_214711.json
```

之所以显式禁用 plugin autoload，是系统环境中注册了缺失的
`typeguard._pytest_plugin`；不禁用时 pytest 在收集本项目测试前就退出，和本次实现无关。

此外已完成：

- `py_compile` 覆盖 `src/touch_anything/scripts/*.py` 和 `tools/touch_anything/*.py`；
- `git diff --check`；
- `colcon build --packages-select touch_anything --symlink-install` 成功；
- 失败小 mask 后 30 秒、450 帧真实全速 shadow replay；
- 第二次大 mask 后 56.4 秒、848 帧真实全速 shadow replay。

两份结构化结果已保存在新 bag 目录：

```text
low_texture_tracker_replay_fixed.json
continuous_tracker_replay_fixed.json
```

单元/回放验证尚未证明：

- 真实 DDS/RViz/rosbag 同时开启时 overlay publish 的最坏阻塞时间；
- 修后在线 tracker callback 的持续实际频率；
- 第一个视觉 waypoint 的 IK/执行结果；
- FAR→NEAR→PRESS 的机械精度和触碰成功；
- exit 在真实负载下的端到端响应时间。

## 下一次真机复测顺序与验收标准

### T0：只看调度，不发送 approach

在人工确认 Arm Camera 目标后暂不允许或不执行 approach，至少观察 20 秒：

- `/arm_camera/color/image_raw` 保持约 15 Hz；
- tracker `frame_index` 在 20 秒内应增加约 250–300，而不是个位数；
- `last_good_age_sec` 正常应低于 0.20 s，瞬时负载下不得持续超过 1.0 s；
- `processing_ms p95 < 100 ms`；
- overlay 可以低于 tracker 频率，但 tracker/state 不能跟随 overlay 一起降到 0.1 Hz；
- 连续发布一次 `exit` 后 1 秒内应看到 `INACTIVE` 和 base lock release。

T0 不通过时禁止继续 T1。应记录 callback 起止时间和 overlay publish duration，而不是再调
SAM/KLT 阈值。

### T1：只执行第一个 guarded visual waypoint

- 确认后若 Dashboard age 瞬时超过 0.75 s，应进入 `AUTO_APPROACH_ARM_STATE_WAIT`；
- 等到下一份约 5 Hz Dashboard state 后应自动恢复规划；
- wait 期间 `/arm/dashboard_command` 不得出现 preview/execute；
- fresh 后最多产生一个 preview request，candidate 接受后最多产生一个 execute request；
- request id 必须一致闭合，不能因 timer 重入重复发送；
- endpoint 必须用新鲜 post-motion TF 验证。

### T2：低纹理小按钮 mask

- 若初始化特征为 `6..9`，允许进入 tracker 几何验证；
- 每帧仍需满足 FB、RANSAC、coverage、area 和 centroid 门禁；
- 有效点降到 `<6` 时应进入 ORB 或 semantic recovery，不能继续传播旧 mask；
- recovery SAM 后必须再次 human confirm；
- 记录 minimum tracks、inlier ratio、coverage 和 mask fraction，而不能只看绿色点是否存在。

### T3：FAR→NEAR，PRESS 仍单独确认

- tracker 与 Arm state 在每条 guarded trajectory 执行期间都继续更新；
- 普通 waypoint 完成边界可以存在，因为当前 actuator 仍是 guarded trajectory step，
  但视觉目标身份不能在每步重置为 SAM；
- `PRESS_CONFIRMATION_REQUIRED` 前不得发送 +X PRESS；
- PRESS 后只有连续外观变化证据满足阈值才返回 home；
- 任一 stale/lost/health/IK/gateway 错误都应停止新命令并保留可审计原因。

### 下一份 bag 的诊断量

本次已加入 tracker generation、overlay 合并/发布/耗时和 Arm state receive age。后续还可
继续补以下 callback 起止时间：

```text
tracker_tick_started_monotonic
tracker_tick_completed_monotonic
overlay_dirty_age_sec
control_tick_lateness_ms
last_command_received_monotonic
last_command_completed_monotonic
```

这样下一次如果 UI 再次“像卡住”，可以直接区分：输入缺失、executor 延迟、锁等待、
OpenCV 计算、DDS publish 反压或状态机主动等待，而不再依赖画面观感推断。

## 2026-09-18 14:05:39 两次 approach：实际卡点、判定与修正

本节对应最新记录：

```text
debug-output/touch-anything-bags/touch_regression_20260918_140539
debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/
  motor_evidence.json
```

### 先回答“两次有没有边运动边 tracking”

严格按三个不同层级回答：

1. **相机是否在机械臂运动时继续出帧：是。** 两个动作窗口内
   `/arm_camera/color/image_raw` 均保持约 15 Hz，header stamp 单调更新。
2. **在线 tracker 是否在机械臂运动时持续消费这些帧：否。** 第一次从 frame 1 只前进到
   frame 2，然后停约 4.1 秒；第二次在动作时间窗中同样没有形成连续帧覆盖。
3. **tracker 新结果是否在同一条轨迹尚未完成时持续修改机械臂目标：没有实现。** 当前网关仍向
   `FollowJointTrajectory` 发送单 waypoint action；action 完成后进入 HOLD，Touch 再验证末端、
   settle、重规划下一条 action。它是 stop-observe-replan，不是 rolling target / visual servo。

因此旧状态字段
`GUARDED_TRAJECTORY_STEPS_WITH_CONTINUOUS_WRIST_CAMERA_TRACKING` 只能表达设计意图；
这份 bag 不满足“连续 tracking 已在线实现”的证据标准，更不能宣称“连续运动视觉伺服”已实现。

### 两次真实动作时间线

时间为 bag 起点后的相对秒。Dashboard 的 `SUCCEEDED` 是动作实际结束事件；Touch state
时间是 Touch 节点消费并处理该事件的时间，两者不能混用。

| 事件 | Approach 1 | Approach 2 |
|---|---:|---:|
| execute command | `+52.447 s` | `+74.184 s` |
| gateway `EXECUTING` | `+52.524 s` | `+74.325 s` |
| gateway `SUCCEEDED` | `+53.963 s` | `+75.702 s` |
| Touch 首次进入 post-motion feedback wait | `+56.794 s` | `+78.412 s` |
| gateway→Touch 可见延迟 | 约 `2.83 s` | 约 `2.71 s` |
| overlay 单次 publish 总耗时 | `4070.77 ms` | `4111.00 ms` |
| 最终失败 | `+60.760 s` | `+83.180 s` |

两次轨迹本身都执行成功。`motor_evidence.json` 中两个 step 的 gateway state 均为
`SUCCEEDED`，第一步从 FAR 前进后，第二次规划时已经测得
`advanced_from_far_m = 0.0178148 m`。所以“机械臂完全没动”不符合记录；准确描述是：

```text
机械臂执行一条约 1.5 s 的有限轨迹
→ 进入 HOLD
→ Touch 的视觉/状态/反馈消费被 overlay 阻塞
→ post-motion gate 等待并失败
→ 不再发送下一条轨迹
```

这也解释了画面中的 `FAR advance: 17.8/35.0 mm`：第一条动作产生了可见推进，但闭环没有
继续完成剩余 17.2 mm。

### 输入没有停止，Touch 进程被诊断输出阻塞

在两个动作窗口中，bag 记录到：

- Arm RGB 共 143 帧且 143 个 header stamp 全部唯一，约 15 Hz；
- 六轴 `/joint_states` 约 5 Hz 持续发布，header age 通常小于 1 ms；
- Dashboard action feedback 约 5 Hz，误差逐步收敛并正常结束；
- tracker 的 `processing_ms` 只有约 28–129 ms，不足以解释 4 秒停顿；
- `visualization.last_publish_duration_ms` 同步跃迁到 4070.77/4111.00 ms。

2026-09-10 的修正只做到“DDS publish 前释放 `self._lock`”。这能避免一个明确的锁反压，
但最新 bag 证明还不充分：Python/rclpy 的大尺寸 raw Image publish 本身可以长时间占用执行路径，
即使不持有共享锁，也会让同一进程内的 tracker、JointState 与 Dashboard 回调无法及时运行。

所以本次把不变量加强为：

> 自动 approach 或任一 Arm action 期间，不调用 raw overlay publish。只设置 dirty 标记并合并
> 中间帧；自动运动暂停、完成或退出后，再发布一张最新 overlay。

这不是降低 tracker 频率。RGB depth-1/latest buffer、KLT/ORB update、关节反馈和控制状态仍继续；
被牺牲的是运动期间的诊断图像刷新。文字状态与结构化 tracker telemetry 仍可发布。

新增诊断量：

```text
visualization.policy =
  LATEST_VALUE_COALESCED_DIAGNOSTIC_ONLY;
  DEFERRED_DURING_AUTOMATIC_OR_ACTIVE_ARM_MOTION
visualization.deferred_for_motion_count
```

### post-motion Cartesian gate 为什么误判

旧逻辑在 Touch **收到** terminal Dashboard state 时记录完成时间，再要求：

```text
joint/TF stamp >= Touch_receive_terminal_time - 0.25 s
```

但本次 terminal state 已被 overlay 阻塞约 2.7–2.8 秒。于是动作结束附近真实存在的新鲜
`/joint_states`，仍会被错误地判为“早于完成”。截图中的第二次错误是：

```text
latest Arm TF = 1789708014.569816 s
required      = 1789708018.600651 s
```

`required` 来自延迟消费时间，不是硬件动作实际完成时间。这是时间语义错误，而不只是把 3 秒
timeout 调大就能解决。

修正后 Arm gateway 在每个执行状态中附带：

```text
execution.event_ros_sec
```

`SENDING / EXECUTING / CANCELLING / SUCCEEDED / FAILED / CANCELLED / BLOCKED` 每次状态变化都
更新该时间。Touch 的 post-motion 下限改为：

```text
minimum_feedback_stamp = gateway_terminal_event_ros_sec - completion_slack
```

只有兼容旧 gateway 消息、缺少 `event_ros_sec` 时，才回退到本地 receive time。诊断同时记录：

```text
terminal_stamp_source
terminal_state_local_receive_ros_sec
terminal_state_transport_delay_sec
```

这样可以区分“硬件真的没有终点反馈”和“反馈已经存在，只是 terminal 消息被本进程晚处理”。

### 新增硬门禁：不是状态显示 TRACKING 就算边运动边 tracking

本次增加成功 tracker frame stamp 的有界历史，并在每条 approach dispatch 时记录
`execute_dispatch_ros_sec`。收到带 gateway event time 的 terminal state 后，只统计动作窗口内：

```text
dispatch_ros_sec < successful_tracker_image_stamp <= completion_event_ros_sec
```

通过条件同时要求：

- 成功 tracker 帧不少于 `approach_tracking_minimum_motion_frames = 3`；
- 将 dispatch、所有成功帧、completion 组成一个时间序列；
- 包括首尾边界在内，最大间隙不超过
  `approach_tracking_maximum_motion_gap_sec = 0.50 s`。

证据写入每一步 approach plan 与 `motor_evidence.json`：

```text
motion_tracking_evidence.continuous
motion_tracking_evidence.motion_window_sec
motion_tracking_evidence.successful_frame_count
motion_tracking_evidence.minimum_required_frames
motion_tracking_evidence.first_successful_frame_stamp_sec
motion_tracking_evidence.last_successful_frame_stamp_sec
motion_tracking_evidence.maximum_gap_sec
motion_tracking_evidence.maximum_allowed_gap_sec
motion_tracking_evidence.reason
```

只检查首尾各一帧仍可能掩盖中间 4 秒冻结，所以必须检查最大 gap。若 Cartesian endpoint 已通过，
但 tracker motion coverage 未通过，状态进入：

```text
APPROACH_TRACKING_NOT_CONTINUOUS
```

并停止发送下一条 trajectory。也就是说，“tracking during motion”从 UI 文案升级为可审计的
运动授权门禁。

### 修正后的当前能力边界

修正后预期实现的是：

```text
有限 guarded trajectory 执行期间
  + Arm RGB 持续进入 latest buffer
  + KLT/ORB 持续更新同一目标身份
  + JointState/Dashboard 持续消费
  + 动作结束后立即用 gateway event time 验证真实 endpoint
  + 短 settle 后用最新 tracker/TF 规划下一小步
```

它解决这次“执行一段后停 7–8 秒并失败”的问题，并把正常步间空档压回到安全 settle/replan
所需时间。但它仍然不是以下架构：

```text
trajectory 尚在执行
  → tracker 新误差进入 visual controller
  → 更新同一条滚动 joint/Cartesian target
  → actuator 不进入每步 HOLD
```

后者需要新增可抢占/可更新的短 horizon command 或 MoveIt Servo 类接口，并把 real trajectory
server 的 HOLD-entry 语义改成只在 watchdog、退出、失去目标或故障时触发。不能通过连续发送
当前 one-point `FollowJointTrajectory` 来伪装，因为网关明确拒绝 active goal 期间的新 goal，
real server 也会在每条成功 action 后进入 HOLD。

### 2026-09-18 修后验证

已执行：

```text
python3 -m py_compile \
  src/touch_anything/scripts/touch_observation_node.py \
  src/so101_arm_control/so101_arm_control/arm_gateway.py

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/touch_anything/test src/so101_arm_control/test

colcon build --packages-select touch_anything so101_arm_control --symlink-install
```

结果：

- `129 passed`；
- `touch_anything` 构建成功；
- `so101_arm_control` 构建成功；
- 新增测试覆盖 motion-window 最少帧/最大 gap；
- 新增测试确认 automatic approach 时 overlay 保持 dirty、不会进入 publisher；
- 原有测试仍确认 DDS publish 时不持有共享状态锁。

离线测试不能证明真机时 DDS 实现、USB 调度和 servo bus 的最终时序，因此本次修正后的
“边运动边 tracking”仍必须由下一份真机 bag 通过下面门禁确认。

### 下一次复测的第一优先级验收

先只执行一个 coarse approach，不进入 PRESS。必须同时满足：

1. gateway `EXECUTING → SUCCEEDED` 时间窗内，
   `motion_tracking_evidence.continuous == true`；
2. 1.5 秒动作中预期应有约 15–22 个成功 tracker 帧；硬门禁虽只要求 3 帧，但
   `maximum_gap_sec` 必须不超过 0.50 秒；
3. `visualization.deferred_for_motion_count` 增加，且运动期间
   `/touch_anything/overlay_image` 不发布新 raw frame；
4. `/arm_camera/color/image_raw` 与六轴 `/joint_states` stamp 持续单调；
5. `terminal_stamp_source == GATEWAY_EVENT_ROS_TIME`；
6. `terminal_state_transport_delay_sec` 正常应接近一个 Dashboard 周期，而不是 2–4 秒；
7. endpoint feedback source 优先为
   `DIRECT_JOINT_STATE_CALIBRATED_FK_PLUS_ARM_MOUNT_TF`；
8. 不再出现“latest TF 在动作开始附近，但 required 被推迟到 Touch 收包时间”的组合；
9. endpoint 通过后自动进入 settle/replan；若 motion tracking evidence 不通过，则必须停在
   `APPROACH_TRACKING_NOT_CONTINUOUS`，不得继续；
10. 复测仍会看到每条 guarded action 的 HOLD/短 settle 边界。这一点是当前已知架构边界，
    不能把它误报成真正连续 actuator motion。

当这个单步门禁通过后，再执行 FAR→NEAR 多步，验证每一段 motion window 都有独立的 tracker
coverage 证据。只有随后引入 rolling target actuator 接口并消除正常步骤的 action-complete/HOLD
边界，才可把“robotarm 持续运动”标为完成。

## 2026-09-18 连续 bag 的 full-rate tracker + 10 Hz shadow control 验证

### 结论先行

刚记录的 bag **可以**回答以下两个问题：

1. 录下来的机械臂运动画面能否被当前 tracker 持续跟踪；
2. 在同一个固定 100 ms 控制周期内，是否有时间消费最新 tracker/TF、计算 image-Jacobian
   修正量，并运行当前真实 IK。

两个答案均为“可以，离线门禁通过”。但它不能把以下命题变成已验证事实：

- 新修正后的 ROS callback 在线运行时一定不会再次被 DDS/GIL/USB 调度阻塞；
- 当前 one-point `FollowJointTrajectory` 在 action 尚未完成时会消费 tracker 修正；
- tracker centroid 等于物体的绝对真值；
- 机械臂会持续无 HOLD 地运动并实时接受 rolling target。

尤其最后一点，当前生产路径尚未实现。离线 shadow controller 是“如果每 100 ms 计算一次，
计算能否按时完成”的可行性证明，不是“已经向电机每 100 ms 下发修正”的执行证明。

### 可复现工具与输入

扩展了只读工具：

```text
tools/touch_anything/replay_continuous_tracker_bag.py
```

输入：

```text
bag:
  debug-output/touch-anything-bags/touch_regression_20260918_140539

human confirmation A:
  debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/
    arm_observation_b03454a2-0641-4f0a-b077-8fcbfdefd26e.json

human confirmation B:
  debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/
    arm_observation_afae76f4-7d6a-43c5-87ab-56966b8c5866.json
```

输出：

```text
debug-output/touch-anything-bags/touch_regression_20260918_140539/
  tracking_control_cycle_replay_b034.json
  tracking_control_cycle_replay_afae.json
```

工具执行以下真实逻辑，不使用伪造的恒定耗时：

```text
human-confirmed RGB/mask
  → ContinuousTargetTracker.initialize()
  → bag 中每一帧 /arm_camera/color/image_raw
  → ContinuousTargetTracker.update()
  → processing 完成后才允许 10 Hz control tick 读取

/tf_static + /tf
  → 通用 TF graph 正反向组合
  → 每个 Arm joint update 重建 base_link <- arm_camera_color_optical_frame

最新 tracker centroid + 录制 K/D + 动态 camera rotation + 初始 D435 range scale
  → base_yz_pixel_alignment_delta()
  → 每周期最大 5 mm 的 bounded shadow output

motor_evidence.json 中两个真实 waypoint/seed/tool axis/reference id
  → solve_pre_touch()，每个 waypoint 重复 20 次
```

所有输出只写 JSON；不发布 ROS topic，不创建 action client，也不触碰 USB/actuator。

### Confirmation A：覆盖两次 approach 的完整结果

从第一个人工确认点开始回放 `111.038 s / 1668 frames`：

| 项目 | 结果 |
|---|---:|
| camera input | `15.0128 Hz` |
| tracker accepted | `1668 / 1668 = 100%` |
| tracker method | `PYRAMID_KLT_RANSAC` 全部帧 |
| ORB / SAM recovery | `0 / 0` |
| accepted 最大 gap | `67.033 ms` |
| tracker processing median / p95 / max | 约 `2.6 / 3.33 / 6.05 ms` |
| valid features min / median | `12 / 34` |
| 10 Hz control ticks | `1111 / 1111` 成功 |
| stale / non-finite / missed control ticks | `0 / 0 / 0` |
| tracker observation age median / p95 | `36.23 / 64.80 ms` |
| dynamic TF age p95 | `177.89 ms` |
| image-Jacobian compute median / p95 / max | `0.083 / 0.087 / 0.462 ms` |
| bounded output tick-to-tick jump p95 | `0.103 mm` |
| IK overall p95 / max | `12.68 / 13.64 ms` |
| IK success | 两 waypoint 均 `20 / 20` |

对 100 ms 周期，保守组合 p95 约为：

```text
tracker 3.33 ms + image-Jacobian 0.09 ms + IK 12.68 ms ≈ 16.10 ms
```

这里没有把三个 p95 当作统计学上的联合 p95，只把它用作保守预算相加。即便如此，距离
100 ms deadline 仍有约 84 ms；tracker 是 15 Hz 输入，控制器在每个 10 Hz tick 都取得了一个
新的已完成 observation，`repeated_observation_ticks = 0`。

### 两个真实动作窗口，不再只看全程平均值

工具直接从 `/arm/dashboard_state.execution.event_ros_sec` 提取动作开始/结束，而不是使用 Touch
延迟收包时间。

| 动作 | duration | tracker frames | 最大 gap（含首尾） | centroid 位移 | control ticks |
|---|---:|---:|---:|---:|---:|
| Approach 1 | `1.439 s` | `22 / 22` | `67.03 ms` | `112.15 px` | `14 / 14` |
| Approach 2 | `1.377 s` | `21 / 21` | `66.68 ms` | `57.25 px` | `14 / 14` |

这回答了“录制的数据里，两次运动画面实际上能不能持续 tracking”：**能。** 旧在线 tracker
之所以没有留下连续证据，不是图像本身不可跟踪，而是 Touch 进程被 raw overlay publish 阻塞。
离线 full-rate 路径在动作窗口内没有 LOST/RECOVERED，且目标跨越 57–112 px 后仍由 KLT/RANSAC
保持同一 track。

动作窗口内相邻 centroid 步长也处于金字塔 KLT 可处理范围：

```text
Approach 1: median 6.75 px, p95 12.02 px, max 14.51 px, min inlier 0.85
Approach 2: median 7.30 px, p95 13.78 px, max 13.82 px, min inlier 0.824
```

### Confirmation B：独立初始化复测

从第二个人工确认点开始回放 `92.188 s / 1385 frames`：

| 项目 | 结果 |
|---|---:|
| tracker accepted | `1385 / 1385 = 100%` |
| accepted 最大 gap | `67.011 ms` |
| tracker processing median / p95 / max | `2.63 / 3.74 / 7.55 ms` |
| valid features min / median | `21 / 40` |
| 10 Hz control ticks | `922 / 922` 成功 |
| stale / non-finite / missed control ticks | `0 / 0 / 0` |
| observation age p95 | `65.37 ms` |
| dynamic TF age p95 | `128.30 ms` |
| control compute p95 / max | `0.089 / 0.415 ms` |
| IK p95 / max | `12.57 / 13.20 ms` |
| 覆盖到的 Approach 2 | tracker `21 / 21`，control `14 / 14` |

第一条 approach 发生在第二次 confirmation 之前，因此报告明确标成
`fully_covered_by_replay=false`，不把零帧误算为 tracker 失败。所有“covered motion window”仍通过。

### “tracking 精度”能证明到哪一层

这份 bag 没有逐帧人工 polygon、AprilTag、外部 motion capture 或已知 3-D 真值，因此不能报告
“绝对误差 X mm/px”。当前能报告的是三层代理量：

1. **连续性：** 3053 个 replay frame 全部接受，最大 gap 约 67 ms；
2. **几何自洽：** 运动窗口最小 RANSAC inlier ratio 约 0.824，未触发 recovery；
3. **独立初始化重复性：** 两次 human-confirmed mask 在共同的 1385 帧上，centroid 差
   `median/p95 = 11.951 px`、`max = 12.530 px`。

第三项主要反映两个人工确认 SAM mask 的形状/质心基准不同。偏差近似恒定，不像运动中随机漂移；
但约 12 px 已接近当前 15 px z-first 对齐阈值，因此不能忽略。最终 PRESS 精度应继续依靠近场
重新确认、fixed-tip/mask inside 条件和真实接触证据，不能把 FAR tracker centroid 当成毫米级真值。

### 同一控制周期门禁定义

报告只有同时满足下列条件才设置：

```text
gates.tracking_and_control_same_cycle_ready = true
```

条件为：

- camera input `>= 10 Hz`；
- tracker accepted rate `>= 5 Hz`，accepted fraction `>= 90%`；
- tracker p95 processing `<= 100 ms`；
- 10 Hz shadow control success fraction `>= 99%`；
- shadow output 全部 finite；
- observation age p95 `<= 150 ms`；
- dynamic TF age p95 `<= 300 ms`；
- image-Jacobian compute p95 不超过周期的 25%；
- 两个录制 waypoint 的真实 IK 全部成功，overall p95 `<= 25 ms`。

本次 A、B 两个报告该 gate 均为 `true`；A 覆盖的两个动作窗口
`all_covered_motion_windows_trackable` 也为 `true`。报告故意把
`real_continuous_arm_motion_verified` 固定为 `false`，避免把 shadow computation 误读成 actuator
streaming 已验证。

### 下一次测试前后的判定

下一次上真机前，现在可以确认的是：

```text
15 Hz tracker + 10 Hz control geometry + 当前 IK
在本机离线计算预算内可以稳定共存；录制的两段真实运动图像可持续跟踪。
```

仍需在下一次真机单步测试中确认修正后的在线调度：

```text
overlay motion-defer 生效
→ tracker motion-window coverage 连续
→ terminal event transport delay 恢复正常
→ endpoint gate 通过
→ 下一 guarded step 能被及时发出
```

若验收目标是“机械臂在一条 action 内每 100 ms 根据 tracker 修正并持续运动”，则不能只靠下一次
bag 确认；需要先实现 rolling/short-horizon actuator 接口、active-goal 更新或等价 servo 通道，
以及 watchdog/lost-target 时唯一触发 HOLD 的安全语义。当前 one-point action 在执行中拒绝新 goal，
所以这部分应继续保持未完成，不能因为 shadow gate 为 true 就把它勾掉。

### 本轮自动化保护

新增单元测试覆盖：

- TF graph 两段正向组合及反向求逆；
- 20 Hz synthetic tracker + 10 Hz control 对最新 observation 的消费；
- 每个 control tick 的 TF/observation freshness；
- bounded output 全 finite，且没有重复 observation tick。

这样以后修改 TF 重建或 shadow clock，不必重新接真机才能发现基本回归。

## 2026-09-18 Bag Tracking Debugger、录制完整性与下一次测试硬门禁

### 决策

下一次真机运动测试的目标不再是重复当前 stop-observe-replan 行为，而是同时验证：

```text
A. Arm 运动期间，tracker 对每个新 Arm RGB 持续产生可用结果；
B. 同一固定控制周期消费最新 tracker/TF/feedback；
C. Action/servo 仍处于运动状态时，控制器实际更新 actuator target；
D. LOST/stale/deadline/watchdog 时停止更新并进入受控 HOLD。
```

只满足 A/B 的 shadow computation 不等于满足 C。当前代码仍是 one-point
`FollowJointTrajectory → SUCCEEDED → HOLD → observe/replan`，所以：

```text
next real motion test ready = false
```

在 rolling target/short-horizon servo 控制通道实现之前，不开始下一次真实 approach/PRESS 测试。

### 新工具：逐帧 Bag Tracking Debugger

入口：

```text
tools/touch_anything/bag_tracking_debugger.py
```

基本命令：

```bash
tools/touch_anything/bag_tracking_debugger.py \
  debug-output/touch-anything-bags/touch_regression_20260918_140539 \
  --initial-arm-observation \
    debug-output/touch-anything/observation_b170be50-6726-4ce1-ac0b-014b6f8c51a9/arm_observation_b03454a2-0641-4f0a-b077-8fcbfdefd26e.json
```

播放器能力：

- 从人工确认 keyframe 开始重新运行生产 `ContinuousTargetTracker`；
- raw image 不整体载入内存，利用 rosbag sqlite row id 随机读取当前帧；
- `Space` 播放/暂停，左右方向键逐帧；
- `0.05x / 0.1x / 0.25x / 0.5x / 1x / 2x / 4x / 8x`；
- 点击 timeline 或输入 tracking 起点后的相对秒跳转；
- timeline 用绿色表示 TRACKING、橙色表示 recovery/health event、红色表示 LOST/error、
  蓝色表示 Action/Touch state/current position；
- 事件列表可跳转到准确帧；
- 可独立启用以下自动断点：
  - `LOST / RUNTIME_ERROR`；
  - KLT↔ORB、RECOVERED、status/method 切换；
  - camera gap、低 point count、低 inlier ratio；
  - Action state、Touch state 切换。

画面叠加：

```text
propagated mask contour/fill
KLT previous→current flow vectors
RANSAC accepted feature points
mask centroid
Arm Camera image center
fixed touch-tip pixel
offline tracker status/method/points/inlier/mask/compute time
recorded online tracker result（新 bag）
gateway Action state/request id
```

当前帧右侧 JSON 保留完整、可复制的诊断信息：

```text
image header stamp / bag receive stamp / transport delay
offline recomputed tracker result + reason + affine
online tracker tick + callback duration + generation（新 bag）
online 10 Hz control tick + tracker age + decision（新 bag）
offline shadow-control bounded/raw YZ delta（已有 replay report 时）
Touch state + approach plan
Dashboard execution event
joint positions
trajectory desired/actual/error
base_link <- arm_camera rotation + TF age
events at current frame
```

工具完全只读，不启动 ROS publisher/action client，不会发送 Arm/base 命令。

### 当前 bag 的新发现

完整 GUI backend 回放结果保存在：

```text
debug-output/touch-anything-bags/touch_regression_20260918_140539/
  bag_tracking_debugger_b034_summary.json
```

结果：

```text
frames: 1669（含初始人工 keyframe）
INITIALIZED: 1
TRACKING: 1668
RECOVERED/LOST/RUNTIME_ERROR: 0
first_loss: null
```

唯一一次低健康度断点在 tracking start 后约 `32.572 s`：

```text
image stamp: 1789708014.650490
method: PYRAMID_KLT_RANSAC
tracked points: 16
inlier ratio: 0.5714286
```

它没有越过当前 minimum inlier `0.50`，下一帧继续正常 TRACKING；其时间位于第二次 approach
开始前约 127 ms，值得在界面逐帧确认遮挡、光照和目标边缘变化，但不是 LOST 起点。

这也说明事件断点不能只找红色 LOST；临界但通过的低 inlier frame 也应保留。

### 为什么旧 bag 仍不能回答“线上具体漏了哪个 tracker tick”

对 `touch_regression_20260918_140539` 的录制完整性审计：

```text
Arm RGB: 2247 frames, 15.0127 Hz, max gap 67.48 ms, stamp monotonic
missing: /arm/mount_transform
missing: /touch_anything/tracker_debug
missing: /touch_anything/control_debug
```

旧 `/touch_anything/state` 只有 156 条，约 2 Hz。它能看见最后一次 tracker result 与聚合状态，
但无法区分：

```text
timer 没有调度
timer 调度但看到重复 image stamp
callback 开始后卡住
tracker result 被 generation reset 丢弃
结果产生但状态/overlay publish 延迟
10 Hz control tick 没运行或主动决定不发命令
```

所以旧 bag 适合“raw frames 是否可跟踪”的离线真值复算，不足以精确还原在线每个 callback。

### 新增每 tick 在线遥测

Touch node 新增两个小型 best-effort JSON topic：

```text
/touch_anything/tracker_debug
/touch_anything/control_debug
```

tracker tick 记录：

```text
image_stamp
generation
RESULT / DISCARDED_GENERATION / EXCEPTION
callback start/completion monotonic time and duration
input receive age at callback start
status/method/reason/centroid/mask/features/inlier/affine
Touch automatic state
gateway execution state/request id
```

control tick 记录：

```text
sequence
callback duration
Touch state before/after
automatic approach before/after
tracker stamp/age/result consumed by this tick
gateway execution state
whether a guarded step was dispatched
whether an active goal target was updated this tick
control policy
exception
```

关键约束：tracker/control callback **只写有界 deque**。另一个 telemetry callback group 每 20 ms
drain 小型 String，因此 rosbag/DDS 变慢不能再次直接阻塞被测 tracker/control callback。queue depth
和 dropped count 同时写入 `/touch_anything/state`；任一 dropped count 非零会让录制审计失败。

2026-09-18 完成 rolling controller 后，control telemetry 写：

```text
rolling_target_control_implemented = true
active_goal_target_updated_this_tick = <本周期是否实际发送 UPDATE>
control_policy = ROLLING_VISUAL_SERVO_10HZ_TRACKER_20HZ_ACTUATOR
rolling_visual_servo_session_id = <session UUID>
rolling_visual_servo_sequence = <单调递增 sequence>
rolling_visual_servo_updates_sent = <Touch 侧累计发送数>
rolling_visual_servo_stop_reason = <正常结束或安全停止原因>
```

这些字段由实际 START/UPDATE/STOP 路径设置，不是为通过门禁而修改的常量。录包审计还会与
`/arm/visual_servo_state` 的 `sequence_applied` 和 `update_count` 交叉验证，所以 Touch 单方面报告
`active_goal_target_updated_this_tick=true` 不能通过门禁。

### 录包脚本审计与补齐

`record_touch_regression_bag.sh` 原来已经记录 raw RGB、TF、JointState、Dashboard、controller、
Action feedback/status、Touch state 和 rosout；新增/补齐：

```text
/arm/mount_transform
/arm/control_mode
/arm/hold_command
/arm/visual_servo_command
/arm/visual_servo_state
/arm/candidate_joint_states
/display_planned_path
/touch_anything/tracker_debug
/touch_anything/control_debug
/chokudomotor/target_angle
/cameraswingmotor/target_angle
/nav_mode/command
/parameter_events
```

录制前检查关键 topic 是否存在；缺失时默认拒绝启动，避免完成真机动作后才发现证据不完整。
`--skip-preflight` 只用于明确的非运动诊断 bag。

录制结束后自动：

1. dump `/touch_observation_node`、`/so101_arm_gateway`、
   `/so101_real_trajectory_server` 参数；
2. 运行 `audit_touch_regression_bag.py`；
3. 写入 `recording_audit.json`；
4. 若核心 topic/per-tick debug 缺失或 debug queue drop 非零，保留 bag 但返回失败。

审计将以下两个事实分开：

```text
motion_tracking_observed
realtime_motion_correction_observed
```

每个 approach motion window 都必须既有 tracker ticks，又有
`active_goal_target_updated_this_tick=true`。只有两者均通过，才允许：

```text
next_real_motion_test_ready = true
```

### rolling control 实现状态（2026-09-18）

没有复用或重入 `FollowJointTrajectory`。新增独立协议 `SO101_VISUAL_SERVO_V1`。2026-09-18
第二轮修正后，session 不再按 2.5 秒 segment 切断，而是覆盖整个 FAR→NEAR 连续 approach；
NEAR 对准后仅为必须的人工 PRESS 确认进入一次 HOLD，确认后以第二个连续 session 执行 PRESS：

```text
Touch preview accepted
  -> Gateway visual_servo_start(sequence=0)
  -> real/mock server ACTIVE
  -> FAR_TRACK_AND_ADVANCE: Touch 10 Hz 短 X horizon + 动态 Y/Z
  -> NEAR_ALIGN: X hold + fixed-tip Y/Z alignment
  -> stable alignment (3 frames) -> STOP/HOLD -> PRESS confirmation
  -> PRESS session: 10 Hz tracking + short X horizon + Y/Z correction
  -> appearance success or maximum press travel
  -> server 20 Hz velocity-limited Goal_Position
  -> visual_servo_stop(sequence=N+1)
  -> server HOLDING
```

Gateway 保持 `owner_request_id`，因此每个 UPDATE 使用唯一 dashboard `request_id` 时，整个运动窗口
仍可归属于最初 `touch-approach-execute-*`。rolling session 活跃时拒绝普通 action、mode switch、
release HOLD 和 safe shutdown，避免两个 command owner 同时写机械臂。

Touch 每个 10 Hz tick 使用：最新 tracker centroid、tracker age、当前 arm-camera TF、当前
`arm/touch_tip` TF、当前 joint feedback 和 gateway effective joint limits。FAR 的 X target 始终是
“当前实测 X + 6 mm short horizon”，并硬限制在 FAR+35 mm NEAR plane；NEAR 对准时 X 保持当前实测值；
PRESS 使用相同 6 mm horizon，并硬限制在 target plane + 最大 press overtravel。Y/Z 由当前质心到
FAR image center 或 NEAR fixed tip 的 pinhole Jacobian 计算，每 tick Cartesian 修正 norm 上限 3 mm，
然后重新求 IK。IK 失败、TF 缺失、tracker 状态不再是
INITIALIZED/TRACKING/RECOVERED，或最后 good frame 超过 200 ms时立即发送 STOP。

Real server 以 20 Hz 读取同一 session 的最新目标，使用 learned raw bounds、calibration margin 和
每关节速度上限逐步写 `Goal_Position`。正常 UPDATE 期间继续处于已监控的六轴 HOLD，不经过
action-complete/HOLD 周期；只有显式 STOP、20 s server session deadline、350 ms command watchdog、模式变化
或保护异常进入终态 HOLD。HOLD monitor 在 servo active 时使用专用 position-error threshold，电流、
温度、扭矩完整性保护继续生效。watchdog 无法建立当前姿态 HOLD 时才释放扭矩并锁存 FAULT。

实现清单：

1. [x] 独立 `/arm/visual_servo_command` 与 `/arm/visual_servo_state`；
2. [x] Touch 10 Hz / actuator 20 Hz，消费 tracker stamp、动态 TF、joint feedback；
3. [x] 6 mm rolling X horizon + FAR/NEAR/PRESS hard cap + 每 tick 最大 3 mm 动态 Y/Z；
4. [x] effective position envelope、IK error/rotation gate、server velocity envelope；
5. [x] server `sequence_applied`/`update_count` acknowledgement 和 per-tick telemetry；
6. [x] tracker/TF/IK/session/command deadline 的 STOP 或 watchdog HOLD；
7. [x] 正常 UPDATE 不产生 action-complete/HOLD；
8. [x] Mock 13 次 server-acknowledged target update，无 overlapping action goal，正常 STOP=HOLDING；
9. [x] Mock 故意停止 UPDATE，350 ms watchdog 后得到 WATCHDOG_HOLD；
10. [x] bag recorder 将 command/state 设为 critical topic，audit 要求 Touch 与 server 双侧证据；
11. [x] FAR→NEAR 同一 session；只在 PRESS 人工确认点 HOLD；PRESS 在第二 session 连续执行；
12. [ ] 重启在线栈后，真机先做一次受监督 FAR→NEAR，录 bag 确认真实 tracking/correction 质量。

### 真机前门禁结论（2026-09-18）

运行态检查发现当时在线 `/touch_observation_node` 仍加载上一版参数
`rolling_visual_servo_duration_sec=2.5`，并停在 `APPROACH_CARTESIAN_NOT_REACHED`；编译安装新版本不会
热替换现有 Python 进程。因此门禁分为：

```text
compiled_continuous_until_press_ready = true
currently_running_stack_has_new_lifecycle = false  # 必须重启完整 Arm/Touch launch
```

重启后必须在任何运动前从 `/touch_anything/state` 确认：

```text
rolling_visual_servo.continuous_until_press = true
rolling_visual_servo.phase = INACTIVE
rolling_visual_servo.gateway.ready = true
```

Mock 报告：

```text
debug-output/touch-anything-regression/mock_20260918_160814/readiness.json
normal_update_count = 13
active_sequences_observed = 0..12
distinct_requested_pan_targets = 13
applied_pan_span_rad = 0.2147
normal_terminal_state = HOLDING
watchdog_terminal_state = WATCHDOG_HOLD
overlapping_action_goal = false
realtime_motion_correction_observed = true   # Mock 运动执行层证据
next_real_motion_test_ready = true
```

这里的 `realtime_motion_correction_observed=true` 明确表示 Mock 执行器在同一 active session 中实际
接受并执行了多次实时目标修正；它不冒充真机观测。下一次真机 bag 的同名字段仍必须由
`audit_touch_regression_bag.py` 根据 tracker/control/server 三方时间窗重新计算。第一次真机测试范围仍仅允许
一个受监督的 FAR→NEAR 连续 session，在 PRESS confirmation HOLD 处结束并 review；若 `/arm/visual_servo_state` 未显示 ready、tracker age 超过
200 ms、control debug drop 非零，或 recording preflight 缺少任一 critical topic，不得开始运动。

## 2026-09-18 15:58 真机 rolling 测试：Z 下坠根因与闭环补偿修正

### 结论

这次测试已经证明「运动中 tracker + 同一 active session 内连续目标更新」在真机上确实运行；目标丢失
不是 tracker/control callback 停止造成的，而是 Cartesian Z 控制方向与真实机构响应没有被闭环吸收。
旧审计中的两个布尔值均为真，只能证明控制链在工作，不能证明控制方向或 Cartesian 精度正确：

```text
bag = debug-output/touch-anything-bags/touch_regression_20260918_155846
motion_tracking_observed = true
realtime_motion_correction_observed = true
tracker ticks in motion = 25
control ticks in motion = 25
active target updates = 24
server applied sequences = 0..25
server faults = 0
```

因此本轮把 readiness 拆成两层：

```text
rolling_transport_and_tracking_proven = true
cartesian_z_closed_loop_proven = false       # 等待下一次真机验证
next_real_motion_test_ready = false          # 在线进程尚未加载本轮补偿
```

### Bag 定量结果

新增只读工具 `tools/touch_anything/analyze_motion_compensation.py`，把每个 visual-servo 关节目标通过
与线上一致的 FK 转成 base Cartesian target，再与时间最近的 `/joint_states` 对齐。报告写入：

```text
debug-output/touch-anything-bags/touch_regression_20260918_155846/
  motion_compensation_analysis.json
```

该 session 持续 `2.523 s`，共有 25 个 command samples：

| 量 | X | Y | Z |
|---|---:|---:|---:|
| 实际末端位移 | +18.709 mm | +10.533 mm | **-44.808 mm** |
| 最后控制目标相对起点 | +25.000 mm | +10.656 mm | **-35.337 mm** |
| 最后目标 − 最后实际 | +6.291 mm | +0.123 mm | **+9.470 mm** |

这说明 44.8 mm 下坠包含两个不同问题：

1. 控制器本身把最后 Z 目标放到了起点下方 35.3 mm；这不是单纯的电机重力下垂；
2. 真实末端又比该目标低 9.47 mm；这是机构/关节跟随的剩余误差。

图像上 tracker centroid 从 `[370.58, 173.87]` 移到 `[495.53, 244.80]`，水平向右约
`124.95 px`。此前真机标定已经确认「目标在图像右侧 => arm tip 应向 +base Z 移动」，但旧 rolling
controller 使用 dynamic-TF image Jacobian，实际给出了相反的 Z 方向。因此 tracker 一直是健康的，
却忠实记录了目标逐渐离开视野。

另一个关键时序是：

```text
sequence 0:
  target_z - actual_z = +39.813 mm
  # 包含历史 FAR +22 mm 向上补偿

sequence 1, 约 104 ms 后:
  target_z - actual_z = -1.387 mm
  # 新目标重新从 current actual 生成，旧的未完成补偿被覆盖
```

所以原有 `approach_far_feedforward_z_up_bias_m=22 mm` 只影响 session 的第一个目标；rolling update
没有携带「上一目标 − 当前实际」残差，导致补偿在第一个 10 Hz 周期后失效。

### 新的每周期控制逻辑

本轮在 `rolling_z_motion_compensation()` 中显式引入目标与实际差值。周期 `k` 使用：

```text
e_z[k] = previous_target_z[k-1] - measured_actual_z[k]

x_scale[k] = clip(
  (20 mm - abs(e_z[k])) / (20 mm - 6 mm),
  0, 1
)

x_horizon_effective[k] = 6 mm * x_scale[k]
z_x_feedforward[k] = 0.88 * x_horizon_effective[k]  # FAR/PRESS only

z_compensation[k] = clip(
  0.80 * e_z[k] + z_x_feedforward[k],
  -25 mm, +25 mm
)

new_target_z[k] = measured_actual_z[k]
                + bounded_visual_delta_z[k]
                + z_compensation[k]
```

行为含义：

- 上一周期要求向上、但实际末端没有到达时，未完成量不会在下一帧消失；
- `|e_z| <= 6 mm` 时允许完整 6 mm X short horizon；
- `6 mm < |e_z| < 20 mm` 时按比例减慢 X；
- `|e_z| >= 20 mm` 时 X horizon 变为 0，先修复 Z，不再边掉落边向前追目标；
- FAR/PRESS 每前进 6 mm，额外请求约 `+5.28 mm Z`，复用历史 FAR 下坠补偿关系；
- compensation 最大绝对值 25 mm，避免错误反馈造成无界累计。

用本 bag 的 sequence-0 残差代入新逻辑：`e_z=+39.8 mm`，因此 X 立即暂停，补偿被限制为
`+25 mm`；而不是旧逻辑在 sequence 1 把 Z 目标改成当前实际附近。这正是“目标与实际运动差值”
进入同一个控制周期的具体位置。

### 真机像素闭环替换

continuous FAR/NEAR/PRESS 不再让未验证的 rendered-TF Jacobian 直接控制 Y/Z，改用已有真机方向：

```text
pixel_error = centroid - alignment_pixel
delta_base_y = -pixel_error_v * 0.20 mm/px
delta_base_z = +pixel_error_u * 0.20 mm/px
norm(delta_base_yz) <= 3 mm/tick
```

其中 FAR 的 `alignment_pixel` 是图像中心，NEAR/PRESS 是固定 tool-tip pixel。原 Jacobian 结果仍逐周期
记录为 `jacobian_diagnostic_delta_base_yz_m`，只用于比较和后续重新标定，不再参与真机 rolling command。
该变化直接覆盖本次故障：目标向图像右侧移动时，新控制给出 `+Z`，不会继续向下。

### 安全保护与 telemetry

增加独立的实际位姿保护：任意 rolling phase 中，若当前实际 Z 相对本 session 起点下降超过
`12 mm`，立即 STOP/HOLD，reason 为 `CARTESIAN_Z_DROP_LIMIT`。12 mm 是失败保护，不是正常容差；
正常补偿应在达到它之前让 X 减速/暂停并把 Z 拉回。

下一次 `/touch_anything/control_debug` 每个实际控制周期新增 `rolling_control`，包含：

```text
actual_tip_base_m
previous_waypoint_base_m
previous_target_error_base_m
new_target_error_base_m
waypoint_base_m
centroid_uv / pixel_error_uv
alignment_mapping
unbounded_delta_base_yz_m / bounded_delta_base_yz_m
jacobian_diagnostic_delta_base_yz_m
z_tracking_error_m
z_drop_from_session_start_m
z_compensation:
  residual_carry_z_m
  model_feedforward_z_m
  compensation_z_m
  command_delta_z_m
  x_horizon_scale
  scaled_x_horizon_m
```

这些值与 `/arm/visual_servo_command`、`/arm/visual_servo_state`、`/joint_states` 同包记录后，可以逐帧
区分：视觉要求、补偿要求、发出的关节目标、server applied sequence、实际 Cartesian 响应。

### 实现与验证状态

涉及文件：

```text
src/touch_anything/scripts/approach_geometry.py
src/touch_anything/scripts/touch_observation_node.py
src/touch_anything/config/touch_anything.yaml
src/touch_anything/test/test_approach_geometry.py
tools/touch_anything/analyze_motion_compensation.py
tools/touch_anything/README.md
```

验证结果：

```text
python py_compile = PASS
touch_anything tests = 79 passed
so101_arm_control + touch_anything tests = 144 passed
colcon build --packages-select so101_arm_control touch_anything = PASS
real-bag Cartesian analysis = PASS
```

回归测试固定了这次 bag 的关键方向：centroid 到 `u=495.53` 时 empirical mapping 必须给出正 Z；
`39.8 mm` 未完成 Z target 必须令 X horizon=0，并在下一目标保留向上补偿。

### 下一次真机启动与验收门禁

截至本次修改完成时，当前仍在运行的 `/touch_observation_node` 是修改前进程：

```text
rolling_visual_servo_continuous_until_press = true
rolling_visual_servo_duration_sec = 2.5
rolling_visual_servo_empirical_m_per_px = Parameter not set
rolling_visual_servo_max_z_drop_m = Parameter not set
```

源码与 install 已构建，但 ROS Python 进程不会热加载。因此不得直接用当前进程开始下一次运动；需要由
operator 按正常流程重启完整 Arm/Touch launch。重启后、运动前必须只读确认：

```text
rolling_visual_servo_empirical_m_per_px = 0.00020
rolling_visual_servo_z_residual_gain = 0.80
rolling_visual_servo_far_z_per_x_gain = 0.88
rolling_visual_servo_max_z_drop_m = 0.012
rolling_visual_servo_continuous_until_press = true
/arm/visual_servo_state.ready = true
```

下一次仍限定为一个受监督 FAR→NEAR session，并在 PRESS confirmation HOLD 处 review。通过标准：

1. tracker/control/server 三方 sequence 连续，debug queue dropped=0；
2. 每个 active update 的 `alignment_mapping=REAL_ROBOT_EMPIRICAL_YZ_V1`；
3. Z residual 增大时 `x_horizon_scale` 确实下降，超过 20 mm 时 X 不再推进；
4. 正常运行不得触发 12 mm Z-drop hard stop，实际最大下坠目标为不超过 8 mm；
5. 图像水平误差不再随 X 前进单调发散，rightward error 必须对应正 Z command；
6. 到达 NEAR 后进入稳定 alignment，再 HOLD 等人工 PRESS 确认；本轮不执行 PRESS；
7. 对新 bag 再运行 `audit_touch_regression_bag.py` 与 `analyze_motion_compensation.py`，确认最后
   target-minus-actual Z 误差收敛且没有把 Z target 放到 session 起点下方。

只有新 bag 同时通过以上 Cartesian 质量门禁后，才更新：

```text
cartesian_z_closed_loop_proven = true
next_real_motion_test_ready = true
```

## 2026-09-18 16:33 bag：tracker freshness 误停修正

### 实际故障时间线

最新 bag：

```text
debug-output/touch-anything-bags/touch_regression_20260918_163317
```

rolling session `touch-visual-servo-fc98d69c-...` 从 START 到 HOLD 仅 `0.634 s`。gateway/server
接受 START 和 sequence `1..6`，没有 WATCHDOG_HOLD/FAULT/REJECTED。Touch 的停止理由是：

```text
tracking unavailable during motion: status=TRACKING, age=0.279s
```

停止周期的真实并发关系：

```text
latest completed tracker age at control tick = 280.5 ms
maximum completed age                   = 200.0 ms

new tracker callback already in progress:
  processing age at control tick        = 36.9 ms
  input receive age at control tick     = 71.0 ms
  completed after that control tick      = 24.3 ms
  eventual result                        = TRACKING / PYRAMID_KLT_RANSAC
```

因此不是目标先 LOST，而是旧逻辑只观察“最后完成时间”，没有观察 tracker callback 已经 claim 了一张
新鲜帧并正在处理。control callback 比该 tracker callback 早 24.3 ms 到达判定点，于是错误 STOP。

### 三态 freshness gate

新增纯函数 `rolling_tracker_freshness_gate()`，输入 completed age、in-flight processing age、
in-flight input receive age及 tracker status，输出：

| 判定 | 条件 | 控制动作 |
|---|---|---|
| `FRESH` | status healthy 且 completed age ≤ 200 ms | 正常生成并发送新目标 |
| `WAIT_INFLIGHT` | completed 已过期，但输入年龄 ≤ 200 ms 且 processing ≤ 150 ms | 本 tick 不发旧目标、不 STOP，等待正在处理的帧 |
| `STOP` | LOST/RUNTIME_ERROR，或 completed/in-flight 均超时 | 立即 STOP/HOLD |

`WAIT_INFLIGHT` 特意不发送重复旧目标，因此不会用陈旧视觉结果刷新执行器 watchdog。若 tracker 没有在
150 ms 内完成，下一个 control tick 会进入 STOP；即使 Touch control callback 本身停止，real server
的独立 350 ms command watchdog 仍会 HOLD。

online tracker 在 claim 最新 Arm-camera frame 时记录：

```text
_tracker_inflight_started_monotonic
_tracker_inflight_image_received_monotonic
_tracker_inflight_stamp
_tracker_inflight_generation
```

callback 的所有成功、LOST、exception、generation-discard 路径都在 `finally` 中清除同一 generation/stamp
的 in-flight 标记，避免遗留假 in-flight 状态。reset/cancel/new confirmation 同时显式清空这些字段。

### 共享锁阻塞修正

该 bag 在 STOP 后还记录到 tracker `8.064 / 6.324 / 5.125 / 5.081 s` 和 control
`6.192 / 4.970 s` 的并发 stall。两种 callback 同时停顿，且 tracker callback 计时从取得共享状态锁
之前开始，说明主要风险是共享 `_lock` 长时间占用，而不是 KLT 单独变慢。

`_on_arm_state()` 会持有外层 `RLock` 运行状态机；过去其中调用 `_set_state()->_publish_state()` 时，
虽然 `_publish_state()` 自己的内层 lock scope 结束，外层 RLock 仍未释放，随后可靠/transient DDS 状态
发布可能在 rosbag/RViz backpressure 下继续占有全局锁。现在 `_set_state()` 检测当前线程仍持有外层
RLock 时，只更新内存状态并记录日志，把 DDS state publication 交给已有 2 Hz state timer。无外层锁
的调用仍立即发布。

新增观测字段：

```text
/touch_anything/tracker_debug.initial_lock_wait_ms
/touch_anything/tracker_debug.frame_claimed_monotonic_sec
/touch_anything/tracker_debug.input_receive_age_ms_at_claim
/touch_anything/control_debug.initial_lock_wait_ms
/touch_anything/control_debug.rolling_tracker_gate
/touch_anything/state.debug_telemetry.state_publish_deferred_count
```

下一次 bag 可区分 callback 是在等待共享锁、处理图像，还是等待 DDS/其他资源。

### 最新 bag 离线回放

新增只读工具：

```text
tools/touch_anything/replay_tracker_freshness_bag.py
```

执行：

```bash
tools/touch_anything/replay_tracker_freshness_bag.py \
  debug-output/touch-anything-bags/touch_regression_20260918_163317 \
  --json-output \
  debug-output/touch-anything-bags/touch_regression_20260918_163317/tracker_freshness_replay.json
```

结果：

```text
tracker_intervals = 19
rolling_control_ticks = 7
legacy_false_stop_ticks = 1
bounded_inflight_recoveries = 1
legacy decision at failure = STOP
new decision at failure = WAIT_INFLIGHT
false_stop_eliminated = true
offline_tracker_freshness_gate_passed = true
```

回放直接使用 bag 中实际 callback monotonic start/end 和 input receive age，不用相机 header stamp；该相机
header 在此 bag 中相对 rosbag record time固定偏移约 0.52 s，不能用于短 session 的 callback freshness。

### 验证与真机门禁

```text
approach geometry/freshness tests = 40 passed
so101_arm_control + touch_anything = 147 passed
Python compile = PASS
colcon build = PASS
20260918_163317 real-bag freshness replay = PASS
```

离线 PASS 证明：对已经发生的 279 ms 误停，新控制会安全等待正在运行且输入新鲜的 callback，并在
24.3 ms 后获得健康结果；它不等于已经证明真实机构能把剩余约 20 mm Z error 完全收敛。因此下次
仍只允许一个受监督 FAR→NEAR session，在 PRESS confirmation HOLD review。

源码/install 已更新，但正在运行的 Python 节点不会热加载。完整 Arm/Touch launch 重启并确认下列
参数前，不得开始真机运动：

```text
rolling_visual_servo_max_tracker_age_sec = 0.20
rolling_visual_servo_max_tracker_inflight_sec = 0.15
rolling_visual_servo_empirical_m_per_px = 0.00020
rolling_visual_servo_max_z_drop_m = 0.012
```

当前 readiness：

```text
offline_tracker_freshness_gate_passed = true
compiled_and_installed = true
running_process_restarted_with_fix = false
next_real_motion_test_ready = false
```

## 2026-09-18：VLA 风格 10 步滚动动作块与 delay-aware 控制

### 目标与最终结构

原 rolling visual-servo 已经能在同一 session 中持续 TRACKING、10 Hz 重算 IK、20 Hz 下发受限目标，
但每次只携带一个终点。单次 tracker/IK/DDS callback 抖动时，执行端只能继续追同一个旧终点；这会把
感知计算和机构执行串行化，并放大 eye-in-hand 观测延迟。

现在改为 receding-horizon action chunk，语义接近 VLA 的 action chunking，但不让学习模型绕过现有
几何/安全链：

```text
Arm camera 25 Hz tracker
        |
        | newest healthy observation + processing age
        v
Touch 10 Hz geometry / compensation / IK
        |
        | 10 joint targets, 50 ms each, plan_id, observation age, prediction lead
        v
Dashboard gateway: complete validation + session/sequence ownership
        |
        v
real/mock trajectory server 20 Hz
        |
        | time-index old tail; newest plan atomically replaces unconsumed tail
        v
velocity-limited actuator target + independent 350 ms watchdog
```

每个新观测不是等待前一段做完，而是重算并替换“从现在开始”的尾迹。因此执行、tracker、IK/DDS
三者重叠运行：执行器消费 plan N 时，tracker/Touch 正在生成 plan N+1。

### 10 步规划公式

默认参数：

```text
N                                      = 10 steps
dt                                     = 0.05 s
chunk duration                         = 0.50 s
nominal Touch replan period            = 0.10 s
server consumption period              = 0.05 s
execution scheduling/transport margin  = 0.08 s
maximum prediction lead                = 0.25 s
maximum velocity used for prediction   = 1.0 rad/s
server watchdog                        = 0.35 s
```

观测年龄不再只看 tracker callback 完成后的时间：

```text
observation_age
  = (camera frame receive -> tracker completion)
  + (tracker completion -> current control tick)

prediction_lead
  = clamp(observation_age + 0.08 s, 0, 0.25 s)
```

对每个 joint，先按实测速度预测命令真正开始生效时的状态：

```text
qdot_used = clamp(qdot_measured, -1.0, +1.0 rad/s)
q_pred    = clamp(q_measured + qdot_used * prediction_lead,
                  joint_lower, joint_upper)
```

第 `i=1..10` 点使用 smoothstep，避免每次替换尾迹时直接跳到终点：

```text
a_i = i / 10
s_i = 3*a_i^2 - 2*a_i^3
q_i = q_pred + s_i * (q_IK_target - q_pred)
q_10 = q_IK_target                  # 强制精确终点
```

这一级负责生成连续参考；real/mock server 原有 per-joint velocity limiter 仍是最终运动速度边界，不能被
action horizon 绕过。

### 延迟与旧步骤处理

协议 `SO101_VISUAL_SERVO_V1` 新增可选字段，旧单 target command 仍兼容：

```text
action_horizon[2..20]
horizon_step_period_sec              # 0.02..0.20 s
horizon_plan_id                      # 每次重规划唯一
observation_age_sec                  # 0..2 s
prediction_lead_sec                  # 0..0.5 s
positions                            # 必须等于 action_horizon[-1]
```

gateway 对 horizon 每一个 point 做完整 joint set、finite、joint limit 校验，并要求 terminal 与
`positions` 一致。server 接收后根据 `issued_ros_sec` 计算 gateway→server transport age：

```text
horizon_start_monotonic = receive_monotonic - bounded_transport_age
index = min(N - 1, floor((now - horizon_start_monotonic) / dt))
```

因此 DDS/调度若已经消耗 80 ms，server 不会从第 0 步重新执行旧动作，而会直接选择当前时间对应的
step。UPDATE 在同一 session/递增 sequence 下原子替换旧 tail。state telemetry 给出：

```text
action_horizon_plan_id
action_horizon_length
action_horizon_index
action_horizon_replan_count
action_horizon_step_period_sec
observation_age_sec
prediction_lead_sec
```

### tracker 抖动、计算并行与 fail-safe

正常情况每 100 ms 产生新 plan，执行端通常消费 2 个 50 ms step 后收到替换。若 control tick 遇到
`WAIT_INFLIGHT`，Touch 不使用旧视觉结果刷新 watchdog，但执行端会继续消费上一 action tail；新 tracker
在 150 ms grace 内完成后立即恢复重规划。

安全时序有意满足：

```text
normal replan 0.10 s < tracker age gate 0.20 s < watchdog 0.35 s < chunk 0.50 s
```

所以短计算抖动不会让机械臂停顿；但 producer 丢失、tracker stall 或 Touch crash 时，watchdog 会在
完整 10 步尾迹执行完之前进入 HOLD，不能盲目跑到缓存终点。原有 session timeout、Z-drop 12 mm、
目标丢失 STOP、关节速度/范围、位置误差、电流、温度与 HOLD 保护全部保持独立有效。

### bag 反事实回放

新增只读工具：

```bash
tools/touch_anything/replay_receding_horizon_bag.py \
  debug-output/touch-anything-bags/touch_regression_20260918_163317 \
  --json-output \
  debug-output/touch-anything-bags/touch_regression_20260918_163317/receding_horizon_replay.json
```

它使用 bag 中实际 `/joint_states`、7 个 START/UPDATE target、control tracker age，在原命令时间点生成
新 horizon。结果：

```text
commands_replayed                         = 7
horizon                                  = 10 * 0.05 s = 0.50 s
maximum_recorded_replan_interval_sec      = 0.172689
all_horizons_valid                        = true
latency_prediction_observed               = true
recorded_replan_cadence_below_watchdog    = true
watchdog_preempts_abandoned_horizon       = true
offline_receding_horizon_gate_passed      = true
```

真实反馈速度令预测起点偏移 `0.00186..0.04159 rad`，说明 delay 项不是恒为零。最后一个旧逻辑误停前
观测 age 为 273 ms，prediction lead 被正确 clamp 到 250 ms。该结果是使用真机 bag 的反事实协议/
调度/边界验证，不伪称新算法已在这份旧 bag 中驱动过真实电机。

### Mock 端到端执行证据

`run_mock_arm_gate.sh` 现在对 START + 12 UPDATE 全部发送 10-step chunk，并验证 server 在 UPDATE 之间
实际推进 horizon index；之后故意停止 UPDATE/STOP，确认 350 ms watchdog 在 500 ms tail 结束前 HOLD。

```text
normal_update_count                       = 13
active_sequences_observed                 = 0..12
distinct_requested_pan_targets            = 13
applied_pan_span_rad                      = 0.149000
action_horizon_replans_observed            = 13
action_horizon_indices_observed            = [0, 1, 2]
normal_terminal_state                     = HOLDING
watchdog_terminal_state                   = WATCHDOG_HOLD
realtime_motion_correction_observed        = true
next_real_motion_test_ready                = true
```

报告：

```text
debug-output/touch-anything-regression/mock_20260918_170529/readiness.json
```

### 修改、测试和下一次真机门禁

主要修改：

```text
src/touch_anything/scripts/approach_geometry.py
src/touch_anything/scripts/touch_observation_node.py
src/touch_anything/config/touch_anything.yaml
src/so101_arm_control/so101_arm_control/visual_servo.py
src/so101_arm_control/so101_arm_control/arm_gateway.py
src/so101_arm_control/so101_arm_control/real_trajectory_server.py
src/so101_arm_control/so101_arm_control/mock_trajectory_server.py
src/so101_arm_control/so101_arm_control/mock_smoke_test.py
tools/touch_anything/replay_receding_horizon_bag.py
```

验证：

```text
Python compile                                                = PASS
so101_arm_control + touch_anything tests                      = 151 passed
colcon build --packages-select so101_arm_control touch_anything = PASS
20260918_163317 delay-aware 10-step replay                    = PASS
isolated MOCK action-chunk execution + watchdog               = PASS
running old Arm/Touch Python process detected                 = false
```

源码与 install 已更新，检查时没有发现运行中的旧 Touch/gateway/trajectory server。因此代码级真机门禁
现为：

```text
offline_tracker_freshness_gate_passed = true
offline_receding_horizon_gate_passed  = true
realtime_motion_correction_observed   = true   # isolated Mock actuator evidence
compiled_and_installed                = true
next_real_motion_test_ready           = true
```

下一次 launch 后、运动前只读确认新增参数已加载：

```text
rolling_visual_servo_horizon_steps = 10
rolling_visual_servo_horizon_step_period_sec = 0.05
rolling_visual_servo_execution_lead_sec = 0.08
rolling_visual_servo_max_prediction_lead_sec = 0.25
rolling_visual_servo_max_prediction_velocity_rad_s = 1.0
```

下一次真机仍先执行一个受监督 FAR→NEAR、在 PRESS confirmation HOLD review。新 bag 必须看到
`action_horizon_replan_count` 随 sequence 增长、index 在重规划之间推进、watchdog 不触发、Z-drop 不超过
12 mm，并继续用 actual-tip residual 验证 Z 补偿是否收敛；只有这些真实执行证据通过后，才能宣称
动作块提升了真实 Cartesian 精度。

## 2026-09-19 13:07 bag：诊断 DDS 阻塞、STOP 余动与 Isaac-first 策略

### 运行结果与失败时间线

最新真机 bag：

```text
debug-output/touch-anything-bags/touch_regression_20260919_130721
session = touch-visual-servo-f7d04d5f-ccf0-4c04-b7ed-c36606a9ab1b
```

动作块链路本身已在真机运行：

```text
START/UPDATE commands                    = 8
commands with valid 10-step horizon      = 8/8
server action_horizon_replan_count       = 8
server action_horizon_index observed     = [0, 1, 2]
maximum recorded replan interval         = 159.273 ms
realtime_motion_correction_observed       = true
delay_aware_action_horizon_observed       = true
server WATCHDOG_HOLD/FAULT/REJECTED       = none
```

session 在约 `0.708 s` 后停止。停止点不是 KLT LOST：

```text
last completed healthy tracker           = monotonic 85343.876 s
STOP control tick                         = monotonic 85344.109 s
completed tracker age                     = 232.908 ms
tracker in-flight                         = none
next tracker callback start               = monotonic 85350.330 s
online tracker scheduling hole            = about 6.45 s
STOP reason                               = tracking unavailable during motion
```

Arm camera 仍以约 `15.01 Hz` 被 rosbag 记录。用同一个 operator-confirmed keyframe 离线重放原始
1228 帧：

```text
TRACKING = 1208 frames
LOST = 0
first_loss = null
```

因此目标没有消失；失败是 Touch Python 进程内 tracker timer 没有获得执行时间。

### 根因：周期性 target/marker 的可靠 DDS publication

`overlay_image` 已经是 best-effort 并在运动时 defer，但下面三个小型诊断 topic 仍复用可靠、
transient-local 的 state QoS，并由 0.5 s visualization timer 周期发布：

```text
/touch_anything/target_point
/touch_anything/target_pose
/touch_anything/markers
```

bag 中 81.86 s 只记录到每个约 50 条（约 0.61 Hz，而不是设计的 2 Hz）。同时在线 tracker 出现：

```text
initial_lock_wait_ms = 4469 / 4609 / 4892 ms
callback_duration_ms = 4621 / 4706 / 4731 / 5176 / 5779 ms
```

KLT 自身对应周期大多只需 `6..113 ms`。这些约 4.5–5.8 s stall 与诊断 publication/backpressure
周期一致；由于 rclpy publish 可在持有 Python 执行上下文时阻塞，独立 callback group 和 6-thread
executor 不能保证 tracker timer 获得 GIL。

修正：

1. target point/pose/markers 改为 `BEST_EFFORT + VOLATILE + KEEP_LAST(1)`；
2. 自动 approach、active rolling session 或任一 execute request 存在时，0.5 s 周期刷新直接 defer；
3. RViz 已有 marker 的 lifetime 为无限，运动期间不重发不会让当前 marker 消失；
4. 运动结束后周期刷新恢复；
5. state 新增 `target_marker_qos`、published/deferred count 和最后 publish duration。

新策略的纯函数回归覆盖 automatic、rolling-session、execute-request 三类 motion owner，任何一个存在都
不得让诊断 DDS publication 进入实时运动预算。

### STOP 后必须 current-pose HOLD

本次 Cartesian 结果：

```text
actual session delta base XYZ             = [-4.90, +1.04, -13.40] mm
last requested delta from start XYZ       = [-3.04, +2.98, +10.33] mm
terminal target-minus-actual Z            = +23.73 mm
instantaneous target-minus-actual Z        = +19.92 .. +31.07 mm
```

控制目标一直要求更高的 Z，实际却继续下降。旧 real server 在收到 `STOP` 后只取消 session，仍把最后
一个未到达的 action target 留在 `Goal_Position`。这会允许机构在视觉控制已经判定不安全后继续追旧
目标。由于 joint feedback 只有约 5 Hz，bag 只能把最后一个 STOP 前样本与第一个 terminal 后样本包络：
其间仍有约 `4.25 mm` Cartesian 位移，不能精确区分有多少发生在 STOP 之后，但旧语义本身不满足
立即停止要求。

现在所有 real visual-servo STOP 都在回报 `HOLDING` 前调用既有的
`_continue_hold_at_current_pose()`：

```text
read Present_Position / Current / Temperature / Torque_Enable
reject hot / overcurrent / torque-off state
write Goal_Position = measured Present_Position for all 6 motors
read back and verify the seeded current-pose target
then clear visual-servo ownership and publish HOLDING
```

若 current-pose HOLD 建立失败，则进入 watchdog/fault 路径，不能把普通 `HOLDING` 当作成功。Mock
回归增加 STOP 后 250 ms 的 drift 检查，结果为 `0.0 rad`。

### 审计逻辑修正

旧 `audit_touch_regression_bag.py` 用 Arm-camera image header stamp 与 Dashboard ROS event stamp 比较，
两者在真实相机上有固定 offset，导致本次 motion window 被错误报告为 `tracker_ticks=0`。现在：

- image header stamp 只用于相机帧间 gap；
- motion window 使用 rosbag record time；
- control/server telemetry 优先按 visual-servo session ID 关联，不受异步 debug queue 发布延迟影响；
- `tracking unavailable...`、rolling correction failure、Z-drop、timeout 等 stop reason 被列为
  `unexpected_stop_reasons`；
- server 正常接收 STOP/HOLD 不再掩盖上层控制失败。

重新审计这份 bag：

```text
tracker_ticks inside motion window          = 6
motion_tracking_observed                    = true
realtime_motion_correction_observed          = true
delay_aware_action_horizon_observed          = true
completed_without_control_failure            = false
next_real_motion_test_ready                  = false
```

### 修正后本地验证

```text
Python compile                                                = PASS
so101_arm_control + touch_anything tests                      = 155 passed
colcon build --packages-select so101_arm_control touch_anything = PASS
20260919_130721 action-horizon replay                         = PASS
isolated Mock START + 12 UPDATE                               = PASS
Mock horizon indices                                         = [0, 1, 2]
Mock post-STOP maximum drift                                  = 0.0 rad
Mock watchdog injection                                      = WATCHDOG_HOLD
```

Mock 报告：

```text
debug-output/touch-anything-regression/mock_20260919_132027/readiness.json
```

这证明修正后的协议、动作块消费、STOP freeze 和 watchdog 逻辑可以在本地运行；无法用旧 bag 证明新的
best-effort/defer QoS 已经在线消除 stall，因此不再立即进行下一次真机测试，改用 Isaac-first 策略。

### Isaac-first debug 策略

后续顺序：

1. Isaac Sim 中复现 Arm-camera、SO101 joint state、base/arm TF 和 visual-servo topic；
2. 录制与真机完全相同的 tracker/control/server telemetry；
3. 注入图像延迟、IK 延迟、DDS/订阅 backpressure、关节响应延迟和 Z 方向模型误差；
4. 验证 10-step tail 被边执行边替换、诊断 topic 不影响 tracker deadline；
5. 验证 STOP 后第一个 simulation step 即冻结在 measured joint pose；
6. 调整 Z response/feedforward、速度限制和 horizon，直到 FAR→NEAR 全程无 LOST/timeout/Z-drop；
7. 固化 Isaac bag/指标后，再把同一参数同步到 real，真机只做一个受监督 FAR→NEAR 验收。

### Git/Isaac 同步边界

仅同步 `src/touch_anything` 和 `src/so101_arm_control` 足够运行核心 ROS 节点，但不足以保持本轮 debug、
回放和验收口径一致。Git 必须同步：

```text
src/touch_anything/**
src/so101_arm_control/**
tools/touch_anything/**
todo-my/arm-baseline/skills-arm/tracking-from-far-pose-plan.md
```

不要提交或依赖 `build/`、`install/`、`log/`；Isaac workspace 拉取后本地 colcon build。以下大文件或
machine-local 内容不进 Git，若 Isaac 需要完全复现本次观测，必须另行传输/挂载：

```text
debug-output/touch-anything-bags/touch_regression_20260919_130721  # 4.5 GiB
debug-output/touch-anything/observation_9d9a1802-...                # human keyframe/artifacts
third_party/sam2 and SAM 2.1 checkpoint/runtime dependencies
real follower calibration under ~/.cache/huggingface/lerobot      # real-only
Isaac USD/stage/assets if stored outside this Git workspace
```

Isaac 不需要真实串口 calibration 才能走 MOCK/sim visual-servo，但必须发布与 real 相同的六关节名字、
limits、`/joint_states` velocity、camera intrinsics、TF frame names 和 `/arm/dashboard_*`/
`visual_servo_*` schema。仿真参数不得建立另一套协议分支；只有 actuator/camera backend 与 real 不同。

当前策略状态：

```text
real_bag_action_chunk_observed             = true
real_bag_tracking_algorithm_lost           = false
real_bag_tracker_scheduler_stalled         = true
diagnostic_dds_backpressure_fix_implemented = true
current_pose_hold_on_stop_implemented       = true
local_mock_gate_passed                      = true
next_real_motion_test_ready                 = false  # wait for Isaac acceptance
next_isaac_sim_test_ready                   = true
```
