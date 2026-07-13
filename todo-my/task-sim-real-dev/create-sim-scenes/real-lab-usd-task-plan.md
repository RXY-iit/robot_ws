# 真实研究室 Isaac Sim USD 场景任务计划

## 1. 任务理解

本任务要把真实研究室做成 Isaac Sim 可运行的 USD 场景。场景服务于机器人实验，不是单纯视觉展示。

第一版成功标准：

- 机器人能被放入场景并稳定站在地板上。
- 墙、门、主要家具和通道的尺寸关系大体正确。
- LiDAR 点云能反映真实房间的主要几何结构。
- RGB-D 相机能看到接近真实布局的桌子、柜子、墙、地板。
- 场景能沿用现有 `robot_sim` 的 USD 和 ROS2 sensor setup 流程。

当前建议的主场景文件：

```text
src/robot_sim/usd/scenes/robot_sim_real_lab.usd
```

该文件应像现有 factory mapping 场景一样 sublayer 机器人主 USD：

```text
src/robot_sim/usd/robot_sim_physics.usd
```

## 2. 阶段划分

### Phase 0: 需求和输入整理

状态：当前执行中。

产物：

- `requiremnt.md`
- `real-lab-usd-task-plan.md`
- `real-lab-measurement-template.yaml`

目标：

- 明确优先级。
- 明确哪些数据需要人工测量。
- 明确 USD 分层和后续生成方式。

### Phase 1: 最小可用静态场景

输入：

- 房间长宽高
- 门/窗/柱子/固定墙体位置
- 主要家具 bbox 和位置
- 原点和坐标轴方向

产物：

```text
src/robot_sim/usd/scenes/robot_sim_real_lab.usd
src/robot_sim/usd/scenes/real_lab/real_lab_structure.usd
src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd
src/robot_sim/usd/scenes/real_lab/real_lab_lighting.usd
src/robot_sim/usd/scenes/real_lab/real_lab_physics.usd
```

内容：

- 地板、墙、天花板、门洞、柱子
- 中央桌群、边柜、书架、大白板、沙发、机器人停放区等主要障碍物
- 简化碰撞体
- 地板摩擦参数
- 基础灯光

验收：

- Isaac Sim 能打开。
- Press Play 后机器人不穿模、不悬空。
- 手动遥控时机器人不异常打滑。
- LiDAR 点云中墙面和主要障碍物位置合理。

### Phase 2: 资产替换和外观接近

输入：

- 物体照片和近照
- 需要优先还原的家具列表

内容：

- 优先搜索并复用 Isaac Sim / Omniverse 官方 sample asset。
- 没有合适 asset 的物体继续用简化几何体。
- 补地板、墙面、桌面、柜体材质。
- 调整灯管和室内亮度。

原则：

- 官方 asset 的物理属性更完整时优先使用。
- 视觉 asset 如果导致碰撞复杂或性能下降，要保留简化 collider。
- 对导航和 LiDAR 重要的尺寸优先级高于外观相似度。

### Phase 3: 语义和感知增强

内容：

- 给 wall、floor、door、desk、cabinet、chair、shelf、box 等加 semantic label。
- 根据 `perception_targets.yaml` 优先支持 box、bottle、bucket、person、pallet 等目标。
- 放置少量可识别目标用于 RGB-D / detector 测试。

### Phase 4: 动态状态扩展

第一版不做复杂动态。后续按需要添加：

- 门 open / closed VariantSet
- 椅子和箱子的预设位置 VariantSet
- 可移动箱子 rigid body
- 可转动门 hinge joint

动态优先级低于静态导航、LiDAR mapping 和碰撞可靠性。

## 3. 推荐 USD 分层结构

```text
robot_sim_real_lab.usd
├── ../robot_sim_physics.usd
└── real_lab/real_lab_root.usd
    ├── real_lab_structure.usd
    ├── real_lab_furniture.usd
    ├── real_lab_lighting.usd
    ├── real_lab_materials.usd
    ├── real_lab_physics.usd
    └── real_lab_semantics.usd
```

说明：

- `robot_sim_real_lab.usd`：场景入口，负责组合机器人和真实研究室环境。
- `real_lab_root.usd`：真实研究室场景根层。
- `real_lab_structure.usd`：房间结构，墙、地板、天花板、门洞、柱子。
- `real_lab_furniture.usd`：桌子、柜子、书架、椅子、箱子、机器人停放架。
- `real_lab_lighting.usd`：灯管、环境光、必要的 area light。
- `real_lab_materials.usd`：地板、墙、柜体、桌面等材质。
- `real_lab_physics.usd`：碰撞、摩擦、刚体、物理覆盖。
- `real_lab_semantics.usd`：语义标签覆盖。

第一版可以把 materials、physics、semantics 合并到少数文件里。等内容变多后再拆细。

## 4. 数据责任划分

我可以做：

- 把测量模板转换为 USD 场景。
- 生成场景分层文件。
- 添加基础几何、碰撞、摩擦、灯光。
- 将场景接入现有 `robot_sim_physics.usd`。
- 写 Isaac Sim 打开、Play、ROS2 sensor 检查步骤。
- 后续根据截图或点云反馈调整尺寸和位置。

需要人工提供：

- 房间真实尺寸。
- 主要物体的位置和尺寸。
- 原点和坐标轴约定。
- 哪些家具必须存在，哪些可以省略。
- 哪些物体可以在现实中移动，哪些是固定障碍。

## 5. 资产使用策略

优先级：

1. Isaac Sim / Omniverse 官方 sample asset，尤其是桌子、柜子、椅子、箱子、灯具、地板材质。
2. Isaac Sim 基础几何体加简化材质。
3. 自制简单 USD prim。
4. 外部模型或照片建模，放到后续阶段。

使用约束：

- asset 的外观可以近似，但占地尺寸和碰撞边界要匹配真实通行空间。
- 对机器人会撞到的物体，必须有稳定的简化 collider。
- 复杂视觉 mesh 不直接作为 collision mesh。
- 对 LiDAR 重要的墙、柜、桌边，不要只做贴图，必须有几何体。

## 6. 第一版测试清单

- [ ] Isaac Sim 打开 `robot_sim_real_lab.usd` 无错误。
- [ ] Play 后机器人稳定落地。
- [ ] 手动遥控前进、横移、旋转无明显异常。
- [ ] 机器人不会穿过墙、桌子、柜子。
- [ ] 主要通道宽度与现实一致。
- [ ] `/point_cloud` 或 `/livox/lidar` 中能看到墙面、桌群、柜子轮廓。
- [ ] RGB 相机能看到桌子、柜子、墙、地板的大体布局。
- [ ] 场景加载和仿真帧率可接受。
