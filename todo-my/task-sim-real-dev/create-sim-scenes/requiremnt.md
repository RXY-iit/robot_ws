# Isaac Sim 真实研究室 USD 场景任务入口

目标：在 Isaac Sim 中创建一个与真实研究室大体一致、可用于机器人导航和传感器仿真的 USD 场景。

当前优先级：

1. 导航 / 避障
2. LiDAR localization / mapping
3. RGB-D 视觉识别
4. 检测 / 分割训练
5. 演示用视觉还原

第一阶段不追求照片级数字孪生。优先保证房间尺寸、主要障碍物、通道、碰撞、摩擦和传感器几何关系正确。

## 已有资料

学习笔记：

- `todo-my/task-sim-real-dev/create-sim-scenes/USD-notes.html`

基础构想：

- `todo-my/task-sim-real-dev/create-sim-scenes/USD-scene-planning.md`

正式任务计划：

- `todo-my/task-sim-real-dev/create-sim-scenes/real-lab-usd-task-plan.md`

现场测量模板：

- `todo-my/task-sim-real-dev/create-sim-scenes/real-lab-measurement-template.yaml`

真实房间照片：

- `todo-my/task-sim-real-dev/create-sim-scenes/IMG_9271.jpg`
- `todo-my/task-sim-real-dev/create-sim-scenes/IMG_9272.jpg`
- `todo-my/task-sim-real-dev/create-sim-scenes/IMG_9273.jpg`
- `todo-my/task-sim-real-dev/create-sim-scenes/IMG_9274.jpg`

## 当前待补数据

后续由人工现场测量补完：

- 房间长、宽、高
- 门、窗、柱子、固定柜子的位置和尺寸
- 中央桌群、边柜、书架、椅子、机器人停放区等主要障碍物 bbox
- 通道净宽，尤其是机器人常走路线
- 原点和坐标轴方向

## 建模原则

- 尽可能优先使用 Isaac Sim / Omniverse 官方 sample asset 或基础几何体。
- 如果官方 asset 有更完整的物理参数，优先复用官方 asset。
- 官方 asset 尺寸和真实物体不完全一致时，以大体尺寸、碰撞边界和通行空间正确为优先。
- 小物体、电线、显示器、天花板管线等细节放到后续阶段，不阻塞第一版。

## 先不复杂化的问题

- 动态变化第一版默认不做，门、椅子、箱子都先作为静态物体。
- 门开关、电梯、可移动椅子、可推动箱子后续用 VariantSet 或物理关节扩展。
- 光照第一版只做基础照明，等几何和传感器跑通后再调真实灯管效果。
