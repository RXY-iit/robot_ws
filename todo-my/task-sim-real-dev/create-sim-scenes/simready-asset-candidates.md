# SimReady Asset 候选记录

目标：后续把第一版 Cube 桌子/椅子替换成 Isaac Sim / Omniverse SimReady Assets 中可用的模型，同时保留简化碰撞体。

## 当前状态

本地命令行搜索没有找到完整的 SimReady 桌子/椅子资产包，只找到 Isaac Sim 扩展测试资源：

- `/home/ruan-x/isaac-sim/extscache/omni.syntheticdata-0.6.13+8131b85d.lx64.r.cp311/omni/syntheticdata/tests/data/scenes/BenchChair_Mini.usda`
- `/home/ruan-x/isaac-sim/extscache/omni.syntheticdata-0.6.13+8131b85d.lx64.r.cp311/omni/syntheticdata/tests/data/scenes/BenchChair_SceneInstance_Mini.usda`
- `/home/ruan-x/isaac-sim/extscache/omni.kit.viewport.actions-107.0.2+8131b85d/data/tests/usd/referenced/VibrationTable_Top.usda`

这些更像测试 fixture，不建议作为正式场景资产。

你在 Asset Browser 截图中确认过的可用候选：

- `Environments/Simple_Room/simple_room.usd`
- `Environments/Office/office.usd`
- `IsaacLab/Mimic/nut_pour_task/nut_pour_assets/table.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Seating/Jobba/Jobba_Loveseat.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Tables/Kettle.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Seating/Newman.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Seating/Reidmeyer.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/Bookshelves/Rockhill.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Carts/ToolboxCart_A/ToolBoxCart_A03_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Hand_Trucks/Gas_Cylinder_A/GasCylinderHandTruck_A02_PR_V_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/DiningSets/DesPeres/DesPeres_Table.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Commercial/Storage/Standard/Standard_LargeDoors.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/FurnitureSets/Maplewood/Maplewood_Loveseat.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Electronics/Televisions/TV_Wall.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Books/BookStack_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Books/BookStack_02.usd`
- `https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/whitehome01/whitehome01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/Bookshelves/Fenton_Var01.usd`
- `https://omniverse-content-staging.s3.us-west-2.amazonaws.com/Assets/simready_content/common_assets/props/box_a05/box_a05.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/usa/country_assets/traffic_lights_tmp/housings_pieces/surroundings/surround_1_square.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Equipment/Carts/FoldingFacilityCart_A/FoldingFacilityCart_A01_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Appliances/Microwaves/microwave.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Appliances/Refrigerators/fridge.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Entertainment/Games/ChessSet/ChessSet_FullSet.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Safety/Floor_Signs/MobileShutOffSystem_A/MobileShutOffSystem_A02_PR_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Safety/Floor_Signs/WetFloor_A/WetFloorSign_A01_PR_NVD_01.usd`

当前模型已经引用 `table.usd` 作为桌子的视觉 asset，原始 cube 保留为不可见 collision proxy。`simple_room.usd` 和 `office.usd` 是完整房间/办公室场景，不适合直接叠到实测房间里；它们更适合作为材质、灯具、吊顶、办公室资产风格参考，或者后续从中挑单个 asset。

2026-07-03 更新：已把上面的 sofa、kettle、chair、Rockhill shelf、toolbox cart、gas cylinder hand truck、DesPeres table 加入 `src/robot_sim/usd/scenes/real_lab/real_lab_asset_refs.usd`。ArchVis/DigitalTwin 资产先按 `0.01` scale 放置；如果打开后大小不对，再根据实际 bounding box 调整。

2026-07-03 更新 2：门、Maplewood sofa、TV、book stacks、whitehome01、Fenton bookshelf、box_a05 已加入 asset refs。`whitehome01` 和 `box_a05` 来自 SimReady staging content，暂时按 scale 1.0 放置；如果大小不对，打开后再调。

2026-07-03 更新 3：`surround_1_square` 已作为右侧窗户边框候选加入；新增 folding facility cart、microwave、fridge、chess set、mobile shutoff floor sign、wet floor sign 到 free space。窗户原来的浅蓝占位面已隐藏，避免挡住窗洞视线。

2026-07-05 更新：新增 folding facility cart 架上 bin、靠墙柜顶箱子、桌面/书柜小物件，并加入本地几何窗框、warehouse 风格地面色块、天花板竖向发光灯柱。

- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Bins/Bin_B/Bin_B06_45x71x32cm_PR_V_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Bins/Bin_B/Bin_B03_14x23x12cm_PR_V_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Cases/Case_A/Case_A01_55x36x22cm_PR_V_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/DigitalTwin/Assets/Warehouse/Storage/Containers/Container_C/Container_C10_40x30x19cm_PR_V_NVD_01.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Lighting/Table%20Lamps/AshleyLamp.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Clocks/AlarmClock_Retro.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Sculptures/CopperScales.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Entertainment/GameRoom/DartBoard.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Decor/Magazines/MagazineStack02.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Misc/Supplies/MechanicalPencil.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Kitchen/Kitchenware/Dinnerware/P_Glassware_Short.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Outdoors/Decor/PinkFlamingos.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Plants/Plant_Succulent_02.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Electronics/GPU/RTX2080Ti.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Entertainment/Games/RubixCube.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Misc/Supplies/Sketchbook.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Electronics/Nvidia_Shield/Shield_Controller.usd`
- `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Electronics/Nvidia_Shield/ShieldTV.usd`

## 推荐下一步

在 Isaac Sim 中打开 Asset Browser / SimReady Explorer，优先搜索：

- `table`
- `desk`
- `office chair`
- `chair`
- `cabinet`
- `shelf`
- `fluorescent`
- `light`

选择标准：

- 能离线加载，或者能缓存到本地。
- 外观接近即可，尺寸可以 scale。
- 不依赖复杂动画或外部服务。
- 视觉 mesh 可以复杂，但导航碰撞仍使用简化 box/collider。

## 替换策略

第一版不要删除当前 Cube 桌子。建议新增 asset Xform，例如：

```text
/World/RealLabEnvironment/FurnitureAssets/table_group_upper_left_a_asset
```

然后保留当前：

```text
/World/RealLabEnvironment/Furniture/table_group_upper_left_a
```

作为 collision proxy，必要时将其 `visibility = "invisible"`。

这样 LiDAR/导航仍稳定，视觉可以逐步变真实。
