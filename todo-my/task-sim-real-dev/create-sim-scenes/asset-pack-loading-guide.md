# Omniverse / Isaac Sim Asset Pack 使用メモ

## 结论

这个方向是对的：先下载官方 asset pack，在 Isaac Sim / Content Browser 里挑出适合 lab 的桌子、椅子、柜子、架子、灯具、箱子等，再把选中的 USD 作为 reference 加进 `robot_sim_real_lab.usd`。

但不要把复杂 asset 直接当作导航/碰撞真值。推荐结构是：

```text
visual asset reference  -> 好看、真实、可替换
simple collision proxy  -> 稳定、尺寸按实测、用于机器人碰撞和导航
```

## 推荐先下载的包

从 NVIDIA OpenUSD Downloadable Asset Packs 页面：

- Commercial Assets Pack
  - 内容：office/commercial furniture，包括 tables、chairs、shelving、reception desks、conference tables。
  - 大小：约 5.8GB。
  - URL: `https://d4i3qtqj3r0z5.cloudfront.net/Commercial_NVD%4010013.zip`

- SimReady Warehouse 01 Assets Pack
  - 内容：step stands、storage racks、shelving、dock boards、ramps、pallets。
  - 大小：约 13.9GB。
  - URL: `https://d4i3qtqj3r0z5.cloudfront.net/SimReady_Warehouse_01_NVD%4010010.zip`

- SimReady Furniture & Misc Assets Pack
  - 内容：chairs、tables、desks、cabinetry、shelving、kitchen items 等，最适合 lab/office。
  - 大小：约 9.4GB。
  - URL: `https://d4i3qtqj3r0z5.cloudfront.net/SimReady_Furniture_Misc_01_NVD%4010010.zip`

- Base Materials Pack
  - 内容：基础 MDL materials。Commercial pack 文档说明它依赖这个包来正确显示材质。
  - 大小：约 8.2GB。
  - URL: `https://d4i3qtqj3r0z5.cloudfront.net/Base_Materials_NVD%4010013.zip`

可选：

- Warehouse Assets Pack
  - 内容：763 个 warehouse component，bins、crates、pallets、conveyors、ramps、ladders 等。
  - 大小：约 18GB。
  - URL: `https://d4i3qtqj3r0z5.cloudfront.net/Warehouse_NVD%4010013.zip`

## 下载

脚本：

```bash
todo-my/task-sim-real-dev/create-sim-scenes/download-omniverse-asset-packs.sh
```

默认下载到：

```text
/home/ruan-x/robot_ws/omniverse_asset_packs/zips
```

支持断点续传。中断后重复运行即可继续。

## Isaac Sim 里加载 NVIDIA Assets

Isaac Sim 4.2 文档中的流程：

1. 打开 `Window -> Browsers -> Assets`。
2. 会出现 `NVIDIA Assets` 窗口。
3. 在左侧分类里浏览，例如：
   - `Industrial -> Buildings -> Warehouse`
   - `Racks`
   - `Shelves`
   - `Commercial`
   - `Residential`
4. 把资产拖到 Viewport 或 Stage。
5. 注意：该文档说明 `NVIDIA Assets` 里的资产是厘米尺度，导入后通常要 scale `0.01`。
6. 如果想让多个资产都自动按比例缩放，可以先建一个 parent prim，例如 `/World/AssetImport_0p01`，scale 设为 `(0.01, 0.01, 0.01)`，再把资产拖进去。

## 对 lab USD 的使用策略

挑选 asset 时优先记录：

```text
asset name
asset URL/path
approx real-world size
intended lab object
needs collision? yes/no
```

例如：

```text
name: warehouse storage rack
path: ...
lab object: left wall shelf
scale: [?, ?, ?]
collision: keep existing proxy
```

我后续会把选中的资产放到：

```text
src/robot_sim/usd/scenes/real_lab/real_lab_asset_refs.usd
```

并保留：

```text
src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd
```

里的 invisible collision proxy。

## 注意

NVIDIA Isaac warehouse 教程明确说明这些 `NVIDIA Assets` 主要是 visual assets。如果需要物理仿真，需要另外 author physics。对我们的机器人场景来说，这正好说明了为什么要保留简化碰撞体。
