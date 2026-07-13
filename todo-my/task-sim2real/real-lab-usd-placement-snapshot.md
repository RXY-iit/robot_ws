# Real Lab USD Placement Snapshot

Last checked: 2026-07-07 21:41 JST

Purpose: record the current authored placement values after Isaac Sim UI Save, so future reopen/move issues can be compared against a known file-backed state.

## File Save Check

- `src/robot_sim/usd/scenes/robot_sim_real_lab.usd`
  - mtime observed: `2026-07-07 21:41:15 +0900`
  - This is the active strongest scene layer for current visual asset placement overrides.
- `src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd`
  - mtime observed: `2026-07-07 17:34:46 +0900`
  - Static collision proxies are defined here. Recent UI Save did not update this file.
- `src/robot_sim/usd/scenes/real_lab/real_lab_asset_refs.usd`
  - mtime observed: `2026-07-06 12:09:02 +0900`
  - Base reference definitions; current UI placement should generally not be authored here.

## Current Saved Visual Placement Overrides

These values are authored in `src/robot_sim/usd/scenes/robot_sim_real_lab.usd`.

```usd
over "toolbox_cart_temp"
{
    quatd xformOp:orient = (1, 0, 0, 0)
    double3 xformOp:scale = (0.01, 0.01, 0.01)
    double3 xformOp:translate = (0.7547242304292607, 6.755776590535806, 0)
}

over "folding_facility_cart_temp"
{
    quatd xformOp:orient = (1, 0, 0, 0)
    double3 xformOp:scale = (0.01, 0.01, 0.01)
    double3 xformOp:translate = (-2.2214326892759244, -1.6182729904703659, 0)
}

over "case_a01_on_wall_cabinet_top"
{
    double3 xformOp:translate = (-3.3098142601906178, -0.11519629513747554, 1.9288763191326803)
}

def Xform "case_a01_on_wall_cabinet_top_01"
{
    double3 xformOp:translate = (-3.3098142601906178, -1.2482422622628246, 1.9247362632312777)
}

over "container_c10_on_wall_cabinet_top"
{
    double3 xformOp:translate = (2.8893760394151955, -6.73344203176952, 1.9257492634787026)
}

def Xform "container_c10_on_wall_cabinet_top_01"
{
    double3 xformOp:translate = (2.3690998611400946, -6.7334420317695205, 1.9257492634787026)
}

over "bookstack_01_left_shelf"
{
    double3 xformOp:translate = (-3.2870241410875938, 1.6467743075726797, 1.12)
}

def Xform "bookstack_01_left_shelf_01"
{
    double3 xformOp:translate = (2.7009012676565147, 6.786315882423974, 1.1228587674496364)
}
```

Effective full paths:

- `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/CabinetTopStorage/case_a01_on_wall_cabinet_top`
- `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/CabinetTopStorage/case_a01_on_wall_cabinet_top_01`
- `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/CabinetTopStorage/container_c10_on_wall_cabinet_top`
- `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/CabinetTopStorage/container_c10_on_wall_cabinet_top_01`
- `/World/RealLabEnvironment/FurnitureAssets/ShelfAssets/bookstack_01_left_shelf`
- `/World/RealLabEnvironment/FurnitureAssets/ShelfAssets/bookstack_01_left_shelf_01`

## Current Static Collision Proxy Positions

These values are authored in `src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd`.

```usd
def Cube "toolbox_cart_proxy"
{
    double3 xformOp:scale = (0.90, 0.55, 0.95)
    double3 xformOp:translate = (0.7441661818199148, 6.833741662439691, 0.475)
}

def Cube "folding_facility_cart_proxy"
{
    double3 xformOp:scale = (0.95, 0.62, 1.15)
    double3 xformOp:translate = (-2.233656604293729, -1.643585872161329, 0.575)
}
```

## Notes

- Visual placement and static collision proxy placement are in different files.
- The duplicate root-level prims that Isaac UI authored outside `/World` were migrated into the effective paths above and removed from the stage root.
- If an asset is moved in Isaac Sim, verify the corresponding proxy is still aligned.
- Move the parent prim, not a referenced child mesh:
  - Good: `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/toolbox_cart_temp`
  - Risky: `/World/RealLabEnvironment/FurnitureAssets/LooseLabAssets/toolbox_cart_temp/sm_toolboxcart_a03_body_01`
- If a Save only dirties Session Layer, reopening will lose the move.
