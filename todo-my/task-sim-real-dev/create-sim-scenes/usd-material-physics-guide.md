# Real Lab USD material and physics guide

## Asset cache and local files

Isaac Sim / Omniverse caches downloaded HTTPS assets here on this machine:

```text
/home/ruan-x/Documents/Omniverse_Cache/client/https
```

This cache is useful for faster second loads, but the filenames are hash names,
not the original asset names. For long-term editable lab assets, prefer this
workflow:

1. In Isaac Sim, open the referenced online asset once.
2. Use `File > Save Flattened As...` or `Collect Asset` into a project folder,
   for example:

```text
/home/ruan-x/robot_ws/models/omniverse_assets/
```

3. Replace the USD reference URL with a relative local path.

Example:

```usda
prepend references = @../../../../models/omniverse_assets/furniture/Newman.usd@
```

Keep the online URLs until an asset is selected and stable. The cache path is
not ideal for hand-authored references because it can be cleared or renamed by
the cache system.

## Isaac Sim UI: collect or save assets locally

Recommended target folder:

```text
/home/ruan-x/robot_ws/models/omniverse_assets/
```

Workflow A, collect the whole current lab stage:

1. Open the lab USD in Isaac Sim.
2. In the `Stage` panel, make sure the stage is saved.
3. Use `File > Collect Asset...`.
4. Set the destination to:

```text
/home/ruan-x/robot_ws/models/omniverse_assets/real_lab_collect/
```

5. Enable options similar to `Collect Dependencies`, `Update References`, and
   `Preserve Directory Structure` if available.
6. Run collect.
7. Open the collected USD from the new folder and confirm textures/assets load
   with network disabled or after clearing the online cache.

Workflow B, save a single selected asset locally:

1. Select the asset root prim, for example a door, chair, table, or shelf.
2. Right click the prim in `Stage`.
3. Choose `Save As...`, `Save Flattened As...`, or `Export Selected...`,
   depending on what the UI exposes for that prim.
4. Save it under a semantic folder, for example:

```text
/home/ruan-x/robot_ws/models/omniverse_assets/tables/table.usd
/home/ruan-x/robot_ws/models/omniverse_assets/doors/standard_large_doors.usd
/home/ruan-x/robot_ws/models/omniverse_assets/chairs/newman.usd
```

5. Reopen the saved local asset once to verify it includes all textures and
   sublayers.
6. Replace the online reference in `real_lab_asset_refs.usd` with a relative
   local path.

Example from:

```usda
prepend references = @https://omniverse-content-production.s3-us-west-2.amazonaws.com/.../table.usd@
```

to:

```usda
prepend references = @../../../../models/omniverse_assets/tables/table.usd@
```

If the saved asset has missing textures, use `Collect Asset` instead of only
`Save As`, because `Save As` may save the USD layer but leave textures referenced
from remote paths.

## Why many objects do not fall

In USD/Isaac Sim, a visible mesh does not fall unless it has physics APIs such
as:

```usda
prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
```

Most lab props in this scene are intentionally visual/static references. That
means:

- They do not fall under gravity.
- They usually do not block the robot unless their own asset includes collision.
- They still appear in RGB cameras and can appear in LiDAR/depth depending on
  the sensor/rendering path.

For navigation and mapping stability, the room uses simplified invisible
collision proxies in:

```text
src/robot_sim/usd/scenes/real_lab/real_lab_furniture.usd
```

This is intentional. Stable static proxies are better for SLAM/navigation than
many small dynamic bodies.

## Can we use an asset's own rigid body?

Yes, but only when that asset is meant to be simulated as a physical object and
it starts in a clean non-overlapping pose.

Good cases:

- A box, bin, cart, chair, or tool that should be pushed or fall.
- The asset already has good colliders.
- It does not overlap another proxy collider.

Risky cases:

- Large furniture where we also authored an invisible proxy.
- Imported assets with unknown scale/origin.
- Assets whose collider starts below the floor or inside another object.

For tables in the lab, choose one of these two modes:

```text
Mode A, stable mapping/navigation:
visual table asset + invisible static proxy
```

```text
Mode B, physical table:
table asset rigid body/collision only, delete or disable the invisible proxy
```

Do not use both for the same table. Overlapping colliders are the common cause
of tables jumping or flying when simulation starts.

Current scene choice:

```text
Mode B, physical table
```

The table proxy cubes in `real_lab_furniture.usd` have
`physics:collisionEnabled = false`. The visual table assets should provide their
own collision. Because the current table asset jumps when simulated as a dynamic
rigid body, the table overrides in `robot_sim_real_lab.usd` set:

```usda
bool physics:kinematicEnabled = true
bool physics:rigidBodyEnabled = true
```

This keeps the table physical/collidable while preventing gravity or penetration
resolution from throwing it upward.

## Are invisible objects seen by cameras?

Usually no.

- RGB camera: invisible prims are not rendered.
- Depth camera: invisible prims are normally not rendered into depth either.
- RTX/LiDAR-style sensors: generally follow render visibility, but behavior can
  vary with sensor implementation and collision/query settings.
- Physics/collision: visibility does not disable collision.

That means an invisible collision proxy can still block the robot physically but
not appear in RGB/depth. This is useful for navigation stability, but it can be
confusing if the robot collides with something the camera cannot see. For
perception-heavy tests, prefer visible colliders or make sure visual mesh and
proxy positions match closely.

## Why a table can fly up when simulation starts

The common causes are:

- A referenced table asset already has its own rigid body/collider.
- It overlaps with our invisible table collision proxy.
- It starts slightly inside the floor or another collider.
- Its origin/scale is different from the proxy, so PhysX resolves a large
  penetration at simulation start.

Preferred fix for this lab:

1. Keep the visual table as visual-only.
2. Keep collision on the invisible cube proxy.
3. If a referenced asset still moves, select the referenced table in Isaac Sim
   and disable its rigid body/collision, or replace it with a static/visual
   version saved locally.

Alternative fix:

1. Use the table asset's own rigid body/collision.
2. Remove or disable the matching invisible proxy in
   `real_lab_furniture.usd`.
3. Ensure the table starts slightly above the floor, then let it settle.

If a table still jumps after disabling the proxy, inspect the table asset itself:

- Its collider may start below the floor.
- Its internal rigid body may have a bad mass or center of mass.
- Its scale/origin may not match the visual mesh.

## How to change simple material colors

For local cube geometry, edit `color3f[] primvars:displayColor` directly:

```usda
color3f[] primvars:displayColor = [(0.16, 0.08, 0.035)]
```

Values are RGB in 0-1 range.

Useful examples:

```text
deep brown: (0.16, 0.08, 0.035)
matte black: (0.02, 0.02, 0.018)
off white: (0.86, 0.84, 0.78)
warehouse gray floor: (0.66, 0.66, 0.62)
deep green door: (0.02, 0.18, 0.1)
```

For reusable materials, edit:

```text
src/robot_sim/usd/scenes/real_lab/real_lab_materials.usd
```

Example:

```usda
def Material "RealLab_DarkBrownTrim_Display"
{
    token outputs:surface.connect = </RealLab_DarkBrownTrim_Display/PreviewSurface.outputs:surface>

    def Shader "PreviewSurface"
    {
        uniform token info:id = "UsdPreviewSurface"
        color3f inputs:diffuseColor = (0.16, 0.08, 0.035)
        float inputs:roughness = 0.5
        token outputs:surface
    }
}
```

Then bind it to a prim:

```usda
rel material:binding = </RealLab_DarkBrownTrim_Display>
```

Referenced commercial assets often have their own internal materials. A parent
material binding may not override child materials. In that case, use Isaac Sim:

1. Select the child mesh/material under the asset.
2. In `Property > Materials`, duplicate or create a material.
3. Adjust `Diffuse Color`, `Roughness`, `Metallic`.
4. Save the stage or save the modified asset locally.
