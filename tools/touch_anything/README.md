# Touch Anything operator notes

## Dependency setup

The current installation is pinned to Meta SAM 2 commit
`2b90b9f5ceec907a1c18123530e92e794ad901a4b5` and the official 2024-09-28
SAM 2.1 Hiera Tiny checkpoint. Recreate ignored local assets with:

```bash
cd /home/matsunaga-h/robot_ws
tools/touch_anything/setup_sam21.sh
```

The model is not loaded during bringup. The first RViz point prompt loads it on
CPU; it is retained for 180 seconds after the last inference and then released.

## Phase-1 workflow

1. Start the normal real-robot bringup and Dashboard.
2. In Arm, select `code2real&sim`.
3. Select `Touch Anything / 任意対象にタッチ` in Task triggers.
4. Confirm Dashboard reports `base=LOCKED` and the safety source reports
   `TOUCH_LOCK`.
5. In RViz `Touch Anything`, freeze the D435 frame.
6. Left-click the target. Right-click or select `Negative` to exclude a region.
7. Inspect the green mask, green 3-D point and orange surface-normal arrow.
8. Confirm the preview or exit Touch Mode.

Evidence for both accepted and rejected observations is written to
`debug-output/touch-anything/observation_<uuid>/`.

Phase 1 never publishes an Arm trajectory. Base adjustment is also disabled
while Touch Mode is active; move within the operator-approved ±0.5 m area before
entering Touch Mode, then reacquire/freeze the image.

Navigation is not shut down automatically. SAM runs once on a frozen frame, so
automatic Nav2 lifecycle mutation would add recovery risk without measured need.
If profiling later proves contention, add a dedicated observation-only launcher
that keeps localization and the cmd_vel safety layer but omits Nav2/local-costmap.
