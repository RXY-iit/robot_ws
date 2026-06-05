"""Create one front box for LiDAR z-range checks.

Run in Isaac Sim Script Editor after opening robot_sim_physics.usd:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_lidar_check_box.py").read())

The box is 2 m tall and placed in front of the robot. Use it to check whether
`/point_cloud` and `/livox/lidar_pc2_accumulated` contain a meaningful z range.
"""

import omni
from pxr import Gf, UsdGeom


BOX_PATH = "/World/lidar_check_box"


def main():
    stage = omni.usd.get_context().get_stage()
    existing = stage.GetPrimAtPath(BOX_PATH)
    if existing and existing.IsValid():
        stage.RemovePrim(BOX_PATH)

    cube = UsdGeom.Cube.Define(stage, BOX_PATH)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(2.0, 0.0, 1.0))
    cube.AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 2.0))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.6, 1.0)])
    print(f"Created 2 m LiDAR check box at {BOX_PATH}")


main()
