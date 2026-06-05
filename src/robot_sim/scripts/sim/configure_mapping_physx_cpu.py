"""Force mapping scene PhysX simulation onto CPU.

Run from Isaac Sim Script Editor before pressing Play, or after loading the
mapping USD if the stage was opened before robot_sim_physics.usd was updated:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/configure_mapping_physx_cpu.py").read())

RTX LiDAR still uses GPU rendering. This only disables PhysX GPU dynamics to
avoid CUDA error 700 during mapping bringup.
"""

import omni
from pxr import Sdf, UsdPhysics


PHYSICS_SCENE_CANDIDATES = (
    "/physicsScene",
    "/World/physicsScene",
)


def _set_attr(prim, name, type_name, value):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name)
    attr.Set(value)
    print(f"Set {prim.GetPath()}.{name} = {value}")


def main():
    stage = omni.usd.get_context().get_stage()
    prim = None
    for path in PHYSICS_SCENE_CANDIDATES:
        candidate = stage.GetPrimAtPath(path)
        if candidate and candidate.IsValid():
            prim = candidate
            break

    if not prim or not prim.IsValid():
        for candidate in stage.Traverse():
            if candidate.IsA(UsdPhysics.Scene):
                prim = candidate
                break

    if not prim or not prim.IsValid():
        raise RuntimeError(
            "Missing physics scene. Tried: "
            + ", ".join(PHYSICS_SCENE_CANDIDATES)
        )

    _set_attr(prim, "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, False)
    _set_attr(prim, "physxScene:broadphaseType", Sdf.ValueTypeNames.Token, "MBP")
    print("PhysX CPU mode configured for mapping scene.")


main()
