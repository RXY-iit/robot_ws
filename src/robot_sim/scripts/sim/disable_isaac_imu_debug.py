"""Temporarily disable Isaac true IMU graph for error isolation.

Use this only to check whether PhysX tensor `getVelocities (2,6)` comes from
IsaacReadIMU. /livox/imu will stop publishing while this is active.

Run from Isaac Sim Script Editor:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_imu_debug.py").read())
"""

import omni
from pxr import Usd


IMU_NODE_TYPES = {
    "isaacsim.sensors.physics.IsaacReadIMU",
    "isaacsim.ros2.bridge.ROS2PublishImu",
}
IMU_GRAPH_HINT = "livox_imu_ros_graph"


def _iter_all_prims(stage):
    if hasattr(stage, "TraverseAll"):
        yield from stage.TraverseAll()
        return

    try:
        yield from Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate)
        return
    except Exception:
        pass

    stack = list(stage.GetPseudoRoot().GetAllChildren())
    while stack:
        prim = stack.pop(0)
        yield prim
        try:
            stack[0:0] = list(prim.GetAllChildren())
        except Exception:
            stack[0:0] = list(prim.GetChildren())


def main():
    stage = omni.usd.get_context().get_stage()
    disabled = []
    for prim in _iter_all_prims(stage):
        path = prim.GetPath().pathString
        attr = prim.GetAttribute("node:type")
        node_type = attr.Get() if attr else None
        if node_type in IMU_NODE_TYPES or IMU_GRAPH_HINT in path:
            prim.SetActive(False)
            disabled.append(path)

    if disabled:
        print("Disabled Isaac IMU graph/node(s) for debug:")
        for path in disabled:
            print(f"  {path}")
        print("/livox/imu from Isaac true IMU will stop until Isaac is restarted or graph is re-enabled.")
    else:
        print("No Isaac IMU graph/node found.")


main()
