"""List mapping-related OmniGraph nodes in the current Isaac stage."""

import omni
from pxr import Usd


INTERESTING_TYPES = {
    "isaacsim.ros2.bridge.ROS2PublishJointState",
    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
    "isaacsim.core.nodes.IsaacArticulationController",
    "isaacsim.ros2.bridge.ROS2PublishClock",
    "isaacsim.sensors.physics.IsaacReadIMU",
    "isaacsim.ros2.bridge.ROS2PublishImu",
}


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
    found = []
    for prim in _iter_all_prims(stage):
        attr = prim.GetAttribute("node:type")
        node_type = attr.Get() if attr else None
        if node_type in INTERESTING_TYPES:
            found.append((node_type, prim.GetPath().pathString, prim.IsActive()))

    print(f"Found {len(found)} mapping-related OmniGraph node(s):")
    for node_type, path, active in found:
        state = "ACTIVE" if active else "inactive"
        print(f"{state} {node_type}")
        print(f"  {path}")


main()
