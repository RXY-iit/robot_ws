"""Temporarily disable IsaacArticulationController for error isolation.

This is a diagnostic tool only. The robot will not accept /sim/target_joint_states
while the controller is disabled. Use it only to check whether the persistent
PhysX tensor `getVelocities (2,6)` error comes from the controller node.

Run from Isaac Sim Script Editor before Play:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_articulation_controller_debug.py").read())
"""

import omni
from pxr import Usd


CONTROLLER_TYPE = "isaacsim.core.nodes.IsaacArticulationController"


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
        attr = prim.GetAttribute("node:type")
        node_type = attr.Get() if attr else None
        if node_type == CONTROLLER_TYPE:
            prim.SetActive(False)
            disabled.append(prim.GetPath().pathString)

    if disabled:
        print("Disabled Isaac articulation controller node(s) for debug:")
        for path in disabled:
            print(f"  {path}")
        print("Robot motion will not work until Isaac is restarted or the node is re-enabled.")
    else:
        print("No IsaacArticulationController node found.")


main()
