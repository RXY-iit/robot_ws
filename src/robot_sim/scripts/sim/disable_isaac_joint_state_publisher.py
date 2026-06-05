"""Disable Isaac's ROS2PublishJointState node for mapping.

The node calls PhysX tensor getVelocities() every tick. In full_warehouse_r.usd
it can report shape (2, 6) even after foreign articulations are removed. GLIM
mapping does not require Isaac-published /joint_states, so disable this node and
keep the articulation controller active for motion commands.

Run from Isaac Sim Script Editor:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/disable_isaac_joint_state_publisher.py").read())
"""

import omni
from pxr import Usd


JOINT_STATE_TYPE = "isaacsim.ros2.bridge.ROS2PublishJointState"
ARTICULATION_CONTROLLER_TYPE = "isaacsim.core.nodes.IsaacArticulationController"


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


def _node_type(prim):
    attr = prim.GetAttribute("node:type")
    if not attr:
        return None
    try:
        return attr.Get()
    except Exception:
        return None


def main():
    stage = omni.usd.get_context().get_stage()
    disabled = []
    controllers = []

    for prim in _iter_all_prims(stage):
        node_type = _node_type(prim)
        if node_type == JOINT_STATE_TYPE:
            prim.SetActive(False)
            disabled.append(prim.GetPath().pathString)
        elif node_type == ARTICULATION_CONTROLLER_TYPE:
            controllers.append((prim.GetPath().pathString, prim.IsActive()))

    if not disabled:
        print(f"No active/inactive {JOINT_STATE_TYPE} node found.")
    else:
        print("Disabled Isaac joint state publisher node(s):")
        for path in disabled:
            print(f"  {path}")

    if controllers:
        print("Articulation controller node(s):")
        for path, active in controllers:
            state = "ACTIVE" if active else "inactive"
            print(f"  {state}: {path}")
    else:
        print("No IsaacArticulationController node found.")
    print("If getVelocities (2,6) continues, the next suspect is articulation_controller.")


main()
