"""Disable the optional D455 sensor asset for LiDAR-only mapping.

The imported RSD455 asset contains a rigid body below the robot base rigid body.
Isaac PhysX reports this as an invalid nested rigid-body hierarchy and can later
hit tensor shape errors. GLIM mapping does not need the D455 camera, so deactivate
the sensor asset before Play. The robot's camera frame Xforms remain in the USD.

Use SetActive(False), not only RemovePrim(): the D455 is usually introduced by a
referenced USD asset, and removing a prim from the current layer may not mask the
referenced prim strongly enough for PhysX.

Run from Isaac Sim Script Editor before pressing Play:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_d455_for_mapping.py").read())
"""

import omni
from pxr import Usd


D455_ROOT_HINT = "/d455_sensor"
D455_RUNTIME_HINTS = (
    "d455_sensor",
    "RSD455",
    "ROS2PublishImage",
    "ROS2PublishCameraInfo",
)


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
        return ""
    try:
        return attr.Get() or ""
    except Exception:
        return ""


def main():
    stage = omni.usd.get_context().get_stage()
    disable_roots = set()

    for prim in _iter_all_prims(stage):
        path = prim.GetPath().pathString
        lower = path.lower()

        if D455_ROOT_HINT in lower:
            root = path.split(D455_ROOT_HINT, 1)[0] + D455_ROOT_HINT
            disable_roots.add(root)
            continue

        node_type = _node_type(prim)
        if any(hint in path or hint in node_type for hint in D455_RUNTIME_HINTS):
            disable_roots.add(path)

    disabled = 0
    for path in sorted(disable_roots, key=len, reverse=True):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            prim.SetActive(False)
            print(f"Disabled D455 mapping-optional prim: {path}")
            disabled += 1

    active_remaining = []
    for prim in _iter_all_prims(stage):
        path = prim.GetPath().pathString
        if any(hint.lower() in path.lower() for hint in D455_RUNTIME_HINTS):
            if prim.IsActive():
                active_remaining.append(path)

    if disabled:
        print(f"Disabled {disabled} D455/runtime prim(s).")
    else:
        print("No D455 mapping-optional prims found.")

    if active_remaining:
        print("WARNING: active D455/RSD455 prim(s) remain:")
        for path in active_remaining[:40]:
            print(f"  {path}")
        print("Do not press Play until these are inactive.")
    else:
        print("OK: D455 is disabled for LiDAR-only mapping.")


main()
