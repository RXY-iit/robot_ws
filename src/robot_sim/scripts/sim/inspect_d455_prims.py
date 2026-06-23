"""Inspect composed D455 prims under the movable camera holder.

Run from Isaac Sim Script Editor after opening/reloading the stage:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/inspect_d455_prims.py").read())
"""

import omni
from pxr import Usd, UsdGeom


D455_ROOT = "/World/robot_sim/camera_swing_link/d455_sensor"


def _attr_value(prim, name):
    attr = prim.GetAttribute(name)
    if not attr:
        return ""
    try:
        value = attr.Get()
    except Exception:
        return "<unreadable>"
    return "" if value is None else str(value)


def _material_targets(prim):
    rel = prim.GetRelationship("material:binding")
    if not rel:
        return ""
    try:
        return ", ".join(str(path) for path in rel.GetTargets())
    except Exception:
        return "<unreadable>"


def main():
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(D455_ROOT)
    if not root or not root.IsValid():
        raise RuntimeError(f"D455 root not found: {D455_ROOT}")

    print(f"D455 prim tree under {D455_ROOT}")
    print("Columns: active | type | visibility | purpose | material | path")
    for prim in Usd.PrimRange(root):
        imageable = UsdGeom.Imageable(prim)
        visibility = ""
        purpose = ""
        if imageable:
            visibility = _attr_value(prim, "visibility")
            purpose = _attr_value(prim, "purpose")

        print(
            f"{'ACTIVE' if prim.IsActive() else 'inactive':8s} "
            f"{prim.GetTypeName() or '-':12s} "
            f"vis={visibility or '-':10s} "
            f"purpose={purpose or '-':8s} "
            f"mat={_material_targets(prim) or '-'} "
            f"{prim.GetPath().pathString}"
        )


main()
