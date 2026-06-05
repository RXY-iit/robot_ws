"""Remove non-robot articulations from the currently opened Isaac stage.

Run from Isaac Sim Script Editor after loading full_warehouse_r.usd and before
starting the mapping sensors:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_foreign_articulations.py").read())

The mapping pipeline expects only the robot articulation. Extra forklifts,
vehicles, or animated robots in the warehouse can make PhysX tensor APIs return
shape (2, 6) instead of (6,), which can later cascade into CUDA/render errors.
"""

import omni
from pxr import Usd, UsdPhysics

try:
    from pxr import PhysxSchema
except Exception:
    PhysxSchema = None


ROBOT_ROOT = "/World/robot_sim"
ROBOT_ARTICULATION = "/World/robot_sim/base_footprint"
BAD_ROBOT_ARTICULATION_ROOTS = {
    "/World/robot_sim/root_joint",
}
ARTICULATION_SCHEMAS = {
    "PhysicsArticulationRootAPI",
    "PhysxArticulationAPI",
}
NAME_HINTS = (
    "forklift",
    "pallet_jack",
    "vehicle",
    "cart",
    "agv",
    "robot_",
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


def _has_articulation_schema(prim):
    schemas = set(prim.GetAppliedSchemas())
    if schemas.intersection(ARTICULATION_SCHEMAS):
        return True

    try:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return True
    except Exception:
        pass

    if PhysxSchema is not None:
        try:
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                return True
        except Exception:
            pass

    return False


def _hint_removal_root(path):
    parts = [part for part in path.split("/") if part]
    if len(parts) <= 3:
        return path

    for index, part in enumerate(parts):
        lower = part.lower()
        if any(hint in lower for hint in NAME_HINTS):
            return "/" + "/".join(parts[: index + 1])

    return path


def _remove_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)
        print(f"Removed foreign articulation/root prim: {path}")
        return True
    return False


def main():
    stage = omni.usd.get_context().get_stage()
    removal_roots = set()
    found_articulations = []

    for prim in _iter_all_prims(stage):
        path = prim.GetPath().pathString
        lower = path.lower()
        is_active = prim.IsActive()

        if path in BAD_ROBOT_ARTICULATION_ROOTS:
            if _has_articulation_schema(prim):
                found_articulations.append((path, is_active, "bad_robot_root"))
            removal_roots.add(path)
            continue

        if path == ROBOT_ARTICULATION or path.startswith(f"{ROBOT_ROOT}/"):
            if _has_articulation_schema(prim):
                found_articulations.append((path, is_active, "robot"))
            continue

        if _has_articulation_schema(prim):
            found_articulations.append((path, is_active, "foreign"))
            removal_roots.add(path)
            continue

        if any(hint in lower for hint in NAME_HINTS):
            if not path.startswith(ROBOT_ROOT):
                removal_roots.add(_hint_removal_root(path))
                print(f"Found foreign moving-object hint: {path}")

    print("Articulation scan before cleanup:")
    if not found_articulations:
        print("  no articulation schemas found")
    for path, is_active, kind in found_articulations:
        state = "ACTIVE" if is_active else "inactive"
        print(f"  {state} {kind}: {path}")

    removed = 0
    for path in sorted(removal_roots, key=len, reverse=True):
        if (not path.startswith(ROBOT_ROOT) or path in BAD_ROBOT_ARTICULATION_ROOTS) and _remove_prim(stage, path):
            removed += 1

    if removed == 0:
        print("No foreign articulations removed.")
    else:
        print(f"Removed {removed} foreign articulation/root prim(s).")
    remaining = []
    for prim in _iter_all_prims(stage):
        if _has_articulation_schema(prim):
            remaining.append((prim.GetPath().pathString, prim.IsActive()))

    print("Articulation scan after cleanup:")
    for path, is_active in remaining:
        state = "ACTIVE" if is_active else "inactive"
        print(f"  {state}: {path}")

    active_remaining = [
        path for path, is_active in remaining
        if is_active and path != ROBOT_ARTICULATION
    ]
    if active_remaining:
        print("WARNING: active foreign articulation(s) remain:")
        for path in active_remaining:
            print(f"  {path}")
        print("Remove these manually or extend cleanup rules before Play.")
    else:
        print("OK: only the robot articulation should remain active.")


main()
