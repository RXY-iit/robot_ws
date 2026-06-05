"""List all articulation roots in the current Isaac stage.

Run from Isaac Sim Script Editor after loading a USD:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/list_articulations.py").read())
"""

import omni
from pxr import Usd, UsdPhysics

try:
    from pxr import PhysxSchema
except Exception:
    PhysxSchema = None


ARTICULATION_SCHEMAS = {
    "PhysicsArticulationRootAPI",
    "PhysxArticulationAPI",
}
ROBOT_ARTICULATION = "/World/robot_sim/base_footprint"
BAD_ROBOT_ARTICULATION_ROOTS = {
    "/World/robot_sim/root_joint",
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


def _articulation_schemas(prim):
    schemas = set(prim.GetAppliedSchemas())
    found = set(schemas.intersection(ARTICULATION_SCHEMAS))

    try:
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            found.add("PhysicsArticulationRootAPI")
    except Exception:
        pass

    if PhysxSchema is not None:
        try:
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                found.add("PhysxArticulationAPI")
        except Exception:
            pass

    return found


def main():
    stage = omni.usd.get_context().get_stage()
    found = []
    for prim in _iter_all_prims(stage):
        schemas = _articulation_schemas(prim)
        if schemas:
            found.append((prim.GetPath().pathString, prim.IsActive(), schemas))

    print(f"Found {len(found)} articulation prim(s):")
    for path, active, schemas in found:
        state = "ACTIVE" if active else "inactive"
        if path == ROBOT_ARTICULATION:
            label = "robot"
        elif path in BAD_ROBOT_ARTICULATION_ROOTS:
            label = "BAD_ROOT"
        else:
            label = "foreign"
        print(f"{state} {label} {path}")
        print(f"  schemas: {sorted(schemas)}")


main()
