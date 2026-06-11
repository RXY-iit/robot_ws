"""Prepare the referenced D455 asset for camera-only mapping experiments.

Run from Isaac Sim's Script Editor before pressing Play. This keeps Camera
prims active, but disables rigid-body and collision simulation below the D455
reference so the sensor cannot create a nested rigid body inside the robot
articulation.

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/prepare_d455_camera_only.py").read())
"""

import omni
from pxr import Usd, UsdPhysics


D455_ROOT = (
    "/World/robot_sim/base_footprint/base_link/"
    "camera_bottom_screw_frame/camera_link/d455_sensor"
)
PHYSICS_SENSOR_HINTS = ("imu", "accelerometer", "gyroscope")


def _iter_prims(root):
    yield from Usd.PrimRange(root)


def main():
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(D455_ROOT)
    if not root or not root.IsValid():
        raise RuntimeError(f"D455 root not found: {D455_ROOT}")

    root.SetActive(True)

    rigid_bodies_removed = 0
    mass_apis_removed = 0
    collisions_disabled = 0
    joints_disabled = 0
    physics_sensors_disabled = 0
    cameras = []

    # Disable child physics sensors before touching their parent rigid body.
    # Removing RSD455 first can let its Imu_Sensor leave a stale tensor reader
    # that starts reporting getVelocities shape (2, 6) when Play begins.
    for prim in list(_iter_prims(root)):
        path = prim.GetPath().pathString
        lower_path = path.lower()
        if (
            prim.GetTypeName() != "Camera"
            and any(hint in lower_path for hint in PHYSICS_SENSOR_HINTS)
        ):
            prim.SetActive(False)
            physics_sensors_disabled += 1
            print(f"Disabled D455 non-camera physics sensor: {path}")

    for prim in list(_iter_prims(root)):
        path = prim.GetPath().pathString
        if prim.GetTypeName() == "Camera":
            cameras.append(path)

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            rigid_bodies_removed += 1
            print(f"Removed D455 RigidBodyAPI: {path}")

        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)
            mass_apis_removed += 1

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            collisions_disabled += 1

        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False)
            joints_disabled += 1

    print("D455 camera-only preparation complete:")
    print(f"  root: {D455_ROOT}")
    print(f"  rigid body APIs removed: {rigid_bodies_removed}")
    print(f"  mass APIs removed: {mass_apis_removed}")
    print(f"  collisions disabled: {collisions_disabled}")
    print(f"  joints disabled: {joints_disabled}")
    print(f"  non-camera physics sensors disabled: {physics_sensors_disabled}")
    print(f"  camera prims found: {len(cameras)}")
    for path in cameras:
        print(f"    {path}")

    if not cameras:
        raise RuntimeError("D455 camera-only mode found no Camera prims.")

    remaining_rigid_bodies = [
        prim.GetPath().pathString
        for prim in _iter_prims(root)
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    if remaining_rigid_bodies:
        raise RuntimeError(
            "D455 still contains RigidBodyAPI after cleanup:\n  "
            + "\n  ".join(remaining_rigid_bodies)
        )

    active_physics_sensors = [
        prim.GetPath().pathString
        for prim in _iter_prims(root)
        if prim.IsActive()
        and prim.GetTypeName() != "Camera"
        and any(
            hint in prim.GetPath().pathString.lower()
            for hint in PHYSICS_SENSOR_HINTS
        )
    ]
    if active_physics_sensors:
        raise RuntimeError(
            "D455 still contains active non-camera physics sensors:\n  "
            + "\n  ".join(active_physics_sensors)
        )

    print("OK: D455 cameras remain active without participating in robot physics.")


main()
