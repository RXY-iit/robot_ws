"""Remove experimental Isaac physics IMU ROS graph/prims.

Run from Isaac Sim Script Editor after the true IMU test triggers PhysX tensor
errors, then restart the stage before continuing LiDAR-only mapping:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_livox_imu_ros.py").read())
"""

import omni
from pxr import Usd


PATH_HINTS = (
    "livox_imu_ros_graph",
    "isaac_livox_imu",
)


def _remove_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)
        print(f"Removed prim: {path}")
        return True
    return False


def main():
    stage = omni.usd.get_context().get_stage()
    paths = []
    root = stage.GetPrimAtPath("/World/robot_sim")
    if root and root.IsValid():
        for prim in Usd.PrimRange(root):
            path = prim.GetPath().pathString
            lower = path.lower()
            if any(hint in lower for hint in PATH_HINTS):
                paths.append(path)

    if _remove_prim(stage, "/World/robot_sim/livox_imu_ros_graph"):
        paths = [p for p in paths if p != "/World/robot_sim/livox_imu_ros_graph"]

    removed = 0
    for path in sorted(set(paths), key=len, reverse=True):
        if _remove_prim(stage, path):
            removed += 1

    print(f"Removed {removed} experimental Isaac IMU runtime prim(s).")
    print("Restart Isaac Sim before the next mapping run if CUDA errors occurred.")


main()
