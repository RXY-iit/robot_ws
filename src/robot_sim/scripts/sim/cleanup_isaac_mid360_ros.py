"""Clean stale Isaac RTX LiDAR ROS publishers for robot_sim mapping.

Run this in Isaac Sim Script Editor when /point_cloud has multiple publishers:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/cleanup_isaac_mid360_ros.py").read())

Then run the MID360 setup script exactly once.
"""

import omni
import omni.replicator.core as rep
from pxr import Usd


LIVOX_PARENT = "/World/robot_sim/base_footprint/base_link/livox_frame"
LIVOX_PATH = "localization_lidar"
SENSOR_PATH = f"{LIVOX_PARENT}/{LIVOX_PATH}"


def _try_detach_writer():
    try:
        writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    except Exception as exc:
        print(f"Could not get RtxLidarROS2PublishPointCloud writer: {exc}")
        return

    for call in (
        lambda: writer.detach(),
        lambda: writer.detach([]),
        lambda: writer.detach(None),
    ):
        try:
            call()
            print("Detached RtxLidarROS2PublishPointCloud writer.")
            return
        except TypeError:
            continue
        except Exception as exc:
            print(f"Writer detach attempt failed: {exc}")
            continue
    print("Writer detach was not available; removing stage prims only.")


def _remove_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if prim and prim.IsValid():
        stage.RemovePrim(path)
        print(f"Removed prim: {path}")
        return True
    return False


def _remove_runtime_lidar_prims(stage):
    removed = 0
    if _remove_prim(stage, SENSOR_PATH):
        removed += 1

    for root_path in ("/Render", "/Replicator"):
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            continue

        paths = []
        for prim in Usd.PrimRange(root):
            path = prim.GetPath().pathString
            lower = path.lower()
            if (
                "localization_lidar" in lower
                or "rtxlidar" in lower
                or "lidar" in lower
                or "pointcloud" in lower
            ):
                paths.append(path)

        for path in sorted(paths, key=len, reverse=True):
            if _remove_prim(stage, path):
                removed += 1

    print(f"Removed {removed} lidar-related runtime prim(s).")


def main():
    stage = omni.usd.get_context().get_stage()
    _try_detach_writer()
    _remove_runtime_lidar_prims(stage)
    print("MID360 ROS cleanup done.")
    print("Now run once:")
    print(
        '  exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/'
        'setup_isaac_mid360_like_ros.py").read())'
    )


main()
