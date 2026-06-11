"""Inspect active RTX LiDAR and ROS publishing pipeline state."""

import omni
import omni.graph.core as og
from pxr import Usd


def main():
    stage = omni.usd.get_context().get_stage()
    interesting = []

    for root_path in ("/World", "/Render", "/Replicator"):
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            continue
        for prim in Usd.PrimRange(root):
            path = prim.GetPath().pathString
            lower = path.lower()
            if any(
                token in lower
                for token in (
                    "localization_lidar",
                    "livox_lidar_ros_graph",
                    "rtxlidar",
                    "robotsimlivox",
                    "replicator_",
                    "writersyncgate",
                    "dispatchsync",
                )
            ):
                interesting.append((path, prim.IsActive(), prim.GetTypeName()))

    print(f"Found {len(interesting)} RTX LiDAR pipeline-related prim(s):")
    for path, active, type_name in interesting:
        print(f"  {'ACTIVE' if active else 'inactive'} {type_name or '-'} {path}")

    graph = og.get_graph_by_path("/World/robot_sim/livox_lidar_ros_graph")
    if graph and graph.is_valid():
        print("LiDAR ROS graph nodes:")
        for node in graph.get_nodes():
            print(f"  {node.get_type_name()}  {node.get_prim_path()}")


main()
