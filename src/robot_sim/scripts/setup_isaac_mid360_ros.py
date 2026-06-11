"""Create the robot_sim localization LiDAR and ROS2 PointCloud2 publisher.

Run from Isaac Sim Script Editor after opening robot_sim_physics.usd and
pressing Play. The output topic intentionally stays named
`/point_cloud` so it matches common 3D LiDAR odometry examples. The ROS adapter
then converts it to the same `/livox/lidar` Livox CustomMsg topic used by the
real robot stack.

Supported LIDAR_MODE values:

- os1_128:
  Default. Isaac Sim built-in Ouster OS1 REV6 128-channel profile at 10 Hz and
  1024 horizontal resolution. This is the best current choice for localization
  algorithm validation because it provides stable dense 3D geometry.

- os1_32:
  Lower-load Ouster OS1 32-channel profile at 10 Hz and 1024 horizontal
  resolution. Use this if os1_128 is too heavy.

- hesai_xt32:
  Isaac Sim built-in Hesai XT32 profile. 32 channels, 10 Hz, traditional
  rotary 3D LiDAR.

- sick_multiscan136:
  Isaac Sim built-in SICK multiScan136 profile. Wider vertical coverage, but it
  can be significantly heavier.

Why not a MID360 profile here?
Isaac Sim 5.0 does not ship an official Livox MID360 RTX LiDAR model. MID360's
non-repetitive scan can only be approximated with custom profiles/scripts. For
localization bringup, a stable built-in 3D LiDAR plus the Livox CustomMsg
adapter is currently more useful than a visually misleading MID360 mimic.
"""

import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.timeline
from pxr import Gf


# Parent frame in robot_sim_physics.usd. The ROS frame_id also uses this name.
LIVOX_PARENT = "/World/robot_sim/base_footprint/base_link/livox_frame"

# Runtime sensor prim name under LIVOX_PARENT. Re-running the script removes and
# recreates this prim so changes to LIDAR_MODE take effect.
LIVOX_PATH = "localization_lidar"

# Raw Isaac RTX LiDAR PointCloud2 topic. MOLA examples and many 3D LiDAR
# odometry demos use `/point_cloud` as the input topic. RViz should normally
# compare this raw topic against the adapter's accumulated
# `/livox/lidar_pc2_accumulated` topic.
TOPIC = "point_cloud"

# Frame used by the raw PointCloud2 publisher and by the adapter output.
FRAME_ID = "livox_frame"
GRAPH_PATH = "/World/robot_sim/livox_lidar_ros_graph"

# Override from Script Editor before exec(...) when needed, for example:
# LIDAR_MODE = "os1_32"
LIDAR_MODE = globals().get("LIDAR_MODE", "os1_128")

# IsaacSensorCreateRtxLidar accepts a model config plus optional variant.
# The config/variant names below come from Isaac Sim 5.0's RTX LiDAR library.
LIDAR_PROFILES = {
    "os1_128": ("OS1", "OS1_REV6_128ch10hz1024res"),
    "os1_32": ("OS1", "OS1_REV6_32ch10hz1024res"),
    "hesai_xt32": ("Hesai_XT32_SD10", None),
    "sick_multiscan136": ("SICK_multiScan136", None),
}


def main():
    if LIDAR_MODE not in LIDAR_PROFILES:
        supported = ", ".join(sorted(LIDAR_PROFILES))
        raise ValueError(f"Unsupported LIDAR_MODE={LIDAR_MODE!r}. Use one of: {supported}")

    sensor_path = f"{LIVOX_PARENT}/{LIVOX_PATH}"
    stage = omni.usd.get_context().get_stage()
    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Press Play before creating the RTX LiDAR publisher.")

    existing = stage.GetPrimAtPath(sensor_path)
    if existing and existing.IsValid():
        raise RuntimeError(
            f"RTX LiDAR already exists at {sensor_path}. Restart Isaac Sim instead "
            "of recreating an active RTX sensor."
        )
    existing_graph = stage.GetPrimAtPath(GRAPH_PATH)
    if existing_graph and existing_graph.IsValid():
        raise RuntimeError(
            f"LiDAR ROS graph already exists: {GRAPH_PATH}. Restart Isaac Sim "
            "instead of creating another RTX publishing pipeline."
        )

    config, variant = LIDAR_PROFILES[LIDAR_MODE]
    command_kwargs = {
        "path": LIVOX_PATH,
        "parent": LIVOX_PARENT,
        "config": config,
        "translation": Gf.Vec3d(0.0, 0.0, 0.0),
        "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        "visibility": False,
        "force_camera_prim": False,
    }
    if variant:
        command_kwargs["variant"] = variant

    _, sensor = omni.kit.commands.execute("IsaacSensorCreateRtxLidar", **command_kwargs)

    render_product = rep.create.render_product(
        sensor.GetPath(),
        # RTX LiDAR data comes from the sensor AOV; a 1x1 render product avoids
        # spending GPU work on an unused image-sized target.
        resolution=(1, 1),
        render_vars=["GenericModelOutput", "RtxSensorMetadata"],
        name="RobotSimLivox",
    )

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishPointCloud", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("PublishPointCloud.inputs:renderProductPath", render_product.path),
                ("PublishPointCloud.inputs:topicName", TOPIC),
                ("PublishPointCloud.inputs:frameId", FRAME_ID),
                ("PublishPointCloud.inputs:type", "point_cloud"),
                ("PublishPointCloud.inputs:fullScan", True),
                ("PublishPointCloud.inputs:useSystemTime", False),
                ("PublishPointCloud.inputs:resetSimulationTimeOnStop", False),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishPointCloud.inputs:execIn"),
            ],
        },
    )
    print(
        f"Created {sensor.GetPath()} using LIDAR_MODE={LIDAR_MODE!r} "
        f"and ROS2 PointCloud2 topic /{TOPIC}"
    )
    print("Publisher uses ROS2RtxLidarHelper with fullScan=True.")
    print("For RViz, prefer /livox/lidar_pc2_accumulated from the ROS adapter.")


main()
