"""Experimental MID360-like RTX LiDAR publisher for A/B comparison.

Run from Isaac Sim Script Editor after opening robot_sim_physics.usd and
pressing Play:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/setup_isaac_mid360_like_ros.py").read())

This is NOT an official Livox MID360 model. Isaac Sim 5.0 does not ship one.
The goal is only to compare whether this MID360-like custom profile behaves
better or worse than the built-in Ouster OS1 128ch profile for localization.

Output:
  /point_cloud

The existing ROS adapter then publishes:
  /livox/lidar_pc2_accumulated
  /livox/lidar
"""

import omni
import omni.replicator.core as rep
from pxr import Gf, Usd


LIVOX_PARENT = "/World/robot_sim/base_footprint/base_link/livox_frame"
LIVOX_PATH = "localization_lidar"
TOPIC = "point_cloud"
FRAME_ID = "livox_frame"

MID360_ELEVATION_DEG = [
    -7.0, -5.525, -4.050, -2.575, -1.1004, 0.374, 1.849,
    3.324, 4.799, 6.274, 7.7494, 9.2249, 10.699, 12.174,
    13.645, 15.1243, 16.5999, 18.074, 19.5499, 21.024,
    22.493, 23.9749, 25.44, 26.924, 28.39, 29.8743,
    31.3499, 32.824, 34.29, 35.774, 37.2486, 38.724,
    40.19, 41.674, 43.14, 44.624, 46.09, 47.574, 49.048,
    50.524,
]


def _mid360_like_attrs():
    elevations = []
    azimuths = []
    channels = []
    group_offsets = [0.0, 0.45, 0.95, 1.55]
    for group, azimuth_offset in enumerate(group_offsets):
        elevation_offset = ((group % 3) - 1) * 0.12
        for channel, elevation in enumerate(MID360_ELEVATION_DEG, start=1):
            elevations.append(elevation + elevation_offset)
            azimuths.append(azimuth_offset)
            channels.append(channel)

    emitters = len(elevations)
    report_period_ns = int(1e9 / 20000)
    fire_times = [
        int(i * report_period_ns / max(1, emitters - 1))
        for i in range(emitters)
    ]

    return {
        "omni:sensor:Core:scanType": "ROTARY",
        "omni:sensor:Core:intensityProcessing": "NORMALIZATION",
        "omni:sensor:Core:rotationDirection": "CW",
        "omni:sensor:Core:rayType": "IDEALIZED",
        "omni:sensor:Core:nearRangeM": 0.1,
        "omni:sensor:Core:farRangeM": 40.0,
        "omni:sensor:Core:rangeResolutionM": 0.004,
        "omni:sensor:Core:rangeAccuracyM": 0.025,
        "omni:sensor:Core:avgPowerW": 0.002,
        "omni:sensor:Core:minReflectance": 0.1,
        "omni:sensor:Core:minReflectanceRange": 70.0,
        "omni:sensor:Core:wavelengthNm": 905.0,
        "omni:sensor:Core:pulseTimeNs": 6,
        "omni:sensor:Core:azimuthErrorMean": 0.02,
        "omni:sensor:Core:azimuthErrorStd": 0.15,
        "omni:sensor:Core:elevationErrorMean": 0.02,
        "omni:sensor:Core:elevationErrorStd": 0.15,
        "omni:sensor:Core:maxReturns": 2,
        "omni:sensor:Core:scanRateBaseHz": 10.0,
        "omni:sensor:Core:reportRateBaseHz": 20000,
        "omni:sensor:Core:numberOfEmitters": emitters,
        "omni:sensor:Core:numberOfChannels": 40,
        "omni:sensor:Core:rangeOffset": 0.03,
        "omni:sensor:Core:intensityMappingType": "LINEAR",
        "omni:sensor:Core:emitterState:s001:azimuthDeg": azimuths,
        "omni:sensor:Core:emitterState:s001:elevationDeg": elevations,
        "omni:sensor:Core:emitterState:s001:fireTimeNs": fire_times,
        "omni:sensor:Core:emitterState:s001:distanceCorrectionM": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:focalDistM": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:focalSlope": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:horOffsetM": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:reportRateDiv": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:vertOffsetM": [0.0] * emitters,
        "omni:sensor:Core:emitterState:s001:channelId": channels,
    }


def _try_detach_existing_writer():
    try:
        writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    except Exception as exc:
        print(f"Warning: could not get existing RTX LiDAR ROS writer: {exc}")
        return

    for call in (
        lambda: writer.detach(),
        lambda: writer.detach([]),
        lambda: writer.detach(None),
    ):
        try:
            call()
            print("Detached existing RTX LiDAR ROS writer.")
            return
        except TypeError:
            continue
        except Exception as exc:
            print(f"Warning: writer detach attempt failed: {exc}")


def _remove_lidar_runtime_prims(stage):
    sensor_path = f"{LIVOX_PARENT}/{LIVOX_PATH}"
    existing = stage.GetPrimAtPath(sensor_path)
    if existing and existing.IsValid():
        stage.RemovePrim(sensor_path)
        print(f"Removed old LiDAR prim: {sensor_path}")

    render_root = stage.GetPrimAtPath("/Render")
    if not render_root or not render_root.IsValid():
        return

    paths = []
    for prim in Usd.PrimRange(render_root):
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
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(path)
            print(f"Removed old LiDAR render prim: {path}")


def main():
    sensor_path = f"{LIVOX_PARENT}/{LIVOX_PATH}"
    stage = omni.usd.get_context().get_stage()
    _try_detach_existing_writer()
    _remove_lidar_runtime_prims(stage)

    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=LIVOX_PATH,
        parent=LIVOX_PARENT,
        config="OS1",
        variant="OS1_REV6_32ch20hz1024res",
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=False,
        force_camera_prim=False,
        **_mid360_like_attrs(),
    )

    render_product = rep.create.render_product(sensor.GetPath(), resolution=(1, 1))
    writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
    writer.initialize(topicName=TOPIC, frameId=FRAME_ID)
    writer.attach([render_product])
    print(
        f"Created experimental MID360-like LiDAR at {sensor.GetPath()} "
        f"and ROS2 PointCloud2 topic /{TOPIC}"
    )
    print("For RViz, prefer /livox/lidar_pc2_accumulated from the ROS adapter.")


main()
