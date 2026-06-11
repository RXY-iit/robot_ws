"""Publish the robot D455 camera to ROS2 topics from Isaac Sim 5.0.

Run prepare_d455_camera_only.py before Play, then run this script exactly once
after Play. The D455 is a recording-only side channel and is not used by GLIM.
The conservative default resolution/rate limits extra RTX load while validating
that camera recording does not destabilize LiDAR mapping.
"""

import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.timeline
from pxr import Usd, UsdPhysics


D455_ROOT = (
    "/World/robot_sim/base_footprint/base_link/"
    "camera_bottom_screw_frame/camera_link/d455_sensor"
)
PHYSICS_SENSOR_HINTS = ("imu", "accelerometer", "gyroscope")

COLOR_TOPIC = "camera/camera/color/image_raw"
COLOR_INFO_TOPIC = "camera/camera/color/camera_info"
DEPTH_TOPIC = "camera/camera/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "camera/camera/depth/camera_info"

COLOR_FRAME_ID = "camera_color_optical_frame"
DEPTH_FRAME_ID = "camera_depth_optical_frame"

RESOLUTION = (640, 360)
PUBLISH_HZ = 10
QUEUE_SIZE = 1
NODE_NAMESPACE = ""


def _camera_prims_under(root_path):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return []

    cameras = []
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "Camera":
            cameras.append(prim.GetPath().pathString)
    return cameras


def _camera_prims_in_stage():
    stage = omni.usd.get_context().get_stage()
    cameras = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Camera":
            cameras.append(prim.GetPath().pathString)
    return cameras


def _find_d455_cameras():
    stage = omni.usd.get_context().get_stage()
    candidate_roots = [
        D455_ROOT,
        D455_ROOT.replace("/World/robot_sim/", "/World/robot_sim_physics/robot_sim/"),
    ]

    for root in candidate_roots:
        cameras = _camera_prims_under(root)
        if cameras:
            print(f"Using D455 root: {root}")
            return cameras

    discovered_roots = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        lower = path.lower()
        if lower.endswith("/d455_sensor") or "d455_sensor" in lower:
            discovered_roots.append(path)

    for root in discovered_roots:
        cameras = _camera_prims_under(root)
        if cameras:
            print(f"Using discovered D455 root: {root}")
            return cameras

    all_cameras = _camera_prims_in_stage()
    likely = [
        path for path in all_cameras
        if any(key in path.lower() for key in ("d455", "realsense", "camera"))
    ]
    if likely:
        print("D455 root was not found; using likely camera prims from stage.")
        return likely

    debug_paths = []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if any(key in path.lower() for key in ("d455", "camera", "depth", "color")):
            debug_paths.append(path)

    raise RuntimeError(
        "No Camera prims found for D455. Matching non-camera prims:\n  "
        + "\n  ".join(debug_paths[:80])
    )


def _rigid_bodies_under(root_path):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return []

    rigid_bodies = []
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_bodies.append(prim.GetPath().pathString)
    return rigid_bodies


def _active_physics_sensors_under(root_path):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return []

    sensors = []
    for prim in Usd.PrimRange(root):
        path = prim.GetPath().pathString
        if (
            prim.IsActive()
            and prim.GetTypeName() != "Camera"
            and any(hint in path.lower() for hint in PHYSICS_SENSOR_HINTS)
        ):
            sensors.append(path)
    return sensors


def _pick_camera(cameras, keywords, fallback_index=0):
    for path in cameras:
        name = path.lower()
        if any(keyword in name for keyword in keywords):
            return path
    if cameras:
        return cameras[min(fallback_index, len(cameras) - 1)]
    raise RuntimeError("No Camera prims found for D455")


def _set_gate_step(render_product, render_var, hz):
    render_product_path = _render_product_path(render_product)
    step_size = max(1, int(round(60.0 / float(hz))))
    try:
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(
            render_var + "IsaacSimulationGate",
            render_product_path,
        )
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    except Exception as exc:
        print(f"Warning: could not set publish gate for {render_var}: {exc}")


def _render_product_path(render_product):
    path = getattr(render_product, "path", render_product)
    return str(path)


def _publish_image(render_product, render_var, topic_name, frame_id):
    render_product_path = _render_product_path(render_product)
    writer = rep.writers.get(render_var + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=NODE_NAMESPACE,
        queueSize=QUEUE_SIZE,
        topicName=topic_name,
    )
    writer.attach([render_product_path])
    _set_gate_step(render_product_path, render_var, PUBLISH_HZ)


def _publish_camera_info(render_product, topic_name, frame_id):
    try:
        from isaacsim.ros2.bridge import read_camera_info
    except Exception:
        from isaacsim.ros2.core import read_camera_info

    render_product_path = _render_product_path(render_product)
    camera_info, _ = read_camera_info(render_product_path=render_product_path)
    writer = rep.writers.get("ROS2PublishCameraInfo")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=NODE_NAMESPACE,
        queueSize=QUEUE_SIZE,
        topicName=topic_name,
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape([1, 9]),
        r=camera_info.r.reshape([1, 9]),
        p=camera_info.p.reshape([1, 12]),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )
    writer.attach([render_product_path])
    _set_gate_step(render_product_path, "PostProcessDispatch", PUBLISH_HZ)


def main():
    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Press Play before creating D455 ROS publishers.")

    rigid_bodies = _rigid_bodies_under(D455_ROOT)
    if rigid_bodies:
        raise RuntimeError(
            "D455 still contains RigidBodyAPI. Restart Isaac Sim and "
            "run prepare_d455_camera_only.py BEFORE Play:\n  "
            + "\n  ".join(rigid_bodies)
        )

    physics_sensors = _active_physics_sensors_under(D455_ROOT)
    if physics_sensors:
        raise RuntimeError(
            "D455 still contains active non-camera physics sensors. Restart "
            "Isaac Sim and run prepare_d455_camera_only.py BEFORE Play:\n  "
            + "\n  ".join(physics_sensors)
        )

    cameras = _find_d455_cameras()
    print("D455 camera prim candidates:")
    for path in cameras:
        print(f"  {path}")

    color_camera = _pick_camera(cameras, ["color", "rgb"], 0)
    depth_camera = _pick_camera(cameras, ["depth", "pseudo_depth"], 0)

    color_rp = rep.create.render_product(color_camera, resolution=RESOLUTION)
    depth_rp = rep.create.render_product(depth_camera, resolution=RESOLUTION)

    rgb_var = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.Rgb.name
    )
    depth_var = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )

    _publish_image(color_rp, rgb_var, COLOR_TOPIC, COLOR_FRAME_ID)
    _publish_camera_info(color_rp, COLOR_INFO_TOPIC, COLOR_FRAME_ID)
    _publish_image(depth_rp, depth_var, DEPTH_TOPIC, DEPTH_FRAME_ID)
    _publish_camera_info(depth_rp, DEPTH_INFO_TOPIC, DEPTH_FRAME_ID)

    print("D455 ROS2 publishers ready:")
    print(f"  /{COLOR_TOPIC}        frame_id={COLOR_FRAME_ID}")
    print(f"  /{COLOR_INFO_TOPIC}   frame_id={COLOR_FRAME_ID}")
    print(f"  /{DEPTH_TOPIC}        frame_id={DEPTH_FRAME_ID}")
    print(f"  /{DEPTH_INFO_TOPIC}   frame_id={DEPTH_FRAME_ID}")
    print(f"  recording profile: {RESOLUTION[0]}x{RESOLUTION[1]} @ {PUBLISH_HZ} Hz")
    print("Run this setup exactly once per Isaac Sim session.")


main()
