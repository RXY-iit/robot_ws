"""Create an Isaac physics IMU and publish it to ROS2 /livox/imu.

Reads sensor data via ImuSensorInterface (C++ physics binding) in a physics step
callback instead of using the OmniGraph IsaacReadIMU node.

Root cause of the old approach: IsaacReadIMU creates 2 ArticulationView
registrations internally in Isaac Sim 5.x, causing:
    getVelocities: expected total size 6, received total size 12 with shape (2, 6)
This error appeared even with IsaacArticulationController disabled, confirming the
bug is inside IsaacReadIMU itself. Removing the node eliminates the conflict.

This version uses isaacsim.sensors.physics.ImuSensorInterface directly via a
physics step subscription. It avoids the known-bad IsaacReadIMU OmniGraph node;
validate in the Isaac console that the PhysX getVelocities (2,6) error no
longer appears after pressing Play.

Run from Isaac Sim Script Editor BEFORE pressing Play:

    exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_livox_imu_ros.py").read())

Then press Play. After Play is stable, run setup_isaac_mid360_like_ros.py.
"""

import os
import omni
import omni.graph.core as og
import omni.kit.app
import omni.physx
from pxr import Gf, Usd

# ── Sensor placement ───────────────────────────────────────────────────────────
RIGID_BODY_PARENT = "/World/robot_sim/base_footprint"
OLD_LIVOX_PARENT  = "/World/robot_sim/base_footprint/base_link/livox_frame"
IMU_NAME     = "isaac_livox_imu"
IMU_PATH     = f"{RIGID_BODY_PARENT}/{IMU_NAME}"
OLD_IMU_PATH = f"{OLD_LIVOX_PARENT}/{IMU_NAME}"

# URDF chain: base_footprint → base_link (z=0.1125) → livox_frame (xyz=0.24,0,1.38 pitch=30 deg)
IMU_TRANSLATION = Gf.Vec3d(0.240, 0.000, 1.4925)
IMU_ORIENTATION = Gf.Quatd(0.9659258, 0.0, 0.2588190, 0.0)

# ── ROS2 publish graph ─────────────────────────────────────────────────────────
GRAPH_PATH = "/World/robot_sim/livox_imu_ros_graph"
TOPIC_NAME = "livox/imu"
FRAME_ID   = "livox_frame"
SENSOR_HZ  = 200.0  # sensor buffer period; actual publish rate ≤ physics Hz (120)
SIM_TIME_OUTPUT = "/World/ROS2_ActionGraph/read_sim_time.outputs:simulationTime"

# ── Subscription persistence ───────────────────────────────────────────────────
# Store the subscription handle on the omni module so it survives exec() re-runs
# and is not garbage-collected (GC would silently unsubscribe the callback).

def _save_sub(sub):
    omni._livox_imu_physx_sub = sub          # noqa: SLF001

def _cancel_old_sub():
    if hasattr(omni, "_livox_imu_physx_sub"):
        omni._livox_imu_physx_sub = None     # release → unsubscribe old callback


# ── Helpers ────────────────────────────────────────────────────────────────────

def _enable_extension(name):
    mgr = omni.kit.app.get_app().get_extension_manager()
    if not mgr.is_extension_enabled(name):
        mgr.set_extension_enabled_immediate(name, True)
        print(f"  enabled extension: {name}")


def _remove_prim(stage, path):
    p = stage.GetPrimAtPath(path)
    if p and p.IsValid():
        stage.RemovePrim(path)
        print(f"  removed: {path}")


def _remove_stale_imu_prims(stage):
    paths = {GRAPH_PATH, IMU_PATH, OLD_IMU_PATH}
    robot = stage.GetPrimAtPath("/World/robot_sim")
    if robot and robot.IsValid():
        for prim in Usd.PrimRange(robot):
            path = prim.GetPath().pathString
            lower = path.lower()
            if "livox_imu_ros_graph" in lower or "isaac_livox_imu" in lower:
                paths.add(path)
    for path in sorted(paths, key=len, reverse=True):
        _remove_prim(stage, path)


def _acquire_imu_interface():
    from isaacsim.sensors.physics import _sensor
    iface = _sensor.acquire_imu_sensor_interface()
    print("  ImuSensorInterface acquired (C++ direct binding, no IsaacReadIMU node)")
    return iface


def _make_publish_graph(stage):
    """Minimal OmniGraph: OnPhysicsStep → ROS2PublishImu.  No IsaacReadIMU."""
    sim_time_attr = stage.GetAttributeAtPath(SIM_TIME_OUTPUT)
    if not sim_time_attr or not sim_time_attr.IsValid():
        raise RuntimeError(
            "Existing Isaac sim-time output not found: "
            f"{SIM_TIME_OUTPUT}. Open robot_sim/full_warehouse scene first."
        )

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": GRAPH_PATH,
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                ("PublishIMU",    "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnPhysicsStep.outputs:step", "PublishIMU.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("PublishIMU.inputs:topicName",                TOPIC_NAME),
                ("PublishIMU.inputs:frameId",                  FRAME_ID),
                ("PublishIMU.inputs:queueSize",                1000),
                ("PublishIMU.inputs:publishAngularVelocity",   True),
                ("PublishIMU.inputs:publishLinearAcceleration", True),
                ("PublishIMU.inputs:publishOrientation",        True),
            ],
        },
    )
    print(f"  publish-only IMU graph created: {GRAPH_PATH}")
    print("  No IsaacReadIMU OmniGraph node is created")
    print(f"  IMU timestamp source: {SIM_TIME_OUTPUT} (copied in Python callback)")


def _subscribe_physics_reader(imu_iface):
    """Physics step callback: read via C++ iface, push values into OmniGraph inputs.

    The Python callback updates the latest true IMU values each physics step.
    ROS2PublishImu is also triggered by OnPhysicsStep so publishing cannot pause
    independently of the physics reader, as happened with OnPlaybackTick.
    """
    _la = f"{GRAPH_PATH}/PublishIMU.inputs:linearAcceleration"
    _av = f"{GRAPH_PATH}/PublishIMU.inputs:angularVelocity"
    _or = f"{GRAPH_PATH}/PublishIMU.inputs:orientation"
    _ts = f"{GRAPH_PATH}/PublishIMU.inputs:timeStamp"

    _debug_counter = [0]

    def _on_step(_):  # step_size in seconds, unused but required by subscribe_physics_step_events
        try:
            r = imu_iface.get_sensor_reading(IMU_PATH, read_gravity=True)
        except TypeError:
            try:
                readings = imu_iface.get_sensor_readings(IMU_PATH, use_latest_data=True)
            except TypeError:
                readings = imu_iface.get_sensor_readings(IMU_PATH)
            if not readings:
                return
            r = readings[0] if hasattr(readings, "__getitem__") else readings

        if not r or (hasattr(r, "is_valid") and not r.is_valid):
            return

        def _f(obj, *names):
            for n in names:
                if hasattr(obj, n):
                    v = getattr(obj, n)
                    return float(v) if not hasattr(v, "__len__") else v
            return 0.0

        lin_acc = (_f(r, "lin_acc_x"), _f(r, "lin_acc_y"), _f(r, "lin_acc_z"))
        ang_vel = (_f(r, "ang_vel_x"), _f(r, "ang_vel_y"), _f(r, "ang_vel_z"))

        # ROS2PublishImu expects x,y,z,w. Isaac wrapper examples internally store
        # w,x,y,z, but the publisher input is passed through to sensor_msgs/Imu.
        if hasattr(r, "orientation_w"):
            q_xyzw = (_f(r, "orientation_x"), _f(r, "orientation_y"),
                      _f(r, "orientation_z"), _f(r, "orientation_w"))
        elif hasattr(r, "orientation"):
            o = r.orientation
            if hasattr(o, "GetReal"):                        # Gf.Quatd
                im = o.GetImaginary()
                q_xyzw = (float(im[0]), float(im[1]), float(im[2]), float(o.GetReal()))
            else:
                try:
                    vals = tuple(float(x) for x in o)       # Isaac reading: x,y,z,w
                    q_xyzw = vals if len(vals) == 4 else (0.0, 0.0, 0.0, 1.0)
                except Exception:
                    q_xyzw = (0.0, 0.0, 0.0, 1.0)
        else:
            q_xyzw = (0.0, 0.0, 0.0, 1.0)

        sensor_t = _f(r, "time", "sensor_time")
        try:
            sim_t = float(og.Controller.get(og.Controller.attribute(SIM_TIME_OUTPUT)))
        except Exception:
            sim_t = sensor_t

        # Print first few readings to confirm data format is correct.  ROS echo
        # should show approximately x=0, y=0.2588, z=0, w=0.9659 for orientation.
        _debug_counter[0] += 1
        if _debug_counter[0] <= 3:
            print(f"[IMU step {_debug_counter[0]}] lin_acc={lin_acc}  ang_vel={ang_vel}  "
                  f"orient_xyzw={q_xyzw}  sensor_t={sensor_t:.3f}  sim_t={sim_t:.3f}")

        try:
            og.Controller.set(og.Controller.attribute(_la), lin_acc)
            og.Controller.set(og.Controller.attribute(_av), ang_vel)
            og.Controller.set(og.Controller.attribute(_or), q_xyzw)
            og.Controller.set(og.Controller.attribute(_ts), sim_t)
        except Exception:
            pass   # OmniGraph not ready on very first steps — silently skip

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_step)
    print("  physics step reader subscribed → /livox/imu inputs updated each step")
    return sub


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    os.environ.setdefault("ROS_DOMAIN_ID", "20")
    os.environ.pop("CYCLONEDDS_URI", None)

    _enable_extension("isaacsim.sensors.physics")
    _enable_extension("isaacsim.ros2.bridge")

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(RIGID_BODY_PARENT).IsValid():
        raise RuntimeError(f"Parent prim not found: {RIGID_BODY_PARENT}")

    _cancel_old_sub()

    print("Removing stale IMU prims …")
    _remove_stale_imu_prims(stage)

    ok, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=IMU_NAME,
        parent=RIGID_BODY_PARENT,
        sensor_period=1.0 / SENSOR_HZ,
        translation=IMU_TRANSLATION,
        orientation=IMU_ORIENTATION,
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
    )
    if not ok or sensor is None:
        raise RuntimeError("IsaacSensorCreateImuSensor failed")
    print(f"IMU prim created: {IMU_PATH}")

    _make_publish_graph(stage)

    imu_iface = _acquire_imu_interface()
    sub = _subscribe_physics_reader(imu_iface)
    _save_sub(sub)

    print()
    print("IMU setup complete (run BEFORE Play).")
    print("Next steps:")
    print("  1. While still stopped: run cleanup_isaac_mid360_ros.py")
    print("  2. Press Play")
    print("  3. Check first IMU readings in console")
    print("  4. Run setup_isaac_mid360_like_ros.py exactly once")
    print("  5. ros2 topic hz /livox/imu --window 20")
    print("  6. ros2 topic echo /livox/imu --once")


main()
