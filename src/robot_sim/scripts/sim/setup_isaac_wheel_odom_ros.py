"""Create Isaac physics odometry publishers for robot_sim.

Run from Isaac Sim Script Editor after pressing Play:

exec(open("/home/ruan-x/robot_ws/src/robot_sim/scripts/sim/setup_isaac_wheel_odom_ros.py").read())

Publishes:
  /wheel_odom  nav_msgs/Odometry, frame_id=odom, child_frame_id=base_footprint
  /tf          odom -> base_footprint
"""

import omni
import omni.graph.core as og
import omni.timeline
from pxr import Sdf


ROBOT_CHASSIS = "/World/robot_sim/base_footprint"
GRAPH_PATH = "/World/robot_sim/wheel_odom_ros_graph"
SIM_TIME_OUTPUT = "/World/ROS2_ActionGraph/read_sim_time.outputs:simulationTime"
ROS_CONTEXT_OUTPUT = "/World/ROS2_ActionGraph/ros2_context.outputs:context"


def main():
    stage = omni.usd.get_context().get_stage()
    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Press Play before creating /wheel_odom publishers.")

    chassis = stage.GetPrimAtPath(ROBOT_CHASSIS)
    if not chassis or not chassis.IsValid():
        raise RuntimeError(f"Robot chassis prim not found: {ROBOT_CHASSIS}")

    existing = stage.GetPrimAtPath(GRAPH_PATH)
    if existing and existing.IsValid():
        stage.RemovePrim(GRAPH_PATH)
        print(f"Removed old wheel odom graph: {GRAPH_PATH}")

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishWheelOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [Sdf.Path(ROBOT_CHASSIS)]),
                ("PublishWheelOdom.inputs:topicName", "wheel_odom"),
                ("PublishWheelOdom.inputs:odomFrameId", "odom"),
                ("PublishWheelOdom.inputs:chassisFrameId", "base_footprint"),
                ("PublishWheelOdom.inputs:publishRawVelocities", False),
                ("PublishOdomTF.inputs:topicName", "tf"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishWheelOdom.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishWheelOdom.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishWheelOdom.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishWheelOdom.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishWheelOdom.inputs:angularVelocity"),
                ("ReadSimTime.outputs:simulationTime", "PublishWheelOdom.inputs:timeStamp"),
                ("ROS2Context.outputs:context", "PublishWheelOdom.inputs:context"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                ("ROS2Context.outputs:context", "PublishOdomTF.inputs:context"),
            ],
        },
    )

    print("Created /wheel_odom and /tf odom->base_footprint publishers.")


main()
