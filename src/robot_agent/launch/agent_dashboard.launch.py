import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_layout_path = os.path.join(
        get_package_share_directory("robot_agent"), "foxglove", "default-layout.json"
    )
    mode = LaunchConfiguration("mode")
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")
    map_id = LaunchConfiguration("map_id")
    robot_ws = LaunchConfiguration("robot_ws")
    foxglove_port = LaunchConfiguration("foxglove_port")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    enable_camera_compressed_relay = LaunchConfiguration("enable_camera_compressed_relay")
    camera_raw_topic = LaunchConfiguration("camera_raw_topic")
    enable_foxglove_ui = LaunchConfiguration("enable_foxglove_ui")
    foxglove_ui_port = LaunchConfiguration("foxglove_ui_port")
    foxglove_ui_container_name = LaunchConfiguration("foxglove_ui_container_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
            DeclareLaunchArgument("map_id", default_value="l402_2d_map_clean_0509"),
            DeclareLaunchArgument("robot_ws", default_value="/home/matsunaga-h/robot_ws"),
            # foxglove_bridge defaults to port 8765, which collides with the dashboard's
            # own HTTP/WebSocket port above -- keep it on a separate port.
            DeclareLaunchArgument("foxglove_port", default_value="8766"),
            DeclareLaunchArgument("enable_foxglove", default_value="false"),
            DeclareLaunchArgument("enable_camera_compressed_relay", default_value="false"),
            DeclareLaunchArgument(
                "camera_raw_topic", default_value="/camera/camera/color/image_raw"
            ),
            # Self-hosted Lichtblick (open-source Foxglove Studio continuation, MPL-2.0,
            # see todo-my/task-agent/dashboard-redesign/redesign.md Phase 0 for why) --
            # served locally via its own Docker image so the Dashboard's <iframe> can embed
            # it same-network without hitting the closed-source app.foxglove.dev, which
            # would likely block framing.
            DeclareLaunchArgument("enable_foxglove_ui", default_value="false"),
            DeclareLaunchArgument("foxglove_ui_port", default_value="8080"),
            DeclareLaunchArgument("foxglove_ui_container_name", default_value="robot_agent_lichtblick"),
            SetEnvironmentVariable("ROBOT_MAP_ID", map_id),
            SetEnvironmentVariable("ROBOT_WS", robot_ws),
            UnsetEnvironmentVariable("CYCLONEDDS_URI"),
            UnsetEnvironmentVariable("RMW_IMPLEMENTATION"),
            Node(
                package="robot_agent",
                executable="agent_node",
                name="agent_node",
                output="screen",
                parameters=[{"mode": mode}],
            ),
            Node(
                package="robot_agent",
                executable="dashboard_node",
                name="agent_dashboard_node",
                output="screen",
                parameters=[
                    {
                        "host": dashboard_host,
                        "port": dashboard_port,
                        "foxglove_bridge_port": foxglove_port,
                        "foxglove_ui_port": foxglove_ui_port,
                        "foxglove_ui_enabled": enable_foxglove_ui,
                    }
                ],
            ),
            # Real hardware only: publishes /agent/real_robot_state and handles the
            # dashboard's drive-motor restart control. Not part of the sim workspace.
            Node(
                package="robot_agent",
                executable="real_robot_monitor",
                name="real_robot_monitor",
                output="screen",
            ),
            # Read-only seven-domain evidence collector. Missing physical
            # instrumentation is reported explicitly instead of inferred healthy.
            Node(
                package="robot_agent",
                executable="health_monitor",
                name="health_monitor",
                output="screen",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{"port": foxglove_port}],
                condition=IfCondition(enable_foxglove),
            ),
            # Camera preview for Foxglove/Dashboard: republish the raw camera feed as a
            # compressed topic so remote viewers don't pull full raw bandwidth. Actual
            # frame-rate throttling happens on the subscriber side (Foxglove layout / panel
            # settings), this node only changes the encoding.
            Node(
                package="image_transport",
                executable="republish",
                name="dashboard_camera_compressed_relay",
                arguments=["raw", "compressed"],
                remappings=[
                    ("in", camera_raw_topic),
                    ("out/compressed", [camera_raw_topic, "/compressed"]),
                ],
                output="screen",
                condition=IfCondition(enable_camera_compressed_relay),
            ),
            ExecuteProcess(
                cmd=[
                    "docker",
                    "run",
                    "--rm",
                    "--name",
                    foxglove_ui_container_name,
                    "-p",
                    [foxglove_ui_port, ":8080"],
                    "-v",
                    f"{default_layout_path}:/lichtblick/default-layout.json:ro",
                    "ghcr.io/lichtblick-suite/lichtblick:latest",
                ],
                output="screen",
                condition=IfCondition(enable_foxglove_ui),
            ),
        ]
    )
