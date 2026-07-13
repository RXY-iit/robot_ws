from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
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
                parameters=[{"host": dashboard_host, "port": dashboard_port}],
            ),
        ]
    )
