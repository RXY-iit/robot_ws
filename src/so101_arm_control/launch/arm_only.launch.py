from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")
    arm_launch = os.path.join(
        get_package_share_directory("so101_arm_control"),
        "launch",
        "arm_real_dashboard_test.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(arm_launch),
                launch_arguments={
                    "rviz": "true",
                    "dashboard": "true",
                    "robot_state_publishers": "true",
                    "dashboard_host": dashboard_host,
                    "dashboard_port": dashboard_port,
                }.items(),
            ),
            # Arm-only still exposes the same read-only motor evidence on
            # /health, without starting Nav2, sensors, the mobile base, or the
            # full agent runtime.
            Node(
                package="robot_agent",
                executable="health_monitor",
                name="health_monitor",
                output="screen",
            ),
        ]
    )
