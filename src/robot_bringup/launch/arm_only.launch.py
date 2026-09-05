"""SO101 real-hardware bringup with the complete robot URDF, but no mobile hardware.

This launch reuses test_all.launch.py so Arm Only and Full share exactly the same
base_link -> Arm actual/preview mounting transforms.  Every non-Arm hardware
group is explicitly disabled.
"""

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

    integrated_bringup = os.path.join(
        get_package_share_directory("robot_bringup"),
        "launch",
        "test_all.launch.py",
    )
    rviz_config = os.path.join(
        get_package_share_directory("so_arm101_description"),
        "rviz",
        "so101_robot_mount.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(integrated_bringup),
                launch_arguments={
                    "lidar": "false",
                    "camera": "false",
                    "arm_camera": "true",
                    "drive_motors": "false",
                    "steer_motors": "false",
                    "serial_motors": "false",
                    "joy": "false",
                    "cmd_vel_pipeline": "false",
                    "camera_motor_joy": "false",
                    "lift": "false",
                    "lift_abzo_monitor": "false",
                    "arm": "true",
                    "arm_shutdown_on_server_exit": "true",
                    "rviz": "false",
                    "static_odom": "true",
                    "use_glim_loc": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="so101_robot_mount_rviz",
                arguments=["-d", rviz_config],
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="dashboard_node",
                name="agent_dashboard_node",
                parameters=[{"host": dashboard_host, "port": dashboard_port}],
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="health_monitor",
                name="health_monitor",
                output="screen",
            ),
        ]
    )
