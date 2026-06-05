import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_joy = LaunchConfiguration("start_joy")
    direct_cmd_vel = LaunchConfiguration("direct_cmd_vel")
    start_rviz = LaunchConfiguration("start_rviz")
    joy_dev = LaunchConfiguration("joy_dev")

    share = get_package_share_directory("robot_sim")
    bringup_launch = os.path.join(share, "launch", "sim_manual_bringup.launch.py")
    rviz_config = os.path.join(share, "rviz", "sim_sensor_check.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_joy",
            default_value="true",
            description="Start Joy-Con/gamepad teleop chain.",
        ),
        DeclareLaunchArgument(
            "direct_cmd_vel",
            default_value="false",
            description="Use direct /sim/cmd_vel instead of the full mode/safety chain.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Open RViz with camera and LiDAR displays.",
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joy-Con/gamepad device path.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "start_joy": start_joy,
                "direct_cmd_vel": direct_cmd_vel,
                "start_pc2_adapter": "true",
                "joy_dev": joy_dev,
            }.items(),
        ),

        Node(
            package="robot_sim",
            executable="sim_sensor_check_node",
            name="sim_sensor_check_node",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(start_rviz),
            output="screen",
        ),
    ])
