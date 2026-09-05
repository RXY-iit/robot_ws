from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


CONDA_PYTHON = "/home/matsunaga-h/miniconda3/envs/lerobot/bin/python"
READ_ONLY_NODE = (
    "/home/matsunaga-h/robot_ws/src/so101_arm_control/so101_arm_control/"
    "real_read_only_joint_state_publisher.py"
)


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("rviz")
    visual_xacro = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "urdf", "so_arm101_visual.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "rviz", "so101_real_read_only.rviz"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", visual_xacro]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="so101_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    CONDA_PYTHON,
                    READ_ONLY_NODE,
                    "--ack-read-only",
                    "--rate-hz",
                    "5",
                    "--max-outside-ticks",
                    "5",
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="so101_real_read_only_rviz",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
