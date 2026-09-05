from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("rviz")
    use_dashboard = LaunchConfiguration("dashboard")
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")

    visual_xacro = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "urdf", "so_arm101_visual.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "rviz", "so101_mock.rviz"]
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", visual_xacro]), value_type=str
    )
    candidate_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                visual_xacro,
                " prefix:=candidate_ color:='1.0 0.45 0.08 1.0'",
            ]
        ),
        value_type=str,
    )

    actual_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="so101_robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )
    candidate_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="arm/candidate",
        name="robot_state_publisher",
        parameters=[{"robot_description": candidate_description}],
        remappings=[("joint_states", "/arm/candidate_joint_states")],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("dashboard", default_value="true"),
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
            actual_state_publisher,
            candidate_state_publisher,
            Node(
                package="so101_arm_control",
                executable="mock_trajectory_server",
                name="so101_mock_trajectory_server",
                output="screen",
            ),
            Node(
                package="so101_arm_control",
                executable="arm_gateway",
                name="so101_arm_gateway",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="so101_rviz",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
            Node(
                package="robot_agent",
                executable="dashboard_node",
                name="agent_dashboard_node",
                parameters=[{"host": dashboard_host, "port": dashboard_port}],
                condition=IfCondition(use_dashboard),
                output="screen",
            ),
        ]
    )
