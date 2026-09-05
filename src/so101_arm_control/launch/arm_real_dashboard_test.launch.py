from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


CONDA_PYTHON = "/home/matsunaga-h/miniconda3/envs/lerobot/bin/python"
REAL_SERVER = (
    "/home/matsunaga-h/robot_ws/src/so101_arm_control/so101_arm_control/"
    "real_trajectory_server.py"
)
LEADER_READER = (
    "/home/matsunaga-h/robot_ws/src/so101_arm_control/so101_arm_control/"
    "leader_state_publisher.py"
)


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("rviz")
    use_dashboard = LaunchConfiguration("dashboard")
    use_robot_state_publishers = LaunchConfiguration("robot_state_publishers")
    use_leader = LaunchConfiguration("leader")
    shutdown_on_server_exit = LaunchConfiguration("shutdown_on_server_exit")
    dashboard_host = LaunchConfiguration("dashboard_host")
    dashboard_port = LaunchConfiguration("dashboard_port")
    visual_xacro = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "urdf", "so_arm101_visual.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so_arm101_description"), "rviz", "so101_mock.rviz"]
    )
    real_config = PathJoinSubstitution(
        [FindPackageShare("so101_arm_control"), "config", "arm_real_dashboard_test.yaml"]
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
                " prefix:=candidate_ joint_prefix:=candidate_ color:='1.0 0.45 0.08 1.0'",
            ]
        ),
        value_type=str,
    )

    real_server = ExecuteProcess(
        cmd=[CONDA_PYTHON, REAL_SERVER],
        name="so101_real_trajectory_server_process",
        output="both",
        additional_env={"PYTHONUNBUFFERED": "1"},
    )
    leader_reader = ExecuteProcess(
        cmd=[CONDA_PYTHON, LEADER_READER],
        name="so101_leader_state_publisher_process",
        output="both",
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=IfCondition(use_leader),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("dashboard", default_value="true"),
            DeclareLaunchArgument(
                "leader",
                default_value="true",
                description=(
                    "Start the read-only calibrated 5 V leader adapter. Missing leader "
                    "hardware leaves real2sim available and teleoperate blocked."
                ),
            ),
            DeclareLaunchArgument(
                "shutdown_on_server_exit",
                default_value="true",
                description=(
                    "Shut down the parent launch if the guarded hardware server exits. "
                    "Integrated Full bringup sets this false so navigation stays alive."
                ),
            ),
            DeclareLaunchArgument(
                "robot_state_publishers",
                default_value="true",
                description=(
                    "Publish standalone SO101 actual/candidate models. Set false when "
                    "embedded in mobile-robot bringup to avoid duplicate TF and "
                    "/robot_description publishers."
                ),
            ),
            DeclareLaunchArgument("dashboard_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("dashboard_port", default_value="8765"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="so101_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_robot_state_publishers),
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="arm/candidate",
                name="robot_state_publisher",
                parameters=[{"robot_description": candidate_description}],
                remappings=[("joint_states", "/arm/candidate_joint_states")],
                condition=IfCondition(use_robot_state_publishers),
                output="screen",
            ),
            leader_reader,
            real_server,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=real_server,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="guarded real trajectory server exited; stopping dashboard test"
                            )
                        )
                    ],
                ),
                condition=IfCondition(shutdown_on_server_exit),
            ),
            Node(
                package="so101_arm_control",
                executable="arm_gateway",
                name="so101_arm_gateway",
                parameters=[{"config_file": real_config}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="so101_real_dashboard_rviz",
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
