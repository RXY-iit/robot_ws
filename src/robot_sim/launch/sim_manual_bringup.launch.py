import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    start_joy = LaunchConfiguration('start_joy')
    start_pc2_adapter = LaunchConfiguration('start_pc2_adapter')
    direct_cmd_vel = LaunchConfiguration('direct_cmd_vel')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_sim'),
            'urdf',
            'robot_sim.urdf.xacro',
        ]),
    ])

    joy_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'joy_teleop.yaml',
    )

    sim_drive_config = os.path.join(
        get_package_share_directory('robot_sim'),
        'config',
        'sim_omni_drive.yaml',
    )

    adapter_config = os.path.join(
        get_package_share_directory('robot_sim'),
        'config',
        'livox_pc2_adapter.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joy-Con/gamepad device path.',
        ),
        DeclareLaunchArgument(
            'start_joy',
            default_value='true',
            description='Start joy_node and teleop_twist_joy.',
        ),
        DeclareLaunchArgument(
            'start_pc2_adapter',
            default_value='false',
            description='Start PointCloud2 -> Livox CustomMsg adapter.',
        ),
        DeclareLaunchArgument(
            'direct_cmd_vel',
            default_value='false',
            description=(
                'Subscribe sim drive directly to /sim/cmd_vel and skip the '
                'mode/safety /cmd_vel chain. Use for isolated Isaac motion tests.'
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str,
                ),
            }],
            output='screen',
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': joy_dev}],
            condition=IfCondition(start_joy),
            output='screen',
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_config],
            remappings=[('cmd_vel', '/teleop/cmd_vel')],
            condition=IfCondition(PythonExpression([
                "'", start_joy, "' == 'true' and '", direct_cmd_vel, "' != 'true'",
            ])),
            output='screen',
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_config],
            remappings=[('cmd_vel', '/sim/cmd_vel')],
            condition=IfCondition(PythonExpression([
                "'", start_joy, "' == 'true' and '", direct_cmd_vel, "' == 'true'",
            ])),
            output='screen',
        ),

        Node(
            package='nav_pkg',
            executable='nav_mode_switch_node.py',
            name='nav_mode_switch_node',
            condition=UnlessCondition(direct_cmd_vel),
            output='screen',
        ),

        Node(
            package='safety_layer',
            executable='cmd_vel_safety_node',
            name='cmd_vel_safety_node',
            condition=UnlessCondition(direct_cmd_vel),
            output='screen',
        ),

        Node(
            package='robot_sim',
            executable='sim_omni_drive_command_node',
            name='sim_omni_drive_command_node',
            parameters=[
                sim_drive_config,
                {'cmd_vel_topic': '/sim/cmd_vel'},
            ],
            condition=IfCondition(direct_cmd_vel),
            output='screen',
        ),

        Node(
            package='robot_sim',
            executable='sim_omni_drive_command_node',
            name='sim_omni_drive_command_node',
            parameters=[sim_drive_config],
            condition=UnlessCondition(direct_cmd_vel),
            output='screen',
        ),

        Node(
            package='robot_sim',
            executable='pointcloud2_to_livox_custom_msg_node',
            name='pointcloud2_to_livox_custom_msg_node',
            parameters=[adapter_config],
            condition=IfCondition(start_pc2_adapter),
            output='screen',
        ),
    ])
