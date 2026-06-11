from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_sim'),
            'urdf',
            'robot_sim.urdf.xacro',
        ]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock for robot_state_publisher.',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str,
                ),
            }],
            output='screen',
        ),
    ])
