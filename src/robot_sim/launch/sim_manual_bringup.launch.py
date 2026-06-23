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
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_arm = LaunchConfiguration('start_arm')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_sim'),
            'urdf',
            'robot_sim.urdf.xacro',
        ]),
    ])

    joy_config = os.path.join(
        get_package_share_directory('robot_sim'),
        'config',
        'sim_joy_teleop.yaml',
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Isaac /clock for sim bringup helper nodes.',
        ),
        DeclareLaunchArgument(
            'start_arm',
            default_value='true',
            description=(
                'Start lift_joy_node, camera_motor_joy_node, and sim_lift_command_node '
                'to control lift_joint + camera_swing_joint in Isaac Sim via Joy-Con. '
                'Uses same /lift/command and /cameraswingmotor/target_angle topics as real robot.'
            ),
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
            parameters=[{
                # Sim uses L1+Y for lift up, so keep emergency stop on topic only.
                'joy_estop_button': -1,
                'max_vx': 0.30,
                'max_vy': 0.20,
                'max_wz': 1.0,
            }],
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
            parameters=[adapter_config, {'use_sim_time': use_sim_time}],
            condition=IfCondition(start_pc2_adapter),
            output='screen',
        ),

        # --- Arm control: lift + camera swing ---
        # lift_joy_node and camera_motor_joy_node are reused from real-robot stack
        # (serial_transciever package) — no hardware dependency, they only talk ROS2.
        # sim_lift_command_node bridges their output → /sim/arm_joint_states → Isaac Sim.
        Node(
            package='serial_transciever',
            executable='lift_joy_node',
            name='lift_joy_node',
            parameters=[{
                'enable_button': 4,   # L1 — same dead-man as real robot
                'up_button':     3,   # Y
                'down_button':   0,   # A
            }],
            condition=IfCondition(PythonExpression([
                "'", start_joy, "' == 'true' and '", start_arm, "' == 'true'",
            ])),
            output='screen',
        ),

        Node(
            package='serial_transciever',
            executable='camera_motor_joy_node',
            name='camera_motor_joy_node',
            parameters=[{
                'enable_button':          4,
                'pan_axis':               6,
                'tilt_axis':              7,
                'pan_sign':               1.0,
                'tilt_sign':              -1.0,
                'pan_step_deg':           5.0,
                'tilt_step_deg':          5.0,
                'command_period_sec':     0.05,
                'initial_tilt_angle':     67.0,   # Dynamixel neutral = horizontal forward
                'initial_pan_angle':      0.0,    # sim pan neutral = forward
            }],
            condition=IfCondition(PythonExpression([
                "'", start_joy, "' == 'true' and '", start_arm, "' == 'true'",
            ])),
            output='screen',
        ),

        Node(
            package='robot_sim',
            executable='sim_lift_command_node',
            name='sim_lift_command_node',
            parameters=[{
                'publish_rate':       30.0,
                'lift_speed_m_s':     0.06,
                'lift_lower_m':        0.0,
                'lift_upper_m':        0.3,
                'camera_pan_neutral_deg': 0.0,
                'camera_pan_lower_rad':  -1.5708,  # -π/2
                'camera_pan_upper_rad':   1.5708,  # +π/2
                'camera_neutral_deg': 67.0,
                'camera_down_offset_deg': 42.0,
                'camera_lower_rad':    -1.57080,  # -90°
                'camera_upper_rad':    2.30383,   # 132°
                'robot_cmd_vel_topic': '/cmd_vel',
                'robot_mode_topic':    '/robot_mode',
                'stop_linear_threshold': 0.01,
                'stop_angular_threshold': 0.02,
                'cmd_vel_timeout_sec': 0.5,
            }],
            condition=IfCondition(start_arm),
            output='screen',
        ),
    ])
