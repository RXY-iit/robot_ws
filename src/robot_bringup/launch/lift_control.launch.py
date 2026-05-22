"""
lift_control.launch.py — Standalone launch for the vertical lift mechanism.

Starts:
  lift_serial_node  Serial bridge to Arduino (AZD-KD JOG control via GPIO).
  lift_joy_node     Maps Joy-Con L1+Y → UP, L1+A → DOWN on /lift/command.

The joy_node (publishing /joy) must already be running — either from
teleop.launch.py or test_all.launch.py with joy:=true.

Usage:
  # With full bringup already running:
  ros2 launch robot_bringup lift_control.launch.py

  # Override serial port or range:
  ros2 launch robot_bringup lift_control.launch.py \\
    serial_port:=/dev/ttyACM1 \\
    position_max_mm:=300.0

  # Disable auto-homing (start from assumed position 0):
  ros2 launch robot_bringup lift_control.launch.py auto_home_on_start:=false

Key topics:
  /lift/command   (std_msgs/String)   "UP" | "DOWN" | "STOP" | "HOME"
  /lift/position  (std_msgs/Float32)  estimated position [mm]
  /lift/state     (std_msgs/String)   current motion state

Confirm Joy-Con button mapping with:
  ros2 topic echo /joy
Then adjust up_button / down_button args if needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyACM0',
            description='Serial port for the Arduino (e.g. /dev/ttyACM0 or /dev/ttyACM1)',
        ),
        DeclareLaunchArgument(
            'position_min_mm', default_value='0.0',
            description='Software lower limit [mm]. Commands reaching this limit are stopped.',
        ),
        DeclareLaunchArgument(
            'position_max_mm', default_value='200.0',
            description='Software upper limit [mm]. Commands reaching this limit are stopped.',
        ),
        DeclareLaunchArgument(
            'home_position_mm', default_value='100.0',
            description='Position value assigned after homing completes [mm].',
        ),
        DeclareLaunchArgument(
            'auto_home_on_start', default_value='false',
            description='If true, perform homing automatically 2.5 s after node startup.',
        ),
        DeclareLaunchArgument(
            'enable_soft_limits', default_value='false',
            description='If true, enforce position_min/max_mm limits in software.',
        ),
        DeclareLaunchArgument(
            'jog_speed_mm_s', default_value='60.0',
            description='AZD-KD JOG speed [mm/s]. Must match driver parameter 21.',
        ),
        DeclareLaunchArgument(
            'enable_button', default_value='4',
            description='Joy-Con dead-man button index (default 4 = L1).',
        ),
        DeclareLaunchArgument(
            'up_button', default_value='3',
            description='Joy-Con UP button index (default 3 = Y). Verify with: ros2 topic echo /joy',
        ),
        DeclareLaunchArgument(
            'down_button', default_value='0',
            description='Joy-Con DOWN button index (default 0 = A). Verify with: ros2 topic echo /joy',
        ),
    ]

    lift_serial = Node(
        package='serial_transciever',
        executable='lift_serial_node',
        name='lift_serial_node',
        output='screen',
        parameters=[{
            'serial_port':        LaunchConfiguration('serial_port'),
            'baud_rate':          9600,
            'jog_speed_mm_s':     LaunchConfiguration('jog_speed_mm_s'),
            'position_min_mm':    LaunchConfiguration('position_min_mm'),
            'position_max_mm':    LaunchConfiguration('position_max_mm'),
            'home_position_mm':   LaunchConfiguration('home_position_mm'),
            'auto_home_on_start': LaunchConfiguration('auto_home_on_start'),
            'enable_soft_limits': LaunchConfiguration('enable_soft_limits'),
        }],
    )

    lift_joy = Node(
        package='serial_transciever',
        executable='lift_joy_node',
        name='lift_joy_node',
        output='screen',
        parameters=[{
            'enable_button': LaunchConfiguration('enable_button'),
            'up_button':     LaunchConfiguration('up_button'),
            'down_button':   LaunchConfiguration('down_button'),
        }],
    )

    return LaunchDescription(args + [lift_serial, lift_joy])
