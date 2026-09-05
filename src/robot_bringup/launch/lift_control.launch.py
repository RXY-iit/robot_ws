"""
lift_control.launch.py — Standalone launch for the vertical lift mechanism.

Starts:
  lift_serial_node  Serial bridge to Arduino (AZD-KD JOG control via GPIO).
  lift_joy_node     Maps hold LT → UP, hold RT → DOWN on /lift/command.

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
Then adjust up_axis / down_axis / trigger_active_below if needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351E062C1-if00',
            description='Stable by-id path for the verified lift Arduino UNO',
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
            'initial_position_mm', default_value='0.0',
            description='Unreferenced startup estimate [mm]',
        ),
        DeclareLaunchArgument(
            'initial_position_reference', default_value='UNREFERENCED_SOFTWARE_ESTIMATE',
            description='Evidence label; confirm endpoints with SET_MIN or SET_MAX',
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
            'enable_abzo_monitor', default_value='false',
            description='Start the read-only AZD-KD Modbus monitor. Requires a separate USB-RS485 path.',
        ),
        DeclareLaunchArgument(
            'abzo_serial_port', default_value='',
            description='Stable /dev/serial/by-id path for the isolated AZD-KD USB-RS485 adapter.',
        ),
        DeclareLaunchArgument(
            'abzo_slave_id', default_value='1',
            description='AZD-KD Modbus device address (1..31; address 0 is broadcast).',
        ),
        DeclareLaunchArgument(
            'abzo_baud_rate', default_value='115200',
            description='Must match the AZD-KD BAUD switch.',
        ),
        DeclareLaunchArgument(
            'abzo_position_mm_per_step', default_value='0.0',
            description='Signed mm/step calibration. Zero keeps the output truthfully raw/uncalibrated.',
        ),
        DeclareLaunchArgument(
            'abzo_position_offset_mm', default_value='0.0',
            description='Position offset [mm], applied only after mm/step is confirmed.',
        ),
        DeclareLaunchArgument(
            'abzo_physical_limits_configured', default_value='false',
            description='Set true only after FW-LS/RV-LS/HOMES wiring and MEXE02 assignment are verified.',
        ),
        DeclareLaunchArgument(
            'abzo_software_limits_configured', default_value='false',
            description='Set true only after AZD-KD software-limit values are verified in MEXE02.',
        ),
        DeclareLaunchArgument(
            'up_axis', default_value='2',
            description='Joy LT axis for UP (default 2; released +1, pressed toward -1).',
        ),
        DeclareLaunchArgument(
            'down_axis', default_value='5',
            description='Joy RT axis for DOWN (default 5; released +1, pressed toward -1).',
        ),
        DeclareLaunchArgument(
            'trigger_active_below', default_value='-0.5',
            description='A lift trigger is active only below this axis value.',
        ),
        DeclareLaunchArgument(
            'joy_timeout_sec', default_value='0.30',
            description='Send STOP if /joy disappears while lift motion is commanded.',
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
            'initial_position_mm': LaunchConfiguration('initial_position_mm'),
            'initial_position_reference': LaunchConfiguration(
                'initial_position_reference'),
            'auto_home_on_start': LaunchConfiguration('auto_home_on_start'),
            'enable_soft_limits': LaunchConfiguration('enable_soft_limits'),
            'expected_usb_vendor_id': '2341',
            'expected_usb_product_id': '0043',
            'require_usb_identity': True,
        }],
    )

    lift_joy = Node(
        package='serial_transciever',
        executable='lift_joy_node',
        name='lift_joy_node',
        output='screen',
        parameters=[{
            'up_axis': LaunchConfiguration('up_axis'),
            'down_axis': LaunchConfiguration('down_axis'),
            'trigger_active_below': LaunchConfiguration('trigger_active_below'),
            'joy_timeout_sec': LaunchConfiguration('joy_timeout_sec'),
        }],
    )

    lift_abzo_monitor = Node(
        package='serial_transciever',
        executable='lift_abzo_monitor_node',
        name='lift_abzo_monitor_node',
        condition=IfCondition(LaunchConfiguration('enable_abzo_monitor')),
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('abzo_serial_port'),
            'slave_id': LaunchConfiguration('abzo_slave_id'),
            'baud_rate': LaunchConfiguration('abzo_baud_rate'),
            'poll_rate_hz': 5.0,
            'position_mm_per_step': LaunchConfiguration('abzo_position_mm_per_step'),
            'position_offset_mm': LaunchConfiguration('abzo_position_offset_mm'),
            'physical_limit_inputs_configured': LaunchConfiguration('abzo_physical_limits_configured'),
            'software_limits_configured': LaunchConfiguration('abzo_software_limits_configured'),
        }],
    )

    return LaunchDescription(args + [lift_serial, lift_joy, lift_abzo_monitor])
