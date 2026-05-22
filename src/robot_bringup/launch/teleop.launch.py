"""
teleop.launch.py — Joy-Con / gamepad teleoperation for the omni-drive base.

Starts:
  joy_node              reads /dev/input/jsX → publishes /joy
  teleop_twist_joy_node converts /joy → /teleop/cmd_vel  (dead-man: hold L1/LB)

nav_mode_switch_node relays /teleop/cmd_vel to /cmd_vel only in MANUAL mode.
This prevents joystick commands from bypassing AUTO navigation authority.

Usage:
  # Terminal 1 — full hardware bringup (motors must be running to actually move)
  ros2 launch robot_bringup test_all.launch.py

  # Terminal 2 — start teleop
  ros2 launch robot_bringup teleop.launch.py

  # Verify
  ros2 topic echo /joy       # Joy-Con input
  ros2 topic echo /teleop/cmd_vel   # teleop velocity command

Optional args:
  joy_dev:=/dev/input/js0   joystick device (default js0)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')

    config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config', 'joy_teleop.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path (e.g. /dev/input/js0 or js1)',
        ),

        # joy_node: reads raw joystick input and publishes sensor_msgs/Joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev': joy_dev}],
            output='screen',
        ),

        # teleop_twist_joy: converts Joy → Twist and publishes to /teleop/cmd_vel.
        # nav_mode_switch_node is the only relay to the real /cmd_vel topic.
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config],
            remappings=[('cmd_vel', '/teleop/cmd_vel')],
            output='screen',
        ),
    ])
