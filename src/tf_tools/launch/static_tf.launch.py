"""
Publish static transforms that are not covered by robot_state_publisher.
Goal TF tree:
  map → odom → base_link → livox_frame
                          → camera_link

NOTE: base_link → sensors are already declared in the URDF.
      This file handles any additional external static TFs if needed
      (e.g. map → odom for testing without a localization node).

Usage (testing only, not for production):
  ros2 launch tf_tools static_tf.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # map → odom (identity, for testing without a localization node)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # odom → base_link (identity, for testing without an odom node)
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    return LaunchDescription([
        map_to_odom,
        odom_to_base,
    ])
