from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("touch_anything"), "config", "touch_anything.yaml"]
    )
    return LaunchDescription([
        Node(
            package="touch_anything",
            executable="touch_observation_node.py",
            name="touch_observation_node",
            parameters=[config],
            output="screen",
        )
    ])
