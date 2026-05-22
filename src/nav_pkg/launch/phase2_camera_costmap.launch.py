"""
Phase 2 camera local-costmap verification launch.

Starts only the Nav2 pieces needed to test:

  RealSense depth image
    -> depth_image_proc::PointCloudXyzNode
    -> /camera/depth/points
    -> local_costmap VoxelLayer

This launch intentionally does not start planner_server, behavior_server,
bt_navigator, nav_mode_switch_node, or any motor command relay.  Use it with
robot_bringup test_all.launch.py in sensor-only mode:

  ros2 launch robot_bringup test_all.launch.py \\
    lidar:=false camera:=true drive_motors:=false steer_motors:=false \\
    serial_motors:=false joy:=false camera_motor_joy:=false lift:=false \\
    rviz:=false static_odom:=true use_glim_loc:=false

Then:

  ros2 launch nav_pkg phase2_camera_costmap.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('nav_pkg'), 'config', 'nav2_params.yaml')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[('cmd_vel', '/phase2_unused_cmd_vel')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_phase2_camera_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server'],
            'bond_timeout': 4.0,
        }],
    )

    depth_to_pointcloud = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                remappings=[
                    ('image_rect', '/camera/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/camera/depth/camera_info'),
                    ('points', '/camera/depth/points'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        depth_to_pointcloud,
        controller_server,
        lifecycle_manager,
    ])
