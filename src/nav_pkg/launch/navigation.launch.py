"""
navigation.launch.py — Nav2 + safety_layer + mode-switch.

Node chain:
  controller_server / behavior_server  →  /nav2/cmd_vel
  nav_mode_switch_node  →  /cmd_vel_raw
  cmd_vel_safety_node   →  /cmd_vel  →  omni_base_driver

Sensor architecture:
  Livox Mid360  → localization only (FAST-LIO2 + GICP).
  RealSense D435 depth → /camera/depth/points → VoxelLayer local costmap.

Controllers:
  FollowPath    : MPPI (primary, holonomic, uses linear_y)
  FollowPathRPP : RegulatedPurePursuit (fallback)

Joy-Con buttons:
  A (index 0): AUTO  — Nav2 cmd_vel relayed to motors
  B (index 1): MANUAL — Nav2 goal cancelled, robot stops
  Y (index 3): Emergency stop toggle (via cmd_vel_safety_node)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
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
        remappings=[('cmd_vel', '/nav2/cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
        remappings=[('cmd_vel', '/nav2/cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': False}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
            'bond_timeout': 4.0,
        }],
    )

    nav_mode_switch = Node(
        package='nav_pkg',
        executable='nav_mode_switch_node.py',
        name='nav_mode_switch_node',
        output='screen',
    )

    # Safety layer: watchdog + speed clamp + emergency stop.
    # Subscribes /cmd_vel_raw (nav_mode_switch output), publishes /cmd_vel.
    cmd_vel_safety = Node(
        package='safety_layer',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        output='screen',
        parameters=[{
            'watchdog_timeout': 0.5,
            'max_vx': 0.15,
            'max_vy': 0.10,
            'max_wz': 0.5,
        }],
    )

    # depth_image_proc: RealSense depth → PointCloud2 for VoxelLayer.
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
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        nav_mode_switch,
        cmd_vel_safety,
        depth_to_pointcloud,
    ])
