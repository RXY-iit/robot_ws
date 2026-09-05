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
  A (index 0): AUTO  — Nav2 cmd_vel relayed to motors by default
  B (index 1): MANUAL — Nav2 goal cancelled, robot stops by default
  Y (index 3): Emergency stop toggle (via cmd_vel_safety_node)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('nav_pkg'), 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_button = LaunchConfiguration('auto_button')
    manual_button = LaunchConfiguration('manual_button')
    mode_command_topic = LaunchConfiguration('mode_command_topic')
    joy_estop_button = LaunchConfiguration('joy_estop_button')
    enable_motion_policy = LaunchConfiguration('enable_motion_policy')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/nav2/cmd_vel')],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/nav2/cmd_vel')],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
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

    joy_source_arbiter = Node(
        package='nav_pkg',
        executable='joy_source_arbiter_node.py',
        name='joy_source_arbiter_node',
        output='screen',
    )

    nav_mode_switch = Node(
        package='nav_pkg',
        executable='nav_mode_switch_node.py',
        name='nav_mode_switch_node',
        output='screen',
        parameters=[{
            'auto_button': auto_button,
            'manual_button': manual_button,
            'manual_deadman_button': 4,
            'manual_teleop_timeout_sec': 0.25,
            'mode_command_topic': mode_command_topic,
            'enable_motion_policy': enable_motion_policy,
            'base_frame': 'base_footprint',
            'plan_topic': '/plan',
            'align_angle_rad': 0.50,
            'align_distance_m': 0.25,
            'finish_distance_m': 0.18,
            'finish_yaw_gate_rad': 0.45,
            'finish_max_vx': 0.08,
            'finish_max_wz': 0.25,
            'reverse_distance_m': 0.55,
            'reverse_lateral_window_m': 0.25,
            'near_back_distance_m': 1.50,
            'near_back_angle_rad': 1.0472,
            'near_back_lateral_scale': 0.10,
            'policy_lookahead_m': 1.2,
            'lateral_scale': 0.10,
            'lateral_to_yaw_gain': 1.4,
            'min_align_wz': 0.18,
            'max_align_wz': 0.55,
            'max_reverse_vx': 0.12,
            'reverse_yaw_gain': 1.0,
            'local_costmap_node': '/local_costmap/local_costmap',
            'normal_local_inflation_radius': 0.45,
            'narrow_local_inflation_radius': 0.33,
            'narrow_speed_hard_limit_mps': 0.09,
            'narrow_centerline_gain': 0.8,
        }],
    )

    # Safety layer: watchdog + speed clamp + emergency stop.
    # Subscribes /cmd_vel_raw (nav_mode_switch output), publishes /cmd_vel.
    cmd_vel_safety = Node(
        package='safety_layer',
        executable='cmd_vel_safety_node',
        name='cmd_vel_safety_node',
        output='screen',
        parameters=[{
            # Valid Joy updates showed gaps up to 0.186 s. L1 release is handled
            # immediately by nav_mode_switch; this is the publisher-failure guard.
            'watchdog_timeout': 0.30,
            'max_vx': 0.30,
            'max_vy': 0.20,
            'max_wz': 1.0,
            # Match MPPI's real-robot acceleration model. High jerk ceilings
            # prevent this final guard from adding a second slow ramp.
            'max_accel_vx': 0.50,
            'max_accel_vy': 0.12,
            'max_accel_wz': 0.45,
            'max_jerk_vx': 10.0,
            'max_jerk_vy': 10.0,
            'max_jerk_wz': 10.0,
            'joy_estop_button': joy_estop_button,
            'manual_bypass_slew': True,
            # Nav2/MPPI is ~10 Hz on the NUC.  Continue the slew-limited AUTO
            # output at motor-friendly frequency between controller updates.
            'auto_output_frequency': 50.0,
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
                    ('/camera_info', '/camera/camera/depth/camera_info'),
                    ('points', '/camera/depth/points'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_button', default_value='0'),
        DeclareLaunchArgument('manual_button', default_value='1'),
        DeclareLaunchArgument('mode_command_topic', default_value='/nav_mode/command'),
        DeclareLaunchArgument('joy_estop_button', default_value='3'),
        DeclareLaunchArgument('enable_motion_policy', default_value='true'),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,
        joy_source_arbiter,
        nav_mode_switch,
        cmd_vel_safety,
        depth_to_pointcloud,
    ])
