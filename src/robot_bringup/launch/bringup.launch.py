"""
Master bringup launch — starts the full mobile robot system:
  - robot_state_publisher  (URDF → TF tree: base_link → livox_frame, camera_link)
  - Livox MID360 LiDAR driver
  - RealSense D435 camera driver (realsense2_camera)

Usage:
  # Normal operation (GICP localization, xfer_format=0 PointCloud2)
  ros2 launch robot_bringup bringup.launch.py

  # With FAST-LIO2 (switches Livox to xfer_format=1 CustomMsg)
  ros2 launch robot_bringup bringup.launch.py fast_lio_mode:=true

Launch arguments:
  fast_lio_mode    Switch Livox driver to xfer_format=1 (CustomMsg) required by
                   FAST-LIO2.  The livox_custom_to_pc2_node relay (started by
                   fast_lio_localization.launch.py with_fast_lio:=true) converts
                   CustomMsg → PointCloud2 for GICP and pointcloud_to_laserscan.
                   Default: false  (xfer_format=0 PointCloud2, normal GICP mode)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    fast_lio_mode = LaunchConfiguration('fast_lio_mode', default='false')

    # ---- robot_state_publisher (TF from URDF) ----
    robot_desc_pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---- Livox MID360 LiDAR ----
    # Shared parameters except xfer_format
    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json',
    )
    livox_common_params = [
        {'multi_topic': 0},
        {'data_src': 0},
        {'publish_freq': 10.0},
        {'output_data_type': 0},
        {'frame_id': 'livox_frame'},
        {'lvx_file_path': '/home/livox/livox_test.lvx'},
        {'user_config_path': livox_config_path},
        {'cmdline_input_bd_code': 'livox0000000001'},
    ]

    # xfer_format=0: PointCloud2 — used by GICP localization (default)
    livox_pc2_group = GroupAction(
        condition=UnlessCondition(fast_lio_mode),
        actions=[
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[{'xfer_format': 0}] + livox_common_params,
            ),
        ],
    )

    # xfer_format=1: CustomMsg — required by FAST-LIO2
    # The livox_custom_to_pc2 relay node (in fast_lio_localization.launch.py)
    # converts CustomMsg → PointCloud2 for GICP and pointcloud_to_laserscan.
    livox_custommsg_group = GroupAction(
        condition=IfCondition(fast_lio_mode),
        actions=[
            Node(
                package='livox_ros_driver2',
                executable='livox_ros_driver2_node',
                name='livox_lidar_publisher',
                output='screen',
                parameters=[{'xfer_format': 1}] + livox_common_params,
            ),
        ],
    )

    # ---- RealSense D435 camera ----
    # Publishes: /camera/camera/color/image_raw, /camera/camera/depth/image_rect_raw
    # TF chain: camera_link → camera_color_frame → camera_color_optical_frame (auto)
    # base_frame_id=camera_link ties driver TF to our URDF link
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py',
            )
        ),
        launch_arguments={
            'camera_name':      'camera',
            'camera_namespace': 'camera',
            'base_frame_id':    'camera_link',
            'publish_tf':       'true',
            'enable_depth':     'true',
            'enable_color':     'true',
            'enable_infra1':    'false',
            'enable_infra2':    'false',
            'rgb_camera.color_profile': '640,480,30',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'fast_lio_mode', default_value='false',
            description='true = Livox xfer_format=1 (CustomMsg for FAST-LIO2); '
                        'false = xfer_format=0 (PointCloud2 for GICP only)',
        ),
        rsp_node,
        livox_pc2_group,
        livox_custommsg_group,
        camera_launch,
    ])
