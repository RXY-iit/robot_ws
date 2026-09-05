import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolved_arm_camera_node(context):
    """Start the identity-stable SO101 wrist camera capture endpoint."""
    configured_device = LaunchConfiguration('arm_camera_device').perform(context)
    resolved_device = os.path.realpath(configured_device)
    return [
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='arm_camera',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': resolved_device,
                'framerate': float(
                    LaunchConfiguration('arm_camera_framerate').perform(context)),
                'io_method': 'mmap',
                'frame_id': 'arm_camera_color_optical_frame',
                'pixel_format': 'mjpeg2rgb',
                'av_device_format': 'YUV422P',
                'image_width': int(
                    LaunchConfiguration('arm_camera_width').perform(context)),
                'image_height': int(
                    LaunchConfiguration('arm_camera_height').perform(context)),
                'camera_name': 'arm_camera',
                # Measured from the audited 2026-09-03 ChArUco dataset.
                'camera_info_url': (
                    'package://robot_bringup/config/arm_camera_calibrated.yaml'
                ),
                'brightness': -1,
                'contrast': -1,
                'saturation': -1,
                'sharpness': -1,
                'gain': -1,
                'auto_white_balance': True,
                'autoexposure': True,
                'autofocus': False,
            }],
            remappings=[
                ('image_raw', 'color/image_raw'),
                ('camera_info', 'color/camera_info'),
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory('camera_pan_tilt_calibration')

    args = [
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument(
            'arm_camera_device',
            default_value='/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0',
            description='Stable V4L2 path for the SO101 wrist camera.'),
        DeclareLaunchArgument('arm_camera_width', default_value='640'),
        DeclareLaunchArgument('arm_camera_height', default_value='480'),
        DeclareLaunchArgument('arm_camera_framerate', default_value='15.0'),
        DeclareLaunchArgument('output_root', description='Persistent source-tree output directory.'),
        DeclareLaunchArgument('squares_x', default_value='7'),
        DeclareLaunchArgument('squares_y', default_value='5'),
        DeclareLaunchArgument('square_length_m', default_value='0.0338'),
        DeclareLaunchArgument('marker_length_m', default_value='0.0165'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_100'),
        DeclareLaunchArgument('min_charuco_corners', default_value='8'),
        DeclareLaunchArgument('max_sensor_age_sec', default_value='0.50'),
        DeclareLaunchArgument('max_arm_hardware_age_sec', default_value='1.50'),
        DeclareLaunchArgument('stability_window_sec', default_value='0.80'),
        DeclareLaunchArgument('stability_tolerance_raw', default_value='0.15'),
        DeclareLaunchArgument('arm_stability_tolerance_rad', default_value='0.01'),
        DeclareLaunchArgument('direction_delta_raw', default_value='0.02'),
        DeclareLaunchArgument(
            'board_pose_reference', default_value='',
            description='Optional operator record describing how T_base_board was measured.'),
        DeclareLaunchArgument('board_base_x_m', default_value='nan'),
        DeclareLaunchArgument('board_base_y_m', default_value='nan'),
        DeclareLaunchArgument('board_base_z_m', default_value='nan'),
        DeclareLaunchArgument('board_base_roll_rad', default_value='nan'),
        DeclareLaunchArgument('board_base_pitch_rad', default_value='nan'),
        DeclareLaunchArgument('board_base_yaw_rad', default_value='nan'),
        DeclareLaunchArgument('camera_pan_axis', default_value='6'),
        DeclareLaunchArgument('camera_tilt_axis', default_value='7'),
        DeclareLaunchArgument('camera_pan_step_deg', default_value='2.0'),
        DeclareLaunchArgument('camera_tilt_step_deg', default_value='5.0'),
        DeclareLaunchArgument('camera_command_period_sec', default_value='0.05'),
        DeclareLaunchArgument('camera_initial_pan_angle', default_value='267.0'),
        DeclareLaunchArgument('camera_initial_tilt_angle', default_value='102.0'),
        DeclareLaunchArgument(
            'arm_joint_feedback', default_value='true',
            description=(
                'Start the deployed SO101 follower in torque-off real2sim mode so '
                'fresh measured /joint_states are mandatory calibration evidence.'
            ),
        ),
        DeclareLaunchArgument(
            'lift_physical_reference', default_value='PHYSICAL_LOWER_LIMIT',
            description='Operator-confirmed fixed lift reference for this session.'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py',
            )
        ),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'base_frame_id': 'camera_link',
            'publish_tf': 'true',
            # This tool exclusively owns the D435.  A device reset avoids
            # inheriting the intermittently observed "Depth stream start
            # failure" state from a previous camera process/session.
            'initial_reset': 'true',
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
            'diagnostics_period': '1.0',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
        }.items(),
    )

    arm_camera = OpaqueFunction(function=_resolved_arm_camera_node)

    arm_joint_feedback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('so101_arm_control'),
                'launch', 'arm_real_dashboard_test.launch.py',
            )
        ),
        condition=IfCondition(LaunchConfiguration('arm_joint_feedback')),
        launch_arguments={
            'rviz': 'false',
            'dashboard': 'false',
            'leader': 'false',
            'robot_state_publishers': 'true',
            'shutdown_on_server_exit': 'true',
        }.items(),
    )

    joy = Node(
        package='joy',
        executable='joy_node',
        name='camera_calibration_joy_node',
        parameters=[{'dev': LaunchConfiguration('joy_dev')}],
        # Deliberately publish canonical /joy directly: this launch starts no
        # base teleop consumer, cmd_vel pipeline, Nav2, drive or lift nodes.
        remappings=[('joy', '/joy')],
        output='screen',
    )

    serial_motor = Node(
        package='serial_transciever',
        executable='chokudo_cameraswing_air_serial_node',
        name='camera_calibration_pan_tilt_serial',
        parameters=[{'log_only_on_change': True, 'angle_change_threshold': 0.1}],
        output='screen',
    )

    joy_motor = Node(
        package='serial_transciever',
        executable='camera_motor_joy_node',
        name='camera_calibration_pan_tilt_joy',
        parameters=[{
            'enable_button': 4,
            'pan_axis': ParameterValue(LaunchConfiguration('camera_pan_axis'), value_type=int),
            'tilt_axis': ParameterValue(LaunchConfiguration('camera_tilt_axis'), value_type=int),
            'pan_sign': 1.0,
            'tilt_sign': -1.0,
            'pan_step_deg': ParameterValue(
                LaunchConfiguration('camera_pan_step_deg'), value_type=float),
            'tilt_step_deg': ParameterValue(
                LaunchConfiguration('camera_tilt_step_deg'), value_type=float),
            'command_period_sec': ParameterValue(
                LaunchConfiguration('camera_command_period_sec'), value_type=float),
            'initial_pan_angle': ParameterValue(
                LaunchConfiguration('camera_initial_pan_angle'), value_type=float),
            'initial_tilt_angle': ParameterValue(
                LaunchConfiguration('camera_initial_tilt_angle'), value_type=float),
        }],
        output='screen',
    )

    capture = Node(
        package='camera_pan_tilt_calibration',
        executable='capture_node.py',
        name='camera_pan_tilt_calibration_capture',
        parameters=[{
            'output_root': LaunchConfiguration('output_root'),
            'squares_x': ParameterValue(LaunchConfiguration('squares_x'), value_type=int),
            'squares_y': ParameterValue(LaunchConfiguration('squares_y'), value_type=int),
            'square_length_m': ParameterValue(
                LaunchConfiguration('square_length_m'), value_type=float),
            'marker_length_m': ParameterValue(
                LaunchConfiguration('marker_length_m'), value_type=float),
            'dictionary': LaunchConfiguration('dictionary'),
            'min_charuco_corners': ParameterValue(
                LaunchConfiguration('min_charuco_corners'), value_type=int),
            'max_sensor_age_sec': ParameterValue(
                LaunchConfiguration('max_sensor_age_sec'), value_type=float),
            'max_arm_hardware_age_sec': ParameterValue(
                LaunchConfiguration('max_arm_hardware_age_sec'), value_type=float),
            'stability_window_sec': ParameterValue(
                LaunchConfiguration('stability_window_sec'), value_type=float),
            'stability_tolerance_raw': ParameterValue(
                LaunchConfiguration('stability_tolerance_raw'), value_type=float),
            'arm_stability_tolerance_rad': ParameterValue(
                LaunchConfiguration('arm_stability_tolerance_rad'), value_type=float),
            'direction_delta_raw': ParameterValue(
                LaunchConfiguration('direction_delta_raw'), value_type=float),
            'require_arm_joint_states': ParameterValue(
                LaunchConfiguration('arm_joint_feedback'), value_type=bool),
            'lift_physical_reference': LaunchConfiguration('lift_physical_reference'),
            'board_pose_reference': LaunchConfiguration('board_pose_reference'),
            'board_base_x_m': ParameterValue(
                LaunchConfiguration('board_base_x_m'), value_type=float),
            'board_base_y_m': ParameterValue(
                LaunchConfiguration('board_base_y_m'), value_type=float),
            'board_base_z_m': ParameterValue(
                LaunchConfiguration('board_base_z_m'), value_type=float),
            'board_base_roll_rad': ParameterValue(
                LaunchConfiguration('board_base_roll_rad'), value_type=float),
            'board_base_pitch_rad': ParameterValue(
                LaunchConfiguration('board_base_pitch_rad'), value_type=float),
            'board_base_yaw_rad': ParameterValue(
                LaunchConfiguration('board_base_yaw_rad'), value_type=float),
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='camera_pan_tilt_calibration_rviz',
        arguments=['-d', os.path.join(package_share, 'config', 'calibration.rviz')],
        output='screen',
    )

    return LaunchDescription(
        args + [
            camera, arm_camera, arm_joint_feedback, joy, serial_motor,
            joy_motor, capture, rviz,
        ]
    )
