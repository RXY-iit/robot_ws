"""
test_all.launch.py  — Full hardware test launch

Starts all sensors and motors and opens RViz.

Launch arguments (all default true — set false to skip hardware not connected):
  lidar:=true/false          Livox MID360
  fast_lio_mode:=true/false  Switch Livox to xfer_format=1 (CustomMsg) for FAST-LIO2.
                             Must also pass with_fast_lio:=true to localization launch.
                             Default: false (xfer_format=0 PointCloud2, GICP-only mode)
  camera:=true/false         RealSense D435
  arm_camera:=true/false     SO101 wrist USB camera (independent /arm_camera namespace)
  drive_motors:=true/false   BLV-R x3 via Modbus (om_modbus_master + drive_motor.py)
  steer_motors:=true/false   Dynamixel x3 (read_write_node + steer/cmd/odom nodes)
  serial_motors:=true/false  Linear + camera-swing via OpenRB-150 serial
  joy:=true/false            Joy-Con / gamepad input and /cmd_vel teleop
  cmd_vel_pipeline:=true/false
                             Launch MANUAL/AUTO mode switch + safety relay.
                             Keep true for standalone teleop. Set false when
                             another launcher starts these nodes.
  camera_motor_joy:=true/false  Joy-Con control for camera pan/tilt motors
  arm:=true/false            Guarded SO101 follower server + Dashboard gateway
  rviz:=true/false           RViz2 window

Example — skip serial motors if board not connected:
  ros2 launch robot_bringup test_all.launch.py serial_motors:=false

Example — start in FAST-LIO mode (also pass with_fast_lio:=true to localization):
  ros2 launch robot_bringup test_all.launch.py fast_lio_mode:=true

Node / topic map
  robot_state_publisher     → /robot_description, TF: base_link→livox_frame, camera_link
  static_transform_publisher→ TF: map→odom  (identity, test only — no localization node yet)
  livox_lidar_publisher     → /livox/lidar, /livox/imu
  realsense2_camera         → /camera/camera/color/image_raw, /camera/camera/depth/…
  om_modbusRTU_node         → /om_query0, /om_response0, /om_state0  (Modbus layer)
  drive_motor               → /drive_odom (DriveMotor)
  read_write_node           → service: get_position, sub: set_position
  steer_motor_node          → /steer_odom (SteerMotor)
  cmd_vel_to_motor_node     → sub: /cmd_vel  pub: /steer_ang, /drive_vel
  robot_odom_node           → /wheel_odom (Odometry), TF: odom→base_footprint
  joy_node + teleop_twist_joy_node → /joy, /teleop/cmd_vel
  nav_mode_switch_node       → /teleop/cmd_vel or /nav2/cmd_vel → /cmd_vel_raw
  cmd_vel_safety_node        → /cmd_vel_raw → /cmd_vel
  chokudo_cameraswing…      → /chokudomotor/angle, /cameraswingmotor/angle
  camera_motor_joy_node     → /joy → /chokudomotor/target_angle, /cameraswingmotor/target_angle
  rviz2                     → visualization
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolved_arm_camera_node(context):
    """Resolve the stable V4L by-id link for usb_cam.

    usb_cam in ROS 2 Humble rejects relative V4L symlink targets such as
    ``../../video6``. Resolve the identity-stable link at launch time while
    still giving the driver the canonical character-device path it accepts.
    """
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

    # ------------------------------------------------------------------ args
    args = [
        DeclareLaunchArgument('lidar',         default_value='true',
                              description='Launch Livox MID360 LiDAR driver'),
        DeclareLaunchArgument('fast_lio_mode', default_value='false',
                              description='true = Livox xfer_format=1 (CustomMsg for FAST-LIO2); '
                                          'false = xfer_format=0 (PointCloud2 for GICP only). '
                                          'Also pass with_fast_lio:=true to localization launch.'),
        DeclareLaunchArgument('camera',        default_value='true',
                              description='Launch RealSense D435 camera driver'),
        DeclareLaunchArgument('arm_camera', default_value='true',
                              description='Launch the SO101 wrist USB camera on /arm_camera/color/image_raw.'),
        DeclareLaunchArgument(
            'arm_camera_device',
            default_value='/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0',
            description='Stable V4L2 capture path for the SO101 wrist camera.'),
        DeclareLaunchArgument('arm_camera_width', default_value='640',
                              description='SO101 wrist camera image width.'),
        DeclareLaunchArgument('arm_camera_height', default_value='480',
                              description='SO101 wrist camera image height.'),
        DeclareLaunchArgument('arm_camera_framerate', default_value='15.0',
                              description='SO101 wrist camera frame rate; 15 FPS limits Full-mode CPU/USB load.'),
        DeclareLaunchArgument('drive_motors',  default_value='true',
                              description='Launch BLV-R Modbus drive motors (x3)'),
        DeclareLaunchArgument('steer_motors',  default_value='true',
                              description='Launch Dynamixel steer motors (x3) + odom'),
        DeclareLaunchArgument('serial_motors', default_value='true',
                              description='Launch linear + camera-swing motors (OpenRB-150 serial)'),
        DeclareLaunchArgument('joy', default_value='true',
                              description='Launch Joy-Con / gamepad input and teleop nodes'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick device path (e.g. /dev/input/js0 or js1)'),
        DeclareLaunchArgument('cmd_vel_pipeline', default_value='true',
                              description='Launch nav_mode_switch_node + cmd_vel_safety_node. '
                                          'Set false if another launcher owns /cmd_vel_raw and /cmd_vel.'),
        DeclareLaunchArgument('camera_motor_joy', default_value='true',
                              description='Launch Joy-Con camera pan/tilt motor control node. '
                                          'Requires /joy from teleop.launch.py or joy_node.'),
        DeclareLaunchArgument('camera_pan_axis', default_value='6',
                              description='Joy axis for camera base pan. Default 6 is D-pad left/right.'),
        DeclareLaunchArgument('camera_tilt_axis', default_value='7',
                              description='Joy axis for camera tilt. Default 7 is D-pad up/down.'),
        DeclareLaunchArgument('camera_pan_step_deg', default_value='2.0',
                              description='Pan target angle delta per Joy command timer tick.'),
        DeclareLaunchArgument('camera_tilt_step_deg', default_value='5.0',
                              description='Tilt target angle delta per Joy command timer tick.'),
        DeclareLaunchArgument('camera_command_period_sec', default_value='0.05',
                              description='Camera motor command timer period. Smaller is smoother.'),
        DeclareLaunchArgument('camera_initial_pan_angle', default_value='267.0',
                              description='Startup pan angle. Set nan to disable.'),
        DeclareLaunchArgument('camera_initial_tilt_angle', default_value='102.0',
                              description='Startup tilt angle. Set nan to disable.'),
        DeclareLaunchArgument('rviz',          default_value='true',
                              description='Open RViz2'),
        DeclareLaunchArgument('static_odom',   default_value='false',
                              description='Publish static odom→base_footprint TF for sensor-only testing. '
                                          'Set true only when robot_odom_node is not running.'),
        DeclareLaunchArgument('use_glim_loc',  default_value='true',
                              description='Skip static map→odom TF (default: true). '
                                          'GLIM publishes map→odom dynamically. '
                                          'Set false only for sensor-only testing without GLIM.'),
        DeclareLaunchArgument('lift',          default_value='true',
                              description='Launch vertical lift control nodes (lift_serial_node + lift_joy_node). '
                                          'Requires Arduino on lift_serial_port. Default true.'),
        DeclareLaunchArgument(
            'lift_serial_port',
            default_value='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351E062C1-if00',
            description='Stable by-id path for the verified lift Arduino UNO.'),
        DeclareLaunchArgument('lift_position_max_mm', default_value='200.0',
                              description='Lift software upper limit [mm].'),
        DeclareLaunchArgument('lift_position_min_mm', default_value='0.0',
                              description='Lift software lower limit [mm].'),
        DeclareLaunchArgument('lift_home_position_mm', default_value='100.0',
                              description='Lift home position [mm] assigned after homing.'),
        DeclareLaunchArgument('lift_initial_position_mm', default_value='0.0',
                              description='Unreferenced startup estimate [mm]. Current physical position was operator-confirmed at the lower endpoint on 2026-09-03.'),
        DeclareLaunchArgument(
            'lift_initial_position_reference',
            default_value='UNREFERENCED_SOFTWARE_ESTIMATE',
            description='Evidence label; use SET_MIN/SET_MAX after operator endpoint confirmation.'),
        DeclareLaunchArgument('lift_auto_home', default_value='false',
                              description='Auto-home the lift on startup (2.5 s after launch).'),
        DeclareLaunchArgument('lift_soft_limits', default_value='false',
                              description='Enable software position limits for the lift.'),
        DeclareLaunchArgument('lift_abzo_monitor', default_value='false',
                              description='Enable read-only AZD-KD Modbus/ABZO monitor. Default false until wired.'),
        DeclareLaunchArgument('lift_abzo_serial_port', default_value='',
                              description='Stable /dev/serial/by-id path of the isolated lift USB-RS485 adapter.'),
        DeclareLaunchArgument('lift_abzo_slave_id', default_value='1',
                              description='AZD-KD Modbus address; address 0 is not valid for reads.'),
        DeclareLaunchArgument('lift_abzo_baud_rate', default_value='115200',
                              description='AZD-KD RS-485 baud rate; must match the BAUD switch.'),
        DeclareLaunchArgument('lift_abzo_position_mm_per_step', default_value='0.0',
                              description='Signed calibrated mm/step. Zero means raw ABZO only.'),
        DeclareLaunchArgument('lift_abzo_position_offset_mm', default_value='0.0',
                              description='Calibrated ABZO position offset [mm].'),
        DeclareLaunchArgument('lift_abzo_physical_limits_configured', default_value='false',
                              description='True only after FW-LS/RV-LS/HOMES wiring/assignment is verified.'),
        DeclareLaunchArgument('lift_abzo_software_limits_configured', default_value='false',
                              description='True only after AZD-KD software-limit parameters are verified.'),
        DeclareLaunchArgument('arm', default_value='true',
                              description='Launch guarded SO101 follower control and Dashboard gateway. '
                                          'Starts torque-off in real2sim mode.'),
        DeclareLaunchArgument(
            'touch_anything', default_value='true',
            description='Launch the preview-only Touch Anything observation node. '
                        'SAM is not loaded until an RViz point prompt is submitted.'),
        DeclareLaunchArgument('arm_shutdown_on_server_exit', default_value='false',
                              description='Shut down this launch if the guarded Arm server exits. '
                                          'Full keeps running by default; Arm Only overrides this to true.'),
        DeclareLaunchArgument('arm_visual_x', default_value='0.32084',
                              description='SO101/lift mount X from base_link [m]; '
                                          'retained pre-remount estimate pending calibration.'),
        DeclareLaunchArgument('arm_visual_y', default_value='0.01262',
                              description='SO101/lift mount Y from base_link [m]; '
                                          'retained pre-remount estimate pending calibration.'),
        DeclareLaunchArgument('arm_visual_z_min', default_value='0.89250',
                              description='Directly measured SO101 base-link centre height above ground '
                                          '(1.005 m) expressed from base_link at 0.1125 m [m].'),
        DeclareLaunchArgument('arm_visual_z_max', default_value='1.12250',
                              description='Upper software estimate preserving the existing measured 0.23 m lift span.'),
        DeclareLaunchArgument('arm_visual_roll', default_value='0.0',
                              description='SO101 mount roll from base_link [rad].'),
        DeclareLaunchArgument('arm_visual_pitch', default_value='0.0',
                              description='SO101 mount pitch [rad]; 0 keeps the physically '
                                          'remounted base parallel to the ground.'),
        DeclareLaunchArgument('arm_visual_yaw', default_value='-0.01532',
                              description='SO101 mount yaw from base_link [rad]; '
                                          'corrected after the remount front/back check.'),
    ]

    # ------------------------------------------------- robot_state_publisher
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
        parameters=[robot_description],
    )

    # A second robot_description/TF tree is used for the follower.  Keeping the
    # arm/ link namespace and /arm/robot_description topic avoid colliding with
    # the mobile robot's base_link and /robot_description while allowing the
    # navigation RViz process to display both models.
    arm_visual_xacro = os.path.join(
        get_package_share_directory('so_arm101_description'),
        'urdf', 'so_arm101_visual.urdf.xacro')
    arm_robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', arm_visual_xacro,
                ' prefix:=arm/ parent:=arm/world wrist_camera:=true',
            ]), value_type=str)
    }
    arm_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='so101_navigation_robot_state_publisher',
        condition=IfCondition(LaunchConfiguration('arm')),
        output='screen',
        parameters=[arm_robot_description],
        remappings=[
            ('robot_description', '/arm/robot_description'),
            ('joint_states', '/arm/visual_joint_states'),
        ],
    )
    arm_visual_joint_calibrator = Node(
        package='so101_arm_control',
        executable='arm_visual_joint_state_calibrator',
        name='arm_visual_joint_state_calibrator',
        condition=IfCondition(LaunchConfiguration('arm')),
        output='screen',
    )
    # A separate orange TF tree visualizes the Dashboard Preview candidate.
    # Its joints intentionally use the candidate_ prefix published by
    # arm_gateway, while its links live under arm_preview/ to keep both trees
    # visible alongside the real arm and the mobile robot.
    arm_preview_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', arm_visual_xacro,
                " prefix:=arm_preview/ joint_prefix:=candidate_ "
                "parent:=arm_preview/world color:='1.0 0.45 0.08 1.0'",
            ]), value_type=str)
    }
    arm_preview_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='so101_navigation_preview_robot_state_publisher',
        condition=IfCondition(LaunchConfiguration('arm')),
        output='screen',
        parameters=[arm_preview_description],
        remappings=[
            ('robot_description', '/arm/candidate/robot_description'),
            ('joint_states', '/arm/candidate_joint_states'),
        ],
    )
    arm_visual_mount = Node(
        package='so101_arm_control',
        executable='arm_mount_tf_publisher',
        name='so101_arm_mount_tf_publisher',
        condition=IfCondition(LaunchConfiguration('arm')),
        parameters=[{
            'parent_frame': 'base_link',
            'x_m': LaunchConfiguration('arm_visual_x'),
            'y_m': LaunchConfiguration('arm_visual_y'),
            'mount_z_min_m': LaunchConfiguration('arm_visual_z_min'),
            'mount_z_max_m': LaunchConfiguration('arm_visual_z_max'),
            'position_min_mm': LaunchConfiguration('lift_position_min_mm'),
            'position_max_mm': LaunchConfiguration('lift_position_max_mm'),
            'initial_position_mm': LaunchConfiguration('lift_initial_position_mm'),
            'roll': LaunchConfiguration('arm_visual_roll'),
            'pitch': LaunchConfiguration('arm_visual_pitch'),
            'yaw': LaunchConfiguration('arm_visual_yaw'),
        }],
        output='screen',
    )

    # ---- static TF: map → odom  (identity, until localization node exists) ----
    # Skipped when use_glim_loc:=true because GLIM publishes map→odom dynamically.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_glim_loc')),
    )

    # ---- static TF: odom → base_footprint  (identity, active only when steer_motors=false) ----
    # When robot_odom_node IS running it publishes this TF dynamically.
    # When motors are not connected this static version keeps the TF chain intact
    # so that LiDAR / camera data can be visualised in RViz without hardware.
    odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('static_odom')),
    )

    # --------------------------------------------------------- Livox MID360 --
    # xfer_format is a C++ int parameter — cannot be set via PythonExpression string.
    # Use two separate Groups: one for each format, guarded by fast_lio_mode condition.
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

    # xfer_format=0: PointCloud2 — default GICP mode
    lidar_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('lidar')),
        actions=[
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration('fast_lio_mode')),
                actions=[Node(
                    package='livox_ros_driver2',
                    executable='livox_ros_driver2_node',
                    name='livox_lidar_publisher',
                    output='screen',
                    parameters=[{'xfer_format': 0}] + livox_common_params,
                )],
            ),
            # xfer_format=1: CustomMsg — required by FAST-LIO2
            GroupAction(
                condition=IfCondition(LaunchConfiguration('fast_lio_mode')),
                actions=[Node(
                    package='livox_ros_driver2',
                    executable='livox_ros_driver2_node',
                    name='livox_lidar_publisher',
                    output='screen',
                    parameters=[{'xfer_format': 1}] + livox_common_params,
                )],
            ),
        ],
    )

    # ------------------------------------------------------- RealSense D435 --
    camera_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('camera')),
        actions=[
            IncludeLaunchDescription(
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
                    # robot_state_publisher owns the calibrated camera chain;
                    # prevent the RealSense driver from publishing duplicate
                    # camera_link/color/depth TF children.
                    'publish_tf':       'false',
                    'enable_depth':     'true',
                    'enable_color':     'true',
                    # Touch Anything selects pixels in the color image and
                    # therefore requires depth registered into that exact
                    # pixel coordinate system. The raw depth stream alone is
                    # not valid evidence for color-pixel deprojection.
                    'align_depth.enable': 'true',
                    'enable_sync':      'true',
                    'enable_infra1':    'false',
                    'enable_infra2':    'false',
                    'diagnostics_period': '1.0',
                    'rgb_camera.color_profile': '640,480,30',
                }.items(),
            ),
        ],
    )

    # ------------------------------------------------ SO101 wrist camera --
    # The by-id index0 endpoint is the ARC/Sonix video capture stream; index1
    # is metadata and must not be selected. Keep its ROS namespace separate
    # from the mobile robot's RealSense D435 topics.
    arm_camera_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('arm_camera')),
        actions=[OpaqueFunction(function=_resolved_arm_camera_node)],
    )

    # ------------------------------------------ BLV-R drive motors (Modbus) --
    # 1) om_modbus_master: low-level Modbus RTU layer (stable FTDI ID, IDs 1/2/3)
    # 2) drive_motor.py:   high-level velocity control + /drive_odom publisher
    drive_motors_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('drive_motors')),
        actions=[
            # Modbus RTU driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('om_modbus_master'),
                        'launch', 'om_modbus_master_launch.py',
                    )
                ),
                launch_arguments={
                    'com':        '/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FT7E9M3C-if00-port0',
                    'topicID':    '0',
                    'baudrate':   '230400',
                    # /om_state0 is a repeated state snapshot, not motor
                    # feedback.  The driver also publishes a ready snapshot on
                    # every completed transaction (~25 Hz), so a 10 Hz periodic
                    # refresh preserves direct state/error visibility without
                    # the measured ~125 Hz callback load.
                    'updateRate': '10',
                    'firstGen':   '',
                    'secondGen':  '1,2,3',
                    'globalID':   '10',
                    'axisNum':    '3',
                }.items(),
            ),
            # drive_motor.py — not a colcon-installed node, run directly as Python script
            ExecuteProcess(
                cmd=[
                    '/usr/bin/python3',
                    os.path.join(
                        os.path.expanduser('~'), 'robot_ws', 'src',
                        'om_modbus_master_V201', 'om_modbus_master',
                        'sample', 'BLV_R', 'drive_motor.py',
                    ),
                ],
                output='screen',
                name='drive_motor',
            ),
        ],
    )

    # --------------------------------- Dynamixel steer motors + odom nodes --
    # 1) read_write_node:      Dynamixel SDK driver  (service: get_position)
    # 2) steer_motor_node:     reads position → publishes /steer_odom
    # 3) cmd_vel_to_motor_node: /cmd_vel → /steer_ang + /drive_vel
    # 4) robot_odom_node:      /steer_odom + /drive_odom → /wheel_odom + TF odom→base_footprint
    steer_motors_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('steer_motors')),
        actions=[
            Node(
                package='dynamixel_sdk_examples',
                executable='read_write_node',
                name='dynamixel_driver',
                output='screen',
            ),
            Node(
                package='omni_base_driver',
                executable='steer_motor_node',
                name='steer_motor_node',
                output='screen',
            ),
            Node(
                package='omni_base_driver',
                executable='cmd_vel_to_motor_node',
                name='cmd_vel_to_motor_node',
                output='screen',
            ),
            Node(
                package='omni_base_driver',
                executable='robot_odom_node',
                name='robot_odom_node',
                output='screen',
            ),
        ],
    )

    # --------------------------------------------------------- Joy-Con input --
    joy_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('joy')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('robot_bringup'),
                        'launch', 'teleop.launch.py',
                    )
                ),
                launch_arguments={
                    'joy_dev': LaunchConfiguration('joy_dev'),
                }.items(),
            ),
        ],
    )

    # -------------------------------------- Cmd_vel arbitration + safety --
    # teleop.launch.py publishes /teleop/cmd_vel only. These nodes are needed
    # for standalone test_all teleop to reach /cmd_vel and the motor driver.
    cmd_vel_pipeline_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('cmd_vel_pipeline')),
        actions=[
            Node(
                package='nav_pkg',
                executable='joy_source_arbiter_node.py',
                name='joy_source_arbiter_node',
                output='screen',
            ),
            Node(
                package='nav_pkg',
                executable='nav_mode_switch_node.py',
                name='nav_mode_switch_node',
                output='screen',
                parameters=[{
                    'manual_deadman_button': 4,
                    'manual_teleop_timeout_sec': 0.25,
                }],
            ),
            Node(
                package='safety_layer',
                executable='cmd_vel_safety_node',
                name='cmd_vel_safety_node',
                output='screen',
                parameters=[{
                    'watchdog_timeout': 0.30,
                    'manual_bypass_slew': True,
                }],
            ),
        ],
    )

    # --------------------------------------- Linear + camera-swing (serial) --
    # chokudo_cameraswing_air_serial_node controls both motors via OpenRB-150
    # Port: /dev/serial/by-id/usb-ROBOTIS_OpenRB-150_…
    # camera_motor_joy_node reuses the same target topics for the new camera
    # base pan motor (old chokudo motor) and the retained camera swing motor.
    serial_motors_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('serial_motors')),
        actions=[
            Node(
                package='serial_transciever',
                executable='chokudo_cameraswing_air_serial_node',
                name='chokudo_cameraswing_air_serial_node',
                output='screen',
            ),
            Node(
                condition=IfCondition(LaunchConfiguration('camera_motor_joy')),
                package='serial_transciever',
                executable='camera_motor_joy_node',
                name='camera_motor_joy_node',
                output='screen',
                parameters=[{
                    'enable_button': 4,
                    'pan_axis': ParameterValue(
                        LaunchConfiguration('camera_pan_axis'), value_type=int),
                    'tilt_axis': ParameterValue(
                        LaunchConfiguration('camera_tilt_axis'), value_type=int),
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
            ),
        ],
    )

    # Read-only raw-feedback bridge for the two calibrated URDF joints.  It
    # publishes nothing while either hardware feedback source is absent/stale.
    camera_pan_tilt_joint_state = Node(
        package='serial_transciever',
        executable='camera_pan_tilt_joint_state_node',
        name='camera_pan_tilt_joint_state_node',
        condition=IfCondition(LaunchConfiguration('serial_motors')),
        output='screen',
    )

    # ----------------------------------------------- Vertical lift control --
    # lift_serial_node: Arduino serial bridge + position tracking + soft limits
    # lift_joy_node:    hold LT → UP, hold RT → DOWN (shares /joy with other nodes)
    lift_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('lift')),
        actions=[
            Node(
                package='serial_transciever',
                executable='lift_serial_node',
                name='lift_serial_node',
                output='screen',
                parameters=[{
                    'serial_port':        LaunchConfiguration('lift_serial_port'),
                    'baud_rate':          9600,
                    'jog_speed_mm_s':     60.0,
                    'position_min_mm':    LaunchConfiguration('lift_position_min_mm'),
                    'position_max_mm':    LaunchConfiguration('lift_position_max_mm'),
                    'home_position_mm':   LaunchConfiguration('lift_home_position_mm'),
                    'initial_position_mm': LaunchConfiguration('lift_initial_position_mm'),
                    'initial_position_reference': LaunchConfiguration(
                        'lift_initial_position_reference'),
                    'expected_usb_vendor_id': '2341',
                    'expected_usb_product_id': '0043',
                    'require_usb_identity': True,
                    'auto_home_on_start': LaunchConfiguration('lift_auto_home'),
                    'enable_soft_limits': LaunchConfiguration('lift_soft_limits'),
                }],
            ),
            Node(
                package='serial_transciever',
                executable='lift_joy_node',
                name='lift_joy_node',
                output='screen',
                parameters=[{
                    'up_axis': 2,                  # LT: +1 released, -1 pressed
                    'down_axis': 5,                # RT: +1 released, -1 pressed
                    'trigger_active_below': -0.5,
                    'joy_timeout_sec': 0.30,
                }],
            ),
            Node(
                package='serial_transciever',
                executable='lift_abzo_monitor_node',
                name='lift_abzo_monitor_node',
                condition=IfCondition(LaunchConfiguration('lift_abzo_monitor')),
                output='screen',
                parameters=[{
                    'serial_port': LaunchConfiguration('lift_abzo_serial_port'),
                    'slave_id': LaunchConfiguration('lift_abzo_slave_id'),
                    'baud_rate': LaunchConfiguration('lift_abzo_baud_rate'),
                    'poll_rate_hz': 5.0,
                    'position_mm_per_step': LaunchConfiguration(
                        'lift_abzo_position_mm_per_step'),
                    'position_offset_mm': LaunchConfiguration(
                        'lift_abzo_position_offset_mm'),
                    'physical_limit_inputs_configured': LaunchConfiguration(
                        'lift_abzo_physical_limits_configured'),
                    'software_limits_configured': LaunchConfiguration(
                        'lift_abzo_software_limits_configured'),
                }],
            ),
        ],
    )

    # -------------------------------------- SO101 follower arm control --
    # Embedded mode owns the follower serial bus and publishes Arm control and
    # health topics while reusing the unified Agent Dashboard. Its standalone
    # model publishers remain disabled; the prefixed navigation model above is
    # the sole Arm TF/description owner in the integrated stack.
    arm_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('arm')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('so101_arm_control'),
                        'launch', 'arm_real_dashboard_test.launch.py',
                    )
                ),
                launch_arguments={
                    'rviz': 'false',
                    'dashboard': 'false',
                    'robot_state_publishers': 'false',
                    'shutdown_on_server_exit': LaunchConfiguration(
                        'arm_shutdown_on_server_exit'),
                }.items(),
            ),
        ],
    )

    # Touch observation is deliberately a separate process from the Arm
    # gateway. It owns no motor/action publisher and lazy-loads SAM only after
    # a point prompt. The shared safety layer consumes its base-lock evidence.
    touch_anything_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('touch_anything')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('touch_anything'),
                        'launch', 'touch_observation.launch.py',
                    )
                ),
            ),
        ],
    )

    # ----------------------------------------------------------------- RViz --
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(robot_desc_pkg, 'rviz', 'robot.rviz')],
        output='screen',
    )

    return LaunchDescription(
        args + [
            rsp_node,
            arm_rsp_node,
            arm_visual_joint_calibrator,
            arm_visual_mount,
            arm_preview_rsp_node,
            map_to_odom,
            odom_to_base,
            lidar_launch,
            camera_launch,
            arm_camera_node,
            drive_motors_group,
            steer_motors_group,
            joy_group,
            cmd_vel_pipeline_group,
            serial_motors_group,
            camera_pan_tilt_joint_state,
            lift_group,
            arm_group,
            touch_anything_group,
            rviz_node,
        ]
    )
