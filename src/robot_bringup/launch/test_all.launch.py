"""
test_all.launch.py  — Full hardware test launch

Starts all sensors and motors and opens RViz.

Launch arguments (all default true — set false to skip hardware not connected):
  lidar:=true/false          Livox MID360
  fast_lio_mode:=true/false  Switch Livox to xfer_format=1 (CustomMsg) for FAST-LIO2.
                             Must also pass with_fast_lio:=true to localization launch.
                             Default: false (xfer_format=0 PointCloud2, GICP-only mode)
  camera:=true/false         RealSense D435
  drive_motors:=true/false   BLV-R x3 via Modbus (om_modbus_master + drive_motor.py)
  steer_motors:=true/false   Dynamixel x3 (read_write_node + steer/cmd/odom nodes)
  serial_motors:=true/false  Linear + camera-swing via OpenRB-150 serial
  joy:=true/false            Joy-Con / gamepad input and /cmd_vel teleop
  camera_motor_joy:=true/false  Joy-Con control for camera pan/tilt motors
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
  joy_node + teleop_twist_joy_node → /joy, /cmd_vel
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
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument('lift_serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for the lift Arduino (e.g. /dev/ttyACM0).'),
        DeclareLaunchArgument('lift_position_max_mm', default_value='200.0',
                              description='Lift software upper limit [mm].'),
        DeclareLaunchArgument('lift_position_min_mm', default_value='0.0',
                              description='Lift software lower limit [mm].'),
        DeclareLaunchArgument('lift_home_position_mm', default_value='100.0',
                              description='Lift home position [mm] assigned after homing.'),
        DeclareLaunchArgument('lift_auto_home', default_value='false',
                              description='Auto-home the lift on startup (2.5 s after launch).'),
        DeclareLaunchArgument('lift_soft_limits', default_value='false',
                              description='Enable software position limits for the lift.'),
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
                    'publish_tf':       'true',
                    'enable_depth':     'true',
                    'enable_color':     'true',
                    'enable_infra1':    'false',
                    'enable_infra2':    'false',
                    'rgb_camera.color_profile': '640,480,30',
                }.items(),
            ),
        ],
    )

    # ------------------------------------------ BLV-R drive motors (Modbus) --
    # 1) om_modbus_master: low-level Modbus RTU layer (ttyUSB0, IDs 1/2/3)
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
                    'com':        '/dev/ttyUSB0',
                    'topicID':    '0',
                    'baudrate':   '230400',
                    'updateRate': '1000',
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

    # ----------------------------------------------- Vertical lift control --
    # lift_serial_node: Arduino serial bridge + position tracking + soft limits
    # lift_joy_node:    L1+Y → UP, L1+A → DOWN (shares /joy with other nodes)
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
                    'enable_button': 4,   # L1 — same dead-man as base teleop
                    'up_button':     3,   # Y
                    'down_button':   0,   # A
                }],
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
            map_to_odom,
            odom_to_base,
            lidar_launch,
            camera_launch,
            drive_motors_group,
            steer_motors_group,
            joy_group,
            serial_motors_group,
            lift_group,
            rviz_node,
        ]
    )
