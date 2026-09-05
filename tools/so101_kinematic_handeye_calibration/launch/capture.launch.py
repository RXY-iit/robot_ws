import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share=get_package_share_directory('so101_kinematic_handeye_calibration')
    args=[DeclareLaunchArgument('output_root'),DeclareLaunchArgument('squares_x',default_value='7'),
      DeclareLaunchArgument('squares_y',default_value='5'),DeclareLaunchArgument('square_length_m',default_value='0.0338'),
      DeclareLaunchArgument('marker_length_m',default_value='0.0165'),DeclareLaunchArgument('min_charuco_corners',default_value='8'),
      DeclareLaunchArgument('stability_window_sec',default_value='1.0'),DeclareLaunchArgument('joint_max_span_rad',default_value='0.008')]
    recorder=Node(package='so101_kinematic_handeye_calibration',executable='capture_node.py',
      parameters=[{'output_root':LaunchConfiguration('output_root'),
        'squares_x':ParameterValue(LaunchConfiguration('squares_x'),value_type=int),
        'squares_y':ParameterValue(LaunchConfiguration('squares_y'),value_type=int),
        'square_length_m':ParameterValue(LaunchConfiguration('square_length_m'),value_type=float),
        'marker_length_m':ParameterValue(LaunchConfiguration('marker_length_m'),value_type=float),
        'min_charuco_corners':ParameterValue(LaunchConfiguration('min_charuco_corners'),value_type=int),
        'stability_window_sec':ParameterValue(LaunchConfiguration('stability_window_sec'),value_type=float),
        'joint_max_span_rad':ParameterValue(LaunchConfiguration('joint_max_span_rad'),value_type=float)}],output='screen')
    rviz=Node(package='rviz2',executable='rviz2',name='so101_calibration_rviz',
      arguments=['-d',os.path.join(share,'config','capture.rviz')],output='screen')
    return LaunchDescription(args+[recorder,rviz])
