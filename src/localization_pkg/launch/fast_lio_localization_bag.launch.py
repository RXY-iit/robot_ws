"""Rosbag-replay entrypoint for GICP localization.

Use this only with `ros2 bag play --clock`. It forces simulated time
(`use_sim_time:=true`) so TF, scans, and /clock share the bag timestamp axis.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("localization_pkg")
    main_launch = os.path.join(pkg_share, "launch", "fast_lio_localization.launch.py")

    args = [
        DeclareLaunchArgument(
            "map",
            default_value="/home/matsunaga-h/robot_ws/maps/l402_2d_map_0503.yaml",
        ),
        DeclareLaunchArgument(
            "pcd_map",
            default_value="/home/matsunaga-h/robot_ws/maps/saved-map/map-l402-0503/l402_points_0503",
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("with_fast_lio", default_value="false"),
        DeclareLaunchArgument("use_fast_lio_hint", default_value="false"),
        DeclareLaunchArgument("scan_min_height", default_value="-0.8"),
        DeclareLaunchArgument("scan_max_height", default_value="-0.3"),
    ]

    include_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(main_launch),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "pcd_map": LaunchConfiguration("pcd_map"),
            "initial_x": LaunchConfiguration("initial_x"),
            "initial_y": LaunchConfiguration("initial_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
            "with_fast_lio": LaunchConfiguration("with_fast_lio"),
            "use_fast_lio_hint": LaunchConfiguration("use_fast_lio_hint"),
            "use_sim_time": "true",
            "scan_min_height": LaunchConfiguration("scan_min_height"),
            "scan_max_height": LaunchConfiguration("scan_max_height"),
        }.items(),
    )

    return LaunchDescription(args + [include_main])
