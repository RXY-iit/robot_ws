"""Start map_server + AMCL + pointcloud_to_laserscan for Nav2.

AMCL publishes map→odom TF from the 2D occupancy map and /scan.
pointcloud_to_laserscan converts /livox/lidar → /scan for costmap sensors.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("localization_pkg")
    default_map = "/home/matsunaga-h/robot_ws/maps/l402_2d_map_0503.yaml"
    default_amcl_params = os.path.join(pkg_share, "config", "amcl.yaml")

    map_yaml = LaunchConfiguration("map")
    amcl_params = LaunchConfiguration("amcl_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_glim_loc = LaunchConfiguration("use_glim_loc")
    use_pointcloud_to_scan = LaunchConfiguration("use_pointcloud_to_scan")
    scan_min_height = LaunchConfiguration("scan_min_height")
    scan_max_height = LaunchConfiguration("scan_max_height")
    scan_range_min = LaunchConfiguration("scan_range_min")
    scan_range_max = LaunchConfiguration("scan_range_max")

    pointcloud_to_scan = GroupAction(
        condition=IfCondition(use_pointcloud_to_scan),
        actions=[
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", "/livox/lidar"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_frame": "base_footprint",
                        "transform_tolerance": 0.05,
                        "min_height": scan_min_height,
                        "max_height": scan_max_height,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.0087,
                        "scan_time": 0.1,
                        "range_min": scan_range_min,
                        "range_max": scan_range_max,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
            )
        ],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        condition=UnlessCondition(use_glim_loc),
        parameters=[
            amcl_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Legacy/debug mode: when an external localizer provides map→odom, only map_server is managed.
    lifecycle_manager_glim = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=IfCondition(use_glim_loc),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server"],
                "bond_timeout": 4.0,
            }
        ],
    )

    # Standard AMCL mode: lifecycle_manager manages map_server + amcl
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=UnlessCondition(use_glim_loc),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["map_server", "amcl"],
                "bond_timeout": 4.0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("amcl_params", default_value=default_amcl_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "use_glim_loc",
                default_value="false",
                description="Deprecated external-localizer mode. Keep false for Nav2 AMCL.",
            ),
            DeclareLaunchArgument(
                "use_pointcloud_to_scan",
                default_value="true",
                description="Convert /livox/lidar → /scan for Nav2 costmap sensors.",
            ),
            DeclareLaunchArgument(
                "scan_min_height",
                default_value="0.20",
                description="Minimum point height in base_footprint used to create /scan.",
            ),
            DeclareLaunchArgument(
                "scan_max_height",
                default_value="1.00",
                description="Maximum point height in base_footprint used to create /scan.",
            ),
            DeclareLaunchArgument(
                "scan_range_min",
                default_value="0.80",
                description="Minimum range used in /scan. Increase to remove robot self-points.",
            ),
            DeclareLaunchArgument(
                "scan_range_max",
                default_value="25.00",
                description="Maximum range used in /scan.",
            ),
            pointcloud_to_scan,
            map_server,
            amcl,
            lifecycle_manager_glim,
            lifecycle_manager,
        ]
    )
