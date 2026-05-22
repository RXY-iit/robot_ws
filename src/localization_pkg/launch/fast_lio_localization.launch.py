"""fast_lio_localization.launch.py — GICP prior-map localization stack.

Replaces GLIM localization for Nav2 navigation.

Nodes launched:
  gicp_localizer_node        — GICP scan-to-map, publishes map→odom TF
  map_server                 — serves /map (OccupancyGrid) for Nav2 global planner
  lifecycle_manager          — activates map_server
  pointcloud_to_laserscan    — lidar → /scan for Nav2 obstacle_layer
  [with_fast_lio=true only]
    fast_lio                 — LiDAR-IMU odometry (needs fast_lio pkg built)
    livox_custom_to_pc2_node — relay: /livox/lidar (CustomMsg) → /livox/lidar_pc2 (PointCloud2)
    (gicp and laserscan automatically remapped to /livox/lidar_pc2)

Topic routing:
  with_fast_lio=false  (default):  GICP + laserscan ← /livox/lidar (PointCloud2, xfer_format=0)
  with_fast_lio=true:              FAST-LIO ← /livox/lidar (CustomMsg, xfer_format=1)
                                   relay converts to /livox/lidar_pc2 (PointCloud2)
                                   GICP + laserscan ← /livox/lidar_pc2

Launch arguments:
  map              2D map yaml path (for Nav2 costmap / map_server)
  pcd_map          3D PLY map for GICP localization
  initial_x        initial robot X in map frame (m, default 0.0)
  initial_y        initial robot Y in map frame (m, default 0.0)
  initial_yaw      initial robot yaw in map frame (rad, default 0.0)
  with_fast_lio    launch FAST-LIO2 + relay node (default false)
  use_fast_lio_hint use /fast_lio/odometry as GICP initial-pose hint
                    (auto-enabled when with_fast_lio=true)
  use_sim_time     (default false)

Usage examples:
  # Normal startup
  ros2 launch localization_pkg fast_lio_localization.launch.py

  # With FAST-LIO2 (requires bringup.launch.py fast_lio_mode:=true for xfer_format=1)
  ros2 launch localization_pkg fast_lio_localization.launch.py with_fast_lio:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("localization_pkg")

    default_2d_map = "/home/matsunaga-h/robot_ws/maps/l402_2d_map_0503.yaml"
    default_pcd_map = (
        "/home/matsunaga-h/robot_ws/maps/saved-map/map-l402-0503/l402_points_0503"
    )
    default_gicp_params = os.path.join(pkg_share, "config", "gicp_localizer.yaml")

    # ── args ──────────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "map", default_value=default_2d_map,
            description="2D occupancy map yaml for Nav2 map_server / global costmap",
        ),
        DeclareLaunchArgument(
            "pcd_map", default_value=default_pcd_map,
            description="3D PLY map for GICP localization",
        ),
        DeclareLaunchArgument(
            "initial_x", default_value="0.0",
            description="Robot starting X in map frame (m)",
        ),
        DeclareLaunchArgument(
            "initial_y", default_value="0.0",
            description="Robot starting Y in map frame (m)",
        ),
        DeclareLaunchArgument(
            "initial_yaw", default_value="0.0",
            description="Robot starting yaw in map frame (rad)",
        ),
        DeclareLaunchArgument(
            "with_fast_lio", default_value="false",
            description="Launch FAST-LIO2 LiDAR-IMU odometry node (Phase 2). "
                        "Requires fast_lio package built via tools/setup_fast_lio.sh",
        ),
        DeclareLaunchArgument(
            "use_fast_lio_hint", default_value="false",
            description="Let gicp_localizer subscribe to /fast_lio/odometry as a hint.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
        ),
        DeclareLaunchArgument(
            "scan_min_height", default_value="-0.8",
            description="min z in base_footprint frame for /scan slice (m). "
                        "LiDAR at 1.3 m: -0.8 → ground+0.5 m. "
                        "Captures wall mid-section, avoids floor/ceiling.",
        ),
        DeclareLaunchArgument(
            "scan_max_height", default_value="-0.3",
            description="max z in base_footprint frame for /scan slice (m). "
                        "-0.3 → ground+1.0 m. Stays below ceiling.",
        ),
    ]

    # When with_fast_lio=true, Livox publishes CustomMsg; relay converts to
    # /livox/lidar_pc2.  GICP and laserscan must read from there instead.
    lidar_input_topic = PythonExpression(
        ["'/livox/lidar_pc2' if '", LaunchConfiguration("with_fast_lio"), "' == 'true' else '/livox/lidar'"]
    )

    # use_fast_lio_hint is automatically true when with_fast_lio=true
    effective_hint = PythonExpression(
        ["'true' if '", LaunchConfiguration("with_fast_lio"), "' == 'true' else '",
         LaunchConfiguration("use_fast_lio_hint"), "'"]
    )

    # ── GICP localizer ────────────────────────────────────────────────────────
    gicp_node = Node(
        package="localization_pkg",
        executable="gicp_localizer_node",
        name="gicp_localizer",
        output="screen",
        parameters=[
            default_gicp_params,
            {
                "map_path": LaunchConfiguration("pcd_map"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "use_fast_lio_hint": effective_hint,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("/livox/lidar", lidar_input_topic)],
    )

    # ── map_server (Nav2 lifecycle node) ─────────────────────────────────────
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": LaunchConfiguration("map"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": ["map_server"],
                "bond_timeout": 4.0,
            }
        ],
    )

    # ── pointcloud_to_laserscan → /scan ──────────────────────────────────────
    # /scan height slice targets wall mid-section in base_footprint frame.
    # LiDAR is at ~1.3 m above ground.  In base_footprint frame (origin at ground):
    #   ground+0.5 m = base_footprint z  0.5 m  = LiDAR z  -0.8 m
    #   ground+1.0 m = base_footprint z  1.0 m  = LiDAR z  -0.3 m
    # (LiDAR z: positive = up from sensor, which is 1.3 m above base_footprint)
    # This slice is visible from >1.4 m away (LiDAR 30° downward tilt blind zone).
    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", lidar_input_topic),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_frame": "base_footprint",
                "transform_tolerance": 0.05,
                "min_height": LaunchConfiguration("scan_min_height"),
                "max_height": LaunchConfiguration("scan_max_height"),
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.80,
                "range_max": 25.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
    )

    # ── FAST-LIO2 + relay (optional, requires bringup fast_lio_mode:=true) ───
    # Requires: tools/setup_fast_lio.sh has been run and fast_lio pkg is built.
    # When active:
    #   - fast_lio subscribes /livox/lidar (CustomMsg, xfer_format=1)
    #   - relay converts /livox/lidar → /livox/lidar_pc2 (PointCloud2) for GICP + laserscan
    #   - /fast_lio/odometry used as GICP initial-pose hint (delta-based)
    fast_lio_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("with_fast_lio")),
        actions=[
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fast_lio",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "fast_lio_mid360.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("/Odometry", "/fast_lio/odometry"),
                    ("/cloud_registered", "/fast_lio/cloud_registered"),
                    ("/cloud_registered_body", "/fast_lio/cloud_registered_body"),
                    ("/cloud_effected", "/fast_lio/cloud_effected"),
                    ("/Laser_map", "/fast_lio/laser_map"),
                    ("/path", "/fast_lio/path"),
                ],
            ),
            Node(
                package="localization_pkg",
                executable="livox_custom_to_pc2_node",
                name="livox_custom_to_pc2",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ],
    )

    return LaunchDescription(
        args + [
            gicp_node,
            map_server,
            lifecycle_manager,
            pointcloud_to_scan,
            fast_lio_group,
        ]
    )
