"""fast_lio_localization.launch.py — GICP prior-map localization stack.

Replaces GLIM localization for Nav2 navigation.

Nodes launched:
  gicp_localizer_node        — GICP scan-to-map, publishes map→odom TF
  map_server                 — serves /map (OccupancyGrid) for Nav2 global planner
  lifecycle_manager          — activates map_server
  [with_fast_lio=true only]
    fast_lio                 — LiDAR-IMU odometry (needs fast_lio pkg built)
    livox_custom_to_pc2_node — relay: /livox/lidar (CustomMsg) → /livox/lidar_pc2 (PointCloud2)
    (GICP is automatically remapped to /livox/lidar_pc2)

Topic routing:
  with_fast_lio=false  (default):  GICP ← /livox/lidar (PointCloud2, xfer_format=0)
  with_fast_lio=true:              FAST-LIO ← /livox/lidar (CustomMsg, xfer_format=1)
                                   relay converts to /livox/lidar_pc2 (PointCloud2)
                                   GICP ← /livox/lidar_pc2

Nav2 obstacle detection is intentionally not fed by this LiDAR.  The installed
LiDAR has a near-field blind region; the active local costmap is fed by the
RealSense depth point cloud configured in nav_pkg/config/nav2_params.yaml.

Launch arguments:
  map              2D map yaml path (for Nav2 costmap / map_server)
  pcd_map          3D PLY map for GICP localization
  initial_x        initial robot X in map frame (m, default 0.0)
  initial_y        initial robot Y in map frame (m, default 0.0)
  initial_yaw      initial robot yaw in map frame (rad, default 0.0)
  scan_min_height  minimum lidar-frame z kept for GICP input
  scan_max_height  maximum lidar-frame z kept for GICP input
  with_fast_lio    launch FAST-LIO2 + relay node (default false)
  use_fast_lio_hint use /fast_lio/odometry as GICP initial-pose hint
                    (explicit opt-in; starting FAST-LIO alone does not enable it)
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
from launch.actions import DeclareLaunchArgument, GroupAction, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "scan_min_height", default_value="-1000000000.0",
            description="Minimum lidar-frame z kept for GICP input; large negative disables",
        ),
        DeclareLaunchArgument(
            "scan_max_height", default_value="1000000000.0",
            description="Maximum lidar-frame z kept for GICP input; large positive disables",
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
    ]

    # When with_fast_lio=true, Livox publishes CustomMsg; relay converts to
    # /livox/lidar_pc2.  GICP must read from there instead.
    lidar_input_topic = PythonExpression(
        ["'/livox/lidar_pc2' if '", LaunchConfiguration("with_fast_lio"), "' == 'true' else '/livox/lidar'"]
    )

    args.append(DeclareLaunchArgument("gicp_input_topic", default_value=lidar_input_topic,
        description="GICP-only PointCloud2 input override; default preserves existing routing"))

    # Starting FAST-LIO must not override an explicit hint opt-out.
    effective_hint = LaunchConfiguration("use_fast_lio_hint")

    # ── GICP localizer ────────────────────────────────────────────────────────
    gicp_node = Node(
        package="localization_pkg",
        executable="gicp_localizer_node",
        name="gicp_localizer",
        output="screen",
        additional_env={"OMP_NUM_THREADS": os.environ.get("GICP_OMP_THREADS", "2"),
                        "OPENBLAS_NUM_THREADS": "1"},
        parameters=[
            default_gicp_params,
            {
                "map_path": LaunchConfiguration("pcd_map"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "scan_min_height": LaunchConfiguration("scan_min_height"),
                "scan_max_height": LaunchConfiguration("scan_max_height"),
                "use_fast_lio_hint": effective_hint,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("/livox/lidar", LaunchConfiguration("gicp_input_topic"))],
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

    # ── FAST-LIO2 + relay (optional, requires bringup fast_lio_mode:=true) ───
    # Requires: tools/setup_fast_lio.sh has been run and fast_lio pkg is built.
    # When active:
    #   - fast_lio subscribes /livox/lidar (CustomMsg, xfer_format=1)
    #   - relay converts /livox/lidar → /livox/lidar_pc2 (PointCloud2) for GICP
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
                    {"use_sim_time": LaunchConfiguration("use_sim_time"),
                     "common.max_scan_backlog_sec": ParameterValue(PythonExpression([
                         "0.5 if '", LaunchConfiguration("use_sim_time"), "' == 'true' else 0.0"]), value_type=float),
                     "common.max_lidar_gap_sec": ParameterValue(PythonExpression([
                         "2.0 if '", LaunchConfiguration("use_sim_time"), "' == 'true' else 0.0"]), value_type=float)},
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
            UnsetEnvironmentVariable("PYTHONNOUSERSITE"),
            gicp_node,
            map_server,
            lifecycle_manager,
            fast_lio_group,
        ]
    )
