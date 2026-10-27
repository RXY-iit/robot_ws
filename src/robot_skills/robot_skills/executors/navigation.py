from __future__ import annotations

import math
import os
import json
import threading
import time
from typing import Any

import yaml
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from robot_skills import result_codes as codes
from robot_skills.executors.base import dry_run_result, make_result, now
from robot_skills.schemas import Pose2D, SkillContext, SkillResult
from robot_skills.motion_clock import MotionClock
from robot_skills.motion_evidence import MotionProgress, translation_error, swept_grid_clear


ACTION_STATUS_SUCCEEDED = 4
ACTION_STATUS_CANCELED = 5
ACTION_STATUS_ABORTED = 6
PROGRESS_CHECK_INTERVAL_SEC = 2.0
NARROW_PASSAGE_HEADING_TOLERANCE_RAD = math.radians(5.0)


def update_recovered_pose_metrics(metrics, goal, final_pose, source):
    """Keep pre-recovery evidence, but report final error against the original goal."""
    keys = ("final_pose", "final_xy_error_m", "final_yaw_error_rad")
    metrics.setdefault("pre_recovery_pose_evidence", {k: metrics.get(k) for k in keys})
    metrics["final_pose_source"] = source
    metrics["final_pose"] = final_pose
    metrics["final_xy_error_m"] = None
    metrics["final_yaw_error_rad"] = None
    if final_pose is not None:
        metrics["final_xy_error_m"] = math.hypot(final_pose["x"] - goal.x, final_pose["y"] - goal.y)
        delta = final_pose["yaw"] - goal.yaw
        metrics["final_yaw_error_rad"] = abs(math.atan2(math.sin(delta), math.cos(delta)))


def near_goal_yaw_error(current, goal, xy_tolerance):
    # None delegates tolerance to Nav2. Without its configured radius we cannot
    # classify a pose as near-goal, including when dynamic_timeout is disabled.
    if xy_tolerance is None:
        return None
    if current is None or math.hypot(current["x"] - goal.x, current["y"] - goal.y) > xy_tolerance:
        return None
    delta = current["yaw"] - goal.yaw
    return abs(math.atan2(math.sin(delta), math.cos(delta)))


def yaw_progress_reference(previous_best, current_error):
    """Require cumulative improvement, not arbitrary rotation or oscillation."""
    if current_error is None:
        return previous_best, False
    if previous_best is None:
        return current_error, False
    if previous_best - current_error >= 0.05:
        return current_error, True
    return previous_best, False


def motion_failure_code(reason):
    if reason == "navigation_localization_unhealthy":
        return codes.FAILED_LOCALIZATION
    if str(reason).startswith("micro_adjust_"):
        return codes.FAILED_SAFETY
    if reason == "relative_motion_off_path":
        return codes.FAILED_NAVIGATION
    if reason == "wheel_odom_stale":
        return codes.FAILED_PRECHECK
    return codes.FAILED_TIMEOUT


def pose_to_stamped(node, pose: Pose2D) -> PoseStamped:
    out = PoseStamped()
    out.header.frame_id = pose.frame_id
    out.header.stamp = node.get_clock().now().to_msg()
    out.pose.position.x = pose.x
    out.pose.position.y = pose.y
    out.pose.position.z = 0.0
    out.pose.orientation.z = math.sin(pose.yaw / 2.0)
    out.pose.orientation.w = math.cos(pose.yaw / 2.0)
    return out


class NavigationSkills:
    def __init__(self, node, localization_metrics=None):
        self.node = node
        self._localization_metrics = localization_metrics
        self._latest_footprint = None
        self._latest_footprint_time = 0.0
        self._footprint_sub = node.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint", self._footprint_cb, 1)
        self._action_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._compute_path_client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        self._cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel_raw", 10)
        self._latest_local_costmap = None
        self._latest_local_costmap_time = 0.0
        self._local_costmap_sub = node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._local_costmap_cb,
            10,
        )
        self._local_costmap_raw_sub = node.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap_raw",
            self._local_costmap_cb,
            10,
        )
        self._latest_wheel_odom_pose: dict[str, float] | None = None
        self._latest_wheel_odom_time = 0.0
        self._latest_wheel_odom_stamp = None
        self._wheel_odom_lock = threading.Lock()
        self._wheel_odom_sub = node.create_subscription(
            Odometry,
            "/wheel_odom",
            self._wheel_odom_cb,
            10,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._goal_marker_pub = node.create_publisher(
            MarkerArray,
            "/robot_skills/navigation_goal_markers",
            marker_qos,
        )
        self._route_marker_pub = node.create_publisher(
            MarkerArray,
            "/robot_skills/navigation_route_markers",
            marker_qos,
        )
        self._navigation_progress_pub = node.create_publisher(String, "/agent/navigation_progress", 10)
        self._navigation_route_state_pub = node.create_publisher(String, "/agent/navigation_route_state", 10)
        self._motion_policy_profile_pub = node.create_publisher(
            String,
            "/motion_policy/profile",
            10,
        )
        self._active_goal_handle = None
        self._pending_nav_goals = 0
        self._nav_result_futures = []
        self.stopping = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        self._navigation_annotations = self._load_navigation_annotations()
        self._nav2_map = self._load_nav2_map()
        self._current_route_path_points: list[dict[str, float]] | None = None
        self._last_global_plan_error: dict[str, Any] | None = None
        self._direct_motion_cancel = False

    def goto_pose(
        self,
        context: SkillContext,
        pose: Pose2D,
        timeout_sec: float = 120.0,
        xy_goal_tolerance_m: float | None = None,
        yaw_goal_tolerance_rad: float | None = None,
        tolerance_profile: str = "default",
        dynamic_timeout: bool = False,
        max_timeout_sec: float = 360.0,
        recovery_policy: dict[str, Any] | None = None,
        apply_navigation_annotations: bool = True,
        publish_goal_markers: bool = True,
        alternate_goal_attempts: int = 1,
    ) -> SkillResult:
        started = now()
        if getattr(context, "recovery_check", lambda: False)():
            return make_result("goto_pose", False, codes.CANCELLED, "Navigation held by Recovery Agent.", started)
        started_monotonic = time.monotonic()
        task_xy_tolerance = float(xy_goal_tolerance_m) if xy_goal_tolerance_m is not None else None
        task_yaw_tolerance = float(yaw_goal_tolerance_rad) if yaw_goal_tolerance_rad is not None else None
        requested_timeout = float(timeout_sec)
        effective_timeout = requested_timeout
        timeout_policy = "fixed"
        start_pose = None
        start_xy_distance = None
        timeout_extensions = 0
        local_clearance = None
        local_passage_result = None
        navigation_alignment_result = None
        navigation_annotation_result = None
        navigation_annotation_selected = None
        navigation_annotation_diagnostics = None
        pre_align_result = None
        pre_align_heading_delta = None
        pre_align_reason = None
        pre_align_skip_reason = None
        recovery_policy = recovery_policy if isinstance(recovery_policy, dict) else {}
        allow_relaxed_tolerance = self._recovery_policy_allows(
            recovery_policy,
            "allow_relaxed_tolerance",
            True,
        )
        allow_alternate_goal = self._recovery_policy_allows(
            recovery_policy,
            "allow_alternate_goal",
            True,
        )
        if pose.frame_id != "map":
            return make_result(
                "goto_pose",
                False,
                codes.FAILED_SCHEMA,
                f'goto_pose requires frame_id="map", got "{pose.frame_id}".',
                started,
            )
        if context.mode == "dry_run":
            return dry_run_result(
                "goto_pose",
                "Dry-run navigation accepted.",
                {
                    "pose": pose.to_dict(),
                    "timeout_policy": "dynamic" if dynamic_timeout else "fixed",
                    "requested_timeout_sec": requested_timeout,
                    "effective_timeout_sec": effective_timeout,
                    "tolerance_profile": tolerance_profile,
                    "xy_goal_tolerance_m": task_xy_tolerance,
                    "yaw_goal_tolerance_rad": task_yaw_tolerance,
                    "recovery_policy": recovery_policy,
                },
            )
        if context.robot_mode != "AUTO":
            return make_result(
                "goto_pose",
                False,
                codes.NEEDS_HUMAN,
                "Robot is not AUTO. Press A / confirm AUTO before navigation.",
                started,
                {"robot_mode": context.robot_mode, "pose": pose.to_dict()},
            )
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return make_result(
                "goto_pose",
                False,
                codes.FAILED_NAV2_INACTIVE,
                "Nav2 NavigateToPose action server is not available.",
                started,
                {"pose": pose.to_dict()},
            )

        if not self._navigation_localization_ok():
            return make_result("goto_pose", False, codes.FAILED_LOCALIZATION,
                               "Navigation requires fresh accepted localization evidence.", started)

        start_pose = self._current_map_pose()
        reviewed_path = None
        if context.navigation_review is not None:
            if start_pose is None:
                return make_result("goto_pose", False, codes.FAILED_LOCALIZATION,
                                   "Cannot review route without a current map pose.", started)
            reviewed_path = self._compute_global_path(start_pose, pose)
            if not reviewed_path:
                return make_result("goto_pose", False, codes.FAILED_NAVIGATION,
                                   "No global path available for route review.", started,
                                   {"global_plan_error": self._last_global_plan_error})
            annotation = self._select_navigation_annotation(start_pose, pose, reviewed_path) if apply_navigation_annotations else None
            blocked = self._review_before_motion(context, pose, start_pose, reviewed_path, annotation,
                                                 tolerance_profile, started)
            if blocked is not None:
                return blocked
        if start_pose is not None:
            start_xy_distance = math.hypot(start_pose["x"] - pose.x, start_pose["y"] - pose.y)
            if apply_navigation_annotations:
                global_plan_points = reviewed_path if reviewed_path is not None else self._compute_global_path(start_pose, pose)
                global_plan_error = self._last_global_plan_error
                if global_plan_points is None:
                    if publish_goal_markers:
                        self._current_route_path_points = None
                        self._publish_navigation_progress(
                            "global_plan_failed",
                            pose,
                            {
                                "global_plan_error": global_plan_error,
                                "start_pose": start_pose,
                                "target_pose": pose.to_dict(),
                            },
                        )
                        self._publish_navigation_route_state(
                            "global_plan_failed",
                            self._navigation_route_items(
                                pose,
                                None,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                                active_kind="final_goal",
                                failed_kinds=["final_goal"],
                                final_status="failed",
                            ),
                            active_kind="final_goal",
                        )
                        self._clear_navigation_goal_markers()
                        self._clear_navigation_route_markers()
                    message = self._global_plan_failure_message(pose, global_plan_error)
                    return make_result(
                        "goto_pose",
                        False,
                        codes.FAILED_NAVIGATION,
                        message,
                        started,
                        {
                            "pose": pose.to_dict(),
                            "start_pose": start_pose,
                            "start_xy_distance_m": start_xy_distance,
                            "global_plan_error": global_plan_error,
                            "tolerance_profile": tolerance_profile,
                            "xy_goal_tolerance_m": task_xy_tolerance,
                            "yaw_goal_tolerance_rad": task_yaw_tolerance,
                        },
                    )
                if publish_goal_markers:
                    self._current_route_path_points = global_plan_points
                navigation_annotation_selected = self._select_navigation_annotation(
                    start_pose,
                    pose,
                    global_plan_points,
                )
                if navigation_annotation_selected is None:
                    navigation_annotation_diagnostics = self._diagnose_navigation_annotation(
                        start_pose,
                        pose,
                        global_plan_points,
                    )
                if navigation_annotation_selected is not None:
                    alignment_pose_dict = navigation_annotation_selected.get("alignment_goal_pose")
                    waypoint_goal = Pose2D.from_dict(navigation_annotation_selected["waypoint_pose"])
                    waypoint_xy_tolerance = float(
                        navigation_annotation_selected.get("waypoint_xy_goal_tolerance_m", 0.45)
                    )
                    sampled_waypoint, waypoint_sampling = self._sample_open_area_waypoint(
                        waypoint_goal,
                        pose,
                        waypoint_xy_tolerance,
                    )
                    if waypoint_sampling is not None:
                        waypoint_goal = sampled_waypoint
                        navigation_annotation_selected["waypoint_pose"] = waypoint_goal.to_dict()
                        navigation_annotation_selected["waypoint_sampling"] = waypoint_sampling
                    if publish_goal_markers:
                        self._publish_navigation_goal_markers(
                            pose,
                            navigation_annotation_selected,
                            final_tolerance_m=task_xy_tolerance,
                        )
                        self._publish_navigation_route_state(
                            "planned",
                            self._navigation_route_items(
                                pose,
                                navigation_annotation_selected,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                            ),
                            active_index=0 if alignment_pose_dict else 1,
                        )
                    if alignment_pose_dict:
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "annotation_alignment",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="alignment",
                                    reached_kinds=[],
                                ),
                                active_kind="alignment",
                            )
                        self._publish_navigation_progress(
                            "annotation_alignment",
                            pose,
                            {
                                "passage_id": navigation_annotation_selected.get("passage_id"),
                                "alignment_goal_pose": alignment_pose_dict,
                                "waypoint_pose": waypoint_goal.to_dict(),
                            },
                        )
                        alignment_goal = Pose2D.from_dict(alignment_pose_dict)
                        alignment_timeout = float(navigation_annotation_selected.get("alignment_timeout_sec", 16.0))
                        alignment_xy_tolerance = float(
                            navigation_annotation_selected.get("alignment_xy_goal_tolerance_m", 0.45)
                        )
                        alignment_yaw_tolerance = float(
                            navigation_annotation_selected.get(
                                "alignment_yaw_goal_tolerance_rad",
                                math.radians(20.0),
                            )
                        )
                        alignment_yaw_tolerance = min(
                            alignment_yaw_tolerance,
                            NARROW_PASSAGE_HEADING_TOLERANCE_RAD,
                        )
                        current_pose = self._current_map_pose() or start_pose
                        alignment_xy_error = math.hypot(
                            float(current_pose["x"]) - alignment_goal.x,
                            float(current_pose["y"]) - alignment_goal.y,
                        )
                        alignment_yaw_delta = self._normalize_angle(alignment_goal.yaw - float(current_pose["yaw"]))
                        alignment_use_policy = navigation_annotation_selected.get("alignment_use_policy") or {}
                        rotate_only_distance = float(
                            alignment_use_policy.get(
                                "skip_if_robot_near_pose_m",
                                min(0.15, alignment_xy_tolerance),
                            )
                        )
                        if alignment_xy_error <= rotate_only_distance:
                            if abs(alignment_yaw_delta) <= alignment_yaw_tolerance:
                                navigation_alignment_result = make_result(
                                    "goto_pose_annotation_alignment",
                                    True,
                                    codes.OK,
                                    "Already inside annotation alignment pose tolerance.",
                                    started,
                                    {
                                        "control_mode": "alignment_tolerance_check",
                                        "alignment_goal_pose": alignment_goal.to_dict(),
                                        "start_pose": current_pose,
                                        "xy_error_m": alignment_xy_error,
                                        "yaw_error_rad": abs(alignment_yaw_delta),
                                    },
                                )
                            else:
                                navigation_alignment_result = self._direct_rotate(
                                    "goto_pose_annotation_alignment",
                                    context,
                                    alignment_yaw_delta,
                                    min(alignment_timeout, max(3.0, abs(alignment_yaw_delta) / 0.22 + 1.0)),
                                    {
                                        "reason": "inside_alignment_zone_shortest_yaw_adjust",
                                        "alignment_goal_pose": alignment_goal.to_dict(),
                                        "xy_error_m": alignment_xy_error,
                                        "rotate_only_distance_m": rotate_only_distance,
                                        "target_yaw_error_rad": abs(alignment_yaw_delta),
                                        "yaw_goal_tolerance_rad": alignment_yaw_tolerance,
                                    },
                                    yaw_tolerance_rad=alignment_yaw_tolerance,
                                )
                        else:
                            navigation_alignment_result = self.goto_pose(
                                context,
                                alignment_goal,
                                timeout_sec=alignment_timeout,
                                xy_goal_tolerance_m=alignment_xy_tolerance,
                                yaw_goal_tolerance_rad=alignment_yaw_tolerance,
                                tolerance_profile="annotation_alignment_goal",
                                dynamic_timeout=True,
                                max_timeout_sec=max(24.0, alignment_timeout),
                                recovery_policy=recovery_policy,
                                apply_navigation_annotations=False,
                                publish_goal_markers=False,
                                alternate_goal_attempts=0,
                            )
                        if not navigation_alignment_result.success:
                            if publish_goal_markers:
                                self._publish_navigation_route_state(
                                    "annotation_alignment_failed",
                                    self._navigation_route_items(
                                        pose,
                                        navigation_annotation_selected,
                                        task_xy_tolerance,
                                        task_yaw_tolerance,
                                        active_kind="alignment",
                                        failed_kinds=["alignment"],
                                    ),
                                    active_kind="alignment",
                                )
                                self._clear_navigation_goal_markers()
                                self._clear_navigation_route_markers()
                            return make_result(
                                "goto_pose",
                                False,
                                navigation_alignment_result.code,
                                (
                                    "Navigation annotation alignment goal failed before narrow passage: "
                                    f"{navigation_alignment_result.message}"
                                ),
                                started,
                                {
                                    "pose": pose.to_dict(),
                                    "navigation_annotation": navigation_annotation_selected,
                                    "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                                    "navigation_alignment_result": navigation_alignment_result.to_dict(),
                                    "start_pose": start_pose,
                                    "start_xy_distance_m": start_xy_distance,
                                },
                            )
                        start_pose = self._current_map_pose() or start_pose
                        start_xy_distance = math.hypot(start_pose["x"] - pose.x, start_pose["y"] - pose.y)
                    reachable_waypoint, waypoint_reachability = self._sample_reachable_nav2_goal(
                        start_pose,
                        waypoint_goal,
                        pose,
                        max_radius_m=0.70,
                    )
                    navigation_annotation_selected["waypoint_reachability"] = waypoint_reachability
                    if reachable_waypoint is None:
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "annotation_waypoint_unreachable",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="waypoint",
                                    reached_kinds=["alignment"] if navigation_alignment_result else [],
                                    failed_kinds=["waypoint"],
                                ),
                                active_kind="waypoint",
                            )
                            self._clear_navigation_goal_markers()
                            self._clear_navigation_route_markers()
                        return make_result(
                            "goto_pose",
                            False,
                            codes.FAILED_NAVIGATION,
                            "Navigation annotation waypoint is not reachable from the current pose.",
                            started,
                            {
                                "pose": pose.to_dict(),
                                "navigation_annotation": navigation_annotation_selected,
                                "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                                "navigation_alignment_result": (
                                    navigation_alignment_result.to_dict() if navigation_alignment_result else None
                                ),
                                "start_pose": start_pose,
                                "start_xy_distance_m": start_xy_distance,
                            },
                        )
                    if self._pose_key(reachable_waypoint.to_dict()) != self._pose_key(waypoint_goal.to_dict()):
                        waypoint_goal = reachable_waypoint
                        navigation_annotation_selected["waypoint_pose"] = waypoint_goal.to_dict()
                        if publish_goal_markers:
                            self._publish_navigation_goal_markers(
                                pose,
                                navigation_annotation_selected,
                                final_tolerance_m=task_xy_tolerance,
                            )
                            self._publish_navigation_route_state(
                                "annotation_waypoint",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="waypoint",
                                    reached_kinds=["alignment"] if navigation_alignment_result else [],
                                ),
                                active_kind="waypoint",
                            )
                    waypoint_timeout = float(navigation_annotation_selected.get("waypoint_timeout_sec", 24.0))
                    waypoint_yaw_tolerance = float(
                        navigation_annotation_selected.get("waypoint_yaw_goal_tolerance_rad", math.radians(20.0))
                    )
                    waypoint_yaw_tolerance = min(
                        waypoint_yaw_tolerance,
                        NARROW_PASSAGE_HEADING_TOLERANCE_RAD,
                    )
                    self._publish_navigation_progress(
                        "annotation_waypoint",
                        waypoint_goal,
                        {
                            "passage_id": navigation_annotation_selected.get("passage_id"),
                            "final_goal_pose": pose.to_dict(),
                            "waypoint_reachability": waypoint_reachability,
                        },
                    )
                    if publish_goal_markers:
                        self._publish_navigation_route_state(
                            "annotation_waypoint",
                            self._navigation_route_items(
                                pose,
                                navigation_annotation_selected,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                                active_kind="waypoint",
                                reached_kinds=["alignment"] if navigation_alignment_result else [],
                            ),
                            active_kind="waypoint",
                        )
                    traversal_policy = navigation_annotation_selected.get("traversal_policy") or {}
                    self._publish_narrow_motion_profile(
                        navigation_annotation_selected,
                        waypoint_goal,
                        traversal_policy,
                    )
                    try:
                        navigation_annotation_result = self.goto_pose(
                            context,
                            waypoint_goal,
                            timeout_sec=waypoint_timeout,
                            xy_goal_tolerance_m=waypoint_xy_tolerance,
                            yaw_goal_tolerance_rad=waypoint_yaw_tolerance,
                            tolerance_profile="annotation_waypoint",
                            dynamic_timeout=True,
                            max_timeout_sec=max(30.0, waypoint_timeout),
                            recovery_policy=recovery_policy,
                            apply_navigation_annotations=False,
                            publish_goal_markers=False,
                            alternate_goal_attempts=0,
                        )
                    finally:
                        self._clear_narrow_motion_profile()
                    if not navigation_annotation_result.success:
                        navigation_annotation_selected["waypoint_failure_action"] = "fail_before_final_goal"
                        navigation_annotation_selected["waypoint_failure_code"] = navigation_annotation_result.code
                        navigation_annotation_selected["waypoint_failure_message"] = navigation_annotation_result.message
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "annotation_waypoint_failed",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="waypoint",
                                    reached_kinds=["alignment"] if navigation_alignment_result else [],
                                    failed_kinds=["waypoint"],
                                ),
                                active_kind="waypoint",
                            )
                            self._clear_navigation_goal_markers()
                            self._clear_navigation_route_markers()
                        return make_result(
                            "goto_pose",
                            False,
                            navigation_annotation_result.code,
                            (
                                "Navigation annotation waypoint failed before final goal: "
                                f"{navigation_annotation_result.message}"
                            ),
                            started,
                            {
                                "pose": pose.to_dict(),
                                "navigation_annotation": navigation_annotation_selected,
                                "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                                "navigation_alignment_result": (
                                    navigation_alignment_result.to_dict() if navigation_alignment_result else None
                                ),
                                "navigation_annotation_result": navigation_annotation_result.to_dict(),
                                "start_pose": start_pose,
                                "start_xy_distance_m": start_xy_distance,
                            },
                        )
                    else:
                        start_pose = self._current_map_pose() or start_pose
                        start_xy_distance = math.hypot(start_pose["x"] - pose.x, start_pose["y"] - pose.y)
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "final_goal",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="final_goal",
                                    reached_kinds=["alignment", "waypoint"],
                                ),
                                active_kind="final_goal",
                            )
                elif publish_goal_markers:
                    self._publish_navigation_goal_markers(pose, None, final_tolerance_m=task_xy_tolerance)
                    self._publish_navigation_route_state(
                        "final_goal",
                        self._navigation_route_items(pose, None, task_xy_tolerance, task_yaw_tolerance),
                        active_kind="final_goal",
                    )
            elif publish_goal_markers:
                self._current_route_path_points = reviewed_path if reviewed_path is not None else self._compute_global_path(start_pose, pose)
                global_plan_error = self._last_global_plan_error
                if self._current_route_path_points is None:
                    self._publish_navigation_progress(
                        "global_plan_failed",
                        pose,
                        {
                            "global_plan_error": global_plan_error,
                            "start_pose": start_pose,
                            "target_pose": pose.to_dict(),
                        },
                    )
                    self._publish_navigation_route_state(
                        "global_plan_failed",
                        self._navigation_route_items(
                            pose,
                            None,
                            task_xy_tolerance,
                            task_yaw_tolerance,
                            active_kind="final_goal",
                            failed_kinds=["final_goal"],
                            final_status="failed",
                        ),
                        active_kind="final_goal",
                    )
                    self._clear_navigation_goal_markers()
                    self._clear_navigation_route_markers()
                    message = self._global_plan_failure_message(pose, global_plan_error)
                    return make_result(
                        "goto_pose",
                        False,
                        codes.FAILED_NAVIGATION,
                        message,
                        started,
                        {
                            "pose": pose.to_dict(),
                            "start_pose": start_pose,
                            "start_xy_distance_m": start_xy_distance,
                            "global_plan_error": global_plan_error,
                            "tolerance_profile": tolerance_profile,
                            "xy_goal_tolerance_m": task_xy_tolerance,
                            "yaw_goal_tolerance_rad": task_yaw_tolerance,
                        },
                    )
                self._publish_navigation_goal_markers(pose, None, final_tolerance_m=task_xy_tolerance)
                self._publish_navigation_route_state(
                    "final_goal",
                    self._navigation_route_items(pose, None, task_xy_tolerance, task_yaw_tolerance),
                    active_kind="final_goal",
                )
        elif publish_goal_markers:
            self._publish_navigation_goal_markers(pose, None, final_tolerance_m=task_xy_tolerance)
            self._publish_navigation_route_state(
                "final_goal",
                self._navigation_route_items(pose, None, task_xy_tolerance, task_yaw_tolerance),
                active_kind="final_goal",
            )

        # Shared waypoint baselines dispatch the same Nav2 goals without extra direct motion.
        if start_pose is not None and recovery_policy.get("allow_direct_preparation", True):
            pre_align_heading_delta = self._target_heading_delta(start_pose, pose)
            local_clearance = self._local_clearance_snapshot()
            if (
                start_xy_distance is not None
                and start_xy_distance >= 0.8
                and local_clearance
                and local_clearance.get("front_clear") is True
                and local_clearance.get("sides_congested") is True
            ):
                local_passage_result = self._direct_translate(
                    "goto_pose_local_passage_forward",
                    context,
                    0.35,
                    0.0,
                    5.0,
                    {
                        "reason": "front_clear_sides_congested",
                        "local_clearance": local_clearance,
                        "start_pose": start_pose,
                        "target_pose": pose.to_dict(),
                    },
                )
                start_pose = self._current_map_pose() or start_pose
                start_xy_distance = math.hypot(start_pose["x"] - pose.x, start_pose["y"] - pose.y)
                pre_align_heading_delta = self._target_heading_delta(start_pose, pose)
                local_clearance = self._local_clearance_snapshot()
            if (
                start_xy_distance is not None
                and start_xy_distance >= 0.8
                and pre_align_heading_delta is not None
                and abs(pre_align_heading_delta) >= math.radians(45.0)
            ):
                rotation_clear = None if local_clearance is None else local_clearance.get("rotation_clear")
                if rotation_clear is False:
                    pre_align_skip_reason = "rotation_clearance_blocked"
                else:
                    pre_align_reason = "long_distance_face_travel_direction"
                    pre_align_result = self._direct_rotate(
                        "goto_pose_pre_align_heading",
                        context,
                        pre_align_heading_delta,
                        min(8.0, max(3.0, abs(pre_align_heading_delta) / 0.25 + 1.0)),
                        {
                            "reason": pre_align_reason,
                            "target_heading_delta_rad": pre_align_heading_delta,
                            "trigger_distance_m": 0.8,
                            "trigger_heading_delta_rad": math.radians(45.0),
                            "local_clearance": local_clearance,
                            "start_pose": start_pose,
                            "target_pose": pose.to_dict(),
                        },
                    )
                    start_pose = self._current_map_pose() or start_pose
        if dynamic_timeout and start_xy_distance is not None:
            effective_timeout = self._estimate_navigation_timeout(
                start_xy_distance,
                requested_timeout,
                float(max_timeout_sec),
            )
            timeout_policy = "distance_estimated"

        self._publish_navigation_progress(
            "final_goal",
            pose,
            {
                "timeout_policy": timeout_policy,
                "requested_timeout_sec": requested_timeout,
                "effective_timeout_sec": effective_timeout,
                "start_xy_distance_m": start_xy_distance,
                "tolerance_profile": tolerance_profile,
                "xy_goal_tolerance_m": task_xy_tolerance,
                "yaw_goal_tolerance_rad": task_yaw_tolerance,
                "navigation_annotation": navigation_annotation_selected,
                "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
            },
        )

        if getattr(context, "recovery_check", lambda: False)():
            return make_result("goto_pose", False, codes.CANCELLED, "Navigation held before Nav2 dispatch by Recovery Agent.", started)
        goal = NavigateToPose.Goal()
        goal.pose = pose_to_stamped(self.node, pose)
        send_done = threading.Event()
        result_done = threading.Event()
        holder: dict[str, Any] = {"accepted": False, "result": None, "error": None}

        def on_result(future):
            try:
                holder["result"] = future.result()
            except Exception as exc:
                holder["error"] = repr(exc)
            result_done.set()

        def on_goal(future):
            try:
                goal_handle = future.result()
                self._active_goal_handle = goal_handle
                holder["accepted"] = bool(goal_handle.accepted)
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    self._nav_result_futures.append(result_future)
                    result_future.add_done_callback(on_result)
                    if (self.stopping or getattr(context, "cancel_check", lambda: False)()
                            or getattr(context, "recovery_check", lambda: False)()):
                        goal_handle.cancel_goal_async()
                else:
                    result_done.set()
            except Exception as exc:
                holder["error"] = repr(exc)
                result_done.set()
            finally:
                self._pending_nav_goals -= 1
            send_done.set()

        self._nav_result_futures = [f for f in self._nav_result_futures if not f.done()]
        self._pending_nav_goals += 1
        try:
            self._action_client.send_goal_async(goal).add_done_callback(on_goal)
        except Exception:
            self._pending_nav_goals -= 1
            raise

        if not send_done.wait(timeout=5.0):
            if publish_goal_markers:
                self._clear_navigation_goal_markers()
                self._clear_navigation_route_markers()
            return make_result("goto_pose", False, codes.FAILED_TIMEOUT, "Timed out sending Nav2 goal.", started)
        if holder.get("error"):
            if publish_goal_markers:
                self._clear_navigation_goal_markers()
                self._clear_navigation_route_markers()
            return make_result("goto_pose", False, codes.FAILED_NAVIGATION, holder["error"], started)
        if not holder["accepted"]:
            if publish_goal_markers:
                self._clear_navigation_goal_markers()
                self._clear_navigation_route_markers()
            return make_result("goto_pose", False, codes.FAILED_NAVIGATION, "Nav2 goal was rejected.", started)

        hard_deadline = time.monotonic() + (max_timeout_sec if dynamic_timeout else effective_timeout)
        progress_watchdog_sec = effective_timeout
        last_progress_check_time = time.monotonic()
        last_progress_reset_time = last_progress_check_time
        last_progress_distance = start_xy_distance
        last_progress_pose = start_pose
        visited_progress_cells = set()
        if start_pose:
            visited_progress_cells.add((round(start_pose["x"] / .1), round(start_pose["y"] / .1)))
        best_goal_yaw_error = near_goal_yaw_error(start_pose, pose, task_xy_tolerance)
        best_distance = start_xy_distance
        progress_reset_count = 0
        last_progress_publish_time = 0.0
        staged_recovery: dict[str, Any] = {
            "micro_adjust": {"attempted": False},
            "relaxed_tolerance": {"attempted": False},
            "goal_change": {"attempted": False},
        }
        while not result_done.wait(timeout=0.2):
            if getattr(context, "recovery_check", lambda: False)():
                self.cancel_navigation(context)
                return make_result("goto_pose", False, codes.CANCELLED, "Navigation paused by Recovery Agent.", started,
                                   {"recovery_pause": True})
            if not self._navigation_localization_ok():
                self.cancel_navigation(context)
                self._publish_stop()
                return make_result("goto_pose", False, codes.FAILED_LOCALIZATION,
                                   "Navigation stopped: accepted localization evidence is unhealthy.", started,
                                   {"pose": pose.to_dict(), "stop_reason": "localization_unhealthy"})
            if self._task_goal_reached(pose, task_xy_tolerance, task_yaw_tolerance):
                self.cancel_navigation(context)
                final_pose = self._current_map_pose()
                xy_error = None
                yaw_error = None
                if final_pose is not None:
                    xy_error = math.hypot(final_pose["x"] - pose.x, final_pose["y"] - pose.y)
                    yaw_error = abs(self._normalize_angle(final_pose["yaw"] - pose.yaw))
                if publish_goal_markers:
                    self._publish_navigation_route_state(
                        "task_tolerance_reached",
                        self._navigation_route_items(
                            pose,
                            navigation_annotation_selected,
                            task_xy_tolerance,
                            task_yaw_tolerance,
                            reached_kinds=["alignment", "waypoint", "final_goal"],
                            final_status="reached",
                        ),
                        active_kind="final_goal",
                    )
                return make_result(
                    "goto_pose",
                    True,
                    codes.OK,
                    "Navigation accepted by task tolerance before Nav2 strict goal completion.",
                    started,
                    {
                        "nav2_status": "cancelled_after_task_tolerance_reached",
                        "pose": pose.to_dict(),
                        "duration_sec": time.monotonic() - started_monotonic,
                        "timeout_policy": timeout_policy,
                        "requested_timeout_sec": requested_timeout,
                        "effective_timeout_sec": effective_timeout,
                        "timeout_extensions": timeout_extensions,
                        "navigation_annotation": navigation_annotation_selected,
                        "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                        "navigation_alignment_result": (
                            navigation_alignment_result.to_dict() if navigation_alignment_result else None
                        ),
                        "navigation_annotation_result": (
                            navigation_annotation_result.to_dict() if navigation_annotation_result else None
                        ),
                        "local_clearance": local_clearance,
                        "local_passage_result": local_passage_result.to_dict() if local_passage_result else None,
                        "pre_align_reason": pre_align_reason,
                        "pre_align_skip_reason": pre_align_skip_reason,
                        "pre_align_heading_delta_rad": pre_align_heading_delta,
                        "pre_align_result": pre_align_result.to_dict() if pre_align_result else None,
                        "staged_recovery": staged_recovery,
                        "start_pose": start_pose,
                        "start_xy_distance_m": start_xy_distance,
                        "final_pose": final_pose,
                        "final_xy_error_m": xy_error,
                        "final_yaw_error_rad": yaw_error,
                        "tolerance_profile": tolerance_profile,
                        "xy_goal_tolerance_m": task_xy_tolerance,
                        "yaw_goal_tolerance_rad": task_yaw_tolerance,
                    },
                )
            now_monotonic = time.monotonic()
            if now_monotonic - last_progress_publish_time >= 1.0:
                current = self._current_map_pose()
                current_distance = None
                if current is not None:
                    current_distance = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
                self._publish_navigation_progress(
                    "final_goal",
                    pose,
                    {
                        "timeout_policy": timeout_policy,
                        "requested_timeout_sec": requested_timeout,
                        "effective_timeout_sec": effective_timeout,
                        "progress_watchdog_sec": progress_watchdog_sec if dynamic_timeout else None,
                        "progress_check_interval_sec": PROGRESS_CHECK_INTERVAL_SEC if dynamic_timeout else None,
                        "stalled_sec": (now_monotonic - last_progress_reset_time) if dynamic_timeout else None,
                        "progress_reset_count": progress_reset_count if dynamic_timeout else None,
                        "staged_recovery": staged_recovery,
                        "current_distance_m": current_distance,
                        "best_distance_m": best_distance,
                        "tolerance_profile": tolerance_profile,
                    },
                )
                last_progress_publish_time = now_monotonic
            if dynamic_timeout and now_monotonic - last_progress_check_time >= PROGRESS_CHECK_INTERVAL_SEC:
                current = self._current_map_pose()
                if current is not None:
                    current_distance = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
                    # Final heading alignment is progress even with no XY movement.
                    # Update before staged recovery can relax a still-progressing goal.
                    best_goal_yaw_error, heading_improved = yaw_progress_reference(
                        best_goal_yaw_error, near_goal_yaw_error(current, pose, task_xy_tolerance)
                    )
                    if heading_improved:
                        last_progress_reset_time = now_monotonic
                        progress_reset_count += 1
                        timeout_policy = "progress_watchdog"
                        staged_recovery["heading_progress_resets"] = staged_recovery.get("heading_progress_resets", 0) + 1
                    staged_elapsed = now_monotonic - last_progress_reset_time
                    staged_recovery["stalled_sec"] = staged_elapsed
                    staged_recovery["micro_adjust_threshold_sec"] = progress_watchdog_sec / 3.0
                    staged_recovery["relaxed_tolerance_threshold_sec"] = progress_watchdog_sec * 2.0 / 3.0
                    staged_recovery["goal_change_threshold_sec"] = progress_watchdog_sec
                    if (
                        not staged_recovery["micro_adjust"]["attempted"]
                        and staged_elapsed >= progress_watchdog_sec / 3.0
                    ):
                        staged_recovery["micro_adjust"]["attempted"] = True
                        staged_recovery["micro_adjust"]["trigger_stalled_sec"] = staged_elapsed
                        staged_recovery["micro_adjust"]["current_distance_m"] = current_distance
                        if current_distance <= 1.0:
                            near_goal_recovery = self._try_near_goal_micro_adjust(
                                context,
                                pose,
                                current,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                            )
                            staged_recovery["micro_adjust"]["result"] = (
                                near_goal_recovery.to_dict() if near_goal_recovery else None
                            )
                            self._publish_navigation_progress(
                                "near_goal_micro_adjust",
                                pose,
                                {
                                    "near_goal_recovery": staged_recovery["micro_adjust"]["result"],
                                    "current_distance_m": current_distance,
                                    "trigger_stalled_sec": staged_elapsed,
                                    "timeout_threshold_sec": progress_watchdog_sec / 3.0,
                                },
                            )
                            refreshed = self._current_map_pose()
                            if refreshed is not None:
                                current = refreshed
                                current_distance = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
                            if near_goal_recovery and near_goal_recovery.success and self._task_goal_reached(
                                pose,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                            ):
                                self.cancel_navigation(context)
                                final_pose = self._current_map_pose()
                                if publish_goal_markers:
                                    self._publish_navigation_route_state(
                                        "near_goal_micro_adjust_success",
                                        self._navigation_route_items(
                                            pose,
                                            navigation_annotation_selected,
                                            task_xy_tolerance,
                                            task_yaw_tolerance,
                                            reached_kinds=["alignment", "waypoint", "final_goal"],
                                            final_status="reached",
                                        ),
                                        active_kind="final_goal",
                                    )
                                return make_result(
                                    "goto_pose",
                                    True,
                                    codes.OK,
                                    "Navigation recovered by staged near-goal micro-adjust.",
                                    started,
                                    {
                                        "nav2_status": "staged_micro_adjust_success",
                                        "pose": pose.to_dict(),
                                        "duration_sec": time.monotonic() - started_monotonic,
                                        "timeout_policy": timeout_policy,
                                        "requested_timeout_sec": requested_timeout,
                                        "effective_timeout_sec": effective_timeout,
                                        "timeout_extensions": timeout_extensions,
                                        "navigation_annotation": navigation_annotation_selected,
                                        "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                                        "navigation_alignment_result": (
                                            navigation_alignment_result.to_dict()
                                            if navigation_alignment_result
                                            else None
                                        ),
                                        "navigation_annotation_result": (
                                            navigation_annotation_result.to_dict()
                                            if navigation_annotation_result
                                            else None
                                        ),
                                        "local_clearance": local_clearance,
                                        "local_passage_result": (
                                            local_passage_result.to_dict() if local_passage_result else None
                                        ),
                                        "pre_align_reason": pre_align_reason,
                                        "pre_align_skip_reason": pre_align_skip_reason,
                                        "pre_align_heading_delta_rad": pre_align_heading_delta,
                                        "pre_align_result": pre_align_result.to_dict() if pre_align_result else None,
                                        "progress_watchdog_sec": progress_watchdog_sec,
                                        "progress_check_interval_sec": PROGRESS_CHECK_INTERVAL_SEC,
                                        "progress_reset_count": progress_reset_count,
                                        "best_distance_m": best_distance,
                                        "staged_recovery": staged_recovery,
                                        "start_pose": start_pose,
                                        "start_xy_distance_m": start_xy_distance,
                                        "final_pose": final_pose,
                                        "tolerance_profile": tolerance_profile,
                                        "xy_goal_tolerance_m": task_xy_tolerance,
                                        "yaw_goal_tolerance_rad": task_yaw_tolerance,
                                    },
                                )
                        else:
                            staged_recovery["micro_adjust"]["skipped_reason"] = "current_distance_over_1m"
                    if (
                        not staged_recovery["relaxed_tolerance"]["attempted"]
                        and staged_elapsed >= progress_watchdog_sec * 2.0 / 3.0
                    ):
                        staged_recovery["relaxed_tolerance"]["attempted"] = True
                        staged_recovery["relaxed_tolerance"]["trigger_stalled_sec"] = staged_elapsed
                        staged_recovery["relaxed_tolerance"]["current_distance_m"] = current_distance
                        if allow_relaxed_tolerance and self._pose_within_relaxed_region_tolerance(current, pose):
                            self.cancel_navigation(context)
                            staged_recovery["relaxed_tolerance"]["accepted"] = True
                            self._publish_navigation_progress(
                                "relaxed_tolerance_success",
                                pose,
                                {
                                    "final_xy_error_m": current_distance,
                                    "relaxed_tolerance": {"xy_goal_tolerance_m": 0.65, "yaw_ignored": True},
                                    "trigger_stalled_sec": staged_elapsed,
                                    "timeout_threshold_sec": progress_watchdog_sec * 2.0 / 3.0,
                                },
                            )
                            if publish_goal_markers:
                                self._publish_navigation_route_state(
                                    "relaxed_tolerance_success",
                                    self._navigation_route_items(
                                        pose,
                                        navigation_annotation_selected,
                                        task_xy_tolerance,
                                        task_yaw_tolerance,
                                        reached_kinds=["alignment", "waypoint"],
                                        final_status="accepted_relaxed",
                                    ),
                                    active_kind="final_goal",
                                )
                            return make_result(
                                "goto_pose",
                                True,
                                codes.OK,
                                "Navigation accepted by staged relaxed region tolerance.",
                                started,
                                {
                                    "nav2_status": "staged_relaxed_tolerance_success",
                                    "pose": pose.to_dict(),
                                    "duration_sec": time.monotonic() - started_monotonic,
                                    "timeout_policy": timeout_policy,
                                    "requested_timeout_sec": requested_timeout,
                                    "effective_timeout_sec": effective_timeout,
                                    "timeout_extensions": timeout_extensions,
                                    "navigation_annotation": navigation_annotation_selected,
                                    "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                                    "navigation_alignment_result": (
                                        navigation_alignment_result.to_dict() if navigation_alignment_result else None
                                    ),
                                    "navigation_annotation_result": (
                                        navigation_annotation_result.to_dict()
                                        if navigation_annotation_result
                                        else None
                                    ),
                                    "local_clearance": local_clearance,
                                    "local_passage_result": (
                                        local_passage_result.to_dict() if local_passage_result else None
                                    ),
                                    "pre_align_reason": pre_align_reason,
                                    "pre_align_skip_reason": pre_align_skip_reason,
                                    "pre_align_heading_delta_rad": pre_align_heading_delta,
                                    "pre_align_result": pre_align_result.to_dict() if pre_align_result else None,
                                    "progress_watchdog_sec": progress_watchdog_sec,
                                    "progress_check_interval_sec": PROGRESS_CHECK_INTERVAL_SEC,
                                    "progress_reset_count": progress_reset_count,
                                    "best_distance_m": best_distance,
                                    "staged_recovery": staged_recovery,
                                    "start_pose": start_pose,
                                    "start_xy_distance_m": start_xy_distance,
                                    "final_pose": current,
                                    "final_xy_error_m": current_distance,
                                    "tolerance_profile": tolerance_profile,
                                    "xy_goal_tolerance_m": task_xy_tolerance,
                                    "yaw_goal_tolerance_rad": task_yaw_tolerance,
                                    "relaxed_tolerance": {"xy_goal_tolerance_m": 0.65, "yaw_ignored": True},
                                },
                            )
                        staged_recovery["relaxed_tolerance"]["accepted"] = False
                        if not allow_relaxed_tolerance:
                            staged_recovery["relaxed_tolerance"]["skipped_reason"] = "policy_disallows_relaxed_tolerance"
                        else:
                            staged_recovery["relaxed_tolerance"]["skipped_reason"] = "outside_relaxed_tolerance"
                    pose_delta = None
                    if last_progress_pose is not None:
                        pose_delta = math.hypot(
                            current["x"] - last_progress_pose["x"],
                            current["y"] - last_progress_pose["y"],
                        )
                    distance_improved = best_distance is not None and best_distance - current_distance >= 0.10
                    cell = (round(current["x"] / .1), round(current["y"] / .1))
                    robot_moved = pose_delta is not None and pose_delta >= 0.08 and cell not in visited_progress_cells
                    if distance_improved or robot_moved:
                        visited_progress_cells.add(cell)
                        last_progress_reset_time = now_monotonic
                        last_progress_distance = current_distance
                        last_progress_pose = current
                        best_distance = current_distance if best_distance is None else min(best_distance, current_distance)
                        progress_reset_count += 1
                        timeout_policy = "progress_watchdog"
                    elif now_monotonic - last_progress_reset_time >= progress_watchdog_sec:
                        staged_recovery["goal_change"]["attempted"] = True
                        staged_recovery["goal_change"]["trigger_stalled_sec"] = now_monotonic - last_progress_reset_time
                        timeout_policy = "progress_watchdog_stalled"
                        break
                last_progress_check_time = now_monotonic
            if hard_deadline is not None and time.monotonic() >= hard_deadline:
                break

        if not self._navigation_localization_ok():
            self.cancel_navigation(context)
            self._publish_stop()
            return make_result("goto_pose", False, codes.FAILED_LOCALIZATION,
                               "Localization became unhealthy before navigation completion.", started)
        if not result_done.is_set():
            self.cancel_navigation(context)
            final_pose = self._current_map_pose()
            timeout_local_clearance = self._local_clearance_snapshot()
            if timeout_local_clearance is not None:
                local_clearance = timeout_local_clearance
            metrics = {
                "timeout_sec": effective_timeout,
                "requested_timeout_sec": requested_timeout,
                "effective_timeout_sec": effective_timeout,
                "timeout_policy": timeout_policy,
                "timeout_extensions": timeout_extensions,
                "navigation_annotation": navigation_annotation_selected,
                "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
                "navigation_alignment_result": (
                    navigation_alignment_result.to_dict() if navigation_alignment_result else None
                ),
                "navigation_annotation_result": (
                    navigation_annotation_result.to_dict() if navigation_annotation_result else None
                ),
                "local_clearance": local_clearance,
                "local_passage_result": local_passage_result.to_dict() if local_passage_result else None,
                "pre_align_reason": pre_align_reason,
                "pre_align_skip_reason": pre_align_skip_reason,
                "pre_align_heading_delta_rad": pre_align_heading_delta,
                "pre_align_result": pre_align_result.to_dict() if pre_align_result else None,
                "staged_recovery": staged_recovery,
                "progress_watchdog_sec": progress_watchdog_sec if dynamic_timeout else None,
                "progress_check_interval_sec": PROGRESS_CHECK_INTERVAL_SEC if dynamic_timeout else None,
                "progress_reset_count": progress_reset_count if dynamic_timeout else None,
                "best_distance_m": best_distance,
                "stalled_sec": (time.monotonic() - last_progress_reset_time) if dynamic_timeout else None,
                "pose": pose.to_dict(),
                "start_pose": start_pose,
                "start_xy_distance_m": start_xy_distance,
                "final_pose": final_pose,
                "tolerance_profile": tolerance_profile,
                "xy_goal_tolerance_m": task_xy_tolerance,
                "yaw_goal_tolerance_rad": task_yaw_tolerance,
            }
            if final_pose is not None:
                metrics["final_xy_error_m"] = math.hypot(final_pose["x"] - pose.x, final_pose["y"] - pose.y)
                metrics["final_yaw_error_rad"] = abs(self._normalize_angle(final_pose["yaw"] - pose.yaw))
                metrics["recovery_policy"] = recovery_policy
                metrics["allow_relaxed_tolerance"] = allow_relaxed_tolerance
                metrics["allow_alternate_goal"] = allow_alternate_goal
                if self._pose_within_task_tolerance(
                    final_pose,
                    pose,
                    task_xy_tolerance,
                    task_yaw_tolerance,
                ):
                    metrics["nav2_status"] = "timeout_cancelled_after_task_tolerance_reached"
                    if publish_goal_markers:
                        self._publish_navigation_route_state(
                            "task_tolerance_reached",
                            self._navigation_route_items(
                                pose,
                                navigation_annotation_selected,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                                reached_kinds=["alignment", "waypoint", "final_goal"],
                                final_status="reached",
                            ),
                            active_kind="final_goal",
                        )
                    return make_result(
                        "goto_pose",
                        True,
                        codes.OK,
                        "Navigation reached task tolerance before Nav2 strict goal completion.",
                        started,
                        metrics,
                    )
                near_goal_recovery = self._try_near_goal_micro_adjust(
                    context,
                    pose,
                    final_pose,
                    task_xy_tolerance,
                    task_yaw_tolerance,
                )
                if near_goal_recovery is not None:
                    self._publish_navigation_progress(
                        "near_goal_micro_adjust",
                        pose,
                        {
                            "near_goal_recovery": near_goal_recovery.to_dict(),
                            "final_xy_error_m": metrics.get("final_xy_error_m"),
                            "final_yaw_error_rad": metrics.get("final_yaw_error_rad"),
                        },
                    )
                    metrics["near_goal_recovery"] = near_goal_recovery.to_dict()
                    if near_goal_recovery.success and self._task_goal_reached(
                        pose,
                        task_xy_tolerance,
                        task_yaw_tolerance,
                    ):
                        metrics["nav2_status"] = "timeout_recovered_by_near_goal_micro_adjust"
                        update_recovered_pose_metrics(
                            metrics, pose, self._current_map_pose(), "near_goal_micro_adjust")
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "near_goal_micro_adjust_success",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    reached_kinds=["alignment", "waypoint", "final_goal"],
                                    final_status="reached",
                                ),
                                active_kind="final_goal",
                            )
                        return make_result(
                            "goto_pose",
                            True,
                            codes.OK,
                            "Navigation recovered by near-goal direct micro-adjust.",
                            started,
                            metrics,
                        )
                if allow_relaxed_tolerance and self._pose_within_relaxed_region_tolerance(final_pose, pose):
                    self._publish_navigation_progress(
                        "relaxed_tolerance_success",
                        pose,
                        {
                            "final_xy_error_m": metrics.get("final_xy_error_m"),
                            "final_yaw_error_rad": metrics.get("final_yaw_error_rad"),
                            "relaxed_tolerance": {"xy_goal_tolerance_m": 0.65, "yaw_ignored": True},
                        },
                    )
                    metrics["nav2_status"] = "timeout_accepted_by_region_relaxed_tolerance"
                    metrics["relaxed_tolerance"] = {"xy_goal_tolerance_m": 0.65, "yaw_ignored": True}
                    if publish_goal_markers:
                        self._publish_navigation_route_state(
                            "relaxed_tolerance_success",
                            self._navigation_route_items(
                                pose,
                                navigation_annotation_selected,
                                task_xy_tolerance,
                                task_yaw_tolerance,
                                reached_kinds=["alignment", "waypoint"],
                                final_status="accepted_relaxed",
                            ),
                            active_kind="final_goal",
                        )
                    return make_result(
                        "goto_pose",
                        True,
                        codes.OK,
                        "Navigation accepted by relaxed region tolerance.",
                        started,
                        metrics,
                    )
                if allow_alternate_goal and alternate_goal_attempts > 0:
                    alternate_pose, alternate_sampling = self._sample_trusted_front_open_area_goal(
                        pose,
                        final_pose,
                    )
                    metrics["alternate_goal_sampling"] = alternate_sampling
                    if alternate_pose is not None:
                        self._publish_navigation_progress(
                            "alternate_goal_approach",
                            alternate_pose,
                            {
                                "original_goal_pose": pose.to_dict(),
                                "alternate_goal_sampling": alternate_sampling,
                            },
                        )
                        if publish_goal_markers:
                            self._publish_navigation_goal_markers(
                                pose,
                                {
                                    "waypoint_pose": alternate_pose.to_dict(),
                                    "reason": "local_costmap_trusted_front_alternate_goal",
                                },
                                final_tolerance_m=task_xy_tolerance,
                            )
                            self._publish_navigation_route_state(
                                "alternate_goal_approach",
                                self._navigation_route_items(
                                    pose,
                                    {
                                        "waypoint_pose": alternate_pose.to_dict(),
                                        "waypoint_xy_goal_tolerance_m": max(task_xy_tolerance or 0.25, 0.35),
                                        "reason": "local_costmap_trusted_front_alternate_goal",
                                    },
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="waypoint",
                                    reached_kinds=["alignment"] if navigation_alignment_result else [],
                                ),
                                active_kind="waypoint",
                            )
                        alternate_result = self.goto_pose(
                            context,
                            alternate_pose,
                            timeout_sec=min(24.0, effective_timeout),
                            xy_goal_tolerance_m=max(task_xy_tolerance or 0.25, 0.35),
                            yaw_goal_tolerance_rad=task_yaw_tolerance,
                            tolerance_profile=f"{tolerance_profile}_local_costmap_alternate",
                            dynamic_timeout=True,
                            max_timeout_sec=min(30.0, max_timeout_sec),
                            recovery_policy=recovery_policy,
                            apply_navigation_annotations=False,
                            publish_goal_markers=False,
                            alternate_goal_attempts=alternate_goal_attempts - 1,
                        )
                        metrics["alternate_goal_result"] = alternate_result.to_dict()
                        if alternate_result.success:
                            update_recovered_pose_metrics(
                                metrics, pose, alternate_result.metrics.get("final_pose"),
                                "alternate_goal_result")
                            # A local escape is an intermediate point, not the requested destination.
                            if metrics.get("final_pose") is None or not self._pose_within_task_tolerance(
                                metrics.get("final_pose"), pose, task_xy_tolerance, task_yaw_tolerance
                            ):
                                metrics["original_goal_reached"] = False
                                return make_result(
                                    "goto_pose", False, codes.FAILED_NAVIGATION,
                                    "Local alternate goal reached, but the original navigation goal remains unreached.",
                                    started, metrics,
                                )
                            metrics["original_goal_reached"] = True
                            if publish_goal_markers:
                                self._publish_navigation_route_state(
                                    "alternate_goal_success",
                                    self._navigation_route_items(
                                        pose,
                                        navigation_annotation_selected,
                                        task_xy_tolerance,
                                        task_yaw_tolerance,
                                        reached_kinds=["alignment", "waypoint", "final_goal"],
                                        final_status="reached",
                                    ),
                                    active_kind="final_goal",
                                )
                            return make_result(
                                "goto_pose",
                                True,
                                codes.OK,
                                "Navigation recovered by local-costmap-aware alternate region goal.",
                                started,
                                metrics,
                            )
                if (
                    not allow_alternate_goal
                    and alternate_goal_attempts > 0
                    and self._local_clearance_indicates_blocked(local_clearance)
                ):
                    escape_pose, escape_sampling = self._sample_trusted_front_open_area_goal(
                        pose,
                        final_pose,
                    )
                    metrics["obstacle_escape_sampling"] = escape_sampling
                    if escape_pose is not None:
                        self._publish_navigation_progress(
                            "obstacle_escape_waypoint",
                            escape_pose,
                            {
                                "original_goal_pose": pose.to_dict(),
                                "obstacle_escape_sampling": escape_sampling,
                                "reason": "fixed_goal_blocked_local_costmap_escape",
                            },
                        )
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "obstacle_escape_waypoint",
                                self._navigation_route_items(
                                    pose,
                                    {
                                        "waypoint_pose": escape_pose.to_dict(),
                                        "waypoint_xy_goal_tolerance_m": max(task_xy_tolerance or 0.25, 0.35),
                                        "reason": "fixed_goal_blocked_local_costmap_escape",
                                    },
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    active_kind="waypoint",
                                    reached_kinds=["alignment"] if navigation_alignment_result else [],
                                ),
                                active_kind="waypoint",
                            )
                        escape_result = self.goto_pose(
                            context,
                            escape_pose,
                            timeout_sec=min(20.0, effective_timeout),
                            xy_goal_tolerance_m=max(task_xy_tolerance or 0.25, 0.35),
                            yaw_goal_tolerance_rad=task_yaw_tolerance,
                            tolerance_profile=f"{tolerance_profile}_obstacle_escape",
                            dynamic_timeout=True,
                            max_timeout_sec=min(24.0, max_timeout_sec),
                            recovery_policy=recovery_policy,
                            apply_navigation_annotations=False,
                            publish_goal_markers=False,
                            alternate_goal_attempts=0,
                        )
                        metrics["obstacle_escape_result"] = escape_result.to_dict()
                        if escape_result.success:
                            if publish_goal_markers:
                                self._publish_navigation_route_state(
                                    "final_goal_after_obstacle_escape",
                                    self._navigation_route_items(
                                        pose,
                                        navigation_annotation_selected,
                                        task_xy_tolerance,
                                        task_yaw_tolerance,
                                        active_kind="final_goal",
                                        reached_kinds=["alignment", "waypoint"],
                                    ),
                                    active_kind="final_goal",
                                )
                            retry_result = self.goto_pose(
                                context,
                                pose,
                                timeout_sec=min(requested_timeout, max(18.0, effective_timeout * 0.75)),
                                xy_goal_tolerance_m=task_xy_tolerance,
                                yaw_goal_tolerance_rad=task_yaw_tolerance,
                                tolerance_profile=tolerance_profile,
                                dynamic_timeout=dynamic_timeout,
                                max_timeout_sec=max_timeout_sec,
                                recovery_policy=recovery_policy,
                                apply_navigation_annotations=False,
                                publish_goal_markers=False,
                                alternate_goal_attempts=alternate_goal_attempts - 1,
                            )
                            metrics["obstacle_escape_final_retry_result"] = retry_result.to_dict()
                            if retry_result.success:
                                update_recovered_pose_metrics(
                                    metrics, pose, retry_result.metrics.get("final_pose"),
                                    "obstacle_escape_final_retry_result")
                                if publish_goal_markers:
                                    self._publish_navigation_route_state(
                                        "obstacle_escape_final_success",
                                        self._navigation_route_items(
                                            pose,
                                            navigation_annotation_selected,
                                            task_xy_tolerance,
                                            task_yaw_tolerance,
                                            reached_kinds=["alignment", "waypoint", "final_goal"],
                                            final_status="reached",
                                        ),
                                        active_kind="final_goal",
                                    )
                                return make_result(
                                    "goto_pose",
                                    True,
                                    codes.OK,
                                    "Navigation recovered by fixed-goal obstacle escape waypoint.",
                                    started,
                                    metrics,
                                )
            if publish_goal_markers:
                self._publish_navigation_route_state(
                    "failed_timeout",
                    self._navigation_route_items(
                        pose,
                        navigation_annotation_selected,
                        task_xy_tolerance,
                        task_yaw_tolerance,
                        active_kind="final_goal",
                        reached_kinds=["alignment", "waypoint"] if navigation_annotation_result else [],
                        final_status="failed",
                    ),
                    active_kind="final_goal",
                )
                self._clear_navigation_goal_markers()
                self._clear_navigation_route_markers()
            return make_result(
                "goto_pose",
                False,
                codes.FAILED_TIMEOUT,
                "Navigation timed out and was cancelled.",
                started,
                metrics,
            )

        result = holder.get("result")
        status = getattr(result, "status", None)
        self._active_goal_handle = None
        success = status == ACTION_STATUS_SUCCEEDED
        if status == ACTION_STATUS_SUCCEEDED:
            code = codes.OK
            message = "Navigation succeeded."
        elif status == ACTION_STATUS_CANCELED:
            code = codes.CANCELLED
            message = "Navigation was canceled."
        elif status == ACTION_STATUS_ABORTED:
            code = codes.FAILED_NAVIGATION
            message = "Navigation was aborted by Nav2."
        else:
            code = codes.FAILED_NAVIGATION
            message = f"Navigation finished with status {status}."
        metrics = {
            "nav2_status": status,
            "pose": pose.to_dict(),
            "duration_sec": time.monotonic() - started_monotonic,
            "timeout_policy": timeout_policy,
            "requested_timeout_sec": requested_timeout,
            "effective_timeout_sec": effective_timeout,
            "timeout_extensions": timeout_extensions,
            "navigation_annotation": navigation_annotation_selected,
            "navigation_annotation_diagnostics": navigation_annotation_diagnostics,
            "navigation_alignment_result": (
                navigation_alignment_result.to_dict() if navigation_alignment_result else None
            ),
            "navigation_annotation_result": (
                navigation_annotation_result.to_dict() if navigation_annotation_result else None
            ),
            "local_clearance": local_clearance,
            "local_passage_result": local_passage_result.to_dict() if local_passage_result else None,
            "pre_align_reason": pre_align_reason,
            "pre_align_skip_reason": pre_align_skip_reason,
            "pre_align_heading_delta_rad": pre_align_heading_delta,
            "pre_align_result": pre_align_result.to_dict() if pre_align_result else None,
            "staged_recovery": staged_recovery,
            "start_pose": start_pose,
            "start_xy_distance_m": start_xy_distance,
            "tolerance_profile": tolerance_profile,
            "xy_goal_tolerance_m": task_xy_tolerance,
            "yaw_goal_tolerance_rad": task_yaw_tolerance,
        }
        if not success and code == codes.FAILED_NAVIGATION:
            latest_local_clearance = self._local_clearance_snapshot()
            if latest_local_clearance is not None:
                local_clearance = latest_local_clearance
                metrics["local_clearance"] = local_clearance
        if (
            not success
            and code == codes.FAILED_NAVIGATION
            and allow_alternate_goal
            and alternate_goal_attempts > 0
            and self._local_clearance_indicates_blocked(local_clearance)
        ):
            current_pose_for_escape = self._current_map_pose() or start_pose
            if current_pose_for_escape is not None:
                escape_pose, escape_sampling = self._sample_trusted_front_open_area_goal(
                    pose,
                    current_pose_for_escape,
                )
            else:
                escape_pose, escape_sampling = None, {"available": False, "reason": "current_pose_unavailable"}
            metrics["post_abort_escape_sampling"] = escape_sampling
            if escape_pose is not None:
                self._publish_navigation_progress(
                    "post_abort_escape_waypoint",
                    escape_pose,
                    {
                        "original_goal_pose": pose.to_dict(),
                        "post_abort_escape_sampling": escape_sampling,
                        "reason": "nav2_aborted_blocked_local_costmap_escape",
                    },
                )
                if publish_goal_markers:
                    self._publish_navigation_route_state(
                        "post_abort_escape_waypoint",
                        self._navigation_route_items(
                            pose,
                            {
                                "waypoint_pose": escape_pose.to_dict(),
                                "waypoint_xy_goal_tolerance_m": max(task_xy_tolerance or 0.25, 0.35),
                                "reason": "nav2_aborted_blocked_local_costmap_escape",
                            },
                            task_xy_tolerance,
                            task_yaw_tolerance,
                            active_kind="waypoint",
                            reached_kinds=["alignment", "waypoint"] if navigation_annotation_result else [],
                        ),
                        active_kind="waypoint",
                    )
                escape_result = self.goto_pose(
                    context,
                    escape_pose,
                    timeout_sec=min(20.0, effective_timeout),
                    xy_goal_tolerance_m=max(task_xy_tolerance or 0.25, 0.35),
                    yaw_goal_tolerance_rad=task_yaw_tolerance,
                    tolerance_profile=f"{tolerance_profile}_post_abort_escape",
                    dynamic_timeout=True,
                    max_timeout_sec=min(24.0, max_timeout_sec),
                    recovery_policy=recovery_policy,
                    apply_navigation_annotations=False,
                    publish_goal_markers=False,
                    alternate_goal_attempts=0,
                )
                metrics["post_abort_escape_result"] = escape_result.to_dict()
                if escape_result.success:
                    retry_result = self.goto_pose(
                        context,
                        pose,
                        timeout_sec=min(requested_timeout, max(18.0, effective_timeout * 0.75)),
                        xy_goal_tolerance_m=task_xy_tolerance,
                        yaw_goal_tolerance_rad=task_yaw_tolerance,
                        tolerance_profile=tolerance_profile,
                        dynamic_timeout=dynamic_timeout,
                        max_timeout_sec=max_timeout_sec,
                        recovery_policy=recovery_policy,
                        apply_navigation_annotations=apply_navigation_annotations,
                        publish_goal_markers=False,
                        alternate_goal_attempts=alternate_goal_attempts - 1,
                    )
                    metrics["post_abort_escape_final_retry_result"] = retry_result.to_dict()
                    if retry_result.success:
                        update_recovered_pose_metrics(
                            metrics, pose, retry_result.metrics.get("final_pose"),
                            "post_abort_escape_final_retry_result")
                        if publish_goal_markers:
                            self._publish_navigation_route_state(
                                "post_abort_escape_final_success",
                                self._navigation_route_items(
                                    pose,
                                    navigation_annotation_selected,
                                    task_xy_tolerance,
                                    task_yaw_tolerance,
                                    reached_kinds=["alignment", "waypoint", "final_goal"],
                                    final_status="reached",
                                ),
                                active_kind="final_goal",
                            )
                        return make_result(
                            "goto_pose",
                            True,
                            codes.OK,
                            "Navigation recovered by post-abort local-costmap escape waypoint.",
                            started,
                            metrics,
                        )
        if success:
            final_pose = self._current_map_pose()
            if final_pose is not None:
                xy_error = math.hypot(final_pose["x"] - pose.x, final_pose["y"] - pose.y)
                yaw_error = abs(self._normalize_angle(final_pose["yaw"] - pose.yaw))
                metrics["final_pose"] = final_pose
                metrics["final_xy_error_m"] = xy_error
                metrics["final_yaw_error_rad"] = yaw_error
                if xy_error > 0.25 or yaw_error > 0.35:
                    success = False
                    code = codes.FAILED_NAVIGATION
                    message = (
                        "Nav2 reported success, but final map pose is outside tolerance "
                        f"(xy_error={xy_error:.2f} m, yaw_error={yaw_error:.2f} rad)."
                    )
            else:
                metrics["final_pose_check"] = "tf_unavailable"
        if publish_goal_markers:
            final_status = "reached" if success else ("canceled" if code == codes.CANCELLED else "failed")
            self._publish_navigation_route_state(
                final_status,
                self._navigation_route_items(
                    pose,
                    navigation_annotation_selected,
                    task_xy_tolerance,
                    task_yaw_tolerance,
                    reached_kinds=["alignment", "waypoint"] + (["final_goal"] if success else []),
                    final_status=final_status,
                ),
                active_kind="final_goal",
            )
            if not success:
                self._clear_navigation_goal_markers()
                self._clear_navigation_route_markers()
        return make_result(
            "goto_pose",
            success,
            code,
            message,
            started,
            metrics,
        )

    @staticmethod
    def _estimate_navigation_timeout(
        straight_line_distance_m: float,
        requested_timeout_sec: float,
        max_timeout_sec: float,
    ) -> float:
        # Keep nearby tasks from waiting on a large fixed timeout while still giving
        # long semantic routes enough time to pass through the safety layer and Nav2.
        base_sec = 12.0
        expected_speed_mps = 0.16
        distance_timeout = base_sec + max(0.0, straight_line_distance_m) / expected_speed_mps
        capped = min(max_timeout_sec, distance_timeout)
        if requested_timeout_sec > 0:
            capped = min(requested_timeout_sec, capped)
        return max(18.0, capped)

    def _publish_navigation_progress(
        self,
        phase: str,
        pose: Pose2D | None = None,
        metrics: dict[str, Any] | None = None,
    ) -> None:
        payload: dict[str, Any] = {
            "phase": phase,
            "time_monotonic": time.monotonic(),
        }
        if pose is not None:
            payload["pose"] = pose.to_dict()
        if metrics:
            payload.update(metrics)
        try:
            self._navigation_progress_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception as exc:
            self.node.get_logger().warn(f"Failed to publish navigation progress: {exc!r}")

    def _navigation_route_items(
        self,
        final_goal: Pose2D,
        annotation: dict[str, Any] | None,
        final_tolerance_m: float | None,
        final_yaw_tolerance_rad: float | None,
        active_kind: str | None = None,
        reached_kinds: list[str] | None = None,
        failed_kinds: list[str] | None = None,
        final_status: str | None = None,
    ) -> list[dict[str, Any]]:
        reached = set(reached_kinds or [])
        failed = set(failed_kinds or [])
        items: list[dict[str, Any]] = []

        def status_for(kind: str) -> str:
            if kind in failed:
                return "failed"
            if kind == "final_goal" and final_status:
                return final_status
            if kind in reached:
                return "reached"
            if active_kind == kind:
                return "active"
            return "pending"

        if annotation:
            passage_id = annotation.get("passage_id")
            alignment_pose = annotation.get("alignment_goal_pose")
            if alignment_pose:
                items.append(
                    {
                        "id": f"{passage_id or 'annotation'}:alignment",
                        "name": "alignment",
                        "kind": "alignment",
                        "pose": alignment_pose,
                        "xy_tolerance_m": annotation.get("alignment_xy_goal_tolerance_m"),
                        "yaw_tolerance_rad": annotation.get("alignment_yaw_goal_tolerance_rad"),
                        "status": status_for("alignment"),
                        "insert_by": "navigation_annotation",
                        "passage_id": passage_id,
                        "source": annotation.get("reason"),
                    }
                )
            waypoint_pose = annotation.get("waypoint_pose")
            if waypoint_pose:
                items.append(
                    {
                        "id": f"{passage_id or annotation.get('reason') or 'annotation'}:waypoint",
                        "name": "waypoint",
                        "kind": "waypoint",
                        "pose": waypoint_pose,
                        "xy_tolerance_m": annotation.get("waypoint_xy_goal_tolerance_m"),
                        "yaw_tolerance_rad": annotation.get("waypoint_yaw_goal_tolerance_rad"),
                        "status": status_for("waypoint"),
                        "insert_by": "navigation_annotation",
                        "passage_id": passage_id,
                        "source": annotation.get("reason"),
                    }
                )
        items.append(
            {
                "id": "final_goal",
                "name": "final_goal",
                "kind": "final_goal",
                "pose": final_goal.to_dict(),
                "xy_tolerance_m": final_tolerance_m,
                "yaw_tolerance_rad": final_yaw_tolerance_rad,
                "status": status_for("final_goal"),
                "insert_by": "mission_goal",
                "source": "resolved_semantic_goal",
            }
        )
        return items

    def _publish_navigation_route_state(
        self,
        phase: str,
        items: list[dict[str, Any]],
        active_index: int | None = None,
        active_kind: str | None = None,
    ) -> None:
        active_item_index = active_index
        if active_item_index is None and active_kind:
            for index, item in enumerate(items):
                if item.get("kind") == active_kind:
                    active_item_index = index
                    break
        if active_item_index is None:
            for index, item in enumerate(items):
                if item.get("status") == "active":
                    active_item_index = index
                    break
        payload = {
            "phase": phase,
            "time_monotonic": time.monotonic(),
            "active_index": active_item_index,
            "items": items,
        }
        current_pose = self._current_map_pose()
        if current_pose is not None:
            payload["current_pose"] = current_pose
        if self._current_route_path_points:
            payload["path_points"] = self._current_route_path_points
        try:
            self._navigation_route_state_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        except Exception as exc:
            self.node.get_logger().warn(f"Failed to publish navigation route state: {exc!r}")
        self._clear_navigation_goal_markers()
        if phase in {"reached", "failed", "canceled", "task_tolerance_reached", "relaxed_tolerance_success"}:
            self._clear_navigation_route_markers()
        else:
            self._publish_navigation_route_markers(payload)

    def _publish_navigation_route_markers(self, route_state: dict[str, Any]) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 0
        marker_id = self._append_annotation_context_markers(markers, marker_id, route_state)
        marker_id = self._append_route_sequence_marker(markers, marker_id, route_state)
        marker_id = self._append_route_path_marker(markers, marker_id, route_state.get("path_points") or [])
        for item in route_state.get("items", []):
            status = str(item.get("status") or "")
            kind = str(item.get("kind") or "")
            if status == "reached" and kind != "final_goal":
                continue
            if status == "canceled":
                continue
            pose = item.get("pose")
            if not isinstance(pose, dict):
                continue
            marker_id = self._append_pose_marker(
                markers,
                marker_id,
                self._route_marker_label(item),
                pose,
                self._route_marker_color(kind, status),
                item.get("xy_tolerance_m"),
                namespace_prefix="navigation_route",
            )
        self._append_route_status_marker(markers, marker_id, route_state)
        self._route_marker_pub.publish(markers)

    def _append_annotation_context_markers(
        self,
        markers: MarkerArray,
        marker_id: int,
        route_state: dict[str, Any],
    ) -> int:
        selected_passages = {
            str(item.get("passage_id"))
            for item in route_state.get("items", [])
            if item.get("passage_id")
        }
        stamp = self.node.get_clock().now().to_msg()
        for passage in self._navigation_annotations.get("narrow_passages", []):
            passage_id = str(passage.get("id") or "narrow_passage")
            selected = passage_id in selected_passages
            enabled = bool(passage.get("enabled", True))
            rgba = (
                (1.0, 0.45, 0.02, 0.95)
                if selected
                else ((0.05, 0.75, 0.75, 0.58) if enabled else (0.45, 0.45, 0.45, 0.32))
            )

            centerline = passage.get("centerline") or []
            if len(centerline) >= 2:
                line = Marker()
                line.header.frame_id = "map"
                line.header.stamp = stamp
                line.ns = "navigation_annotation_centerline"
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.075 if selected else 0.035
                line.color.r, line.color.g, line.color.b, line.color.a = rgba
                for item in centerline:
                    point = Point()
                    point.x = float(item.get("x", 0.0))
                    point.y = float(item.get("y", 0.0))
                    point.z = 0.065
                    line.points.append(point)
                markers.markers.append(line)

                middle = centerline[len(centerline) // 2]
                label = Marker()
                label.header.frame_id = "map"
                label.header.stamp = stamp
                label.ns = "navigation_annotation_label"
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = float(middle.get("x", 0.0))
                label.pose.position.y = float(middle.get("y", 0.0))
                label.pose.position.z = 0.32
                label.pose.orientation.w = 1.0
                label.scale.z = 0.15
                label.color.r, label.color.g, label.color.b, label.color.a = rgba
                state = "SELECTED" if selected else ("enabled" if enabled else "disabled")
                label.text = f"{passage_id} [{state}]"
                markers.markers.append(label)

            for zone in passage.get("alignment_zones", []):
                polygon = zone.get("polygon") or []
                if len(polygon) < 3:
                    continue
                boundary = Marker()
                boundary.header.frame_id = "map"
                boundary.header.stamp = stamp
                boundary.ns = "navigation_annotation_alignment_zone"
                boundary.id = marker_id
                marker_id += 1
                boundary.type = Marker.LINE_STRIP
                boundary.action = Marker.ADD
                boundary.pose.orientation.w = 1.0
                boundary.scale.x = 0.025
                boundary.color.r, boundary.color.g, boundary.color.b, boundary.color.a = rgba
                for item in list(polygon) + [polygon[0]]:
                    point = Point()
                    point.x = float(item.get("x", 0.0))
                    point.y = float(item.get("y", 0.0))
                    point.z = 0.05
                    boundary.points.append(point)
                markers.markers.append(boundary)
        return marker_id

    def _append_route_status_marker(
        self,
        markers: MarkerArray,
        marker_id: int,
        route_state: dict[str, Any],
    ) -> int:
        items = route_state.get("items", [])
        anchor = next(
            (item.get("pose") for item in reversed(items) if isinstance(item.get("pose"), dict)),
            route_state.get("current_pose"),
        )
        if not isinstance(anchor, dict):
            return marker_id
        passage_ids = [str(item.get("passage_id")) for item in items if item.get("passage_id")]
        marker = Marker()
        marker.header.frame_id = str(anchor.get("frame_id") or "map")
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "navigation_route_status"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(anchor.get("x", 0.0))
        marker.pose.position.y = float(anchor.get("y", 0.0))
        marker.pose.position.z = 0.72
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.19
        if passage_ids:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.45, 0.02, 0.98)
            marker.text = f"NARROW ROUTE: {passage_ids[0]}"
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.2, 0.7, 1.0, 0.92)
            marker.text = "DIRECT ROUTE: no narrow passage selected"
        markers.markers.append(marker)
        return marker_id + 1

    def _append_route_sequence_marker(
        self,
        markers: MarkerArray,
        marker_id: int,
        route_state: dict[str, Any],
    ) -> int:
        points: list[dict[str, Any]] = []
        current_pose = route_state.get("current_pose")
        if isinstance(current_pose, dict):
            points.append(current_pose)
        for item in route_state.get("items", []):
            if item.get("status") in {"reached", "canceled"} and item.get("kind") != "final_goal":
                continue
            pose = item.get("pose")
            if isinstance(pose, dict):
                points.append(pose)
        if len(points) < 2:
            return marker_id
        stamp = self.node.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = str(points[0].get("frame_id") or "map")
        line.header.stamp = stamp
        line.ns = "navigation_route_sequence"
        line.id = marker_id
        marker_id += 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.015
        line.color.r = 0.10
        line.color.g = 0.45
        line.color.b = 1.0
        line.color.a = 0.72
        for pose in points:
            point = Point()
            point.x = float(pose.get("x", 0.0))
            point.y = float(pose.get("y", 0.0))
            point.z = 0.055
            line.points.append(point)
        markers.markers.append(line)
        return marker_id

    def _append_route_path_marker(
        self,
        markers: MarkerArray,
        marker_id: int,
        path_points: list[dict[str, float]],
    ) -> int:
        if len(path_points) < 2:
            return marker_id
        stamp = self.node.get_clock().now().to_msg()
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = stamp
        line.ns = "navigation_route_global_path"
        line.id = marker_id
        marker_id += 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.018
        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = 0.5
        stride = max(1, len(path_points) // 220)
        for path_point in path_points[::stride]:
            point = Point()
            point.x = float(path_point.get("x", 0.0))
            point.y = float(path_point.get("y", 0.0))
            point.z = 0.035
            line.points.append(point)
        markers.markers.append(line)
        return marker_id

    @staticmethod
    def _route_marker_label(item: dict[str, Any]) -> str:
        status = str(item.get("status") or "")
        label = str(item.get("name") or item.get("kind") or "route")
        tolerance = item.get("xy_tolerance_m")
        if tolerance is not None:
            try:
                label = f"{label} {float(tolerance):.2f}m"
            except (TypeError, ValueError):
                pass
        if status and status not in {"pending", "active"}:
            label = f"{label} {status}"
        return label

    @staticmethod
    def _route_marker_color(kind: str, status: str) -> tuple[float, float, float, float]:
        if status == "failed":
            return (0.9, 0.1, 0.08, 0.95)
        if status == "accepted_relaxed":
            return (0.55, 0.25, 1.0, 0.95)
        if status == "reached":
            return (0.25, 0.65, 0.25, 0.75)
        if kind == "alignment":
            return (0.1, 0.75, 0.35, 0.95)
        if kind == "waypoint":
            return (1.0, 0.58, 0.08, 0.95)
        return (0.15, 0.35, 1.0, 0.95)

    @staticmethod
    def _local_clearance_indicates_blocked(local_clearance: dict[str, Any] | None) -> bool:
        if not isinstance(local_clearance, dict):
            return False
        nearest = local_clearance.get("nearest_obstacle_m")
        try:
            if nearest is not None and float(nearest) <= 0.30:
                return True
        except (TypeError, ValueError):
            pass
        if local_clearance.get("sides_congested") is True and local_clearance.get("front_clear") is False:
            return True
        regions = local_clearance.get("regions") or {}
        front = regions.get("front") or {}
        left = regions.get("left") or {}
        right = regions.get("right") or {}
        try:
            front_blocked = float(front.get("blocked_ratio", 0.0))
            left_blocked = float(left.get("blocked_ratio", 0.0))
            right_blocked = float(right.get("blocked_ratio", 0.0))
        except (TypeError, ValueError):
            return False
        return front_blocked >= 0.35 or (left_blocked >= 0.45 and right_blocked >= 0.45)

    def _clear_navigation_goal_markers(self) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._goal_marker_pub.publish(markers)

    def _clear_navigation_route_markers(self) -> None:
        self._current_route_path_points = None
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._route_marker_pub.publish(markers)

    @staticmethod
    def _pose_key(pose_dict: dict[str, Any]) -> tuple[float, float, float]:
        return (
            round(float(pose_dict.get("x", 0.0)), 3),
            round(float(pose_dict.get("y", 0.0)), 3),
            round(float(pose_dict.get("yaw", 0.0)), 3),
        )

    def _sample_reachable_nav2_goal(
        self,
        start_pose: dict[str, float],
        desired_goal: Pose2D,
        final_goal: Pose2D,
        max_radius_m: float = 0.70,
    ) -> tuple[Pose2D | None, dict[str, Any]]:
        radii = [0.0, min(0.25, max_radius_m), min(0.45, max_radius_m), max_radius_m]
        angles = [
            0.0,
            math.pi / 4.0,
            math.pi / 2.0,
            3.0 * math.pi / 4.0,
            math.pi,
            -3.0 * math.pi / 4.0,
            -math.pi / 2.0,
            -math.pi / 4.0,
        ]
        candidates: list[tuple[float, Pose2D, dict[str, Any]]] = []
        tested = 0
        for radius in radii:
            offsets = [(0.0, 0.0)] if radius <= 1e-6 else [
                (radius * math.cos(angle), radius * math.sin(angle)) for angle in angles
            ]
            for dx, dy in offsets:
                tested += 1
                candidate = Pose2D(
                    x=desired_goal.x + dx,
                    y=desired_goal.y + dy,
                    yaw=desired_goal.yaw,
                    frame_id=desired_goal.frame_id,
                )
                path = self._compute_global_path(start_pose, candidate, timeout_sec=0.8)
                if not path:
                    continue
                path_length = self._path_length(path)
                offset_distance = math.hypot(dx, dy)
                final_distance = math.hypot(candidate.x - final_goal.x, candidate.y - final_goal.y)
                score = offset_distance * 0.7 + final_distance * 0.04 + path_length * 0.02
                candidates.append(
                    (
                        score,
                        candidate,
                        {
                            "offset_from_requested_m": {"x": dx, "y": dy},
                            "path_points": len(path),
                            "path_length_m": path_length,
                            "distance_to_final_goal_m": final_distance,
                            "score": score,
                        },
                    )
                )
        if not candidates:
            return None, {
                "available": False,
                "reason": "no_reachable_nav2_waypoint_candidate",
                "tested_candidates": tested,
                "requested_pose": desired_goal.to_dict(),
                "max_radius_m": max_radius_m,
            }
        score, pose, details = min(candidates, key=lambda item: item[0])
        return pose, {
            "available": True,
            "tested_candidates": tested,
            "selected_pose": pose.to_dict(),
            "selected_score": score,
            "selected": details,
            "candidate_count": len(candidates),
            "max_radius_m": max_radius_m,
        }

    @staticmethod
    def _path_length(points: list[dict[str, float]]) -> float:
        if len(points) < 2:
            return 0.0
        total = 0.0
        last = points[0]
        for point in points[1:]:
            total += math.hypot(float(point["x"]) - float(last["x"]), float(point["y"]) - float(last["y"]))
            last = point
        return total

    def _publish_navigation_goal_markers(
        self,
        final_goal: Pose2D,
        annotation: dict[str, Any] | None,
        final_tolerance_m: float | None = None,
    ) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        marker_id = self._append_pose_marker(
            markers,
            marker_id,
            "final_goal",
            final_goal.to_dict(),
            (0.15, 0.35, 1.0, 0.95),
            final_tolerance_m,
        )
        if annotation:
            alignment_pose = annotation.get("alignment_goal_pose")
            if alignment_pose:
                marker_id = self._append_pose_marker(
                    markers,
                    marker_id,
                    "alignment",
                    alignment_pose,
                    (0.1, 0.75, 0.35, 0.95),
                    annotation.get("alignment_xy_goal_tolerance_m"),
                )
            waypoint_pose = annotation.get("waypoint_pose")
            if waypoint_pose:
                marker_id = self._append_pose_marker(
                    markers,
                    marker_id,
                    "waypoint",
                    waypoint_pose,
                    (1.0, 0.58, 0.08, 0.95),
                    annotation.get("waypoint_xy_goal_tolerance_m"),
                )
        self._goal_marker_pub.publish(markers)

    def _append_pose_marker(
        self,
        markers: MarkerArray,
        marker_id: int,
        label: str,
        pose_dict: dict[str, Any],
        rgba: tuple[float, float, float, float],
        tolerance_m: float | None = None,
        namespace_prefix: str = "navigation_goal",
    ) -> int:
        pose = Pose2D.from_dict(pose_dict)
        stamp = self.node.get_clock().now().to_msg()

        arrow = Marker()
        arrow.header.frame_id = pose.frame_id
        arrow.header.stamp = stamp
        arrow.ns = namespace_prefix
        arrow.id = marker_id
        marker_id += 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = pose.x
        arrow.pose.position.y = pose.y
        arrow.pose.position.z = 0.10
        arrow.pose.orientation.z = math.sin(pose.yaw / 2.0)
        arrow.pose.orientation.w = math.cos(pose.yaw / 2.0)
        arrow.scale.x = 0.2
        arrow.scale.y = 0.025
        arrow.scale.z = 0.025
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = rgba
        markers.markers.append(arrow)

        tolerance_value = None
        try:
            if tolerance_m is not None:
                tolerance_value = max(0.02, float(tolerance_m))
        except (TypeError, ValueError):
            tolerance_value = None
        if tolerance_value is not None:
            circle = Marker()
            circle.header.frame_id = pose.frame_id
            circle.header.stamp = stamp
            circle.ns = f"{namespace_prefix}_tolerance"
            circle.id = marker_id
            marker_id += 1
            circle.type = Marker.LINE_STRIP
            circle.action = Marker.ADD
            circle.pose.orientation.w = 1.0
            circle.scale.x = 0.015
            circle.color.r, circle.color.g, circle.color.b, circle.color.a = rgba
            circle.color.a = min(circle.color.a, 0.65)
            for index in range(49):
                theta = (2.0 * math.pi * index) / 48.0
                point = Point()
                point.x = pose.x + tolerance_value * math.cos(theta)
                point.y = pose.y + tolerance_value * math.sin(theta)
                point.z = 0.04
                circle.points.append(point)
            markers.markers.append(circle)

        text = Marker()
        text.header.frame_id = pose.frame_id
        text.header.stamp = stamp
        text.ns = f"{namespace_prefix}_label"
        text.id = marker_id
        marker_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = pose.x
        text.pose.position.y = pose.y
        text.pose.position.z = 0.45
        text.scale.z = 0.22
        text.color.r, text.color.g, text.color.b, text.color.a = rgba
        text.text = f"{label} tol={tolerance_value:.2f}m" if tolerance_value is not None and "m" not in label else label
        markers.markers.append(text)
        return marker_id

    def _load_navigation_annotations(self) -> dict[str, Any]:
        map_id = os.environ.get("ROBOT_MAP_ID", "").strip()
        if not map_id:
            return {}
        roots = [
            os.getcwd(),
            os.environ.get("ROBOT_WS", ""),
            os.environ.get("ROBOT_WORKSPACE", ""),
            self._repo_root(),
        ]
        seen: set[str] = set()
        for root in roots:
            if not root:
                continue
            bundle_dir = os.path.join(root, "maps", map_id)
            if bundle_dir in seen:
                continue
            seen.add(bundle_dir)
            metadata_path = os.path.join(bundle_dir, "metadata.yaml")
            annotation_name = "navigation_annotations.yaml"
            if os.path.exists(metadata_path):
                try:
                    with open(metadata_path, "r", encoding="utf-8") as f:
                        metadata = yaml.safe_load(f) or {}
                    annotation_name = (
                        metadata.get("bundle_files", {}).get("navigation_annotations")
                        or metadata.get("navigation_annotations")
                        or annotation_name
                    )
                except Exception as exc:
                    self.node.get_logger().warn(f"Failed to load map metadata for navigation annotations: {exc!r}")
            annotation_path = os.path.join(bundle_dir, str(annotation_name))
            if os.path.exists(annotation_path):
                try:
                    with open(annotation_path, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f) or {}
                    data["_path"] = annotation_path
                    return data
                except Exception as exc:
                    self.node.get_logger().warn(f"Failed to load navigation annotations: {exc!r}")
                    return {}
        return {}

    def _load_nav2_map(self) -> dict[str, Any] | None:
        map_id = os.environ.get("ROBOT_MAP_ID", "").strip()
        if not map_id:
            return None
        roots = [
            os.getcwd(),
            os.environ.get("ROBOT_WS", ""),
            os.environ.get("ROBOT_WORKSPACE", ""),
            self._repo_root(),
        ]
        seen: set[str] = set()
        for root in roots:
            if not root:
                continue
            bundle_dir = os.path.join(root, "maps", map_id)
            if bundle_dir in seen:
                continue
            seen.add(bundle_dir)
            metadata_path = os.path.join(bundle_dir, "metadata.yaml")
            map_name = "nav2.yaml"
            if os.path.exists(metadata_path):
                try:
                    with open(metadata_path, "r", encoding="utf-8") as f:
                        metadata = yaml.safe_load(f) or {}
                    map_name = (
                        metadata.get("bundle_files", {}).get("nav2_map")
                        or metadata.get("nav2_map")
                        or map_name
                    )
                except Exception as exc:
                    self.node.get_logger().warn(f"Failed to load map metadata for Nav2 map diagnostics: {exc!r}")
            map_yaml_path = os.path.join(bundle_dir, str(map_name))
            if not os.path.exists(map_yaml_path):
                continue
            try:
                with open(map_yaml_path, "r", encoding="utf-8") as f:
                    map_yaml = yaml.safe_load(f) or {}
                image_path = str(map_yaml.get("image", ""))
                if not os.path.isabs(image_path):
                    image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)
                pgm = self._load_pgm_image(image_path)
                if pgm is None:
                    return None
                width, height, pixels = pgm
                return {
                    "path": map_yaml_path,
                    "image": image_path,
                    "width": width,
                    "height": height,
                    "pixels": pixels,
                    "resolution": float(map_yaml.get("resolution", 0.05)),
                    "origin": list(map_yaml.get("origin", [0.0, 0.0, 0.0])),
                    "occupied_thresh": float(map_yaml.get("occupied_thresh", 0.65)),
                    "free_thresh": float(map_yaml.get("free_thresh", 0.25)),
                }
            except Exception as exc:
                self.node.get_logger().warn(f"Failed to load Nav2 map diagnostics: {exc!r}")
                return None
        return None

    def _load_pgm_image(self, path: str) -> tuple[int, int, list[int]] | None:
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as exc:
            self.node.get_logger().warn(f"Failed to read Nav2 PGM map for diagnostics: {exc!r}")
            return None

        index = 0

        def next_token() -> bytes:
            nonlocal index
            while index < len(data):
                ch = data[index]
                if ch == ord("#"):
                    while index < len(data) and data[index] not in (10, 13):
                        index += 1
                elif chr(ch).isspace():
                    index += 1
                else:
                    break
            start = index
            while index < len(data) and not chr(data[index]).isspace():
                index += 1
            return data[start:index]

        magic = next_token()
        if magic not in (b"P5", b"P2"):
            self.node.get_logger().warn(f"Unsupported Nav2 map image format for diagnostics: {magic!r}")
            return None
        width = int(next_token())
        height = int(next_token())
        max_value = int(next_token())
        if width <= 0 or height <= 0 or max_value <= 0:
            return None
        if magic == b"P5":
            if index < len(data) and chr(data[index]).isspace():
                index += 1
            raw = data[index : index + width * height]
            if len(raw) < width * height:
                return None
            if max_value == 255:
                pixels = list(raw)
            else:
                pixels = [int(v * 255 / max_value) for v in raw]
        else:
            pixels = [int(int(next_token()) * 255 / max_value) for _ in range(width * height)]
        return width, height, pixels

    @staticmethod
    def _repo_root() -> str:
        # .../src/robot_skills/robot_skills/executors/navigation.py -> workspace root.
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

    def _global_plan_failure_message(
        self,
        target_pose: Pose2D,
        global_plan_error: dict[str, Any] | None,
    ) -> str:
        target = f"({target_pose.x:.2f}, {target_pose.y:.2f})"
        diagnosis = (global_plan_error or {}).get("diagnosis")
        if diagnosis == "start_footprint_occupied":
            return (
                "Failed to generate a valid path because the robot footprint overlaps "
                f"occupied map cells at the current pose; target was {target}."
            )
        if diagnosis == "target_footprint_occupied":
            return (
                f"Failed to generate a valid path to {target} because the target footprint "
                "overlaps occupied map cells."
            )
        if diagnosis == "start_or_target_outside_map":
            return f"Failed to generate a valid path to {target}; start or target is outside the Nav2 map."
        return f"Failed to generate a valid path to {target}; navigation was not started."

    def _global_plan_map_diagnostics(
        self,
        start_pose: dict[str, float],
        target_pose: Pose2D,
    ) -> dict[str, Any] | None:
        if not self._nav2_map:
            return None
        start_diag = self._pose_footprint_map_diagnostics(
            float(start_pose["x"]),
            float(start_pose["y"]),
            float(start_pose.get("yaw", 0.0)),
        )
        target_diag = self._pose_footprint_map_diagnostics(target_pose.x, target_pose.y, target_pose.yaw)
        diagnosis = "path_unreachable_or_global_costmap_blocked"
        if start_diag.get("outside_map") or target_diag.get("outside_map"):
            diagnosis = "start_or_target_outside_map"
        elif start_diag.get("lethal_cells", 0) > 0:
            diagnosis = "start_footprint_occupied"
        elif target_diag.get("lethal_cells", 0) > 0:
            diagnosis = "target_footprint_occupied"
        return {
            "diagnosis": diagnosis,
            "map": {
                "path": self._nav2_map.get("path"),
                "image": self._nav2_map.get("image"),
                "resolution": self._nav2_map.get("resolution"),
            },
            "start_footprint": start_diag,
            "target_footprint": target_diag,
        }

    def _pose_footprint_map_diagnostics(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        nav_map = self._nav2_map
        if not nav_map:
            return {"available": False, "reason": "nav2_map_not_loaded"}
        width = int(nav_map["width"])
        height = int(nav_map["height"])
        pixels = nav_map["pixels"]
        resolution = float(nav_map["resolution"])
        origin = nav_map["origin"]
        ox = float(origin[0])
        oy = float(origin[1])
        footprint = [(0.34, 0.32), (0.34, -0.32), (-0.52, -0.32), (-0.52, 0.32)]
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        poly = [
            (
                x + cos_yaw * fx - sin_yaw * fy,
                y + sin_yaw * fx + cos_yaw * fy,
            )
            for fx, fy in footprint
        ]

        def world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
            px = int((wx - ox) / resolution)
            py = height - 1 - int((wy - oy) / resolution)
            return px, py

        def point_in_poly(px_world: float, py_world: float) -> bool:
            inside = False
            count = len(poly)
            for index in range(count):
                x1, y1 = poly[index]
                x2, y2 = poly[(index + 1) % count]
                if (y1 > py_world) != (y2 > py_world):
                    x_intersect = (x2 - x1) * (py_world - y1) / (y2 - y1 + 1e-12) + x1
                    if px_world < x_intersect:
                        inside = not inside
            return inside

        center_px, center_py = world_to_pixel(x, y)
        outside_map = not (0 <= center_px < width and 0 <= center_py < height)
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        min_px, min_py = world_to_pixel(min(xs), max(ys))
        max_px, max_py = world_to_pixel(max(xs), min(ys))
        free = lethal = other = outside = 0
        for py in range(min_py, max_py + 1):
            for px in range(min_px, max_px + 1):
                wx = ox + (px + 0.5) * resolution
                wy = oy + (height - 1 - py + 0.5) * resolution
                if not point_in_poly(wx, wy):
                    continue
                if not (0 <= px < width and 0 <= py < height):
                    outside += 1
                    continue
                value = int(pixels[py * width + px])
                if value < 10:
                    lethal += 1
                elif value > 250:
                    free += 1
                else:
                    other += 1
        center_value = None
        if 0 <= center_px < width and 0 <= center_py < height:
            center_value = int(pixels[center_py * width + center_px])
        outside_map = outside_map or outside > 0
        return {
            "available": True,
            "x": x,
            "y": y,
            "yaw": yaw,
            "center_pixel": {"x": center_px, "y": center_py, "value": center_value},
            "free_cells": free,
            "lethal_cells": lethal,
            "unknown_or_mid_cells": other,
            "outside_cells": outside,
            "outside_map": outside_map,
            "footprint_polygon": [{"x": px, "y": py} for px, py in poly],
        }

    def _review_before_motion(self, context, pose, start_pose, path, annotation, role, started):
        cancelled = getattr(context, "cancel_check", lambda: False)
        if cancelled():
            return make_result("goto_pose", False, codes.CANCELLED, "Cancelled before route review.", started)
        try:
            decision = context.navigation_review({"path": path, "target_pose": pose.to_dict(),
                                                  "annotation": annotation, "execution_role": role})
        except Exception as exc:
            decision = {"decision": "ask_human", "reason": f"Route review failed: {exc}"}
        # Model latency must not let a cancellation or changed localization dispatch a goal.
        if cancelled():
            return make_result("goto_pose", False, codes.CANCELLED, "Cancelled during route review.", started)
        if not isinstance(decision, dict) or decision.get("decision") != "execute":
            if not isinstance(decision, dict):
                decision = {"decision": "ask_human", "reason": "Invalid route review response"}
            return make_result("goto_pose", False, codes.NEEDS_HUMAN,
                               "Navigation paused before dispatch: " + str(decision.get("reason") or "route review requires clarification"),
                               started, {"navigation_review": decision})
        if not self._navigation_localization_ok():
            return make_result("goto_pose", False, codes.FAILED_LOCALIZATION,
                               "Localization changed during route review.", started)
        current = self._current_map_pose()
        if (current is None or math.hypot(current["x"] - start_pose["x"], current["y"] - start_pose["y"]) > 0.15
                or abs(self._normalize_angle(current.get("yaw", 0.0) - start_pose.get("yaw", 0.0))) > 0.15):
            return make_result("goto_pose", False, codes.FAILED_PRECHECK,
                               "Robot pose changed during route review; replan before moving.", started)
        return None

    def _compute_global_path(
        self,
        start_pose: dict[str, float],
        target_pose: Pose2D,
        timeout_sec: float = 2.0,
    ) -> list[dict[str, float]] | None:
        self._last_global_plan_error = None
        target = {"x": float(target_pose.x), "y": float(target_pose.y), "frame_id": target_pose.frame_id}
        start = {
            "x": float(start_pose["x"]),
            "y": float(start_pose["y"]),
            "yaw": float(start_pose.get("yaw", 0.0)),
            "frame_id": target_pose.frame_id,
        }

        def fail(reason: str, **extra: Any) -> None:
            self._last_global_plan_error = {
                "reason": reason,
                "start_pose": start,
                "target_pose": target,
                **extra,
            }

        if not self._compute_path_client.wait_for_server(timeout_sec=0.5):
            fail("compute_path_to_pose_server_unavailable")
            return None

        goal = ComputePathToPose.Goal()
        goal.goal = pose_to_stamped(self.node, target_pose)
        goal.start = pose_to_stamped(
            self.node,
            Pose2D(
                x=start["x"],
                y=start["y"],
                yaw=start["yaw"],
                frame_id=target_pose.frame_id,
            ),
        )
        goal.use_start = True
        goal.planner_id = ""

        send_done = threading.Event()
        result_done = threading.Event()
        holder: dict[str, Any] = {"accepted": False, "result": None, "error": None}

        def on_result(future):
            try:
                holder["result"] = future.result()
            except Exception as exc:
                holder["error"] = repr(exc)
            result_done.set()

        def on_goal(future):
            try:
                goal_handle = future.result()
                holder["accepted"] = bool(goal_handle.accepted)
                if goal_handle.accepted:
                    goal_handle.get_result_async().add_done_callback(on_result)
                else:
                    result_done.set()
            except Exception as exc:
                holder["error"] = repr(exc)
                result_done.set()
            send_done.set()

        try:
            self._compute_path_client.send_goal_async(goal).add_done_callback(on_goal)
        except Exception as exc:
            fail("compute_path_to_pose_request_exception", error=repr(exc))
            self.node.get_logger().warn(f"Failed to request Nav2 global plan: {exc!r}")
            return None

        if not send_done.wait(timeout=timeout_sec):
            fail("compute_path_to_pose_send_timeout", timeout_sec=timeout_sec)
            return None
        if holder.get("error") or not holder.get("accepted"):
            fail(
                "compute_path_to_pose_goal_rejected",
                accepted=bool(holder.get("accepted")),
                error=holder.get("error"),
            )
            return None
        if not result_done.wait(timeout=timeout_sec):
            fail("compute_path_to_pose_result_timeout", timeout_sec=timeout_sec)
            return None
        result_wrapper = holder.get("result")
        if holder.get("error") or result_wrapper is None:
            fail("compute_path_to_pose_result_error", error=holder.get("error"))
            return None
        status = getattr(result_wrapper, "status", None)
        if status != ACTION_STATUS_SUCCEEDED:
            map_diagnostics = self._global_plan_map_diagnostics(start_pose, target_pose)
            fail(
                "compute_path_to_pose_failed",
                action_status=status,
                message=f"Failed to generate a valid path to ({target_pose.x:.2f}, {target_pose.y:.2f}).",
                diagnosis=(map_diagnostics or {}).get("diagnosis"),
                map_diagnostics=map_diagnostics,
            )
            return None
        path = getattr(getattr(result_wrapper, "result", None), "path", None)
        poses = getattr(path, "poses", None)
        if not poses:
            result = getattr(result_wrapper, "result", None)
            map_diagnostics = self._global_plan_map_diagnostics(start_pose, target_pose)
            fail(
                "compute_path_to_pose_empty_path",
                action_status=status,
                error_code=getattr(result, "error_code", None),
                error_msg=getattr(result, "error_msg", None),
                message=f"Failed to generate a valid path to ({target_pose.x:.2f}, {target_pose.y:.2f}).",
                diagnosis=(map_diagnostics or {}).get("diagnosis"),
                map_diagnostics=map_diagnostics,
            )
            return None
        return [
            {"x": float(stamped.pose.position.x), "y": float(stamped.pose.position.y)}
            for stamped in poses
        ]

    def _select_navigation_annotation(
        self,
        start_pose: dict[str, float],
        target_pose: Pose2D,
        global_plan_points: list[dict[str, float]] | None = None,
    ) -> dict[str, Any] | None:
        passages = self._navigation_annotations.get("narrow_passages", [])
        if not passages:
            return None
        start = (float(start_pose["x"]), float(start_pose["y"]))
        target = (float(target_pose.x), float(target_pose.y))
        remaining_distance = math.dist(start, target)
        best: dict[str, Any] | None = None
        best_score = None

        for passage in passages:
            if not passage.get("enabled", True):
                continue
            entry = passage.get("entry") or {}
            exit_ = passage.get("exit") or {}
            if not {"x", "y"}.issubset(entry) or not {"x", "y"}.issubset(exit_):
                continue
            entry_xy = (float(entry["x"]), float(entry["y"]))
            exit_xy = (float(exit_["x"]), float(exit_["y"]))
            trigger = passage.get("trigger_policy") or {}
            waypoint_policy = passage.get("waypoint") or {}
            min_distance = float(trigger.get("min_remaining_goal_distance_m", 0.8))
            if remaining_distance < min_distance:
                continue

            current_to_entry = math.dist(start, entry_xy)
            current_to_exit = math.dist(start, exit_xy)
            target_to_entry = math.dist(target, entry_xy)
            target_to_exit = math.dist(target, exit_xy)
            side_margin = float(trigger.get("same_side_skip_margin_m", 0.30))
            if current_to_entry <= current_to_exit:
                waypoint_xy = exit_xy
                waypoint_yaw = float(exit_.get("yaw", entry.get("yaw", target_pose.yaw)))
                direction = "entry_to_exit"
                current_side = "entry"
            else:
                waypoint_xy = entry_xy
                reverse_yaw = float(entry.get("yaw", exit_.get("yaw", target_pose.yaw))) + math.pi
                waypoint_yaw = self._normalize_angle(reverse_yaw)
                direction = "exit_to_entry"
                current_side = "exit"

            if target_to_entry + side_margin < target_to_exit:
                target_side = "entry"
            elif target_to_exit + side_margin < target_to_entry:
                target_side = "exit"
            else:
                target_side = "ambiguous"
            if target_side == current_side:
                continue

            skip_near_exit = float(trigger.get("skip_if_robot_near_exit_m", 0.45))
            if math.dist(start, waypoint_xy) <= skip_near_exit:
                continue
            if math.dist(target, waypoint_xy) <= skip_near_exit:
                continue

            endpoint_distance = min(current_to_entry, current_to_exit, target_to_entry, target_to_exit)
            expected_width = float(passage.get("expected_width_m", 0.8))
            intersection_threshold = max(0.55, expected_width * 0.85)
            plan_segment_distance = None
            route_crosses_passage = False
            if global_plan_points and len(global_plan_points) >= 2:
                plan_segment_distance = self._route_distance_to_segment(global_plan_points, entry_xy, exit_xy)
                route_crosses_passage = plan_segment_distance <= intersection_threshold
            if not route_crosses_passage:
                continue

            score = min(plan_segment_distance if plan_segment_distance is not None else float("inf"), endpoint_distance)
            if best_score is None or score < best_score:
                alignment = self._select_passage_alignment(passage, current_side, start_pose, target_pose.frame_id)
                best_score = score
                best = {
                    "applied": True,
                    "annotation_path": self._navigation_annotations.get("_path"),
                    "passage_id": passage.get("id"),
                    "description": passage.get("description"),
                    "direction": direction,
                    "reason": "global_plan_intersects_narrow_passage",
                    "current_side": current_side,
                    "target_side": target_side,
                    "global_plan_points": len(global_plan_points or []),
                    "plan_segment_distance_m": plan_segment_distance,
                    "endpoint_distance_m": endpoint_distance,
                    "expected_width_m": expected_width,
                    "start_pose": start_pose,
                    "final_goal_pose": target_pose.to_dict(),
                    "waypoint_pose": {
                        "x": waypoint_xy[0],
                        "y": waypoint_xy[1],
                        "yaw": waypoint_yaw,
                        "frame_id": target_pose.frame_id,
                    },
                    "waypoint_xy_goal_tolerance_m": float(waypoint_policy.get("xy_goal_tolerance_m", 0.45)),
                    "waypoint_yaw_goal_tolerance_rad": float(
                        waypoint_policy.get("yaw_goal_tolerance_rad", math.radians(20.0))
                    ),
                    "waypoint_timeout_sec": float(waypoint_policy.get("timeout_sec", 24.0)),
                    "traversal_policy": dict(passage.get("traversal_policy") or {}),
                    # Orient the centerline in the actual traversal direction so
                    # signed cross-track error has the same meaning both ways.
                    "centerline": list(passage.get("centerline") or [])
                    if direction == "entry_to_exit"
                    else list(reversed(passage.get("centerline") or [])),
                }
                if alignment is not None:
                    best.update(alignment)
        return best

    def _diagnose_navigation_annotation(
        self,
        start_pose: dict[str, float],
        target_pose: Pose2D,
        global_plan_points: list[dict[str, float]] | None = None,
    ) -> dict[str, Any]:
        passages = self._navigation_annotations.get("narrow_passages", [])
        diagnostics: dict[str, Any] = {
            "available": bool(passages),
            "selected": False,
            "global_plan_points": len(global_plan_points or []),
            "passages": [],
        }
        if not passages:
            diagnostics["reason"] = "no_navigation_annotations_loaded"
            return diagnostics
        if not global_plan_points or len(global_plan_points) < 2:
            diagnostics["reason"] = "global_plan_missing_or_too_short"

        start = (float(start_pose["x"]), float(start_pose["y"]))
        target = (float(target_pose.x), float(target_pose.y))
        remaining_distance = math.dist(start, target)
        for passage in passages:
            item: dict[str, Any] = {"id": passage.get("id"), "enabled": bool(passage.get("enabled", True))}
            if not passage.get("enabled", True):
                item["skip_reason"] = "disabled"
                diagnostics["passages"].append(item)
                continue
            entry = passage.get("entry") or {}
            exit_ = passage.get("exit") or {}
            if not {"x", "y"}.issubset(entry) or not {"x", "y"}.issubset(exit_):
                item["skip_reason"] = "missing_entry_or_exit"
                diagnostics["passages"].append(item)
                continue
            entry_xy = (float(entry["x"]), float(entry["y"]))
            exit_xy = (float(exit_["x"]), float(exit_["y"]))
            trigger = passage.get("trigger_policy") or {}
            min_distance = float(trigger.get("min_remaining_goal_distance_m", 0.8))
            item["remaining_distance_m"] = remaining_distance
            if remaining_distance < min_distance:
                item["skip_reason"] = "remaining_distance_below_trigger"
                item["min_remaining_goal_distance_m"] = min_distance
                diagnostics["passages"].append(item)
                continue

            current_to_entry = math.dist(start, entry_xy)
            current_to_exit = math.dist(start, exit_xy)
            target_to_entry = math.dist(target, entry_xy)
            target_to_exit = math.dist(target, exit_xy)
            side_margin = float(trigger.get("same_side_skip_margin_m", 0.30))
            current_side = "entry" if current_to_entry <= current_to_exit else "exit"
            if target_to_entry + side_margin < target_to_exit:
                target_side = "entry"
            elif target_to_exit + side_margin < target_to_entry:
                target_side = "exit"
            else:
                target_side = "ambiguous"
            item.update(
                {
                    "current_side": current_side,
                    "target_side": target_side,
                    "current_to_entry_m": current_to_entry,
                    "current_to_exit_m": current_to_exit,
                    "target_to_entry_m": target_to_entry,
                    "target_to_exit_m": target_to_exit,
                }
            )
            if target_side == current_side:
                item["skip_reason"] = "target_on_same_passage_side"
                diagnostics["passages"].append(item)
                continue

            waypoint_xy = exit_xy if current_side == "entry" else entry_xy
            skip_near_exit = float(trigger.get("skip_if_robot_near_exit_m", 0.45))
            item["skip_if_robot_near_exit_m"] = skip_near_exit
            if math.dist(start, waypoint_xy) <= skip_near_exit:
                item["skip_reason"] = "robot_already_near_waypoint"
                diagnostics["passages"].append(item)
                continue
            if math.dist(target, waypoint_xy) <= skip_near_exit:
                item["skip_reason"] = "target_already_near_waypoint"
                diagnostics["passages"].append(item)
                continue

            expected_width = float(passage.get("expected_width_m", 0.8))
            intersection_threshold = max(0.55, expected_width * 0.85)
            plan_segment_distance = None
            if global_plan_points and len(global_plan_points) >= 2:
                plan_segment_distance = self._route_distance_to_segment(global_plan_points, entry_xy, exit_xy)
            item["plan_segment_distance_m"] = plan_segment_distance
            item["intersection_threshold_m"] = intersection_threshold
            if plan_segment_distance is None:
                item["skip_reason"] = "global_plan_missing_or_too_short"
            elif plan_segment_distance > intersection_threshold:
                item["skip_reason"] = "global_plan_does_not_intersect_passage"
            else:
                item["skip_reason"] = "would_select"
            diagnostics["passages"].append(item)

        if "reason" not in diagnostics:
            reasons = [str(item.get("skip_reason")) for item in diagnostics["passages"] if item.get("skip_reason")]
            diagnostics["reason"] = reasons[0] if reasons else "no_matching_navigation_annotation"
        return diagnostics

    def _sample_open_area_waypoint(
        self,
        waypoint: Pose2D,
        final_goal: Pose2D,
        xy_goal_tolerance_m: float,
    ) -> tuple[Pose2D, dict[str, Any] | None]:
        base = self._local_costmap_pose_stats(waypoint, radius_m=0.38)
        if base is None:
            return waypoint, None

        max_radius = max(0.25, min(0.70, float(xy_goal_tolerance_m) + 0.20))
        radii = [0.0, min(0.25, max_radius), min(0.45, max_radius), max_radius]
        angles = [
            0.0,
            math.pi / 4.0,
            math.pi / 2.0,
            3.0 * math.pi / 4.0,
            math.pi,
            -3.0 * math.pi / 4.0,
            -math.pi / 2.0,
            -math.pi / 4.0,
        ]
        candidates: list[tuple[float, Pose2D, dict[str, Any]]] = []
        for radius in radii:
            if radius <= 1e-6:
                pose = waypoint
                offset = (0.0, 0.0)
                stats = base
            else:
                for angle in angles:
                    dx = radius * math.cos(angle)
                    dy = radius * math.sin(angle)
                    pose = Pose2D(
                        x=waypoint.x + dx,
                        y=waypoint.y + dy,
                        yaw=waypoint.yaw,
                        frame_id=waypoint.frame_id,
                    )
                    offset = (dx, dy)
                    stats = self._local_costmap_pose_stats(pose, radius_m=0.38)
                    if stats is None:
                        continue
                    score = self._open_area_score(stats, pose, waypoint, final_goal)
                    candidates.append(
                        (
                            score,
                            pose,
                            {
                                **stats,
                                "offset_from_annotation_m": {"x": dx, "y": dy},
                                "score": score,
                            },
                        )
                    )
                continue
            score = self._open_area_score(stats, pose, waypoint, final_goal)
            candidates.append(
                (
                    score,
                    pose,
                    {
                        **stats,
                        "offset_from_annotation_m": {"x": offset[0], "y": offset[1]},
                        "score": score,
                    },
                )
            )

        if not candidates:
            return waypoint, {
                "available": False,
                "reason": "no_local_costmap_sampling_candidates",
                "base": base,
            }

        base_score = self._open_area_score(base, waypoint, waypoint, final_goal)
        best_score, best_pose, best_stats = min(candidates, key=lambda item: item[0])
        changed = math.hypot(best_pose.x - waypoint.x, best_pose.y - waypoint.y) >= 0.05
        meaningful_improvement = best_score <= base_score - 0.18 or (
            base.get("blocked_ratio", 0.0) > 0.02
            and best_stats.get("blocked_ratio", 1.0) < base.get("blocked_ratio", 0.0)
        )
        info = {
            "available": True,
            "source": "local_costmap_open_area_sampling",
            "selected": changed and meaningful_improvement,
            "base": {**base, "score": base_score},
            "selected_candidate": best_stats,
            "candidate_count": len(candidates),
            "max_sample_radius_m": max_radius,
        }
        if changed and meaningful_improvement:
            return best_pose, info
        return waypoint, info

    def _open_area_score(
        self,
        stats: dict[str, Any],
        pose: Pose2D,
        original_waypoint: Pose2D,
        final_goal: Pose2D,
    ) -> float:
        offset = math.hypot(pose.x - original_waypoint.x, pose.y - original_waypoint.y)
        final_distance = math.hypot(pose.x - final_goal.x, pose.y - final_goal.y)
        return (
            float(stats.get("blocked_ratio", 1.0)) * 6.0
            + float(stats.get("inflated_ratio", 1.0)) * 1.6
            + float(stats.get("unknown_ratio", 1.0)) * 1.0
            + offset * 0.25
            + final_distance * 0.03
        )

    def _local_costmap_pose_stats(self, pose: Pose2D, radius_m: float) -> dict[str, Any] | None:
        msg = self._latest_local_costmap
        if msg is None:
            return None
        age = time.monotonic() - self._latest_local_costmap_time
        if age > 2.0:
            return None
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None
        frame_id = msg.header.frame_id or pose.frame_id
        transformed = self._transform_xy(frame_id, pose.frame_id, pose.x, pose.y)
        if transformed is None:
            return None
        frame_x, frame_y = transformed
        origin = msg.info.origin
        origin_yaw = self._yaw_from_quaternion(origin.orientation)
        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)
        dx = frame_x - float(origin.position.x)
        dy = frame_y - float(origin.position.y)
        grid_x_m = origin_cos * dx + origin_sin * dy
        grid_y_m = -origin_sin * dx + origin_cos * dy
        center_x = int(math.floor(grid_x_m / resolution))
        center_y = int(math.floor(grid_y_m / resolution))
        if center_x < 0 or center_x >= width or center_y < 0 or center_y >= height:
            return {
                "available": False,
                "reason": "sample_pose_outside_local_costmap",
                "frame_id": frame_id,
                "age_sec": age,
                "center_cell": {"x": center_x, "y": center_y},
                "blocked_ratio": 1.0,
                "inflated_ratio": 1.0,
                "unknown_ratio": 1.0,
            }

        radius_cells = max(1, int(math.ceil(radius_m / resolution)))
        blocked = 0
        inflated = 0
        unknown = 0
        cells = 0
        nearest_obstacle = None
        data = msg.data
        for my in range(max(0, center_y - radius_cells), min(height, center_y + radius_cells + 1)):
            for mx in range(max(0, center_x - radius_cells), min(width, center_x + radius_cells + 1)):
                cell_dx = (mx - center_x) * resolution
                cell_dy = (my - center_y) * resolution
                distance = math.hypot(cell_dx, cell_dy)
                if distance > radius_m:
                    continue
                cells += 1
                cost = int(data[my * width + mx])
                if cost < 0:
                    unknown += 1
                    continue
                if cost >= 65:
                    blocked += 1
                    nearest_obstacle = distance if nearest_obstacle is None else min(nearest_obstacle, distance)
                if cost >= 25:
                    inflated += 1
        cell_count = max(1, cells)
        return {
            "available": True,
            "frame_id": frame_id,
            "age_sec": age,
            "sample_radius_m": radius_m,
            "center_cell": {"x": center_x, "y": center_y},
            "cells": cells,
            "blocked": blocked,
            "inflated": inflated,
            "unknown": unknown,
            "blocked_ratio": blocked / cell_count,
            "inflated_ratio": inflated / cell_count,
            "unknown_ratio": unknown / cell_count,
            "nearest_obstacle_m": nearest_obstacle,
        }

    def _transform_xy(
        self,
        target_frame: str,
        source_frame: str,
        x: float,
        y: float,
    ) -> tuple[float, float] | None:
        if target_frame == source_frame:
            return float(x), float(y)
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        yaw = self._yaw_from_quaternion(tf.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        return tx + c * float(x) - s * float(y), ty + s * float(x) + c * float(y)

    def _route_distance_to_segment(
        self,
        route_points: list[dict[str, float]],
        b0: tuple[float, float],
        b1: tuple[float, float],
    ) -> float:
        best = None
        previous = route_points[0]
        for current in route_points[1:]:
            a0 = (float(previous["x"]), float(previous["y"]))
            a1 = (float(current["x"]), float(current["y"]))
            distance = self._segment_distance(a0, a1, b0, b1)
            best = distance if best is None else min(best, distance)
            previous = current
            if best <= 1e-6:
                return 0.0
        return float("inf") if best is None else best

    def _select_passage_alignment(
        self,
        passage: dict[str, Any],
        current_side: str,
        start_pose: dict[str, float],
        frame_id: str,
    ) -> dict[str, Any] | None:
        zones = passage.get("alignment_zones") or []
        if not isinstance(zones, list):
            return None
        start_xy = (float(start_pose["x"]), float(start_pose["y"]))
        start_yaw = float(start_pose.get("yaw", 0.0))
        best = None
        best_distance = None
        for zone in zones:
            if not isinstance(zone, dict) or not zone.get("enabled", False):
                continue
            if str(zone.get("side") or "").lower() != current_side:
                continue
            pose = zone.get("pose") or zone.get("staging_pose") or {}
            if not {"x", "y", "yaw"}.issubset(pose):
                continue
            pose_xy = (float(pose["x"]), float(pose["y"]))
            distance = math.dist(start_xy, pose_xy)
            yaw_delta = abs(self._normalize_angle(start_yaw - float(pose["yaw"])))
            policy = zone.get("use_policy") or {}
            skip_distance = float(policy.get("skip_if_robot_near_pose_m", 0.12))
            yaw_tolerance = min(
                float(zone.get("yaw_goal_tolerance_rad", math.radians(20.0))),
                NARROW_PASSAGE_HEADING_TOLERANCE_RAD,
            )
            if distance <= skip_distance and yaw_delta <= yaw_tolerance:
                continue
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best = {
                    "alignment_zone": {
                        "id": zone.get("id"),
                        "side": current_side,
                        "description": zone.get("description"),
                        "distance_from_start_m": distance,
                        "yaw_delta_from_start_rad": yaw_delta,
                        "polygon": zone.get("polygon"),
                    },
                    "alignment_goal_pose": {
                        "x": float(pose["x"]),
                        "y": float(pose["y"]),
                        "yaw": float(pose["yaw"]),
                        "frame_id": pose.get("frame_id", frame_id),
                    },
                    "alignment_xy_goal_tolerance_m": float(zone.get("xy_goal_tolerance_m", 0.45)),
                    "alignment_yaw_goal_tolerance_rad": yaw_tolerance,
                    "alignment_timeout_sec": float(zone.get("timeout_sec", 16.0)),
                    "alignment_use_policy": policy,
                }
        return best

    @classmethod
    def _segment_distance(
        cls,
        a0: tuple[float, float],
        a1: tuple[float, float],
        b0: tuple[float, float],
        b1: tuple[float, float],
    ) -> float:
        if cls._segments_intersect(a0, a1, b0, b1):
            return 0.0
        return min(
            cls._point_to_segment_distance(a0, b0, b1),
            cls._point_to_segment_distance(a1, b0, b1),
            cls._point_to_segment_distance(b0, a0, a1),
            cls._point_to_segment_distance(b1, a0, a1),
        )

    @staticmethod
    def _point_to_segment_distance(
        point: tuple[float, float],
        a: tuple[float, float],
        b: tuple[float, float],
    ) -> float:
        px, py = point
        ax, ay = a
        bx, by = b
        dx = bx - ax
        dy = by - ay
        denom = dx * dx + dy * dy
        if denom <= 1e-9:
            return math.dist(point, a)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
        closest = (ax + t * dx, ay + t * dy)
        return math.dist(point, closest)

    @classmethod
    def _segments_intersect(
        cls,
        a0: tuple[float, float],
        a1: tuple[float, float],
        b0: tuple[float, float],
        b1: tuple[float, float],
    ) -> bool:
        def orient(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

        def on_segment(p, q, r):
            return (
                min(p[0], r[0]) - 1e-9 <= q[0] <= max(p[0], r[0]) + 1e-9
                and min(p[1], r[1]) - 1e-9 <= q[1] <= max(p[1], r[1]) + 1e-9
            )

        o1 = orient(a0, a1, b0)
        o2 = orient(a0, a1, b1)
        o3 = orient(b0, b1, a0)
        o4 = orient(b0, b1, a1)
        if o1 * o2 < 0.0 and o3 * o4 < 0.0:
            return True
        if abs(o1) <= 1e-9 and on_segment(a0, b0, a1):
            return True
        if abs(o2) <= 1e-9 and on_segment(a0, b1, a1):
            return True
        if abs(o3) <= 1e-9 and on_segment(b0, a0, b1):
            return True
        if abs(o4) <= 1e-9 and on_segment(b0, a1, b1):
            return True
        return False

    def _task_goal_reached(
        self,
        pose: Pose2D,
        xy_goal_tolerance_m: float | None,
        yaw_goal_tolerance_rad: float | None,
    ) -> bool:
        if xy_goal_tolerance_m is None and yaw_goal_tolerance_rad is None:
            return False
        current = self._current_map_pose()
        if current is None:
            return False
        if xy_goal_tolerance_m is not None:
            xy_error = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
            if xy_error > max(0.0, xy_goal_tolerance_m):
                return False
        if yaw_goal_tolerance_rad is not None:
            yaw_error = abs(self._normalize_angle(current["yaw"] - pose.yaw))
            if yaw_error > max(0.0, yaw_goal_tolerance_rad):
                return False
        return True

    def _pose_within_task_tolerance(
        self,
        current: dict[str, float],
        pose: Pose2D,
        xy_goal_tolerance_m: float | None,
        yaw_goal_tolerance_rad: float | None,
    ) -> bool:
        if xy_goal_tolerance_m is None and yaw_goal_tolerance_rad is None:
            return False
        if xy_goal_tolerance_m is not None:
            xy_error = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
            if xy_error > max(0.0, xy_goal_tolerance_m):
                return False
        if yaw_goal_tolerance_rad is not None:
            yaw_error = abs(self._normalize_angle(current["yaw"] - pose.yaw))
            if yaw_error > max(0.0, yaw_goal_tolerance_rad):
                return False
        return True

    @staticmethod
    def _recovery_policy_allows(policy: dict[str, Any], key: str, default: bool) -> bool:
        if str(policy.get("recovery_mode", "")).lower() == "fixed_point":
            return False
        if key not in policy:
            return default
        return bool(policy.get(key))

    def _pose_within_relaxed_region_tolerance(
        self,
        current: dict[str, float],
        pose: Pose2D,
        xy_goal_tolerance_m: float = 0.65,
    ) -> bool:
        xy_error = math.hypot(current["x"] - pose.x, current["y"] - pose.y)
        return xy_error <= max(0.0, xy_goal_tolerance_m)

    def _try_near_goal_micro_adjust(
        self,
        context: SkillContext,
        pose: Pose2D,
        current_pose: dict[str, float],
        xy_goal_tolerance_m: float | None,
        yaw_goal_tolerance_rad: float | None,
    ) -> SkillResult | None:
        xy_error = math.hypot(current_pose["x"] - pose.x, current_pose["y"] - pose.y)
        if xy_error > 1.0:
            return None
        target_in_base = self._transform_xy("base_footprint", pose.frame_id, pose.x, pose.y)
        if target_in_base is None:
            return None
        base_x, base_y = target_in_base
        distance = math.hypot(base_x, base_y)
        if distance < 0.08:
            return None
        max_step = 0.35
        scale = min(1.0, max_step / distance)
        return self._direct_translate(
            "goto_pose_near_goal_micro_adjust",
            context,
            base_x * scale,
            base_y * scale,
            5.0,
            {
                "reason": "near_goal_timeout_direct_xy_adjust",
                "target_pose": pose.to_dict(),
                "current_pose": current_pose,
                "xy_error_m": xy_error,
                "target_in_base": {"x": base_x, "y": base_y},
                "command_scale": scale,
                "xy_goal_tolerance_m": xy_goal_tolerance_m,
                "yaw_goal_tolerance_rad": yaw_goal_tolerance_rad,
            },
            motion_guard=lambda: self._micro_adjust_guard(base_x * scale, base_y * scale),
        )

    def _footprint_cb(self, msg):
        self._latest_footprint = msg
        self._latest_footprint_time = time.monotonic()

    def _navigation_localization_ok(self):
        callback = self._localization_metrics
        return callback is not None and callback().get("localization_healthy") is True

    def _micro_adjust_guard(self, dx, dy, rotating=False):
        if not self._navigation_localization_ok():
            return "navigation_localization_unhealthy"
        grid, footprint = self._latest_local_costmap, self._latest_footprint
        wall = time.monotonic()
        if grid is None or footprint is None or wall - min(self._latest_local_costmap_time, self._latest_footprint_time) > 2:
            return "micro_adjust_collision_evidence_stale"
        source_now = self.node.get_clock().now().nanoseconds / 1e9
        for msg in (grid, footprint):
            age = source_now - (msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
            if not -0.1 <= age <= 2:
                return "micro_adjust_collision_stamp_stale"
        frame = grid.header.frame_id
        if frame != footprint.header.frame_id:
            return "micro_adjust_footprint_frame_mismatch"
        try:
            tf = self._tf_buffer.lookup_transform(frame, "base_footprint", Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return "micro_adjust_tf_unavailable"
        tf_age = source_now - (tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9)
        if not -0.1 <= tf_age <= 2:
            return "micro_adjust_tf_stale"
        yaw = self._yaw_from_quaternion(tf.transform.rotation)
        delta = (math.cos(yaw) * dx - math.sin(yaw) * dy, math.sin(yaw) * dx + math.cos(yaw) * dy)
        origin = grid.info.origin
        polygon = [(p.x, p.y) for p in footprint.polygon.points]
        if rotating and polygon:
            cx, cy = tf.transform.translation.x, tf.transform.translation.y
            radius = max(math.hypot(x - cx, y - cy) for x, y in polygon)
            # Enclose every orientation, including corners between samples.
            polygon = [(cx + sx * radius, cy + sy * radius) for sx, sy in ((-1, -1), (-1, 1), (1, 1), (1, -1))]
        clear = swept_grid_clear(grid.data, grid.info.width, grid.info.height, grid.info.resolution,
                                 (origin.position.x, origin.position.y, self._yaw_from_quaternion(origin.orientation)),
                                 polygon, delta)
        return None if clear else "micro_adjust_swept_footprint_blocked"

    def _sample_trusted_front_open_area_goal(
        self,
        original_goal: Pose2D,
        current_pose: dict[str, float],
    ) -> tuple[Pose2D | None, dict[str, Any]]:
        msg = self._latest_local_costmap
        if msg is None:
            return None, {"available": False, "reason": "local_costmap_unavailable"}
        age = time.monotonic() - self._latest_local_costmap_time
        if age > 2.0:
            return None, {"available": False, "reason": "local_costmap_stale", "age_sec": age}

        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None, {"available": False, "reason": "invalid_local_costmap_geometry"}

        frame_id = msg.header.frame_id or original_goal.frame_id
        try:
            base_tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                frame_id,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            return None, {
                "available": False,
                "reason": "base_to_local_costmap_tf_unavailable",
                "frame_id": frame_id,
                "error": repr(exc),
            }

        origin = msg.info.origin
        origin_yaw = self._yaw_from_quaternion(origin.orientation)
        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)
        tf_yaw = self._yaw_from_quaternion(base_tf.transform.rotation)
        tf_cos = math.cos(tf_yaw)
        tf_sin = math.sin(tf_yaw)
        tf_tx = float(base_tf.transform.translation.x)
        tf_ty = float(base_tf.transform.translation.y)

        data = msg.data
        max_angle = math.radians(80.0)
        candidates: list[tuple[float, Pose2D, dict[str, Any]]] = []
        step = max(1, int(round(0.10 / resolution)))
        for my in range(0, height, step):
            cell_y = (my + 0.5) * resolution
            for mx in range(0, width, step):
                cost = int(data[my * width + mx])
                if cost < 0 or cost >= 25:
                    continue
                cell_x = (mx + 0.5) * resolution
                frame_x = float(origin.position.x) + origin_cos * cell_x - origin_sin * cell_y
                frame_y = float(origin.position.y) + origin_sin * cell_x + origin_cos * cell_y
                base_x = tf_tx + tf_cos * frame_x - tf_sin * frame_y
                base_y = tf_ty + tf_sin * frame_x + tf_cos * frame_y
                distance = math.hypot(base_x, base_y)
                if distance < 0.30 or distance > 1.80:
                    continue
                angle = abs(math.atan2(base_y, base_x))
                if angle > max_angle:
                    continue
                map_xy = self._transform_xy(original_goal.frame_id, frame_id, frame_x, frame_y)
                if map_xy is None:
                    continue
                candidate = Pose2D(
                    x=map_xy[0],
                    y=map_xy[1],
                    yaw=original_goal.yaw,
                    frame_id=original_goal.frame_id,
                )
                stats = self._local_costmap_pose_stats(candidate, radius_m=0.38)
                if not stats or not stats.get("available", False):
                    continue
                if (
                    float(stats.get("blocked_ratio", 1.0)) > 0.0
                    or float(stats.get("inflated_ratio", 1.0)) > 0.42
                    or float(stats.get("unknown_ratio", 1.0)) > 0.20
                ):
                    continue
                goal_distance = math.hypot(candidate.x - original_goal.x, candidate.y - original_goal.y)
                current_distance = math.hypot(candidate.x - current_pose["x"], candidate.y - current_pose["y"])
                score = (
                    goal_distance
                    + current_distance * 0.20
                    + float(stats.get("inflated_ratio", 0.0)) * 1.5
                    + angle * 0.10
                )
                candidates.append(
                    (
                        score,
                        candidate,
                        {
                            **stats,
                            "score": score,
                            "base_xy": {"x": base_x, "y": base_y},
                            "base_distance_m": distance,
                            "base_angle_rad": math.atan2(base_y, base_x),
                            "goal_distance_m": goal_distance,
                            "current_distance_m": current_distance,
                            "cell": {"x": mx, "y": my},
                        },
                    )
                )

        info: dict[str, Any] = {
            "available": True,
            "source": "trusted_front_local_costmap_sampling",
            "front_sector_deg": 80.0,
            "min_distance_m": 0.30,
            "max_distance_m": 1.80,
            "candidate_count": len(candidates),
            "age_sec": age,
        }
        if not candidates:
            info["selected"] = False
            info["reason"] = "no_trusted_front_free_candidate"
            return None, info
        _score, best_pose, best_stats = min(candidates, key=lambda item: item[0])
        info["selected"] = True
        info["selected_candidate"] = best_stats
        info["selected_pose"] = best_pose.to_dict()
        return best_pose, info

    def move_relative(
        self,
        context: SkillContext,
        direction: str,
        distance_m: float,
        timeout_sec: float = 30.0,
    ) -> SkillResult:
        started = now()
        direction = str(direction).lower()
        distance = float(distance_m)
        if direction not in {"forward", "back", "left", "right"}:
            return make_result(
                "move_relative",
                False,
                codes.FAILED_SCHEMA,
                "direction must be one of: forward, back, left, right.",
                started,
                {"direction": direction},
            )
        if distance <= 0.0 or distance > 2.0:
            return make_result(
                "move_relative",
                False,
                codes.FAILED_SCHEMA,
                "distance_m must be in (0, 2.0].",
                started,
                {"distance_m": distance},
            )
        current = self._current_map_pose()
        local_dx = 0.0
        local_dy = 0.0
        if direction == "forward":
            local_dx = distance
        elif direction == "back":
            local_dx = -distance
        elif direction == "left":
            local_dy = distance
        elif direction == "right":
            local_dy = -distance
        if context.mode == "dry_run":
            return dry_run_result(
                "move_relative",
                "Dry-run direct relative motion accepted.",
                {
                    "control_mode": "direct_cmd_vel_raw",
                    "direction": direction,
                    "distance_m": distance,
                    "current_pose": current,
                },
            )
        return self._direct_translate(
            "move_relative",
            context,
            local_dx,
            local_dy,
            timeout_sec,
            {"direction": direction, "distance_m": distance, "current_pose": current},
        )

    def move_relative_xy(
        self,
        context: SkillContext,
        dx_m: float,
        dy_m: float,
        timeout_sec: float = 30.0,
    ) -> SkillResult:
        started = now()
        dx = float(dx_m)
        dy = float(dy_m)
        distance = math.hypot(dx, dy)
        if distance <= 0.0 or distance > 2.0:
            return make_result(
                "move_relative_xy",
                False,
                codes.FAILED_SCHEMA,
                "relative xy distance must be in (0, 2.0].",
                started,
                {"dx_m": dx, "dy_m": dy, "distance_m": distance},
            )
        current = self._current_map_pose()
        if context.mode == "dry_run":
            return dry_run_result(
                "move_relative_xy",
                "Dry-run direct relative xy motion accepted.",
                {
                    "control_mode": "direct_cmd_vel_raw",
                    "dx_m": dx,
                    "dy_m": dy,
                    "distance_m": distance,
                    "current_pose": current,
                },
            )
        return self._direct_translate(
            "move_relative_xy",
            context,
            dx,
            dy,
            timeout_sec,
            {"dx_m": dx, "dy_m": dy, "distance_m": distance, "current_pose": current},
        )

    def rotate_relative(
        self,
        context: SkillContext,
        yaw_deg: float,
        timeout_sec: float = 30.0,
    ) -> SkillResult:
        started = now()
        yaw_delta = math.radians(float(yaw_deg))
        if abs(yaw_delta) <= 0.0 or abs(yaw_delta) > math.pi:
            return make_result(
                "rotate_relative",
                False,
                codes.FAILED_SCHEMA,
                "yaw_deg must be in [-180, 180] and non-zero.",
                started,
                {"yaw_deg": yaw_deg},
            )
        current = self._current_map_pose()
        if context.mode == "dry_run":
            return dry_run_result(
                "rotate_relative",
                "Dry-run direct relative rotation accepted.",
                {
                    "control_mode": "direct_cmd_vel_raw",
                    "yaw_deg": float(yaw_deg),
                    "current_pose": current,
                },
            )
        return self._direct_rotate(
            "rotate_relative",
            context,
            yaw_delta,
            timeout_sec,
            {"yaw_deg": float(yaw_deg), "current_pose": current},
        )

    def _direct_translate(
        self,
        skill_name: str,
        context: SkillContext,
        dx_m: float,
        dy_m: float,
        timeout_sec: float,
        metrics: dict[str, Any],
        motion_guard=None,
    ) -> SkillResult:
        started = now()
        start_pose = self._current_map_pose()
        start_wheel_pose = self._current_wheel_odom_pose()
        distance = math.hypot(dx_m, dy_m)
        if start_wheel_pose is None:
            return make_result(
                skill_name,
                False,
                codes.FAILED_LOCALIZATION,
                "Fresh wheel odom is unavailable; map corrections cannot measure relative motion.",
                started,
                metrics,
            )
        if distance <= 0.0:
            return make_result(skill_name, True, codes.OK, "No relative motion requested.", started, metrics)

        self._direct_motion_cancel = False
        speed = min(0.18, max(0.06, distance / 1.6))
        vx = speed * dx_m / distance
        vy = speed * dy_m / distance
        motion_clock = MotionClock(float(timeout_sec), context.mode == "sim",
                                   self.node.get_clock().now().nanoseconds / 1e9, time.monotonic())
        progress = MotionProgress(distance, motion_clock.start_source, motion_clock.start_wall, .01)
        along = cross = 0.0
        position_error = distance
        reached = False
        canceled = False
        last_pose = start_pose
        last_wheel_pose = start_wheel_pose
        while not motion_clock.expired(self.node.get_clock().now().nanoseconds / 1e9, time.monotonic()):
            if self._direct_motion_cancel or getattr(context, "recovery_check", lambda: False)():
                canceled = True
                break
            if motion_guard is not None:
                motion_clock.reason = motion_guard()
                if motion_clock.reason:
                    break
            current_wheel = self._current_wheel_odom_pose()
            if start_wheel_pose is not None and current_wheel is None:
                motion_clock.reason = "wheel_odom_stale"
                metrics["wheel_odom_rejection"] = dict(getattr(self, "_last_wheel_odom_evidence", {}))
                break
            current = self._current_map_pose()
            if current_wheel is not None and start_wheel_pose is not None:
                last_wheel_pose = current_wheel
                along, cross, position_error = translation_error(start_wheel_pose, current_wheel, dx_m, dy_m)
                if position_error <= max(.01, distance * .05):
                    reached = True
                    break
                if cross > max(.05, distance * .15) or along < -.05 or along > distance + .05:
                    motion_clock.reason = "relative_motion_off_path"
                    break
            if progress.stalled(position_error, self.node.get_clock().now().nanoseconds / 1e9, time.monotonic()):
                motion_clock.reason = "motion_no_progress"
                break
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._publish_stop()
        elapsed = time.monotonic() - motion_clock.start_wall
        metrics.update(timeout_time_basis="sim" if motion_clock.simulation else "wall",
                       wall_timeout_sec=motion_clock.wall_limit, timeout_reason=motion_clock.reason)
        final_pose = self._current_map_pose() or last_pose
        final_wheel_pose = self._current_wheel_odom_pose() or last_wheel_pose
        final_distance = None
        distance_source = "map_tf"
        if final_wheel_pose is not None and start_wheel_pose is not None:
            final_distance = math.hypot(
                final_wheel_pose["x"] - start_wheel_pose["x"],
                final_wheel_pose["y"] - start_wheel_pose["y"],
            )
            distance_source = "wheel_odom"
        elif final_pose is not None and start_pose is not None:
            final_distance = math.hypot(final_pose["x"] - start_pose["x"], final_pose["y"] - start_pose["y"])
        metrics.update(
            {
                "control_mode": "direct_cmd_vel_raw",
                "command_topic": "/cmd_vel_raw",
                "command_vx": vx,
                "command_vy": vy,
                "command_wz": 0.0,
                "duration_sec": elapsed,
                "start_pose": start_pose,
                "final_pose": final_pose,
                "start_wheel_odom_pose": start_wheel_pose,
                "final_wheel_odom_pose": final_wheel_pose,
                "travelled_m": final_distance,
                "along_track_m": along,
                "cross_track_m": cross,
                "position_error_m": position_error,
                "travelled_source": distance_source,
                "target_distance_m": distance,
                "canceled": canceled,
            }
        )
        success = not canceled and motion_clock.reason is None and reached
        code = codes.CANCELLED if canceled else (codes.OK if success else motion_failure_code(motion_clock.reason))
        message = (
            "Direct relative motion was canceled."
            if canceled
            else (
                "Direct relative motion completed."
                if success
                else f"Direct relative motion stopped before completion: {motion_clock.reason}."
            )
        )
        return make_result(skill_name, success, code, message, started, metrics)

    def _direct_rotate(
        self,
        skill_name: str,
        context: SkillContext,
        yaw_delta: float,
        timeout_sec: float,
        metrics: dict[str, Any],
        yaw_tolerance_rad: float | None = None,
    ) -> SkillResult:
        started = now()
        start_pose = self._current_map_pose()
        start_wheel_pose = self._current_wheel_odom_pose()
        if start_wheel_pose is None:
            return make_result(
                skill_name,
                False,
                codes.FAILED_LOCALIZATION,
                "Fresh wheel odom is unavailable; map corrections cannot measure relative rotation.",
                started,
                metrics,
            )
        feedback_start_pose = start_wheel_pose or start_pose
        target_abs = abs(yaw_delta)
        # Sim drive rejects wheel speeds below 0.055 m/s; 0.18 rad/s clears
        # that deadband for this chassis without changing the driver policy.
        minimum_speed = .18 if context.mode == "sim" else .08
        speed = min(0.25, max(minimum_speed, target_abs / 2.0))
        wz = speed if yaw_delta > 0.0 else -speed
        target_yaw = self._normalize_angle(float(feedback_start_pose["yaw"]) + yaw_delta)
        motion_clock = MotionClock(float(timeout_sec), context.mode == "sim",
                                   self.node.get_clock().now().nanoseconds / 1e9, time.monotonic())
        progress = MotionProgress(target_abs, motion_clock.start_source, motion_clock.start_wall, .02)
        accumulated_yaw = 0.0
        previous_yaw = float(feedback_start_pose["yaw"])
        reached = False
        canceled = False
        last_pose = start_pose
        last_wheel_pose = start_wheel_pose
        yaw_error = target_abs
        tolerance = (float(yaw_tolerance_rad) if yaw_tolerance_rad is not None
                     else max(0.02, min(0.05, target_abs * 0.10)))
        self._direct_motion_cancel = False
        while not motion_clock.expired(self.node.get_clock().now().nanoseconds / 1e9, time.monotonic()):
            if self._direct_motion_cancel or getattr(context, "recovery_check", lambda: False)():
                canceled = True
                break
            if skill_name.startswith("goto_pose_"):
                motion_clock.reason = self._micro_adjust_guard(0, 0, rotating=True)
                if motion_clock.reason:
                    break
            current_wheel = self._current_wheel_odom_pose()
            if start_wheel_pose is not None and current_wheel is None:
                motion_clock.reason = "wheel_odom_stale"
                metrics["wheel_odom_rejection"] = dict(getattr(self, "_last_wheel_odom_evidence", {}))
                break
            current = self._current_map_pose()
            feedback_current = current_wheel or current
            if current_wheel is not None:
                last_wheel_pose = current_wheel
            if current is not None:
                last_pose = current
            if feedback_current is not None:
                current_yaw = float(feedback_current["yaw"])
                accumulated_yaw += self._normalize_angle(current_yaw - previous_yaw)
                previous_yaw = current_yaw
                remaining = yaw_delta - accumulated_yaw
                yaw_error = abs(remaining)
                if yaw_error <= tolerance:
                    reached = True
                    break
                wz = math.copysign(min(speed, max(minimum_speed, yaw_error * .8)), remaining)
            if progress.stalled(yaw_error, self.node.get_clock().now().nanoseconds / 1e9, time.monotonic()):
                motion_clock.reason = "motion_no_progress"
                break
            twist = Twist()
            twist.angular.z = wz
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._publish_stop()
        elapsed = time.monotonic() - motion_clock.start_wall
        metrics.update(timeout_time_basis="sim" if motion_clock.simulation else "wall",
                       wall_timeout_sec=motion_clock.wall_limit, timeout_reason=motion_clock.reason)
        final_pose = self._current_map_pose() or last_pose
        final_wheel_pose = self._current_wheel_odom_pose() or last_wheel_pose
        feedback_final_pose = final_wheel_pose or final_pose
        final_yaw_delta = None
        if feedback_final_pose is not None:
            final_yaw_delta = abs(self._normalize_angle(feedback_final_pose["yaw"] - feedback_start_pose["yaw"]))
            yaw_error = abs(self._normalize_angle(target_yaw - float(feedback_final_pose["yaw"])))
        metrics.update(
            {
                "control_mode": "direct_cmd_vel_raw",
                "command_topic": "/cmd_vel_raw",
                "command_vx": 0.0,
                "command_vy": 0.0,
                "command_wz": wz,
                "duration_sec": elapsed,
                "start_pose": start_pose,
                "final_pose": final_pose,
                "start_wheel_odom_pose": start_wheel_pose,
                "final_wheel_odom_pose": final_wheel_pose,
                "turned_rad": final_yaw_delta,
                "signed_accumulated_yaw_rad": accumulated_yaw,
                "turned_source": "wheel_odom" if final_wheel_pose is not None and start_wheel_pose is not None else "map_tf",
                "target_yaw_delta_rad": target_abs,
                "target_yaw_rad": target_yaw,
                "final_yaw_error_rad": yaw_error,
                "canceled": canceled,
            }
        )
        metrics["yaw_goal_tolerance_rad"] = tolerance
        success = not canceled and motion_clock.reason is None and reached
        code = codes.CANCELLED if canceled else (codes.OK if success else motion_failure_code(motion_clock.reason))
        message = (
            "Direct relative rotation was canceled."
            if canceled
            else (
                "Direct relative rotation completed."
                if success
                else f"Direct relative rotation stopped before completion: {motion_clock.reason}."
            )
        )
        return make_result(skill_name, success, code, message, started, metrics)

    def _publish_stop(self):
        stop = Twist()
        for _ in range(8):
            self._cmd_vel_pub.publish(stop)
            time.sleep(0.02)

    def cancel_navigation(self, context: SkillContext | None = None) -> SkillResult:
        started = now()
        self._direct_motion_cancel = True
        self._publish_stop()
        self._clear_navigation_goal_markers()
        self._clear_navigation_route_markers()
        if self._active_goal_handle is None:
            return make_result("cancel_motion", True, codes.OK, "Stop requested for direct motion; no active Nav2 goal.", started)
        self._active_goal_handle.cancel_goal_async()
        self._active_goal_handle = None
        return make_result("cancel_motion", True, codes.CANCELLED, "Cancel requested for active Nav2 goal.", started)

    def _wheel_odom_cb(self, msg: Odometry):
        if msg.header.frame_id.lstrip("/") != "odom" or msg.child_frame_id.lstrip("/") != "base_footprint":
            return
        pose = msg.pose.pose
        with self._wheel_odom_lock:
            self._recovery_wheel_twist = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
            self._latest_wheel_odom_pose = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": self._yaw_from_quaternion(pose.orientation),
            }
            self._latest_wheel_odom_time = time.monotonic()
            self._latest_wheel_odom_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def recovery_motion_stopped(self):
        # Both the goal result (not merely cancel request acceptance) and fresh
        # physical motion feedback are needed before Recovery may reseed.
        if self._pending_nav_goals:
            return False
        for future in self._nav_result_futures:
            if not future.done():
                return False
            try:
                result = future.result()
                if result is None or result.status not in {4, 5, 6}:
                    return False
            except Exception:
                return False  # A failed result transport is not a terminal goal acknowledgement.
        with self._wheel_odom_lock:
            velocity = getattr(self, "_recovery_wheel_twist", None)
            wall_age = time.monotonic() - self._latest_wheel_odom_time
            source_age = (self.node.get_clock().now().nanoseconds / 1e9 - self._latest_wheel_odom_stamp
                          if self._latest_wheel_odom_stamp is not None else None)
            return (velocity is not None and 0 <= wall_age <= .75
                    and source_age is not None and 0 <= source_age <= .75
                    and all(math.isfinite(value) and abs(value) <= .02 for value in velocity))

    def _current_wheel_odom_pose(self) -> dict[str, float] | None:
        with self._wheel_odom_lock:
            wall_age = time.monotonic() - self._latest_wheel_odom_time
            source_age = (self.node.get_clock().now().nanoseconds / 1e9 - self._latest_wheel_odom_stamp
                          if self._latest_wheel_odom_stamp is not None else None)
            evidence = {"receive_age_sec": wall_age, "source_age_sec": source_age,
                        "stamp_sec": self._latest_wheel_odom_stamp, "max_age_sec": .75,
                        "future_tolerance_sec": .05, "reason": None}
            self._last_wheel_odom_evidence = evidence
            if self._latest_wheel_odom_pose is None:
                evidence["reason"] = "not_received"
                return None
            if wall_age > 0.75:
                evidence["reason"] = "receive_stale"
                return None
            if source_age is not None and not -0.05 <= source_age <= 0.75:
                evidence["reason"] = "stamp_future" if source_age < -.05 else "source_stale"
                return None
            return dict(self._latest_wheel_odom_pose)

    def _local_costmap_cb(self, msg: OccupancyGrid):
        self._latest_local_costmap = msg
        self._latest_local_costmap_time = time.monotonic()

    def _local_clearance_snapshot(self) -> dict[str, Any] | None:
        msg = self._latest_local_costmap
        if msg is None:
            return None
        age = time.monotonic() - self._latest_local_costmap_time
        if age > 2.0:
            return {
                "available": False,
                "reason": "local_costmap_stale",
                "age_sec": age,
            }
        width = int(msg.info.width)
        height = int(msg.info.height)
        resolution = float(msg.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return {
                "available": False,
                "reason": "invalid_local_costmap_geometry",
                "age_sec": age,
            }
        frame_id = msg.header.frame_id or "map"
        try:
            tf = self._tf_buffer.lookup_transform(
                "base_footprint",
                frame_id,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            return {
                "available": False,
                "reason": "local_costmap_tf_unavailable",
                "frame_id": frame_id,
                "error": repr(exc),
                "age_sec": age,
            }

        origin = msg.info.origin
        origin_yaw = self._yaw_from_quaternion(origin.orientation)
        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)
        tf_yaw = self._yaw_from_quaternion(tf.transform.rotation)
        tf_cos = math.cos(tf_yaw)
        tf_sin = math.sin(tf_yaw)
        tf_tx = float(tf.transform.translation.x)
        tf_ty = float(tf.transform.translation.y)

        regions = {
            "front": self._empty_region_stats(),
            "left": self._empty_region_stats(),
            "right": self._empty_region_stats(),
            "rotate": self._empty_region_stats(),
        }
        nearest_obstacle = None

        def add(region: str, cost: int, distance: float):
            stats = regions[region]
            stats["cells"] += 1
            if cost < 0:
                stats["unknown"] += 1
                return
            if cost >= 65:
                stats["blocked"] += 1
                stats["nearest_obstacle_m"] = (
                    distance
                    if stats["nearest_obstacle_m"] is None
                    else min(stats["nearest_obstacle_m"], distance)
                )
            if cost >= 25:
                stats["inflated"] += 1

        data = msg.data
        for my in range(height):
            cell_y = (my + 0.5) * resolution
            for mx in range(width):
                cost = int(data[my * width + mx])
                cell_x = (mx + 0.5) * resolution
                frame_x = float(origin.position.x) + origin_cos * cell_x - origin_sin * cell_y
                frame_y = float(origin.position.y) + origin_sin * cell_x + origin_cos * cell_y
                base_x = tf_tx + tf_cos * frame_x - tf_sin * frame_y
                base_y = tf_ty + tf_sin * frame_x + tf_cos * frame_y
                if base_x < -0.85 or base_x > 1.0 or base_y < -0.9 or base_y > 0.9:
                    continue
                distance = math.hypot(base_x, base_y)
                if cost >= 65:
                    nearest_obstacle = distance if nearest_obstacle is None else min(nearest_obstacle, distance)
                if 0.18 <= base_x <= 0.85 and abs(base_y) <= 0.28:
                    add("front", cost, distance)
                if -0.25 <= base_x <= 0.45 and 0.30 <= base_y <= 0.75:
                    add("left", cost, distance)
                if -0.25 <= base_x <= 0.45 and -0.75 <= base_y <= -0.30:
                    add("right", cost, distance)
                if distance <= 0.70 and not (-0.55 <= base_x <= 0.38 and abs(base_y) <= 0.36):
                    add("rotate", cost, distance)

        for stats in regions.values():
            cells = max(1, int(stats["cells"]))
            stats["blocked_ratio"] = stats["blocked"] / cells
            stats["inflated_ratio"] = stats["inflated"] / cells
            stats["unknown_ratio"] = stats["unknown"] / cells

        front = regions["front"]
        left = regions["left"]
        right = regions["right"]
        rotate = regions["rotate"]
        front_clear = front["cells"] > 0 and front["blocked_ratio"] <= 0.03 and front["inflated_ratio"] <= 0.35
        left_close = left["blocked_ratio"] >= 0.05 or left["inflated_ratio"] >= 0.45
        right_close = right["blocked_ratio"] >= 0.05 or right["inflated_ratio"] >= 0.45
        rotation_clear = (
            rotate["cells"] > 0
            and rotate["blocked"] <= 1
            and (rotate["nearest_obstacle_m"] is None or rotate["nearest_obstacle_m"] >= 0.62)
            and rotate["inflated_ratio"] <= 0.18
        )
        return {
            "available": True,
            "frame_id": frame_id,
            "age_sec": age,
            "nearest_obstacle_m": nearest_obstacle,
            "front_clear": front_clear,
            "left_close": left_close,
            "right_close": right_close,
            "sides_congested": left_close and right_close,
            "rotation_clear": rotation_clear,
            "regions": regions,
        }

    @staticmethod
    def _empty_region_stats() -> dict[str, Any]:
        return {
            "cells": 0,
            "blocked": 0,
            "inflated": 0,
            "unknown": 0,
            "nearest_obstacle_m": None,
        }

    def _current_map_pose(self) -> dict[str, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                "map",
                "base_footprint",
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException:
            return None
        translation = tf.transform.translation
        yaw = self._yaw_from_quaternion(tf.transform.rotation)
        return {"x": float(translation.x), "y": float(translation.y), "yaw": yaw}

    def _publish_narrow_motion_profile(
        self,
        annotation: dict[str, Any],
        waypoint: Pose2D,
        traversal_policy: dict[str, Any],
    ):
        # Sim narrow passages still need a cap, but 0.09 m/s made longer
        # waypoint legs look stalled and could trip mission-level timeouts.
        max_speed = max(
            0.03,
            min(float(traversal_policy.get("max_speed_mps", 0.12)), 0.14),
        )
        allow_lateral = bool(traversal_policy.get("allow_lateral_assist", True))
        profile = {
            "active": True,
            "profile": "narrow_passage",
            "passage_id": annotation.get("passage_id"),
            "frame_id": waypoint.frame_id,
            "heading_yaw_rad": waypoint.yaw,
            # Translation is allowed only within 10 degrees of passage heading.
            "heading_tolerance_rad": math.radians(10.0),
            "max_speed_mps": max_speed,
            "centerline": list(annotation.get("centerline") or []),
            "centerline_tolerance_m": float(
                traversal_policy.get("centerline_tolerance_m", 0.08)
            ),
            "allow_lateral_assist": allow_lateral,
            # Lateral correction must remain diagonal with forward motion.
            "max_lateral_mps": min(0.04, max_speed * 0.35) if allow_lateral else 0.0,
            "max_lateral_ratio": 0.45 if allow_lateral else 0.0,
            "timeout_sec": max(
                30.0,
                float(annotation.get("waypoint_timeout_sec", 20.0)) + 5.0,
            ),
        }
        self._motion_policy_profile_pub.publish(
            String(data=json.dumps(profile, ensure_ascii=False))
        )
        annotation["active_traversal_profile"] = profile

    def _clear_narrow_motion_profile(self):
        self._motion_policy_profile_pub.publish(
            String(data=json.dumps({"active": False, "profile": "normal"}))
        )

    def _target_heading_delta(self, current: dict[str, float], target: Pose2D) -> float | None:
        dx = float(target.x) - float(current["x"])
        dy = float(target.y) - float(current["y"])
        if math.hypot(dx, dy) < 1e-6:
            return None
        target_heading = math.atan2(dy, dx)
        return self._normalize_angle(target_heading - float(current["yaw"]))

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))
