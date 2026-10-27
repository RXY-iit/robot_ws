#!/usr/bin/env python3
"""
nav_mode_switch_node.py — A/B mode switch for Nav2.

Subscribes to /joy and optional /nav_mode/command.
Pressing B forces MANUAL mode. Pressing A forces AUTO mode by default.

  MANUAL (default): /teleop/cmd_vel is relayed to /cmd_vel_raw.
  AUTO:             /nav2/cmd_vel (from Nav2 controller_server) is relayed to /cmd_vel_raw.
                    cmd_vel_safety_node clamps /cmd_vel_raw and publishes /cmd_vel.
                    Pressing B cancels the active Nav2 goal, publishes zero vel,
                    and returns to MANUAL.

Topic map:
  Sub:  /joy              (sensor_msgs/Joy)
  Sub:  /nav_mode/command (std_msgs/String: AUTO/MANUAL)
  Sub:  /teleop/cmd_vel   (geometry_msgs/Twist) — from teleop_twist_joy
  Sub:  /nav2/cmd_vel     (geometry_msgs/Twist) — from controller_server remap
  Pub:  /cmd_vel_raw      (geometry_msgs/Twist) — to cmd_vel_safety_node
  Pub:  /robot_mode       (std_msgs/String)     — "AUTO" or "MANUAL"

Joy button mapping (Linux jsX driver):
  index 0 = A  ← AUTO
  index 1 = B  ← MANUAL + cancel Nav2 goal
  index 4 = L1 (dead-man for teleop_twist_joy)
"""
import json
import math
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


class NavModeSwitchNode(Node):
    MODE_AUTO = 'AUTO'
    MODE_MANUAL = 'MANUAL'

    def __init__(self):
        super().__init__('nav_mode_switch_node')

        self.mode = self.MODE_MANUAL
        self.prev_buttons = []
        self.last_teleop_time = None

        self.declare_parameter('auto_button', 0)
        self.declare_parameter('manual_button', 1)
        self.declare_parameter('manual_deadman_button', 4)
        self.declare_parameter('mode_command_topic', '/nav_mode/command')
        self.declare_parameter('enable_motion_policy', True)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('align_angle_rad', 0.75)
        self.declare_parameter('align_distance_m', 0.35)
        self.declare_parameter('finish_distance_m', 0.18)
        self.declare_parameter('finish_yaw_gate_rad', 0.45)
        self.declare_parameter('finish_max_vx', 0.08)
        self.declare_parameter('finish_max_wz', 0.25)
        self.declare_parameter('reverse_distance_m', 0.45)
        self.declare_parameter('reverse_lateral_window_m', 0.35)
        self.declare_parameter('near_back_distance_m', 1.5)
        self.declare_parameter('near_back_angle_rad', 1.0472)
        self.declare_parameter('near_back_lateral_scale', 0.15)
        self.declare_parameter('policy_lookahead_m', 0.8)
        self.declare_parameter('lateral_scale', 0.25)
        self.declare_parameter('lateral_to_yaw_gain', 1.0)
        self.declare_parameter('min_align_wz', 0.25)
        self.declare_parameter('max_align_wz', 0.8)
        self.declare_parameter('max_reverse_vx', 0.12)
        self.declare_parameter('reverse_yaw_gain', 0.8)
        self.declare_parameter('manual_teleop_timeout_sec', 0.25)
        self.declare_parameter('local_costmap_node', '/local_costmap/local_costmap')
        self.declare_parameter('normal_local_inflation_radius', 0.45)
        self.declare_parameter('narrow_local_inflation_radius', 0.33)
        self.declare_parameter('narrow_speed_hard_limit_mps', 0.09)
        self.declare_parameter('narrow_centerline_gain', 0.8)
        self.declare_parameter('localization_gate_enabled', True)
        self.declare_parameter('localization_score_topic', '/gicp_loc/score')
        self.declare_parameter('localization_pose_topic', '/gicp_loc/pose')
        self.declare_parameter('localization_score_warn_threshold', 0.25)
        self.declare_parameter('localization_data_stale_sec', 3.0)
        self.declare_parameter('localization_min_good_samples', 3)
        self.declare_parameter('initialpose_settle_sec', 10.0)
        self.auto_button = int(self.get_parameter('auto_button').value)
        self.manual_button = int(self.get_parameter('manual_button').value)
        self.manual_deadman_button = int(
            self.get_parameter('manual_deadman_button').value)
        self.mode_command_topic = str(self.get_parameter('mode_command_topic').value)
        self.enable_motion_policy = self._param_bool(
            self.get_parameter('enable_motion_policy').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.plan_topic = str(self.get_parameter('plan_topic').value)
        self.align_angle = float(self.get_parameter('align_angle_rad').value)
        self.align_distance = float(self.get_parameter('align_distance_m').value)
        self.finish_distance = float(self.get_parameter('finish_distance_m').value)
        self.finish_yaw_gate = float(self.get_parameter('finish_yaw_gate_rad').value)
        self.finish_max_vx = float(self.get_parameter('finish_max_vx').value)
        self.finish_max_wz = float(self.get_parameter('finish_max_wz').value)
        self.reverse_distance = float(self.get_parameter('reverse_distance_m').value)
        self.reverse_lateral_window = float(self.get_parameter('reverse_lateral_window_m').value)
        self.near_back_distance = float(self.get_parameter('near_back_distance_m').value)
        self.near_back_angle = float(self.get_parameter('near_back_angle_rad').value)
        self.near_back_lateral_scale = float(self.get_parameter('near_back_lateral_scale').value)
        self.policy_lookahead = float(self.get_parameter('policy_lookahead_m').value)
        self.lateral_scale = float(self.get_parameter('lateral_scale').value)
        self.lateral_to_yaw_gain = float(self.get_parameter('lateral_to_yaw_gain').value)
        self.min_align_wz = float(self.get_parameter('min_align_wz').value)
        self.max_align_wz = float(self.get_parameter('max_align_wz').value)
        self.max_reverse_vx = float(self.get_parameter('max_reverse_vx').value)
        self.reverse_yaw_gain = float(self.get_parameter('reverse_yaw_gain').value)
        self.manual_teleop_timeout_sec = float(
            self.get_parameter('manual_teleop_timeout_sec').value)
        self.local_costmap_node = str(self.get_parameter('local_costmap_node').value)
        self.normal_local_inflation_radius = float(
            self.get_parameter('normal_local_inflation_radius').value)
        self.narrow_local_inflation_radius = float(
            self.get_parameter('narrow_local_inflation_radius').value)
        self.narrow_speed_hard_limit = float(
            self.get_parameter('narrow_speed_hard_limit_mps').value)
        self.narrow_centerline_gain = float(
            self.get_parameter('narrow_centerline_gain').value)
        self.localization_gate_enabled = self._param_bool(
            self.get_parameter('localization_gate_enabled').value)
        self.localization_score_topic = str(
            self.get_parameter('localization_score_topic').value)
        self.localization_pose_topic = str(
            self.get_parameter('localization_pose_topic').value)
        self.localization_score_warn_threshold = float(
            self.get_parameter('localization_score_warn_threshold').value)
        self.localization_data_stale_sec = float(
            self.get_parameter('localization_data_stale_sec').value)
        self.localization_min_good_samples = int(
            self.get_parameter('localization_min_good_samples').value)
        self.initialpose_settle_sec = float(
            self.get_parameter('initialpose_settle_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_path = None
        self.narrow_profile = None
        self.narrow_profile_time = 0.0
        self.narrow_heading_aligning = False
        self.last_centerline_log_time = 0.0
        self.latest_gicp_score = None
        self.last_gicp_score_time = 0.0
        self.last_localization_pose_time = 0.0
        self.localization_good_samples = 0
        self.last_initialpose_time = 0.0
        self.last_localization_gate_log_time = 0.0

        # Inflation is the only costmap value changed by the passage profile.
        # The configured polygon footprint remains untouched.
        self.costmap_param_client = self.create_client(
            SetParameters,
            f'{self.local_costmap_node.rstrip("/")}/set_parameters',
        )
        self.desired_local_inflation_radius = self.normal_local_inflation_radius
        self.applied_local_inflation_radius = None
        self.inflation_update_pending = False

        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_subscription(String, self.mode_command_topic, self._mode_command_cb, 10)
        self.create_subscription(Twist, '/teleop/cmd_vel', self._teleop_cmd_vel_cb, 10)
        self.create_subscription(Twist, '/nav2/cmd_vel', self._nav2_cmd_vel_cb, 10)
        self.create_subscription(Path, self.plan_topic, self._plan_cb, 10)
        self.create_subscription(Float32, self.localization_score_topic, self._gicp_score_cb, 10)
        self.create_subscription(PoseStamped, self.localization_pose_topic, self._gicp_pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._initialpose_cb, 10)
        self.create_subscription(
            String,
            '/motion_policy/profile',
            self._motion_profile_cb,
            10,
        )

        # Publish to /cmd_vel_raw; cmd_vel_safety_node applies watchdog/clamp → /cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.policy_pub = self.create_publisher(String, '/motion_policy/status', 10)

        self._cancel_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(2.0, self._publish_mode)
        self.create_timer(0.02, self._manual_watchdog_tick)
        self.create_timer(0.5, self._sync_local_inflation_radius)
        self.get_logger().info(
            f'NavModeSwitchNode started. Mode: {self.mode}. '
            f'auto_button={self.auto_button}, manual_button={self.manual_button}, '
            f'manual_deadman_button={self.manual_deadman_button}, '
            f'mode_command_topic={self.mode_command_topic}, '
            f'enable_motion_policy={self.enable_motion_policy}.')

    def _joy_cb(self, msg: Joy):
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        if self.auto_button >= 0 and len(msg.buttons) > self.auto_button:
            if msg.buttons[self.auto_button] == 1 and self.prev_buttons[self.auto_button] == 0:
                self._set_auto()

        if self.manual_button >= 0 and len(msg.buttons) > self.manual_button:
            if msg.buttons[self.manual_button] == 1 and self.prev_buttons[self.manual_button] == 0:
                self._set_manual()

        # Valid Joy updates on the real robot occasionally have 0.10--0.19 s
        # gaps.  Stop from the actual L1/LB falling edge instead of using an
        # unrealistically short topic watchdog as the normal dead-man path.
        if (
            self.mode == self.MODE_MANUAL
            and self.manual_deadman_button >= 0
            and len(msg.buttons) > self.manual_deadman_button
            and self.prev_buttons[self.manual_deadman_button] == 1
            and msg.buttons[self.manual_deadman_button] == 0
        ):
            self._stop_robot()
            self.last_teleop_time = None

        self.prev_buttons = list(msg.buttons)

    def _mode_command_cb(self, msg: String):
        command = msg.data.upper().strip()
        if command == self.MODE_AUTO:
            self._set_auto()
        elif command == self.MODE_MANUAL:
            self._set_manual()
        else:
            self.get_logger().warn(
                f'Ignoring unknown {self.mode_command_topic} command: {msg.data!r}')

    def _set_auto(self):
        if self.mode != self.MODE_AUTO:
            self.mode = self.MODE_AUTO
            self.get_logger().info('Button A: MANUAL → AUTO (Nav2 cmd_vel relay active)')
        else:
            self.get_logger().info('Button A: already AUTO')
        if self.localization_gate_enabled and not self._localization_gate_ok():
            self.get_logger().warn(
                'AUTO requested while localization is not healthy; Nav2 cmd_vel will stay blocked '
                f'({self._localization_gate_reason()}).')
        self._publish_mode()

    def _set_manual(self):
        if self.mode != self.MODE_MANUAL:
            self.get_logger().info('Button B: AUTO → MANUAL (Nav2 goals cancelled)')
        else:
            self.get_logger().info('Button B: already MANUAL (Nav2 goals cancelled)')
        self.mode = self.MODE_MANUAL
        # Do not retain a passage controller across an operator cancellation.
        # The Agent also clears the profile in its finally block, but clearing
        # locally here makes the B-button stop independent of that round trip.
        self._clear_narrow_profile('MANUAL')
        self._stop_robot()
        self._cancel_nav2_goals()
        self._publish_mode()

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def _cancel_nav2_goals(self):
        if self._cancel_client.service_is_ready():
            req = CancelGoal.Request()
            self._cancel_client.call_async(req)
        else:
            self.get_logger().warn('Nav2 cancel service not ready — goals may still be active')

    def _nav2_cmd_vel_cb(self, msg: Twist):
        if self.mode == self.MODE_AUTO:
            if self.localization_gate_enabled and not self._localization_gate_ok():
                self._stop_robot()
                self._publish_policy_status(
                    f'LOCALIZATION_GATE_BLOCKED:{self._localization_gate_reason()}')
                now_mono = time.monotonic()
                if now_mono - self.last_localization_gate_log_time >= 1.0:
                    self.last_localization_gate_log_time = now_mono
                    self.get_logger().warn(
                        'Blocking Nav2 cmd_vel until localization recovers: '
                        f'{self._localization_gate_reason()}')
                return
            try:
                out, status = self._apply_motion_policy(msg)
            except Exception as exc:
                self.get_logger().error(
                    f'Motion policy failed; passing Nav2 cmd_vel through: {exc!r}')
                out, status = msg, 'PASS_THROUGH_POLICY_ERROR'
            self.cmd_vel_pub.publish(out)
            self._publish_policy_status(status)

    def _teleop_cmd_vel_cb(self, msg: Twist):
        self.last_teleop_time = time.monotonic()
        if self.mode == self.MODE_MANUAL:
            self.cmd_vel_pub.publish(msg)

    def _manual_watchdog_tick(self):
        if self.mode != self.MODE_MANUAL:
            return
        if (
            self.last_teleop_time is None
            or time.monotonic() - self.last_teleop_time > self.manual_teleop_timeout_sec
        ):
            self.cmd_vel_pub.publish(Twist())

    def _publish_mode(self):
        out = String()
        out.data = self.mode
        self.mode_pub.publish(out)

    def _plan_cb(self, msg: Path):
        if msg.poses:
            self.latest_path = msg

    def _gicp_score_cb(self, msg: Float32):
        self.latest_gicp_score = float(msg.data)
        self.last_gicp_score_time = time.monotonic()
        if self.latest_gicp_score < self.localization_score_warn_threshold:
            self.localization_good_samples += 1
        else:
            self.localization_good_samples = 0

    def _gicp_pose_cb(self, _msg: PoseStamped):
        self.last_localization_pose_time = time.monotonic()

    def _initialpose_cb(self, _msg):
        self.last_initialpose_time = time.monotonic()
        self.localization_good_samples = 0
        self.latest_gicp_score = None
        self.last_gicp_score_time = 0.0
        self.last_localization_pose_time = 0.0

    def _localization_gate_ok(self):
        return self._localization_gate_reason() == 'OK'

    def _localization_gate_reason(self):
        now_mono = time.monotonic()
        if (
            self.last_initialpose_time > 0.0
            and now_mono - self.last_initialpose_time < self.initialpose_settle_sec
        ):
            return 'INITIALPOSE_SETTLING'
        if self.latest_gicp_score is None:
            return 'GICP_SCORE_MISSING'
        if now_mono - self.last_gicp_score_time > self.localization_data_stale_sec:
            return 'GICP_SCORE_STALE'
        if now_mono - self.last_localization_pose_time > self.localization_data_stale_sec:
            return 'GICP_POSE_STALE'
        if self.latest_gicp_score >= self.localization_score_warn_threshold:
            return f'GICP_SCORE_WARN:{self.latest_gicp_score:.3f}'
        if self.localization_good_samples < self.localization_min_good_samples:
            return (
                f'GICP_GOOD_SAMPLES:{self.localization_good_samples}/'
                f'{self.localization_min_good_samples}')
        return 'OK'

    def _motion_profile_cb(self, msg: String):
        try:
            profile = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f'Ignoring invalid motion policy profile: {exc}')
            return
        if not profile.get('active') or profile.get('profile') != 'narrow_passage':
            self._clear_narrow_profile('profile inactive')
            return
        self.narrow_profile = profile
        self.narrow_profile_time = time.monotonic()
        self.narrow_heading_aligning = False
        self.desired_local_inflation_radius = self.narrow_local_inflation_radius
        self._sync_local_inflation_radius()
        self.get_logger().info(
            'Narrow-passage profile active: '
            f'passage={profile.get("passage_id")} '
            f'heading={profile.get("heading_yaw_rad")} '
            f'tolerance={profile.get("heading_tolerance_rad")} '
            f'max_speed={profile.get("max_speed_mps")}'
        )

    def _apply_motion_policy(self, msg: Twist):
        if not self.enable_motion_policy:
            return msg, 'PASS_THROUGH_DISABLED'

        if self.narrow_profile is not None:
            timeout_sec = float(self.narrow_profile.get('timeout_sec', 30.0))
            if time.monotonic() - self.narrow_profile_time <= timeout_sec:
                return self._apply_narrow_passage_policy(msg)
            self.get_logger().warn('Narrow-passage profile expired; returning to normal policy')
            self._clear_narrow_profile('profile timeout')

        path_target = self._path_target_in_base_frame()
        if path_target is None:
            return self._reduce_lateral_motion(msg), 'PASS_THROUGH_NO_GOAL_TF'

        target_x, target_y, target_distance, goal_x, goal_y, goal_distance, goal_yaw = path_target
        target_angle = math.atan2(target_y, target_x)
        goal_angle = math.atan2(goal_y, goal_x)

        # Bounded close-range policy only: keep long-range obstacle avoidance and
        # path selection inside Nav2/MPPI, but avoid wasteful turn-around when the
        # remaining goal is clearly behind the robot and nearby.
        if self._is_near_back_goal(goal_angle, goal_distance):
            out = Twist()
            out.linear.x = -min(self.max_reverse_vx, max(0.03, goal_distance * 0.35))
            out.linear.y = self._clamp(
                goal_y * self.near_back_lateral_scale,
                -self.reverse_lateral_window,
                self.reverse_lateral_window,
            )
            out.angular.z = self._clamp(-self.reverse_yaw_gain * goal_y,
                                        -self.max_align_wz, self.max_align_wz)
            return out, 'NEAR_BACK_REVERSE'

        if (
            goal_distance <= self.reverse_distance
            and goal_x < -0.05
            and abs(goal_y) <= self.reverse_lateral_window
        ):
            out = Twist()
            out.linear.x = -min(self.max_reverse_vx, max(0.03, goal_distance * 0.5))
            out.angular.z = self._clamp(-self.reverse_yaw_gain * goal_y,
                                        -self.max_align_wz, self.max_align_wz)
            return out, 'SHORT_REVERSE'

        if goal_distance <= self.finish_distance:
            out = self._reduce_lateral_motion(msg)
            out.linear.x = self._clamp(out.linear.x, -self.finish_max_vx, self.finish_max_vx)
            out.linear.y = 0.0
            if abs(goal_yaw) > self.finish_yaw_gate:
                out.linear.x = 0.0
                out.angular.z = self._clamp(0.5 * goal_yaw,
                                            -self.finish_max_wz, self.finish_max_wz)
                return out, 'FINAL_YAW_ALIGN'
            out.angular.z = self._clamp(out.angular.z, -self.finish_max_wz, self.finish_max_wz)
            return out, 'FINAL_APPROACH'

        if target_distance >= self.align_distance and abs(target_angle) >= self.align_angle:
            out = Twist()
            out.angular.z = math.copysign(
                min(self.max_align_wz, max(self.min_align_wz, abs(target_angle) * 0.8)),
                target_angle,
            )
            return out, 'ALIGN_THEN_FORWARD'

        return self._reduce_lateral_motion(msg), 'FORWARD_TRACK'

    def _apply_narrow_passage_policy(self, msg: Twist):
        profile = self.narrow_profile or {}
        heading_error = self._heading_error(
            float(profile.get('heading_yaw_rad', 0.0)),
            str(profile.get('frame_id') or 'map'),
        )
        if heading_error is None:
            return Twist(), 'NARROW_STOP_NO_HEADING_TF'

        tolerance = max(
            math.radians(2.0),
            min(
                float(profile.get('heading_tolerance_rad', math.radians(10.0))),
                math.radians(10.0),
            ),
        )
        # Hysteresis prevents repeated turn/translate switching around 10 deg.
        # Once alignment starts, finish it to 5 deg before translating again.
        resume_tolerance = min(tolerance, math.radians(5.0))
        if abs(heading_error) > tolerance:
            self.narrow_heading_aligning = True
        if self.narrow_heading_aligning and abs(heading_error) > resume_tolerance:
            out = Twist()
            out.angular.z = math.copysign(
                min(0.25, max(0.07, abs(heading_error) * 0.7)),
                heading_error,
            )
            return out, 'NARROW_HEADING_ALIGN'
        self.narrow_heading_aligning = False

        max_speed = max(
            0.03,
            min(
                float(profile.get('max_speed_mps', self.narrow_speed_hard_limit)),
                self.narrow_speed_hard_limit,
            ),
        )
        out = Twist()
        # Forward only. A zero/negative MPPI vx remains stopped; this policy
        # never forces motion through an obstacle or asks for reverse traversal.
        out.linear.x = self._clamp(msg.linear.x, 0.0, max_speed)
        allow_lateral = bool(profile.get('allow_lateral_assist', True))
        if allow_lateral and out.linear.x > 0.01:
            max_lateral = max(0.0, min(float(profile.get('max_lateral_mps', 0.035)), 0.04))
            max_ratio = max(0.0, min(float(profile.get('max_lateral_ratio', 0.45)), 0.5))
            lateral_limit = min(max_lateral, out.linear.x * max_ratio)
            requested_lateral = msg.linear.y
            centerline_error = self._centerline_error(profile)
            if centerline_error is not None:
                # Positive error means the robot is left of the directed
                # centerline, therefore a negative body-y correction is needed.
                correction = -self.narrow_centerline_gain * centerline_error
                tolerance = max(
                    0.02,
                    float(profile.get('centerline_tolerance_m', 0.08)),
                )
                if abs(centerline_error) > tolerance:
                    # Outside the center band the centerline owns lateral sign;
                    # MPPI may only add a correction toward, never away from it.
                    requested_lateral = correction
                else:
                    requested_lateral += correction
                now = time.monotonic()
                if now - self.last_centerline_log_time >= 1.0:
                    self.last_centerline_log_time = now
                    self.get_logger().info(
                        'Narrow centerline error: '
                        f'{centerline_error:+.3f} m '
                        f'(tolerance=+/-{tolerance:.3f} m)')
            out.linear.y = self._clamp(
                requested_lateral, -lateral_limit, lateral_limit)
        else:
            # Pure sideways correction is forbidden in the narrow passage.
            out.linear.y = 0.0
        # During constrained traversal the passage heading, not MPPI's free
        # angular command, owns yaw. A small deadband avoids endless micro-spin.
        out.angular.z = 0.0
        if abs(heading_error) > math.radians(2.0):
            out.angular.z = self._clamp(heading_error * 1.0, -0.18, 0.18)
        return out, 'NARROW_FORWARD_DIAGONAL'

    def _centerline_error(self, profile):
        centerline = profile.get('centerline') or []
        if len(centerline) < 2:
            return None
        try:
            start = centerline[0]
            end = centerline[-1]
            x0, y0 = float(start['x']), float(start['y'])
            x1, y1 = float(end['x']), float(end['y'])
            tf = self.tf_buffer.lookup_transform(
                str(profile.get('frame_id') or 'map'),
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except (KeyError, TypeError, ValueError, TransformException):
            return None
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return None
        robot_x = float(tf.transform.translation.x)
        robot_y = float(tf.transform.translation.y)
        return (dx * (robot_y - y0) - dy * (robot_x - x0)) / length

    def _clear_narrow_profile(self, reason):
        was_active = self.narrow_profile is not None
        self.narrow_profile = None
        self.narrow_profile_time = 0.0
        self.narrow_heading_aligning = False
        self.desired_local_inflation_radius = self.normal_local_inflation_radius
        self._sync_local_inflation_radius()
        if was_active:
            self.get_logger().info(
                f'Narrow-passage profile cleared ({reason}); restoring local '
                f'inflation to {self.normal_local_inflation_radius:.2f} m')

    def _sync_local_inflation_radius(self):
        desired = self.desired_local_inflation_radius
        if self.inflation_update_pending:
            return
        if (
            self.applied_local_inflation_radius is not None
            and abs(self.applied_local_inflation_radius - desired) < 1e-6
        ):
            return
        if not self.costmap_param_client.service_is_ready():
            return
        self.inflation_update_pending = True
        request = SetParameters.Request()
        request.parameters = [Parameter(
                'inflation_layer.inflation_radius',
                Parameter.Type.DOUBLE,
                desired,
            ).to_parameter_msg()]
        future = self.costmap_param_client.call_async(request)
        future.add_done_callback(
            lambda completed, requested=desired: self._inflation_update_done(
                completed, requested))

    def _inflation_update_done(self, future, requested):
        self.inflation_update_pending = False
        try:
            response = future.result()
            results = list(response.results)
            if not results or not all(result.successful for result in results):
                reasons = ', '.join(
                    result.reason for result in results if not result.successful)
                self.get_logger().warn(
                    f'Failed to set local inflation to {requested:.2f} m: {reasons}')
                return
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to set local inflation to {requested:.2f} m: {exc!r}')
            return
        self.applied_local_inflation_radius = requested
        self.get_logger().info(
            f'Local inflation radius applied: {requested:.2f} m')

    def _heading_error(self, target_yaw, frame_id):
        try:
            tf = self.tf_buffer.lookup_transform(
                frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None
        current_yaw = self._yaw_from_quaternion(tf.transform.rotation)
        return self._normalize_angle(target_yaw - current_yaw)

    def _is_near_back_goal(self, goal_angle, goal_distance):
        if goal_distance > self.near_back_distance:
            return False
        back_error = abs(self._normalize_angle(goal_angle - math.pi))
        return back_error <= self.near_back_angle

    def _path_target_in_base_frame(self):
        if self.latest_path is None or not self.latest_path.poses:
            return None

        path_frame = (
            self.latest_path.header.frame_id
            or self.latest_path.poses[-1].header.frame_id
            or 'map'
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                path_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return None

        base = tf.transform.translation
        yaw = self._yaw_from_quaternion(tf.transform.rotation)

        nearest_i = min(
            range(len(self.latest_path.poses)),
            key=lambda i: self._distance_xy(self.latest_path.poses[i].pose.position, base),
        )
        target_i = nearest_i
        walked = 0.0
        for i in range(nearest_i + 1, len(self.latest_path.poses)):
            prev = self.latest_path.poses[i - 1].pose.position
            curr = self.latest_path.poses[i].pose.position
            walked += math.hypot(curr.x - prev.x, curr.y - prev.y)
            target_i = i
            if walked >= self.policy_lookahead:
                break

        target = self.latest_path.poses[target_i].pose.position
        goal_pose = self.latest_path.poses[-1].pose
        goal = goal_pose.position

        target_x, target_y = self._map_delta_to_base(target.x - base.x, target.y - base.y, yaw)
        goal_x, goal_y = self._map_delta_to_base(goal.x - base.x, goal.y - base.y, yaw)
        goal_yaw = self._normalize_angle(self._yaw_from_quaternion(goal_pose.orientation) - yaw)
        return (
            target_x,
            target_y,
            math.hypot(target_x, target_y),
            goal_x,
            goal_y,
            math.hypot(goal_x, goal_y),
            goal_yaw,
        )

    @staticmethod
    def _map_delta_to_base(dx, dy, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    @staticmethod
    def _distance_xy(point, base):
        return math.hypot(point.x - base.x, point.y - base.y)

    def _reduce_lateral_motion(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y * self.lateral_scale
        out.angular.z = msg.angular.z + msg.linear.y * self.lateral_to_yaw_gain
        return out

    def _publish_policy_status(self, status: str):
        msg = String()
        msg.data = status
        self.policy_pub.publish(msg)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _param_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ('1', 'true', 'yes', 'on')
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = NavModeSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
