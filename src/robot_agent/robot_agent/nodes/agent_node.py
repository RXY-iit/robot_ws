from __future__ import annotations

import json
import math
import threading
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from rosgraph_msgs.msg import Clock
from rclpy.clock import Clock as RosClock, ClockType

from robot_agent.agents.mission_agent import MissionAgent
from robot_agent.mission_log import MissionLogger
from robot_agent.schemas import HealthState
from robot_agent.skill_client import SkillClient
from robot_agent.stack_lifecycle import ShutdownParticipant


class AgentNode(Node):
    def __init__(self):
        super().__init__("agent_node")
        self.declare_parameter("mode", "real")
        self.mode = str(self.get_parameter("mode").value)
        self.skill_client = SkillClient(self, mode=self.mode)
        self.logger_store = MissionLogger()
        self.mission_agent = MissionAgent(
            self.skill_client,
            event_cb=self._publish_event,
            state_cb=self._publish_state,
        )
        self.declare_parameter("agent_mode", "jev_primary")
        self.mission_agent.set_agent_mode(str(self.get_parameter("agent_mode").value))
        self.mission_agent.enable_human_resume = True
        self.robot_mode_request_pub = self.create_publisher(String, "/nav_mode/command", 10)
        self.mission_agent.request_auto_mode = lambda: self.robot_mode_request_pub.publish(String(data="AUTO"))
        self._active_lock = threading.Lock()
        self._active_thread: threading.Thread | None = None
        self._last_mission_id: str | None = None
        self._current_mission_id: str | None = None
        self._last_mission_state = None
        self._current_active_skill_name: str | None = None
        self._last_health_cancel_mission_id: str | None = None
        self._last_local_planner_state_query = 0.0
        self._local_planner_state_query_pending = False
        self._local_planner_state_query_pending_since = 0.0
        self._local_planner_state_query_id = 0
        self._local_planner_state_pending_id: int | None = None
        self._shutdown_started = 0.0
        self._shutdown_nav_result = None
        self._shutdown_stop_samples = 0
        self._shutdown_odom_received = 0.0
        self._shutdown_arm_received = 0.0
        self._shutdown_arm_state = {}

        self.status_pub = self.create_publisher(String, "/agent/status", 10)
        self.event_pub = self.create_publisher(String, "/agent/mission_events", 10)
        self.state_pub = self.create_publisher(String, "/agent/mission_state", 10)
        self.health_pub = self.create_publisher(String, "/agent/health_state", 10)
        self.human_prompt_pub = self.create_publisher(String, "/agent/human_prompt", 10)
        self.mode_state_pub = self.create_publisher(String, "/agent/mode_state", 10)

        self.create_subscription(String, "/agent/text_command", self._command_cb, 10)
        self.create_subscription(String, "/agent/cancel", self._cancel_cb, 10)
        self.create_subscription(String, "/agent/mode_command", self._mode_command_cb, 10)
        self.create_subscription(String, "/agent/mission_control", self._mission_control_cb, 10)
        self.create_subscription(String, "/arm/dashboard_state", self._shutdown_arm_cb, 10)
        self.create_subscription(String, "/robot_mode", self._robot_mode_cb, 10)
        self.create_subscription(String, "/safety_status", self._safety_status_cb, 10)
        self.create_subscription(String, "/gicp_loc/accepted_state", self._accepted_localization_cb, 10)
        self.create_subscription(String, "/gicp_loc/recovery_status", self._localization_recovery_cb, 10)
        self._last_sim_clock = None
        self.create_subscription(Clock, "/clock", self._clock_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav2_cmd_vel_cb, 10)
        self.create_subscription(Odometry, "/wheel_odom", self._wheel_odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10)
        self.local_planner_state_client = self.create_client(GetState, "/controller_server/get_state")
        self._shutdown = ShutdownParticipant(self, "agent_node", self._prepare_shutdown, self._shutdown_ready)

        self.create_timer(2.0, self._heartbeat, clock=RosClock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"AgentNode ready in mode={self.mode}. Publish commands to /agent/text_command."
        )
        self._publish_agent_mode_state()

    def _robot_mode_cb(self, msg: String):
        self.skill_client.update_robot_mode(msg.data)

    def _safety_status_cb(self, msg: String):
        self.skill_client.update_safety_status(msg.data)

    def _gicp_score_cb(self, msg: Float32):
        self.skill_client.update_gicp_score(float(msg.data))

    def _clock_cb(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec / 1e9
        if self._last_sim_clock is not None and stamp < self._last_sim_clock:
            self.skill_client.system.invalidate_localization_anchor()
        self._last_sim_clock = stamp

    def _localization_recovery_cb(self, msg):
        try:
            state = json.loads(msg.data)
            self.skill_client.system._recovery_accumulating = state.get("phase") == "accumulating"
            self.skill_client.system._recovery_status_received = time.monotonic()
        except (ValueError, TypeError):
            return

    def _accepted_localization_cb(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict):
                return
            stamp = float(state["stamp_sec"])
            pose = state["map_base_pose"]
            anchor = state["odom_base_pose"]
            score = float(state["rmse_m"])
            if (not isinstance(pose, list) or not isinstance(anchor, list)
                    or len(pose) != 3 or len(anchor) != 3
                    or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose + anchor)
                    or not math.isfinite(score) or score < 0):
                return
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if not 0 <= age <= 2 or state["base_frame"] != "base_footprint" or state["map_frame"] != "map" or state["odom_frame"] != "odom":
                return
            system = self.skill_client.system
            if system._evidence_stamp is not None and stamp <= system._evidence_stamp:
                return
            system.update_gicp_score(score)
            system.update_odom_pose(*pose, stamp=stamp, odom_anchor=tuple(anchor))
        except (ValueError, TypeError, KeyError):
            return

    def _cmd_vel_cb(self, msg: Twist):
        self.skill_client.update_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def _nav2_cmd_vel_cb(self, msg: Twist):
        self.skill_client.update_nav2_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def _wheel_odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        p = msg.pose.pose.position
        if msg.header.frame_id.lstrip("/") != "odom":
            return
        self.skill_client.system.update_wheel_odom_pose(p.x, p.y, yaw, stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
        if self._shutdown_started:
            twist = msg.twist.twist
            stopped = all(math.isfinite(v) and abs(v) < .02 for v in
                          (twist.linear.x, twist.linear.y, twist.angular.z))
            self._shutdown_stop_samples = self._shutdown_stop_samples + 1 if stopped else 0
            self._shutdown_odom_received = time.monotonic()

    def _shutdown_arm_cb(self, msg):
        try:
            self._shutdown_arm_state = json.loads(msg.data)
            self._shutdown_arm_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _prepare_shutdown(self):
        with self._active_lock:
            self._shutdown_started = time.monotonic()
            self.skill_client.stopping = True
            self.skill_client.navigation.stopping = True
            skill = self._current_active_skill_name or ""
            self._shutdown_need_base = skill in {"goto_pose", "move_relative", "move_relative_xy", "rotate_relative"}
            self._shutdown_need_arm = skill.startswith(("arm_", "touch_", "grasp_", "place_"))
            goal = self.skill_client.navigation._active_goal_handle
            if goal is not None:
                self._shutdown_nav_result = goal.get_result_async()
                self._shutdown_need_base = True
            self._request_cancel_locked("Stop Agent stack")

    def _shutdown_ready(self):
        if self._active_thread and self._active_thread.is_alive():
            return False, "Waiting for Mission worker to exit after cancellation"
        navigation = self.skill_client.navigation
        if navigation._pending_nav_goals or any(not future.done() for future in navigation._nav_result_futures):
            return False, "Waiting for pending Nav2 goal/result callbacks"
        if self._shutdown_nav_result is not None:
            if not self._shutdown_nav_result.done() or self._shutdown_nav_result.result().status not in {4, 5, 6}:
                return False, "Waiting for terminal Nav2 goal acknowledgement"
        if self._shutdown_need_base and (self._shutdown_stop_samples < 3 or time.monotonic() - self._shutdown_odom_received > 2):
            return False, "Waiting for fresh stationary wheel odometry after cancellation"
        if self._shutdown_need_arm:
            execution = self._shutdown_arm_state.get("execution") or {}
            if (self._shutdown_arm_received <= self._shutdown_started or time.monotonic() - self._shutdown_arm_received > 2
                    or execution.get("state") not in {"IDLE", "SUCCEEDED", "CANCELLED", "FAILED", "ABORTED"}
                    or execution.get("visual_servo_session_id")):
                return False, "Waiting for fresh Arm terminal feedback"
        return True, "Mission drained; Agent-owned motion cancellation completed"

    def _localization_pose_cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        p = msg.pose.position
        self.skill_client.update_odom_pose(p.x, p.y, yaw)

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        self.skill_client.update_initialpose()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._publish_event(
            {
                "type": "manual_initialpose_received",
                "payload": {
                    "frame_id": msg.header.frame_id,
                    "x": p.x,
                    "y": p.y,
                    "yaw_rad": yaw,
                    "yaw_deg": yaw * 180.0 / 3.141592653589793,
                    "settle_sec": self.skill_client.system._initialpose_settle_sec,
                },
            }
        )

    def _command_cb(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        with self._active_lock:
            if self._is_cancel_text(text):
                self._request_cancel_locked(text)
                return
            if self._shutdown.stopping:
                self._publish_status("STOPPING: new missions are disabled")
                return
            if self._active_thread and self._active_thread.is_alive():
                self._publish_status("BUSY: mission already running")
                return
            self._active_thread = threading.Thread(target=self._run_mission, args=(text,), daemon=True)
            self._active_thread.start()

    def _cancel_cb(self, msg: String):
        with self._active_lock:
            self._request_cancel_locked(msg.data)

    def _mission_control_cb(self, msg: String):
        if self._shutdown.stopping:
            return
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(command, dict) or not self.mission_agent.human_action(command):
            self.get_logger().warn("Ignored stale or unsupported mission control request")

    def _mode_command_cb(self, msg: String):
        if self._shutdown.stopping:
            return
        selected = self.mission_agent.set_agent_mode(msg.data)
        self._publish_agent_mode_state()
        self._publish_event(
            {
                "type": "agent_mode_changed",
                "payload": {"agent_mode": selected, "requested": msg.data},
            }
        )
        self._publish_status(f"agent_mode={selected}")

    def _request_cancel_locked(self, text: str = "cancel"):
        run_id = self._current_mission_id or self._last_mission_id or "manual_cancel"
        result = self.mission_agent.cancel(run_id)
        self._publish_event(
            {
                "type": "cancel_requested",
                "payload": result.to_dict(),
                "mission_id": run_id,
                "command": text,
            }
        )
        self.human_prompt_pub.publish(String(data=""))
        self._publish_status("CANCEL requested")
        state = self._last_mission_state
        if state is not None and not (self._active_thread and self._active_thread.is_alive()) and state.status not in {
            "DONE", "DONE_WITH_STUB", "FAILED", "FAILED_SKILL", "CANCELLED", "REJECTED"
        }:
            # A NEEDS_HUMAN mission may have no running worker left to publish
            # its terminal state. Close it explicitly after cancelling skills.
            self.mission_agent._finish_cancelled(deepcopy(state))

    def _run_mission(self, text: str):
        state = self.mission_agent.run(text, mode=self.mode)
        self._last_mission_id = state.mission_id
        self._publish_status(f"{state.status}: {state.final_message or ''}")
        if state.question_for_human:
            self.human_prompt_pub.publish(String(data=state.question_for_human))

    def _publish_event(self, event: dict):
        from robot_agent.schemas import utc_now
        event = dict(event)
        event.setdefault("time", utc_now())
        event.setdefault("ros_time_sec", self.get_clock().now().nanoseconds / 1e9)
        data = json.dumps(event, ensure_ascii=False)
        self.logger_store.write_event(event)
        self.event_pub.publish(String(data=data))

    def _publish_state(self, state):
        self._last_mission_state = deepcopy(state)
        self._current_mission_id = state.mission_id
        self._current_active_skill_name = self._active_skill_name_from_state(state)
        state_msg = json.dumps(state.to_dict(), ensure_ascii=False)
        self.state_pub.publish(String(data=state_msg))
        if not state.question_for_human:
            self.human_prompt_pub.publish(String(data=""))

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _publish_agent_mode_state(self):
        jev_agent = self.mission_agent.jev_agent
        jev_status = jev_agent.availability_summary()
        self.mode_state_pub.publish(
            String(
                data=json.dumps(
                    {
                        "agent_mode": self.mission_agent.agent_mode,
                        "available_modes": ["classic", "jev_primary", "jev_shadow"],
                        "jev_available": jev_status.get("available", False),
                        "jev_provider": jev_status.get("provider", "local"),
                        "jev_status": jev_status,
                    },
                    ensure_ascii=False,
                )
            )
        )

    @staticmethod
    def _is_cancel_text(text: str) -> bool:
        return text.strip().lower().rstrip(".!。！") in {
            "cancel", "cancel all", "cancel task", "cancel mission", "stop", "stop all", "stop task", "abort",
            "停止", "中止", "キャンセル", "全体キャンセル", "全部取消"}

    def _heartbeat(self):
        self._query_local_planner_state()
        self._publish_agent_mode_state()
        self.skill_client.set_local_planner_required(
            self._current_active_skill_name == "goto_pose",
            f"active_skill={self._current_active_skill_name or 'idle'}",
        )
        health = HealthState.from_summary(self.skill_client.system.health_summary(self.skill_client.context("heartbeat", 1.0)))
        health_data = health.to_dict()
        self.health_pub.publish(String(data=json.dumps(health_data, ensure_ascii=False)))
        self._cancel_motion_if_health_bad(health)
        localization_details = (
            health.components.get("localization").details
            if health.components.get("localization")
            else {}
        )
        summary = {
            "type": "agent_heartbeat",
            "mode": self.mode,
            "robot_mode": self.skill_client.system.robot_mode,
            "safety_status": self.skill_client.system.safety_status,
            "localization_warning": localization_details.get(
                "localization_warning", self.skill_client.system._localization_warning
            ),
            "stationary_drift_m": localization_details.get(
                "stationary_drift_m", self.skill_client.system._stationary_drift_m
            ),
            "trusted_pose_delta_m": localization_details.get("trusted_pose_delta_m"),
            "trusted_pose_delta_yaw_rad": localization_details.get("trusted_pose_delta_yaw_rad"),
            "gicp_score": self.skill_client.system._gicp_score,
            "health_overall": health.overall,
            "recommended_action": health.recommended_action,
            "local_planner": health.components.get("local_planner").to_dict()
            if health.components.get("local_planner")
            else None,
        }
        self.event_pub.publish(String(data=json.dumps(summary, ensure_ascii=False)))

    def _query_local_planner_state(self):
        now_time = time.time()
        if self._local_planner_state_query_pending:
            pending_age = now_time - self._local_planner_state_query_pending_since
            if pending_age <= 3.0:
                return
            # A lifecycle request can remain unresolved forever when Nav2 is
            # restarted while Agent stays alive. Do not let that old future
            # permanently block queries to the replacement controller_server.
            self.get_logger().warn(
                'controller_server lifecycle query timed out; retrying against current service')
            self._local_planner_state_query_pending = False
            self._local_planner_state_pending_id = None
        if now_time - self._last_local_planner_state_query < 1.0:
            return
        self._last_local_planner_state_query = now_time
        if not self.local_planner_state_client.service_is_ready():
            return
        self._local_planner_state_query_pending = True
        self._local_planner_state_query_pending_since = now_time
        self._local_planner_state_query_id += 1
        query_id = self._local_planner_state_query_id
        self._local_planner_state_pending_id = query_id
        future = self.local_planner_state_client.call_async(GetState.Request())
        future.add_done_callback(
            lambda completed, request_id=query_id: self._local_planner_state_cb(
                completed,
                request_id,
            )
        )

    def _local_planner_state_cb(self, future, query_id: int | None = None):
        if (
            query_id is not None
            and query_id != self._local_planner_state_pending_id
        ):
            # Ignore a late response from the controller instance that was
            # replaced during a Nav2 restart.
            return
        self._local_planner_state_query_pending = False
        self._local_planner_state_query_pending_since = 0.0
        self._local_planner_state_pending_id = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to query controller_server lifecycle state: {exc!r}")
            return
        current_state = response.current_state
        self.skill_client.update_local_planner_state(current_state.label, current_state.id)

    @staticmethod
    def _active_skill_name_from_state(state) -> str | None:
        if state.status not in {"EXECUTE_STEP", "RECOVERING", "PAUSED_HEALTH"}:
            return None
        plan = state.plan or []
        index = getattr(state, "current_step_index", 0)
        if not isinstance(index, int) or index < 0 or index >= len(plan):
            return None
        return plan[index].name

    def _cancel_motion_if_health_bad(self, health: HealthState):
        if health.overall != "BAD":
            return
        with self._active_lock:
            active = bool(self._active_thread and self._active_thread.is_alive())
            mission_id = self._current_mission_id or self._last_mission_id or "health_watchdog"
            active_skill = self._current_active_skill_name
            if not active or self._last_health_cancel_mission_id == mission_id:
                return
            if not self._health_bad_blocks_active_skill(health, active_skill):
                return
            self._last_health_cancel_mission_id = mission_id
        result = self.mission_agent.cancel_motion_only(mission_id)
        self._publish_event(
            {
                "type": "health_cancel_requested",
                "mission_id": mission_id,
                "payload": {
                    "reason": "health_state_BAD",
                    "active_skill": active_skill,
                    "health": health.to_dict(),
                    "cancel_result": result.to_dict(),
                },
            }
        )

    @staticmethod
    def _health_bad_blocks_active_skill(health: HealthState, active_skill: str | None) -> bool:
        components = health.components or {}
        safety_bad = components.get("safety") is not None and components["safety"].level == "BAD"
        localization_bad = components.get("localization") is not None and components["localization"].level == "BAD"
        local_planner_bad = components.get("local_planner") is not None and components["local_planner"].level == "BAD"
        if safety_bad:
            return True
        if active_skill == "goto_pose":
            return localization_bad or local_planner_bad
        return False

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        import math

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
