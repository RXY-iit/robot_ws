from __future__ import annotations

import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from std_msgs.msg import Float32, String

from robot_agent.agents.mission_agent import MissionAgent
from robot_agent.mission_log import MissionLogger
from robot_agent.schemas import HealthState
from robot_agent.skill_client import SkillClient


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
        self._active_lock = threading.Lock()
        self._active_thread: threading.Thread | None = None
        self._last_mission_id: str | None = None
        self._current_mission_id: str | None = None
        self._current_active_skill_name: str | None = None
        self._last_health_cancel_mission_id: str | None = None
        self._last_local_planner_state_query = 0.0
        self._local_planner_state_query_pending = False
        self._local_planner_state_query_pending_since = 0.0
        self._local_planner_state_query_id = 0
        self._local_planner_state_pending_id: int | None = None

        self.status_pub = self.create_publisher(String, "/agent/status", 10)
        self.event_pub = self.create_publisher(String, "/agent/mission_events", 10)
        self.state_pub = self.create_publisher(String, "/agent/mission_state", 10)
        self.health_pub = self.create_publisher(String, "/agent/health_state", 10)
        self.human_prompt_pub = self.create_publisher(String, "/agent/human_prompt", 10)

        self.create_subscription(String, "/agent/text_command", self._command_cb, 10)
        self.create_subscription(String, "/agent/cancel", self._cancel_cb, 10)
        self.create_subscription(String, "/robot_mode", self._robot_mode_cb, 10)
        self.create_subscription(String, "/safety_status", self._safety_status_cb, 10)
        self.create_subscription(Float32, "/gicp_loc/score", self._gicp_score_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(Twist, "/nav2/cmd_vel", self._nav2_cmd_vel_cb, 10)
        self.create_subscription(PoseStamped, "/gicp_loc/pose", self._localization_pose_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10)
        self.local_planner_state_client = self.create_client(GetState, "/controller_server/get_state")

        self.create_timer(2.0, self._heartbeat)
        self.get_logger().info(
            f"AgentNode ready in mode={self.mode}. Publish commands to /agent/text_command."
        )

    def _robot_mode_cb(self, msg: String):
        self.skill_client.update_robot_mode(msg.data)

    def _safety_status_cb(self, msg: String):
        self.skill_client.update_safety_status(msg.data)

    def _gicp_score_cb(self, msg: Float32):
        self.skill_client.update_gicp_score(float(msg.data))

    def _cmd_vel_cb(self, msg: Twist):
        self.skill_client.update_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    def _nav2_cmd_vel_cb(self, msg: Twist):
        self.skill_client.update_nav2_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

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
            if self._active_thread and self._active_thread.is_alive():
                if self._is_cancel_text(text):
                    self._request_cancel_locked(text)
                    return
                self._publish_status("BUSY: mission already running")
                return
            self._active_thread = threading.Thread(target=self._run_mission, args=(text,), daemon=True)
            self._active_thread.start()

    def _cancel_cb(self, msg: String):
        with self._active_lock:
            self._request_cancel_locked(msg.data)

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
        self._publish_status("CANCEL requested")

    def _run_mission(self, text: str):
        state = self.mission_agent.run(text, mode=self.mode)
        self._last_mission_id = state.mission_id
        self._publish_status(f"{state.status}: {state.final_message or ''}")
        if state.question_for_human:
            self.human_prompt_pub.publish(String(data=state.question_for_human))

    def _publish_event(self, event: dict):
        data = json.dumps(event, ensure_ascii=False)
        self.logger_store.write_event(event)
        self.event_pub.publish(String(data=data))

    def _publish_state(self, state):
        self._current_mission_id = state.mission_id
        self._current_active_skill_name = self._active_skill_name_from_state(state)
        state_msg = json.dumps(state.to_dict(), ensure_ascii=False)
        self.state_pub.publish(String(data=state_msg))

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    @staticmethod
    def _is_cancel_text(text: str) -> bool:
        lowered = text.strip().lower()
        return any(token in lowered for token in ["cancel", "stop", "abort", "停止", "中止", "キャンセル"])

    def _heartbeat(self):
        self._query_local_planner_state()
        self.skill_client.set_local_planner_required(
            self._current_active_skill_name == "goto_pose",
            f"active_skill={self._current_active_skill_name or 'idle'}",
        )
        health = HealthState.from_summary(self.skill_client.system.health_summary(self.skill_client.context("heartbeat", 1.0)))
        health_data = health.to_dict()
        self.health_pub.publish(String(data=json.dumps(health_data, ensure_ascii=False)))
        self._cancel_motion_if_health_bad(health)
        summary = {
            "type": "agent_heartbeat",
            "mode": self.mode,
            "robot_mode": self.skill_client.system.robot_mode,
            "safety_status": self.skill_client.system.safety_status,
            "localization_warning": self.skill_client.system._localization_warning,
            "stationary_drift_m": self.skill_client.system._stationary_drift_m,
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
            if not active or self._last_health_cancel_mission_id == mission_id:
                return
            self._last_health_cancel_mission_id = mission_id
        result = self.mission_agent.cancel_motion_only(mission_id)
        self._publish_event(
            {
                "type": "health_cancel_requested",
                "mission_id": mission_id,
                "payload": {
                    "reason": "health_state_BAD",
                    "health": health.to_dict(),
                    "cancel_result": result.to_dict(),
                },
            }
        )

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
