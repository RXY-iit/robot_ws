#!/usr/bin/env python3
"""
mission_manager_node.py — Phase 4 Mission Manager.

State machine:
  IDLE         → waiting for a task command
  NAVIGATING   → Nav2 goal active
  WAITING      → short pause before retry (blocked path)
  BLOCKED      → retries exhausted, waiting for human intervention
  FAILED       → unrecoverable error
  MANUAL       → B button / MANUAL mode active (Nav2 suspended)

Topics:
  Sub:  /robot_mode        (std_msgs/String)   — "AUTO" | "MANUAL"
  Sub:  /mission_command   (std_msgs/String)   — task commands (see below)
  Pub:  /mission_status    (std_msgs/String)   — current state + detail
  Pub:  /emergency_stop    (std_msgs/Bool)     — forwarded from FAILED state

Mission commands (publish to /mission_command):
  "go_to:<waypoint_id>"         — navigate to named waypoint
  "go_to_pose:<x>,<y>,<yaw>"   — navigate to explicit pose
  "cancel"                      — cancel current navigation
  "wait:<seconds>"              — wait in place
  "clear_costmaps"              — clear both costmaps
  "rotate"                      — spin 180°

Parameters:
  waypoint_map_path:  str   — path to waypoint_map.yaml
  max_retries:        int   — max navigation retries before BLOCKED (default 3)
  retry_wait_sec:     float — seconds to wait between retries (default 5.0)
"""
import threading
import time
import yaml
import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from mission_manager.operation_lib import OperationLib


class MissionState:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    WAITING = "WAITING"
    BLOCKED = "BLOCKED"
    FAILED = "FAILED"
    MANUAL = "MANUAL"


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('waypoint_map_path', '')
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_wait_sec', 5.0)

        self._max_retries = self.get_parameter('max_retries').value
        self._retry_wait = self.get_parameter('retry_wait_sec').value

        self._state = MissionState.IDLE
        self._detail = ""
        self._retry_count = 0
        self._lock = threading.Lock()
        self._task_thread: threading.Thread = None
        self._robot_mode = "MANUAL"

        # Load waypoints
        wp_path = self.get_parameter('waypoint_map_path').value
        if not wp_path:
            wp_path = os.path.join(
                get_package_share_directory('mission_manager'),
                'config', 'waypoint_map.yaml')
        self._waypoints = self._load_waypoints(wp_path)

        # Nav2
        self._navigator = BasicNavigator()

        # Operation lib
        self._op = OperationLib(self, self._navigator, self._waypoints)

        # Subscriptions
        self.create_subscription(String, '/robot_mode', self._mode_cb, 10)
        self.create_subscription(String, '/mission_command', self._command_cb, 10)

        # Publishers
        self._status_pub = self.create_publisher(String, '/mission_status', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'MissionManagerNode ready. Waypoints: {list(self._waypoints.keys())}')

    # ------------------------------------------------------------------
    # Waypoint loading
    # ------------------------------------------------------------------

    def _load_waypoints(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            wps = data.get('waypoints', {})
            self.get_logger().info(f'Loaded {len(wps)} waypoints from {path}')
            return wps
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints from {path}: {e}')
            return {}

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _mode_cb(self, msg: String):
        prev = self._robot_mode
        self._robot_mode = msg.data
        if msg.data == "MANUAL" and prev != "MANUAL":
            self.get_logger().info('robot_mode → MANUAL: suspending mission.')
            with self._lock:
                self._state = MissionState.MANUAL
                self._detail = "B button pressed"
            self._op.cancel_navigation()

    def _command_cb(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f'mission_command received: "{cmd}"')

        if cmd == "cancel":
            self._op.cancel_navigation()
            with self._lock:
                self._state = MissionState.IDLE
                self._detail = "cancelled by command"
            return

        if cmd == "clear_costmaps":
            self._op.clear_costmaps()
            return

        if cmd.startswith("wait:"):
            try:
                secs = float(cmd.split(":")[1])
            except ValueError:
                self.get_logger().error(f'Invalid wait command: "{cmd}"')
                return
            self._start_task(lambda: self._op.wait(secs))
            return

        if cmd == "rotate":
            self._start_task(lambda: self._run_nav_task(
                lambda: self._op.rotate_in_place(math.pi)))
            return

        if cmd.startswith("go_to:"):
            wp_id = cmd[len("go_to:"):]
            self._start_task(lambda: self._run_nav_task(
                lambda: self._op.go_to(wp_id)))
            return

        if cmd.startswith("go_to_pose:"):
            try:
                parts = cmd[len("go_to_pose:"):].split(",")
                x, y, yaw = float(parts[0]), float(parts[1]), float(parts[2])
            except (ValueError, IndexError):
                self.get_logger().error(f'Invalid go_to_pose command: "{cmd}"')
                return
            self._start_task(lambda: self._run_nav_task(
                lambda: self._op.go_to_pose(x, y, yaw)))
            return

        self.get_logger().warn(f'Unknown mission_command: "{cmd}"')

    # ------------------------------------------------------------------
    # Task execution
    # ------------------------------------------------------------------

    def _start_task(self, fn):
        if self._robot_mode == "MANUAL":
            self.get_logger().warn('Cannot start task: robot is in MANUAL mode. Press A for AUTO.')
            return
        if self._task_thread and self._task_thread.is_alive():
            self.get_logger().warn('A task is already running. Send "cancel" first.')
            return
        self._retry_count = 0
        self._task_thread = threading.Thread(target=fn, daemon=True)
        self._task_thread.start()

    def _run_nav_task(self, nav_fn):
        """Execute a navigation function with retry logic."""
        while self._retry_count <= self._max_retries:
            if self._robot_mode == "MANUAL":
                with self._lock:
                    self._state = MissionState.MANUAL
                return

            with self._lock:
                self._state = MissionState.NAVIGATING
                self._detail = f"attempt {self._retry_count + 1}/{self._max_retries + 1}"

            result = nav_fn()

            if result == TaskResult.SUCCEEDED:
                with self._lock:
                    self._state = MissionState.IDLE
                    self._detail = "goal succeeded"
                self.get_logger().info('Mission: SUCCEEDED.')
                return

            if result == TaskResult.CANCELED:
                with self._lock:
                    self._state = MissionState.IDLE
                    self._detail = "goal cancelled"
                return

            # FAILED or timeout
            self._retry_count += 1
            if self._retry_count > self._max_retries:
                break

            self.get_logger().warn(
                f'Navigation failed (attempt {self._retry_count}/{self._max_retries}). '
                f'Waiting {self._retry_wait}s before retry.')
            with self._lock:
                self._state = MissionState.WAITING
                self._detail = f"retry {self._retry_count} in {self._retry_wait}s"
            self._op.clear_costmaps()
            time.sleep(self._retry_wait)

        # Retries exhausted
        with self._lock:
            self._state = MissionState.BLOCKED
            self._detail = "retries exhausted — waiting for human"
        self.get_logger().error(
            'Mission: BLOCKED. Retries exhausted. Manual intervention required.')

    # ------------------------------------------------------------------
    # Status publisher
    # ------------------------------------------------------------------

    def _publish_status(self):
        with self._lock:
            state = self._state
            detail = self._detail
        msg = String()
        msg.data = f"{state}: {detail}" if detail else state
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
