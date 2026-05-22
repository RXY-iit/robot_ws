"""
operation_lib.py — High-level navigation operations for mission_manager.

Uses nav2_simple_commander.BasicNavigator as the Nav2 interface.
All blocking calls are designed to be called from a separate thread
(not the ROS spin thread).

Usage example:
    from mission_manager.operation_lib import OperationLib
    op = OperationLib(node, navigator)
    result = op.go_to("shelf_a")
    result = op.go_to_pose(x=1.0, y=2.0, yaw=0.0)
"""
import time
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node


def _make_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    sin_h = math.sin(yaw / 2.0)
    cos_h = math.cos(yaw / 2.0)
    pose.pose.orientation.z = sin_h
    pose.pose.orientation.w = cos_h
    return pose


class OperationLib:
    def __init__(self, node: Node, navigator: BasicNavigator, waypoints: dict):
        self._node = node
        self._nav = navigator
        self._waypoints = waypoints   # dict loaded from waypoint_map.yaml
        self._estop_pub = node.create_publisher(Bool, '/emergency_stop', 1)
        self._log = node.get_logger()

    # ------------------------------------------------------------------
    # Navigation primitives
    # ------------------------------------------------------------------

    def go_to(self, waypoint_id: str, timeout_sec: float = 120.0) -> TaskResult:
        """Navigate to a named waypoint from waypoint_map.yaml."""
        if waypoint_id not in self._waypoints:
            self._log.error(f'go_to: unknown waypoint "{waypoint_id}"')
            return TaskResult.FAILED
        wp = self._waypoints[waypoint_id]
        self._log.info(f'go_to: navigating to "{waypoint_id}" '
                       f'({wp["x"]:.2f}, {wp["y"]:.2f}, yaw={wp["yaw"]:.2f})')
        return self.go_to_pose(wp['x'], wp['y'], wp['yaw'], timeout_sec)

    def go_to_pose(self, x: float, y: float, yaw: float,
                   timeout_sec: float = 120.0) -> TaskResult:
        """Navigate to an arbitrary map-frame pose."""
        pose = _make_pose(self._nav, x, y, yaw)
        self._nav.goToPose(pose)
        deadline = time.time() + timeout_sec
        while not self._nav.isTaskComplete():
            if time.time() > deadline:
                self._log.warn('go_to_pose: timeout — cancelling navigation.')
                self.cancel_navigation()
                return TaskResult.FAILED
            time.sleep(0.1)
        return self._nav.getResult()

    def follow_waypoints(self, waypoint_ids: list,
                         timeout_sec: float = 300.0) -> TaskResult:
        """Follow a sequence of named waypoints."""
        poses = []
        for wid in waypoint_ids:
            if wid not in self._waypoints:
                self._log.error(f'follow_waypoints: unknown waypoint "{wid}"')
                return TaskResult.FAILED
            wp = self._waypoints[wid]
            poses.append(_make_pose(self._nav, wp['x'], wp['y'], wp['yaw']))
        self._log.info(f'follow_waypoints: {waypoint_ids}')
        self._nav.followWaypoints(poses)
        deadline = time.time() + timeout_sec
        while not self._nav.isTaskComplete():
            if time.time() > deadline:
                self._log.warn('follow_waypoints: timeout — cancelling.')
                self.cancel_navigation()
                return TaskResult.FAILED
            time.sleep(0.1)
        return self._nav.getResult()

    def cancel_navigation(self):
        """Cancel the current Nav2 goal."""
        self._nav.cancelTask()
        self._log.info('cancel_navigation: goal cancelled.')

    # ------------------------------------------------------------------
    # Recovery / utility operations
    # ------------------------------------------------------------------

    def wait(self, seconds: float):
        """Block for the given duration."""
        self._log.info(f'wait: {seconds}s')
        time.sleep(seconds)

    def clear_costmaps(self):
        """Clear both local and global costmaps."""
        self._nav.clearAllCostmaps()
        self._log.info('clear_costmaps: both costmaps cleared.')

    def clear_local_costmap(self):
        self._nav.clearLocalCostmap()

    def clear_global_costmap(self):
        self._nav.clearGlobalCostmap()

    def rotate_in_place(self, angle_rad: float = math.pi,
                        timeout_sec: float = 30.0) -> TaskResult:
        """Spin in place by angle_rad (positive = CCW). Uses Nav2 Spin behavior."""
        from geometry_msgs.msg import Pose
        self._log.info(f'rotate_in_place: {math.degrees(angle_rad):.0f}°')
        self._nav.spin(spin_dist=angle_rad)
        deadline = time.time() + timeout_sec
        while not self._nav.isTaskComplete():
            if time.time() > deadline:
                self._log.warn('rotate_in_place: timeout.')
                self.cancel_navigation()
                return TaskResult.FAILED
            time.sleep(0.1)
        return self._nav.getResult()

    def emergency_stop(self):
        """Assert hardware emergency stop via /emergency_stop topic."""
        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)
        self._log.error('emergency_stop: E-STOP asserted.')

    def release_emergency_stop(self):
        msg = Bool()
        msg.data = False
        self._estop_pub.publish(msg)
        self._log.info('release_emergency_stop: E-STOP cleared.')

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def get_nav_result(self) -> TaskResult:
        """Return the result of the last completed Nav2 task."""
        return self._nav.getResult()

    def is_nav_complete(self) -> bool:
        return self._nav.isTaskComplete()
