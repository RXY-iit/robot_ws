from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_arm_control.mock_trajectory_server import trajectory_errors


JOINTS = [
    {"name": "a", "lower": -1.0, "upper": 1.0},
    {"name": "b", "lower": 0.0, "upper": 2.0},
]


def target(names=("a", "b"), positions=(0.5, 1.0), sec=2):
    trajectory = JointTrajectory()
    trajectory.joint_names = list(names)
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start = Duration(sec=sec)
    trajectory.points = [point]
    return trajectory


def test_accepts_complete_ordered_bounded_trajectory():
    assert trajectory_errors(JOINTS, target()) == []


def test_rejects_wrong_order_and_limits():
    wrong_order = trajectory_errors(JOINTS, target(names=("b", "a")))
    outside_limit = trajectory_errors(JOINTS, target(positions=(3.0, 0.0)))
    assert any("exactly match" in error for error in wrong_order)
    assert any("outside" in error for error in outside_limit)


def test_rejects_zero_duration():
    assert any("time_from_start" in error for error in trajectory_errors(JOINTS, target(sec=0)))
