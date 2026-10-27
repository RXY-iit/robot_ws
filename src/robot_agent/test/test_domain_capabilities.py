import time
from types import SimpleNamespace

from robot_agent.capabilities import capability_snapshot
from robot_agent.dry_run_skill_client import DryRunSkillClient
from robot_skills import result_codes as codes


def test_relative_capability_does_not_require_global_localization():
    client = SimpleNamespace(
        navigation=SimpleNamespace(_current_wheel_odom_pose=lambda: {"x": 0},
                                   _action_client=SimpleNamespace(server_is_ready=lambda: True)),
        system=SimpleNamespace(robot_mode="AUTO", localization_is_healthy=lambda: False))
    relative = capability_snapshot(client, "navigation", {"operation": "relative_motion"})
    global_nav = capability_snapshot(client, "navigation", {"operation": "global_navigation"})
    assert relative["ready"] is True
    assert global_nav["ready"] is False
    assert "localization_healthy" not in relative["evidence"]


def test_perception_observation_does_not_require_auto_or_a_detector():
    client = SimpleNamespace(perception=SimpleNamespace(_latest_image_time=time.monotonic(),
                                                        _latest_image=object(), color_topic="/camera"))
    snapshot = capability_snapshot(client, "perception", {})
    assert snapshot["ready"] is True
    assert snapshot["available"] is True


def test_grasp_placeholder_is_not_a_capability():
    result = capability_snapshot(SimpleNamespace(), "arm", {"operation": "grasp"})
    assert result["supported"] is result["available"] is result["ready"] is False


def test_alias_dispatch_and_cancelled_run_cannot_restart():
    client = DryRunSkillClient()
    context = client.context("test")
    result = client.execute_skill("set_arm_pose", {"pose_label": "home"}, context)
    assert result.skill_name == "set_arm_pose"
    assert result.metrics["implementation_skill"] == "set_pose"
    client.cancel_all("test")
    result = client.execute_skill("set_arm_pose", {"pose_label": "home"}, context)
    assert result.code == codes.CANCELLED
    assert client.execute_skill("get_robot_status", {}, client.context("next")).success
