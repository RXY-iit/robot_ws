from __future__ import annotations

from robot_agent.agents.arm_agent import ArmAgent
from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_agent.schemas import MissionIntent
from types import SimpleNamespace


def test_arm_pose_command_does_not_steal_navigation_home():
    parser = DeterministicParser()

    nav = parser.parse("go to home")
    arm = parser.parse("set pose home")

    assert nav.task_type == "navigate_to_place"
    assert nav.target == "home"
    assert arm.task_type == "set_pose"
    assert arm.target == "home"


def test_arm_manipulation_commands_parse_to_task_types():
    parser = DeterministicParser()

    cases = [
        ("wave hello", "set_pose", "wave", None),
        ("touch the red cube", "touch_object", "red cube", "red cube"),
        ("press the button", "press_object", "button", "button"),
        ("pick up the red cube", "grasp_object", "red cube", "red cube"),
        ("go to the panel and press the green button", "press_object_near_place", "panel", "green button"),
    ]

    for text, task_type, target, object_query in cases:
        intent = parser.parse(text)
        assert intent.task_type == task_type
        assert intent.target == target
        assert intent.object_query == object_query


def test_arm_agent_builds_press_plan_with_detection_bridge():
    plan = ArmAgent().build_plan(MissionIntent("press_object", target="green button", object_query="green button"))

    assert [call.name for call in plan] == [
        "set_pose",
        "capture_observation",
        "detect_known_object",
        "touch",
    ]
    assert plan[0].payload["name"] == "see_front"
    assert plan[2].payload["query"] == "green button"
    assert plan[3].payload == {"target": "green button", "mode": "press"}


def test_sequence_command_parses_ordered_steps_without_stealing_near_place_arm_command():
    parser = DeterministicParser()

    sequence = parser.parse("go back 1 m ,then set pose home")
    move_then_look = parser.parse("go back a little , then see left")
    near_place = parser.parse("go to the panel and press the green button")

    assert sequence.task_type == "sequence"
    steps = sequence.constraints["steps"]
    assert [step["task_type"] for step in steps] == ["move_relative", "set_pose"]
    assert steps[0]["constraints"] == {"direction": "back", "distance_m": 1.0}
    assert steps[1]["target"] == "home"
    assert move_then_look.task_type == "sequence"
    move_then_look_steps = move_then_look.constraints["steps"]
    assert [step["task_type"] for step in move_then_look_steps] == ["move_relative", "look_direction"]
    assert move_then_look_steps[0]["constraints"] == {"direction": "back", "distance_m": 0.2}
    assert move_then_look_steps[1]["constraints"] == {"direction": "left", "band": "middle"}
    assert near_place.task_type == "press_object_near_place"


def test_japanese_look_left_is_camera_look_not_base_rotation():
    intent = DeterministicParser().parse("左を見て")

    assert intent.task_type == "look_direction"
    assert intent.constraints["direction"] == "left"


def test_arm_refinement_rejects_unknown_pose_before_skill_execution():
    arm = SimpleNamespace(validate_pose=lambda name, mode: "Unknown arm pose")
    result = ArmAgent().refine(MissionIntent("set_arm_pose", target="unknown_pose"),
                              SimpleNamespace(arm=arm, mode="sim"))
    assert result.question == "Unknown arm pose"
    assert not result.plan
