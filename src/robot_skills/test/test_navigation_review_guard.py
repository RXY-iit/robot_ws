from types import SimpleNamespace

import pytest

from robot_skills.executors.navigation import NavigationSkills
from robot_skills.schemas import Pose2D, SkillContext
from robot_skills import result_codes as codes


@pytest.mark.parametrize("scenario,code", [("execute", None), ("cancel", codes.CANCELLED),
    ("lost_localization", codes.FAILED_LOCALIZATION), ("pose_changed", codes.FAILED_PRECHECK),
    ("exception", codes.NEEDS_HUMAN), ("ask_human", codes.NEEDS_HUMAN), ("malformed", codes.NEEDS_HUMAN)])
def test_review_guard_rechecks_evidence_after_api(scenario, code):
    nav = NavigationSkills.__new__(NavigationSkills)
    state = {"cancelled": False, "healthy": True, "x": 0.0}
    nav._navigation_localization_ok = lambda: state["healthy"]
    nav._current_map_pose = lambda: {"x": state["x"], "y": 0.0}
    def review(evidence):
        assert evidence["path"] == [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}]
        if scenario == "exception":
            raise RuntimeError("API failed")
        state.update(cancelled=scenario == "cancel", healthy=scenario != "lost_localization",
                     x=1.0 if scenario == "pose_changed" else 0.0)
        return None if scenario == "malformed" else {"decision": "ask_human" if scenario == "ask_human" else "execute"}
    context = SkillContext("mission", navigation_review=review)
    context.cancel_check = lambda: state["cancelled"]
    result = nav._review_before_motion(context, Pose2D(1, 0, 0), {"x": 0, "y": 0},
                [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}], None, "default", "start")
    assert (result.code if result else None) == code


def test_goto_reviews_even_internal_unmarked_segments_before_dispatch():
    nav = NavigationSkills.__new__(NavigationSkills)
    nav._action_client = SimpleNamespace(wait_for_server=lambda **kw: True)
    nav._navigation_localization_ok = lambda: True
    nav._current_map_pose = lambda: {"x": 0, "y": 0, "yaw": 0}
    nav._compute_global_path = lambda *_: [{"x": 0, "y": 0}, {"x": 1, "y": 0}]
    calls = []
    context = SkillContext("mission", robot_mode="AUTO",
        navigation_review=lambda e: calls.append(e) or {"decision": "ask_human"})
    result = nav.goto_pose(context, Pose2D(1, 0, 0), apply_navigation_annotations=False, publish_goal_markers=False)
    assert result.code == codes.NEEDS_HUMAN and len(calls) == 1
