import time

import pytest

from robot_skills.executors.arm import ArmSkills
from robot_skills.schemas import SkillContext


def arm_with_presets():
    arm = ArmSkills(None)
    arm._state_received_at = time.monotonic()
    arm._state = {"presets": [{"id": "home", "allowed_modes": ["MOCK"]}]}
    return arm


def test_unknown_pose_cannot_publish_or_reuse_previous_candidate():
    arm = arm_with_presets()
    arm._publish_arm = lambda *a, **k: pytest.fail("Invalid pose must not publish")
    result = arm.set_pose(SkillContext("test"), "unknown_pose")
    assert not result.success
    assert "Unknown arm pose" in result.message


def test_pose_requires_fresh_and_mode_approved_catalog():
    arm = arm_with_presets()
    assert arm.validate_pose("home_pose", "sim") is None
    assert "not approved" in arm.validate_pose("home", "real")
    assert "Unknown" in arm.validate_pose("wave", "sim")
    arm._state_received_at -= 4
    assert "Fresh" in arm.validate_pose("home", "sim")


@pytest.mark.parametrize("state", ["BLOCKED", "FAILED", "CANCELLED"])
def test_rejected_ack_wins_over_old_valid_candidate(state):
    arm = arm_with_presets()
    arm._state.update(candidate={"valid": True, "id": "old"},
                      execution={"state": state, "last_request_id": "new", "message": "rejected"})
    with pytest.raises(RuntimeError, match="rejected"):
        arm._wait_for_candidate(SkillContext("test"), "unknown_pose", "new")


def test_matching_planned_ack_accepts_candidate():
    arm = arm_with_presets()
    arm._state.update(candidate={"valid": True, "id": "new-candidate"},
                      execution={"state": "PLANNED", "last_request_id": "new"})
    assert arm._wait_for_candidate(SkillContext("test"), "home", "new")["id"] == "new-candidate"
