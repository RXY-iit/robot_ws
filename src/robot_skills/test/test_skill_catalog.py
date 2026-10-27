from pathlib import Path

from robot_skills.registry import SkillRegistry
from robot_skills.skill_catalog import SkillCatalog


ROOT = Path(__file__).resolve().parents[3]


def test_skill_catalog_matches_registry():
    registry = SkillRegistry(str(ROOT / "src/robot_skills/config/skill_registry.yaml"))
    catalog = SkillCatalog(str(ROOT / "src/robot_skills/config/skill_catalog.v2.yaml"))

    assert catalog.validate_registry(registry) == []


def test_relative_motion_marks_localization_irrelevant():
    catalog = SkillCatalog(str(ROOT / "src/robot_skills/config/skill_catalog.v2.yaml"))
    health = {
        "components": {
            "mode": {"level": "OK", "summary": "AUTO"},
            "safety": {"level": "OK", "summary": "WATCHDOG"},
            "localization": {
                "level": "BAD",
                "summary": "GICP_SCORE_MISSING",
                "details": {"latest_wheel_odom_pose": [0.0, 0.0, 0.0]},
            },
        }
    }

    review = catalog.review_skill_preconditions("move_relative", health)

    assert review.allowed
    assert "localization" in review.ignored_bad_components
    assert "global_localization_healthy" in review.irrelevant
