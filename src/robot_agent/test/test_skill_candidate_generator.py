from pathlib import Path

from robot_agent.candidates.skill_candidate_generator import SkillCandidateGenerator
from robot_agent.jev_agent import JevStrengthenAgent
from robot_agent.parsers.deterministic_parser import DeterministicParser
from robot_agent.schemas import MissionIntent, SkillCall
from robot_skills import result_codes as codes
from robot_skills.skill_catalog import SkillCatalog


ROOT = Path(__file__).resolve().parents[3]


def test_candidate_generator_repairs_typo_and_keeps_sequence_order():
    catalog = SkillCatalog(str(ROOT / "src/robot_skills/config/skill_catalog.v2.yaml"))
    generator = SkillCandidateGenerator(DeterministicParser(), catalog)

    candidates = generator.generate("go back a littlt and turn left")
    sequence = next(candidate for candidate in candidates if candidate.candidate_id == "move_relative_then_rotate_relative")

    assert sequence.task_type == "sequence"
    assert [step["task_type"] for step in sequence.steps] == ["move_relative", "rotate_relative"]
    assert sequence.steps[0]["constraints"]["direction"] == "back"
    assert sequence.steps[1]["constraints"]["yaw_deg"] == 90
    assert any("littlt" in note for note in sequence.notes)


def test_candidate_generator_handles_after_that_sequence_connector():
    catalog = SkillCatalog(str(ROOT / "src/robot_skills/config/skill_catalog.v2.yaml"))
    generator = SkillCandidateGenerator(DeterministicParser(), catalog)

    candidates = generator.generate("go back a little ,after that see right")
    sequence = next(candidate for candidate in candidates if candidate.candidate_id == "move_relative_then_look_direction")

    assert sequence.task_type == "sequence"
    assert [step["task_type"] for step in sequence.steps] == ["move_relative", "look_direction"]
    assert sequence.steps[0]["constraints"]["direction"] == "back"
    assert sequence.steps[1]["constraints"]["direction"] == "right"


def test_jev_local_parse_prefers_complete_sequence_candidate(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    monkeypatch.delenv("JEV_API_URL", raising=False)
    decision = JevStrengthenAgent().parse_command("go back a littlt and turn left")

    assert decision.intent.task_type == "sequence"
    steps = decision.intent.constraints["steps"]
    assert [step["task_type"] for step in steps] == ["move_relative", "rotate_relative"]


def test_jev_command_parse_uses_task_graph_questions_not_best_candidate(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    monkeypatch.delenv("JEV_API_URL", raising=False)
    decision = JevStrengthenAgent().parse_command("go back a little and turn left")
    question_ids = {item["id"] for item in decision.trace.questions}

    assert "best_candidate_plan" not in question_ids
    assert "command_shape" in question_ids
    assert "sequence_step_count" in question_ids
    assert "subgoal_dependency_policy" in question_ids


def test_jev_supervision_target_not_found_before_relative_motion_asks_human_not_recovery(monkeypatch):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    monkeypatch.delenv("JEV_API_URL", raising=False)
    agent = JevStrengthenAgent()
    plan = [
        SkillCall(name="detect_known_object", payload={"query": "red object"}),
        SkillCall(name="await_robot_auto_mode", payload={"timeout_sec": 0.1}),
        SkillCall(name="move_relative", payload={"direction": "back", "distance_m": 0.2}),
    ]
    intent = MissionIntent(
        task_type="sequence",
        constraints={
            "dependency_policy": "ordered_but_independent",
            "steps": [
                {"task_type": "search_object_here", "object_query": "red object", "constraints": {}},
                {"task_type": "move_relative", "constraints": {"direction": "back", "distance_m": 0.2}},
            ],
        },
    )

    decision = agent.supervise_step(
        user_text="check red object and go back a little",
        intent=intent,
        plan=plan,
        step_index=0,
        result={
            "skill_name": "detect_known_object",
            "success": False,
            "code": codes.NEEDS_HUMAN,
            "message": "No confident match for red object.",
        },
        health={"components": {"safety": {"level": "OK"}}},
    )

    assert decision.action == "ask_human_continue_retry_stop"
    question_results = {item["id"]: item.get("result") for item in decision.trace.questions}
    assert question_results["failure_category"] == "target_not_found"
    assert question_results["needs_recovery_agent"] is False
    assert any("move back" in choice["label"].lower() for choice in decision.human_choices)
