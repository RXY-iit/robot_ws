import json

import pytest

from robot_agent.command_grounding import structure_questions, binding_questions, tokens
from robot_agent.jev_agent import JevStrengthenAgent, JEV_SCHEMA_VERSION


@pytest.fixture
def agent(monkeypatch, tmp_path):
    monkeypatch.delenv("JEV_API_KEY", raising=False)
    instance = JevStrengthenAgent()
    instance.call_log_path = str(tmp_path / "jev.jsonl")
    instance.api_configured = True
    return instance


def test_probe_uses_production_question_builders(agent):
    parse, bind, review = agent._representative_probe_bodies()
    assert parse["questions"] == structure_questions(agent.task_catalog)
    assert bind["questions"] == binding_questions(["move_relative"], tokens("go back 20cm"))
    assert review["questions"] == agent._plan_review_questions()
    assert all(body["state"]["schema_version"] == JEV_SCHEMA_VERSION for body in (parse, bind, review))


@pytest.mark.parametrize("invalid", ["empty", "missing", "illegal", "error", None])
def test_probe_requires_every_answer_and_logs_separately(agent, monkeypatch, invalid):
    def post(body):
        answers = {k: {"choice": next(iter(q["criteria"]))} for k, q in body["questions"].items()}
        if invalid == "empty":
            answers = {}
        elif invalid == "missing":
            answers.pop(next(iter(answers)))
        elif invalid == "illegal":
            answers[next(iter(answers))] = {"choice": "not_in_options"}
        return {"_error": "timeout"} if invalid == "error" else {"answers": answers}
    monkeypatch.setattr(agent, "_post_decision", post)
    agent._probe_api()
    assert agent.api_available == (invalid is None)
    with open(agent.call_log_path) as stream:
        records = [json.loads(line) for line in stream]
    assert len(records) == 3
    assert all(r["purpose"] == "probe" and len(r["questions_sha256"]) == 64 for r in records)
    assert all(r["schema_version"] == JEV_SCHEMA_VERSION for r in records)


def test_question_hash_is_order_independent_and_changes_with_content(agent):
    body = {"state": {"schema_version": JEV_SCHEMA_VERSION},
            "questions": {"a": {"type": "choice", "criteria": {"yes": "Yes", "no": "No"}}}}
    agent._write_call_log("test", body, {}, 1)
    body["questions"]["a"]["criteria"] = {"no": "No", "yes": "Yes"}
    agent._write_call_log("test", body, {}, 1)
    body["questions"]["a"]["instructions"] = "Changed question"
    agent._write_call_log("test", body, {}, 1)
    with open(agent.call_log_path) as stream:
        first, second, third = [json.loads(line)["questions_sha256"] for line in stream]
    assert first == second and second != third
