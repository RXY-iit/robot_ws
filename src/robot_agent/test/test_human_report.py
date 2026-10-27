from robot_agent.human_report import HumanReportManager


def test_usage_counts_are_not_credentials():
    from robot_agent.human_report import redacted
    assert redacted({"input_tokens_est": 100, "prompt_tokens": 80, "access_token": "private"}) == {
        "input_tokens_est": 100, "prompt_tokens": 80, "access_token": "[REDACTED]"}


def test_ocr_tokens_are_evidence_but_credentials_remain_redacted():
    from robot_agent.human_report import redacted
    assert redacted({"matched_tokens": ["06"], "required_tokens": ["06", "6"],
                     "access_token": "secret", "nested": {"api_key": "key"}}) == {
        "matched_tokens": ["06"], "required_tokens": ["06", "6"],
        "access_token": "[REDACTED]", "nested": {"api_key": "[REDACTED]"}}


def test_recovery_transitions_preserve_progress_and_final_without_duplicates(tmp_path):
    manager = HumanReportManager(tmp_path)
    manager._start("m1")
    progress = {"request_id": "r1", "status": "IN_PROGRESS", "final": False}
    final = {"request_id": "r1", "status": "RESOLVED", "final": True}
    for response in (progress, progress, final, final):
        manager._record_recovery_response(response)
    history = manager.current["recovery_history"]
    assert [r["status"] for r in history["response_transitions"]] == ["IN_PROGRESS", "RESOLVED"]
    assert history["responses"][0]["final"] is True


def test_resolved_response_does_not_leave_current_blocker(tmp_path):
    manager = HumanReportManager(tmp_path)
    manager._start("m1")
    incident = {"id": "i1", "key": "loc", "severity": "blocking", "status": "CONFIRMED"}
    manager._record_incident(incident)
    assert manager.current["blocking_components"]
    manager._record_incident({**incident, "status": "RESOLVED"})
    assert not manager.current["blocking_components"]


def test_healthy_relevant_evidence_refreshes_without_heartbeat_writes(tmp_path):
    manager = HumanReportManager(tmp_path)
    manager._start("m1")
    def health(score):
        return {"domains": [{"id": "localization", "items": [
            {"id": "gicp_score", "status": "OK", "value": score}]}]}
    assert manager._on_health_dashboard_state(health(.1))
    assert not manager._on_health_dashboard_state(health(.2))
    assert len(manager.current["health_transitions"]) == 1
    assert manager.current["health_summary"]["dashboard"]["relevant_evidence"][0]["value"] == .2


def test_warning_without_operator_request_does_not_claim_human_help(tmp_path):
    manager = HumanReportManager(tmp_path)
    manager._on_recovery_state({"incidents": [{"id": "w1", "key": "camera", "severity": "advisory"}], "execution": {}})
    assert manager.current["why_call_human"] == []
    assert manager.current["blocking_components"] == []


def test_report_keeps_history_but_refreshes_current_recovery(tmp_path):
    manager = HumanReportManager(tmp_path)
    manager._start(None, "")
    incident = {"id": "i1", "key": "camera", "status": "CONFIRMED", "severity": "blocking"}
    request = {"id": "r1", "incident_key": "camera", "text": {"en": "Reconnect camera"}}
    manager._on_recovery_state({"incidents": [incident], "execution": {"operator_requests": [request]}})
    assert manager.current["why_call_human"] == ["Reconnect camera"]
    assert manager.current["status"] == "NEEDS_HUMAN"
    manager._on_recovery_state({"incidents": [], "recently_closed": [{**incident, "status": "RESOLVED"}], "execution": {}})
    assert manager.current["status"] == "RESOLVED"
    assert manager.current["blocking_components"] == []
    assert manager.current["recovery_history"]["incidents"][0]["status"] == "RESOLVED"
    assert manager.current["operator_actions"]
    assert manager.current["current_situation"]["recovery"]["operator_requests"] == []


def test_report_uses_health_verdicts_not_nested_workflow_states():
    result = HumanReportManager._compact_health_dashboard({
        "capabilities": [{"id": "arm", "status": "AVAILABLE"}],
        "domains": [{"id": "sensors", "status": "AVAILABLE", "items": [
            {"id": "arm", "status": "OK", "value": {"trigger": {"status": "BLOCKED"}}},
            {"id": "camera", "status": "WARN", "summary": "Low rate"},
        ]}],
    })
    assert [item["name"] for item in result["unhealthy"]] == ["camera"]
