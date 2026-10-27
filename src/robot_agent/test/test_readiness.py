from robot_agent.readiness import ReadinessTracker


def evidence():
    tracker = ReadinessTracker()
    empty = tracker.update([], sim=True, now="t0")
    names = {c["id"] for g in empty["groups"] for c in g["checks"]} - {"clock_advancing"}
    items = [{"id": name, "status": "OK", "summary": name} for name in names]
    items += [{"id": "sim_rtf", "status": "WARN", "summary": "RTF 0.4",
               "value": {"clock_receive_age_sec": .02, "not_advancing_sec": .02}}]
    return [{"id": "H1", "items": items}]


def group(state, name):
    return next(g for g in state["groups"] if g["id"] == name)


def test_no_fixed_delay_missing_evidence_never_becomes_ready():
    tracker = ReadinessTracker()
    for now in ("t0", "t999"):
        assert all(g["status"] == "WAITING" for g in tracker.update([], sim=True, now=now)["groups"])


def test_advisory_rtf_and_manual_do_not_block_evidence_readiness():
    domains = evidence()
    next(i for i in domains[0]["items"] if i["id"] == "delivery_robot_mode")["value"] = "MANUAL"
    state = ReadinessTracker().update(domains, sim=True, now="t0")
    assert all(g["status"] == "READY" for g in state["groups"])
    assert state["supplemental"][0]["status"] == "WARN"


def test_lost_localization_regresses_navigation_not_other_groups():
    domains = evidence()
    tracker = ReadinessTracker()
    tracker.update(domains, sim=True, now="t1")
    item = next(i for i in domains[0]["items"] if i["id"] == "gicp_pose")
    item["status"] = "STALE"
    state = tracker.update(domains, sim=True, now="t2")
    nav = group(state, "navigation")
    assert nav["status"] == "BLOCKED"
    assert nav["first_ready_at"] == "t1" and nav["ready_since"] is None
    assert group(state, "perception")["status"] == "READY"
    item["status"] = "OK"
    assert group(tracker.update(domains, sim=True, now="t3"), "navigation")["ready_since"] == "t3"


def test_paused_clock_blocks_base_without_global_rtf_bypass():
    domains = evidence()
    domains[0]["items"][-1]["value"]["not_advancing_sec"] = 6
    assert group(ReadinessTracker().update(domains, sim=True, now="t0"), "base")["status"] == "BLOCKED"


def test_rgbd_and_model_history_not_falsely_claimed_ready():
    domains = evidence()
    domains[0]["items"] += [{"id": "sim_sync:rgb_depth", "status": "WARN", "summary": "0.317s skew"},
                             {"id": "perception_runtime", "status": "IDLE", "summary": "No inference yet"}]
    state = ReadinessTracker().update(domains, sim=True, now="t0")
    assert group(state, "perception")["title"] == "Visual acquisition"
    assert {x["status"] for x in state["supplemental"]} == {"WARN", "IDLE"}


def test_new_monitor_resets_first_ready_time_and_real_does_not_require_sim_clock():
    domains = evidence()
    domains[0]["items"] = [i for i in domains[0]["items"] if not i["id"].startswith("sim_")]
    state = ReadinessTracker().update(domains, sim=False, now="new-session")
    assert group(state, "base")["first_ready_at"] == "new-session"
