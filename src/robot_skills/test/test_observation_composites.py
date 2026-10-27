from robot_skills.executors.observation import observation_plan, execute_observation
from robot_skills.executors.base import make_result
from robot_skills.schemas import SkillContext
from robot_skills import result_codes as codes
from robot_skills.registry import SkillRegistry
from robot_skills.skill_catalog import SkillCatalog


def test_extensions_share_registry_and_catalog():
    registry, catalog = SkillRegistry(), SkillCatalog()
    assert catalog.validate_registry(registry) == []
    assert {"search_at_place", "check_arm_capability", "set_arm_pose"} <= set(registry.list_names())


def test_nine_view_scan_and_explicit_model():
    plan, views = observation_plan("search_at_place", {"query": "red box", "provider": "yolo11s_ocr"})
    assert views == 9
    assert len([x for x in plan if x[0] == "capture_observation"]) == 9
    assert plan[-1][0] == "detect_known_object"


def test_cancel_and_failed_gate_do_not_run_next_action():
    class Client:
        def __init__(self, fail=False):
            self.calls = []
            self.fail = fail
        def is_cancelled(self, _):
            return len(self.calls) >= 2
        def execute_skill(self, name, args, ctx):
            self.calls.append(name)
            return make_result(name, not self.fail, codes.FAILED_PRECHECK if self.fail else codes.OK, "result")
    for failed in (False, True):
        client = Client(failed)
        result = execute_observation(client, "move_camera_and_capture", {"pan_deg": 0, "tilt_deg": 25}, SkillContext("r"))
        assert not result.success
        assert client.calls == (["await_robot_stationary"] if failed else ["await_robot_stationary", "set_camera_pose"])


def test_invalid_composite_does_not_move():
    class Client:
        def execute_skill(self, *args):
            raise AssertionError("invalid plan must not execute")
    result = execute_observation(Client(), "search_at_place", {"query": "red box", "provider": "invented"}, SkillContext("r"))
    assert result.code == codes.FAILED_SCHEMA


def test_composite_attempts_isolate_images_and_preserve_partial_capture_on_cancel():
    class Client:
        def __init__(self):
            self.scopes = []
            self.captured = False
        def is_cancelled(self, _):
            return self.captured
        def execute_skill(self, name, args, ctx):
            self.scopes.append(ctx.evidence_id)
            observations = []
            if name == "capture_observation":
                self.captured = True
                observations = [{"image_path": "test-image.jpg", "view_id": args["view_id"]}]
            return make_result(name, True, codes.OK, "result", observations=observations)
    first, second = Client(), Client()
    context = SkillContext("run", evidence_id="task")
    for client in (first, second):
        result = execute_observation(client, "capture_view_set", {}, context)
        assert result.code == codes.CANCELLED
        assert result.observations[0]["image_path"] == "test-image.jpg"
        assert len(set(client.scopes)) == 1
    assert first.scopes[0] != second.scopes[0]


def test_many_checks_capture_once_and_aggregate_not_only_last_result():
    class Client:
        def __init__(self):
            self.calls, self.scopes = [], []
        def is_cancelled(self, _):
            return False
        def execute_skill(self, name, args, ctx):
            self.calls.append(name)
            self.scopes.append(ctx.evidence_id)
            observations = [{"image_path": args["view_id"]}] if name == "capture_observation" else []
            return make_result(name, True, codes.OK, "done", observations=observations,
                               metrics={"inference_completed": True, "goal_satisfied": args.get("query") == "bottle"})
    for relation, satisfied in (("all", False), ("any", True)):
        client = Client()
        result = execute_observation(client, "search_at_place", {"aggregation": relation, "checks": [
            {"query": "cup", "provider": "yolo11s"}, {"query": "bottle", "provider": "yolo11s"}]}, SkillContext("r"))
        assert result.success and result.metrics["goal_satisfied"] is satisfied
        assert client.calls.count("capture_observation") == 9
        assert client.calls.count("detect_known_object") == 2
        assert result.metrics["usable_views"] == 9
        assert len(result.metrics["check_results"]) == 2
        assert len(set(client.scopes)) == 1


def test_partial_capability_never_claims_complete_goal():
    class Client:
        def is_cancelled(self, _):
            return False
        def execute_skill(self, name, args, ctx):
            return make_result(name, True, codes.OK, "done", metrics={"goal_satisfied": True},
                               observations=[{"image_path": args["view_id"]}] if name == "capture_observation" else [])
    result = execute_observation(Client(), "search_at_place", {
        "checks": [{"query": "color_red", "provider": "yolo11s"}],
        "unresolved_requirements": [{"kind": "object", "reason": "box unsupported"}]}, SkillContext("r"))
    assert result.success and result.metrics["outcome"] == "PARTIAL"
    assert result.metrics["goal_satisfied"] is False
