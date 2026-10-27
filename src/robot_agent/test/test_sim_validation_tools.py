import asyncio
import importlib.util
from pathlib import Path
import subprocess
from types import SimpleNamespace

import aiohttp
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("mission_case", ROOT / "tools/sim/run_mission_case.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize("kind", [
    aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
    aiohttp.WSMsgType.CLOSING, aiohttp.WSMsgType.ERROR,
])
def test_closed_transport_is_explicit_not_json_decode_failure(kind):
    async def receive():
        return SimpleNamespace(type=kind)
    with pytest.raises(ConnectionError, match="Dashboard transport ended"):
        asyncio.run(runner.receive_event(SimpleNamespace(receive=receive)))


def test_mission_event_decodes_string_body():
    async def receive():
        return SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                               data='{"topic":"/agent/mission_state","data":"{\\"status\\":\\"DONE\\"}"}')
    assert asyncio.run(runner.receive_event(SimpleNamespace(receive=receive))) == (
        "/agent/mission_state", {"status": "DONE"})


def test_sim_launcher_does_not_inherit_mixed_rmw():
    script = (ROOT / "tools/run_agent_dashboard_sim_clean.sh").read_text()
    # Execute only the non-secret environment policy, never the launch command.
    policy = script.split('export ROS_DOMAIN_ID=', 1)[1].split('export ROBOT_MAP_ID=', 1)[0]
    result = subprocess.run(
        ["bash", "-c", "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
         + "export ROS_DOMAIN_ID=" + policy + '\nprintf "%s" "$RMW_IMPLEMENTATION"'],
        capture_output=True, text=True, check=True)
    assert result.stdout.endswith("rmw_fastrtps_cpp")


def test_completed_human_wait_cannot_cancel_resumed_task():
    budget = runner.HumanWaitBudget()
    budget.observe("NEEDS_HUMAN", 10)
    assert budget.expired(16, 5)
    budget.observe("EXECUTE_STEP", 12)
    assert not budget.expired(100, 5)
    budget.observe("NEEDS_HUMAN", 101)
    assert not budget.expired(103, 5)
    assert budget.expired(107, 5)


def test_action_only_suppresses_its_current_human_wait():
    budget = runner.HumanWaitBudget()
    budget.observe("NEEDS_HUMAN", 10)
    budget.action_sent = True
    assert not budget.expired(20, 5)
    budget.observe("EXECUTE_STEP", 21)
    budget.observe("NEEDS_HUMAN", 30)
    assert budget.expired(36, 5)


@pytest.mark.parametrize("status,expected", [
    ("FAILED_TIMEOUT", True), ("FAILED_SKILL", True), ("CANCELLED", True),
    ("DONE", True), ("RECOVERING", False), (None, False),
])
def test_terminal_confirmation_matches_execution_loop(status, expected):
    assert runner.is_terminal(status) == expected


@pytest.mark.parametrize("observed_status,confirmed", [("EXECUTE_STEP", False), ("DONE", True)])
def test_transport_loss_saves_evidence_without_reconnecting_or_cancelling(
        monkeypatch, tmp_path, observed_status, confirmed):
    import json

    class Clock:
        value = 0

        def monotonic(self):
            self.value += .5
            return self.value

    class Socket:
        closed = False
        observed = False
        mode_timeout = False

        def __init__(self):
            self.sent = []

        async def __aenter__(self):
            return self

        async def __aexit__(self, *_):
            return False

        async def send_json(self, value):
            self.sent.append(value)

        async def receive(self):
            if any(item["type"] == "command" for item in self.sent):
                if self.observed:
                    self.closed = True
                    return SimpleNamespace(type=aiohttp.WSMsgType.CLOSED)
                self.observed = True
                topic, data = "/agent/mission_state", {
                    "mission_id": "test-transport", "status": observed_status,
                    "current_step_index": 0, "skill_results": [],
                }
            elif self.sent:
                if not self.mode_timeout:
                    self.mode_timeout = True
                    raise asyncio.TimeoutError()
                topic, data = "/agent/mode_state", {"agent_mode": "jev_primary"}
            else:
                topic, data = "/agent/health_state", {
                    "components": {"localization": {"details": {"mode": "sim"}}}}
            return SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                                   data=json.dumps({"topic": topic, "data": data}))

    socket = Socket()
    connections = []

    class Session:
        async def __aenter__(self):
            return self

        async def __aexit__(self, *_):
            return False

        def ws_connect(self, *args, **kwargs):
            connections.append(args)
            return socket

    monkeypatch.setattr(runner, "time", Clock())
    monkeypatch.setattr(runner.aiohttp, "ClientSession", Session)
    output = tmp_path / "missions.jsonl"
    args = SimpleNamespace(url="http://test/ws", case="transport", mode="jev_primary",
                           command="status", timeout=100, human_action=None,
                           cancel_on_skill=None, hold_seconds=5, output=str(output))
    result = asyncio.run(runner.run(args))
    assert result["observation_status"] == "TRANSPORT_LOST"
    assert result["final_status_confirmed"] is confirmed
    assert result["status"] == observed_status
    assert not result["cancel_sent"]
    assert len(connections) == 1
    assert not any(item["type"] == "cancel" for item in socket.sent)
    assert json.loads(output.read_text()) == result
