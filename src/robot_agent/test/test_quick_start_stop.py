import importlib.util
import json
from pathlib import Path
from urllib.error import URLError

import pytest


spec = importlib.util.spec_from_file_location(
    "agent_stack_stop", Path(__file__).resolve().parents[3] / "tools/agent_stack_stop.py")
stop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stop)


def client(monkeypatch, config=None, states=None):
    instance = stop.AgentStackClient()
    calls = []
    config = config if config is not None else {
        "stack_stop_token": "session-token", "stack_mode": "sim", "stack_domain_id": "20"}
    states = iter(states or [{"state": "BLOCKED", "detail": "Waiting for motion feedback"}])

    def request(path, token=None):
        calls.append((path, token))
        if path == "/api/config":
            return config
        state = next(states)
        if isinstance(state, Exception):
            raise state
        return state

    monkeypatch.setattr(instance, "request", request)
    monkeypatch.setattr(instance, "_stack_exited", lambda domain: True)
    monkeypatch.setattr(stop.time, "sleep", lambda _: None)
    return instance, calls


@pytest.mark.parametrize("config", [
    {}, {"stack_stop_token": "old"},
    {"stack_stop_token": "token", "stack_mode": "real", "stack_domain_id": "20"},
    {"stack_stop_token": "token", "stack_mode": "sim", "stack_domain_id": "21"},
])
def test_incompatible_stack_never_posts(monkeypatch, config):
    instance, calls = client(monkeypatch, config)
    with pytest.raises(RuntimeError):
        instance.stop("sim", "20", lambda _: None)
    assert calls == [("/api/config", None)]


def test_blocked_shutdown_is_not_forced(monkeypatch):
    instance, calls = client(monkeypatch)
    messages = []
    instance.stop("sim", 20, messages.append)
    assert messages == ["BLOCKED: Waiting for motion feedback"]
    assert calls[-1] == ("/api/stack/stop", "session-token")


def test_acknowledged_disconnect(monkeypatch):
    instance, calls = client(monkeypatch, states=[
        {"state": "STOPPING"}, {"state": "INTENTIONALLY_OFFLINE"}, URLError("closed")])
    messages = []
    instance.stop("sim", 20, messages.append)
    assert "shutdown acknowledged" in messages[-1]
    assert sum(path == "/api/stack/stop" for path, _ in calls) == 1


def test_disconnect_waits_for_old_launcher_exit(monkeypatch):
    instance, calls = client(monkeypatch, states=[
        {"state": "INTENTIONALLY_OFFLINE"}, URLError("closed"), URLError("closed")])
    exited = iter([False, True])
    monkeypatch.setattr(instance, "_stack_exited", lambda domain: next(exited))
    messages = []
    instance.stop("sim", 20, messages.append)
    assert any("waiting for Agent processes" in message for message in messages)
    assert "launch lock released" in messages[-1]
    assert sum(path == "/api/stack/stop" for path, _ in calls) == 1


def test_shutdown_probe_does_not_release_someone_elses_lock(monkeypatch):
    import types
    import sys

    def held(*_):
        raise RuntimeError("locked")
    monkeypatch.setitem(sys.modules, "agent_stack_guard", types.SimpleNamespace(
        find_existing=lambda *_: [], acquire_launch_lock=held))
    assert not stop.AgentStackClient._stack_exited(20)


def test_disconnect_without_ack_is_not_success(monkeypatch):
    instance, _ = client(monkeypatch, states=[{"state": "STOPPING"}, URLError("closed")])
    with pytest.raises(RuntimeError, match="before shutdown confirmation"):
        instance.stop("sim", 20, lambda _: None)


def test_wait_limit_never_forces_exit(monkeypatch):
    instance, calls = client(monkeypatch, states=[{"state": "STOPPING"}])
    with pytest.raises(RuntimeError, match="no forced termination"):
        instance.stop("sim", 20, lambda _: None, wait_sec=0)
    assert len(calls) == 2


def test_session_replacement_never_stopped(monkeypatch):
    instance = stop.AgentStackClient()
    calls = []
    responses = iter([
        {"stack_stop_token": "old", "stack_mode": "sim", "stack_domain_id": "20"},
        {"state": "STOPPING"}, {"stack_stop_token": "new"}])

    def request(path, token=None):
        calls.append(path)
        return next(responses)

    monkeypatch.setattr(instance, "request", request)
    monkeypatch.setattr(stop.time, "sleep", lambda _: None)
    with pytest.raises(RuntimeError, match="session changed"):
        instance.stop("sim", 20, lambda _: None)
    assert calls.count("/api/stack/stop") == 1


def test_request_headers_and_confirmation(monkeypatch):
    from io import BytesIO
    instance = stop.AgentStackClient(18765)
    requests = []

    def open_request(request, timeout):
        requests.append(request)
        assert timeout == 3
        return BytesIO(b'{"state":"STOPPING"}')

    monkeypatch.setattr(instance.opener, "open", open_request)
    instance.request("/api/stack/stop", "secret-token")
    request = requests[0]
    assert request.full_url == "http://127.0.0.1:18765/api/stack/stop"
    assert request.get_header("Origin") == "http://127.0.0.1:18765"
    assert request.get_header("X-agent-stop-token") == "secret-token"
    assert json.loads(request.data) == {"confirm": True}
