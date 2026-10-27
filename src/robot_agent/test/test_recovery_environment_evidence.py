import builtins
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from robot_agent.recovery.primitives import LocalSystemAdapter
import robot_agent.recovery.primitives as primitives


def test_isaac_timeline_heartbeat_runs_without_ros_or_play_and_replaces_subscription(monkeypatch, tmp_path):
    subscriptions = []
    state = {"playing": False, "stopped": True, "now": 100.0}
    class Subscription:
        def __init__(self, callback):
            self.callback, self.closed = callback, False
        def unsubscribe(self):
            self.closed = True
    def subscribe(callback):
        subscriptions.append(Subscription(callback))
        return subscriptions[-1]
    timeline = SimpleNamespace(get_timeline_interface=lambda: SimpleNamespace(
        is_playing=lambda: state["playing"], is_stopped=lambda: state["stopped"]))
    app = SimpleNamespace(get_app=lambda: SimpleNamespace(get_update_event_stream=lambda: SimpleNamespace(create_subscription_to_pop=subscribe)))
    kit = SimpleNamespace(app=app)
    for name, value in {"omni": SimpleNamespace(kit=kit, timeline=timeline), "omni.kit": kit,
                        "omni.kit.app": app, "omni.timeline": timeline}.items():
        monkeypatch.setitem(sys.modules, name, value)
    monkeypatch.setattr(builtins, "_robot_recovery_environment_subscription", None, raising=False)
    source = Path(__file__).resolve().parents[2] / "robot_sim/scripts/sim/publish_recovery_environment.py"
    spec = importlib.util.spec_from_file_location("timeline_fixture", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.time = SimpleNamespace(monotonic=lambda: state["now"])
    module.os = SimpleNamespace(getpid=lambda: 1234, getuid=lambda: 1000, environ={"ROS_DOMAIN_ID": "214"})
    module.install(tmp_path)
    record = tmp_path / "1234.json"
    assert json.loads(record.read_text())["timeline"] == "STOPPED"
    state.update(stopped=False, now=101)
    subscriptions[-1].callback(None)
    assert json.loads(record.read_text())["timeline"] == "PAUSED"
    state.update(playing=True, now=102)
    subscriptions[-1].callback(None)
    assert json.loads(record.read_text())["timeline"] == "PLAYING"
    module.install(tmp_path)
    assert subscriptions[0].closed and not subscriptions[1].closed
    assert len(list(tmp_path.glob("*.json"))) == 1


@pytest.mark.parametrize("domain,denied,expected", [("20",False,True),("21",False,None),("20",True,None)])
def test_process_reader_domain_or_permissions_cannot_fake_offline(monkeypatch, domain, denied, expected):
    monkeypatch.setenv("ROS_DOMAIN_ID", "20")
    monkeypatch.setattr(primitives.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=0, stdout="1234\n"))
    class FakePath:
        def __init__(self, path):
            self.path = str(path)
        @property
        def name(self):
            return self.path.rsplit("/",1)[-1]
        def read_bytes(self):
            if denied:
                raise PermissionError("fixture")
            return b"/home/test/isaac-sim/kit/kit\0app.kit\0" if self.path.endswith("cmdline") else f"ROS_DOMAIN_ID={domain}\0".encode()
        def read_text(self):
            return json.dumps({"pid":1234,"domain":domain,"monotonic":99.9,"timeline":"PAUSED"})
    monkeypatch.setattr(primitives, "Path", FakePath)
    result = LocalSystemAdapter().simulation_evidence(100)
    assert result["process_alive"] is expected
    if expected:
        assert result["timeline"] == "PAUSED"


def test_no_matching_process_is_offline_but_probe_error_is_unknown(monkeypatch):
    monkeypatch.setattr(primitives.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=1, stdout=""))
    assert LocalSystemAdapter().simulation_evidence(100)["process_alive"] is False
    monkeypatch.setattr(primitives.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=2, stdout=""))
    assert LocalSystemAdapter().simulation_evidence(100)["process_alive"] is None
