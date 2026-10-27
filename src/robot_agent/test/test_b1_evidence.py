import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String
from builtin_interfaces.msg import Time

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location("b1_evidence", ROOT / "tools/sim/b1_evidence.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_full_route_and_timestamped_independent_evidence(tmp_path):
    published = []
    topics = []
    def subscribe(kind, topic, cb, qos):
        topics.append(topic)
    node = SimpleNamespace(create_subscription=subscribe,
        create_publisher=lambda *a: SimpleNamespace(publish=published.append),
        create_timer=lambda *a: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))))
    observer = module.B1Evidence(node, tmp_path, ROOT / "src/robot_skills/config/waypoint_list_a.yaml")
    assert len(published[-1].markers) == 15
    assert len(published[-1].markers[0].points) == 7
    assert {"/wheel_odom", "/gicp_loc/pose", "/gicp_loc/score"} <= set(topics)
    observer.on_clock(Clock(clock=Time(sec=10)))
    pose = PoseStamped()
    pose.header.stamp.sec = 9
    pose.header.frame_id = "map"
    observer.receive("/gicp_loc/pose", pose)
    observer.receive("/gicp_loc/score", Float32(data=.12))
    observer.on_clock(Clock(clock=Time(sec=0)))
    observer.close()
    records = [json.loads(line) for line in (tmp_path / "b1_evidence.jsonl").read_text().splitlines()]
    p = next(r for r in records if r["event"] == "/gicp_loc/pose")
    assert p["sim_sec"] == 10 and p["source_stamp_sec"] == 9
    assert p["data"]["header"]["frame_id"] == "map"
    score = next(r for r in records if r["event"] == "/gicp_loc/score")
    assert score["source_stamp_sec"] is None
    assert score["wall_unix_sec"] > 0 and score["wall_monotonic_sec"] > 0
    assert any(r["event"] == "clock_reset" for r in records)


def test_health_burst_and_capture_freshness(tmp_path, monkeypatch):
    tick = [100.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: tick[0])
    node = SimpleNamespace(create_subscription=lambda *a: None,
        create_publisher=lambda *a: None, create_timer=lambda *a: None)
    observer = module.B1Evidence(node, tmp_path)
    status_path = tmp_path / "b1_capture_status.json"
    assert not json.loads(status_path.read_text())["capture_receiving"]
    observer.on_clock(Clock(clock=Time(sec=10)))
    for topic in json.loads(status_path.read_text())["required_topics"]:
        if topic != "/clock":
            observer.receive(topic, String(data='{"status":"OK"}'))
    # All three transitions within one wall second must be preserved.
    for status in ["WARN", "BAD", "OK"]:
        observer.receive("/agent/health_state", String(data=json.dumps({"status": status})))
    observer.publish()
    assert json.loads(status_path.read_text())["capture_receiving"]
    tick[0] += 4
    observer.publish()
    status = json.loads(status_path.read_text())
    assert not status["capture_receiving"]
    assert "/gicp_loc/score" in status["stale_topics"]
    assert status["clock_advance_age_wall_sec"] == 4
    observer.close()
    records = [json.loads(line) for line in (tmp_path / "b1_evidence.jsonl").read_text().splitlines()]
    assert [r["data"]["status"] for r in records if r["event"] == "/agent/health_state"] == ["OK", "WARN", "BAD", "OK"]
