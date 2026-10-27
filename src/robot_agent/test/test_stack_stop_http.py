import asyncio
import json
import time
from types import SimpleNamespace

from aiohttp.test_utils import TestClient, TestServer

from robot_agent.nodes import dashboard_node


def test_stop_http_requires_confirmation_and_matching_acks(monkeypatch):
    monkeypatch.setenv("ROBOT_AGENT_SESSION_ID", "test-session")
    monkeypatch.setattr(dashboard_node, "launch_parent", lambda: 123)
    stopped = []
    monkeypatch.setattr(dashboard_node, "stop_launch", lambda: stopped.append("SIGINT verified parent"))
    monkeypatch.setattr(dashboard_node, "HumanReportManager", lambda _: SimpleNamespace())
    monkeypatch.setattr(dashboard_node.DashboardState, "_build_quick_commands", lambda _: ([], ""))
    state = dashboard_node.DashboardState()
    published = []
    ready = [False]

    def publish(msg):
        request = json.loads(msg.data)
        published.append(request)
        for name in ("agent_node", "recovery_agent"):
            state.stack_ack(SimpleNamespace(data=json.dumps({**request, "node": name,
                "ready": ready[0], "detail": "draining"})))

    state.stack_control_pub = SimpleNamespace(publish=publish)

    async def scenario():
        client = TestClient(TestServer(dashboard_node._build_app(state)))
        await client.start_server()
        origin = str(client.make_url("/")).rstrip("/")
        headers = {"Origin": origin, "X-Agent-Stop-Token": state.stack_token}
        try:
            assert (await client.post("/api/stack/stop", json={"confirm": True})).status == 403
            assert (await client.post("/api/stack/stop", headers={**headers, "Origin": "https://evil.example"},
                                      json={"confirm": True})).status == 403
            assert (await client.post("/api/stack/stop", headers=headers, json={"confirm": False})).status == 400
            assert not stopped and state.stack_state["state"] == "RUNNING"
            assert (await client.post("/api/stack/stop", headers=headers, json={"confirm": True})).status == 202
            await asyncio.sleep(.1)
            assert published and not stopped
            first_request = state.stack_state["request_id"]
            # Idempotent stop request; it does not create another drain operation.
            await client.post("/api/stack/stop", headers=headers, json={"confirm": True})
            assert state.stack_state["request_id"] == first_request
            state.stack_ack(SimpleNamespace(data=json.dumps({"session_id": "old", "request_id": first_request,
                                                            "node": "agent_node", "ready": True})))
            assert state.stack_acks["agent_node"][1]["ready"] is False
            ready[0] = True
            await asyncio.wait_for(state.stack_task, timeout=3)
            assert stopped == ["SIGINT verified parent"]
            assert state.stack_state["state"] == "INTENTIONALLY_OFFLINE"
        finally:
            await client.close()
    asyncio.run(scenario())


def test_unmanaged_stop_is_rejected_without_signalling(monkeypatch):
    def unmanaged():
        raise RuntimeError("Unmanaged launch")
    monkeypatch.setattr(dashboard_node, "launch_parent", unmanaged)
    monkeypatch.setattr(dashboard_node, "HumanReportManager", lambda _: SimpleNamespace())
    monkeypatch.setattr(dashboard_node.DashboardState, "_build_quick_commands", lambda _: ([], ""))
    state = dashboard_node.DashboardState()
    async def scenario():
        client = TestClient(TestServer(dashboard_node._build_app(state)))
        await client.start_server()
        try:
            response = await client.post("/api/stack/stop", json={"confirm": True}, headers={
                "Origin": str(client.make_url("/")).rstrip("/"), "X-Agent-Stop-Token": state.stack_token})
            assert response.status == 409
            assert state.stack_task is None
        finally:
            await client.close()
    asyncio.run(scenario())


def test_missing_ack_blocks_without_force_kill(monkeypatch):
    monkeypatch.setenv("ROBOT_AGENT_SESSION_ID", "test-session")
    monkeypatch.setattr(dashboard_node, "launch_parent", lambda: 123)
    monkeypatch.setattr(dashboard_node, "STACK_STOP_WAIT_SEC", .01)
    monkeypatch.setattr(dashboard_node, "HumanReportManager", lambda _: SimpleNamespace())
    monkeypatch.setattr(dashboard_node.DashboardState, "_build_quick_commands", lambda _: ([], ""))
    signals = []
    monkeypatch.setattr(dashboard_node, "stop_launch", lambda: signals.append("SIGINT"))
    state = dashboard_node.DashboardState()
    state.stack_control_pub = SimpleNamespace(publish=lambda _: None)
    asyncio.run(state.stop_stack())
    assert state.stack_state["state"] == "BLOCKED"
    assert not signals
