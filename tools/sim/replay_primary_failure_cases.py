#!/usr/bin/env python3
"""Real JEV, synthetic skills/recovery: never publishes robot commands.

Requires an explicitly configured JEV_API_KEY. This is not a live sensor fault
injection or validation of Recovery playbook execution.
"""
import argparse
import json
import os
from pathlib import Path
from types import SimpleNamespace

from robot_agent.agents.mission_agent import MissionAgent
from robot_skills.executors.base import make_result
from robot_skills.schemas import SkillContext
from robot_skills import result_codes as codes


class ReplayClient:
    node = None

    def __init__(self):
        self.calls = []
        self.failure = None
        self.failures_left = 0

    def context(self, run_id, max_duration_sec=120):
        return SkillContext(run_id, robot_mode="AUTO", max_duration_sec=max_duration_sec)

    def execute_skill(self, name, payload, context):
        self.calls.append(name)
        if self.failure and name == self.failure[0] and self.failures_left:
            self.failures_left -= 1
            return make_result(name, False, self.failure[1], self.failure[2], metrics=self.failure[3])
        return make_result(name, True, codes.OK, "Synthetic executor success", metrics={"completed_time_monotonic": 1.0})

    def cancel_all(self, run_id):
        return make_result("cancel_motion", True, codes.OK, "Synthetic stop")

    stop_all = cancel_all


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if not os.environ.get('JEV_API_KEY'):
        raise RuntimeError('Explicit live JEV configuration required')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['JEV_CALL_LOG_PATH'] = str(output.with_name('failure-replay-jev-calls.jsonl'))
    client = ReplayClient()
    events = []
    agent = MissionAgent(client, event_cb=events.append)
    agent.set_agent_mode('jev_primary')
    if not agent.jev_agent.is_live_enabled():
        raise RuntimeError('JEV probe failed')
    cases = [
        ('F01-camera-repair', 'look left, then set pose home',
         ('capture_observation', codes.FAILED_PERCEPTION, 'Camera publisher stopped; no fresh image.', {'image_age_sec': 30, 'image_topic': '/camera/camera/color/image_raw'}), 1, 'resolved'),
        ('F02-motion-odom', 'go back 20cm, then set pose home',
         ('move_relative', codes.FAILED_NAVIGATION, 'Direct motion stopped: wheel_odom_stale.', {'timeout_reason': 'wheel_odom_stale', 'travelled_m': .08, 'control_mode': 'direct_cmd_vel_raw', 'wheel_odom_rejection': {'reason': 'source_stale', 'source_age_sec': 2.0, 'max_age_sec': .75}}), 1, 'resolved'),
        ('F03-recovery-cancel', 'look left',
         ('capture_observation', codes.FAILED_PERCEPTION, 'Camera publisher stopped; no fresh image.', {'image_age_sec': 30}), 1, 'cancel'),
        ('F04-persistent-camera', 'look left',
         ('capture_observation', codes.FAILED_PERCEPTION, 'Camera publisher stopped; no fresh image.', {'image_age_sec': 30}), 10, 'resolved'),
        ('F05-auto-required', 'go back 20cm',
         ('await_robot_auto_mode', codes.NEEDS_HUMAN, 'Robot is in MANUAL mode; AUTO is required.', {'robot_mode': 'MANUAL'}), 1, 'resolved'),
    ]
    for case, command, failure, count, response_mode in cases:
        client.calls.clear()
        events.clear()
        client.failure, client.failures_left = failure, count
        recovery_requests = []
        def publish(message):
            request = json.loads(message.data)
            recovery_requests.append(request)
            agent._on_recovery_response(SimpleNamespace(data=json.dumps({
                'request_id': request['request_id'], 'mission_id': request['mission_id'],
                'status': 'IN_PROGRESS', 'final': False})))
            if response_mode == 'cancel':
                agent.cancel(request['mission_id'])
            agent._on_recovery_response(SimpleNamespace(data=json.dumps({
                'request_id': request['request_id'], 'mission_id': request['mission_id'],
                'status': 'RESOLVED', 'final': True, 'advice': 'resume_from_step',
                'summary': 'Synthetic recovery reply; not a real playbook result'})))
        agent._recovery_request_pub = SimpleNamespace(publish=publish)
        state = agent.run(command)
        record = {'case': case, 'evidence_kind': 'live_jev_synthetic_execution',
                  'state': state.to_dict(), 'calls': client.calls, 'recovery_requests': recovery_requests,
                  'events': events}
        with output.open('a') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        print(json.dumps({'case': case, 'status': state.status, 'outcome': state.outcome,
                          'calls': client.calls, 'recovery_calls': len(recovery_requests),
                          'decisions': [(c['stage'], c['provider'], c['adopted_decision']) for c in state.jev_trace['calls']]}, ensure_ascii=False), flush=True)
        if not agent.jev_agent.is_live_enabled():
            raise RuntimeError('API unavailable; stopping instead of counting fallback')


if __name__ == '__main__':
    main()
