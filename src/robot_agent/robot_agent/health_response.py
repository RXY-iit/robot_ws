"""Robot response severity, distinct from evidence and capability readiness."""
import math

DOMAIN_CHECKS = {
    'H1': ['device presence/link', 'native alarm and temperature limits'],
    'H2': ['source/receive timestamps and clock progress', 'CPU/memory/thermal evidence'],
    'H3': ['command authority and safety interlocks', 'command versus measured motion'],
    'H4': ['stream freshness/rate', 'payload, synchronization and camera model'],
    'H5': ['RMSE plus match coverage', 'accepted pose freshness and motion consistency'],
    'H6': ['lifecycle and TF', 'controller diagnostics, path and progress'],
    'H7': ['agent heartbeat', 'task/executor status and recovery handoff'],
}

ITEM_CHECKS = {
    'gicp_score': ['raw RMSE persistence and match fitness/count', 'accepted-state age and correction rejection', 'human: consider slower motion if navigation is active'],
    'gicp_pose': ['source versus receive age', 'accepted_state versus broadcast pose; inspect rejection reason'],
    'localization_consistency': ['command/measured motion versus drift', 'independent truth before treating slow slide as localization error'],
    'host_resources': ['native max/crit temperatures; CPU, memory and trend', 'human: reduce optional workload; no automatic process termination'],
    'sim_rtf': ['clock is advancing', 'source-time rate versus wall-time delivery', 'human: reduce rendering/sensor workload; low RTF alone is advisory'],
    'sim_sync:rgb_depth': ['nearest timestamp skew and frame/model', 'check whether current capability requires depth pairing'],
    'lidar_device_status': ['native device diagnostics and delivery age', 'check clock/input adapter before driver restart'],
    'fast_lio_odom': ['source versus receive age', 'FAST-LIO timing reset latch and input delivery'],
}

def response_policy(item, domain):
    status = item.get('status', 'UNKNOWN')
    if status in {'ERROR', 'BLOCKED', 'STALE'}:
        level, action = 'BAD', 'diagnose_playbook_or_escalate'
    elif status in {'WARN', 'DEGRADED', 'UNKNOWN', 'NOT_OBSERVED'}:
        level, action = 'WARN', 'check_and_notify'
    else:
        level, action = 'OK', 'skip'
    return dict(level=level, action=action, checks=ITEM_CHECKS.get(item.get('id'), DOMAIN_CHECKS.get(domain, ['inspect source evidence'])),
                automatic_motion_change=False, readiness_authority=False,
                calibration='item evidence rules; uncalibrated limits retained')

def annotate_domains(domains):
    for domain in domains:
        for item in domain.get('items', []):
            item['response_policy'] = response_policy(item, domain['id'])

def robot_summary(items, ignored=()):
    ignored = set(ignored)
    entries = []
    for key, item in items.items():
        policy = item.get('response_policy') or response_policy(item, '')
        raw = policy['level']
        effective = 'OK' if raw == 'WARN' and key in ignored else raw
        if raw != 'OK':
            entries.append(dict(item=key, observed_level=raw, level=effective,
                                ignored=raw != effective, action='skip' if effective == 'OK' else policy['action'],
                                checks=policy['checks']))
    if not items:
        return dict(level='WARN', scope='robot_health_response', motion_authority=False, items=[], note='No health snapshot available; cannot claim healthy.')
    level = max((e['level'] for e in entries), key={'OK':0,'WARN':1,'BAD':2}.get, default='OK')
    return dict(level=level, scope='robot_health_response', motion_authority=False,
                items=entries, note='Ignored warnings retain raw evidence; BAD cannot be ignored here.')

class ScoreBand:
    """Sim-only candidate thresholds; continuous fresh evidence required to clear.

    WARN enters after 3s; BAD after 1s; recovery below .15/.18 lasts 10s.
    A source gap >3s resets persistence, never hides stale delivery evidence.
    """
    def __init__(self, warn=.16, bad=.20):
        if not 0 < warn < bad:
            raise ValueError('Expected 0 < warn < bad')
        self.warn, self.bad = warn, bad
        self.level = 'OK'
        self.pending = None
        self.since = None
        self.last = None

    def update(self, score, now):
        if not math.isfinite(score) or score < 0:
            self.level = 'BAD'; self.pending = None; self.last = now
            return self.level
        if self.last is not None and now-self.last > 3:
            self.pending = None
        self.last = now
        target = 'BAD' if score >= self.bad else 'WARN' if score >= self.warn else 'OK'
        if self.level == 'BAD' and score >= self.bad * .9:
            target = 'BAD'
        elif self.level == 'WARN' and score >= self.warn - .01 and target == 'OK':
            target = 'WARN'
        if target == self.level:
            self.pending = None
        else:
            if self.pending != target:
                self.pending, self.since = target, now
            rising = {'OK':0,'WARN':1,'BAD':2}[target] > {'OK':0,'WARN':1,'BAD':2}[self.level]
            delay = (1 if target == 'BAD' else 3) if rising else 10
            if now-self.since >= delay:
                self.level, self.pending = target, None
        return self.level
