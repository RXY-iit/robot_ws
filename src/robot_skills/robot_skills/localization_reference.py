"""Quality-qualified localization reference; no injector or oracle input."""
import math
from collections import deque


def angle(v):
    return math.atan2(math.sin(v), math.cos(v))


def propagate(p, a, b):
    c, s = math.cos(p[2]-a[2]), math.sin(p[2]-a[2])
    return (p[0]+c*(b[0]-a[0])-s*(b[1]-a[1]),
            p[1]+s*(b[0]-a[0])+c*(b[1]-a[1]), angle(p[2]+b[2]-a[2]))


class LocalizationReference:
    def __init__(self, score_limit=.15, stable_sec=3., max_path=3., max_turn=1.0, max_age=120.):
        self.score_limit, self.stable_sec = score_limit, stable_sec
        self.max_path, self.max_turn, self.max_age = max_path, max_turn, max_age
        self.epoch = 0; self.last_odom = None; self.path = self.turn = 0.
        self.history = deque(maxlen=5000)
        self.anchor = None; self.candidate = None; self.verified = False
        self.reset_required = False
        self.frozen = False; self.reason = 'no_qualified_reference'

    def invalidate(self, reason):
        self.reset_required = True
        self.history.clear()
        self.epoch += 1; self.anchor = None; self.candidate = None
        self.verified = False; self.frozen = True; self.reason = reason

    def odom(self, pose, stamp):
        if not all(math.isfinite(v) for v in (*pose, stamp)):
            self.invalidate('nonfinite_odom'); self.last_odom = None; return
        if self.last_odom:
            old, at = self.last_odom; dt = stamp-at
            distance = math.dist(pose[:2], old[:2]); turn = abs(angle(pose[2]-old[2]))
            if dt == 0 and distance < 1e-6 and turn < 1e-6:return
            if dt <= 0 or dt > 2 or distance > 2*dt+.15 or turn > 3*dt+.15:
                self.invalidate('odom_discontinuity')
            else:self.path += distance; self.turn += turn
        self.last_odom = (tuple(pose), stamp)
        self.history.append((stamp, self.path, self.turn))

    def disturb(self, reason):
        self.frozen = True; self.verified = False; self.candidate = None; self.reason = reason

    def prediction(self, pose, wall_now):
        a = self.anchor
        if not a or a['epoch'] != self.epoch:return None
        if not 0 <= wall_now-a['wall'] <= self.max_age:return None
        if self.path-a['path'] > self.max_path or self.turn-a['turn'] > self.max_turn:return None
        return propagate(a['pose'], a['odom'], pose)

    def observe(self, pose, odom, stamp, wall_now, score, settling=False):
        if (odom is None or not all(math.isfinite(v) for v in (*pose, *odom, stamp, score))
                or not 0 <= score < self.score_limit or settling):
            self.disturb('input_not_qualified'); return False
        if self.last_odom is None or not 0 <= self.last_odom[1]-stamp <= 2:
            self.disturb('unaligned_odom'); return False
        # Cumulative travel must be anchored at the SCAN stamp, not at the
        # later callback/receipt time. Otherwise sensor lag erases travel budget.
        before = next((h for h in reversed(self.history) if h[0] <= stamp), None)
        after = next((h for h in self.history if h[0] >= stamp), None)
        if before is None or after is None:
            self.disturb('missing_scan_time_odom_history'); return False
        ratio = (stamp-before[0])/(after[0]-before[0]) if after[0]>before[0] else 0.
        path_at_scan = before[1]+ratio*(after[1]-before[1])
        turn_at_scan = before[2]+ratio*(after[2]-before[2])
        # A candidate during recovery must agree with the OLD qualified anchor.
        # Never refresh that anchor first and then validate against itself.
        if self.frozen and (self.anchor is not None or self.reset_required):
            predicted = self.prediction(odom, wall_now)
            if predicted is None:
                self.reason = 'reference_unavailable_manual_alignment_required'; return False
            if math.dist(predicted[:2], pose[:2]) > .15 or abs(angle(predicted[2]-pose[2])) > .20:
                self.candidate = None; self.reason = 'reference_disagreement'; return False
        c = self.candidate
        if c:
            expected = propagate(c['pose'], c['odom'], odom)
            if stamp <= c['last'] or math.dist(expected[:2], pose[:2]) > .15 or abs(angle(expected[2]-pose[2])) > .20:
                self.disturb('inconsistent_map_odom'); return False
        if c is None:
            c = self.candidate = dict(pose=pose, odom=odom, start=stamp, last=stamp, count=0)
        c['last'] = stamp; c['count'] += 1
        if c['count'] < 3 or stamp-c['start'] < self.stable_sec:
            self.reason = 'collecting_stable_reference'; return False
        self.anchor = dict(pose=tuple(pose), odom=tuple(odom), stamp=stamp, wall=wall_now,
                           epoch=self.epoch, path=path_at_scan, turn=turn_at_scan, score=score)
        self.verified = True; self.frozen = False; self.reason = 'verified'
        return True

    def metrics(self, wall_now):
        a = self.anchor
        return dict(reference_policy_version=1, reference_qualified=a is not None,
                    reference_verified=self.verified, reference_reason=self.reason,
                    reference_epoch=self.epoch, trusted_reference_epoch=a['epoch'] if a else None,
                    reference_score=a['score'] if a else None,
                    reference_path_m=self.path-a['path'] if a else None,
                    reference_turn_rad=self.turn-a['turn'] if a else None,
                    reference_age_wall_sec=wall_now-a['wall'] if a else None)
