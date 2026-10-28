"""Pure, testable B1 models. No ROS commands or experiment labels enter Recovery."""
import math


def wrap(v):return math.atan2(math.sin(v), math.cos(v))


class OdomError:
    """Integrate local increments; stopping growth retains accumulated error."""
    def __init__(self, distance_scale=0., turn_scale=0., turn_bias_rad_per_m=0.):
        values=(distance_scale,turn_scale,turn_bias_rad_per_m)
        if not all(math.isfinite(v) for v in values):raise ValueError('Finite gains required')
        self.gains=values;self.previous=None;self.pose=None;self.active=False
        self.distance=self.turn=0.;self.epoch=0

    def update(self, raw, stamp):
        if not all(math.isfinite(v) for v in (*raw,stamp)):raise ValueError('Nonfinite odometry')
        if self.previous is None:
            self.previous=(tuple(raw),stamp);self.pose=tuple(raw);return self.pose,(0.,0.,0.)
        old,at=self.previous;dt=stamp-at
        if dt<=0 or dt>2:raise ValueError('Odometry timeline discontinuity; explicit stopped reset required')
        dx,dy=raw[0]-old[0],raw[1]-old[1];da=wrap(raw[2]-old[2])
        distance=math.hypot(dx,dy)
        if distance>2*dt+.15 or abs(da)>3*dt+.15:raise ValueError('Raw odometry jumped')
        c,s=math.cos(old[2]),math.sin(old[2]);bx=c*dx+s*dy;by=-s*dx+c*dy
        scale,turn_scale,bias=self.gains if self.active else (0.,0.,0.)
        bx*=1+scale;by*=1+scale
        # Turn-related bias only grows while actually turning.
        injected_da=da*(1+turn_scale)+(math.copysign(1.,da)*bias*distance if abs(da)>1e-5 else 0.)
        x,y,a=self.pose;c,s=math.cos(a),math.sin(a)
        self.pose=(x+c*bx-s*by,y+s*bx+c*by,wrap(a+injected_da))
        self.previous=(tuple(raw),stamp);self.distance+=distance;self.turn+=abs(da)
        return self.pose,(bx/dt,by/dt,injected_da/dt)


class StableSegment:
    """Only arm after success at A and acceptance of the goal at B."""
    def __init__(self, start_xy, end_xy, stable_sec=2., distance=.3):
        self.start_xy=start_xy;self.end_xy=end_xy;self.stable_sec=stable_sec;self.distance=distance
        self.departure=False;self.arrival=False;self.origin=None;self.stable_since=None

    def goal(self, xy, event):
        if math.dist(xy,self.start_xy)<.05 and event=='succeeded':self.departure=True
        if self.departure and math.dist(xy,self.end_xy)<.05 and event=='accepted':self.arrival=True

    def update(self, xy, yaw, speed, angular, sim):
        heading=math.atan2(self.end_xy[1]-self.start_xy[1],self.end_xy[0]-self.start_xy[0])
        if not self.arrival:return False
        if self.origin is None:self.origin=tuple(xy)
        stable=speed>.03 and abs(angular)<.08 and abs(wrap(yaw-heading))<.2
        if not stable:self.stable_since=None;return False
        if self.stable_since is None:self.stable_since=sim
        return sim-self.stable_since>=self.stable_sec and math.dist(xy,self.origin)>=self.distance


class FaultSchedule:
    """Committed sim-time windows; goal cancellation never changes their duration."""
    def __init__(self):
        self.starts = {}; self.durations = {}; self.clock = None

    def start(self, factor, sim, duration):
        if factor in self.starts or not all(math.isfinite(x) for x in (sim,duration)) or duration<=0:
            raise ValueError('Single finite positive fault window required')
        self.starts[factor]=sim;self.durations[factor]=duration

    def elapsed(self, factor, sim):
        if self.clock is not None and sim<self.clock:raise ValueError('Simulation clock reset invalidates episode')
        self.clock=sim
        return sim-self.starts[factor]>=self.durations[factor]
