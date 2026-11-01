#!/usr/bin/env python3
"""Sim isolated physics-odom -> wheel_odom + sole odom TF; no live default replacement.
Requires Isaac B1_ODOM_RELAY=1 setup. start/stop control error growth; stop
NEVER clears accumulated drift. Restart/re-anchor only while MANUAL stationary.
"""
import argparse,json,math,os,time
from collections import deque
from pathlib import Path
from b1_fault_models import OdomError, OdomBurst


def repeated_sample(previous, raw, stamp):
    """Only identical pose/time samples may be discarded without integration."""
    return previous is not None and previous == (tuple(raw), stamp)


class TransformEchoes:
    """Consume exactly one echo per published TF; Humble lacks callback GIDs."""
    def __init__(self):
        self.pending = deque(maxlen=512)

    @staticmethod
    def key(t):
        v, q = t.transform.translation, t.transform.rotation
        return (t.header.stamp.sec, t.header.stamp.nanosec,
                t.header.frame_id, t.child_frame_id, v.x, v.y, v.z, q.x, q.y, q.z, q.w)

    def expect(self, t):
        self.pending.append(self.key(t))

    def consume(self, t):
        try:
            self.pending.remove(self.key(t))
            return True
        except ValueError:
            return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distance-scale',type=float,default=0.)
    p.add_argument('--turn-scale',type=float,default=0.)
    p.add_argument('--turn-bias-rad-per-m',type=float,default=0.)
    p.add_argument('--growth-sec',type=float,default=20.,help='Fixed simulation seconds of error growth per manual start')
    p.add_argument('--burst-profile',choices=['small','medium','large'],help='1 sim-second pose burst: lateral/yaw = .10m/3deg, .30m/10deg, .75m/20deg')
    p.add_argument('--burst-sec',type=float,default=1.)
    p.add_argument('--burst-sign',type=int,choices=[-1,1],default=1,help='Positive is left/CCW in the start estimated body frame')
    p.add_argument('--log',required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':p.error('Simulation Domain 20 required')
    model=OdomError(a.distance_scale,a.turn_scale,a.turn_bias_rad_per_m)
    burst=None
    if a.burst_profile:
        if any(model.gains):p.error('Burst profiles cannot be combined with gain flags; test factors separately')
        lateral,yaw={'small':(.10,3.),'medium':(.30,10.),'large':(.75,20.)}[a.burst_profile]
        model=OdomBurst(0.,a.burst_sign*lateral,math.radians(a.burst_sign*yaw),a.burst_sec)
        burst=dict(profile=a.burst_profile,duration_sim_sec=a.burst_sec,
                   target_body_delta=model.target,frame='estimated_body_at_start',envelope='smoothstep')
    if not math.isfinite(a.growth_sec) or a.growth_sec<=0:p.error('--growth-sec must be positive and finite')
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster
    rclpy.init();n=Node('b1_odom_injector');out=Path(a.log);out.parent.mkdir(parents=True,exist_ok=True);f=out.open('x',buffering=1)
    pub=n.create_publisher(Odometry,'/wheel_odom',10);status=n.create_publisher(String,'/b1/odom_injection_status',10);tf=n.create_publisher(TFMessage,'/tf',10)
    state={'mode':None,'mode_at':0.,'speed':None,'input_at':0.,'blocked':None,'run':0,'last_tf':None,'startup_checked':False,'start_stamp':None,'duplicates':0}
    echoes = TransformEchoes()
    def emit(event,**kw):
        d=dict(event=event,wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),
            sim_sec=model.previous[1] if model.previous else None,run=state['run'],active=model.active,
            gains=model.gains,growth_sim_sec=a.burst_sec if burst else a.growth_sec,start_sim_sec=state['start_stamp'],
            blocked=state['blocked'],startup_checked=state['startup_checked'],
            duplicate_samples=state['duplicates'],burst=burst,
            burst_fraction=model.fraction if burst else None)
        d.update(kw)
        f.write(json.dumps(d)+'\n');status.publish(String(data=json.dumps(d)))
    def command(m):
        cmd=m.data.strip()
        if cmd=='stop':model.active=False;emit('growth_stopped',run=state['run'],residual_retained=True);return
        if cmd!='start':emit('command_rejected',reason='Use start/stop; reset requires stopped restart');return
        if state['blocked'] or model.active or time.monotonic()-state['input_at']>2 or state['mode']!='AUTO' or time.monotonic()-state['mode_at']>2:
            emit('command_rejected',reason='Fresh isolated input/AUTO required, no active run');return
        if model.previous is None or not state['startup_checked']:
            emit('command_rejected',reason='No initialized odometry');return
        state['run']+=1
        if burst:model.start()
        else:model.active=True
        state['start_stamp']=model.previous[1]
        emit('injection_started',run=state['run'],gains=model.gains)
    def foreign_tf(m):
        # Humble callbacks carry only the message. Account for each own echo
        # once, including delayed echoes. Identical extra publications also
        # block: they consume the token and leave the later duplicate unmatched.
        for t in m.transforms:
            if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint':
                if echoes.consume(t):continue
                if state['startup_checked']:
                    state['blocked']='conflicting odom TF';emit('blocked',reason=state['blocked'])
    def receive(m):
        state['input_at']=time.monotonic();v=m.twist.twist
        state['speed']=abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)
        if not state['startup_checked']:
            if state['mode']!='MANUAL' or state['speed']>.01:return
            if n.count_publishers('/wheel_odom')!=1:
                state['blocked']='another wheel_odom publisher';emit('blocked',reason=state['blocked']);return
            state['startup_checked']=True
        if n.count_publishers('/wheel_odom')!=1:state['blocked']='multiple wheel_odom authorities'
        if state['blocked']:return
        q=m.pose.pose.orientation;raw=(m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        if repeated_sample(model.previous,raw,stamp):
            state['duplicates']+=1
            return
        if not burst and model.active and stamp-state['start_stamp']>=a.growth_sec:
            model.active=False
            emit('growth_stopped',sim_sec=stamp,residual_retained=True,reason='fixed_sim_duration_elapsed')
        was_active=model.active
        try:pose,twist=model.update(raw,stamp)
        except ValueError as e:
            state['blocked']=str(e);emit('blocked',reason=str(e),
                input_stamp_sec=stamp,previous_sample=model.previous,input_pose=raw);return
        if burst and was_active and not model.active:
            emit('growth_stopped',residual_retained=True,reason='burst_complete')
        import copy
        msg=copy.deepcopy(m);msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.position.x,msg.pose.pose.position.y=pose[:2]
        msg.pose.pose.orientation.x=msg.pose.pose.orientation.y=0.
        msg.pose.pose.orientation.z=math.sin(pose[2]/2);msg.pose.pose.orientation.w=math.cos(pose[2]/2)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z=twist
        t=TransformStamped();t.header=msg.header;t.child_frame_id=msg.child_frame_id
        t.transform.translation.x,t.transform.translation.y=pose[:2];t.transform.rotation=msg.pose.pose.orientation
        state['last_tf']=t;echoes.expect(t);pub.publish(msg);tf.publish(TFMessage(transforms=[t]))
        emit('odom',sim_sec=stamp,run=state['run'],active=model.active,raw_pose=raw,output_pose=pose,
             position_error_m=math.dist(raw[:2],pose[:2]),yaw_error_rad=math.atan2(math.sin(pose[2]-raw[2]),math.cos(pose[2]-raw[2])),
             accumulated_distance_m=model.distance,accumulated_turn_rad=model.turn,
             covariance_policy='source covariance preserved; does not encode injected systematic bias')
    n.create_subscription(String,'/robot_mode',lambda m:state.update(mode=m.data,mode_at=time.monotonic()),10)
    n.create_subscription(String,'/b1/odom_injection_command',command,10)
    n.create_subscription(Odometry,'/b1/physics_odom',receive,10)
    n.create_subscription(TFMessage,'/tf',foreign_tf,10)
    # Keep startup failures visible to late subscribers, even if odometry stops.
    n.create_timer(1.,lambda:emit('blocked' if state['blocked'] else 'status',
        reason=state['blocked'],mode=state['mode'],
        input_age_wall_sec=time.monotonic()-state['input_at'] if state['input_at'] else None))
    emit('ready',gains=model.gains,requires_isaac_odom_relay=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        f.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
