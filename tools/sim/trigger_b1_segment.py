#!/usr/bin/env python3
"""One episode: observed A success -> B accepted -> turn accumulation / stable straight noise.
Start before A dispatch. Recovery sees no experiment commands or labels.
"""
import argparse,json,math,os,time,hashlib
from pathlib import Path
from b1_fault_models import StableSegment, FaultSchedule


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--from-xy',type=float,nargs=2,required=True);p.add_argument('--to-xy',type=float,nargs=2,required=True)
    p.add_argument('--factors',choices=['none','L','O','M','LO','LM','OM','LOM'],required=True)
    p.add_argument('--stable-sec',type=float,default=2);p.add_argument('--distance',type=float,default=.3)
    p.add_argument('--growth-sec',type=float,default=20);p.add_argument('--timeout',type=float,default=900)
    p.add_argument('--truth-calibration',type=Path,required=True);p.add_argument('--map-manifest',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':p.error('Sim Domain 20 / FastDDS required')
    if not all(math.isfinite(v) for v in [*a.from_xy,*a.to_xy,a.stable_sec,a.distance,a.growth_sec,a.timeout]) or min(a.stable_sec,a.distance,a.growth_sec,a.timeout)<=0 or math.dist(a.from_xy,a.to_xy)<.5:p.error('Finite positive durations/distance and distinct endpoints required')
    calibration=json.loads(a.truth_calibration.read_text())
    transform=calibration['map_from_physics_se2']
    if len(transform)!=3 or not all(math.isfinite(v) for v in transform):p.error('Finite calibrated SE2 required')
    manifest=None
    if 'M' in a.factors:
        if not a.map_manifest:p.error('M needs static map manifest, loaded before task')
        manifest=json.loads(a.map_manifest.read_text())
        if hashlib.sha256(Path(manifest['output']).read_bytes()).hexdigest()!=manifest['output_sha256']:p.error('Map hash mismatch')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    rclpy.init();n=rclpy.create_node('b1_segment_trigger')
    a.output.parent.mkdir(parents=True,exist_ok=True);f=a.output.open('x',buffering=1)
    gate=StableSegment(a.from_xy,a.to_xy,a.stable_sec,a.distance)
    latest={};accepted={};state={'goal':None,'sim':None,'straight':False,'ended':False,'start_o':None,'completed_at':None,'no_straight':False}
    schedule=FaultSchedule()
    requested={};confirmed=set();t0=time.monotonic()
    pubs={k:n.create_publisher(String,'/b1/'+topic+'_injection_command',10) for k,topic in [('L','lidar'),('O','odom')]}
    def log(event,**kw):f.write(json.dumps(dict(event=event,wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),sim_sec=state['sim'],**kw))+'\n')
    def command(k,cmd):
        if pubs[k].get_subscription_count()!=1:raise RuntimeError(k+' command requires exactly one consumer')
        status=latest.get(k,(0,{}));d=status[1]
        if cmd=='start':
            if time.monotonic()-status[0]>3 or d.get('active') or d.get('event')=='blocked':raise RuntimeError(k+' unavailable or already active')
            requested[k]=(time.monotonic(),d.get('run',0))
        pubs[k].publish(String(data=json.dumps({'command':cmd,'profile':'noise'}) if k=='L' else cmd))
        log('command',factor=k,command=cmd,previous_status=d)
    def receive(k,m):
        try:d=json.loads(m.data)
        except ValueError:return
        latest[k]=(time.monotonic(),d)
        if k in requested and d.get('run',0)>requested[k][1] and d.get('active'):
            if k not in confirmed:log('injection_confirmed',factor=k,status=d);confirmed.add(k)
    def event(m):
        try:d=json.loads(m.data);xy=d['goal_xy'];kind=d['event'];gid=d['goal_id']
        except (ValueError,KeyError,TypeError):return
        log('waypoint_event',data=d)
        if d.get('frame_id')!='map':return
        if kind=='accepted':accepted[gid]=xy
        # Ignore unrelated, old or unobserved success messages.
        if kind=='succeeded' and accepted.get(gid)!=xy:return
        gate.goal(xy,kind)
        if kind=='accepted' and gate.arrival:
            if state['goal'] is not None:
                log('later_goal_accepted',goal_id=gid,decision='no repeated injection; keep precommitted fault schedule');return
            state['goal']=gid
            if 'M' in a.factors:
                at,status=latest.get('localizer',(0,{}))
                if time.monotonic()-at>3 or Path(status.get('map_path','')).resolve()!=Path(manifest['output']).resolve():raise RuntimeError('Active localization map does not match M manifest')
            if 'O' in a.factors:
                command('O','start');state['start_o']=state['sim'];schedule.start('O',state['sim'],a.growth_sec)
        if gid==state['goal'] and kind in ('succeeded','terminated'):
            log('segment_ended',result=kind);state['ended']=True
            if not state['straight']:state['no_straight']=True
    def odom(m):
        latest['odom']=(time.monotonic(),m)
        if state['straight'] or state['no_straight'] or state['sim'] is None:return
        q=m.pose.pose.orientation;pos=m.pose.pose.position;v=m.twist.twist
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        c,s=math.cos(transform[2]),math.sin(transform[2])
        xy=(transform[0]+c*pos.x-s*pos.y,transform[1]+s*pos.x+c*pos.y)
        if gate.update(xy,yaw+transform[2],math.hypot(v.linear.x,v.linear.y),v.angular.z,state['sim']):
            if time.monotonic()-latest.get('mode',(0,''))[0]>3 or latest['mode'][1]!='AUTO':return
            state['straight']=True;log('straight_observation_started',physics_pose=[pos.x,pos.y,yaw])
            if 'L' in a.factors:command('L','start')
    def clock(m):
        sim=m.clock.sec+m.clock.nanosec/1e9
        if state['sim'] is not None and sim<state['sim']:
            raise RuntimeError('Clock reset invalidates current episode')
        state.update(sim=sim,clock_at=time.monotonic())
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    n.create_subscription(String,'/robot_mode',lambda m:latest.update(mode=(time.monotonic(),m.data)),10)
    n.create_subscription(String,'/b1/waypoint_events',event,100)
    # Uncorrupted physics odom is an experiment trigger only, not Recovery input.
    n.create_subscription(Odometry,'/b1/ground_truth_odom',odom,qos_profile_sensor_data)
    for k,topic in [('L','/b1/lidar_injection_status'),('O','/b1/odom_injection_status'),('localizer','/gicp_loc/recovery_status')]:n.create_subscription(String,topic,lambda m,k=k:receive(k,m),100)
    log('armed',factors=a.factors,from_xy=a.from_xy,to_xy=a.to_xy,map_manifest=manifest,truth_calibration=calibration,growth_sec=a.growth_sec,stable_sec=a.stable_sec,distance=a.distance)
    try:
        while time.monotonic()-t0<a.timeout:
            rclpy.spin_once(n,timeout_sec=.1)
            for k,(sent,run) in requested.items():
                if k not in confirmed and time.monotonic()-sent>10:raise RuntimeError(k+' not acknowledged; not resent')
            if state['goal'] and (time.monotonic()-latest.get('odom',(t0,None))[0]>3 or time.monotonic()-state.get('clock_at',t0)>3):raise RuntimeError('Clock/physics evidence stale')
            if state['start_o'] is not None and schedule.elapsed('O',state['sim']):
                command('O','stop');state['start_o']=None;log('odom_growth_ended',residual_retained=True)
            # Navigation cancellation is an outcome, NOT a command to remove
            # the fault early for COR. Both sides keep the same sim-time schedule.
            lidar_done = ('L' not in requested or ('L' in confirmed and
                not latest.get('L',(0,{}))[1].get('active',True)))
            odom_done = 'O' not in requested or ('O' in confirmed and state['start_o'] is None)
            if (state['straight'] or state['no_straight']) and lidar_done and odom_done:
                if state['completed_at'] is None:state['completed_at']=state['sim']
                if state['sim']-state['completed_at']>=10:break
        else:raise RuntimeError('Episode timeout; cleanup is intervention, trial requires review')
        if not state['straight']:raise RuntimeError('No stable straight observation; record turn outcome, exclude from straight-segment comparison')
        log('episode_finished',confirmed=sorted(confirmed),original_goal_terminated=state['ended'],fault_schedule_preserved=True)
    finally:
        for k in requested:
            if not latest.get(k,(0,{}))[1].get('active',True):continue
            try:command(k,'stop')
            except Exception as e:log('cleanup_command_failed',factor=k,error=str(e))
        log('episode_closed',odom_residual_must_be_reset_manually='O' in requested)
        f.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
