#!/usr/bin/env python3
"""Sim-only initial-estimate perturbation; preview unless --execute.
Not physical kidnap or LiDAR degradation. May invalidate trusted anchors.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path
from urllib.request import ProxyHandler, build_opener

def check_motion_gate(mode, mode_age, odom_age, recent_speeds, *, during_navigation=False, restore=False):
    if restore and during_navigation:
        raise ValueError("--restore cannot be combined with --during-navigation")
    expected = "AUTO" if during_navigation else "MANUAL"
    if mode != expected or mode_age >= 2:
        raise ValueError(f"Fresh {expected} required" + ("" if during_navigation else "; for moving AUTO injection use --during-navigation"))
    if not recent_speeds or odom_age >= 1 or not all(math.isfinite(v) for v in recent_speeds):
        raise ValueError("Fresh finite wheel odometry required")
    if not during_navigation and max(recent_speeds) >= .01:
        raise ValueError("Stationary wheel odometry required; moving injection needs --during-navigation")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dx',type=float,default=1.0)
    p.add_argument('--dy',type=float,default=0.0)
    p.add_argument('--dyaw-deg',type=float,default=15.0)
    p.add_argument('--restore',type=Path,help='Restore pre-injection pose from a previous output (only if robot has not moved)')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--during-navigation',action='store_true',help='Allow one perturbation in AUTO while moving; sim only, incompatible with --restore')
    a=p.parse_args()
    if a.restore and a.during_navigation:p.error('--restore cannot be combined with --during-navigation')
    if not all(math.isfinite(v) for v in (a.dx,a.dy,a.dyaw_deg)):p.error('Offsets must be finite')
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':
        p.error('Requires ROS_DOMAIN_ID=20 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8765/api/config',timeout=3) as f:c=json.load(f)
    if c.get('stack_mode')!='sim' or str(c.get('stack_domain_id'))!='20':p.error('Sim Dashboard required')
    import rclpy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init();n=rclpy.create_node('manual_initialpose_injector');data={};clock=[];stationary=[]
    def accept(m):data['accepted']=(json.loads(m.data),time.monotonic())
    def mission(m):
        try:data['mission']=(json.loads(m.data),time.monotonic())
        except (TypeError,ValueError):pass
    def odom(m):
        v=m.twist.twist
        stationary.append((time.monotonic(),max(abs(v.linear.x),abs(v.linear.y),abs(v.angular.z))))
        data['odom']=m
    n.create_subscription(String,'/gicp_loc/accepted_state',accept,10)
    n.create_subscription(String,'/agent/mission_state',mission,10)
    n.create_subscription(String,'/robot_mode',lambda m:data.update(mode=(m.data,time.monotonic())),10)
    n.create_subscription(Odometry,'/wheel_odom',odom,10)
    n.create_subscription(Clock,'/clock',lambda m:clock.append((m.clock,time.monotonic())),10)
    pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    try:
        deadline=time.monotonic()+6
        while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
        now=time.monotonic()
        recent=[v for t,v in stationary if now-t<2]
        mode,mode_received=data.get('mode',('',0))
        check_motion_gate(mode,now-mode_received,now-stationary[-1][0] if stationary else float('inf'),
                          recent,during_navigation=a.during_navigation,restore=bool(a.restore))
        assert len(clock)>1 and now-clock[-1][1]<1,'Fresh sim clock required'
        stamp=lambda m:m.sec+m.nanosec/1e9
        assert stamp(clock[-1][0])>stamp(clock[0][0]),'Isaac must be Playing'
        if a.restore:
            old=json.loads(a.restore.read_text());before=old['before_pose'];target=before
            o=data['odom'].pose.pose;p0=old['odom_pose']
            assert math.hypot(o.position.x-p0[0],o.position.y-p0[1])<.05,'Robot moved; use RViz correct pose instead'
            yaw=2*math.atan2(o.orientation.z,o.orientation.w)
            assert abs(math.atan2(math.sin(yaw-p0[2]),math.cos(yaw-p0[2])))<.05,'Robot rotated; use RViz correct pose instead'
        else:
            state,received=data.get('accepted',({},0))
            assert now-received<3 and state.get('map_frame')=='map','Fresh accepted map pose required'
            before=state['map_base_pose'];target=[before[0]+a.dx,before[1]+a.dy,before[2]+math.radians(a.dyaw_deg)]
        msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=clock[-1][0]
        msg.pose.pose.position.x=float(target[0]);msg.pose.pose.position.y=float(target[1])
        msg.pose.pose.orientation.z=math.sin(target[2]/2);msg.pose.pose.orientation.w=math.cos(target[2]/2)
        msg.pose.covariance[0]=.25;msg.pose.covariance[7]=.25;msg.pose.covariance[35]=math.radians(15)**2
        o=data['odom'].pose.pose
        record={'wall_unix_sec':time.time(),'sim_sec':stamp(clock[-1][0]),'kind':'restore_initialpose' if a.restore else 'initial_estimate_perturbation','before_pose':before,'target_pose':target,'odom_pose':[o.position.x,o.position.y,2*math.atan2(o.orientation.z,o.orientation.w)],'execute_requested':a.execute,'publish_called':False}
        mission_state,mission_received=data.get('mission',({},None))
        velocity=data['odom'].twist.twist
        record.update(during_navigation=a.during_navigation,robot_mode=mode,
            mission_id=mission_state.get('mission_id'),mission_status=mission_state.get('status'),
            mission_receive_age_sec=now-mission_received if mission_received is not None else None,
            velocity={'linear_x':velocity.linear.x,'linear_y':velocity.linear.y,'angular_z':velocity.angular.z},
            accepted_state_stamp_sec=data.get('accepted',({},0))[0].get('stamp_sec'),
            accepted_state_receive_age_sec=now-data['accepted'][1] if 'accepted' in data else None,
            offset_basis='latest accepted map_base_pose (not propagated to current odometry time)')
        out=Path('debug-output/manual-injection')/f'initialpose-{time.time_ns()}.json';out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(record,indent=2))
        if a.execute:
            assert pub.get_subscription_count()>0,'No initialpose subscriber'
            pub.publish(msg);record['publish_called']=True;out.write_text(json.dumps(record,indent=2))
            end=time.monotonic()+1
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
        print(json.dumps(record,indent=2));print('Evidence:',out)
        print('Publish is not proof of fault or recovery. Observe accepted_state / Health / Recovery.')
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
