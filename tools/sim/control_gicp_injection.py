#!/usr/bin/env python3
"""Direct ROS command with state acknowledgement; no CLI daemon or automatic retries."""
import argparse
import json
import os
import time


def recording_ready(capture, received, now):
    if now-received>3 or not capture.get('recording'):
        return False
    # Check logging, not robot health. BAD is an outcome for Recovery/Nav2.
    return all(capture.get('counts',{}).get(topic,0)>0 and capture.get('ages',{}).get(topic,float('inf'))<3
               for topic in ('/clock','/b1/ground_truth_odom'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['status','start','stop'])
    p.add_argument('--factor',choices=['lidar','odom'],default='lidar')
    p.add_argument('--profile',choices=['noise','legacy','mild','bias','occlusion','severe','shelf_drift','shelf_drift_noisy','thermal'],default='noise')
    p.add_argument('--require-healthy-motion',action='store_true',help='Only start with recent OK localization, AUTO and moving wheel odometry')
    a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':
        p.error('Set ROS_DOMAIN_ID=20 and RMW_IMPLEMENTATION=rmw_fastrtps_cpp')
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    rclpy.init();n=Node('b1_injection_control');state={};received=[0.];capture={};capture_at=[0.]
    def cb(m):
        try:d=json.loads(m.data)
        except ValueError:return
        state.clear();state.update(d);received[0]=time.monotonic()
    status_topic='/b1/'+a.factor+'_injection_status'
    command_topic='/b1/'+a.factor+'_injection_command'
    injector_name='b1_gicp_cloud_injector' if a.factor=='lidar' else 'b1_odom_injector'
    n.create_subscription(String,status_topic,cb,10)
    def capture_cb(m):
        try:d=json.loads(m.data)
        except ValueError:return
        capture.clear();capture.update(d);capture_at[0]=time.monotonic()
    n.create_subscription(String,'/b1/capture_status',capture_cb,10)
    health_samples=[];motion={};mode={}
    def health_cb(m):
        try:level=json.loads(m.data).get('components',{}).get('localization',{}).get('level')
        except ValueError:return
        health_samples.append((time.monotonic(),level))
    def odom_cb(m):
        v=m.twist.twist
        motion.update(received=time.monotonic(),speed=abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z))
    n.create_subscription(String,'/agent/health_state',health_cb,10)
    n.create_subscription(String,'/robot_mode',lambda m:mode.update(value=m.data,received=time.monotonic()),10)
    n.create_subscription(Odometry,'/wheel_odom',odom_cb,10)
    pub=n.create_publisher(String,command_topic,10)
    try:
        # Allow discovery of the actual injector, not just the recording subscriber.
        end=time.monotonic()+5
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            names=[i.node_name for i in n.get_subscriptions_info_by_topic(command_topic)]
            if (state and time.monotonic()-received[0]<3
                    and names.count(injector_name)==1 and pub.get_subscription_count()>=len(names)
                    and (a.command!='start' or recording_ready(capture,capture_at[0],time.monotonic()))):
                break
        if not state or time.monotonic()-received[0]>3:raise RuntimeError('No fresh injector status; no command sent')
        print('Before:',json.dumps(state,ensure_ascii=False),flush=True)
        if a.command=='status':return
        names=[i.node_name for i in n.get_subscriptions_info_by_topic(command_topic)]
        if names.count(injector_name)!=1 or pub.get_subscription_count()<len(names):
            raise RuntimeError('Require one discovered, matched injector; no command sent')
        output=state.get('output','/wheel_odom' if a.factor=='odom' else '')
        if not output or n.count_publishers(status_topic)!=1 or n.count_publishers(output)!=1:
            raise RuntimeError('Duplicate/missing injector publishers; no command sent')
        if a.command=='start' and state.get('active'):
            print('Already active; start not resent');return
        if a.command=='stop' and not state.get('active'):
            print('Already inactive; stop not sent');return
        if a.command=='start':
            if (n.count_publishers('/b1/capture_status')!=1
                    or not recording_ready(capture,capture_at[0],time.monotonic())):
                raise RuntimeError('Start capture_b1_trial.py first; fresh recording /clock and /b1/ground_truth_odom required. No command sent.')
            if a.factor=='lidar':
                consumer='gicp_localizer' if state.get('scope','gicp')=='gicp' else 'pointcloud2_to_livox_custom_msg_node'
                consumers=[i.node_name for i in n.get_subscriptions_info_by_topic(output)]
                if consumers.count(consumer)!=1:
                    raise RuntimeError('Relay is not connected to exactly one '+consumer+'; check B1 input remapping')
            if state.get('event')=='blocked':raise RuntimeError('Injector blocked: '+str(state.get('reason')))
            print('Recording:',capture['output'],flush=True)
        if a.command=='start' and a.require_healthy_motion:
            now=time.monotonic();recent=[(t,v) for t,v in health_samples if now-t<5]
            if (len(recent)<2 or now-recent[-1][0]>2.5 or any(v!='OK' for _,v in recent)
                    or mode.get('value')!='AUTO' or now-mode.get('received',0)>2
                    or now-motion.get('received',0)>1 or motion.get('speed',0)<.03):
                raise RuntimeError('Fresh stable OK localization, AUTO and actual motion required; no injection sent')
        run=state.get('run',0);sent=time.monotonic()
        pub.publish(String(data=json.dumps({"command":a.command,"profile":a.profile}) if a.factor=='lidar' else a.command))
        while time.monotonic()-sent<10:
            rclpy.spin_once(n,timeout_sec=.1)
            if received[0]<=sent:continue
            ok=(state.get('active') and state.get('run',0)==run+1) if a.command=='start' else not state.get('active')
            if ok:
                print('Confirmed:',json.dumps(state,ensure_ascii=False));return
        raise RuntimeError('No state acknowledgement. Command may be rejected or undelivered; inspect injection.jsonl. No automatic resend.')
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
