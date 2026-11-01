"""Manual controls must verify recording, not implement robot recovery policy."""
import copy
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools/sim'))
from control_gicp_injection import recording_ready


def test_recording_requires_fresh_written_clock_and_physics_but_not_healthy_robot():
    data=dict(recording=True,counts={'/clock':20,'/b1/ground_truth_odom':10},
              ages={'/clock':.1,'/b1/ground_truth_odom':.2},health='BAD')
    assert recording_ready(data,99,100)
    assert not recording_ready(data,95,100)
    data['ages']['/b1/ground_truth_odom']=8
    assert not recording_ready(data,99,100)
    data['ages']['/b1/ground_truth_odom']=.2;data['counts']['/b1/ground_truth_odom']=0
    assert not recording_ready(data,99,100)
    assert not recording_ready({},0,100)


@pytest.mark.parametrize('burst',[False,True])
def test_odom_manual_start_fixed_sim_duration_retains_error_without_navigation_commands(monkeypatch,tmp_path,burst):
    import rclpy
    import rclpy.node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    import inject_odom_error
    subscribers={};published={}
    class Node:
        def __init__(self,*args):pass
        def create_publisher(self,cls,topic,qos):
            published[topic]=[]
            return SimpleNamespace(publish=lambda m:published[topic].append(copy.deepcopy(m)))
        def create_subscription(self,cls,topic,callback,qos):subscribers[topic]=callback
        def count_publishers(self,topic):return 1
        def create_timer(self,*args):pass
        def destroy_node(self):pass
    def odom(stamp,yaw):
        m=Odometry();m.header.stamp.sec=stamp;m.header.frame_id='odom';m.child_frame_id='base_footprint'
        m.pose.pose.orientation.z=math.sin(yaw/2);m.pose.pose.orientation.w=math.cos(yaw/2)
        m.twist.twist.angular.z=.2 if stamp else 0.
        return m
    def spin(node):
        subscribers['/robot_mode'](String(data='MANUAL'))
        subscribers['/b1/physics_odom'](odom(0,0.))
        subscribers['/robot_mode'](String(data='AUTO'))
        subscribers['/b1/odom_injection_command'](String(data='start'))
        subscribers['/b1/physics_odom'](odom(1,.2))
        # No task event or health-driven intervention; sim source time alone stops growth.
        subscribers['/b1/physics_odom'](odom(2,.4))
        subscribers['/b1/physics_odom'](odom(3,.6))
        subscribers['/b1/odom_injection_command'](String(data='stop'))
    monkeypatch.setattr(rclpy,'init',lambda:None)
    monkeypatch.setattr(rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(rclpy,'spin',spin)
    monkeypatch.setattr(rclpy.node,'Node',Node)
    monkeypatch.setenv('ROS_DOMAIN_ID','20');monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_fastrtps_cpp')
    logfile=tmp_path/'odom.jsonl'
    flags=['--burst-profile','medium','--burst-sec','2'] if burst else ['--turn-scale','0.1','--growth-sec','2']
    monkeypatch.setattr(sys,'argv',['inject_odom_error.py',*flags,'--log',str(logfile)])
    inject_odom_error.main()
    records=[json.loads(line) for line in logfile.read_text().splitlines()]
    values=[row for row in records if row['event']=='odom']
    assert [v['active'] for v in values]==[False,True,False,False]
    assert values[1]['yaw_error_rad']==pytest.approx(math.radians(5) if burst else .02)
    assert values[-1]['yaw_error_rad']==pytest.approx(math.radians(10) if burst else .02)
    if burst:assert values[-1]['position_error_m']==pytest.approx(.3)
    stops=[r for r in records if r['event']=='growth_stopped' and r.get('reason')==('burst_complete' if burst else 'fixed_sim_duration_elapsed')]
    assert len(stops)==1 and stops[0]['sim_sec']==2 and stops[0]['residual_retained']
    assert records[-1]['active'] is False
    assert set(published)=={'/wheel_odom','/tf','/b1/odom_injection_status'}
    assert all(r['gains']==([0.,0.,0.] if burst else [0.,.1,0.]) for r in records)
