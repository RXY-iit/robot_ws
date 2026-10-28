#!/usr/bin/env python3
"""Read-only, bounded COR/TOR/idle evidence capture (no motion commands)."""
import argparse,json,time,hashlib,math
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=1800);a=p.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict
    from std_msgs.msg import String,Float32
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as NavPath
    from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
    from rosgraph_msgs.msg import Clock
    from action_msgs.msg import GoalStatusArray
    from nav2_msgs.msg import BehaviorTreeLog
    from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
    rclpy.init();n=rclpy.create_node('b1_trial_observer');sim=[None];last={};counts={};wheel={}
    output=Path(a.output);output.parent.mkdir(parents=True,exist_ok=True);f=output.open('x',buffering=1)
    def record(event,data,source=None):
        f.write(json.dumps(dict(event=event,wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),sim_sec=sim[0],source_stamp_sec=source,data=data),ensure_ascii=False)+'\n');counts[event]=counts.get(event,0)+1
    def receive(topic,m):
        now=time.monotonic()
        period = .1 if topic in ['/wheel_odom','/fast_lio/odometry','/cmd_vel','/cmd_vel_raw','/nav2/cmd_vel'] else (.5 if topic=='/navigate_to_pose/_action/feedback' else (1. if topic=='/plan' else 0.))
        if period and now-last.get(topic,0)<period:return
        last[topic]=now
        if topic=='/wheel_odom':
            wheel.update(x=m.pose.pose.position.x,y=m.pose.pose.position.y,frame=m.header.frame_id,at=now)
        data=message_to_ordereddict(m)
        if isinstance(m,String):
            try:data=json.loads(m.data)
            except ValueError:data={'raw':m.data}
        h=getattr(m,'header',None);stamp=h.stamp.sec+h.stamp.nanosec/1e9 if h else None
        record(topic,data,stamp)
    def clock(m):sim[0]=m.clock.sec+m.clock.nanosec/1e9;last['/clock']=time.monotonic()
    n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
    for topic in ['/robot_mode','/motion_policy/status','/safety_status','/agent/health_state','/b1/readonly_health_state','/agent/health_dashboard_state','/agent/recovery_state','/agent/mission_state','/agent/mission_events','/agent/recovery_events','/agent/recovery_response','/gicp_loc/accepted_state','/gicp_loc/recovery_status','/b1/lidar_injection_status','/b1/lidar_injection_command']:
        n.create_subscription(String,topic,lambda m,t=topic:receive(t,m),100)
    for topic,cls in [('/wheel_odom',Odometry),('/fast_lio/odometry',Odometry),('/cmd_vel',Twist),('/cmd_vel_raw',Twist),('/nav2/cmd_vel',Twist),('/gicp_loc/score',Float32),('/initialpose',PoseWithCovarianceStamped),('/navigate_to_pose/_action/status',GoalStatusArray),('/navigate_to_pose/_action/feedback',NavigateToPose_FeedbackMessage),('/behavior_tree_log',BehaviorTreeLog),('/plan',NavPath)]:
        n.create_subscription(cls,topic,lambda m,t=topic:receive(t,m),qos_profile_sensor_data)
    def costmap(m):
        now=time.monotonic()
        if now-last.get('/local_costmap/costmap',0)<1:return
        last['/local_costmap/costmap']=now
        d={'frame_id':m.header.frame_id,'wheel_age_wall_sec':now-wheel.get('at',0),
           'metric':'nearest lethal cell centre to wheel-odom base centre; not physical truth',
           'nearest_lethal_cell_m':None}
        if wheel.get('frame')==m.header.frame_id and d['wheel_age_wall_sec']<1:
            import numpy as np
            cells=np.asarray(m.data).reshape(m.info.height,m.info.width)
            ys,xs=np.where(cells>=99)
            q=m.info.origin.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            dx=(xs+.5)*m.info.resolution;dy=(ys+.5)*m.info.resolution
            x=m.info.origin.position.x+math.cos(yaw)*dx-math.sin(yaw)*dy
            y=m.info.origin.position.y+math.sin(yaw)*dx+math.cos(yaw)*dy
            if len(xs):d['nearest_lethal_cell_m']=float(np.min(np.hypot(x-wheel['x'],y-wheel['y'])))
            d.update(lethal_cells=int(len(xs)),resolution_m=m.info.resolution,base_xy=[wheel['x'],wheel['y']])
        record('/b1/local_costmap_proximity',d,m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    n.create_subscription(OccupancyGrid,'/local_costmap/costmap',costmap,qos_profile_sensor_data)
    record('capture_started',{'recorder_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'duration_wall_sec':a.duration,'fast_lio_max_hz':10,'wheel_odom_max_hz':10,'nav_feedback_max_hz':2,'plan_max_hz':1,'bt_and_health_unthrottled':True,'truth_note':'wheel_odom is not independently calibrated map truth'})
    started=time.monotonic();tick=started
    try:
        while rclpy.ok() and time.monotonic()-started<a.duration:
            rclpy.spin_once(n,timeout_sec=.1)
            if time.monotonic()-tick>=5:
                tick=time.monotonic();state={'wall_unix_sec':time.time(),'sim_sec':sim[0],'counts':counts,'ages':{k:tick-v for k,v in last.items()}}
                output.with_suffix('.status.json').write_text(json.dumps(state));record('capture_heartbeat',state)
    except KeyboardInterrupt:pass
    finally:
        record('capture_finished',{'counts':counts});f.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
