#!/usr/bin/env python3
"""One-shot sim fault trigger after a fixed wheel-odom displacement; shared COR/TOR."""
import argparse,json,math,os,time
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--distance',type=float,default=1.0);p.add_argument('--profile',choices=['mild','bias','occlusion','severe','shelf_drift','shelf_drift_noisy'],required=True);p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=240);p.add_argument('--near-map-xy',type=float,nargs=2,metavar=('X','Y'));p.add_argument('--radius',type=float,default=.6);a=p.parse_args()
    if not math.isfinite(a.distance) or a.distance<=0 or not math.isfinite(a.timeout) or a.timeout<=0:p.error('Positive finite distance and timeout required')
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':p.error('Sim Domain 20 / rmw_fastrtps_cpp required')
    if a.near_map_xy and (not all(math.isfinite(v) for v in a.near_map_xy) or not math.isfinite(a.radius) or a.radius<=0):p.error('Finite trigger position and positive radius required')
    import rclpy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    rclpy.init();n=rclpy.create_node('b1_distance_trigger');latest={};origin=[None]
    out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True);f=out.open('x',buffering=1)
    def log(event,**data):f.write(json.dumps(dict(event=event,wall_unix_sec=time.time(),**data))+'\n')
    def cb(key,m):
        if key=='odom':
            pos=m.pose.pose.position;v=m.twist.twist;value={'x':pos.x,'y':pos.y,'yaw_q':[m.pose.pose.orientation.z,m.pose.pose.orientation.w],'speed':abs(v.linear.x)+abs(v.linear.y)+abs(v.angular.z)}
            if origin[0] is None:origin[0]=value;log('origin',pose=value)
        elif key=='goals':value=any(s.status==2 for s in m.status_list)
        elif key=='mode':value=m.data
        else:
            try:value=json.loads(m.data)
            except ValueError:return
        latest[key]=(time.monotonic(),value)
    for key,topic,kind in [('odom','/wheel_odom',Odometry),('goals','/navigate_to_pose/_action/status',GoalStatusArray),('mode','/robot_mode',String),('accepted','/gicp_loc/accepted_state',String),('health','/agent/health_dashboard_state',String),('injection','/b1/lidar_injection_status',String)]:
        n.create_subscription(kind,topic,lambda m,k=key:cb(k,m),10)
    pub=n.create_publisher(String,'/b1/lidar_injection_command',10);start=time.monotonic();sent=None;initial_run=None
    log('armed',profile=a.profile,distance_m=a.distance,timeout_wall_sec=a.timeout,near_map_xy=a.near_map_xy,radius_m=a.radius)
    try:
        while time.monotonic()-start<a.timeout:
            rclpy.spin_once(n,timeout_sec=.1);now=time.monotonic()
            if sent is not None:
                t,d=latest.get('injection',(0,{}))
                if t>sent and d.get('run',0)>initial_run and d.get('active') and d.get('profile')==a.profile:
                    log('confirmed',status=d);print('Injection confirmed',d,flush=True);return
                if now-sent>10:raise RuntimeError('No acknowledgement; not resent')
                continue
            if not all(k in latest and now-latest[k][0]<3 for k in ['odom','mode','accepted','health','injection']):continue
            od=latest['odom'][1];dist=math.hypot(od['x']-origin[0]['x'],od['y']-origin[0]['y'])
            if a.near_map_xy:
                pose=latest['accepted'][1].get('map_base_pose',[])
                if len(pose)!=3 or math.hypot(pose[0]-a.near_map_xy[0],pose[1]-a.near_map_xy[1])>a.radius:continue
            elif dist<a.distance:continue
            health=latest['health'][1];caps=health.get('capabilities',[])
            nav=next((c for c in caps if c.get('id')=='auto_navigation'),{})
            if nav.get('status')!='AVAILABLE' or latest['mode'][1]!='AUTO' or od['speed']<.03:continue
            if not latest.get('goals',(0,False))[1]:continue
            inj=latest['injection'][1]
            if inj.get('active'):raise RuntimeError('Already injecting')
            names=[i.node_name for i in n.get_subscriptions_info_by_topic('/b1/lidar_injection_command')]
            if names.count('b1_gicp_cloud_injector')!=1 or pub.get_subscription_count()<len(names):continue
            initial_run=inj['run'];sent=now
            log('sent',displacement_m=dist,pose=od,accepted=latest['accepted'][1],health_capability=nav,previous_run=initial_run)
            pub.publish(String(data=json.dumps({'command':'start','profile':a.profile})))
        raise RuntimeError('Trigger timeout; no confirmed injection')
    finally:
        f.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
