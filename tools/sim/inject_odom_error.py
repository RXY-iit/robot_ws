#!/usr/bin/env python3
"""Sim isolated physics-odom -> wheel_odom + sole odom TF; no live default replacement.
Requires Isaac B1_ODOM_RELAY=1 setup. start/stop control error growth; stop
NEVER clears accumulated drift. Restart/re-anchor only while MANUAL stationary.
"""
import argparse,json,math,os,time
from pathlib import Path
from b1_fault_models import OdomError


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--distance-scale',type=float,default=0.)
    p.add_argument('--turn-scale',type=float,default=0.)
    p.add_argument('--turn-bias-rad-per-m',type=float,default=0.)
    p.add_argument('--log',required=True);a=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':p.error('Simulation Domain 20 required')
    model=OdomError(a.distance_scale,a.turn_scale,a.turn_bias_rad_per_m)
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster
    rclpy.init();n=Node('b1_odom_injector');out=Path(a.log);out.parent.mkdir(parents=True,exist_ok=True);f=out.open('x',buffering=1)
    pub=n.create_publisher(Odometry,'/wheel_odom',10);status=n.create_publisher(String,'/b1/odom_injection_status',10);tf=n.create_publisher(TFMessage,'/tf',10)
    state={'mode':None,'mode_at':0.,'speed':None,'input_at':0.,'blocked':None,'run':0,'last_tf':None,'startup_checked':False}
    def emit(event,**kw):
        d=dict(event=event,wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),**kw)
        f.write(json.dumps(d)+'\n');status.publish(String(data=json.dumps(d)))
    def command(m):
        cmd=m.data.strip()
        if cmd=='stop':model.active=False;emit('growth_stopped',run=state['run'],residual_retained=True);return
        if cmd!='start':emit('command_rejected',reason='Use start/stop; reset requires stopped restart');return
        if state['blocked'] or model.active or time.monotonic()-state['input_at']>2 or state['mode']!='AUTO' or time.monotonic()-state['mode_at']>2:
            emit('command_rejected',reason='Fresh isolated input/AUTO required, no active run');return
        state['run']+=1;model.active=True;emit('injection_started',run=state['run'],gains=model.gains)
    def foreign_tf(m, info):
        own = [bytes(p.endpoint_gid) for p in n.get_publishers_info_by_topic('/tf')
               if p.node_name == n.get_name() and p.node_namespace == n.get_namespace()]
        if bytes(info.publisher_gid) in own:return
        # Compare against the last frame we broadcast; a conflicting authority
        # makes the experiment invalid, never silently fight another publisher.
        for t in m.transforms:
            if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint':
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
        try:pose,twist=model.update(raw,stamp)
        except ValueError as e:
            state['blocked']=str(e);emit('blocked',reason=str(e));return
        import copy
        msg=copy.deepcopy(m);msg.header.frame_id='odom';msg.child_frame_id='base_footprint'
        msg.pose.pose.position.x,msg.pose.pose.position.y=pose[:2]
        msg.pose.pose.orientation.x=msg.pose.pose.orientation.y=0.
        msg.pose.pose.orientation.z=math.sin(pose[2]/2);msg.pose.pose.orientation.w=math.cos(pose[2]/2)
        msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z=twist
        t=TransformStamped();t.header=msg.header;t.child_frame_id=msg.child_frame_id
        t.transform.translation.x,t.transform.translation.y=pose[:2];t.transform.rotation=msg.pose.pose.orientation
        state['last_tf']=t;pub.publish(msg);tf.publish(TFMessage(transforms=[t]))
        emit('odom',sim_sec=stamp,run=state['run'],active=model.active,raw_pose=raw,output_pose=pose,
             position_error_m=math.dist(raw[:2],pose[:2]),yaw_error_rad=math.atan2(math.sin(pose[2]-raw[2]),math.cos(pose[2]-raw[2])),
             accumulated_distance_m=model.distance,accumulated_turn_rad=model.turn,
             covariance_policy='source covariance preserved; does not encode injected systematic bias')
    n.create_subscription(String,'/robot_mode',lambda m:state.update(mode=m.data,mode_at=time.monotonic()),10)
    n.create_subscription(String,'/b1/odom_injection_command',command,10)
    n.create_subscription(Odometry,'/b1/physics_odom',receive,10)
    n.create_subscription(TFMessage,'/tf',foreign_tf,10)
    emit('ready',gains=model.gains,requires_isaac_odom_relay=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:f.close();n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
