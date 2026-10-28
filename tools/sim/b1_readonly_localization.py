#!/usr/bin/env python3
"""TOR-only observer: same SystemSkills localization checks, no action/command clients.
Health Monitor must remap /agent/health_state to /b1/readonly_health_state.
"""
import json,math,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RosClock,ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from action_msgs.msg import GoalStatusArray
from robot_skills.executors.system import SystemSkills

class Observer(Node):
    def __init__(self):
        super().__init__('b1_readonly_localization',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.system=SystemSkills(self,'sim');self.last_clock=None
        self.wheel_velocity=None;self.goal_status=None;self.goal_status_received=0.
        self.pub=self.create_publisher(String,'/b1/readonly_health_state',10)
        self.create_subscription(String,'/robot_mode',lambda m:self.system.update_robot_mode(m.data),10)
        self.create_subscription(String,'/safety_status',lambda m:self.system.update_safety_status(m.data),10)
        self.create_subscription(String,'/gicp_loc/accepted_state',self.accepted,10)
        self.create_subscription(String,'/gicp_loc/recovery_status',self.recovery,10)
        self.create_subscription(Clock,'/clock',self.clock,10)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.system.update_cmd_vel(m.linear.x,m.linear.y,m.angular.z),10)
        self.create_subscription(Twist,'/nav2/cmd_vel',lambda m:self.system.update_nav2_cmd_vel(m.linear.x,m.linear.y,m.angular.z),10)
        self.create_subscription(Odometry,'/wheel_odom',self.odom,10)
        self.create_subscription(PoseWithCovarianceStamped,'/initialpose',lambda m:self.system.update_initialpose(source='/initialpose'),10)
        self.create_subscription(PoseWithCovarianceStamped,'/operator/initialpose',self.operator_initialpose,10)
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.status,
                                 QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.,self.publish,clock=RosClock(clock_type=ClockType.STEADY_TIME))
    def clock(self,m):
        t=m.clock.sec+m.clock.nanosec/1e9
        if self.last_clock is not None and t<self.last_clock:self.system.invalidate_localization_anchor()
        self.last_clock=t
    def recovery(self,m):
        try:
            d=json.loads(m.data);self.system._recovery_accumulating=d.get('phase')=='accumulating';self.system._recovery_status_received=time.monotonic()
        except (ValueError,TypeError,AttributeError):pass
    def odom(self,m):
        if m.header.frame_id.lstrip('/')!='odom':return
        self.wheel_velocity=(m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)
        q=m.pose.pose.orientation;p=m.pose.pose.position;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.system.update_wheel_odom_pose(p.x,p.y,yaw,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
    def status(self,m):
        self.goal_status=[s.status for s in m.status_list];self.goal_status_received=time.monotonic()
    def operator_initialpose(self,m):
        s=self.system;p=m.pose.pose.position;q=m.pose.pose.orientation
        source_age=None if s._wheel_odom_stamp is None else self.get_clock().now().nanoseconds/1e9-s._wheel_odom_stamp
        stopped=(self.wheel_velocity is not None and source_age is not None and 0<=source_age<=.75
                 and time.monotonic()-s._last_wheel_odom_time<=.75
                 and all(math.isfinite(v) and abs(v)<=.02 for v in self.wheel_velocity)
                 and self.goal_status is not None
                 and all(status in {4,5,6} for status in self.goal_status))
        valid=(m.header.frame_id.lstrip('/')=='map' and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w))
               and .99<=sum(v*v for v in (q.x,q.y,q.z,q.w))<=1.01)
        s.update_initialpose(operator_alignment=stopped and valid,source='/operator/initialpose')
    def accepted(self,m):
        try:
            d=json.loads(m.data);stamp=float(d['stamp_sec']);pose=d['map_base_pose'];anchor=d['odom_base_pose'];score=float(d['rmse_m'])
            if not isinstance(pose,list) or not isinstance(anchor,list) or len(pose)!=3 or len(anchor)!=3:return
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in pose+anchor) or not math.isfinite(score) or score<0:return
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if not 0<=age<=2 or d['base_frame']!='base_footprint' or d['map_frame']!='map' or d['odom_frame']!='odom':return
            if self.system._evidence_stamp is not None and stamp<=self.system._evidence_stamp:return
            self.system.update_gicp_score(score);self.system.update_odom_pose(*pose,stamp=stamp,odom_anchor=tuple(anchor))
        except (ValueError,KeyError,TypeError):return
    def publish(self):
        # Planner lifecycle is checked directly by Health Monitor; this observer
        # only supplies the shared localization consistency calculation.
        d=self.system.health_summary();d['source']='b1_readonly_localization';d['decision_authority']='none'
        self.pub.publish(String(data=json.dumps(d,ensure_ascii=False)))

def main():
    rclpy.init();n=Observer()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
