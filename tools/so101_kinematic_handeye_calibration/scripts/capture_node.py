#!/usr/bin/env python3
"""Read-only synchronized evidence capture for SO101 kinematics and hand-eye calibration."""
from __future__ import annotations
from collections import deque
from datetime import datetime, timezone
import json, math, os, platform, re, shutil, subprocess, time
from pathlib import Path
from typing import Any
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

JOINTS = ('shoulder_pan_joint','shoulder_lift_joint','elbow_flex_joint',
          'wrist_flex_joint','wrist_roll_joint','gripper_joint')
MOTORS = ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
def utc_now(): return datetime.now(timezone.utc).isoformat()
def stamp(msg): return float(msg.header.stamp.sec)+float(msg.header.stamp.nanosec)*1e-9
def safe(value): return re.sub(r'[^A-Za-z0-9_.-]+','_',value.strip()).strip('._')[:64]
def info_dict(msg):
    return {'stamp_sec':stamp(msg),'frame_id':msg.header.frame_id,'width':int(msg.width),
            'height':int(msg.height),'distortion_model':msg.distortion_model,
            'd':list(map(float,msg.d)),'k':list(map(float,msg.k)),
            'r':list(map(float,msg.r)),'p':list(map(float,msg.p))}

class Recorder(Node):
    def __init__(self):
        super().__init__('so101_kinematic_handeye_capture')
        root = str(self.declare_parameter('output_root','').value).strip()
        if not root: raise RuntimeError('output_root must be set')
        self.root=Path(root).expanduser().resolve(); self.root.mkdir(parents=True,exist_ok=True)
        self.sx=int(self.declare_parameter('squares_x',7).value)
        self.sy=int(self.declare_parameter('squares_y',5).value)
        self.square=float(self.declare_parameter('square_length_m',0.0338).value)
        self.marker=float(self.declare_parameter('marker_length_m',0.0165).value)
        self.min_corners=int(self.declare_parameter('min_charuco_corners',8).value)
        self.max_age=float(self.declare_parameter('max_age_sec',0.6).value)
        self.window=float(self.declare_parameter('stability_window_sec',1.0).value)
        self.max_span=float(self.declare_parameter('joint_max_span_rad',0.008).value)
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.board=cv2.aruco.CharucoBoard((self.sx,self.sy),self.square,self.marker,dictionary)
        self.detector=cv2.aruco.CharucoDetector(self.board); self.bridge=CvBridge()
        self.image=None; self.camera_info=None; self.joints=None; self.hardware=None
        self.joint_history=deque(maxlen=300); self.raw_history=deque(maxlen=300)
        self.transforms={}; self.index=0; self.hold_until=0.0
        tag=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.session=self.root/f'session_{tag}'; self.session.mkdir()
        self._write(self.session/'session.json',self._session())
        self.pub=self.create_publisher(String,'/so101_calibration/status',10)
        self.create_subscription(String,'/so101_calibration/capture_request',self.capture,10)
        self.create_subscription(Image,'/arm_camera/color/image_raw',self.image_cb,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/arm_camera/color/camera_info',self.info_cb,qos_profile_sensor_data)
        self.create_subscription(JointState,'/joint_states',self.joint_cb,qos_profile_sensor_data)
        self.create_subscription(String,'/arm/hardware_status',self.hardware_cb,10)
        self.create_subscription(TFMessage,'/tf',self.tf_cb,50)
        static=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage,'/tf_static',self.tf_cb,static)
        self.create_timer(0.5,self.readiness)
        self.get_logger().info(f'Read-only output: {self.session}')

    def _session(self):
        head=None
        try: head=subprocess.run(['git','rev-parse','HEAD'],cwd='/home/matsunaga-h/robot_ws',capture_output=True,text=True,timeout=2,check=True).stdout.strip()
        except Exception: pass
        return {'schema_version':1,'created_at_utc':utc_now(),
          'purpose':'SO101 joint zero/kinematic parameters and eye-in-hand calibration evidence',
          'host':platform.node(),'ros_domain_id':os.getenv('ROS_DOMAIN_ID'),'workspace_git_head':head,
          'board':{'type':'ChArUco','squares_x':self.sx,'squares_y':self.sy,
                   'square_length_m':self.square,'marker_length_m':self.marker,'dictionary':'DICT_5X5_100'},
          'sources':{'image':'/arm_camera/color/image_raw','camera_info':'/arm_camera/color/camera_info',
                     'joint_angles':'/joint_states','raw_encoder_ticks':'/arm/hardware_status','tf':['/tf','/tf_static']},
          'truth_policy':['No command publisher is created by this recorder.',
            'Board setup ID denotes one physically fixed board pose; change it whenever the board moves.',
            'Joint radians and direct raw positions are both retained; neither is inferred from the other.']}
    def image_cb(self,msg): self.image=(msg,time.monotonic())
    def info_cb(self,msg): self.camera_info=(msg,time.monotonic())
    def joint_cb(self,msg):
        now=time.monotonic(); values={n:float(v) for n,v in zip(msg.name,msg.position)}
        self.joints=(msg,values,now)
        if all(n in values and math.isfinite(values[n]) for n in JOINTS): self.joint_history.append((now,values))
    def hardware_cb(self,msg):
        now=time.monotonic(); parsed=None
        try:
            candidate=json.loads(msg.data)
            if isinstance(candidate,dict): parsed=candidate
        except Exception: pass
        self.hardware=(msg.data,parsed,now)
        if parsed and isinstance(parsed.get('raw_position'),dict):
            raw=parsed['raw_position']
            if all(m in raw for m in MOTORS): self.raw_history.append((now,{m:float(raw[m]) for m in MOTORS}))
    def tf_cb(self,msg):
        for t in msg.transforms:
            key=f'{t.header.frame_id}->{t.child_frame_id}'
            self.transforms[key]={'stamp_sec':stamp(t),'parent':t.header.frame_id,'child':t.child_frame_id,
              'translation_m':[float(t.transform.translation.x),float(t.transform.translation.y),float(t.transform.translation.z)],
              'rotation_xyzw':[float(t.transform.rotation.x),float(t.transform.rotation.y),float(t.transform.rotation.z),float(t.transform.rotation.w)]}
    def stability(self,now):
        recent=[x for x in self.joint_history if now-x[0] <= self.window+0.3]
        coverage=0.0 if len(recent)<2 else recent[-1][0]-recent[0][0]
        spans={n:(max(x[1][n] for x in recent)-min(x[1][n] for x in recent)) for n in JOINTS} if recent else {}
        return {'stable':len(recent)>=2 and coverage>=self.window-0.05 and all(v<=self.max_span for v in spans.values()),
                'coverage_sec':coverage,'samples':len(recent),'span_rad':spans,'max_span_rad':self.max_span}
    def evidence(self):
        now=time.monotonic(); errors=[]
        for name,value in [('arm image',self.image),('CameraInfo',self.camera_info),('/joint_states',self.joints),('/arm/hardware_status',self.hardware)]:
            if value is None: errors.append(f'no {name}')
            elif now-value[-1]>self.max_age: errors.append(f'{name} stale {now-value[-1]:.2f}s')
        stable=self.stability(now)
        if not stable['stable']: errors.append(f'joints not stable coverage={stable["coverage_sec"]:.2f}s')
        raw=None
        if self.hardware and self.hardware[1]: raw=self.hardware[1].get('raw_position')
        if not isinstance(raw,dict): errors.append('hardware_status has no raw_position')
        else:
            missing=[m for m in MOTORS if m not in raw]
            if missing: errors.append('raw_position missing '+','.join(missing))
        corners=ids=mc=mi=None; image=None
        if self.image:
            try:
                image=self.bridge.imgmsg_to_cv2(self.image[0],desired_encoding='bgr8')
                corners,ids,mc,mi=self.detector.detectBoard(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
                count=0 if ids is None else len(ids)
                if count<self.min_corners: errors.append(f'ChArUco corners={count}; need {self.min_corners}')
            except Exception as exc: errors.append(f'ChArUco detection failed: {exc}')
        return now,stable,raw,image,corners,ids,mc,mi,errors
    def readiness(self):
        if time.monotonic()<self.hold_until:return
        *_,errors=self.evidence()
        self.status('WAITING: '+'; '.join(errors[:4]) if errors else 'READY: arm stable, raw positions complete, ChArUco visible; press Capture')
    def capture(self,request):
        parts=request.data.split('\t',1); setup=parts[0].strip(); label=parts[1].strip() if len(parts)>1 else ''
        if not setup: self.status('REJECTED: board setup ID required',4); return
        now,stable,raw,image,corners,ids,mc,mi,errors=self.evidence()
        if errors: self.status('REJECTED: '+'; '.join(errors),5); return
        self.index+=1; name=f'sample_{self.index:04d}_{safe(setup)}'+(f'_{safe(label)}' if label else '')
        partial=self.session/f'.{name}.partial'; final=self.session/name
        if partial.exists(): shutil.rmtree(partial)
        partial.mkdir()
        try:
            annotated=image.copy()
            if mi is not None: cv2.aruco.drawDetectedMarkers(annotated,mc,mi)
            cv2.aruco.drawDetectedCornersCharuco(annotated,corners,ids,(0,255,0))
            if not cv2.imwrite(str(partial/'arm_color.png'),image): raise IOError('failed writing image')
            if not cv2.imwrite(str(partial/'arm_color_charuco.png'),annotated): raise IOError('failed writing annotation')
            boardpts=np.asarray(self.board.getChessboardCorners(),dtype=float)
            flatids=np.asarray(ids,dtype=int).reshape(-1); pixels=np.asarray(corners,dtype=float).reshape(-1,2)
            charuco=[{'id':int(i),'pixel_uv':list(map(float,p)),'board_xyz_m':list(map(float,boardpts[i]))} for i,p in zip(flatids,pixels)]
            pose=self.board_pose(corners,ids,self.camera_info[0])
            traces={n:[{'seconds_before_capture':now-t,'position_rad':v[n]} for t,v in self.joint_history if now-t<=5.0] for n in JOINTS}
            rawtraces={m:[{'seconds_before_capture':now-t,'raw_position':v[m]} for t,v in self.raw_history if now-t<=5.0] for m in MOTORS}
            msg,values,received=self.joints; hwraw,hw,hwreceived=self.hardware
            metadata={'schema_version':1,'sample_id':self.index,'board_setup_id':setup,'pose_label':label,
              'captured_at_utc':utc_now(),'image':{'stamp_sec':stamp(self.image[0]),'frame_id':self.image[0].header.frame_id,
                'receive_age_sec':now-self.image[1],'width':self.image[0].width,'height':self.image[0].height},
              'camera_info':info_dict(self.camera_info[0]),'charuco':{'corners':charuco,'camera_from_board':pose},
              'joint_state':{'stamp_sec':stamp(msg),'receive_age_sec':now-received,
                 'position_rad':{n:values[n] for n in JOINTS}},'joint_stability':stable,
              'raw_position':{m:float(raw[m]) for m in MOTORS},'hardware_status_receive_age_sec':now-hwreceived,
              'hardware_status_snapshot':hw,'joint_trace_5s':traces,'raw_trace_5s':rawtraces,
              'tf_snapshot':dict(sorted(self.transforms.items())),
              'notes':['Board must remain fixed for all samples sharing board_setup_id.',
                       'camera_from_board is an observation, not a committed URDF transform.']}
            self._write(partial/'metadata.json',metadata); partial.rename(final)
            with (self.session/'manifest.jsonl').open('a',encoding='utf-8') as f:
                f.write(json.dumps(metadata,ensure_ascii=False)+'\n');f.flush();os.fsync(f.fileno())
        except Exception as exc: self.status(f'ERROR: {exc}',6); return
        self.status(f'SAVED: {name}; corners={len(ids)}; board setup={setup}',4)
    def board_pose(self,corners,ids,info):
        if len(info.k)<9 or info.k[0]<=0 or info.k[4]<=0:return {'status':'UNAVAILABLE_UNCALIBRATED_CAMERA_INFO'}
        obj,img=self.board.matchImagePoints(corners,ids)
        ok,rvec,tvec=cv2.solvePnP(obj,img,np.asarray(info.k,dtype=float).reshape(3,3),np.asarray(info.d,dtype=float))
        if not ok:return {'status':'SOLVEPNP_FAILED'}
        return {'status':'OBSERVED_SOLVEPNP','rvec':np.asarray(rvec).reshape(-1).tolist(),'tvec_m':np.asarray(tvec).reshape(-1).tolist()}
    def status(self,text,hold=0):
        self.pub.publish(String(data=text)); self.hold_until=time.monotonic()+hold
    @staticmethod
    def _write(path,data):
        with path.open('w',encoding='utf-8') as f: json.dump(data,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')

def main(args=None):
    rclpy.init(args=args); node=Recorder()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
