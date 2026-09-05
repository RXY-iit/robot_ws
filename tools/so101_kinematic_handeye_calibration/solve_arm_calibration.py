#!/usr/bin/env python3
"""Constrained SO101 joint-affine and eye-in-hand solve from fixed-board groups."""
from __future__ import annotations
import argparse, glob, json, math
from pathlib import Path
import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS=("shoulder_pan_joint","shoulder_lift_joint","elbow_flex_joint","wrist_flex_joint","wrist_roll_joint")
REFERENCE=np.array([0.03217642458100545,1.74533,-0.295563703024748,-0.0373075230972284,-1.2867643467643466])
OLD_SCALE=np.array([1.07092812,0.98232268,0.97438663,1.05793015,1.37093532])
OLD_OFFSET=np.array([0.,0.,0.,0.,-0.3361107058855241])
CHAIN=(
 ((0.0388353,-8.97657e-09,0.0624),(math.pi,0.,-math.pi)),
 ((-0.0303992,-0.0182778,-0.0542),(-math.pi/2,-math.pi/2,0.)),
 ((-0.11257,-0.028,0.),(0.,0.,math.pi/2)),
 ((-0.1349,0.0052,0.),(0.,0.,-1.57)),
 ((0.,-0.0611,0.0181),(-1.57,3.14,0.)),)
OLD_EYE=np.r_[np.array([0.005322287314,0.054910286339,-0.051643416757]),
              Rotation.from_euler('xyz',[2.530848248611,0.025610528222,-0.149162742190]).as_rotvec()]

def T(xyz=(0,0,0), rotvec=None, rpy=None):
 out=np.eye(4);out[:3,3]=xyz
 if rotvec is not None:out[:3,:3]=Rotation.from_rotvec(rotvec).as_matrix()
 if rpy is not None:out[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
 return out
def Rz(a):return T(rotvec=[0,0,a])
def fk(q,scale,offset):
 values=REFERENCE+scale*(q-REFERENCE)+offset; out=np.eye(4)
 for (xyz,rpy),a in zip(CHAIN,values):out=out@T(xyz,rpy=rpy)@Rz(a)
 return out
def load(root,square,marker):
 board=cv2.aruco.CharucoBoard((7,5),square,marker,cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))
 points=np.asarray(board.getChessboardCorners(),float); result=[]
 for f in glob.glob(str(root/'session_*'/'sample_*'/'metadata.json')):
  d=json.load(open(f)); cs=d['charuco']['corners']; ids=np.array([c['id'] for c in cs]); uv=np.array([c['pixel_uv'] for c in cs],float)
  K=np.array(d['camera_info']['k'],float).reshape(3,3);D=np.array(d['camera_info']['d'],float)
  ok,r,t=cv2.solvePnP(points[ids],uv,K,D)
  if not ok:continue
  result.append({'path':f,'setup':d['board_setup_id'],'q':np.array([d['joint_state']['position_rad'][j] for j in JOINTS]),
                 'C_B':T(np.asarray(t).reshape(3),np.asarray(r).reshape(3)),'corners':len(cs)})
 return result
def unpack(x,setups):
 scale=x[:5];offset=OLD_OFFSET.copy();offset[1:4]=x[5:8];eye=T(x[8:11],x[11:14]);boards={}
 k=14
 for s in setups:boards[s]=T(x[k:k+3],x[k+3:k+6]);k+=6
 return scale,offset,eye,boards
def initial(samples,setups):
 boards=[]
 for s in setups:
  a=next(v for v in samples if v['setup']==s)
  boards.append(fk(a['q'],OLD_SCALE,OLD_OFFSET)@T(OLD_EYE[:3],OLD_EYE[3:])@a['C_B'])
 vals=list(OLD_SCALE)+list(OLD_OFFSET[1:4])+list(OLD_EYE)
 for b in boards:vals += list(b[:3,3])+list(Rotation.from_matrix(b[:3,:3]).as_rotvec())
 return np.array(vals)
def errors(x,samples,setups,priors=True):
 scale,offset,eye,boards=unpack(x,setups);out=[]
 for a in samples:
  pred=fk(a['q'],scale,offset)@eye@a['C_B']; delta=np.linalg.inv(boards[a['setup']])@pred
  out.extend((delta[:3,3]/0.004).tolist());out.extend((Rotation.from_matrix(delta[:3,:3]).as_rotvec()/math.radians(0.7)).tolist())
 if priors:
  out.extend(((scale-OLD_SCALE)/0.08).tolist())
  out.extend(((offset[1:4]-OLD_OFFSET[1:4])/math.radians(8)).tolist())
  out.extend(((eye[:3,3]-OLD_EYE[:3])/0.04).tolist())
  out.extend(((Rotation.from_matrix(eye[:3,:3]).as_rotvec()-OLD_EYE[3:])/math.radians(20)).tolist())
 return np.array(out)
def metrics(x,samples,setups):
 scale,offset,eye,boards=unpack(x,setups);te=[];re=[]
 for a in samples:
  d=np.linalg.inv(boards[a['setup']])@fk(a['q'],scale,offset)@eye@a['C_B'];te.append(np.linalg.norm(d[:3,3]));re.append(math.degrees(np.linalg.norm(Rotation.from_matrix(d[:3,:3]).as_rotvec())))
 return {'count':len(te),'translation_rms_m':float(np.sqrt(np.mean(np.square(te)))),'translation_median_m':float(np.median(te)),'translation_max_m':float(max(te)),
         'rotation_rms_deg':float(np.sqrt(np.mean(np.square(re)))),'rotation_median_deg':float(np.median(re)),'rotation_max_deg':float(max(re))}
def main():
 p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,default=Path(__file__).parent/'output');p.add_argument('--square-length-m',type=float,default=.0338);p.add_argument('--marker-length-m',type=float,default=.0165);p.add_argument('--report',type=Path,default=Path(__file__).parent/'arm_calibration_candidate.json');a=p.parse_args()
 samples=load(a.output_root,a.square_length_m,a.marker_length_m);setups=sorted(set(v['setup'] for v in samples));train=[];valid=[]
 for s in setups:
  group=[v for v in samples if v['setup']==s]
  for i,v in enumerate(group):(valid if i%5==0 else train).append(v)
 x0=initial(samples,setups);sol=least_squares(errors,x0,args=(train,setups,True),loss='soft_l1',f_scale=1,max_nfev=5000,verbose=1)
 scale,offset,eye,boards=unpack(sol.x,setups)
 report={'schema_version':1,'status':'CANDIDATE_NOT_DEPLOYED','sample_count':len(samples),'setups':setups,
   'board_dimensions_m':{'square':a.square_length_m,'marker':a.marker_length_m},'split':'every fifth within each board setup held out',
   'old_metrics_all':metrics(x0,samples,setups),'candidate_train':metrics(sol.x,train,setups),'candidate_validation':metrics(sol.x,valid,setups),'candidate_all':metrics(sol.x,samples,setups),
   'solution':{'scale':dict(zip(JOINTS,map(float,scale))),'offset_rad':dict(zip(JOINTS,map(float,offset))),
      'gripper_to_arm_camera_optical_xyz_m':eye[:3,3].tolist(),'gripper_to_arm_camera_optical_rpy_rad':Rotation.from_matrix(eye[:3,:3]).as_euler('xyz').tolist()},
   'observability':{'shoulder_pan_offset':'FIXED_GAUGE_UNOBSERVABLE_WITH_UNKNOWN_BOARD_POSES','wrist_roll_offset':'FIXED_GAUGE_COUPLED_TO_EYE_ROTATION','link_geometry':'FIXED_TO_MEASURED_URDF_TO_AVOID_OVERPARAMETERIZATION'},
   'optimizer':{'success':bool(sol.success),'message':sol.message,'cost':float(sol.cost)}}
 a.report.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
