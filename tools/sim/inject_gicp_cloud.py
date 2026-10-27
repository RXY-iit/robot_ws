#!/usr/bin/env python3
"""Sim-only GICP range-noise relay. Starts in byte-identical passthrough.
Commands on /b1/lidar_injection_command: start, stop. Durations use /clock.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import time
import numpy as np



# Exploratory calibration profiles, not validated severity labels.
PROFILES = {
    "shelf_drift_noisy": dict(noise=1., bias=0., yaw=0., sector=0., dropout=0., map_bias_y=1.5),
    "shelf_drift": dict(noise=.05, bias=0., yaw=0., sector=0., dropout=0., map_bias_y=1.5),
    "mild": dict(noise=.3, bias=0., yaw=0., sector=0., dropout=0.),
    "bias": dict(noise=.1, bias=1.5, yaw=20., sector=0., dropout=0.),
    "occlusion": dict(noise=.3, bias=0., yaw=0., sector=300., dropout=0.),
    "severe": dict(noise=.5, bias=1.5, yaw=25., sector=300., dropout=1.),
}

def strength(elapsed, ramp, hold, release, maximum):
    if elapsed < 0:return 0.0
    if elapsed < ramp:return maximum * elapsed / ramp
    if elapsed < ramp + hold:return maximum
    if elapsed < ramp + hold + release:return maximum * (1-(elapsed-ramp-hold)/release)
    return 0.0


def corrupt_cloud(msg, sigma, seed, bias=0., yaw_deg=0., sector_deg=0., metrics=None, translation=None):
    """Alter only float32 XYZ bytes; preserve layout, padding, fields and header."""
    translation = np.asarray(translation if translation is not None else [0., 0., 0.], dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("Finite XYZ translation required")
    passthrough = not np.any(translation) and sigma == 0 and bias == 0 and yaw_deg == 0 and sector_deg == 0
    if passthrough and metrics is None:return msg
    offsets={f.name:f.offset for f in msg.fields if f.datatype==7 and f.count==1}
    if not all(k in offsets for k in ('x','y','z')):raise ValueError('Requires float32 x/y/z fields')
    if msg.row_step < msg.width*msg.point_step or len(msg.data)<msg.row_step*msg.height:
        raise ValueError('Invalid cloud layout')
    if any(offsets[k]+4>msg.point_step for k in ('x','y','z')):raise ValueError('Invalid XYZ offsets')
    out=copy.deepcopy(msg);buf=bytearray(msg.data)
    views=[np.ndarray((msg.height,msg.width),dtype=('>' if msg.is_bigendian else '<')+'f4',buffer=buf,
                      offset=offsets[k],strides=(msg.row_step,msg.point_step)) for k in ('x','y','z')]
    xyz=np.stack(views,axis=-1).astype(np.float64);radius=np.linalg.norm(xyz,axis=-1)
    valid=np.isfinite(xyz).all(axis=-1)&(radius>1e-6)
    rng=np.random.default_rng(seed)
    noise=rng.normal(0,sigma,radius.shape)
    scale=np.ones_like(radius);scale[valid]=np.maximum(.01,radius[valid]+noise[valid])/radius[valid]
    changed=xyz*scale[...,None]
    angle=math.radians(yaw_deg);c,s=math.cos(angle),math.sin(angle)
    x=changed[...,0].copy();y=changed[...,1].copy()
    changed[...,0]=c*x-s*y+bias;changed[...,1]=s*x+c*y
    changed += translation
    occluded=valid & (np.abs(np.arctan2(xyz[...,1],xyz[...,0])) < math.radians(sector_deg)/2)
    if metrics is not None:
        retained = valid & ~occluded
        n, kept = int(valid.sum()), int(retained.sum())
        radial = (np.maximum(.01, radius + noise) - radius)[retained]
        delta = (changed - xyz)[retained]
        metrics.update(input_valid_points=n, occluded_valid_points=int(occluded.sum()),
            output_valid_points=kept, occluded_fraction_valid=(float(occluded.sum()) / n if n else None),
            noise_sample_points=kept, radial_noise_sum_m=float(radial.sum()),
            radial_noise_sumsq_m2=float(np.sum(radial ** 2)),
            radial_noise_rmse_m=float(np.sqrt(np.mean(radial ** 2))) if kept else None,
            xyz_perturbation_sumsq_m2=float(np.sum(delta ** 2)),
            xyz_perturbation_rmse_m=float(np.sqrt(np.sum(delta ** 2) / kept)) if kept else None)
    if passthrough:return msg
    for i,v in enumerate(views):
        v[valid]=changed[...,i][valid]
        v[occluded]=np.nan
    if np.any(occluded):out.is_dense=False
    out.data=bytes(buf)
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',default='/livox/lidar_pc2')
    p.add_argument('--output',default='/b1/gicp_input')
    p.add_argument('--ramp-sec',type=float,default=30)
    p.add_argument('--hold-sec',type=float,default=15)
    p.add_argument('--release-sec',type=float,default=10)
    p.add_argument('--max-noise-m',type=float,default=.30)
    p.add_argument('--seed',type=int,default=20261027)
    p.add_argument('--log-dir',type=Path,default=None)
    a=p.parse_args()
    if a.input==a.output:p.error('Input and output must differ')
    if any(not math.isfinite(v) or v<=0 for v in (a.ramp_sec,a.hold_sec,a.release_sec,a.max_noise_m)) or a.seed<0:p.error('Positive finite durations/noise and nonnegative seed required')
    if os.environ.get('ROS_DOMAIN_ID')!='20' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':p.error('Requires sim Domain 20 / rmw_fastrtps_cpp')
    from urllib.request import build_opener,ProxyHandler
    with build_opener(ProxyHandler({})).open('http://127.0.0.1:8765/api/config',timeout=3) as f:cfg=json.load(f)
    if cfg.get('stack_mode')!='sim' or str(cfg.get('stack_domain_id'))!='20':p.error('Sim Dashboard required')
    import rclpy
    from tf2_ros import Buffer, TransformListener
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    class Injector(Node):
        def __init__(self):
            super().__init__('b1_gicp_cloud_injector')
            self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
            self.sim=None;self.clock_at=0.;self.advance_at=0.;self.start=None;self.start_stamp=None;self.run=0;self.count=0;self.last_input=0.;self.last_sigma=0.;self.last_metrics={};self.profile="legacy";self.fault=dict(noise=a.max_noise_m,bias=0.,yaw=0.,sector=0.,dropout=0.)
            folder=a.log_dir or Path('debug-output')/('b1-cloud-inject-'+time.strftime('%Y%m%dT%H%M%S'))
            folder.mkdir(parents=True,exist_ok=True);self.log=(folder/'injection.jsonl').open('x',buffering=1)
            self.pub=self.create_publisher(PointCloud2,a.output,qos_profile_sensor_data)
            self.status=self.create_publisher(String,'/b1/lidar_injection_status',10)
            self.create_subscription(Clock,'/clock',self.clock,10)
            self.create_subscription(PointCloud2,a.input,self.cloud,qos_profile_sensor_data)
            self.create_subscription(String,'/b1/lidar_injection_command',self.command,10)
            self.create_timer(1.,self.heartbeat)
            self.record('observer_started',config={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()})
            print('Passthrough ready; logs:',folder,flush=True)
        def record(self,event,**values):
            self.log.write(json.dumps(dict(event=event,wall_unix_sec=time.time(),wall_monotonic_sec=time.monotonic(),sim_sec=self.sim,run=self.run,**values))+'\n')
        def clock(self,m):
            t=m.clock.sec+m.clock.nanosec/1e9
            if self.sim is not None and t<self.sim:
                self.start=None;self.record('clock_reset_injection_stopped',next_sim=t)
            if t!=self.sim:self.advance_at=time.monotonic()
            self.sim=t;self.clock_at=time.monotonic()
            if self.start is not None and t-self.start>=a.ramp_sec+a.hold_sec+a.release_sec:
                self.start=None;self.record('schedule_completed')
        def command(self,m):
            cmd=m.data.strip();profile="legacy"
            if cmd.startswith("{"):
                try:
                    value=json.loads(cmd);cmd=value["command"];profile=value.get("profile","legacy")
                except (ValueError,KeyError,TypeError):
                    self.record('command_rejected',reason='Invalid command JSON');return
            if profile not in {"legacy",*PROFILES}:
                self.record('command_rejected',reason='Unknown profile');return
            if cmd=='stop':self.start=None;self.record('manual_stop');return
            if cmd!='start':self.record('command_rejected',reason='Use start or stop');return
            now=time.monotonic()
            if self.start is not None:self.record('command_rejected',reason='Already active');return
            if self.sim is None or now-self.advance_at>2 or now-self.last_input>2:
                self.record('command_rejected',reason='Fresh advancing clock and input required');return
            # Output must be isolated and actually consumed before arming.
            if self.count_publishers(a.output)!=1 or self.count_subscribers(a.output)<1:
                self.record('command_rejected',reason='Require sole output publisher and at least one subscriber');return
            self.profile=profile
            self.fault=dict(PROFILES[profile]) if profile!='legacy' else dict(noise=a.max_noise_m,bias=0.,yaw=0.,sector=0.,dropout=0.)
            self.run+=1;self.start=self.sim;self.start_stamp=None
            self.record('injection_started',profile=self.profile,fault=self.fault,ramp_sec=a.ramp_sec,hold_sec=a.hold_sec,release_sec=a.release_sec)
        def cloud(self,m):
            self.last_input=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
            if self.start is not None and time.monotonic()-self.clock_at>2:
                self.start=None;self.record('clock_stale_injection_stopped')
            elapsed=self.sim-self.start if self.start is not None else -1
            level=strength(elapsed,a.ramp_sec,a.hold_sec,a.release_sec,1.)
            sigma=level*self.fault['noise']
            if self.start_stamp is None and self.start is not None:self.start_stamp=stamp
            # Relative source stamp gives stable noise per frame for repeated input sequences.
            frame_seed=np.random.SeedSequence([a.seed,int(round(max(0,stamp-(self.start_stamp or stamp))*1e9))])
            t=time.monotonic()
            drop=bool(np.random.default_rng(frame_seed).random()<level*self.fault['dropout'])
            metrics={}
            translation=[0.,0.,0.]
            if level and self.fault.get('map_bias_y'):
                try:
                    tf=self.tf_buffer.lookup_transform(m.header.frame_id,'map',rclpy.time.Time())
                    q=tf.transform.rotation
                    # Rotate a map +Y vector into the incoming cloud frame (translation is a vector).
                    vector=np.array([0.,level*self.fault['map_bias_y'],0.])
                    qv=np.array([q.x,q.y,q.z]);cross=np.cross(qv,vector)
                    translation=(vector+2*(q.w*cross+np.cross(qv,cross))).tolist()
                except Exception as e:
                    self.start=None;level=0.;sigma=0.;drop=False
                    self.record('transform_unavailable_injection_stopped',error=str(e))
            try:out=m if drop else corrupt_cloud(m,sigma,frame_seed,
                bias=level*self.fault['bias'],yaw_deg=level*self.fault['yaw'],sector_deg=level*self.fault['sector'],metrics=metrics if self.start is not None else None,translation=translation)
            except (ValueError,TypeError) as e:
                self.start=None;self.record('invalid_cloud_injection_stopped',error=str(e));out=m;sigma=0.;drop=False;level=0.;metrics={"invalid_cloud": True}
            if not drop:self.pub.publish(out)
            # Whole-frame drops have no measured output perturbation: do not invent point statistics.
            if drop:metrics = dict(output_valid_points=0, whole_frame_dropped=True)
            self.count+=1;self.last_sigma=sigma;self.last_metrics=metrics
            self.record('cloud',source_stamp_sec=stamp,frame_id=m.header.frame_id,points=m.width*m.height,
                        translation_cloud_xyz_m=translation,map_bias_y_m=level*self.fault.get('map_bias_y',0.),sigma_m=sigma,level=level,profile=self.profile,dropped=drop,metrics=metrics,dropout_probability=level*self.fault['dropout'],bias_m=level*self.fault['bias'],yaw_deg=level*self.fault['yaw'],sector_deg=level*self.fault['sector'],elapsed_sim_sec=elapsed,processing_wall_sec=time.monotonic()-t)
        def heartbeat(self):
            self.status.publish(String(data=json.dumps(dict(active=self.start is not None,sim_sec=self.sim,
                input=a.input,output=a.output,count=self.count,input_age_wall_sec=time.monotonic()-self.last_input,
                metrics_version=2,log_path=str(self.log.name),last_cloud_metrics=self.last_metrics,sigma_m_last_cloud=self.last_sigma,profile=self.profile,fault=self.fault,seed=a.seed,run=self.run))))
    rclpy.init();n=Injector()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.record('observer_stopped');n.log.close();n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
