#!/usr/bin/env python3
"""Plot captured factors and responses without filling missing localization evidence.
Input is capture_b1_trial.py JSONL; optional second capture supports paired views.
"""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def se2(m,o):
    a=m[2]-o[2];c,s=math.cos(a),math.sin(a)
    return [m[0]-c*o[0]+s*o[1],m[1]-s*o[0]-c*o[1],math.atan2(math.sin(a),math.cos(a))]


def odom_pose(d):
    p=d['pose']['pose'];q=p['orientation']
    return [p['position']['x'],p['position']['y'],math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))]


def aligned(p,t):
    c,s=math.cos(t[2]),math.sin(t[2]);return [t[0]+c*p[0]-s*p[1],t[1]+s*p[0]+c*p[1],p[2]+t[2]]


def build(rows):
    series={};events=[];truth=[];accepted=[];cal=None;transform0=None
    def add(k,r,v):
        if isinstance(v,(float,int)) and math.isfinite(v):series.setdefault(k,[]).append((r.get('wall_monotonic_sec'),r.get('sim_sec'),v))
    for r in rows:
        e,d=r['event'],r.get('data',{});at=r.get('source_stamp_sec')
        if e=='truth_calibration':cal=d['value']
        if e=='/b1/ground_truth_odom' and at is not None:truth.append((at,odom_pose(d),r))
        if e=='/b1/lidar_injection_status':
            m=d.get('last_cloud_metrics',{})
            for key in ['radial_noise_rmse_m','radial_noise_abs_p95_m','occluded_fraction_valid','input_valid_fraction']:
                add(key,r,m.get(key))
            add('map_roi_fraction',r,m.get('map_roi_exposure',{}).get('fraction'))
            add('plane_residual_m',r,m.get('output_quality',{}).get('local_plane_rms_median_m'))
        if e=='/b1/odom_injection_status':
            add('odom_error_m',r,d.get('position_error_m'));add('odom_yaw_error_rad',r,d.get('yaw_error_rad'))
            if d.get('event') in ['blocked','injection_started','growth_stopped']:events.append(r)
        if e=='/gicp_loc/score':add('raw_score',r,d.get('data'))
        if e=='/gicp_loc/match_evidence':
            add('raw_match_rmse',r,d.get('rmse_m'));add('fitness',r,d.get('fitness'))
            events.append(r)
        if e=='/gicp_loc/accepted_state':
            m,o=d.get('map_base_pose'),d.get('odom_base_pose')
            add('accepted_score',r,d.get('rmse_m'))
            if m and o:
                t=se2(m,o)
                if transform0 is None:transform0=t
                for j,k in enumerate(['map_odom_dx_m','map_odom_dy_m','map_odom_dyaw_rad']):
                    v=t[j]-transform0[j];add(k,r,math.atan2(math.sin(v),math.cos(v)) if j==2 else v)
                accepted.append((d.get('stamp_sec'),m,r))
        if e in ['/agent/health_dashboard_state','/b1/readonly_health_state']:
            items={i['id']:i for dom in d.get('domains',[]) for i in dom.get('items',[])}
            g=items.get('gicp_score',{});status=g.get('status')
            add('score_health',r,{'OK':0,'WARN':1,'DEGRADED':1,'ERROR':2,'STALE':2,'UNKNOWN':3}.get(status,3))
            for cap in d.get('capabilities',[]):
                if cap.get('id')=='auto_navigation':add('nav_readiness',r,{'AVAILABLE':0,'DEGRADED':1,'BLOCKED':2}.get(cap.get('status'),3))
        if e=='/agent/recovery_state':
            response=d.get('robot_health_summary',{});add('robot_response',r,{'OK':0,'WARN':1,'BAD':2}.get(response.get('level'),3))
        event_levels={'/initialpose':('seed_event',1),'/agent/recovery_events':('recovery_event',2),'/agent/mission_events':('mission_event',3),'/b1/waypoint_events':('waypoint_event',4),'/b1/lidar_injection_command':('injection_event',0),'/b1/odom_injection_command':('injection_event',0)}
        if e in event_levels:
            k,v=event_levels[e];add(k,r,v)
        if e=='/cmd_vel':add('speed_mps',r,math.hypot(d.get('linear',{}).get('x',0),d.get('linear',{}).get('y',0)))
        if e in ['/agent/recovery_events','/agent/mission_events','/b1/waypoint_events','/initialpose','/b1/lidar_injection_command','/b1/odom_injection_command','map_manifest']:events.append(r)
        if e=='/gicp_loc/recovery_status':add('accumulating',r,1 if d.get('phase')=='accumulating' else 0)
    # Timestamp alignment never uses latest receipt pose or corrupted wheel odom.
    truth.sort(key=lambda x:x[0]);stamps=np.array([x[0] for x in truth]);matched=0
    if cal and truth:
        transform=cal['map_from_physics_se2']
        for stamp,p,r in accepted:
            if stamp is None:continue
            j=int(np.searchsorted(stamps,stamp));candidates=[k for k in (j-1,j) if 0<=k<len(truth)]
            if not candidates:continue
            k=min(candidates,key=lambda k:abs(stamps[k]-stamp))
            if abs(stamps[k]-stamp)>.1:continue
            gt=aligned(truth[k][1],transform);add('aligned_position_error_m',r,math.dist(gt[:2],p[:2]));matched+=1
    return series,events,dict(truth_samples=len(truth),accepted_samples=len(accepted),aligned_pairs=matched,
        calibration=cal,truth_claim='No absolute accuracy claim without independent map/physics calibration',
        reference='map/odom changes relative to first recorded accepted sample; not necessarily injection start')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('captures',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);data=[];meta={}
    for path in a.captures:
        rows=[json.loads(l) for l in path.read_text().splitlines() if l.strip()];series,events,details=build(rows);data.append((path.stem,series,rows))
        (a.output/(path.stem+'-events.json')).write_text(json.dumps(events,ensure_ascii=False,indent=2))
        details['ranges']={k:{'n':len(v),'min':min(x[2] for x in v),'max':max(x[2] for x in v)} for k,v in series.items()};meta[str(path)]=details
    panels=[('LiDAR injected error [m]',['radial_noise_rmse_m','radial_noise_abs_p95_m']),
        ('Observable quality [m / fraction]',['plane_residual_m','input_valid_fraction','occluded_fraction_valid']),
        ('Odom injected error [m / rad]',['odom_error_m','odom_yaw_error_rad']),
        ('Map ROI exposure fraction (estimated TF)',['map_roi_fraction']),
        ('Raw / accepted score [m]',['raw_score','raw_match_rmse','accepted_score']),
        ('Health/readiness 0 OK, 1 WARN, 2 BAD, 3 unknown',['score_health','nav_readiness','robot_response']),
        ('Map-odom signed correction change [m / rad]',['map_odom_dx_m','map_odom_dy_m','map_odom_dyaw_rad']),
        ('Aligned physics position error [m]',['aligned_position_error_m']),
        ('Command speed [m/s] / scan accumulation flag',['speed_mps','accumulating']),
        ('Events: 0 injection, 1 seed, 2 Recovery, 3 Mission, 4 goal',['injection_event','seed_event','recovery_event','mission_event','waypoint_event'])]
    for basis,col,key in [('wall',0,'wall_monotonic_sec'),('sim',1,'sim_sec')]:
        fig,axes=plt.subplots(len(panels),len(data),figsize=(8*len(data),22),squeeze=False)
        for j,(name,series,rows) in enumerate(data):
            times=[r[key] for r in rows if isinstance(r.get(key),(int,float))];origin=min(times) if times else 0
            for i,(title,keys) in enumerate(panels):
                ax=axes[i,j];drawn=False
                for k in keys:
                    points=[(x[col]-origin,x[2]) for x in series.get(k,[]) if isinstance(x[col],(int,float))]
                    if not points:continue
                    points.sort();x=[];y=[]
                    for t,v in points:
                        if x and t-x[-1]>(5 if basis=='wall' else 2):x.append(float('nan'));y.append(float('nan'))
                        x.append(t);y.append(v)
                    ax.plot(x,y,'o' if k.endswith('_event') else '.-' if k=='accepted_score' else '-',label=k,linewidth=.8,markersize=3 if k.endswith('_event') else 2,drawstyle='steps-post' if k in {'score_health','nav_readiness','robot_response','accumulating'} else 'default');drawn=True
                if keys[0]=='raw_score':
                    statuses=series.get('score_health',[])
                    for left,right in zip(statuses,statuses[1:]):
                        if left[col] is not None and right[col] is not None and 0<right[col]-left[col]<(5 if basis=='wall' else 2):
                            ax.axvspan(left[col]-origin,right[col]-origin,facecolor={0:'green',1:'orange',2:'red',3:'grey'}[left[2]],alpha=.10,edgecolor='none')
                ax.set_title(name+' — '+title);ax.grid(alpha=.2)
                if drawn:ax.legend(fontsize=7)
                else:ax.text(.5,.5,'NO RECORDED EVIDENCE',transform=ax.transAxes,ha='center',color='grey')
                ax.set_xlabel('Elapsed '+basis+' seconds from capture start')
        fig.tight_layout();fig.savefig(a.output/('factors-'+basis+'.png'),dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,len(data),figsize=(8*len(data),7),squeeze=False)
    for j,(name,series,rows) in enumerate(data):
        ax=axes[0,j];cal=next((r['data']['value'] for r in rows if r['event']=='truth_calibration'),None)
        estimated=[r['data']['map_base_pose'] for r in rows if r['event']=='/gicp_loc/accepted_state' and 'map_base_pose' in r['data']]
        goals=[r['data']['goal_xy'] for r in rows if r['event']=='/b1/waypoint_events' and r['data'].get('event')=='accepted']
        if goals:ax.plot([p[0] for p in goals],[p[1] for p in goals],'k--x',linewidth=.7,label='accepted goals, straight connections')
        if estimated:ax.plot([p[0] for p in estimated],[p[1] for p in estimated],'.',markersize=2,label='accepted localization estimate')
        if cal:
            physics=[aligned(odom_pose(r['data']),cal['map_from_physics_se2']) for r in rows if r['event']=='/b1/ground_truth_odom']
            if physics:ax.plot([p[0] for p in physics],[p[1] for p in physics],'-',linewidth=1,label='physics pose, '+cal.get('basis','unspecified calibration'))
        ax.set_title(name);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
        if estimated or cal:ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(a.output/'trajectories.png',dpi=150);plt.close(fig)
    (a.output/'metrics.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
