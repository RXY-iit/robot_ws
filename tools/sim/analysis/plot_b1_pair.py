#!/usr/bin/env python3
"""Render the fixed 2026-10-27 pilot pair, preserving score gaps and provenance."""
import json,math,bisect
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('debug-output/b1-pair-20261027');OUT=Path('todo-my/paper/safeprocess2027/figures');OUT.mkdir(exist_ok=True)
conditions=[('COR','cor2.jsonl','cor-summary.json',1793110171.7646954),('TOR','tor.jsonl','tor-summary.json',1793111080.5658188)]
fig,axs=plt.subplots(4,2,figsize=(15,13),sharex='col');traj,ta=plt.subplots(figsize=(10,6));extra={}
common_transform=None
for column,(name,file,summary,inject_end) in enumerate(conditions):
 s=json.loads((ROOT/summary).read_text());t0=s['injection_wall'];start=s['start_wall'];end=s['end_wall'];rows=[]
 for line in (ROOT/file).open():
  r=json.loads(line)
  if start<=r['wall_unix_sec']<=end:rows.append(r)
 def select(topic):return [r for r in rows if r['event']==topic]
 health=select('/agent/health_state' if name=='COR' else '/b1/readonly_health_state');scores=select('/gicp_loc/score');accepted=select('/gicp_loc/accepted_state');wheel=select('/wheel_odom')
 ref=accepted[0]['data'];m=ref['map_base_pose'];o=ref['odom_base_pose'];angle=m[2]-o[2];c,si=math.cos(angle),math.sin(angle);tx=m[0]-c*o[0]+si*o[1];ty=m[1]-si*o[0]-c*o[1]
 if common_transform is None:common_transform=(c,si,tx,ty)
 residual=[]
 for r in accepted:
  d=r['data'];m=d['map_base_pose'];o=d['odom_base_pose'];err=math.hypot(m[0]-(c*o[0]-si*o[1]+tx),m[1]-(si*o[0]+c*o[1]+ty));residual.append((r['wall_unix_sec'],err))
 # Shared initial COR map<-physics-odom transform, no per-condition trajectory fitting.
 cc,ss,xx,yy=common_transform
 wp=[];path=[0.]
 for r in wheel:
  d=r['data'];p=d['pose']['pose']['position'];v=d['twist']['twist'];speed=max(abs(v['linear']['x']),abs(v['linear']['y']),abs(v['angular']['z']));wp.append((r['wall_unix_sec'],p['x'],p['y'],speed))
 for a,b in zip(wp,wp[1:]):path.append(path[-1]+(math.hypot(b[1]-a[1],b[2]-a[2]) if max(a[3],b[3])>.02 else 0))
 t=np.array([r[0] for r in wp]);path=np.array(path);base=np.interp(t0,t,path);path-=base
 action=next((v['wall'] for v in s['pb_events'] if v['type']=='execution_started' and v['key']=='gicp'),None) if name=='COR' else (s['bt_recovery_events'][0][0] if s['bt_recovery_events'] else None)
 bad=s['first_bad_wall'];before_action=float(np.interp(action,t,path)-np.interp(bad,t,path)) if bad is not None and action is not None else None
 max_res=max(residual,key=lambda x:x[1]);extra[name]={'max_relative_map_odom_correction_m':max_res[1],'max_correction_offset_wall_sec':max_res[0]-t0,'bad_to_first_recovery_motion_path_m':before_action,'first_recovery_offset_wall_sec':action-t0 if action else None,'first_bad_offset_wall_sec':bad-t0 if bad else None,'injection_wall_duration_sec':inject_end-t0,'injection_mean_rtf':40/(inject_end-t0),'reference_accepted':ref}
 axs[0,column].scatter([r['wall_unix_sec']-t0 for r in scores],[r['data']['data'] for r in scores],s=9,color='#173d72');axs[0,column].axhline(.5,color='red',ls=':',label='score cutoff 0.50');axs[0,column].set_ylim(0,.55);axs[0,column].set_title(name);axs[0,column].set_ylabel('Raw GICP RMSE (m)');axs[0,column].legend(fontsize=8)
 ages=[];accepted_times=[r['wall_unix_sec'] for r in accepted]
 for r in health:
  t_health=r['wall_unix_sec'];idx=bisect.bisect_right(accepted_times,t_health)-1
  ages.append(t_health-accepted_times[idx] if idx>=0 else np.nan)
 axs[1,column].plot([r['wall_unix_sec']-t0 for r in health],ages,color='#173d72');axs[1,column].axhline(3,color='red',ls=':');axs[1,column].set_ylabel('Recorder age since last accepted sample (s)');axs[1,column].set_ylim(0,120)
 axs[2,column].plot(t-t0,path,color='#173d72');axs[2,column].set_ylabel('Physics-odom moving path (m)\nrelative to injection start')
 axs[3,column].scatter([v[0]-t0 for v in residual],[v[1] for v in residual],s=9,color='#173d72');axs[3,column].set_ylim(0,1.6);axs[3,column].set_ylabel('Map-odom correction change (m)\nrelative proxy, NOT absolute error')
 bands=[]
 for r in health:
  level=r['data']['components']['localization']['level'];tt=r['wall_unix_sec']
  if bands and bands[-1][2]==level:continue
  bands.append([tt,None,level])
 for j,band in enumerate(bands):
  left=band[0]-t0;right=(bands[j+1][0] if j+1<len(bands) else end)-t0
  color={'OK':'#b8e0c2','WARN':'#ffe19a','BAD':'#f5abab'}.get(band[2],'#cccccc')
  for row in range(4):axs[row,column].axvspan(left,right,color=color,alpha=.4,zorder=-1,linewidth=0)
 for row in range(4):
  ax=axs[row,column];ax.axvline(0,color='black',lw=.8);ax.axvline(inject_end-t0,color='black',ls='--',lw=.8)
  if action:ax.axvline(action-t0,color='#8b008b',ls='-.',lw=.9)
  ax.grid(alpha=.2);ax.set_xlim(-16,350)
 axs[3,column].set_xlabel('Wall seconds from injection start')
 wx=[cc*p[1]-ss*p[2]+xx for p in wp];wy=[ss*p[1]+cc*p[2]+yy for p in wp];ta.plot(wx,wy,label=name+' physics-odom (common initial transform)',lw=1.5)
 ta.scatter(wx[0],wy[0],marker='o',s=40);ta.scatter(wx[-1],wy[-1],marker='x',s=60)
 # Accepted localization path includes only measured samples; not interpolated truth.
 ta.scatter([r['data']['map_base_pose'][0] for r in accepted],[r['data']['map_base_pose'][1] for r in accepted],s=8,alpha=.45,label=name+' accepted map poses')
 if column==0:
  plans=select('/plan')
  if plans:
   poses=plans[0]['data']['poses'];ta.plot([p['pose']['position']['x'] for p in poses],[p['pose']['position']['y'] for p in poses],'k:',label='First COR Nav2 plan')
fig.suptitle('B1 single-segment pilot | green OK / yellow WARN / red BAD\nBlack dashed: fixed injection end; purple: first observed PB / BT recovery; score blanks = missing samples',fontsize=12);fig.tight_layout(rect=(0,0,1,.95));fig.savefig(OUT/'B1-COR-TOR-pair-20261027-timeline.png',dpi=160)
ta.scatter([-23.2],[6.],marker='*',s=130,color='black',label='Frozen goal');ta.axis('equal');ta.grid(alpha=.3);ta.set_xlabel('Map x via fixed initial transform (m)');ta.set_ylabel('Map y via fixed initial transform (m)');ta.set_title('Trajectory proxy: no independent absolute map/world calibration');ta.legend(fontsize=8);traj.tight_layout();traj.savefig(OUT/'B1-COR-TOR-pair-20261027-path.png',dpi=160)
(ROOT/'comparison-extra.json').write_text(json.dumps(extra,indent=2));print(json.dumps({k:{x:y for x,y in v.items() if x!='reference_accepted'} for k,v in extra.items()},indent=2))
