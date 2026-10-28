#!/usr/bin/env python3
"""Compare explicitly bounded shelf trials; odometry is a proxy, never ground truth."""
import argparse,json,math
from pathlib import Path
import numpy as np

def summarize(path,start,end):
    samples={};events=[];last={};transitions=[]
    for line in Path(path).open():
        r=json.loads(line);t=r['wall_unix_sec']
        if not start<=t<=end:continue
        k=r['event'];d=r.get('data',{})
        if k=='/gicp_loc/accepted_state':
            m=d['map_base_pose'];o=d['odom_base_pose'];a=m[2]-o[2]
            correction=[m[0]-math.cos(a)*o[0]+math.sin(a)*o[1],m[1]-math.sin(a)*o[0]-math.cos(a)*o[1],a]
            samples.setdefault('accepted',[]).append([t,*m,*o,*correction])
        elif k=='/wheel_odom':
            p=d['pose']['pose']['position'];v=d['twist']['twist'];speed=math.hypot(v['linear']['x'],v['linear']['y'])
            samples.setdefault('wheel',[]).append([t,p['x'],p['y'],speed,abs(v['angular']['z'])])
        elif k=='/cmd_vel':samples.setdefault('command',[]).append([t,math.hypot(d['linear']['x'],d['linear']['y']),abs(d['angular']['z'])])
        elif k=='/gicp_loc/score':samples.setdefault('score',[]).append([t,d['data']])
        elif k=='/b1/local_costmap_proximity':
            if d.get('nearest_lethal_cell_m') is not None:samples.setdefault('clearance',[]).append([t,d['nearest_lethal_cell_m']])
        elif k=='/agent/health_dashboard_state':
            for domain in d.get('domains',[]):
                for item in domain.get('items',[]):
                    key=item['id'];v=item.get('status')
                    if v!=last.get(key):
                        transitions.append(dict(wall=t,item=key,status=v,summary=item.get('summary'),policy=item.get('response_policy')));last[key]=v
        elif k in {'/agent/recovery_events','/agent/mission_events'}:
            if d.get('type') in {'execution_started','execution_succeeded','execution_failed','mission_recovery_resume_authorized','skill_finished','mission_finished','mission_done'}:
                events.append(dict(wall=t,topic=k,data=d))
        elif k=='/behavior_tree_log':
            for e in d.get('event_log',[]):
                if e.get('current_status')=='RUNNING' and any(x in e.get('node_name','') for x in ['Spin','Wait','Clear','Back']):events.append(dict(wall=t,topic=k,data=e))
    return dict(samples=samples,transitions=transitions,events=events)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trials',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    specs=json.loads(Path(a.trials).read_text());out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(4,len(specs),figsize=(7*len(specs),13),squeeze=False)
    traj,ta=plt.subplots(figsize=(10,6));results={};common=None
    for col,spec in enumerate(specs):
        data=summarize(spec['evidence'],spec['start_wall'],spec['end_wall']);ss=data['samples'];t0=spec['injection_wall'];stats={}
        for key in ['score','clearance']:
            points=ss.get(key,[]);values=[v[1] for v in points if v[0]>=t0]
            stats[key+'_post_injection_range']=[min(values),max(values)] if values else None
        accepted=ss.get('accepted',[]);wheel=ss.get('wheel',[])
        pre=[v for v in accepted if v[0]<t0];ref=pre[-1] if pre else (accepted[0] if accepted else None)
        if ref:
            changes=[[v[0],math.dist(v[7:9],ref[7:9])] for v in accepted]
            stats['max_relative_correction_change_m']=max(v[1] for v in changes)
            # Do not interpolate through initialpose accumulation / missing evidence.
            plot_points=[]
            for i,v in enumerate(changes):
                if i and v[0]-changes[i-1][0]>5:plot_points.append([v[0],float('nan')])
                plot_points.append(v)
            axs[1,col].plot([v[0]-t0 for v in plot_points],[v[1] for v in plot_points])
        for row,key in [(0,'score'),(2,'clearance')]:
            points=ss.get(key,[]);axs[row,col].scatter([v[0]-t0 for v in points],[v[1] for v in points],s=7)
        axs[0,col].axhline(.16,color='orange',ls=':');axs[0,col].axhline(.20,color='red',ls=':')
        if accepted and common is None:common=accepted[0][7:10]
        if wheel and common:
            x,y,ang=common;c,si=math.cos(ang),math.sin(ang)
            ta.plot([x+c*v[1]-si*v[2] for v in wheel],[y+si*v[1]+c*v[2] for v in wheel],label=spec['label']+' wheel (common transform)')
            path=[0.]
            for u,v in zip(wheel,wheel[1:]):path.append(path[-1]+math.dist(u[1:3],v[1:3]))
            axs[3,col].plot([v[0]-t0 for v in wheel],path);stats['wheel_path_m']=path[-1]
            after=[v for v in wheel if v[0]>=t0]
            if after:stats['post_injection_wheel_y_range_m']=[min(v[2] for v in after),max(v[2] for v in after)]
            stats['wheel_start_xy']=wheel[0][1:3];stats['wheel_end_xy']=wheel[-1][1:3]
        ta.scatter([v[1] for v in accepted],[v[2] for v in accepted],s=5,alpha=.35,label=spec['label']+' estimated map pose')
        # Show the actual score item's hysteresis state, not thresholds alone.
        bands=[v for v in data['transitions'] if v['item']=='gicp_score']
        for i,b in enumerate(bands):
            right=bands[i+1]['wall'] if i+1<len(bands) else spec['end_wall']
            color={'OK':'#b8e0c2','WARN':'#ffe19a','DEGRADED':'#ffe19a','ERROR':'#f5abab','STALE':'#cccccc'}.get(b['status'],'#eeeeee')
            axs[0,col].axvspan(b['wall']-t0,right-t0,color=color,alpha=.4,zorder=-1)
        for row,label in enumerate(['GICP score (m); score-item state','Map/odom translation change (m)','Nearest lethal cell to base centre (m)','Wheel-odom path length (m)']):
            axs[row,col].set_ylabel(label);axs[row,col].axvline(0,color='black',ls='--');axs[row,col].grid(alpha=.2)
        if not ss.get('clearance'):axs[2,col].text(.5,.5,'No lethal-cell distance observed',transform=axs[2,col].transAxes,ha='center')
        axs[0,col].set_title(spec['label']);axs[3,col].set_xlabel('Wall seconds relative to injection')
        bad=next((v['wall'] for v in data['transitions'] if v['item']=='gicp_score' and v['status']=='ERROR' and v['wall']>=t0),None)
        stats['first_score_BAD_offset_wall_sec']=bad-t0 if bad is not None else None
        after_bad=[v for v in wheel if bad is not None and v[0]>=bad]
        stats['wheel_path_after_first_score_BAD_m']=sum(math.dist(u[1:3],v[1:3]) for u,v in zip(after_bad,after_bad[1:])) if after_bad else None
        stats['additional_wheel_y_after_BAD_m']=max(v[2] for v in after_bad)-after_bad[0][2] if after_bad else None
        still=None;stop=None
        for v in after_bad:
            if v[3]<.01 and v[4]<.01:
                if still is None:still=v[0]
                if v[0]-still>=2:stop=still;break
            else:still=None
        stats['first_two_second_stop_after_BAD_offset_wall_sec']=stop-t0 if stop is not None else None
        moving_window=[v for v in after_bad if stop is not None and v[0]<=stop]
        stats['wheel_path_BAD_to_first_stop_m']=sum(math.dist(u[1:3],v[1:3]) for u,v in zip(moving_window,moving_window[1:])) if moving_window else None
        stats['BAD_to_first_stop_wall_sec']=stop-bad if stop is not None and bad is not None else None
        command_still=None;command_stop=None;previous_command=None
        for v in ss.get('command',[]):
            if bad is None or v[0]<bad:continue
            if previous_command is not None and v[0]-previous_command>.5:command_still=None
            previous_command=v[0]
            if v[1]<.01 and v[2]<.01:
                if command_still is None:command_still=v[0]
                if v[0]-command_still>=2:command_stop=command_still;break
            else:command_still=None
        stats['first_two_second_zero_command_after_BAD_offset_wall_sec']=command_stop-t0 if command_stop is not None else None
        pause=next((e['wall'] for e in data['events'] if e['topic']=='/agent/mission_events' and e['data'].get('payload',{}).get('metrics',{}).get('recovery_pause')),None)
        pb=next((e['wall'] for e in data['events'] if e['topic']=='/agent/recovery_events' and e['data'].get('type')=='execution_started'),None)
        bt=next((e['wall'] for e in data['events'] if e['topic']=='/behavior_tree_log' and e['wall']>=t0),None)
        for key,value in [('recovery_pause',pause),('PB_start',pb),('BT_start',bt)]:
            stats[key+'_offset_wall_sec']=value-t0 if value is not None else None
            if value is not None:
                for ax in axs[:,col]:ax.axvline(value-t0,color='purple',ls=':',alpha=.6)
        stats['score_item_states']=sorted({v['status'] for v in data['transitions'] if v['item']=='gicp_score'})
        stats['duration_wall_sec']=spec['end_wall']-spec['start_wall']
        results[spec['label']]=dict(spec=spec,metrics=stats,transitions=data['transitions'],events=data['events'])
    xmin=min(s['start_wall']-s['injection_wall'] for s in specs);xmax=max(s['end_wall']-s['injection_wall'] for s in specs)
    for col,spec in enumerate(specs):
        for ax in axs[:,col]:
            ax.set_xlim(xmin,xmax)
            ax.axvline(spec['end_wall']-spec['injection_wall'],color='gray',ls='--',alpha=.5)
        axs[0,col].text(.02,.96,'Black: injection; purple: Recovery/PB/BT; gray: window end',transform=axs[0,col].transAxes,va='top',fontsize=7)
    ta.plot([0,1.4,-3.3,-8.2],[0,6,6,6],'k--o',label='Frozen waypoints');ta.legend(fontsize=8);ta.set_xlabel('Map x via common initial odom transform (m)');ta.set_ylabel('Map y via common initial odom transform (m)');
    for label,x,y in [('home',0,0),('06',1.4,6),('05',-3.3,6),('04',-8.2,6)]:ta.annotate(label,(x,y),xytext=(4,-15),textcoords='offset points')
    ta.annotate('+Y: shelf side',xy=(2.5,7.6),xytext=(2.5,6.8),arrowprops={'arrowstyle':'->'},ha='center')
    ta.axis('equal');ta.grid(alpha=.3);ta.set_title('Trajectory proxy: no independent absolute world calibration')
    fig.tight_layout();traj.tight_layout();fig.savefig(out/'timeline.png',dpi=160);traj.savefig(out/'trajectory.png',dpi=160)
    (out/'comparison.json').write_text(json.dumps(results,ensure_ascii=False,indent=2));print(json.dumps({k:v['metrics'] for k,v in results.items()},indent=2))
if __name__=='__main__':main()
