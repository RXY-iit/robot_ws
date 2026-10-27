#!/usr/bin/env python3
"""Summarize bounded pilot runs; receipt timestamps never relabelled ground truth."""
import argparse,json,math
from pathlib import Path

def load(path):
    return [json.loads(line) for line in Path(path).open()]

def summarize(evidence,trigger,start,end):
    rows=[r for r in load(evidence) if start<=r['wall_unix_sec']<=end]
    bad=[];scores=[];accept=[];wheel=[];bt=[];pb=[];resume=[];health=[];nav=[]
    for r in rows:
        t=r['wall_unix_sec'];topic=r['event'];d=r['data']
        if topic in ['/agent/health_state','/b1/readonly_health_state']:
            c=d.get('components',{}).get('localization',{})
            health.append((t,c.get('level'),c.get('summary')))
            if c.get('level')=='BAD':bad.append(t)
        if topic=='/gicp_loc/score':scores.append((t,d['data']))
        if topic=='/gicp_loc/accepted_state':accept.append(t)
        if topic=='/wheel_odom':
            p=d['pose']['pose']['position'];v=d['twist']['twist'];speed=max(abs(v['linear']['x']),abs(v['linear']['y']),abs(v['angular']['z']))
            wheel.append((t,p['x'],p['y'],speed))
        if topic=='/agent/recovery_events' and d.get('type') in ['execution_started','execution_succeeded','action_auto_approved']:
            pb.append({'wall':t,'type':d['type'],'id':d.get('execution_id'),'key':d.get('incident_key'),'playbook':d.get('playbook_id'),'message':d.get('message')})
        if topic=='/agent/mission_events' and d.get('type') in ['mission_recovery_resume_authorized','skill_finished']:
            resume.append({'wall':t,'type':d['type'],'payload':d.get('payload')})
        if topic=='/navigate_to_pose/_action/feedback':nav.append((t,d['feedback']['number_of_recoveries']))
        if topic=='/behavior_tree_log':
            for e in d['event_log']:
                et=e['timestamp']['sec']+e['timestamp']['nanosec']/1e9
                if start<=et<=end and e['current_status'] in {'RUNNING','SUCCESS'} and any(k in e['node_name'] for k in ['Spin','Wait','Clear','Back']):bt.append((et,e['node_name'],e['current_status']))
    first_bad=next((t for t in bad if t>=trigger),None)
    path_after=0.;first_stop=None;still_since=None
    for prev,current in zip(wheel,wheel[1:]):
        if first_bad is None or current[0]<first_bad:continue
        if max(prev[3],current[3])>.02:path_after+=math.hypot(current[1]-prev[1],current[2]-prev[2])
        if current[3]<.01:
            if still_since is None:still_since=current[0]
            if first_stop is None and current[0]-still_since>=2:first_stop=still_since
        else:still_since=None
    path_until_stop=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(wheel,wheel[1:]) if first_bad is not None and first_stop is not None and first_bad<=b[0]<=first_stop and max(a[3],b[3])>.02)
    during=[v for t,v in scores if t>=trigger]
    def max_gap(times):return max([b-a for a,b in zip(times,times[1:])],default=None)
    return dict(start_wall=start,end_wall=end,injection_wall=trigger,first_bad_wall=first_bad,pre_injection_bad_samples=len([t for t in bad if t<trigger]),first_sustained_stop_wall=first_stop,motion_path_after_first_bad_m=path_after,motion_path_bad_to_first_stop_m=path_until_stop,wheel_start=wheel[0] if wheel else None,wheel_end=wheel[-1] if wheel else None,raw_score_range=[min(during),max(during)] if during else None,raw_score_max_gap_wall_sec=max_gap([t for t,v in scores]),accepted_max_gap_wall_sec=max_gap(accept),bt_recovery_events=sorted(set(bt)),max_nav2_recovery_count=max([v for t,v in nav],default=0),pb_events=pb,mission_events=resume,health=health)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',required=True);p.add_argument('--trigger',type=float,required=True);p.add_argument('--start',type=float,required=True);p.add_argument('--end',type=float,required=True);p.add_argument('--output',required=True);a=p.parse_args()
    result=summarize(a.evidence,a.trigger,a.start,a.end);Path(a.output).write_text(json.dumps(result,indent=2,ensure_ascii=False));print(json.dumps({k:v for k,v in result.items() if k not in ['health','pb_events','mission_events']},indent=2))
if __name__=='__main__':main()
