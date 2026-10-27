#!/usr/bin/env python3
"""Streaming summary of task-free natural health evidence; safe while capture runs."""
import argparse,json,collections,math
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',required=True);p.add_argument('--start-wall-sec',type=float,default=None,help='Optional later start to exclude offline analysis overlap');a=p.parse_args();folder=Path(a.folder);m=json.loads((folder/'manifest.json').read_text());begin=max(m['started_wall_sec']+m['warmup_exclude_wall_sec'],a.start_wall_sec or 0);end=begin;count=collections.Counter();loc=collections.Counter();items={};events=[];ranges={};last_score=None;gap=0.;moving=0;injected=0;tasks=set();commands=0;done=False;first_odom=None;last_odom=None
    def measure(key,value):
        if isinstance(value,(float,int)) and math.isfinite(value):
            r=ranges.setdefault(key,[value,value]);r[0]=min(r[0],value);r[1]=max(r[1],value)
    for line in (folder/'idle.jsonl').open():
        try:r=json.loads(line)
        except ValueError:continue
        t=r['wall_unix_sec'];topic=r['event'];d=r.get('data',{});done=done or topic=='capture_finished'
        if t<begin:continue
        end=max(end,t);count[topic]+=1
        if topic=='/agent/health_state':
            c=d.get('components',{}).get('localization',{});loc[str((c.get('level'),c.get('summary')))]+=1
        elif topic=='/agent/health_dashboard_state':
            for domain in d.get('domains',[]):
                for item in domain.get('items',[]):
                    key=item['id'];status=item.get('status','UNKNOWN');entry=items.setdefault(key,{'states':{},'first_problem':None,'last_problem':None,'examples':[]});entry['states'][status]=entry['states'].get(status,0)+1
                    if status in ['WARN','WARNING','ERROR','STALE','MISSING','DEGRADED','FAULT']:
                        if entry['first_problem'] is None:entry['first_problem']=t
                        entry['last_problem']=t
                        text=item.get('summary','')
                        if text not in entry['examples'] and len(entry['examples'])<4:entry['examples'].append(text)
                    v=item.get('value',{})
                    if key=='host_resources' and isinstance(v,dict):
                        for k in ['cpu_used_percent','memory_used_percent','hwmon_max_c']:measure(k,v.get(k))
                    if key=='sim_rtf' and isinstance(v,dict):measure('rtf',v.get('rtf'))
        elif topic=='/gicp_loc/score':
            measure('raw_gicp_score',d.get('data'))
            if last_score is not None:gap=max(gap,t-last_score)
            last_score=t
        elif topic=='/wheel_odom':
            v=d['twist']['twist'];speed=max(abs(v['linear']['x']),abs(v['linear']['y']),abs(v['angular']['z']));moving+=speed>.02
            pos=d['pose']['pose']['position'];last_odom=[pos['x'],pos['y']]
            if first_odom is None:first_odom=last_odom
        elif topic=='/cmd_vel':commands+=max(abs(d['linear']['x']),abs(d['linear']['y']),abs(d['angular']['z']))>.001
        elif topic=='/b1/lidar_injection_status':injected+=bool(d.get('active'))
        elif topic=='/agent/mission_state':
            if d.get('status') not in ['CANCELLED','DONE','FAILED','IDLE',None]:tasks.add(str((d.get('mission_id'),d.get('status'))))
        elif topic=='/agent/recovery_events' and d.get('type') in ['execution_started','execution_succeeded','execution_failed','operator_request_opened','action_auto_approved']:
            events.append({'wall':t,'type':d.get('type'),'execution_id':d.get('execution_id'),'incident_key':d.get('incident_key'),'playbook':d.get('playbook_id'),'kind':d.get('kind'),'message':d.get('message')})
    out={'capture_finished':done,'analysis_start_wall_sec':begin,'latest_wall_sec':end,'observed_wall_sec_after_warmup':max(0,end-begin),'topic_counts':dict(count),'localization_sample_counts':dict(loc),'ranges':ranges,'max_raw_score_gap_wall_sec':gap,'moving_odom_samples_over_0_02':moving,'nonzero_cmd_vel_samples':commands,'active_injection_samples':injected,'active_mission_states':sorted(tasks),'odom_net_displacement_m':math.dist(first_odom,last_odom) if first_odom else None,'problem_items':{k:v for k,v in items.items() if v['first_problem'] is not None},'recovery_events':events,'interpretation':'Natural idle observation; no claim of prevented future failure or active repair without action and verification evidence.'}
    (folder/'summary.json').write_text(json.dumps(out,indent=2,ensure_ascii=False));print(json.dumps({k:v for k,v in out.items() if k not in ['topic_counts','problem_items','recovery_events']},ensure_ascii=False,indent=2));print('problem items',list(out['problem_items']));print('recovery events',events[:12])
if __name__=='__main__':main()
