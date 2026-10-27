#!/usr/bin/env python3
"""Bounded run1–3 comparison from recorder and per-frame injection evidence.

Trials JSON is a list: {label, injection_run, start_wall, end_wall}.
No missing measurements are converted to zero. Wheel odometry is not truth.
"""
import argparse
import collections
import json
import math
from pathlib import Path


def rows(path):
    with Path(path).open() as f:
        for line in f:
            yield json.loads(line)


def correction(p, q):
    yaw = p[2]-q[2]
    c, s = math.cos(yaw), math.sin(yaw)
    return [p[0]-c*q[0]+s*q[1], p[1]-s*q[0]-c*q[1], yaw]


def summarize(evidence, injection, trial):
    start, end = float(trial['start_wall']), float(trial['end_wall'])
    if not math.isfinite(start+end) or end <= start:
        raise ValueError('Explicit finite start_wall < end_wall required')
    run = trial['injection_run']
    config = None; frames = dropped = measured = valid = occluded = retained = 0
    noise_sum = noise_sq = xyz_sq = 0.; max_source_gap = None; prev_source = None
    completed = None; max_processing = 0.; missing_metrics = 0
    for r in rows(injection):
        if r.get('run') != run: continue
        if r['event']=='injection_started':
            if config is not None: raise ValueError('Duplicate injection run; use one session log')
            config = r
        if r['event'] in ('schedule_completed','manual_stop','clock_reset_injection_stopped','clock_stale_injection_stopped'):
            if start <= r['wall_unix_sec'] <= end:completed = r
        if r['event']!='cloud' or r.get('elapsed_sim_sec',-1)<0: continue
        if not start <= r['wall_unix_sec'] <= end:continue
        frames += 1; dropped += bool(r['dropped'])
        stamp = r.get('source_stamp_sec')
        if prev_source is not None and stamp is not None:
            gap = stamp-prev_source
            max_source_gap = gap if max_source_gap is None else max(max_source_gap,gap)
        prev_source = stamp
        max_processing = max(max_processing,r.get('processing_wall_sec',0.))
        m=r.get('metrics',{})
        if 'input_valid_points' in m:
            measured += 1; valid += m['input_valid_points']; occluded += m['occluded_valid_points']
            retained += m['noise_sample_points']; noise_sum += m['radial_noise_sum_m']
            noise_sq += m['radial_noise_sumsq_m2']; xyz_sq += m['xyz_perturbation_sumsq_m2']
        elif not r['dropped']:missing_metrics += 1
    if config is None: raise ValueError(f'No injection_started for run {run}')
    trigger=config['wall_unix_sec']
    if not start <= trigger <= end:raise ValueError('Injection outside trial window')
    ranges={}; counts=collections.Counter(); item_counts={}; transitions=[]; previous={}
    pb=[]; mission=[]; match_results=[]; robot=[]; readiness=[]; last_readiness={}; last_robot=None
    gaps={}; last_received={}; first_received={}; wheel_first=wheel_last=None; path=0.; anchor=None
    def measure(k,v):
        if isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v):
            a=ranges.setdefault(k,[v,v]);a[0]=min(a[0],v);a[1]=max(a[1],v)
    def change(key,state,t,**extra):
        if previous.get(key)!=state:
            transitions.append(dict(wall=t,from_injection_wall_sec=t-trigger,item=key,state=state,**extra));previous[key]=state
    for r in rows(evidence):
        t=r['wall_unix_sec']
        if not start<=t<=end:continue
        topic=r['event'];d=r.get('data',{});counts[topic]+=1
        if topic in ('/gicp_loc/score','/gicp_loc/accepted_state'):
            first_received.setdefault(topic,t)
            if topic in last_received:gaps[topic]=max(gaps.get(topic,0),t-last_received[topic])
            last_received[topic]=t
        if topic=='/gicp_loc/score':measure('raw_rmse_m',d.get('data'))
        elif topic=='/gicp_loc/accepted_state':
            for k in ('rmse_m','fitness','correspondences'):measure('accepted_'+k,d.get(k))
            if 'map_base_pose' in d and 'odom_base_pose' in d:
                transform=correction(d['map_base_pose'],d['odom_base_pose'])
                if anchor is None:anchor=transform
                measure('map_odom_correction_change_m',math.dist(transform[:2],anchor[:2]))
                measure('map_odom_correction_change_rad',abs(math.atan2(math.sin(transform[2]-anchor[2]),math.cos(transform[2]-anchor[2]))))
        elif topic=='/gicp_loc/recovery_status':
            if not match_results or match_results[-1]['data'] != d:
                match_results.append(dict(wall=t,data=d))
        elif topic=='/wheel_odom':
            p=d['pose']['pose']['position'];xy=[p['x'],p['y']]
            if wheel_first is None:wheel_first=xy
            if wheel_last is not None:path+=math.dist(xy,wheel_last)
            wheel_last=xy
        elif topic in ('/agent/health_state','/b1/readonly_health_state'):
            loc=d.get('components',{}).get('localization',{})
            change('legacy_localization',str((loc.get('level'),loc.get('summary'))),t)
            for k in ('stationary_drift_m','stationary_drift_yaw_rad','trusted_pose_delta_m','trusted_pose_delta_yaw_rad','gicp_score_age_sec','localization_pose_age_sec'):
                measure(k,loc.get('details',{}).get(k))
        elif topic=='/agent/health_dashboard_state':
            seen=set()
            for domain in d.get('domains',[]):
                for item in domain.get('items',[]):
                    key=item['id']
                    if key in seen:continue
                    seen.add(key);status=item.get('status')
                    item_counts.setdefault(key,collections.Counter())[str(status)]+=1
                    change(key,status,t,summary=item.get('summary'),response=item.get('response_policy'),
                           receive_age_sec=item.get('receive_age_sec'),source_age_sec=item.get('source_age_sec'))
                    if key=='sim_rtf':measure('rtf',(item.get('value') or {}).get('rtf'))
            for group in d.get('readiness',{}).get('groups',[]):
                key=group['id'];state=group.get('status')
                if last_readiness.get(key)!=state:
                    readiness.append(dict(wall=t,id=key,status=state,summary=group.get('summary')));last_readiness[key]=state
        elif topic=='/agent/recovery_state':
            summary=d.get('robot_health_summary')
            if summary is not None:
                signature=json.dumps(summary,sort_keys=True)
                if signature!=last_robot:
                    robot.append(dict(wall=t,summary=summary));last_robot=signature
        elif topic=='/agent/recovery_events':
            if d.get('type') in ('execution_started','execution_succeeded','execution_failed','action_auto_approved','incident_acknowledged','incident_ack_cleared'):
                pb.append(dict(wall=t,from_injection_wall_sec=t-trigger,data=d))
        elif topic=='/agent/mission_events':
            if d.get('type') in ('mission_recovery_resume_authorized','skill_finished','mission_finished'):
                mission.append(dict(wall=t,data=d))
    return dict(trial=trial,injection_start=config,injection_end=completed,
        injection_metrics=dict(input_frames=frames,dropped_frames=dropped,drop_fraction=dropped/frames if frames else None,
            measured_forwarded_frames=measured,forwarded_frames_missing_metrics=missing_metrics,
            input_valid_points_on_measured_forwarded_frames=valid,occluded_valid_points=occluded,
            occluded_fraction_valid_on_forwarded_frames=occluded/valid if valid else None,
            retained_points=retained,radial_noise_mean_m=noise_sum/retained if retained else None,
            radial_noise_rmse_m=math.sqrt(noise_sq/retained) if retained else None,
            xyz_perturbation_rmse_m=math.sqrt(xyz_sq/retained) if retained else None,
            max_input_source_gap_sim_sec=max_source_gap,max_processing_wall_sec=max_processing),
        topic_counts=dict(counts),ranges=ranges,item_sample_counts=item_counts,item_transitions=transitions,
        robot_summary_transitions=robot,readiness_transitions=readiness,recovery_events=pb,mission_events=mission,match_results=match_results,
        max_internal_receive_gap_wall_sec=gaps,
        edge_unobserved_wall_sec={k:dict(start_to_first=first_received[k]-start,last_to_end=end-last_received[k]) for k in first_received},
        wheel_odom_path_m=path if wheel_first else None,
        wheel_odom_net_displacement_m=math.dist(wheel_first,wheel_last) if wheel_first else None,
        limits=['Wheel motion and map/odom correction are proxies, NOT true drift or absolute error.',
                'Occlusion ratio excludes preexisting invalid points and whole-frame drops; angle is not point fraction.',
                'Noise statistics use retained forwarded points after range clipping; configured sigma is different.',
                'All ranges cover explicit task window; frame perturbation metrics cover injection only.',
                'Missing topics/metrics are unobserved, not healthy or zero. Correlation does not prove causation.'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','injection-log','trials','output'):p.add_argument('--'+name,required=True)
    a=p.parse_args();dest=Path(a.output);dest.mkdir(parents=True,exist_ok=True)
    trials=json.loads(Path(a.trials).read_text())
    results=[summarize(a.evidence,a.injection_log,t) for t in trials]
    result_path=dest/'comparison.json'
    with result_path.open('x') as f:json.dump(dict(evidence=a.evidence,injection_log=a.injection_log,results=results),f,indent=2,ensure_ascii=False)
    lines=['# 分级注入对比（自动统计，需结合原始事件解释）','',
        '| 案例 | profile | 输入/丢帧 | 实际遮挡比例¹ | 实测径向噪声 RMSE¹ | raw score 范围 | map↔odom 修正变化范围² | PB 开始次数 |',
        '|---|---|---|---|---|---|---|---|']
    for r in results:
        m=r['injection_metrics'];rr=r['ranges']
        lines.append(f"| {r['trial']['label']} | {r['injection_start'].get('profile')} | {m['input_frames']}/{m['dropped_frames']} | {m['occluded_fraction_valid_on_forwarded_frames']} | {m['radial_noise_rmse_m']} | {rr.get('raw_rmse_m')} | {rr.get('map_odom_correction_change_m')} | {sum(e['data'].get('type')=='execution_started' for e in r['recovery_events'])} |")
    lines+=['','¹ 只在有测量的转发帧及有效点上统计，整帧丢弃单独计；None 为未观测。',
            '² 相对窗口首个 accepted_state 的修正变化，不是真实定位误差。',
            '逐项 Health 转移、readiness、ignore、Recovery 与 Mission 事件见 comparison.json；样本数不是持续时间。',
            'PB 可能在 BAD 消失后仍处于验证阶段，应以启动事件对应的根因与当时状态判断，不按总摘要 WARN 推断违规。']
    (dest/'comparison.md').write_text('\n'.join(lines)+'\n')
    print(result_path)

if __name__=='__main__':main()
