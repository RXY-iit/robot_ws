import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('graded_summary',Path(__file__).resolve().parents[3]/'tools/sim/analysis/summarize_graded_trials.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def test_comparison_preserves_missing_and_weighted_occlusion(tmp_path):
    injection=[dict(event='injection_started',run=1,wall_unix_sec=10,profile='occlusion'),
        dict(event='cloud',run=1,elapsed_sim_sec=1,wall_unix_sec=11,dropped=False,source_stamp_sec=1,metrics=dict(input_valid_points=10,occluded_valid_points=4,noise_sample_points=6,radial_noise_sum_m=0,radial_noise_sumsq_m2=6,xyz_perturbation_sumsq_m2=24)),
        dict(event='cloud',run=1,elapsed_sim_sec=2,wall_unix_sec=12,dropped=True,source_stamp_sec=2,metrics={}),
        dict(event='schedule_completed',run=1,wall_unix_sec=12)]
    evidence=[dict(event='/gicp_loc/score',wall_unix_sec=11,data={'data':.14}),
              dict(event='/gicp_loc/score',wall_unix_sec=12,data={'data':.21}),
              dict(event='/gicp_loc/score',wall_unix_sec=100,data={'data':9})]
    a=tmp_path/'i';b=tmp_path/'e'
    a.write_text(''.join(json.dumps(r)+'\n' for r in injection));b.write_text(''.join(json.dumps(r)+'\n' for r in evidence))
    r=m.summarize(b,a,dict(label='run1',injection_run=1,start_wall=9,end_wall=13))
    assert r['injection_metrics']['drop_fraction']==.5
    assert r['injection_metrics']['occluded_fraction_valid_on_forwarded_frames']==.4
    assert r['injection_metrics']['radial_noise_rmse_m']==1
    assert r['injection_metrics']['xyz_perturbation_rmse_m']==2
    assert r['ranges']['raw_rmse_m']==[.14,.21]
    assert r['wheel_odom_path_m'] is None
    assert not r['robot_summary_transitions']
    assert r['edge_unobserved_wall_sec']['/gicp_loc/score']==dict(start_to_first=2,last_to_end=1)
    with pytest.raises(ValueError):m.summarize(b,a,dict(label='x',injection_run=2,start_wall=9,end_wall=13))
