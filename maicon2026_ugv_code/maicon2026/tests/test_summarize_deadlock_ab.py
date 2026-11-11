import importlib.util
import json
from pathlib import Path
import sys

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
spec=importlib.util.spec_from_file_location('paired_ab',SCRIPTS/'summarize_deadlock_ab.py')
ab=importlib.util.module_from_spec(spec);spec.loader.exec_module(ab)


def fixture(root,variant,seed=0,arrival=245.,revision='same'):
    p=root/variant/'selfplay'/f'seed_{seed:02d}';p.mkdir(parents=True)
    result=dict(metric_version=ab.METRIC_VERSION,simulated_t=(arrival if arrival is not None else 300.)+.4,
        termination_policy='official_first_capture_or_300s',
        first_capture=dict(t=arrival,camp='BLUE',by='B2') if arrival is not None else None,
        score={'t_end':arrival if arrival is not None else 300.,'agents':{'B2':dict(m2_points=10,m3={'points':30},hits_taken=0,m5_reached=arrival)}},
        diagnostics=dict(deadlock_events=[],oscillation_events=[],categories={},stall_ratio=.01),
        coverage_210={'B2':{'coverage':1.}})
    (p/'result.json').write_text(json.dumps(result),encoding='utf-8')
    (p/'source_hashes.json').write_text(json.dumps({'manager.py':revision}),encoding='utf-8')


def test_missing_candidate_is_unknown_not_accepted(tmp_path):
    fixture(tmp_path,'v2')
    r=ab.evaluate(tmp_path,variants=('v2','progress'),seeds=[0],scenarios=('selfplay',))
    assert r['completed_valid']==1
    assert not r['all_requested_criteria_passed']
    assert r['missing']==[['progress','selfplay',0]]


def test_lost_first_m5_capture_cannot_hide_in_success_only_mean(tmp_path):
    fixture(tmp_path,'v2');fixture(tmp_path,'progress',arrival=None)
    r=ab.evaluate(tmp_path,variants=('v2','progress'),seeds=[0],scenarios=('selfplay',))
    pair=next(c for c in r['cases'] if c['variant']=='progress')['paired']
    assert pair['m2_m3_no_worse']
    assert pair['m5_v2_first_capture']['t']==245.
    assert pair['m5_candidate_first_capture'] is None
    assert not pair['m5_no_worse']
    assert not r['all_requested_criteria_passed']


def test_continuation_cannot_enter_official_matrix(tmp_path):
    fixture(tmp_path,'v2')
    p=tmp_path/'v2/selfplay/seed_00/result.json'
    result=json.loads(p.read_text());result['termination_policy']='diagnostic_continue_after_capture'
    p.write_text(json.dumps(result))
    r=ab.evaluate(tmp_path,variants=('v2',),seeds=[0],scenarios=('selfplay',))
    assert r['completed_valid']==0
    assert r['rejected'][0]['reason']=='nonofficial continuation fixture'


def test_historical_unscored_face_progress_never_counts_as_strict_completion(tmp_path):
    fixture(tmp_path,'v2')
    p=tmp_path/'v2/selfplay/seed_00/result.json'
    result=json.loads(p.read_text());result['metric_version']=6
    p.write_text(json.dumps(result))
    r=ab.evaluate(tmp_path,variants=('v2',),seeds=[0],scenarios=('selfplay',))
    assert r['completed_valid']==0 and not r['all_requested_criteria_passed']
    assert 'unscored or unavailable structure faces' in r['rejected'][0]['reason']


def test_earlier_opponent_capture_is_not_blue_improvement():
    baseline=dict(m5_arrivals={'B2':245.},first_capture=dict(t=245.,camp='BLUE'),
                  termination_policy='official_first_capture_or_300s',m2_m3=30.,hits=0)
    candidate=dict(baseline,m5_arrivals={'B2':None},first_capture=dict(t=242.,camp='RED'))
    pair=ab.pair_checks(candidate,baseline,'mixed_allies')
    assert pair['m5_blue_win_regression']
    assert not pair['m5_no_worse']


def test_mixed_candidate_revisions_never_global_pass(tmp_path):
    for seed in (0,1):
        fixture(tmp_path,'v2',seed,revision='frozen_v2')
        fixture(tmp_path,'progress',seed,revision=f'candidate_{seed}')
    r=ab.evaluate(tmp_path,variants=('v2','progress'),seeds=[0,1],scenarios=('selfplay',))
    assert all(s['accepted'] for s in r['summaries'] if s['variant']=='progress')
    assert not r['fixed_source_revisions']
    assert not r['all_requested_criteria_passed']
