import importlib.util
import json
from pathlib import Path

spec=importlib.util.spec_from_file_location('compare_sim',Path(__file__).resolve().parents[1]/'scripts'/'compare_sim.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sample(t,**over):
    row=dict(t=t,robot='B2',posture='AVOID',level=0,goal=[2.,2.],remaining_m=2.,
             confirmed_faces=0,observed_faces=0,shots=0,normal_wait=False,mode='EXPLORE',v=.1)
    row.update(over)
    return row


TRUTH=dict(pose=[0.,0.,0.],nearest_ugv_m=2.)


def test_acceptance_monitor_keeps_requested_twelve_second_threshold():
    monitor=module.Monitor()
    for t in range(13): monitor.update(sample(t),TRUTH)
    assert monitor.events==[]
    monitor.update(sample(12.1),TRUTH)
    assert len(monitor.events)==1
    assert monitor.events[0]['t']==12.1


def test_goal_churn_cannot_hide_deadlock_and_valid_face_breaks_it():
    monitor=module.Monitor()
    for t in range(14): monitor.update(sample(t,goal=[float(t),2.]),TRUTH)
    assert len(monitor.events)==1
    monitor.update(sample(14,observed_faces=1),TRUTH)
    assert monitor.states['B2']['stalled']==0


def test_normal_observation_pauses_stall_without_erasing_previous_stall():
    monitor=module.Monitor()
    for t in range(11): monitor.update(sample(t),TRUTH)
    for t in range(11,21): monitor.update(sample(t,normal_wait=True),TRUTH)
    assert not monitor.events
    monitor.update(sample(21),TRUTH)
    monitor.update(sample(22),TRUTH)
    monitor.update(sample(23),TRUTH)
    assert len(monitor.events)==1


def test_movement_reversals_record_decision_oscillation():
    monitor=module.Monitor()
    for t,v in enumerate((.1,-.1,.1,-.1)):
        monitor.update(sample(t,v=v),TRUTH)
    assert len(monitor.flips)==1


def test_server_report_reset_is_not_decision_oscillation():
    monitor=module.Monitor()
    for t,level in enumerate((0,1,2,3)):
        monitor.update(sample(t,level=level),TRUTH)
    monitor.update(sample(4,level=0,mode='REPORT',normal_wait=True),TRUTH)
    assert not monitor.flips


def test_six_agent_selfplay_and_mixed_allies_are_distinct():
    from maicon26.config import Config
    cfg=Config();cfg.depth.enable=False
    cfg.progress.enabled=False
    selfplay,dummies=module.build_scenario('selfplay',7,cfg)
    assert len(selfplay.agents)==6 and len(selfplay.world.ugvs)==6 and not dummies
    assert selfplay.world.m5_ends_match is True
    mixed,dummies=module.build_scenario('mixed_allies',7,cfg)
    assert [a.name for a in mixed.agents]==['B-UGV-2']
    assert len(mixed.world.ugvs)==6 and len(dummies)==5
    diagnostic,_=module.build_scenario('selfplay',7,cfg,continue_after_capture=True)
    assert diagnostic.world.m5_ends_match is False


def test_resume_requires_exact_source_revision(tmp_path):
    policy='official_first_capture_or_300s'
    (tmp_path/'result.json').write_text(json.dumps(dict(metric_version=module.METRIC_VERSION,termination_policy=policy)))
    (tmp_path/'source_hashes.json').write_text(json.dumps({'manager.py':'source02'}))
    assert module.reusable_case(tmp_path,{'manager.py':'source02'},policy)[0]
    ok,reason=module.reusable_case(tmp_path,{'manager.py':'source03'},policy)
    assert not ok and reason=='superseded source revision'


def test_archived_same_metric_result_never_reenters_summary(tmp_path):
    archived=tmp_path/'progress/selfplay/seed_00_superseded_source';archived.mkdir(parents=True)
    # Deliberately minimal: archived results must be excluded before metrics
    # are read, even when their schema version is current.
    (archived/'result.json').write_text(json.dumps({'metric_version':module.METRIC_VERSION}))
    assert module.summarize(tmp_path)==[]
    manifest=json.loads((tmp_path/'manifest.json').read_text())
    assert manifest['excluded_diagnostics'][0]['reason']=='archived result; never an active case'


def test_derived_metric_requires_exact_preserved_trace_and_configuration(tmp_path):
    import hashlib
    result=dict(metric_version=5,diagnostics={'old':'preserved'})
    rp=tmp_path/'result.json';rp.write_text(json.dumps(result))
    tp=tmp_path/'ticks.jsonl.gz';tp.write_bytes(b'preserved compressed trace')
    cp=tmp_path/'configuration.json';cp.write_text('{"seed":0}')
    original=rp.read_bytes()
    analysis=dict(analysis_version=6,original_metric_version=5,valid_for_acceptance=False,stage_wait_correction_verified=True,
        original_result_sha256=hashlib.sha256(original).hexdigest(),
        original_ticks_sha256=hashlib.sha256(tp.read_bytes()).hexdigest(),
        original_configuration_sha256=hashlib.sha256(cp.read_bytes()).hexdigest(),
        diagnostics={'corrected':'wait proximity'})
    ap=tmp_path/'analysis_v6.json';ap.write_text(json.dumps(analysis))
    assert module.read_case_result(tmp_path)[0] is None
    derived,reason=module.read_stage_corrected_result(tmp_path)
    assert reason is None and derived['metric_version']==6
    assert not derived['strict_acceptance_eligible']
    assert derived['diagnostics']==analysis['diagnostics']
    assert rp.read_bytes()==original
    cp.write_text('{"seed":1}')
    assert module.read_stage_corrected_result(tmp_path)==(None,'derived analysis configuration identity mismatch')
    cp.write_text('{"seed":0}')
    tp.write_bytes(b'changed trace')
    assert module.read_stage_corrected_result(tmp_path)==(None,'derived analysis trace identity mismatch')
    tp.write_bytes(b'preserved compressed trace')
    analysis['stage_wait_correction_verified']=False;ap.write_text(json.dumps(analysis))
    assert module.read_stage_corrected_result(tmp_path)==(None,'derived analysis has unresolved wait causes')


def test_stage_wait_requires_arrival_and_route_attribution_uses_production_selector():
    from types import SimpleNamespace as N
    a=N(cfg=N(progress=N(enabled=True)),deadlock_telemetry=lambda:dict(goal=[2.,0.],normal_wait=False,observed_faces=0),
        stack=N(pose=N(x=0.,y=0.,th=0.),navigator=N(path=[])),name='B2',mode=N(value='STAGE'),
        _v_cmd=0.,_w_cmd=0.,robot=N(status=lambda:N(hits_taken=0)),
        laser=N(shots=0,firing=lambda:False),omap=N(confirmed_faces=lambda:[]),
        coverage=N(boxes={}),stats={},_rc={'path':[(0.,0.),(1.,1.)]},
        _ex_cand=N(path=[(0.,0.),(0.,2.),(2.,2.),(2.,0.)]),
        _ex_phase='travel',t_start=0.,drive_locked=lambda now:False)
    far=module.telemetry(a,230.)
    assert not far['normal_wait'] and far['stage_goal_distance_m']==2.
    assert far['remaining_path_source']=='euclidean_fallback'
    a._remaining_path_route=lambda goal:('exploration',a._ex_cand.path)
    assert module.telemetry(a,230.)['remaining_path_source']=='explore_path'
    a.stack.pose.x=1.71
    arrived=module.telemetry(a,230.)
    assert arrived['normal_wait'] and 'stage_arrived' in arrived['normal_wait_reasons']
    a.stack.pose.x=1.69
    assert not module.telemetry(a,230.)['normal_wait']
    a.laser.firing=lambda:True
    assert 'laser_active' in module.telemetry(a,230.)['normal_wait_reasons']


def test_unscored_and_long_structure_observations_cannot_erase_stall():
    from types import SimpleNamespace as N
    box=N(available=True,eligible=False,sector='R-S1',faces=[])
    own_box=N(available=True,eligible=True,sector='B-S1',faces=[])
    missing=N(available=False,eligible=True,sector='R-S1',faces=[])
    coverage=N(boxes={1:box,2:own_box,3:missing},box_available=lambda b:b.available,
        m3_candidate_box=lambda b:b.eligible,box_sector=lambda b:b.sector)
    a=N(cfg=N(progress=N(enabled=True)),team='BLUE',
        deadlock_telemetry=lambda:dict(goal=[2.,0.],normal_wait=False,remaining_m=2.,posture='AVOID',level=0),
        stack=N(pose=N(x=0.,y=0.,th=0.),navigator=N(path=[])),name='B2',mode=N(value='EXPLORE'),
        _v_cmd=.1,_w_cmd=0.,robot=N(status=lambda:N(hits_taken=0)),
        laser=N(shots=0,firing=lambda:False),omap=N(confirmed_faces=lambda:[]),
        coverage=coverage,stats={},_rc={},_ex_cand=None,_ex_phase='travel',t_start=0.,
        drive_locked=lambda now:False)
    monitor=module.Monitor()
    for t in range(14):
        if t==6:
            for b in coverage.boxes.values():b.faces.append(N(views=1))
        row=module.telemetry(a,t)
        assert row['observed_faces']==0
        monitor.update(row,TRUTH)
    assert row['all_observed_faces']==3 and len(monitor.events)==1
    # Native production telemetry must never be overwritten by a later
    # read of the full coverage ledger.
    a.deadlock_telemetry=lambda:dict(goal=[2.,0.],normal_wait=False,observed_faces=7)
    assert module.telemetry(a,14.)['observed_faces']==7
    a._ex_cand=N(path=[]);a._ex_phase='observe';a._ex_obs_t0=0.;a._v_cmd=0.
    assert module.telemetry(a,12.)['normal_wait']
    expired=module.telemetry(a,12.1)
    assert not expired['normal_wait'] and expired['observation_age_s']==12.1
