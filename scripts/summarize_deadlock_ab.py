#!/usr/bin/env python3
"""Strict paired acceptance for completed, versioned 2D sensor-model trials.

Missing/obsolete results remain unknown. Videos and exit status are not proof
of acceptance. This reads evaluator truth only, never supplies it to an Agent.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from statistics import mean

from compare_sim import METRIC_VERSION, SCENARIOS, VARIANTS, read_case_result


def trial_metrics(result):
    agents=result['score']['agents']; d=result['diagnostics']
    coverage=result.get('coverage_210') or {}
    return dict(
        deadlocks=len(d['deadlock_events']), flips=len(d['oscillation_events']),
        stall_ratio=d['stall_ratio'], categories=d['categories'],
        coverage_min=min((v['coverage'] for v in coverage.values()),default=None),
        coverage_complete=set(coverage)==set(agents),
        m2_m3=mean(v['m2_points']+(v.get('m3') or {}).get('points',0) for v in agents.values()),
        hits=mean(v['hits_taken'] for v in agents.values()),
        m5_arrivals={uid:v.get('m5_reached') for uid,v in agents.items()},
        first_capture=result.get('first_capture'),termination_policy=result.get('termination_policy'),
        m5_successes=sum(v.get('m5_reached') is not None for v in agents.values()),
        agent_count=len(agents))


def absolute_checks(metrics):
    return dict(zero_deadlocks=metrics['deadlocks']==0,
                stall_at_most_5pct=metrics['stall_ratio']<=.05,
                zero_oscillation=metrics['flips']==0,
                every_scored_face_at_210s=metrics['coverage_complete'] and metrics['coverage_min']==1.)


def pair_checks(candidate,baseline,scenario):
    # A failed arrival cannot disappear by averaging successful arrivals only.
    same_agents=set(candidate['m5_arrivals'])==set(baseline['m5_arrivals'])
    regressions=[]
    for uid,t0 in baseline['m5_arrivals'].items():
        t1=candidate['m5_arrivals'].get(uid)
        if t0 is not None and (t1 is None or t1>t0+1e-8):
            regressions.append(dict(robot=uid,v2=t0,candidate=t1))
    official=(candidate.get('termination_policy')==baseline.get('termination_policy')=='official_first_capture_or_300s')
    c_capture=candidate.get('first_capture');b_capture=baseline.get('first_capture')
    first_no_worse=(b_capture is None or (c_capture is not None and c_capture['t']<=b_capture['t']+1e-8))
    # In mixed/dummy scenarios BLUE is the tested policy. An earlier RED
    # capture must not be presented as improved M5 success for BLUE.
    blue_win_regression=(scenario not in ('selfplay','goal_race') and b_capture is not None
                         and b_capture.get('camp')=='BLUE' and
                         (c_capture is None or c_capture.get('camp')!='BLUE'))
    return dict(m2_m3_no_worse=candidate['m2_m3']>=baseline['m2_m3']-1e-8,
                hunter_hits_delta=candidate['hits']-baseline['hits'],
                hunter_hits_at_most_plus_point3=(candidate['hits']-baseline['hits']<=.3+1e-8) if scenario=='vs_hunters' else None,
                m5_comparison=('first official sensor capture time; individual later arrivals are not required' if official else 'individual continuation arrivals'),
                m5_no_worse=(first_no_worse and not blue_win_regression) if official else same_agents and not regressions,
                m5_blue_win_regression=blue_win_regression if official else None,
                m5_v2_first_capture=b_capture,m5_candidate_first_capture=c_capture,
                m5_regressions=[] if official else regressions)


def evaluate(root,variants=VARIANTS,seeds=range(20),scenarios=SCENARIOS):
    root=Path(root); cases={}; rejected=[]; missing=[]; revisions={v:set() for v in variants}
    for variant in variants:
        for scenario in scenarios:
            for seed in seeds:
                key=(variant,scenario,seed); p=root/variant/scenario/f'seed_{seed:02d}'/'result.json'
                if not p.exists():
                    missing.append(list(key));continue
                result,reason=read_case_result(p.parent)
                if result is None:
                    rejected.append(dict(case=list(key),reason=reason));continue
                if result.get('termination_policy')!='official_first_capture_or_300s':
                    rejected.append(dict(case=list(key),reason='nonofficial continuation fixture'));continue
                capture=result.get('first_capture');end=result.get('score',{}).get('t_end')
                if end is None or result.get('simulated_t',0)<end or (end<300.-1e-8 and
                    (not capture or capture.get('t',0)<240.-1e-8 or abs(capture['t']-end)>.11)):
                    rejected.append(dict(case=list(key),reason='incomplete or unexplained early fixture ending'));continue
                h=p.parent/'source_hashes.json'
                if not h.exists():
                    rejected.append(dict(case=list(key),reason='missing source identity'));continue
                hashes=json.loads(h.read_text(encoding='utf-8'))
                revision=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest()
                revisions[variant].add(revision)
                metrics=trial_metrics(result)
                cases[key]=dict(variant=variant,scenario=scenario,seed=seed,metrics=metrics,
                                source_revision=revision,absolute=absolute_checks(metrics),
                                analysis_origin=result.get('analysis_origin','original run instrumentation'))
    for key,row in cases.items():
        v,scenario,seed=key
        baseline=cases.get(('v2',scenario,seed))
        row['paired']=pair_checks(row['metrics'],baseline['metrics'],scenario) if v!='v2' and baseline else None
    summaries=[]
    for variant in variants:
        for scenario in scenarios:
            group=[row for (v,s,_),row in cases.items() if v==variant and s==scenario]
            if not group:continue
            ms=[r['metrics'] for r in group]; pairs=[r['paired'] for r in group if r['paired']]
            all_complete=len(group)==len(seeds)
            abs_pass=all(all(r['absolute'].values()) for r in group)
            pair_pass=(len(pairs)==len(seeds) and all(p['m2_m3_no_worse'] and p['m5_no_worse'] and
                        p['hunter_hits_at_most_plus_point3'] is not False for p in pairs)) if variant!='v2' else None
            summaries.append(dict(variant=variant,scenario=scenario,completed=len(group),required=len(seeds),
                deadlocks=sum(m['deadlocks'] for m in ms),flips=sum(m['flips'] for m in ms),
                stall_mean=mean(m['stall_ratio'] for m in ms),stall_max=max(m['stall_ratio'] for m in ms),
                coverage_min=min((m['coverage_min'] for m in ms if m['coverage_min'] is not None),default=None),
                m2_m3_mean=mean(m['m2_m3'] for m in ms),hits_mean=mean(m['hits'] for m in ms),
                m5_successes=sum(m['m5_successes'] for m in ms),agent_runs=sum(m['agent_count'] for m in ms),
                paired_cases=len(pairs),absolute_pass_on_completed=abs_pass,paired_pass=pair_pass,
                accepted=all_complete and abs_pass and pair_pass is True))
    fixed_revisions=all(len(values)<=1 for values in revisions.values())
    candidate_revisions=set().union(*(values for key,values in revisions.items() if key!='v2'))
    fixed_revisions=fixed_revisions and len(candidate_revisions)<=1
    candidates=[s for s in summaries if s['variant']!='v2']
    expected=len(variants)*len(scenarios)*len(seeds)
    accepted=(len(cases)==expected and fixed_revisions and bool(candidates) and all(s['accepted'] for s in candidates))
    report=dict(backend='2D synthetic sensors; not Gazebo',metric_version=METRIC_VERSION,
        expected_cases=expected,completed_valid=len(cases),counts_by_variant=dict(Counter(k[0] for k in cases)),
        missing=missing,rejected=rejected,source_revisions={k:sorted(v) for k,v in revisions.items()},
        fixed_source_revisions=fixed_revisions,all_requested_criteria_passed=accepted,
        termination_policy='official_first_capture_or_300s',
        fire_on_blocker_decision='keep disabled until all requested A/B evidence passes',
        summaries=summaries,cases=list(cases.values()))
    (root/'paired_acceptance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',nargs='?',default='recordings/deadlock_official_ab_20261109')
    args=p.parse_args(); r=evaluate(args.root)
    print(json.dumps({k:r[k] for k in ('expected_cases','completed_valid','counts_by_variant','all_requested_criteria_passed')},indent=2))


if __name__=='__main__':main()
