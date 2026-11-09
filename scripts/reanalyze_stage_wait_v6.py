#!/usr/bin/env python3
"""Correct metric5's overly broad STAGE wait exemption without rerunning policy.

Writes analysis_v6.json only; original results, source, traces and video stay
untouched. Unknown wait causes remain unknown and cannot yield acceptance.
"""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
import inspect
from pathlib import Path
from compare_sim import Monitor,source_fingerprints


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analyze(folder):
    folder=Path(folder);rp=folder/'result.json';tp=folder/'ticks.jsonl.gz';cp=folder/'configuration.json'
    result=json.loads(rp.read_text(encoding='utf-8'));cfg=json.loads(cp.read_text(encoding='utf-8'))
    if result.get('metric_version')!=5:raise ValueError('This correction applies only to complete recorded metric5 trials')
    source=Path(result['source_root']);proof={}
    hashes=json.loads((folder/'source_hashes.json').read_text(encoding='utf-8'))
    proof['source_snapshot_unchanged']=source.is_dir() and source_fingerprints(source)==hashes
    proof['known_no_injected_events_fixture']=result['scenario'] in ('selfplay','goal_race')
    proof['no_hits']=all(v['hits_taken']==0 for v in result['score']['agents'].values())
    proof['blocker_fire_off']=not cfg.get('engage',{}).get('fire_on_blocker',False)
    if source.is_dir():
        manager=(source/'maicon26/mission/manager.py').read_text(encoding='utf-8')
        world=(source/'maicon26/sim/world.py').read_text(encoding='utf-8')
        proof['active_strike_firing_precedes_stage']=('if self.mode in (Mode.STRIKE, Mode.HUNT) and firing:' in manager)
        proof['observation_phase_reset_on_mode_change']=('self._ex_cand, self._ex_phase = None, "travel"' in manager)
        assignments=[line.strip() for line in world.splitlines() if 'paused_left =' in line]
        proof['only_drive_pause_assignment_is_hit']=assignments==['self.paused_left = 0.0','v.paused_left = self.cfg.laser.disabled_s']
    supported=all(proof.values()) and len(proof)==7
    lower=Monitor();upper=Monitor();corrected=[];unknown=[];times={};removed_s=0.
    for line in gzip.open(tp,'rt',encoding='utf-8'):
        row=json.loads(line);uid=row['robot'];t=row['t'];dt=max(0.,t-times.get(uid,t));times[uid]=t
        row=dict(row);uncertain=False
        legacy=(row['mode']=='STAGE' and row['normal_wait'] and 225<=t<240 and abs(row.get('v',0))<.01)
        if legacy:
            goal=row.get('goal');pose=row.get('pose')
            distance=math.dist(pose[:2],goal) if goal is not None and pose is not None else None
            if distance is None:
                uncertain=True
            elif distance>.3:
                if supported:
                    row['normal_wait']=False;removed_s+=dt
                    corrected.append(dict(t=t,robot=uid,goal=goal,distance_m=distance,seconds=dt))
                else:uncertain=True
            if uncertain:unknown.append(dict(t=t,robot=uid,distance_m=distance,seconds=dt))
        lower.update(row,row['evaluation_only'])
        high=dict(row)
        if uncertain:high['normal_wait']=False
        upper.update(high,high['evaluation_only'])
    report=dict(analysis_version=6,original_metric_version=5,
        origin='Derived from complete saved controller run; policy and observations were not rerun or changed',
        correction='225–240s STAGE v≈0 is normal waiting only within0.3m of selected goal; other wait causes require evidence',
        original_result_sha256=sha(rp),original_ticks_sha256=sha(tp),original_configuration_sha256=sha(cp),
        analyzer_sha256=sha(__file__),monitor_sha256=hashlib.sha256(inspect.getsource(Monitor).encode()).hexdigest(),
        evidence=proof,evidence_scope='Frozen zero-hit 2D selfplay fixtures; drive pause comes only from hits, active firing keeps STRIKE/HUNT, observation phase resets on STAGE entry',
        stage_wait_correction_verified=not unknown,
        valid_for_acceptance=False,
        acceptance_limitation='Historical counter includes all viewed ledger faces, not only scoring-eligible available M3 faces; per-tick scoring history was not recorded and cannot be reconstructed from one snapshot',
        unknown_wait_rows=len(unknown),unknown_wait_seconds=sum(v['seconds'] for v in unknown),
        removed_wait_rows=len(corrected),removed_wait_seconds=removed_s,corrections=corrected,
        diagnostics=lower.result(),upper_bound_if_unknown_waits_are_active=upper.result(),unknown_waits=unknown,
        original_diagnostics=result['diagnostics'],thresholds_unchanged=dict(deadlock_s=12.,stall_start_s=3.,max_stall_ratio=.05))
    (folder/'analysis_v6.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    d=report['diagnostics'];print(json.dumps(dict(case=str(folder),valid=report['valid_for_acceptance'],
        removed_wait_s=removed_s,unknown_wait_rows=len(unknown),deadlocks=len(d['deadlock_events']),
        flips=len(d['oscillation_events']),stall_ratio=d['stall_ratio']),indent=2))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('cases',nargs='+');args=p.parse_args()
    for case in args.cases:analyze(case)


if __name__=='__main__':main()
