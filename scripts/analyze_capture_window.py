#!/usr/bin/env python3
"""Separate official first-capture window from preserved 300s continuation.

This does not rewrite result.json, change Monitor thresholds, or turn a failed
full continuation benchmark into a pass. Post-capture scores cannot be used as
scores at official match end because per-tick report payloads were not stored.
"""
import argparse
import gzip
import json
from pathlib import Path
from compare_sim import Monitor


def analyze(folder):
    folder=Path(folder);r=json.loads((folder/'result.json').read_text(encoding='utf-8'))
    agents=r['score']['agents']
    arrivals=[(a['m5_reached'],uid) for uid,a in agents.items() if a.get('m5_reached') is not None]
    first=min(arrivals) if arrivals else None
    cutoff=first[0] if first else 300.
    winner=[camp for camp,prefix in [('BLUE','B'),('RED','R')]
            if any(a.get('m5_points')==10 for uid,a in agents.items() if uid.startswith(prefix))]
    monitor=Monitor()
    for line in gzip.open(folder/'ticks.jsonl.gz','rt',encoding='utf-8'):
        row=json.loads(line)
        if row['t']>cutoff+1e-8:break
        monitor.update(row,row['evaluation_only'])
    before=monitor.result();full=r['diagnostics']
    report=dict(metric_version=r['metric_version'],variant=r['variant'],scenario=r['scenario'],seed=r['seed'],
        rule_source={'file':'docs/API_SOURCE_EXTRACT.txt','lines':[387,454],
                     'meaning':'First capture by either alliance ends play; UGVs lock and reporting starts.'},
        original_fixture_m5_ends_match=False,original_300s_metrics_preserved=True,
        first_capture_proxy_s=first[0] if first else None,first_sensor_occupant=first[1] if first else None,
        beneficiary_camp=winner[0] if len(winner)==1 else None,
        time_resolution_s=.1,source='Simulator referee m5_reached and alliance m5_points; not real sensor timestamp',
        official_window_diagnostics=before,
        continuation_after_capture=dict(start_s=cutoff,deadlock_events=[e for e in full['deadlock_events'] if e['t']>cutoff+1e-8],
                                        oscillation_events=[e for e in full['oscillation_events'] if e['t']>cutoff+1e-8]),
        full_continuation_diagnostics=full,
        individual_arrival_completion={uid:a.get('m5_reached') for uid,a in agents.items()},
        official_end_m2_m3_score='unverified: only final300s score saved; do not substitute post-end scores',
        acceptance='No global pass: retain original300s diagnostics and assess official window separately')
    (folder/'capture_window_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folders',nargs='+');args=p.parse_args()
    for folder in args.folders:
        r=analyze(folder);d=r['official_window_diagnostics']
        print(json.dumps(dict(variant=r['variant'],first_capture=r['first_capture_proxy_s'],camp=r['beneficiary_camp'],
            official_window_deadlocks=len(d['deadlock_events']),flips=len(d['oscillation_events']),stall=d['stall_ratio'],
            post_capture_deadlocks=len(r['continuation_after_capture']['deadlock_events'])),indent=2))


if __name__=='__main__':main()
