#!/usr/bin/env python3
"""Budget estimate from completed artifacts; not a throughput benchmark."""
import argparse
import json
from pathlib import Path


def folder_bytes(folder):
    return sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())


def forecast(root):
    root=Path(root)
    samples=[]
    for label,relative in (
        ('v2','deadlock_official_ab_20261109/v2/selfplay/seed_00_legacy_metric_5'),
        ('candidate02','deadlock_official_ab_20261109/progress/selfplay/seed_00_superseded_source'),
        ('candidate03','deadlock_official_ab_20261109/progress/selfplay/seed_00_legacy_metric_5')):
        path=root/relative;r=json.loads((path/'result.json').read_text())
        samples.append(dict(label=label,path=str(path),wall_s=r['wall_s'],
            simulation_s=r['simulated_t'],bytes=folder_bytes(path),
            measurement_scope='run loop only; excludes process startup and final encoder/report cleanup'))
    path=root/'wingbot_selfplay_20261109_candidate03'
    r=json.loads((path/'selfplay_report.json').read_text())
    gazebo=dict(path=str(path),wall_s=r['wall_s'],
        simulation_s=r['end_sim_s']-r['match_start_sim_s'],bytes=folder_bytes(path),
        measurement_scope='coordinator wall time includes readiness and final agent shutdown')
    base=samples[0];candidate=samples[-1]
    paired_seconds=120*base['wall_s']+360*candidate['wall_s']
    full_seconds=120*base['wall_s']*300/base['simulation_s']+360*candidate['wall_s']*300/candidate['simulation_s']
    return dict(scope='Planning estimate only; three single-seed 2D trials and one Gazebo trial, not all scenarios',
        units='decimal GB (1e9 bytes), hours (3600 seconds)',samples_2d=samples,sample_gazebo=gazebo,
        full_2d=dict(cases=480,sequential_hours_at_observed_early_capture=paired_seconds/3600,
            sequential_hours_if_300s_linear=full_seconds/3600,
            ideal_four_worker_hours_at_observed_early_capture=paired_seconds/14400,
            storage_GB=(120*base['bytes']+360*candidate['bytes'])/1e9,
            storage_sample_range_GB=[480*min(s['bytes'] for s in samples)/1e9,480*max(s['bytes'] for s in samples)/1e9]),
        full_gazebo=dict(cases=240,sequential_hours_at_observed_early_capture=240*gazebo['wall_s']/3600,
            sequential_hours_if_300s_linear=240*gazebo['wall_s']*300/gazebo['simulation_s']/3600,
            storage_GB=240*gazebo['bytes']/1e9),
        limitations=[
            'No linear parallel speedup was measured; four-worker number is an optimistic arithmetic bound.',
            'Different scenarios, policies, seeds, target count, host load and early capture alter runtime and storage.',
            'No paused or interrupted case is included in wall-time samples.',
            'Artifacts retain all original traces/video; shared source snapshots and future retries are additional.',
            'Two-dimensional v2 artifact predates snapshot210 export, so comparable future v2 storage may be larger.',
            'Gazebo timing depends on CPU/GPU sensor load; concurrent batches can change latency and should not be used for isolated latency claims.',
            'Scaling startup overhead linearly to 300 simulation seconds is a conservative approximation, not measured runtime.'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',default='recordings')
    p.add_argument('--output',default='recordings/deadlock_official_ab_20261109/resource_forecast.json')
    args=p.parse_args();r=forecast(args.root)
    Path(args.output).write_text(json.dumps(r,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in r.items() if k.startswith('full_')},indent=2))


if __name__=='__main__':main()
