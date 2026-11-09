#!/usr/bin/env python3
"""Plan/run the requested physical Gazebo matrix without treating plans as runs.

Default is manifest-only. --execute launches cases sequentially because six
RGB/LiDAR robot stacks already share the renderer. Interrupted/failed attempts
are preserved. Completed trials are not assumed to meet acceptance criteria.
"""
from __future__ import annotations
import argparse
import hashlib
import html
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from maicon26.sim.sensor_policies import SCENARIOS, policy_assignments

VARIANTS=('v2','progress','blocker_fire','dwell_zero')


def source_fingerprint():
    paths=list((ROOT/'maicon26').rglob('*.py'))
    paths += [ROOT/'scripts'/name for name in
              ('run_wingbot_selfplay.py','run_wingbot_autonomous_trial.py','analyze_wingbot_selfplay.py')]
    for package in ('maicon_arena_gz','wingbot_description'):
        for folder in ('scripts','config','launch'):
            base=ROOT/'ros2'/package/folder
            if base.exists():
                paths += [p for p in base.rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.json','.xml')]
    paths += list((ROOT/'config').glob('*.json'))
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}


def assert_source_pin(args):
    """No matrix may silently mix policies/sensor fixes across its cases."""
    current=source_fingerprint()
    path=args.output/'pinned_source_hashes.json'
    pinned=read(path)
    if pinned is None:
        write(path,dict(hashes=current,created_wall=time.time(),
                       v2_semantics='Shared current HAL, model and sensor fixes; progress.enabled=false policy comparator. Not the original archived 2D source.'))
    elif pinned.get('hashes')!=current:
        old=pinned.get('hashes',{})
        changed=sorted(k for k in set(old)|set(current) if old.get(k)!=current.get(k))
        raise RuntimeError('Source changed since matrix pin; use a new output directory for a new candidate. Changed: '+', '.join(changed[:12]))
    return current


def read(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):return None


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    temporary.replace(path)


def inventory(args):
    root=args.output.resolve();cases=[]
    for variant in args.variants:
        for scenario in args.scenarios:
            for seed in args.seeds:
                folder=root/variant/scenario/f'seed_{seed:02d}'
                attempts=[]
                for path in sorted(folder.glob('attempt_*/selfplay_report.json')):
                    report=read(path)
                    drift=read(folder/(path.parent.name+'_source_drift.json'))
                    attempts.append(dict(path=str(path.parent.relative_to(root)),
                                         completed=bool(report and report.get('completed') and not drift),
                                         source_drift=bool(drift),
                                         errors=report.get('errors',[]) if report else ['Unreadable report']))
                complete=next((a for a in reversed(attempts) if a['completed']),None)
                metrics=read(root/complete['path']/'selfplay_analysis.json') if complete else None
                cases.append(dict(variant=variant,scenario=scenario,seed=seed,
                                  policies=policy_assignments(scenario),attempts=attempts,
                                  completed=complete is not None,metrics=metrics,
                                  accepted=metrics.get('acceptance',{}).get('overall_passed') if metrics else None))
    return cases


def manifest(args):
    cases=inventory(args)
    data=dict(backend='actual Gazebo; six real sensor/physics/HAL namespaces',
              requested_cases=len(cases),completed_cases=sum(c['completed'] for c in cases),
              accepted_cases=sum(c['accepted'] is True for c in cases),
              acceptance_unknown_cases=sum(c['accepted'] is None for c in cases),
              seed_scope='Agent/navigation RNG; competition arena layout remains fixed',
              controller_truth_access=False,alliance_communication=False,
              v2_semantics='Shared current HAL/sensor/model fixes, candidate progress policy disabled. Not the archived original 2D source.',
              candidate_revision_policy='Source/config hashes pinned at first execution; any drift is rejected before the next case or resume.',
              duration_s=args.duration,variants=args.variants,scenarios=args.scenarios,seeds=args.seeds,
              duration_semantics='300 s requested maximum; referee M5 capture may finish the match earlier',
              no_uav=args.no_uav,cases=cases,
              note='Manifest entries and successful process exits are not acceptance evidence.')
    write(args.output/'manifest.json',data)
    body=['<!doctype html><meta charset="utf-8"><title>Gazebo matrix</title>',
          '<style>body{font:15px sans-serif;margin:30px}td,th{padding:8px;border:1px solid #aaa}table{border-collapse:collapse}</style>',
          '<h1>Gazebo 센서 기반 3대3 시험 행렬</h1>',
          f'<p>실행 완료 {data["completed_cases"]}/{len(cases)}. 합격 판정 {data["accepted_cases"]}. 계획된 항목은 실행 결과가 아닙니다.</p>',
          '<table><tr><th>변형</th><th>시나리오</th><th>시드</th><th>실행</th><th>합격</th><th>결과</th></tr>']
    for case in cases:
        last=case['attempts'][-1] if case['attempts'] else None
        link=(f'<a href="{html.escape(last["path"])}/wingbot_selfplay_third_person.mp4">영상</a> '
              f'<a href="{html.escape(last["path"])}/selfplay_report.json">로그</a>') if last else '미실행'
        body.append(f'<tr><td>{case["variant"]}</td><td>{case["scenario"]}</td><td>{case["seed"]}</td><td>{case["completed"]}</td><td>{case["accepted"]}</td><td>{link}</td></tr>')
    body.append('</table>')
    (args.output/'report.html').write_text('\n'.join(body),encoding='utf-8')
    return data


def run(args):
    data=manifest(args)
    if not args.execute:
        print(json.dumps({k:data[k] for k in ('requested_cases','completed_cases','accepted_cases')},indent=2))
        return 0
    helper=ROOT/'scripts/run_wingbot_selfplay.py'
    if '--scenario' not in helper.read_text(encoding='utf-8'):
        raise RuntimeError('The physical coordinator must support sensor-only --scenario policies before this matrix can run.')
    assert_source_pin(args)
    stop=[False];child=[None]
    def interrupt(*_):
        stop[0]=True
        if child[0] is not None and child[0].poll() is None:
            child[0].send_signal(signal.SIGINT)
    old={sig:signal.signal(sig,interrupt) for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        for case in data['cases']:
            if stop[0]:break
            source_hashes=assert_source_pin(args)
            if case['completed'] and args.resume:continue
            while (args.output/'PAUSE_SCHEDULING').exists() and not stop[0]:
                time.sleep(1.)
            if stop[0]:break
            folder=args.output/case['variant']/case['scenario']/f'seed_{case["seed"]:02d}'
            existing=list(folder.glob('attempt_*')) if folder.exists() else []
            number=max((int(p.name.split('_')[-1]) for p in existing),default=-1)+1
            out=folder/f'attempt_{number:03d}'
            folder.mkdir(parents=True,exist_ok=True)
            cmd=[sys.executable,str(helper),'--output',str(out.resolve()),'--variant',case['variant'],
                 '--scenario',case['scenario'],'--seed',str(case['seed']),'--duration',str(args.duration),
                 '--ros-domain-id',str(args.ros_domain_id),'--ready-timeout',str(args.ready_timeout),
                 '--wall-timeout',str(args.wall_timeout)]
            if args.no_uav:cmd.append('--no-uav')
            write(folder/f'attempt_{number:03d}_invocation.json',dict(command=cmd,source_hashes=source_hashes,
                       started_wall=time.time(),policy_map=case['policies']))
            with (folder/f'attempt_{number:03d}.console.log').open('w',encoding='utf-8') as log:
                child[0]=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                code=child[0].wait();child[0]=None
            report=read(out/'selfplay_report.json')
            final_hashes=source_fingerprint()
            if final_hashes!=source_hashes:
                write(folder/f'attempt_{number:03d}_source_drift.json',dict(
                    valid_for_comparison=False,reason='source edited during the physical trial',
                    changed=sorted(k for k in set(final_hashes)|set(source_hashes)
                                   if final_hashes.get(k)!=source_hashes.get(k))))
                raise RuntimeError('Source changed during a trial; preserved recording is diagnostic only, not comparable.')
            if code==0 and report and report.get('completed'):
                evaluator=ROOT/'scripts/analyze_wingbot_selfplay.py'
                if evaluator.exists():
                    subprocess.run([sys.executable,str(evaluator),str(out)],cwd=ROOT,check=False)
            state=manifest(args)
            print(json.dumps(dict(variant=case['variant'],scenario=case['scenario'],seed=case['seed'],
                                  returncode=code,completed=state['completed_cases'])),flush=True)
            if code and args.stop_on_error:break
    finally:
        manifest(args)
        for sig,handler in old.items():signal.signal(sig,handler)
    return 0


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,default=ROOT/'recordings/gazebo_deadlock_matrix_20261109')
    ap.add_argument('--seeds',type=int,nargs='+',default=list(range(10)))
    ap.add_argument('--scenarios',choices=SCENARIOS,nargs='+',default=list(SCENARIOS))
    ap.add_argument('--variants',choices=VARIANTS,nargs='+',default=['v2','progress'])
    ap.add_argument('--execute',action='store_true');ap.add_argument('--resume',action='store_true')
    ap.add_argument('--stop-on-error',action='store_true');ap.add_argument('--no-uav',action='store_true')
    ap.add_argument('--duration',type=float,default=300.)
    ap.add_argument('--ready-timeout',type=float,default=180.)
    ap.add_argument('--wall-timeout',type=float,default=3600.)
    ap.add_argument('--ros-domain-id',type=int,default=81)
    args=ap.parse_args()
    if args.duration!=300.:
        ap.error('Acceptance matrix requires a 300-second match limit; referee early finishes remain valid. Use the individual runner for smoke tests.')
    return run(args)


if __name__=='__main__':raise SystemExit(main())
