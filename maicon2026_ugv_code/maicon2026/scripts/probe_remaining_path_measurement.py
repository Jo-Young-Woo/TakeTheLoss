#!/usr/bin/env python3
"""Read-only reproduction of stale risk path masking valid mission path.

Exit1 exposes the cached-route defect; exit0 confirms route selection only,
not simulator acceptance. This probe does not patch production code.
"""
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from maicon26.mission.manager import Agent
from maicon26.types import Pose2D


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',default='recordings/deadlock_ab_20261109/remaining_path_repro.json')
    args=p.parse_args()
    agent=Agent.__new__(Agent)
    agent.stack=SimpleNamespace(pose=Pose2D(0.,0.,0.),navigator=SimpleNamespace(path=[]))
    goal=(2.,0.);detour=[(0.,0.),(0.,2.),(2.,2.),(2.,0.)]
    agent._ex_cand=SimpleNamespace(path=detour)
    agent._rc={'path':[(0.,0.),(1.,1.)]}
    stale=agent._remaining_path(goal)
    agent._rc={}
    clean=agent._remaining_path(goal)
    expected=sum(math.dist(a,b) for a,b in zip(detour,detour[1:]))
    result=dict(method='actual Agent._remaining_path; synthetic fixed pose and two cached paths',
        goal=goal,valid_exploration_path=detour,stale_risk_path=[(0.,0.),(1.,1.)],
        with_stale_risk_path_m=stale,without_stale_risk_path_m=clean,expected_valid_remaining_path_m=expected,
        stationary_measurement_jump_m=clean-stale,valid_path_selected=abs(stale-expected)<1e-9,
        runtime_stall_causality='unverified; use next real trace remaining_path_source to quantify',
        production_modified_by_probe=False,acceptance_thresholds_modified=False)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result,indent=2))
    return 0 if result['valid_path_selected'] else 1


if __name__=='__main__':raise SystemExit(main())
