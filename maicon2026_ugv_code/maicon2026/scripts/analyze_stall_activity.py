#!/usr/bin/env python3
"""Read-only motion breakdown of independently counted stall periods."""
import argparse
from collections import Counter
import gzip
import json
import math
from pathlib import Path
from compare_sim import Monitor


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('case');args=p.parse_args()
    folder=Path(args.case);monitor=Monitor();poses={};activity=Counter();robots=Counter();modes=Counter()
    for line in gzip.open(folder/'ticks.jsonl.gz','rt',encoding='utf-8'):
        row=json.loads(line);uid=row['robot'];t=row['t']
        dt=t-monitor.states.get(uid,{}).get('last',t)
        pose=row['evaluation_only']['pose'];old=poses.get(uid,pose)
        monitor.update(row,row['evaluation_only'])
        if not row['normal_wait'] and monitor.states[uid]['stalled']>=3 and dt>0:
            speed=math.dist(pose[:2],old[:2])/dt
            yaw_rate=abs((pose[2]-old[2]+math.pi)%(2*math.pi)-math.pi)/dt
            kind='translating' if speed>.02 else 'rotating' if yaw_rate>.1 else 'stationary'
            activity[(row['mode'],kind)]+=dt;robots[uid]+=dt;modes[row['mode']]+=dt
        poses[uid]=pose
    result=dict(source='Evaluator-only successive physical poses; no truth fed back to Agent',
        thresholds_unchanged=True,counts_s_by_mode=dict(modes),counts_s_by_robot=dict(robots),
        activity=[dict(mode=mode,physical_activity=kind,seconds=value) for (mode,kind),value in activity.items()],
        interpretation='Translation/rotation alone is not qualifying goal progress and is not subtracted from the stall metric.')
    (folder/'stall_activity.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
