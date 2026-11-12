"""Offline truth comparison only. Never feed these results to a driving agent."""
import argparse
import bisect
import json
import math
from pathlib import Path

import numpy as np


def analyze(root, until=float('inf')):
    truth = [json.loads(s) for s in (root / 'evaluation_truth_all.jsonl').read_text().splitlines()]
    times = [r['sim_s'] for r in truth]
    out = {}
    for trace in sorted(root.glob('*/agent_trace.jsonl')):
        slot = trace.parent.name
        samples = []
        for line in trace.read_text().splitlines():
            r = json.loads(line)
            if r.get('elapsed', 0) < 5:
                continue
            if r.get('elapsed', 0) > until:
                continue
            t = r['sim_s']
            j = bisect.bisect_left(times, t)
            j = min((k for k in (j - 1, j) if 0 <= k < len(times)), key=lambda k: abs(times[k] - t))
            if abs(times[j] - t) > .25 or slot not in truth[j]['poses']:
                continue
            tx, ty, _, yaw = truth[j]['poses'][slot]
            x, y, th = r['estimated_nav_xy_yaw']
            pos = math.hypot(x - (tx + 3), y - (ty + 2.4))
            angle = abs(math.degrees(math.atan2(math.sin(th - yaw), math.cos(th - yaw))))
            samples.append((t, tx, ty, pos, angle, abs(r['cmd_vw'][0]), r['mode']))
        if not samples:
            continue
        errors = np.array([[r[3], r[4]] for r in samples])
        stalled = []
        i = 0
        while i < len(samples):
            j = i + 1
            while j < len(samples) and samples[j][0] - samples[i][0] < 1:
                j += 1
            if j >= len(samples):
                break
            span = samples[i:j + 1]
            commanded = np.mean([s[5] > .05 for s in span]) >= .8
            excursion = max(math.hypot(s[1] - span[0][1], s[2] - span[0][2]) for s in span)
            if commanded and excursion < .02:
                stalled.append({'t': round(span[0][0], 2), 'duration_s': round(span[-1][0] - span[0][0], 3),
                                'mode': span[0][6], 'position_error_m': round(span[0][3], 3),
                                'heading_error_deg': round(span[0][4], 2)})
            i = j
        out[slot] = {'samples': len(samples), 'position_error_m_p50_p95_max': np.percentile(errors[:, 0], [50, 95, 100]).round(3).tolist(),
                     'heading_error_deg_p50_p95_max': np.percentile(errors[:, 1], [50, 95, 100]).round(2).tolist(),
                     'commanded_stall_s': round(sum(r['duration_s'] for r in stalled), 2), 'stall_windows': stalled}
    return {'source': str(root), 'until_elapsed_s': until if math.isfinite(until) else None,
            'method': 'Nearest truth within 0.25s; arena 6x4.8; exclude startup <5s. Stall: >80% commands >0.05m/s, <0.02m excursion in 1s. This is not a contact sensor.', 'robots': out}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--until', type=float, default=float('inf'))
    args = p.parse_args()
    result = analyze(args.run, args.until)
    name = 'navigation_alignment_analysis' + (f'_{args.until:g}s' if math.isfinite(args.until) else '') + '.json'
    (args.run / name).write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: {a: b for a, b in v.items() if a != 'stall_windows'} for k, v in result['robots'].items()}, indent=2))
