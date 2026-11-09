#!/usr/bin/env python3
"""사전 서브게임 시뮬(뒤돌아 시작 → 레이더 시설 타격). 예: python scripts/run_subgame.py --n 10 --dist 3.0"""
import argparse, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.sim.subgame import best_of_two


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--dist", type=float, default=3.0)
    ap.add_argument("--lateral", type=float, default=0.0)
    a = ap.parse_args()
    ts = []
    for s in range(a.n):
        r = best_of_two(s, dist=a.dist, lateral=a.lateral)
        print(s, r["best"], [t["ok"] for t in r["tries"]])
        if r["best"] is not None:
            ts.append(r["best"])
    if ts:
        print(f"성공 {len(ts)}/{a.n}, 평균 {sum(ts)/len(ts):.2f}s, 최대 {max(ts):.2f}s")


if __name__ == "__main__":
    main()
