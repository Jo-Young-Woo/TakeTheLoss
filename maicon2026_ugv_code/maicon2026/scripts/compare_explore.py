#!/usr/bin/env python3
"""M3 탐색 전략 비교(시뮬): 기존 고정 5x5 순회(lattice) vs 온라인(online). 같은 시드·같은 배치.
  python scripts/compare_explore.py --seeds 1 2 3 4 5 6 [--shuffle 60 150]
시뮬 수치이며 실기 성능이 아니다."""
import argparse, os, sys, warnings
warnings.filterwarnings("ignore")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.sim.bench import make_cfg, run_trial, summarize


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--shuffle", type=float, nargs="*", default=[])
    ap.add_argument("--depth", action="store_true")
    a = ap.parse_args()
    # 세 조건: (기준선) 고정 순회 + 기존 집계 / (집계만 교체) 고정 순회 + 장부 집계 / (제안) 온라인 + 장부 집계
    variants = [("lattice/omap", "lattice", "omap"), ("lattice/ledger", "lattice", "ledger"), ("online/ledger", "online", "ledger")]
    res = {}
    for name, strat, cnt in variants:
        def mut(c, cnt=cnt):
            c.explore.m3_counting = cnt
        rows = [run_trial(s, strat, a.shuffle, cfg=make_cfg(strat, a.depth, mut=mut)) for s in a.seeds]
        res[name] = (rows, summarize(rows))
    print(f"시드 {a.seeds}  shuffle={a.shuffle}")
    keys = [("recall", "M3 재현율"), ("precision", "M3 정밀도"), ("dist_235", "이동거리(235 s까지, m)"), ("revisit", "중복 방문 비율"),
            ("m5_rate", "M5 도착률"), ("m5_t", "M5 도착 시각(s)"), ("collisions", "충돌 틱(평균)"), ("m2_rate", "M2 성공률")]
    names = [v[0] for v in variants]
    print(f"{'지표':24s} " + " ".join(f"{n:>15s}" for n in names))
    for k, label in keys:
        v = [res[n][1][k] for n in names]
        print(f"{label:24s} " + " ".join(f"{x:15.3f}" if x is not None else f"{'-':>15s}" for x in v))
    for s in a.seeds:
        r = [next(x for x in res[n][0] if x["seed"] == s) for n in names]
        print(f"  seed {s}: recall " + " → ".join(f"{x['recall']:.2f}" for x in r) + "  precision " + " → ".join(f"{x['precision']:.2f}" for x in r)
              + "  dist " + " → ".join(f"{x['dist_235']:.0f}" for x in r))


if __name__ == "__main__":
    main()
