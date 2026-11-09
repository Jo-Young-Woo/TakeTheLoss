#!/usr/bin/env python3
"""적 UGV 회피 비교 실험(시뮬): 회피 OFF(기준선) vs ON, 같은 시드·같은 배치·같은 적 행동.

  python scripts/compare_avoid.py --seeds 1 2 3 --scenarios S1_front_chaser S3_rear_chaser
  python scripts/compare_avoid.py --variants base avoid avoid_lidar_only --seeds 1 2 3
  python scripts/compare_avoid.py --list

!! 시뮬 수치다. 적 행동(완벽 정보 추격·조준)·카메라/UAV 잡음 모델은 가정이며 실기 성능이 아니다. 실제 YOLO 가중치·UAV 영상 검출기가 없어서
   '카메라 팀 판정'은 시뮬이 주는 라벨(오판 4 %)로 대신한다. 이 스크립트의 변형(variants)은 그 의존성을 끄거나 악화시켜 민감도를 본다.
"""
import argparse
import os
import sys
import warnings

warnings.filterwarnings("ignore")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.sim.bench import make_cfg, run_trial  # noqa: E402
from maicon26.sim.scenarios import SCENARIOS, build  # noqa: E402

# 변형: (cfg 변경, 월드 노브 변경)
VARIANTS = {
    "base": (lambda c: setattr(c.avoid, "enable", False), {}),
    "avoid": (lambda c: None, {}),
    "avoid_noblob": (lambda c: setattr(c.avoid, "use_blobs", False), {}),
    "avoid_yolo_miss100": (lambda c: None, {"ugv_miss_base": 1.0}),
    "avoid_no_uav": (lambda c: None, {"uav_enabled": False}),
    "avoid_uav_delay2": (lambda c: None, {"uav_latency": 2.0}),
    "avoid_lidar_only": (lambda c: None, {"ugv_miss_base": 1.0, "uav_enabled": False}),
    "avoid_team_flip20": (lambda c: None, {"team_flip": 0.20}),
}


def run_variant(name, scen, seed, scen_over, max_t):
    cfg_mut, knobs = VARIANTS[name]
    cfg = make_cfg("online", mut=cfg_mut)
    base_setup = build(scen, **scen_over)

    def setup(m):
        for k, v in knobs.items():
            setattr(m.world, k, v)
        return base_setup(m)
    return run_trial(seed, "online", cfg=cfg, setup=setup, max_t=max_t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="*", default=[1, 2, 3])
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS))
    ap.add_argument("--variants", nargs="*", default=["base", "avoid"])
    ap.add_argument("--max-t", type=float, default=300.0)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    if a.list:
        print("시나리오:", ", ".join(SCENARIOS))
        print("변형:", ", ".join(VARIANTS))
        return
    keys = [("hits_taken", "피격"), ("collisions", "충돌틱"), ("min_dist", "최소거리m"), ("exposure_s", "노출s"), ("m5", "M5시각"),
            ("m2_ok", "M2"), ("recall", "M3재현"), ("precision", "M3정밀"), ("chatter", "채터")]
    for sc in a.scenarios:
        print(f"\n## {sc}  seeds={a.seeds}")
        print(f"{'변형':22s}" + "".join(f"{lab:>10s}" for _, lab in keys) + f"{'M5도착':>8s}")
        for vn in a.variants:
            rows = [run_variant(vn, sc, s, {}, a.max_t) for s in a.seeds]
            n = len(rows)
            cells = []
            for k, _ in keys:
                vals = [r[k] for r in rows if r[k] is not None]
                if k == "m2_ok":
                    cells.append(f"{sum(1 for r in rows if r[k]) / n:10.2f}")
                elif not vals:
                    cells.append(f"{'-':>10s}")
                else:
                    cells.append(f"{sum(float(v) for v in vals) / len(vals):10.2f}")
            m5 = sum(1 for r in rows if r["m5"] is not None) / n
            print(f"{vn:22s}" + "".join(cells) + f"{m5:8.2f}")


if __name__ == "__main__":
    main()
