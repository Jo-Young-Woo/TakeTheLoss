#!/usr/bin/env python3
"""차체 외곽(직사각형)·LiDAR 오차 예산·경기장 유효성 — 변경 전후 비교(시뮬).

  python scripts/compare_footprint.py arena  --seeds 1-60            # 유효 경기장 생성률, 제외 사유별 건수, 재생성 횟수
  python scripts/compare_footprint.py match  --seeds 1 2 3 4 5 6     # 같은 (유효) 배치에서 변경 전(원 0.14·완화) vs 후(직사각형) 전체 경기
  python scripts/compare_footprint.py lidar  --seeds 1 2 3 4         # LiDAR 편향·잡음·누락·지연·위치추정 오차를 바꿔 충돌·M5 비교

!! 시뮬 수치다. 실기 안전성의 증거가 아니다: LiDAR 오차는 가정값, 충돌 모델은 2D 직사각형(높이·돌출부·바퀴 없음), 상대 UGV 는 없음.
   '변경 전' = Config.use_legacy_circle() (시뮬 충돌 원 0.14 m, A* 0.17 + 실패 시 0.8·0.6 배 완화, 직사각형 검증 없음, 정지 0.17).
"""
import argparse
import os
import sys
import warnings
from collections import Counter
from concurrent.futures import ProcessPoolExecutor

warnings.filterwarnings("ignore")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import numpy as np  # noqa: E402

from maicon26.config import Config  # noqa: E402
from maicon26.sim.run import build_match, run, score  # noqa: E402
from maicon26.sim.world import World  # noqa: E402


def parse_seeds(tokens):
    out = []
    for t in tokens:
        if "-" in t:
            a, b = t.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(t))
    return out


# ====================================================================== 경기장 생성
def arena_stats(seeds, max_tries):
    cnt, att, fails, ok = Counter(), [], 0, 0
    for s in seeds:
        cfg = Config()
        cfg.arena.max_layout_tries = max_tries
        try:
            w = World(cfg, seed=s)
        except Exception as e:                                   # ArenaGenerationError
            fails += 1
            for ex in getattr(e, "meta", {}).get("excluded", []):
                for r in ex["reasons"]:
                    cnt[r.split(":")[0]] += 1
            continue
        ok += 1
        att.append(w.layout_meta["attempts"])
        for ex in w.layout_meta["excluded"]:
            for r in ex["reasons"]:
                cnt[r.split(":")[0]] += 1
    n_excl_layouts = sum(a - 1 for a in att)
    print(f"시드 {len(seeds)}개, 재시도 상한 {max_tries}: 유효 경기장 생성 {ok}/{len(seeds)} ({100 * ok / len(seeds):.0f} %), 상한 초과 중단 {fails}")
    print(f"  첫 시도에 유효: {sum(1 for a in att if a == 1)}/{len(seeds)} ({100 * sum(1 for a in att if a == 1) / len(seeds):.0f} %)")
    print(f"  시도 횟수 분포: {dict(sorted(Counter(att).items()))}  (평균 {np.mean(att):.2f}, 최대 {max(att) if att else 0})")
    print(f"  제외된 무효 배치 {n_excl_layouts}개의 사유별 건수(배치 하나가 여러 사유를 가질 수 있다): {dict(cnt)}")


# ====================================================================== 전체 경기
def _min_clear(w, u):
    """차체 직사각형 경계점과 박스·벽 사이 최소 거리(m). 음수/0 이면 접촉."""
    fp = w._fp()
    c = fp.corners(u.pose.x, u.pose.y, u.pose.th)
    pts = []
    for i in range(4):
        a, b = c[i], c[(i + 1) % 4]
        for t in np.linspace(0, 1, 6, endpoint=False):
            pts.append(a + (b - a) * t)
    pts = np.array(pts)
    best = min(pts[:, 0].min(), w.geo.W - pts[:, 0].max(), pts[:, 1].min(), w.geo.H - pts[:, 1].max())
    for (x0, y0, x1, y1) in w.obstacles():
        if abs((x0 + x1) / 2 - u.pose.x) > 1.0 or abs((y0 + y1) / 2 - u.pose.y) > 1.0:
            continue
        dx = np.maximum(np.maximum(x0 - pts[:, 0], pts[:, 0] - x1), 0)
        dy = np.maximum(np.maximum(y0 - pts[:, 1], pts[:, 1] - y1), 0)
        best = min(best, float(np.hypot(dx, dy).min()))
    return float(best)


def run_one(args):
    seed, mode, layout_seed, knobs, backend = args
    cfg = Config()
    cfg.nav.backend = backend
    cfg.avoid.enable = False                                   # 이번 비교는 항법·충돌만 본다
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    cfg.arena.validate_layout = False                          # 배치는 호출자가 이미 정했다(같은 배치 비교)
    if mode == "legacy":
        cfg.use_legacy_circle()
    for k, v in knobs.items():
        sec, name = k.split(".")
        setattr(getattr(cfg, sec), name, v)
    m = build_match(seed=seed, cfg=cfg, teams=["B2"], layout_seed=layout_seed)
    u = m.world.ugvs[0]
    st = {"min": 9.0, "coll_events": 0, "prev": 0}

    def probe(mm):
        st["min"] = min(st["min"], _min_clear(mm.world, u))
    run(m, max_t=300.0, probe=probe)
    sc = score(m)["agents"][u.id]
    return {"seed": seed, "mode": mode, "coll": u.collisions, "min_clear": st["min"], "m2": bool(sc["m2_ok"]), "m5": sc["m5_reached"],
            "recoveries": m.agents[0].stats["recoveries"], "safe_stops": sum(m.agents[0].stats["safe_stops"].values())}


def summarize(rows):
    n = len(rows)
    m5 = [r["m5"] for r in rows if r["m5"] is not None]
    return {"n": n, "충돌틱": np.mean([r["coll"] for r in rows]), "최소여유(mm)": 1000 * np.mean([r["min_clear"] for r in rows]),
            "최소여유 최솟값(mm)": 1000 * min(r["min_clear"] for r in rows), "M2": np.mean([r["m2"] for r in rows]),
            "M5 도착률": len(m5) / n, "M5 시각": float(np.mean(m5)) if m5 else float("nan"), "복구": np.mean([r["recoveries"] for r in rows]),
            "SAFE_STOP": np.mean([r["safe_stops"] for r in rows])}


def show(title, table):
    print(f"\n## {title}")
    keys = list(next(iter(table.values())).keys())
    print(f"{'조건':26s}" + "".join(f"{k:>16s}" for k in keys))
    for name, s in table.items():
        print(f"{name:26s}" + "".join(f"{s[k]:16.2f}" for k in keys))


def valid_layout_seed(seed):
    return World(Config(), seed=seed).layout_meta["used_layout_seed"]


def match_cmp(seeds, backend, workers):
    jobs = []
    for s in seeds:
        ls = valid_layout_seed(s)
        jobs += [(s, "legacy", ls, {}, backend), (s, "rect", ls, {}, backend)]
    with ProcessPoolExecutor(workers) as ex:
        rows = list(ex.map(run_one, jobs))
    table = {m: summarize([r for r in rows if r["mode"] == m]) for m in ("legacy", "rect")}
    show(f"같은 유효 배치 {len(seeds)}개, backend={backend}: 변경 전(legacy) vs 후(rect)", table)


LIDAR_VARIANTS = {
    "기준(σ0.008·누락1%)": {},
    "편향 -3cm(가깝게)": {"lidar_err.sim_range_bias_m": -0.03},
    "편향 +3cm(멀게)": {"lidar_err.sim_range_bias_m": 0.03},
    "잡음 σ3cm": {"lidar_err.sim_range_sigma_m": 0.03},
    "각도 오차 1°": {"lidar_err.sim_angle_sigma_deg": 1.0},
    "누락 20%": {"lidar_err.sim_dropout_p": 0.20},
    "이상치 3%": {"lidar_err.sim_outlier_p": 0.03},
    "지연 0.2s": {"lidar_err.sim_latency_s": 0.2},
    "위치추정 σ5cm": {"lidar_err.sim_pose_sigma_m": 0.05},
}


def lidar_cmp(seeds, backend, workers):
    jobs, tags = [], []
    for name, kn in LIDAR_VARIANTS.items():
        for mode in ("legacy", "rect"):
            for s in seeds:
                jobs.append((s, mode, valid_layout_seed(s), kn, backend))
                tags.append((name, mode))
    with ProcessPoolExecutor(workers) as ex:
        rows = list(ex.map(run_one, jobs))
    for mode in ("legacy", "rect"):
        table = {name: summarize([r for r, t in zip(rows, tags) if t == (name, mode)]) for name in LIDAR_VARIANTS}
        show(f"LiDAR 오차 시험 — {mode} (시드 {len(seeds)}개, backend={backend})", table)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["arena", "match", "lidar"])
    ap.add_argument("--seeds", nargs="*", default=["1-60"])
    ap.add_argument("--max-tries", type=int, default=30)
    ap.add_argument("--backend", default="builtin")
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()
    seeds = parse_seeds(a.seeds)
    if a.what == "arena":
        arena_stats(seeds, a.max_tries)
    elif a.what == "match":
        match_cmp(seeds, a.backend, a.workers)
    else:
        lidar_cmp(seeds, a.backend, a.workers)


if __name__ == "__main__":
    main()
