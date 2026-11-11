#!/usr/bin/env python3
"""3 vs 3 전체 경기 시뮬. 예: python scripts/run_sim.py --seed 3 [--solo B2] [--config config/default.json]"""
import argparse, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from maicon26.config import Config
from maicon26.sim.run import build_match, run, score


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--config", default=None)
    ap.add_argument("--solo", default=None, help="한 대만 돌리기: B1..B3/R1..R3 (나머지는 정지 상태 상대)")
    ap.add_argument("--max-t", type=float, default=340.0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--nav", choices=["builtin", "nav2-fake"], default="builtin",
                    help="builtin=내장 SLAM+A*, nav2-fake=Nav2Stack 경계(앵커링·map 프레임·액션 흐름)를 시뮬 대역으로 검증")
    ap.add_argument("--shuffle", type=float, nargs="*", default=None, metavar="T",
                    help="경기 중 구조물을 바꾸는 시각(s) 목록. 예: --shuffle 60 150")
    ap.add_argument("--overrides", default=None, help="현장 오버라이드 파일(config/field_overrides.json) — 실측값을 시뮬에도 넣어 볼 때")
    ap.add_argument("--no-depth", action="store_true", help="깊이 카메라/3D 지도 끄기(LiDAR 2D 만)")
    ap.add_argument("--low-barrier", action="store_true",
                    help="LiDAR 평면 아래의 낮은 장벽(높이 12 cm)을 BLUE 시작과 중앙 사이에 둔다 — 깊이 3D 지도 시험용(규정의 구조물이 아님)")
    ap.add_argument("--map3d-out", default=None, metavar="PREFIX",
                    help="경기 뒤 각 로봇의 3D 지도를 PREFIX_<로봇>.ply 로 저장(MeshLab/CloudCompare 로 열기; .npz 는 PREFIX 끝을 .npz 로)")
    a = ap.parse_args()
    cfg = Config.load(a.config)
    if a.overrides:
        from maicon26.field import overrides as _ov
        _ov.apply_to_config(cfg, _ov.load(a.overrides))
    cfg.nav.backend = a.nav
    if a.no_depth:
        cfg.depth.enable = False
    cfg.mission.spin_w, cfg.mission.tour_nx, cfg.mission.tour_ny = 1.8, 3, 3
    if a.solo:
        spec = [a.solo]
        m = build_match(seed=a.seed, cfg=cfg, teams=spec)
    else:
        m = build_match(seed=a.seed, cfg=cfg)
    events = None
    if a.shuffle:
        from maicon26.sim.dynamic import shuffle_structures
        events = [(t, shuffle_structures) for t in a.shuffle]
    if a.low_barrier:
        m.world.add_low_obstacle(1.55, 0.55, 1.75, 4.45, 0.12)
    run(m, max_t=a.max_t, verbose=a.verbose, events=events)
    print(f"경기 종료 t={m.world.end_time}  (실행 {m.wall_s:.1f}s)")
    for k, v in score(m)["agents"].items():
        m1 = bool(v["m1"]) and all(v["m1"].values())
        print(f"{k}: M1={m1} M2={v['m2_ok']} M3={v['m3']} M4 hits={v['m4_hits_dealt']} friendly={v['friendly_hits']} "
              f"shots={v['shots']} M5={v['m5_reached']} M6={v['m6'] and v['m6']['top1']} collisions={v['collisions']}")
    for ag in m.agents:
        if ag.fusion is None:
            continue
        r = ag.map3d_report()
        s = ag.stats
        print(f"  [3D {ag.name}] 깊이 {s['depth_frames']}장 → 점유 복셀 {r['occupied_voxels']}, 시야 범위 {r['coverage']:.0%}, "
              f"항법 층 {s['nav_layer_cells']}셀, 전방 가드 {s['depth_guard_ticks']}틱, 깊이 거리 {s['range_from_depth']}회, "
              f"사선 막힘 {s['beam_blocked_ticks']}틱(통과 {s['beam_override']}), 높이 오검출 {s['height_reject']}, "
              f"깊이↔LiDAR 잔차 {r['lidar_resid_m']} m")
        if a.map3d_out:
            path = a.map3d_out if a.map3d_out.endswith((".ply", ".npz")) else f"{a.map3d_out}_{ag.name}.ply"
            if len(m.agents) > 1 and a.map3d_out.endswith((".ply", ".npz")):
                root, ext = os.path.splitext(a.map3d_out)
                path = f"{root}_{ag.name}{ext}"
            print(f"  [3D {ag.name}] 저장 {path} ({ag.export_map3d(path)} 복셀)")


if __name__ == "__main__":
    main()
