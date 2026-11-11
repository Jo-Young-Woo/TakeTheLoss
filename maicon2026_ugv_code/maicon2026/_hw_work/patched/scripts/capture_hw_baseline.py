#!/usr/bin/env python3
"""수정 전 기준값 저장기 (하드웨어 문서 반영 작업, 0-1장).

시뮬레이션을 쓰지 않고 순수 함수만 부른다. 하드웨어 반영 수정 *전에* 한 번만 돌려
tests/fixtures/hw_baseline/*.json 에 저장하고, 수정 후에는 tests/test_hw_baseline_equivalence.py 가
같은 함수(collect_*)를 다시 불러 저장값과 정확히 같은지(실수는 1e-12 안) 확인한다.

  python scripts/capture_hw_baseline.py            # 기준값이 없을 때만 저장한다(이미 있으면 덮어쓰지 않고 종료)
  python scripts/capture_hw_baseline.py --frozen   # runtime/selected_candidate02 해시 목록만 (동결 런타임이 있는 PC에서)
  python scripts/capture_hw_baseline.py --check-frozen   # 저장된 해시와 현재 해시를 대조
  (--root <저장소> 로 다른 위치의 runtime 을 읽고, --out <파일> 로 저장 위치를 바꿀 수 있다 — 동결 런타임이 있는 PC 에서 쓴다)

※ 임무코드는 어디에도 쓰지 않는다. 가짜 환경변수 K7M2Q9XA 만 쓴다.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
FIXTURES = ROOT / "tests" / "fixtures" / "hw_baseline"
FAKE_CODE = "K7M2Q9XA"
FAKE_SERVER = "http://127.0.0.1:9"
ROOT_TAG = "<ROOT>"


def _jsonable(x):
    """numpy/dataclass/튜플을 JSON 으로. 실수는 repr 정밀도를 그대로 둔다."""
    import dataclasses
    import numpy as np
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {k: _jsonable(v) for k, v in dataclasses.asdict(x).items()}
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return _jsonable(x.tolist())
    if isinstance(x, (np.floating,)):
        return float(x)
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.bool_,)):
        return bool(x)
    if isinstance(x, float) and not math.isfinite(x):
        return {"__float__": repr(x)}
    return x


def _load_cfgs():
    from maicon26.config import CameraCfg, Config
    return {"default": Config(), "waveshare_real": Config.load(str(ROOT / "config" / "waveshare_real.json"))}


# ---------------------------------------------------------------------------------------------- camera_pinhole
def collect_camera_pinhole():
    from maicon26.perception.camera import CameraModel
    from maicon26.types import Pose2D
    out = {}
    pose = Pose2D(1.25, 2.5, 0.37)
    for name, cfg in _load_cfgs().items():
        cam = CameraModel(cfg.camera)
        rows = []
        for u in range(0, 640, 32):
            for v in range(0, 480, 32):
                rows.append({
                    "u": u, "v": v,
                    "bearing": cam.bearing(u), "elevation": cam.elevation(v),
                    "world_ray": cam.world_ray(pose, 12.5, u),
                    "range_from_size": [cam.range_from_size(s, 0.4, u) for s in (8.0, 40.0, 160.0)],
                    "range_from_size_no_u": cam.range_from_size(40.0, 0.4),
                })
        out[name] = {"w": cam.w, "h": cam.h, "f": cam.f, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy, "grid": rows}
    return out


# ---------------------------------------------------------------------------------------------- aim_pinhole
def _aim_inputs():
    """고정 입력 30개: (검출 클래스, 상자, 센서, 팬, 틸트, 시각, 거리)."""
    rng_state = 12345
    items = []
    for i in range(30):
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF
        a = rng_state / 0x7FFFFFFF
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF
        b = rng_state / 0x7FFFFFFF
        cx, cy = 40 + a * 560, 40 + b * 400
        w, h = 30 + (i % 7) * 18, 24 + (i % 5) * 20
        box = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        cls = ("UGV", "RADAR_FACILITY", "GUARD_POST")[i % 3]
        sensors = None
        if i % 2 == 0:
            sensors = [(cx + (-1) ** i * 6.0, cy - 8.0, 5.0 + (i % 4), 0.8)]
        rng_m = None if i % 6 == 0 else 0.5 + 0.12 * i
        items.append((cls, box, sensors, -30.0 + 2.0 * i, -10.0 + 0.7 * i, 0.5 + 0.1 * i, rng_m))
    return items


def collect_aim_pinhole():
    from maicon26.mission.aim import AimController
    from maicon26.types import Detection
    out = {}
    for name, cfg in _load_cfgs().items():
        fresh, seq = [], []
        shared = AimController(cfg)
        for cls, box, sensors, pan, tilt, now, rng_m in _aim_inputs():
            extra = {"sensors": sensors} if sensors else {}
            det = Detection(cls, 0.9, box, 0.0, "RED" if cls == "UGV" else None, extra)
            ctrl = AimController(cfg)
            fresh.append(_jsonable(ctrl.update(det, pan, tilt, now, rng_m)))
            seq.append(_jsonable(shared.update(det, pan, tilt, now, rng_m)))
        out[name] = {"fresh": fresh, "sequence": seq, "auto_resets": shared.auto_resets}
    return out


# ---------------------------------------------------------------------------------------------- fire_hit_mission
def collect_fire_hit_mission():
    from maicon26.hal.mil_api import FireModel, HitModel
    cfg = _load_cfgs()["waveshare_real"]
    lc = cfg.laser
    fm = FireModel(lc.ammo, lc.shot_s, lc.shot_s + lc.cooldown_s)
    fire_steps = []
    script = [(1.0, "fired", None), (3.0, "locked", None), (11.5, "fired", 4200.0), (13.0, "reloading", 2500.0),
              (20.0, "fired", None), (31.0, "empty", None), (32.0, "bogus", None), (40.0, "fired", 1500.0)]
    for t, msg, left in script:
        r = fm.on_response(t, msg, left)
        fire_steps.append({"t": t, "msg": msg, "result": r, "ammo": fm.ammo, "fired": fm.fired,
                           "ready_at": fm.ready_at, "laser_on": fm.laser_on(t + 0.1),
                           "cooldown_left": fm.cooldown_left(t + 0.1), "rejected": dict(fm.rejected)})
    hm = HitModel(lc.disabled_s, cfg.hal.drive_pause_s)
    hp_steps = []
    seq = [(0.0, 4, 4), (1.0, 4, 4), (2.0, 3, 4), (3.0, 3, 4), (4.0, 4, 4), (5.0, 3, 4), (6.0, 2, 4), (7.0, 1, 4),
           (8.0, 0, 4), (20.0, 0, 4)]
    for t, hp, hp_max in seq:
        new = hm.on_state(t, hp, hp_max)
        hp_steps.append({"t": t, "hp": hp, "new": new, "hits": hm.hits, "disabled_left": hm.disabled_left(t),
                         "drive_left": hm.drive_left(t)})
    hm.note_locked_response(25.0, 3.0)
    hp_steps.append({"locked_note_disabled_left": hm.disabled_left(25.0)})
    # 경기 도중 재시작(처음 받은 hp 가 이미 줄어 있음)
    hm2 = HitModel(lc.disabled_s, cfg.hal.drive_pause_s)
    hm2.on_state(0.0, 2, 4)
    return {"fire": fire_steps, "hit": hp_steps, "hit_midrestart_hits": hm2.hits}


# ---------------------------------------------------------------------------------------------- launch_cmd / team_argv
def _scrub(obj):
    s = json.dumps(obj, ensure_ascii=False)
    s = s.replace(str(ROOT).replace("\\", "\\\\"), ROOT_TAG).replace(str(ROOT), ROOT_TAG)
    return json.loads(s)


def collect_launch_cmd():
    from maicon26.launch_profile import nav_launch_command
    argv = ["--config", str(ROOT / "config" / "waveshare_real.json")]
    cmd = nav_launch_command(argv, {}, root=ROOT)
    cmd_slam = nav_launch_command(argv, {"MAICON_SLAM": "slam_toolbox"}, root=ROOT)
    return _scrub({"default": cmd, "slam_toolbox": cmd_slam})


def collect_team_argv():
    spec = importlib.util.spec_from_file_location("hw_baseline_team_node", ROOT / "team" / "team_node.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    saved_env, saved_argv = dict(os.environ), list(sys.argv)
    try:
        for k in list(os.environ):
            if k.startswith("MAICON_") or k in ("TEAM_LAUNCH_NAV",):
                del os.environ[k]
        os.environ["MAICON_SERVER_URL"] = FAKE_SERVER
        os.environ["MAICON_CODE"] = FAKE_CODE
        captured = []
        mod.runpy.run_path = lambda *a, **k: captured.append(list(sys.argv))
        sys.argv = ["team_node.py"]
        rc = mod.main()
        argv = captured[-1] if captured else None
    finally:
        os.environ.clear()
        os.environ.update(saved_env)
        sys.argv = saved_argv
    return _scrub({"returncode": rc, "argv": argv})


# ---------------------------------------------------------------------------------------------- coverage_proj
def collect_coverage_proj():
    import numpy as np
    from maicon26.nav.grid import OccupancyGrid
    from maicon26.perception.coverage import CoverageMap, FaceRec
    out = {}
    for name, cfg in _load_cfgs().items():
        grid = OccupancyGrid(cfg.arena)
        cm = CoverageMap(cfg)
        occ = np.zeros_like(grid.static, dtype=bool)
        face = FaceRec(1, 0, -1.0, 0.0, 3.0, 2.4)          # 법선 -x, 면 중심 (3.0, 2.4)
        face2 = FaceRec(2, 1, 0.0, 1.0, 2.0, 1.0)
        rows = []
        for f in (face, face2):
            for vx in (1.0, 1.4, 1.8, 2.2, 2.6):
                for vy in (0.8, 1.6, 2.4, 3.2):
                    for vth in (None, -0.6, -0.2, 0.0, 0.3, 0.7, 1.3):
                        rows.append({"face": f.key, "vx": vx, "vy": vy, "vth": vth,
                                     "pic": list(cm.picture_visible(vx, vy, vth, f, occ, grid)),
                                     "vis": list(cm.face_visible(vx, vy, vth, f, occ, grid))})
        out[name] = {"fov_half": cm.fov_half, "rows": rows}
    return out


# ---------------------------------------------------------------------------------------------- object_map_pantilt
def collect_object_map_pantilt():
    import numpy as np
    from maicon26.perception.object_map import ObjectMap
    from maicon26.perception.tracks import UgvTracker
    from maicon26.types import Detection, Frame, Pose2D, Scan
    cfg = _load_cfgs()["default"]
    omap = ObjectMap(cfg)
    tracker = UgvTracker(cfg, "BLUE")
    n = 360
    ranges = np.full(n, 3.0)
    scan = Scan(-math.pi, 2 * math.pi / n, ranges.copy(), 0.0, 8.0)
    classes = list(cfg.perception.object_classes) + list(cfg.perception.target_classes)
    dets_log = []
    for i in range(20):
        cls = classes[i % len(classes)]
        cx = 120 + (i * 47) % 400
        cy = 150 + (i * 31) % 180
        w, h = 50 + (i % 5) * 10, 60 + (i % 4) * 12
        pose = Pose2D(1.0 + 0.05 * i, 2.4 - 0.03 * i, 0.1 * i)
        frame = Frame(None, 0.1 * i, pan=-20.0 + 2.0 * i, tilt=0.0)
        r = 0.9 + 0.05 * (i % 6)
        sc = Scan(-math.pi, 2 * math.pi / n, np.full(n, r + 0.05 * (i % 3)), 0.1 * i, 8.0)
        dets = [Detection(cls, 0.8 + 0.01 * (i % 10), (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2), 0.1 * i)]
        omap.observe(dets, pose, frame, sc)
        ugv = Detection("UGV", 0.9, (cx - 20, cy - 25, cx + 20, cy + 25), 0.1 * i, "RED")
        tracker.update_camera([ugv], pose, frame, sc)
        dets_log.append(i)
    faces = [{"x": f.x, "y": f.y, "cx": f.cx, "cy": f.cy, "n": f.n, "kind": f.kind, "last_t": f.last_t,
              "vx": f.vx, "vy": f.vy, "votes": dict(f.votes)} for f in omap.faces]
    tracks = [{"id": getattr(t, "tid", None), "x": t.x, "y": t.y, "n_obs": t.n_obs} for t in getattr(tracker, "tracks", {}).values()] \
        if isinstance(getattr(tracker, "tracks", None), dict) else \
        [{"x": t.x, "y": t.y, "n_obs": t.n_obs} for t in getattr(tracker, "tracks", [])]
    return {"n_dets": len(dets_log), "faces": faces, "base_votes": {k: dict(v) for k, v in omap.base_votes.items()},
            "nonobject_seen": omap.nonobject_seen, "association_stats": dict(omap.association_stats),
            "tracks": tracks}


# ---------------------------------------------------------------------------------------------- frozen runtime hashes
def frozen_runtime_hashes(root: Path = ROOT):
    base = root / "runtime" / "selected_candidate02"
    files, skipped = {}, []
    if not base.exists():
        return {"present": False, "files": {}, "skipped": []}
    for dirpath, dirnames, filenames in os.walk(base, followlinks=False):
        dirnames.sort()
        for name in sorted(filenames) + sorted(dirnames):
            p = Path(dirpath) / name
            rel = p.relative_to(base).as_posix()
            try:
                if p.is_symlink() or (p.is_dir() and name in dirnames and os.path.islink(p)):
                    skipped.append(rel + " (링크/정션: 읽지 않음)")
                    continue
                if p.is_file():
                    if "__pycache__" in rel or rel.endswith(".pyc"):
                        continue
                    files[rel] = hashlib.sha256(p.read_bytes()).hexdigest()
            except OSError as e:
                skipped.append(f"{rel} ({type(e).__name__})")
    return {"present": True, "files": files, "skipped": sorted(skipped)}


COLLECTORS = {
    "camera_pinhole": collect_camera_pinhole,
    "aim_pinhole": collect_aim_pinhole,
    "fire_hit_mission": collect_fire_hit_mission,
    "launch_cmd": collect_launch_cmd,
    "team_argv": collect_team_argv,
    "coverage_proj": collect_coverage_proj,
    "object_map_pantilt": collect_object_map_pantilt,
}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    FIXTURES.mkdir(parents=True, exist_ok=True)
    if "--frozen" in argv or "--check-frozen" in argv:
        def opt(name, default):
            return Path(argv[argv.index(name) + 1]) if name in argv else default
        now = frozen_runtime_hashes(opt("--root", ROOT))
        path = opt("--out", FIXTURES / "frozen_runtime_sha256.json")
        if "--check-frozen" in argv:
            old = json.loads(path.read_text(encoding="utf-8"))
            same = old["files"] == now["files"]
            print(json.dumps({"same": same, "files_before": len(old["files"]), "files_now": len(now["files"]),
                              "skipped_before": old.get("skipped"), "skipped_now": now["skipped"]}, ensure_ascii=False))
            return 0 if same else 1
        if path.exists():
            print("이미 있다 — 덮어쓰지 않는다:", path)
            return 0
        path.write_text(json.dumps(now, ensure_ascii=False, indent=1, sort_keys=True), encoding="utf-8")
        print("저장:", path, "파일", len(now["files"]), "건너뜀", len(now["skipped"]))
        return 0
    written = []
    for name, fn in COLLECTORS.items():
        path = FIXTURES / f"{name}.json"
        if path.exists():
            print("이미 있다 — 덮어쓰지 않는다:", path.name)
            continue
        data = _jsonable(fn())
        path.write_text(json.dumps(data, ensure_ascii=False, indent=1, sort_keys=True), encoding="utf-8")
        written.append(name)
    print("저장:", ", ".join(written) or "(없음)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
