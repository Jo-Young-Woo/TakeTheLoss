"""현장 실험 스켈레톤(maicon26/field/, scripts/field_exp.py)이 '코드로서' 도는지 확인한다.

!! 여기서 시뮬 로봇으로 돌린 수치는 현장 값이 아니다. 이 시험은 (1) 레지스트리가 실제 설정과 어긋나지 않는지, (2) 실험 10개가 끝까지 돌고
기록·제안이 올바른 형식으로 나오는지, (3) 제안 → 오버라이드 → Config 적용이 이어지는지, (4) 분석 수식이 합성 데이터에서 맞는지를 본다.
"""
import json
import math
import os
import subprocess
import sys

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.field import overrides as ov
from maicon26.field import tunables as tun
from maicon26.field.experiments import EXPERIMENTS, execute, replay_depth, rel_pose
from maicon26.field.runner import ORDER, checklist_markdown, make_sim_ctx
from maicon26.nav.stall import StallDetector
from maicon26.types import Pose2D

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


# ====================================================================== 레지스트리
def test_registry_paths_exist_and_are_consistent():
    cfg = Config()
    seen = set()
    for t in tun.TUNABLES:
        assert t.path not in seen, f"중복: {t.path}"
        seen.add(t.path)
        tun.resolve(cfg, t.path)                                            # 없는 경로면 AttributeError
        assert t.status in (tun.PDF, tun.ASSUMED, tun.SIM_TUNED), t
        assert t.exp in EXPERIMENTS or t.exp == "MANUAL" or t.exp.startswith("calib:"), t
        assert t.prio in (1, 2, 3)
        if t.status == tun.SIM_TUNED:
            assert t.basis, f"SIM_TUNED 는 근거(basis)가 있어야 한다: {t.path}"
        if t.exp in EXPERIMENTS:
            assert t.how, f"실험이 있는 항목은 현장 확인법이 있어야 한다: {t.path}"
    assert set(ORDER) == set(EXPERIMENTS)


def test_every_experiment_suggestion_target_is_a_registered_tunable_or_valid_path():
    cfg = Config()
    registered = {t.path for t in tun.TUNABLES}
    for t in tun.TUNABLES:
        if t.exp in EXPERIMENTS:
            ov.set_by_path(Config(), t.path, tun.resolve(cfg, t.path))      # 같은 값으로 쓰기 가능(형식 변환 포함)
    assert "map3d.bump_ratio" in registered and "rig.depth_xyz" in registered


# ====================================================================== 오버라이드
def test_overrides_record_apply_revert_roundtrip(tmp_path):
    cfg = Config()
    data = ov.empty()
    ov.record(data, cfg, "map3d.nav_thr", 2.4, "F06", "허상 줄이려 올림", "rec.json", date="2026-11-07")
    ov.record(data, cfg, "rig.depth_xyz", [0.12, 0.0, 0.11], "F09", "줄자", date="2026-11-07")
    ov.record(data, cfg, "map3d.nav_thr", 2.0, "F06", "다시", date="2026-11-08")       # 같은 경로 재기록
    assert data["history"][-1]["old"] == 2.4 and data["values"]["map3d.nav_thr"] == 2.0
    p = ov.save(data, str(tmp_path / "o.json"))
    cfg2 = ov.load_config(None, p)
    assert cfg2.map3d.nav_thr == 2.0 and cfg2.rig.depth_xyz == (0.12, 0.0, 0.11) and isinstance(cfg2.rig.depth_xyz, tuple)
    assert tun.status_of("map3d.nav_thr", data["provenance"]) == tun.MEASURED
    assert tun.status_of("map3d.stride", data["provenance"]) == tun.SIM_TUNED
    assert Config().map3d.nav_thr == cfg.map3d.nav_thr                               # 원본 기본값은 안 바뀐다
    assert ov.revert(data, "map3d.nav_thr") and "map3d.nav_thr" not in data["values"] and not ov.revert(data, "map3d.nav_thr")
    with pytest.raises(KeyError):
        ov.record(data, cfg, "map3d.no_such", 1, "F06")
    with pytest.raises(KeyError):
        ov.set_by_path(cfg, "nope.x", 1)
    ov.set_by_path(cfg, "map3d.nav_min_neighbors", 2)
    assert cfg.map3d.nav_min_neighbors == 2
    ov.set_by_path(cfg, "map3d.nav_thr", 3)                                          # int → float 로 맞춘다
    assert isinstance(cfg.map3d.nav_thr, float)


def test_missing_overrides_file_means_defaults(tmp_path):
    assert ov.load(str(tmp_path / "none.json")) == ov.empty()
    assert ov.load_config(None, str(tmp_path / "none.json")).map3d.nav_thr == Config().map3d.nav_thr


def test_shipped_overrides_file_is_empty_template():
    d = ov.load(ov.DEFAULT_PATH)
    assert d["values"] == {} and d["provenance"] == {}, "현장 값이 들어오기 전에는 비어 있어야 한다(가짜 실측값 금지)"


# ====================================================================== 실험 10개 드라이런 (시뮬 — 코드가 도는지만)
@pytest.mark.parametrize("eid", ORDER)
def test_experiment_dry_run_completes_and_writes_valid_record(eid, tmp_path):
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    path, rec = execute(EXPERIMENTS[eid], ctx)
    with open(path, encoding="utf-8") as f:
        disk = json.load(f)                                                          # 표준 JSON(NaN/inf 없음)
    assert disk["id"] == eid and disk["robot"] == "sim" and "WARNING" in disk and "시뮬" in disk["WARNING"]
    an = disk["analysis"]
    assert an["verdict"] in ("PASS", "FAIL", "CHECK", "INFO")
    cfg = Config()
    for s in an["suggestions"]:
        ov.set_by_path(cfg, s["path"], s["value"])                                   # 제안 경로·값이 실제 Config 에 들어간다
    assert set(disk["related_tunables"]) == {t.path for t in tun.TUNABLES if t.exp == eid}
    w = ctx.sim_world
    assert w.ugvs[0].cmd_v == 0.0 and w.ugvs[0].cmd_w == 0.0                           # 끝나면 정지 명령


def test_dry_run_specifics(tmp_path):
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    _, r1 = execute(EXPERIMENTS["F01"], ctx)
    m = r1["analysis"]["metrics"]
    assert m["scan"]["clock"] == "same" and m["scan"]["hz"] > 5 and m["depth"]["n"] >= 2
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    _, r3 = execute(EXPERIMENTS["F03"], ctx)
    assert len(r3["analysis"]["metrics"]["per_distance"]) == 3
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    _, r5 = execute(EXPERIMENTS["F05"], ctx)
    assert r5["analysis"]["metrics"]["max_cells_on_block"] >= 0 and r5["data"]["log"]
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    _, r7 = execute(EXPERIMENTS["F07"], ctx)
    mt = r7["analysis"]["metrics"]
    assert mt["free_ratio_min"] is not None and mt["blocked_ratio_max"] is not None
    assert mt["blocked_ratio_max"] < mt["free_ratio_min"]                            # 막힌 쪽 이동비가 더 작다(방향만 확인)


# ====================================================================== 분석 수식 (합성 데이터)
def test_f10_boresight_math_recovers_offset_and_axis_angle():
    lat, a_deg, vert, b_deg = 0.04, 0.5, -0.02, -0.3
    pts = []
    for r in (1.0, 2.5):
        pts.append({"r": r, "left_cm": 100 * (lat + r * math.tan(math.radians(a_deg))), "up_cm": 100 * (vert + r * math.tan(math.radians(b_deg)))})
    an = EXPERIMENTS["F10"].analyze({"points": pts}, Config())
    s = {x.path: x.value for x in an.suggestions}
    assert abs(s["aim.laser_lateral_m"] - lat) < 1e-3 and abs(s["aim.laser_offset_pan_deg"] - a_deg) < 0.01
    assert abs(s["aim.laser_vertical_m"] - vert) < 1e-3 and abs(s["aim.laser_offset_tilt_deg"] - b_deg) < 0.01
    bad = EXPERIMENTS["F10"].analyze({"points": [{"r": 1.0, "left_cm": 1, "up_cm": 0}, {"r": 1.2, "left_cm": 2, "up_cm": 0}]}, Config())
    assert bad.verdict == "CHECK" and not bad.suggestions                            # 거리 차이가 작으면 분리 불가


def test_f10_formula_matches_aim_controller_sign_convention():
    """F10 이 제안한 (오프셋, 축각)을 AimController 에 넣으면, 레이저 점이 닿는 곳이 '표적이 화면 중심 +b_des 에 보여야 한다'와 같은 방향이다."""
    from maicon26.mission.aim import AimController
    from maicon26.types import Detection
    cfg = Config()
    lat, a_deg, r = 0.04, 0.5, 2.0
    cfg.aim.laser_lateral_m, cfg.aim.laser_offset_pan_deg = lat, a_deg
    ac = AimController(cfg)
    want_deg = math.degrees(math.atan2(lat, r)) + a_deg                              # 점이 십자에서 왼쪽으로 벗어난 각 = 표적이 화면 왼쪽에 보여야 하는 각
    u = ac.cam.cx - ac.cam.f * math.tan(math.radians(want_deg))
    det = Detection("RADAR_FACILITY", 0.9, (u - 20, 200.0, u + 20, 280.0), extra={"sensors": [(u, 240.0, 5.0, 0.9)]})
    _, _, res = ac.update(det, 0.0, 0.0, 0.0, r)
    assert abs(res.err_pan) < 0.02                                                   # 그 자리에 보이면 조준 오차 0


def test_f09_parses_vectors_windows_and_reports_bad_input():
    e = EXPERIMENTS["F09"]
    data = {"entries": {"rig.depth_xyz": {"raw": "0.12, 0.01 0.11", "kind": "v3"}, "arena.W": {"raw": "6.2", "kind": "f"},
                        "map3d.target_window_z": {"raw": "0.33", "kind": "win"}, "rig.base_z": {"raw": "abc", "kind": "f"}, "arena.base_cy": {"raw": "1 2", "kind": "v3"}}}
    an = e.analyze(data, Config())
    s = {x.path: x.value for x in an.suggestions}
    assert s["rig.depth_xyz"] == [0.12, 0.01, 0.11] and s["arena.W"] == 6.2 and s["map3d.target_window_z"] == [0.25, 0.41]
    assert "rig.base_z" not in s and "arena.base_cy" not in s and len(an.notes) >= 2


def test_f07_analysis_separates_free_and_blocked_and_flags_odometry_slip():
    v, dt = 0.2, 0.05
    free = [{"t": i * dt, "front": 3.0 - v * i * dt, "odom_travel": v * i * dt} for i in range(64)]
    blocked = [{"t": i * dt, "front": 2.0 - 0.005 * i * dt, "odom_travel": v * i * dt} for i in range(80)]        # 실제는 거의 안 움직이는데 오도메트리는 전진
    an = EXPERIMENTS["F07"].analyze({"v": v, "phases": {"free": free, "blocked": blocked},
                                     "cfg_bump": {"window_s": 1.5, "v_min": 0.08, "ratio": 0.3}}, Config())
    assert an.verdict == "PASS" and an.metrics["detect_time_s"] is not None and not an.metrics["false_alarm_in_free"]
    assert an.suggestions and 0.1 <= an.suggestions[0].value <= 0.6
    assert any("오도메트리" in n for n in an.notes)
    same = EXPERIMENTS["F07"].analyze({"v": v, "phases": {"free": free, "blocked": free},
                                       "cfg_bump": {"window_s": 1.5, "v_min": 0.08, "ratio": 0.3}}, Config())
    assert same.verdict == "FAIL"                                                    # 자유=막힘이면 분리 불가라고 해야 한다


def test_f01_analysis_detects_other_clock_and_suggests_stale_limits():
    n = 40
    scan = [(i * 0.1, 1.7e9 + i * 0.1 - 0.05) for i in range(n)]                    # 에포크 stamp
    cam = [(i * 0.05, i * 0.05 - 0.04) for i in range(n)]
    an = EXPERIMENTS["F01"].analyze({"arrivals": {"scan": scan, "frame": cam, "depth": []}, "unsupported": ["depth"]}, Config())
    assert an.metrics["scan"]["clock"] == "offset" and an.metrics["frame"]["clock"] == "same"
    paths = {s.path: s.value for s in an.suggestions}
    assert 0.3 <= paths["mission.lidar_stale_s"] <= 1.5 and "mission.vision_stale_s" in paths
    assert an.verdict == "CHECK" and any("depth" in n_ for n_ in an.notes)


def test_f03_analysis_unit_mismatch_and_noise_fit():
    rows = [{"nominal": d, "tape": d, "n": 10, "z_median": d * 1000.0, "z_std": 1.0, "valid_ratio": 1.0} for d in (0.5, 1.0, 2.0)]     # mm 로 읽힘
    an = EXPERIMENTS["F03"].analyze({"rows": rows}, Config())
    assert any("단위" in n for n in an.notes) and any(s.path == "depth.unit_scale" for s in an.suggestions)
    k = 0.004
    rows = [{"nominal": d, "tape": d, "n": 10, "z_median": d + 0.005, "z_std": k * d * d, "valid_ratio": 0.95} for d in (0.5, 1.0, 1.5, 2.0, 3.0)]
    an = EXPERIMENTS["F03"].analyze({"rows": rows}, Config())
    s = {x.path: x.value for x in an.suggestions}
    assert abs(s["depth.sim_noise_k"] - k) < 5e-4
    assert s["depth.max_range"] == 2.0                                               # 3 m 에서 σ=3.6 cm 라 2.0 m 까지만 합격


def test_f02_f04_f05_f06_analysis_branches():
    cfg = Config()
    ok = {"fit": {"roll": 0.02, "pitch": 0.05, "roll_deg": 1.1, "pitch_deg": 2.9, "height": 0.13, "inliers": 500, "n_points": 800, "rms_m": 0.004}}
    a = EXPERIMENTS["F02"].analyze(ok, cfg)
    assert a.verdict == "PASS" and {s.path for s in a.suggestions} == {"rig.depth_rpy", "rig.depth_xyz"}
    bad = {"fit": {**ok["fit"], "pitch_deg": 40.0, "inliers": 100}}
    assert EXPERIMENTS["F02"].analyze(bad, cfg).verdict == "CHECK"
    assert EXPERIMENTS["F02"].analyze({"error": "x"}, cfg).verdict == "FAIL"
    d4 = {"report": {"lidar_resid_m": -0.12, "lidar_checks": 40, "occupied_voxels": 100}, "frames": 40, "rig_depth_xyz": [0.10, 0.0, 0.10]}
    a = EXPERIMENTS["F04"].analyze(d4, cfg)
    assert a.verdict == "CHECK" and abs(a.suggestions[0].value[0] - 0.22) < 1e-9          # 잔차 −0.12 → x 를 +0.12
    d4["report"]["lidar_resid_m"] = 0.005
    assert EXPERIMENTS["F04"].analyze(d4, cfg).verdict == "PASS"
    log = [{"t": i * 0.5, "travel": i * 0.07, "layer_in_region": 0 if i < 10 else 8, "layer_outside": 0, "lidar_front": 4.0} for i in range(30)]
    a = EXPERIMENTS["F05"].analyze({"log": log, "block_dist": 3.0}, cfg)
    assert abs(a.metrics["first_detect_range_m"] - (3.0 - 10 * 0.07)) < 1e-6 and a.verdict == "PASS"
    miss = [dict(r, layer_in_region=0) for r in log]
    a = EXPERIMENTS["F05"].analyze({"log": miss, "block_dist": 3.0}, cfg)
    assert a.verdict == "FAIL" and any(s.path == "map3d.nav_thr" and s.value < cfg.map3d.nav_thr for s in a.suggestions)
    a = EXPERIMENTS["F06"].analyze({"layer_cells_final": 9, "layer_cells_max": 12, "moved_forward": True, "report": {}}, cfg)
    assert a.verdict == "FAIL" and any(s.path == "map3d.nav_thr" and s.value > cfg.map3d.nav_thr for s in a.suggestions)
    assert EXPERIMENTS["F06"].analyze({"layer_cells_final": 0, "layer_cells_max": 1, "moved_forward": False, "report": {}}, cfg).verdict == "PASS"


# ====================================================================== 보조 부품
def test_rel_pose_handles_nonzero_start_orientation():
    o0 = Pose2D(1.0, 2.0, math.pi / 2)
    od = Pose2D(1.0, 3.0, math.pi / 2)                                              # 월드에서 +y 로 1 m = 로봇 앞으로 1 m
    r = rel_pose(o0, od)
    assert abs(r.x - 1.0) < 1e-9 and abs(r.y) < 1e-9 and abs(r.th) < 1e-9


def test_stall_detector_logic():
    d = StallDetector(1.5, 0.08, 0.3)
    t, hit = 0.0, None
    while t < 3.0 and hit is None:                                                  # 명령 0.2 m/s 인데 위치 고정
        hit = d.update(t, 0.05, 1.0, 1.0, 0.2)
        t += 0.05
    assert hit is not None and hit["moved"] < 0.01 and 1.4 <= t <= 2.0
    d = StallDetector(1.5, 0.08, 0.3)
    assert all(d.update(i * 0.05, 0.05, 0.2 * i * 0.05, 0.0, 0.2) is None for i in range(100))         # 정상 이동
    d = StallDetector(1.5, 0.08, 0.3)
    assert all(d.update(i * 0.05, 0.05, 0.0, 0.0, 0.05) is None for i in range(100))                   # 느린 명령은 판정 안 함


def test_checklist_lists_every_experiment_and_flags_sim_values():
    md = checklist_markdown(Config(), ov.empty())
    for eid in ORDER:
        assert f"## {eid} ·" in md
    assert "map3d.nav_thr" in md and "SIM_TUNED" in md and "시뮬에서 정한 값 모음" in md and "대회장 실측값이 아니다" in md
    md2 = checklist_markdown(Config(), {"values": {"map3d.nav_thr": 2.4}, "provenance": {"map3d.nav_thr": {"exp": "F06", "date": "2026-11-07"}}, "history": []})
    assert "MEASURED" in md2 and "2.4" in md2
    shipped = open(os.path.join(ROOT, "field", "FIELD_CHECKLIST.md"), encoding="utf-8").read()
    assert shipped == checklist_markdown(Config(), ov.load(ov.DEFAULT_PATH)), "field/FIELD_CHECKLIST.md 가 오래됐다 — `python scripts/field_exp.py checklist --write`"


def test_f08_capture_and_offline_replay(tmp_path):
    ctx = make_sim_ctx(str(tmp_path), seed=3, say=lambda m: None)
    _, rec = execute(EXPERIMENTS["F08"], ctx)
    d = rec["data"]["dir"]
    assert os.path.exists(os.path.join(d, "index.json")) and rec["data"]["counts"]["depth"] > 3 and rec["data"]["counts"]["scan"] > 10
    a = replay_depth(d, Config(), start=(0.95, 2.5, 0.0))
    b = replay_depth(d, Config(), start=(0.95, 2.5, 0.0), overrides={"map3d.nav_thr": 0.8, "map3d.nav_min_neighbors": 0})
    assert a["integrated"] > 0 and a["occupied_voxels"] > 0 and "layer_cells_final" in a
    assert b["layer_cells_max"] >= a["layer_cells_max"]                             # 임계를 풀면 층 셀이 줄지 않는다(튜닝 방향 확인)


class _FakeRealRobot:
    """시뮬이 아닌 '실시간 시계' 경로 시험용 최소 로봇: LiDAR 만 있고 깊이는 스텁(NotImplementedError), 시간은 time.monotonic."""
    def __init__(self):
        import time as _t
        from maicon26.hal.base import Robot
        self._t, self._t0 = _t, _t.monotonic()
        self.cmds = []
        self.__class__ = type("FakeReal", (_FakeRealRobot, Robot), {})

    def now(self): return self._t.monotonic() - self._t0
    def set_velocity(self, v, w): self.cmds.append((v, w))
    def stop(self): self.set_velocity(0.0, 0.0)
    def set_pan_tilt(self, p, t): pass
    def get_pan_tilt(self): return 0.0, 0.0
    def laser_fire(self): raise AssertionError("레이저는 --allow-laser 없이 발사되면 안 된다")
    def get_scan(self):
        from maicon26.types import Scan
        t = self.now()
        return Scan(-math.pi, 2 * math.pi / 360, np.full(360, 3.0), stamp=round(t, 1), range_max=8.0)
    def get_frame(self): return None
    def get_depth(self): raise NotImplementedError("깊이 드라이버 미연결")
    def get_odom(self): return Pose2D()
    def status(self): return None


def test_real_clock_path_interactive_input_and_stub_sensors(tmp_path):
    from maicon26.field.experiments import Ctx, NeedInput
    robot = _FakeRealRobot()
    lines = []
    ctx = Ctx(robot, Config(), str(tmp_path), interactive=False, answers={"duration_s": 1.0}, say=lines.append, robot_kind="real")
    _, rec = execute(EXPERIMENTS["F01"], ctx)                                       # time.sleep 으로 흐르는 시간, 깊이는 스텁
    assert "depth" in rec["data"]["unsupported"] and rec["analysis"]["metrics"]["scan"]["n"] >= 5
    assert rec["robot"] == "real" and "WARNING" not in rec and robot.cmds[-1] == (0.0, 0.0)
    # 대화형: 빈 입력은 기본값, 값 입력은 cast, 6개 질문에 순서대로 답한다(F10: r1, 왼쪽, 위, r2, 왼쪽, 위)
    inputs = iter(["1.0", "5", "", "2.5", "7.5", "0"])
    ctx = Ctx(robot, Config(), str(tmp_path), interactive=True, ask_fn=lambda prompt: next(inputs), say=lines.append, robot_kind="real")
    _, rec = execute(EXPERIMENTS["F10"], ctx)
    s = {x["path"]: x["value"] for x in rec["analysis"]["suggestions"]}
    # 점 5 cm @1 m, 7.5 cm @2.5 m → tan(축각)=0.025/1.5, 평행 오프셋 = 0.05 − 1.0·0.025/1.5 = 3.33 cm
    assert abs(s["aim.laser_lateral_m"] - 0.05 + 0.025 / 1.5) < 1e-3 and abs(s["aim.laser_vertical_m"]) < 1e-9
    # 대화형이 아니고 기본값 없는 질문 → NeedInput
    ctx = Ctx(robot, Config(), str(tmp_path), interactive=False, say=lines.append, robot_kind="real")
    with pytest.raises(NeedInput):
        ctx.ask("must_have", "필수 입력", None)


def test_cli_real_hal_stub_gives_friendly_error():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "field_exp.py"), "run", "F01", "--robot", "wingbot"], cwd=ROOT,
                       capture_output=True, text=True, encoding="utf-8", timeout=60)
    assert r.returncode == 3 and "스텁" in r.stdout


def test_cli_smoke(tmp_path):
    def run(*args):
        return subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "field_exp.py"), *args], cwd=ROOT, capture_output=True, text=True, encoding="utf-8", timeout=300)
    assert "F01" in run("list").stdout
    r = run("status", "--todo")
    assert r.returncode == 0 and "MEASURED" in r.stdout
    r = run("run", "F10", "--robot", "sim", "--out", str(tmp_path))
    assert r.returncode == 0 and "시뮬 드라이런" in r.stdout and "판정" in r.stdout, r.stderr[-400:]
    import glob
    rec = glob.glob(os.path.join(str(tmp_path), "**", "F10_*.json"), recursive=True)
    assert rec
    o = str(tmp_path / "o.json")
    r = run("--overrides", o, "apply", rec[0], "--yes")
    assert "건너뜀(시뮬 기록)" in r.stdout and not (tmp_path / "o.json").exists()      # 시뮬 기록은 기본으로 적용하지 않는다
    r = run("--overrides", o, "apply", rec[0], "--yes", "--force-sim")
    assert r.returncode == 0 and (tmp_path / "o.json").exists()
    assert json.load(open(tmp_path / "o.json", encoding="utf-8"))["provenance"]
