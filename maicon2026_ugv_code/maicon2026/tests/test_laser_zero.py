"""텔레옵 영점(laser_zero.yaml) → aim.laser_* 4개 변환(maicon26/field/laser_zero.py, scripts/field_exp.py zero-import).
기준은 Gazebo 시뮬 모델이다: 레이저−카메라 영점 위치의 옆(+왼쪽)·위(+) 성분을 laser_lateral_m/vertical_m 로 넣고 축각은 0.
시뮬을 쓰지 않는다(순수 계산 + 파일 + 하위 프로세스로 CLI)."""
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

from maicon26.config import Config
from maicon26.field import laser_zero as lz
from maicon26.field import overrides as ov
from maicon26.field import tunables as tun
from maicon26.hal.wingbot_calibration import configure_wingbot_simulation
from maicon26.mission.aim import AimController
from maicon26.types import Detection

ROOT = Path(__file__).resolve().parents[1]
DIST = [0.6, 1.0, 1.5, 2.0]


def gazebo_cfg():
    cfg = Config()
    configure_wingbot_simulation(cfg, ROOT / "ros2/wingbot_description/config/frames.yaml")
    return cfg


def aim_err_at(cfg, u, v, d):
    """이 설정의 조준 코드가, 거리 d 에서 레이저 점이 찍히는 화면 위치(u, v)에 표적 센서가 있을 때 내는 오차(deg)."""
    cfg.aim.require_sensor = False
    ac = AimController(cfg)
    det = Detection("TANK", 0.9, (u - 30, v - 30, u + 30, v + 30), 0.0, None, {"sensors": [(u, v, 20.0, 0.9)]})
    _, _, res = ac.update(det, 0.0, 0.0, 1.0, d)
    return res.err_pan, res.err_tilt


# ------------------------------------------------------------------------------------------ Gazebo 와 같은 모델
def test_gazebo_physical_offsets_are_recovered_exactly():
    cfg = gazebo_cfg()
    cam = lz.camera_for(cfg)
    spots = lz.predict_spot_pixels(cam, cfg.aim.laser_lateral_m, cfg.aim.laser_vertical_m, 0.0, 0.0, DIST)
    res = lz.convert(lz.ZeroData(samples=spots), cam)
    assert res.values["aim.laser_lateral_m"] == pytest.approx(cfg.aim.laser_lateral_m, abs=1e-4)
    assert res.values["aim.laser_vertical_m"] == pytest.approx(0.0465, abs=1e-4)
    assert res.values["aim.laser_offset_pan_deg"] == pytest.approx(0.0, abs=1e-3)
    assert res.values["aim.laser_offset_tilt_deg"] == pytest.approx(0.0, abs=1e-3)
    assert res.rms_px < 1e-3 and res.verdict == "PASS" and not res.warnings


def test_gazebo_model_with_only_two_close_distances_stays_parallel_like_gazebo():
    cfg = gazebo_cfg()
    cam = lz.camera_for(cfg)
    spots = lz.predict_spot_pixels(cam, cfg.aim.laser_lateral_m, cfg.aim.laser_vertical_m, 0.0, 0.0, [1.0, 1.3])
    res = lz.convert(lz.ZeroData(samples=spots), cam)
    assert res.axis_mode == "parallel"
    assert res.values["aim.laser_offset_pan_deg"] == 0.0 and res.values["aim.laser_offset_tilt_deg"] == 0.0
    assert res.values["aim.laser_vertical_m"] == pytest.approx(0.0465, abs=1e-4)
    assert any("모자라" in n for n in res.notes)


@pytest.mark.parametrize("lat,ver,pan,tilt", [(0.0, 0.0465, 0.0, 0.0), (0.03, 0.045, 0.8, -0.5), (-0.02, 0.06, -0.4, 0.3)])
def test_aim_loop_puts_the_target_on_the_measured_dot_at_every_distance(lat, ver, pan, tilt):
    """변환한 값을 조준 코드에 넣으면, 영점에서 잰 점 위치에 표적이 있을 때 오차가 0 이다(각 거리마다)."""
    cfg = gazebo_cfg()
    cam = lz.camera_for(cfg)
    spots = lz.predict_spot_pixels(cam, lat, ver, pan, tilt, DIST)
    res = lz.convert(lz.ZeroData(samples=spots), cam)
    for p, v in res.values.items():
        ov.set_by_path(cfg, p, v)
    for d, u, v in spots:
        ep, et = aim_err_at(cfg, u, v, d)
        assert abs(ep) < 0.06 and abs(et) < 0.06, (d, ep, et)


def test_without_the_conversion_the_same_dots_are_far_off_target():
    """변환 전(기본 0)에는 가까운 거리에서 오차가 1° 를 넘는다 — 변환이 필요한 이유."""
    cfg = gazebo_cfg()
    cfg.aim.laser_lateral_m = cfg.aim.laser_vertical_m = 0.0
    cam = lz.camera_for(cfg)
    d, u, v = lz.predict_spot_pixels(cam, 0.0, 0.0465, 0.0, 0.0, [0.6])[0]
    assert abs(aim_err_at(cfg, u, v, d)[1]) > 1.0


# ------------------------------------------------------------------------------------------ 축각·식(fit)·화각
def test_axis_angle_is_recovered_with_three_or_more_distances():
    cfg = Config()
    cam = lz.camera_for(cfg)
    spots = lz.predict_spot_pixels(cam, 0.03, 0.045, 0.8, -0.5, DIST)
    res = lz.convert(lz.ZeroData(samples=spots), cam)
    assert res.axis_mode == "free"
    assert res.values["aim.laser_lateral_m"] == pytest.approx(0.03, abs=2e-3)
    assert res.values["aim.laser_vertical_m"] == pytest.approx(0.045, abs=2e-3)
    assert res.values["aim.laser_offset_pan_deg"] == pytest.approx(0.8, abs=0.05)
    assert res.values["aim.laser_offset_tilt_deg"] == pytest.approx(-0.5, abs=0.05)
    assert res.rms_px < 0.05 and max(abs(r["err_u"]) for r in res.per_sample) < 0.05        # 맞춘 모델이 점 위치를 되짚는다


def test_fit_only_follows_the_small_angle_formulas_of_the_analysis_doc():
    """u(d)=u0+au/d, v(d)=v0+av/d → lateral ≈ −au/fx, vertical ≈ −av/fy, 축각 = atan((cx−u0)/fx)."""
    cam = lz.camera_for(Config())
    u0, au, v0, av = cam.cx - 3.0, -7.0, cam.cy + 2.0, -10.4
    res = lz.convert(lz.ZeroData(fit=(u0, au, v0, av)), cam, distances=DIST)
    assert res.source == "fit"
    assert res.values["aim.laser_lateral_m"] == pytest.approx(-au / cam.f, abs=1e-3)
    assert res.values["aim.laser_vertical_m"] == pytest.approx(-av / cam.fy, abs=1e-3)
    assert res.values["aim.laser_offset_pan_deg"] == pytest.approx(math.degrees(math.atan2(cam.cx - u0, cam.f)), abs=0.05)
    assert res.values["aim.laser_offset_tilt_deg"] == pytest.approx(math.degrees(math.atan2(cam.cy - v0, cam.fy)), abs=0.05)


def test_fit_only_without_distances_uses_default_distances_and_says_so():
    cam = lz.camera_for(Config())
    res = lz.convert(lz.ZeroData(fit=(cam.cx, 0.0, cam.cy, -10.0)), cam)
    assert res.d_range == (0.6, 2.0) and any("기본 거리" in w for w in res.warnings)


def test_focal_length_comes_from_the_runtime_camera_not_from_the_zero_tools_assumed_fov():
    """같은 픽셀 식이라도 런타임 초점거리가 두 배면 옆 거리는 절반이다(각도 변환은 런타임 카메라로)."""
    fit = (320.0, 0.0, 240.0, -10.0)
    a, b = Config(), Config()
    b.camera.K = (2 * lz.camera_for(a).f, 0.0, 320.0, 0.0, 2 * lz.camera_for(a).fy, 240.0, 0.0, 0.0, 1.0)
    ra = lz.convert(lz.ZeroData(fit=fit), lz.camera_for(a), distances=DIST)
    rb = lz.convert(lz.ZeroData(fit=fit), lz.camera_for(b), distances=DIST)
    assert ra.values["aim.laser_vertical_m"] == pytest.approx(2.0 * rb.values["aim.laser_vertical_m"], rel=0.02)
    assert ra.rms_px < 0.05 and rb.rms_px < 0.05


def test_assumed_fov_mismatch_is_reported_as_a_note():
    cam = lz.camera_for(Config())
    data = lz.ZeroData(fit=(320.0, 0.0, 240.0, -10.0), assumed_hfov_deg=114.6)
    res = lz.convert(data, cam, distances=DIST, runtime_hfov_deg=110.0)
    assert any("화각" in n for n in res.notes)


# ------------------------------------------------------------------------------------------ 입력이 나쁠 때
def test_single_distance_assumes_parallel_axis_and_warns():
    cam = lz.camera_for(Config())
    res = lz.convert(lz.ZeroData(samples=[(1.0, cam.cx, cam.cy - 10.0)]), cam)
    assert res.axis_mode == "parallel" and any("1곳" in w for w in res.warnings)
    with pytest.raises(lz.ZeroFormatError):
        lz.convert(lz.ZeroData(samples=[(1.0, cam.cx, cam.cy - 10.0)]), cam, axis="free")


def test_samples_closer_than_the_aim_floor_are_dropped():
    cam = lz.camera_for(Config())
    s = lz.predict_spot_pixels(cam, 0.0, 0.0465, 0.0, 0.0, [0.5, 1.0, 1.5, 2.0]) + [(0.2, cam.cx, cam.cy)]
    res = lz.convert(lz.ZeroData(samples=s), cam)
    assert res.n_samples == 4 and any("0.3 m 미만" in w for w in res.warnings)
    with pytest.raises(lz.ZeroFormatError):
        lz.convert(lz.ZeroData(samples=[(0.2, cam.cx, cam.cy)]), cam)


def test_huge_offset_or_axis_error_is_flagged_for_checking():
    cam = lz.camera_for(Config())
    bad = [(d, cam.cx + 90.0 / d, cam.cy - 60.0 / d) for d in DIST]           # 옆 40 cm·위 27 cm 쯤 — 부호나 거리 입력 실수
    res = lz.convert(lz.ZeroData(samples=bad), cam)
    assert res.verdict == "CHECK" and any(w.startswith("!") for w in res.warnings)


def test_noisy_dots_give_a_large_rms_and_a_warning():
    cam = lz.camera_for(Config())
    s = lz.predict_spot_pixels(cam, 0.0, 0.0465, 0.0, 0.0, DIST)
    noisy = [(d, u + (6.0 if i % 2 else -6.0), v) for i, (d, u, v) in enumerate(s)]
    res = lz.convert(lz.ZeroData(samples=noisy), cam)
    assert res.rms_px > lz.RMS_PX_GOOD and res.verdict == "CHECK" and any("rms" in w for w in res.warnings)


def test_resolution_difference_is_scaled_and_aspect_mismatch_is_refused():
    cam = lz.camera_for(Config())                                              # 640×480
    half = [(d, u / 2, v / 2) for d, u, v in lz.predict_spot_pixels(cam, 0.0, 0.0465, 0.0, 0.0, DIST)]
    res = lz.convert(lz.ZeroData(samples=half, image_size=(320, 240)), cam)
    assert res.values["aim.laser_vertical_m"] == pytest.approx(0.0465, abs=1e-3) and any("비례" in w for w in res.warnings)
    with pytest.raises(lz.ZeroFormatError):
        lz.convert(lz.ZeroData(samples=half, image_size=(640, 360)), cam)


def test_parse_accepts_alias_keys_and_reports_what_is_missing(tmp_path):
    doc = {"fit": {"u_inf": 321.0, "a_u": -5.0, "v_inf": 239.0, "a_v": -10.0},
           "samples": [{"distance_m": 1.0, "px_u": 316.0, "px_v": 229.0}, {"dist": 0, "u": 1, "v": 1}, "junk"],
           "camera": {"width": 640, "height": 480, "hfov_deg": 114.6}, "rms_px": 0.8}
    z = lz.parse_zero(doc)
    assert z.fit == (321.0, -5.0, 239.0, -10.0) and z.samples == [(1.0, 316.0, 229.0)]
    assert z.image_size == (640, 480) and z.assumed_hfov_deg == 114.6 and z.rms_px == 0.8
    with pytest.raises(lz.ZeroFormatError):
        lz.parse_zero({"fit": {"u0": 1.0}})
    with pytest.raises(lz.ZeroFormatError):
        lz.parse_zero({"derived": {}})
    p = tmp_path / "z.yaml"
    p.write_text(yaml.safe_dump(doc), encoding="utf-8")
    assert lz.load_zero(str(p)).fit == (321.0, -5.0, 239.0, -10.0)


def test_record_has_the_same_shape_as_field_experiment_records():
    cfg = Config()
    cam = lz.camera_for(cfg)
    res = lz.convert(lz.ZeroData(samples=lz.predict_spot_pixels(cam, 0.0, 0.0465, 0.0, 0.0, DIST)), cam)
    rec = lz.to_record(res, cfg, "/x/laser_zero.yaml")
    assert rec["id"] == "ZERO" and rec["robot"] == "real" and rec["analysis"]["verdict"] == "PASS"
    assert [s["path"] for s in rec["analysis"]["suggestions"]] == list(lz.PATHS)
    assert any("카메라 K" in n for n in rec["analysis"]["notes"])                  # 기본 설정에는 K 가 없다 — 다시 변환하라는 안내
    json.dumps(rec)                                                            # JSON 으로 쓸 수 있다
    for p in lz.PATHS:
        tun.resolve(cfg, p)                                                    # 튜너블 경로가 실제 설정 경로다


def test_tunable_hints_point_to_the_zero_import_tool():
    by = {t.path: t for t in tun.TUNABLES}
    assert all("zero-import" in by[p].how for p in lz.PATHS)


# ------------------------------------------------------------------------------------------ CLI
def run_cli(*args):
    return subprocess.run([sys.executable, str(ROOT / "scripts" / "field_exp.py"), *args], cwd=str(ROOT), capture_output=True,
                          text=True, encoding="utf-8", timeout=120)


def write_zero(tmp_path, lat=0.0, ver=0.0465):
    cam = lz.camera_for(Config())
    spots = lz.predict_spot_pixels(cam, lat, ver, 0.0, 0.0, DIST)
    p = tmp_path / "laser_zero.yaml"
    p.write_text(yaml.safe_dump({"fit": {"u0": cam.cx, "au": -lat * cam.f, "v0": cam.cy, "av": -ver * cam.fy},
                                 "samples": [{"d": d, "u": u, "v": v} for d, u, v in spots], "rms_px": 0.4}), encoding="utf-8")
    return p


def test_cli_converts_records_and_applies_with_provenance(tmp_path):
    zero = write_zero(tmp_path, lat=0.02, ver=0.05)
    o, out = tmp_path / "o.json", tmp_path / "rec"
    r = run_cli("--overrides", str(o), "zero-import", str(zero), "--out", str(out), "--apply", "--yes")
    assert r.returncode == 0, r.stdout[-600:] + r.stderr[-600:]
    data = ov.load(str(o))
    assert data["values"]["aim.laser_lateral_m"] == pytest.approx(0.02, abs=2e-3)
    assert data["values"]["aim.laser_vertical_m"] == pytest.approx(0.05, abs=2e-3)
    assert all(data["provenance"][p]["exp"] == "ZERO" for p in lz.PATHS)
    assert list(out.glob("ZERO_*.json"))
    r = run_cli("--overrides", str(o), "status")
    assert "aim.laser_vertical_m" in r.stdout and "ZERO" in r.stdout
    cfg = ov.load_config(None, str(o))                                          # 실행 쪽이 이 파일을 읽으면 값이 들어간다
    assert cfg.aim.laser_vertical_m == pytest.approx(0.05, abs=2e-3)


def test_cli_without_apply_writes_a_record_but_not_the_overrides(tmp_path):
    zero = write_zero(tmp_path)
    o, out = tmp_path / "o.json", tmp_path / "rec"
    r = run_cli("--overrides", str(o), "zero-import", str(zero), "--out", str(out))
    assert r.returncode == 0 and "--apply" in r.stdout
    assert not o.exists() and list(out.glob("ZERO_*.json"))


def test_cli_sim_records_are_refused_by_apply_unless_forced(tmp_path):
    zero = write_zero(tmp_path)
    o, out = tmp_path / "o.json", tmp_path / "rec"
    r = run_cli("--overrides", str(o), "zero-import", str(zero), "--out", str(out), "--robot", "sim", "--apply", "--yes")
    assert r.returncode == 0 and "건너뜀" in r.stdout and not o.exists()
    r = run_cli("--overrides", str(o), "zero-import", str(zero), "--out", str(out), "--robot", "sim", "--apply", "--yes", "--force-sim")
    assert r.returncode == 0 and o.exists()


def test_cli_bad_file_gives_a_friendly_error_without_writing(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("derived: {}\n", encoding="utf-8")
    o, out = tmp_path / "o.json", tmp_path / "rec"
    r = run_cli("--overrides", str(o), "zero-import", str(bad), "--out", str(out), "--apply", "--yes")
    assert r.returncode == 2 and "변환 실패" in r.stdout and not o.exists() and not out.exists()
    r = run_cli("--overrides", str(o), "zero-import", str(tmp_path / "missing.yaml"))
    assert r.returncode == 2
