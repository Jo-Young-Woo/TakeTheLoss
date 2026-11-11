"""실기 UAV 상자 검출기(maicon26/perception/uav_boxes_real.py).

실제 UAV 영상은 아직 없다. 두 가지 입력으로 시험한다.
  · 패키지에 저장된 Gazebo 렌더 2장(측정 경기장·보정·20개 상자 배치가 있어 정답과 대조할 수 있다)
  · 보정 모델로 직접 그린 합성 장면(핀홀 투영: 윗면 z=0.4, 옆면, 벽 안쪽면, UGV 크기 물체 …)
실제 조명·색·보정은 연습 날 scripts/uav_boxes_tool.py 로 맞춘다(문턱은 설정 키).
"""
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

cv2 = pytest.importorskip("cv2")

from maicon26.arena import ArenaGeometry                                    # noqa: E402
from maicon26.config import Config, UavBoxesCfg                             # noqa: E402
from maicon26.link.http import HttpServerLink                               # noqa: E402
from maicon26.perception.uav_boxes_real import (                            # noqa: E402
    RealUavBoxDetector, load_real_uav_detector, probe_points, rects_from_flat, suggest_thresholds)
from maicon26.perception.uav_calibration import load_calibration            # noqa: E402
from maicon26.sim.mockserver import FakeClock, MockServer                   # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2/maicon_arena_gz"
CALIBRATION = PACKAGE / "config/uav_grid_calibration.json"
LAYOUT = PACKAGE / "config/layouts/layout_default.yaml"
IMAGES = ["uav_render_gpu_6ugv.png", "api_uav_live.jpg"]
CODE = "UavReal8"

CAL = load_calibration(CALIBRATION)
W_PX, H_PX = CAL.image_size
AW, AH = CAL.arena_width, CAL.arena_height


def truth():
    layout = yaml.safe_load(LAYOUT.read_text(encoding="utf-8"))
    return [(b["center"][0] + AW / 2, b["center"][1] + AH / 2) for b in layout["boxes"]]


def detector(**over):
    cfg = UavBoxesCfg()
    for key, value in over.items():
        setattr(cfg, key, value)
    return RealUavBoxDetector(CAL, cfg, camera="main")


def match(boxes, expected, tol=0.2):
    """(최대 오차, 정답에 없는 상자 수, 못 찾은 정답 수)"""
    errs, missing = [], 0
    for e in expected:
        d = min((math.hypot(b.cx - e[0], b.cy - e[1]) for b in boxes), default=9.0)
        errs.append(d)
        missing += d > tol
    extra = sum(1 for b in boxes if min(math.hypot(b.cx - e[0], b.cy - e[1]) for e in expected) > tol) if expected else len(boxes)
    return max(errs) if errs else 0.0, extra, missing


def read(name):
    img = cv2.imread(str(PACKAGE / "validation" / name))
    assert img is not None
    return img


# --------------------------------------------------------------------------- 합성 장면
def _px(points, z):
    inverse = np.linalg.inv(CAL.plane_homography(z))
    p = np.hstack([np.asarray(points, float), np.ones((len(points), 1))]) @ inverse.T
    return p[:, :2] / p[:, 2:3]


def _fill(img, points, z, color):
    cv2.fillPoly(img, [np.round(_px(points, z) * 16).astype(np.int32)], color, lineType=cv2.LINE_AA, shift=4)


def render(boxes=(), blobs=(), floor=(52, 58, 50), top=(14, 14, 14), side=(60, 100, 170), wall=(18, 18, 18),
           noise=2.0, seed=0, wall_h=0.30, led=True):
    """보정 모델 그대로(윗면 z=0.4 평면, 바닥 z=0 평면) 그린 영상. 카메라 중심은 보정 파일의 camera_center."""
    img = np.empty((H_PX, W_PX, 3), np.uint8)
    img[:] = wall
    _fill(img, [(0, 0), (AW, 0), (AW, AH), (0, AH)], 0.0, floor)
    for a, b in (((0, 0), (AW, 0)), ((AW, 0), (AW, AH)), ((AW, AH), (0, AH)), ((0, AH), (0, 0))):
        quad = np.vstack([_px([a, b], 0.0), _px([a, b], wall_h)[::-1]])
        cv2.fillPoly(img, [np.round(quad * 16).astype(np.int32)], wall, lineType=cv2.LINE_AA, shift=4)
    cx, cy, _ = CAL.camera_center
    for (x, y) in sorted(boxes, key=lambda p: -math.hypot(p[0] - cx, p[1] - cy)):
        h = 0.2
        corners = [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            quad = np.vstack([_px([a, b], 0.0), _px([a, b], 0.4)[::-1]])
            cv2.fillPoly(img, [np.round(quad * 16).astype(np.int32)], side, lineType=cv2.LINE_AA, shift=4)
        _fill(img, corners, 0.4, top)
        if led:                                                                  # 윗면의 빨간 LED/수광부 구멍
            p = _px([(x, y)], 0.4)[0]
            cv2.circle(img, (int(p[0]), int(p[1])), 3, (40, 40, 230), -1)
    for (x, y, sx, sy, z, color) in blobs:
        _fill(img, [(x - sx / 2, y - sy / 2), (x + sx / 2, y - sy / 2), (x + sx / 2, y + sy / 2), (x - sx / 2, y + sy / 2)], z, color)
    if noise:
        img = np.clip(img.astype(float) + np.random.default_rng(seed).normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


def jpeg(img, quality=85):
    ok, data = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    assert ok
    return data.tobytes()


def centres(boxes):
    return sorted((round(b.cx, 2), round(b.cy, 2)) for b in boxes)


# --------------------------------------------------------------------------- 저장된 Gazebo 렌더 대조
@pytest.mark.parametrize("name", IMAGES)
def test_package_renders_give_all_twenty_boxes_with_no_extras(name):
    boxes = detector().detect(read(name))
    err, extra, missing = match(boxes, truth())
    assert len(boxes) == 20 and missing == 0 and extra == 0
    assert err < 0.03                                                       # 측정 격자 보정 오차 안(실측 약 1.5 cm)
    assert all(b.confidence >= 0.6 and b.sx == b.sy == 0.4 for b in boxes)


def _lift(img, off):
    return np.clip(img.astype(int) + off, 0, 255).astype(np.uint8)


def _gamma(img, g):
    return (255 * (img / 255.0) ** g).astype(np.uint8)


def _noise(img, sigma):
    return np.clip(img.astype(float) + np.random.default_rng(1).normal(0, sigma, img.shape), 0, 255).astype(np.uint8)


def _gradient(img):
    return np.clip(img * np.linspace(0.6, 1.3, img.shape[1])[None, :, None], 0, 255).astype(np.uint8)


PERTURB = {
    "gain0.7": lambda im: np.clip(im.astype(float) * 0.7, 0, 255).astype(np.uint8),
    "gain1.3": lambda im: np.clip(im.astype(float) * 1.3, 0, 255).astype(np.uint8),
    "lift20": lambda im: _lift(im, 20),
    "gamma1.3": lambda im: _gamma(im, 1.3),
    "gamma0.6": lambda im: _gamma(im, 0.6),
    "jpeg25": lambda im: cv2.imdecode(np.frombuffer(jpeg(im, 25), np.uint8), 1),
    "noise10": lambda im: _noise(im, 10),
    "blur2": lambda im: cv2.GaussianBlur(im, (0, 0), 2.0),
    "gradient": _gradient,
}


@pytest.mark.parametrize("kind", sorted(PERTURB))
def test_package_render_survives_exposure_noise_blur_and_compression(kind):
    img = PERTURB[kind](read("api_uav_live.jpg"))
    boxes = detector().detect(img)
    err, extra, missing = match(boxes, truth())
    assert (len(boxes), missing, extra) == (20, 0, 0), kind
    assert err < 0.05


def test_lifted_blacks_give_no_boxes_but_a_note_and_a_repeated_warning(caplog):
    lifted = _lift(read("api_uav_live.jpg"), 45)                           # 검정이 문턱보다 밝아진 프레임(조명·노출 문제)
    det = detector()
    rep = det.analyze(lifted)
    assert rep.ok and rep.boxes == [] and rep.note == "no_dark_region" and rep.summary()["note"] == "no_dark_region"
    with caplog.at_level("WARNING", logger="maicon26.uav_boxes"):
        for _ in range(5):
            assert det.detect(lifted) == []
    assert det.stats["zero_streak"] == 5
    assert sum("상자 0개" in r.getMessage() for r in caplog.records) == 1       # 5장째에 한 번만
    det.detect(read("api_uav_live.jpg"))
    assert det.stats["zero_streak"] == 0                                    # 상자가 다시 잡히면 연속 횟수는 0 으로


def test_a_far_too_dark_exposure_fails_safe_instead_of_guessing():
    dark = _gamma(read("api_uav_live.jpg"), 1.5)                           # 바닥 밝기가 검정 문턱 가까이까지 내려간 노출
    rep = detector().analyze(dark)
    assert not rep.ok and rep.reason == "low_contrast" and rep.boxes == []


def test_pixel_shift_from_a_slightly_wrong_calibration_still_finds_the_boxes():
    img = read("api_uav_live.jpg")
    for shift in (3, 8):
        moved = cv2.warpAffine(img, np.float32([[1, 0, shift], [0, 1, shift]]), (W_PX, H_PX))
        boxes = detector().detect(moved)
        _, extra, missing = match(boxes, truth(), tol=0.25)
        assert missing == 0 and extra == 0, shift


# --------------------------------------------------------------------------- 합성 장면
def test_synthetic_scene_boxes_are_located_and_adjacent_boxes_are_split():
    placed = [(1.0, 1.0), (2.0, 1.0), (2.4, 1.0), (4.5, 3.5), (3.0, 2.4), (1.2, 3.8)]
    rep = detector().analyze(render(placed))
    assert rep.ok and len(rep.boxes) == len(placed)
    err, extra, missing = match(rep.boxes, placed, tol=0.1)
    assert missing == 0 and extra == 0 and err < 0.03
    assert rep.components < len(placed)                                    # 붙은 두 상자는 덩어리 하나에서 두 칸으로 갈렸다


def test_side_faces_in_the_top_plane_view_are_not_extra_boxes():
    """카메라가 윗면 평면 위에 있어 옆면은 펴진 영상에서 카메라 쪽으로 번진다 — 그 번짐이 새 상자가 되면 안 된다."""
    placed = [(0.8, 0.8), (5.2, 4.0), (5.2, 0.8), (0.8, 4.0), (3.0, 1.4)]
    rep = detector().analyze(render(placed, side=(120, 135, 160)))
    assert match(rep.boxes, placed, tol=0.1)[1:] == (0, 0)


def test_ugv_sized_and_small_dark_objects_are_not_boxes():
    blobs = [(1.5, 3.0, 0.20, 0.20, 0.10, (14, 14, 14)),                   # UGV 크기 검정 물체(0.28 m 미만)
             (4.0, 3.0, 0.12, 0.30, 0.05, (14, 14, 14))]
    rep = detector().analyze(render([(2.8, 1.5)], blobs))
    assert rep.ok and centres(rep.boxes) == [(2.8, 1.5)]
    assert rep.rejects.get("small", 0) >= 1


def test_a_huge_dark_region_is_rejected_instead_of_tiled_into_boxes():
    rep = detector().analyze(render([], [(3.0, 2.4, 3.8, 0.6, 0.4, (14, 14, 14))]))
    assert rep.ok and rep.boxes == [] and rep.rejects.get("huge") == 1


def test_black_mat_without_contrast_or_a_covered_floor_is_not_used():
    with pytest.raises(ValueError, match="low_contrast"):
        detector().detect(render([(2.0, 1.0)], floor=(14, 14, 14)))        # 바닥이 검정 문턱과 구분되지 않는다
    with pytest.raises(ValueError, match="dark_frame"):
        detector().detect(np.zeros((H_PX, W_PX, 3), np.uint8))
    with pytest.raises(ValueError, match="bright_frame"):
        detector().detect(np.full((H_PX, W_PX, 3), 255, np.uint8))


def test_unusable_frames_raise_and_are_counted_without_touching_good_state():
    det = detector()
    good = render([(2.0, 1.0)])
    assert len(det.detect(good)) == 1
    for bad, reason in ((np.zeros((10, 10, 3), np.uint8), "size_mismatch"), (good.astype(np.float32), "bad_frame"),
                        (good[:, :, 0], "bad_frame"), (None, "bad_frame")):
        with pytest.raises(ValueError, match=reason):
            det.detect(bad)
    assert det.stats["frames"] == 5 and det.stats["frames_ok"] == 1 and det.stats["unusable"] == 4
    assert det.stats["boxes_last"] == 0 and det.last.reason == "bad_frame"
    assert len(det.detect(good)) == 1                                      # 나쁜 프레임 뒤에도 정상 동작


def test_too_many_boxes_frame_is_dropped_as_a_whole():
    many = [(0.6 + 0.8 * i, 0.8 + 0.8 * j) for i in range(7) for j in range(5)]
    rep = detector(max_boxes=10).analyze(render(many))
    assert not rep.ok and rep.reason == "too_many" and rep.boxes == []


def test_ignore_rects_remove_fixed_structures_and_are_reported():
    placed = [(1.0, 1.0), (2.0, 1.0)]
    rep = detector(ignore_rects=(0.7, 0.7, 1.3, 1.3)).analyze(render(placed))
    assert centres(rep.boxes) == [(2.0, 1.0)]
    assert any(c.status == "ignored" and c.reason == "ignore_zone" for c in rep.cells)
    assert rects_from_flat((0, 0, 1, 1, 2, 2, 3, 4)) == [(0, 0, 1, 1), (2, 2, 3, 4)]
    with pytest.raises(ValueError):
        rects_from_flat((0, 0, 1))


def test_boxes_within_the_edge_margin_are_not_reported_but_ones_clear_of_walls_are():
    rep = detector().analyze(render([(0.2, 2.4), (0.8, 2.4)]))
    assert centres(rep.boxes) == [(0.8, 2.4)]
    assert detector(edge_margin_m=0.0).analyze(render([(0.2, 2.4)])).rejects.get("edge", 0) == 0


def test_grey_topped_boxes_need_the_grey_mode():
    img = render([(1.5, 1.2), (4.2, 3.0)], top=(130, 130, 130), floor=(52, 58, 50))
    assert detector(top_color="black").analyze(img).boxes == []
    for mode in ("grey", "both"):
        rep = detector(top_color=mode).analyze(img)
        assert len(rep.boxes) == 2 and match(rep.boxes, [(1.5, 1.2), (4.2, 3.0)], tol=0.1)[1:] == (0, 0), mode


def test_detection_survives_jpeg_and_confidence_is_in_the_accepted_range():
    placed = [(1.4, 1.0), (3.6, 2.2), (4.8, 3.6)]
    decoded = cv2.imdecode(np.frombuffer(jpeg(render(placed), 40), np.uint8), 1)
    rep = detector().analyze(decoded)
    assert match(rep.boxes, placed, tol=0.1)[1:] == (0, 0)
    assert all(0.6 <= b.confidence <= 0.98 for b in rep.boxes)


def test_detection_is_fast_enough_for_a_one_hertz_stream():
    det = detector()
    img = render([(1.0, 1.0), (2.0, 1.0), (4.0, 3.0)])
    det.detect(img)
    assert det.last.elapsed_ms < 400                                       # 1 FPS 영상에서 여유(CI 가 느려도 통과하는 상한)


# --------------------------------------------------------------------------- 연습용 측정·제안
def test_probe_and_threshold_suggestion_put_the_threshold_between_top_and_floor():
    det = detector()
    img = render([(1.8, 1.0), (4.2, 3.5)])
    box = probe_points(det, img, [(1.8, 1.0), (4.2, 3.5)], plane_z=0.4)
    floor = probe_points(det, img, [(1.0, 3.0), (5.0, 2.0), (2.5, 3.0)], plane_z=0.0)
    assert all(p["v_p80"] < 45 for p in box) and all(p["v_p20"] > 45 for p in floor)
    s = suggest_thresholds(box, floor)
    assert s["ok"] and s["top_color"] == "black"
    assert 20 <= s["v_black_max"] <= 45 and 0.2 <= s["v_black_rel"] <= 0.9
    # 제안값으로 실제 검출이 되는지
    rep = detector(v_black_max=s["v_black_max"], v_black_rel=s["v_black_rel"]).analyze(img)
    assert len(rep.boxes) == 2 and match(rep.boxes, [(1.8, 1.0), (4.2, 3.5)], tol=0.1)[1:] == (0, 0)


def test_threshold_suggestion_never_exceeds_what_the_frame_guard_accepts():
    box = [{"n": 9, "v_p20": 14.0, "v_p50": 15.0, "v_p80": 16.0, "s_p50": 30.0}]
    for floor_v in (30.0, 35.0, 57.0, 90.0, 140.0):
        floor = [{"n": 9, "v_p20": floor_v - 1, "v_p50": floor_v, "v_p80": floor_v + 1, "s_p50": 30.0}]
        s = suggest_thresholds(box, floor)
        if s["ok"]:
            thr = min(s["v_black_max"], max(s["v_black_rel"] * floor_v, 12))
            assert floor_v >= 1.6 * thr and thr >= 16.0, (floor_v, s)           # 문턱이 윗면보다 높고 low_contrast 로 버려지지 않는다
        else:
            assert floor_v < 40                                                 # 바닥이 너무 어두울 때만 제안을 포기한다


def test_threshold_suggestion_gives_up_when_top_and_floor_look_the_same():
    box = [{"v_p20": 50.0, "v_p50": 52.0, "v_p80": 54.0, "v_p95": 60.0, "s_p50": 20.0}]
    floor = [{"v_p20": 49.0, "v_p50": 52.0, "v_p80": 56.0, "v_p95": 60.0, "s_p50": 20.0}]
    assert suggest_thresholds(box, floor)["ok"] is False
    assert suggest_thresholds([], floor)["ok"] is False


# --------------------------------------------------------------------------- 로더·HTTP 훅·진입점
def _measured_arena():
    return ArenaGeometry.from_spec(yaml.safe_load((PACKAGE / "config/arena_spec.yaml").read_text(encoding="utf-8")))


def test_loader_requires_matching_arena_dimensions_and_cube_size():
    arena = _measured_arena()
    det = load_real_uav_detector(str(CALIBRATION), arena, UavBoxesCfg(), camera="main")
    assert det.camera == "main" and isinstance(det, RealUavBoxDetector)
    with pytest.raises(ValueError, match="다르다"):
        load_real_uav_detector(str(CALIBRATION), ArenaGeometry(), UavBoxesCfg())      # 6.12×5.0 설정과 6.0×4.8 보정
    with pytest.raises((OSError, ValueError)):
        load_real_uav_detector(str(CALIBRATION.parent / "none.json"), arena, UavBoxesCfg())
    with pytest.raises(ValueError):
        RealUavBoxDetector(CAL, SimpleNamespace(**{**UavBoxesCfg().__dict__, "ppm": 10}))
    with pytest.raises(TypeError):
        RealUavBoxDetector(object(), UavBoxesCfg())


def _link(det, *, camera="main"):
    clock = FakeClock(10.0)
    mock = MockServer({CODE: "B-2"}, clock=clock)
    mock.set_phase("running")
    link = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=mock,
                          uav_layout_detector=det, uav_layout_source="real_uav_image")
    assert link.me()["phase"] == "running"
    link._started_bg = True
    return clock, mock, link


def test_http_hook_labels_real_footprints_and_counts_them():
    placed = [(1.0, 1.0), (2.0, 1.0), (4.5, 3.5)]
    clock, mock, link = _link(detector())
    mock.capture(jpeg(render(placed)))
    frame = link.get_uav()
    assert frame.layout_source == "real_uav_image" and len(frame.boxes) == 3
    assert match(frame.boxes, placed, tol=0.1)[1:] == (0, 0)
    assert link.stats["drone_layout_ok"] == 1


def test_http_hook_never_applies_a_main_calibration_to_a_different_camera_header():
    det = detector()
    det.camera = "test2"                                                    # 검출기는 test2 용인데 서버는 main 프레임을 준다
    clock, mock, link = _link(det)
    mock.capture(jpeg(render([(1.0, 1.0)])))
    frame = link.get_uav()
    assert frame is not None and frame.boxes == [] and frame.layout_source is None
    assert link.stats["drone_layout_rejected"] == 1 and det.stats["frames"] == 0


def test_http_hook_counts_an_unusable_frame_and_keeps_running():
    clock, mock, link = _link(detector())
    mock.capture(jpeg(np.zeros((H_PX, W_PX, 3), np.uint8)))
    frame = link.get_uav()
    assert frame.boxes == [] and frame.layout_source is None
    assert link.stats["drone_layout_rejected"] == 1
    clock.advance(1.0)
    mock.capture(jpeg(render([(2.0, 1.0)])))
    frame = link.get_uav()
    assert frame is not None and len(frame.boxes) == 1 and frame.layout_source == "real_uav_image"


def test_http_link_rejects_a_blank_layout_source_name():
    with pytest.raises(ValueError):
        HttpServerLink("http://fixture.invalid", CODE, clock=FakeClock(1.0), transport=MockServer({CODE: "B-2"}, clock=FakeClock(1.0)),
                       uav_layout_detector=detector(), uav_layout_source="")


# ------- run_real 진입점(ROS·하드웨어·네트워크 없이) — tests/test_real_runtime.py 의 runtime 고정물을 쓴다
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_real_runtime import runtime  # noqa: E402,F401  (pytest fixture)


def _capture_link_kwargs(runtime, monkeypatch):
    seen = {}
    original = runtime.Link.__init__

    def init(self, *args, **kwargs):
        seen.update(kwargs)
        original(self, *args, **kwargs)
    monkeypatch.setattr(runtime.Link, "__init__", init)
    return seen


def _real_cfg(runtime, **ub):
    runtime.cfg.arena = _measured_arena()
    runtime.cfg.uav_boxes = UavBoxesCfg(**ub)
    runtime.cfg.explore = SimpleNamespace(uav_layout_timeout_s=12.0)
    runtime.cfg.nav2 = SimpleNamespace(start_pose_override=None)             # 측정 경기장은 시작 위치를 nav2 에 알린다


def test_run_real_builds_the_real_detector_from_the_config(runtime, monkeypatch):
    seen = _capture_link_kwargs(runtime, monkeypatch)
    _real_cfg(runtime, enable=True, calibration=str(CALIBRATION), camera="main")
    runtime.mod.main()
    assert isinstance(seen["uav_layout_detector"], RealUavBoxDetector)
    assert seen["uav_layout_source"] == "real_uav_image" and seen["uav_layout_detector"].camera == "main"


def test_run_real_without_a_calibration_file_only_warns_when_configured(runtime, monkeypatch, capsys):
    seen = _capture_link_kwargs(runtime, monkeypatch)
    _real_cfg(runtime, enable=True, calibration="calib/does_not_exist.json")
    runtime.mod.main()
    assert "uav_layout_detector" not in seen and "WARN UAV" in capsys.readouterr().out


def test_run_real_stays_off_by_default_and_rejects_bad_explicit_calibration(runtime, monkeypatch, tmp_path):
    seen = _capture_link_kwargs(runtime, monkeypatch)
    _real_cfg(runtime)                                                      # enable=False
    runtime.mod.main()
    assert "uav_layout_detector" not in seen
    runtime.argv += ["--uav-calibration", str(tmp_path / "missing.json")]
    with pytest.raises(SystemExit) as exc:
        runtime.mod.main()
    assert exc.value.code == 2


def test_run_real_explicit_calibration_and_camera_choice(runtime, monkeypatch):
    seen = _capture_link_kwargs(runtime, monkeypatch)
    _real_cfg(runtime)
    runtime.argv += ["--uav-calibration", str(CALIBRATION), "--uav-camera", "test1"]
    runtime.mod.main()
    assert seen["uav_layout_detector"].camera == "test1" and seen["uav_layout_source"] == "real_uav_image"


def test_run_real_refuses_to_mix_simulation_and_real_uav_calibration(runtime):
    runtime.argv += ["--uav-calibration", str(CALIBRATION), "--simulation-uav-calibration", str(CALIBRATION),
                     "--arena-spec", str(PACKAGE / "config/arena_spec.yaml")]
    with pytest.raises(SystemExit) as exc:
        runtime.mod.main()
    assert exc.value.code == 2


# --------------------------------------------------------------------------- 연습 도구
def _tool(*args):
    return subprocess.run([sys.executable, str(ROOT / "scripts/uav_boxes_tool.py"), *args], capture_output=True, text=True,
                          cwd=str(ROOT), timeout=120)


def test_practice_tool_detect_writes_overlay_and_json(tmp_path):
    frame = tmp_path / "frame.jpg"
    frame.write_bytes((PACKAGE / "validation/api_uav_live.jpg").read_bytes())
    out = tmp_path / "res"
    r = _tool("detect", "--image", str(frame), "--calibration", str(CALIBRATION), "--out", str(out),
              "--sectors", "--arena-spec", str(PACKAGE / "config/arena_spec.yaml"))
    assert r.returncode == 0, r.stderr
    assert "상자 20개" in r.stdout
    for suffix in ("_overlay.png", "_top.png", "_mask.png", ".json"):
        assert Path(str(out) + suffix).is_file(), suffix
    data = json.loads(Path(str(out) + ".json").read_text(encoding="utf-8"))
    assert data["summary"]["ok"] is True and len(data["boxes"]) == 20


def test_practice_tool_set_overrides_and_rejects_unknown_keys(tmp_path):
    frame = tmp_path / "frame.jpg"
    frame.write_bytes((PACKAGE / "validation/api_uav_live.jpg").read_bytes())
    ok = _tool("detect", "--image", str(frame), "--calibration", str(CALIBRATION), "--set", "top_color=grey")
    assert ok.returncode == 0 and "상자 0개" in ok.stdout                    # 검정 윗면을 회색 모드로 찾으면 안 잡힌다
    bad = _tool("detect", "--image", str(frame), "--calibration", str(CALIBRATION), "--set", "no_such_key=1")
    assert bad.returncode != 0 and "알 수 없는 uav_boxes 키" in (bad.stderr + bad.stdout)


def test_practice_tool_probe_prints_a_config_snippet(tmp_path):
    frame = tmp_path / "frame.jpg"
    frame.write_bytes((PACKAGE / "validation/api_uav_live.jpg").read_bytes())
    r = _tool("probe", "--image", str(frame), "--calibration", str(CALIBRATION), "--box", "1.8", "1.0", "--box", "4.2", "3.79",
              "--floor", "1.0", "3.0", "--floor", "5.0", "2.0")
    assert r.returncode == 0, r.stderr
    assert '"uav_boxes"' in r.stdout and "v_black_max" in r.stdout


def test_default_config_keeps_the_detector_off():
    cfg = Config()
    assert cfg.uav_boxes.enable is False and cfg.uav_boxes.calibration is None and cfg.uav_boxes.camera == "main"
