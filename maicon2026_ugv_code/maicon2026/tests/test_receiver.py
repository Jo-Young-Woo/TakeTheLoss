"""수광센서(원) 겨냥: 영상에서 센서 찾기(합성 이미지) + 조준 컨트롤러 + 시뮬 종단 시험."""
import math

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.mission.aim import AimController
from maicon26.perception.receiver import attach_learned_receivers, find_receivers
from maicon26.sim.run import build_match
from maicon26.types import Detection

cv2 = pytest.importorskip("cv2")


def _scene():
    return np.full((480, 640, 3), (90, 100, 95), np.uint8)


def _near(res, u, v, tol=1.5):
    assert res, "센서를 찾지 못했다"
    assert math.hypot(res[0][0] - u, res[0][1] - v) <= tol, res


# ====================================================================== 영상에서 센서 찾기 (합성 이미지)
def test_finds_dark_circle_on_white_panel():
    im = _scene(); cv2.rectangle(im, (300, 200), (400, 300), (245, 245, 245), -1); cv2.circle(im, (352, 238), 9, (40, 40, 40), -1)
    _near(find_receivers(im, (290, 190, 410, 310), 1), 352, 238)


def test_finds_white_circle_on_dark_panel():
    im = _scene(); cv2.rectangle(im, (300, 200), (400, 300), (60, 60, 60), -1); cv2.circle(im, (340, 262), 8, (250, 250, 250), -1)
    _near(find_receivers(im, (290, 190, 410, 310), 1), 340, 262)


def test_ignores_distractors_on_the_white_panel():
    im = _scene(); cv2.rectangle(im, (300, 200), (400, 300), (245, 245, 245), -1)
    cv2.rectangle(im, (310, 270), (380, 280), (30, 30, 30), -1); cv2.rectangle(im, (360, 210), (390, 225), (20, 20, 20), -1)
    cv2.circle(im, (335, 240), 10, (35, 35, 35), -1)
    _near(find_receivers(im, (290, 190, 410, 310), 1), 335, 240)


def test_ugv_returns_both_sensors_left_to_right_and_skips_wheel_like_circle():
    im = _scene(); cv2.rectangle(im, (250, 220), (390, 330), (70, 70, 80), -1)
    cv2.circle(im, (285, 250), 9, (250, 250, 250), -1); cv2.circle(im, (355, 251), 9, (250, 250, 250), -1)
    cv2.circle(im, (320, 312), 7, (200, 200, 200), -1)                  # 높이가 다른 방해 원
    res = find_receivers(im, (245, 215, 395, 335), 2)
    assert len(res) == 2 and res[0][0] < res[1][0]
    assert math.hypot(res[0][0] - 285, res[0][1] - 250) < 1.5 and math.hypot(res[1][0] - 355, res[1][1] - 251) < 1.5


def test_no_sensor_returns_empty_and_small_noisy_circle_is_found():
    im = _scene(); cv2.rectangle(im, (300, 200), (400, 300), (245, 245, 245), -1)
    assert find_receivers(im, (290, 190, 410, 310), 1) == []
    rng = np.random.default_rng(0)
    im = _scene(); cv2.rectangle(im, (300, 200), (370, 260), (240, 240, 240), -1); cv2.circle(im, (333, 228), 4, (40, 40, 40), -1)
    im = np.clip(im.astype(int) + rng.normal(0, 6, im.shape), 0, 255).astype(np.uint8)
    _near(find_receivers(im, (295, 195, 375, 265), 1), 333, 228)
    assert find_receivers(None, (0, 0, 50, 50)) == []                   # 영상이 없으면(시뮬) 빈 목록


# ---- PDF 사진 모양: 표적 = 사진 상단 중앙의 순백색 원판(구름 하늘 위), UGV = 검은 차체 위 좌/우 흰 돔
def _sky_face(seed=0, size=200, x0=220, y0=120, cloud=True):
    rng = np.random.default_rng(seed)
    im = _scene()
    face = np.zeros((size, size, 3), np.float32)
    for i in range(size):                                            # 하늘 그라데이션(위: 짙은 파랑 → 아래: 옅은 회청)
        face[i, :] = (235 - 0.25 * i, 190 + 0.1 * i, 120 + 0.35 * i)
    if cloud:                                                        # 흐릿한 구름(밝지만 얼룩지고 가장자리가 부드럽다)
        cl = np.zeros((size, size), np.float32)
        for (cx, cy, rx, ry) in ((60, 55, 38, 16), (150, 40, 30, 12), (95, 150, 55, 20)):
            cv2.ellipse(cl, (cx, cy), (rx, ry), rng.integers(0, 30), 0, 360, 1.0, -1)
        cl = cv2.GaussianBlur(cl, (0, 0), 7)
        face = face * (1 - cl[..., None]) + 246.0 * cl[..., None]
    face += rng.normal(0, 4, face.shape)
    im[y0:y0 + size, x0:x0 + size] = np.clip(face, 0, 255).astype(np.uint8)
    return im, (x0, y0, x0 + size, y0 + size)


def test_target_white_disc_found_on_cloudy_photo_face():
    for seed in range(3):
        im, box = _sky_face(seed)
        cx, cy = box[0] + 100, box[1] + 32                           # 가로 50 %, 세로 16 %
        cv2.circle(im, (cx, cy), 15, (255, 255, 255), -1)             # 지름 약 15 %
        res = find_receivers(im, box, 1, kind="target")
        _near(res, cx, cy, 1.5)
        assert 13 <= res[0][2] <= 17


def test_target_cloud_alone_is_not_a_sensor():
    im, box = _sky_face(1)
    assert find_receivers(im, box, 1, kind="target") == [] or find_receivers(im, box, 1, kind="target")[0][3] < 0.5


def test_target_disc_off_center_still_found_but_prior_prefers_top_center():
    im, box = _sky_face(2, cloud=False)
    cv2.circle(im, (box[0] + 100, box[1] + 32), 15, (255, 255, 255), -1)          # 진짜 센서(상단 중앙)
    cv2.circle(im, (box[0] + 40, box[1] + 160), 15, (255, 255, 255), -1)          # 하단의 비슷한 흰 원(방해)
    res = find_receivers(im, box, 1, kind="target")
    _near(res, box[0] + 100, box[1] + 32, 1.5)


def _ugv_scene(dome_l=(105, 215), dome_r=(215, 217)):
    im = _scene()
    cv2.rectangle(im, (90, 150), (250, 300), (28, 28, 30), -1)                    # 검은 차체(bbox 160x150)
    for (cx, cy) in (dome_l, dome_r):                                              # 음영이 있는 하얀 돔
        cv2.ellipse(im, (cx, cy), (16, 13), 0, 0, 360, (205, 205, 208), -1)
        cv2.ellipse(im, (cx - 3, cy - 3), (11, 8), 0, 0, 360, (248, 248, 250), -1)
    cv2.circle(im, (150, 260), 4, (255, 255, 255), -1); cv2.circle(im, (186, 261), 4, (255, 255, 255), -1)   # 정면 램프(작다)
    cv2.rectangle(im, (115, 275), (225, 297), (212, 212, 215), -1)                  # 은색 명판(가로로 길다)
    cv2.line(im, (120, 286), (220, 286), (60, 60, 60), 2)
    return im, (85.0, 145.0, 255.0, 305.0)


def test_ugv_white_domes_found_as_a_left_right_pair():
    im, box = _ugv_scene()
    res = find_receivers(im, box, 2, kind="ugv")
    assert len(res) == 2 and res[0][0] < res[1][0]
    assert math.hypot(res[0][0] - 105, res[0][1] - 215) < 2.0 and math.hypot(res[1][0] - 215, res[1][1] - 217) < 2.0, res
    assert all(10 <= r[2] <= 18 for r in res)                                     # 램프(r≈4)·명판이 아니라 돔


def test_ugv_one_dome_visible_returns_the_single_dome():
    im, box = _ugv_scene()
    cv2.rectangle(im, (190, 190), (240, 245), (28, 28, 30), -1)                    # 오른쪽 돔을 가린다
    res = find_receivers(im, box, 2, kind="ugv")
    assert len(res) == 1 and res[0][0] < 150


def test_learned_receiver_boxes_are_attached_to_their_parent():
    parent = Detection("UGV", 0.9, (200, 200, 400, 340), 1.0)
    other = Detection("DRUM", 0.9, (10, 10, 60, 80), 1.0)
    s1 = Detection("RECEIVER", 0.8, (240, 240, 260, 260), 1.0)
    s2 = Detection("RECEIVER", 0.7, (330, 241, 350, 261), 1.0)
    out = attach_learned_receivers([parent, other, s1, s2])
    assert [d.cls for d in out] == ["UGV", "DRUM"]
    sens = out[0].extra["sensors"]
    assert [round(s[0]) for s in sens] == [250, 340] and "sensors" not in out[1].extra


# ====================================================================== 조준 컨트롤러
def _det(cls, box, sensors=None):
    d = Detection(cls, 0.9, box, 1.0)
    if sensors is not None:
        d.extra["sensors"] = sensors
    return d


def test_aims_at_the_sensor_not_the_box_center_and_follows_one_side():
    cfg = Config()
    ac = AimController(cfg)
    box = (200.0, 150.0, 440.0, 330.0)                                    # 중심 (320, 240)
    sens = [(262.0, 290.0, 10.0, 0.9), (378.0, 291.0, 10.0, 0.9)]
    pan_c, tilt_c, res = ac.update(_det("UGV", box, sens), 0.0, 0.0, 0.0, 1.5)
    assert res.sensor_locked and res.sensor_ok
    assert pan_c > 0 and tilt_c < 0                                       # 왼쪽 센서(262, 290): 왼쪽(+팬)·아래(-틸트)로 돈다
    for k in range(1, 6):                                                 # 두 센서가 보여도 처음 고른 왼쪽을 계속 쫓는다
        pan_c, tilt_c, res = ac.update(_det("UGV", box, sens), pan_c, tilt_c, 0.05 * k, 1.5)
    assert ac._last_pt[0] < 320


def test_tolerance_shrinks_to_sensor_radius():
    cfg = Config()
    ac = AimController(cfg)
    # 가운데 정렬된 센서에서 0.5° 어긋난 상태: 큰 센서면 정렬, 작은 센서면 아직 아님
    f = ac.cam.f
    off = f * math.tan(math.radians(0.5))
    near = [(320.0 + off, 240.0, 3.0, 0.9)]
    big = [(320.0 + off, 240.0, 14.0, 0.9)]
    _, _, r_small = AimController(cfg).update(_det("DRUM", (280, 200, 360, 280), near), 0.0, 0.0, 0.0, 1.2)
    _, _, r_big = AimController(cfg).update(_det("DRUM", (280, 200, 360, 280), big), 0.0, 0.0, 0.0, 1.2)
    assert r_big.aligned and not r_small.aligned


def test_without_sensor_firing_waits_for_fallback_then_allows():
    cfg = Config()
    ac = AimController(cfg)
    d = _det("RADAR_FACILITY", (280.0, 200.0, 360.0, 280.0))
    _, _, r0 = ac.update(d, 0.0, 0.0, 0.0, 1.2)
    assert not r0.sensor_locked and not r0.sensor_ok                      # 센서 미확보 → 발사 보류
    t, r1 = 0.0, r0
    while t < cfg.aim.sensor_fallback_s + 0.1:                            # 센서 없는 검출이 연속으로 들어오는 동안(공백이 있으면 추적이 새로 시작된다)
        t += 0.1
        _, _, r1 = ac.update(d, 0.0, 0.0, t, 1.2)
        assert r1.sensor_ok == (t >= cfg.aim.sensor_fallback_s)
    assert r1.sensor_ok                                                   # 끝내 못 찾으면 대체 추측 허용
    cfg2 = Config(); cfg2.aim.require_sensor = False
    assert AimController(cfg2).update(d, 0.0, 0.0, 0.0, 1.2)[2].sensor_ok
    ac.update(_det("RADAR_FACILITY", (280.0, 200.0, 360.0, 280.0), [(320.0, 215.0, 6.0, 0.9)]), 0.0, 0.0, 3.0, 1.2)
    assert ac._no_sensor_since is None                                    # 찾으면 타이머 해제


# ====================================================================== 시뮬 종단
def _strip_sensors(monkeypatch):
    from maicon26.sim.world import World
    orig = World.camera_detections

    def stripped(self, u):
        ds = orig(self, u)
        for d in ds:
            d.extra.pop("sensors", None)
        return ds

    monkeypatch.setattr(World, "camera_detections", stripped)


def test_subgame_hits_the_disc_even_when_the_box_excludes_it():
    """bbox 가 시설 물체만 덮고 흰 원판은 그 위 하늘에 있어도(PDF p12 사진), 센서를 직접 겨냥하면 맞힌다."""
    from maicon26.sim.subgame import run_try
    cfg = Config()
    cfg.perception.sim_target_box = "object"
    for s in range(3):
        r = run_try(s, cfg)
        assert r["ok"] and r["t"] < 25.0, (s, r)


def test_bbox_ratio_guess_cannot_hit_when_the_disc_is_outside_the_box(monkeypatch):
    """대조군: 센서 위치 없이 bbox 비율로 추측하면(예전 방식) 같은 조건에서 못 맞힌다 — 센서 찾기가 필요한 이유."""
    from maicon26.sim.subgame import run_try
    _strip_sensors(monkeypatch)
    cfg = Config()
    cfg.perception.sim_target_box = "object"
    cfg.aim.require_sensor = False
    assert not any(run_try(s, cfg)["ok"] for s in range(3))


def test_require_sensor_holds_fire_until_fallback_in_sim(monkeypatch):
    """센서를 못 찾으면 첫 sensor_fallback_s 동안은 발사하지 않는다(8발을 추측에 쓰지 않는다)."""
    from maicon26.sim.subgame import build_subgame
    _strip_sensors(monkeypatch)
    cfg = Config()
    cfg.perception.sim_target_box = "object"
    w, run = build_subgame(0, cfg)
    first_shot = None
    for _ in range(int(40 / 0.05)):
        w.step(0.05)
        run.tick()
        if run.laser.shots and first_shot is None:
            first_shot = w.t
            break
    # 표적을 처음 조준 단계에서 보기 시작한 시각 이후 sensor_fallback_s 가 지나기 전에는 쏘지 않았다
    assert first_shot is None or first_shot >= cfg.aim.sensor_fallback_s


def test_sim_detections_carry_sensor_positions_inside_their_boxes():
    m = build_match(seed=1, cfg=Config(), teams=["B2", "R2"])
    w = m.world
    n_t = n_u = 0
    for _ in range(int(60 / 0.05)):
        for a in m.agents:
            a.tick()
        w.step(0.05)
        for r in m.robots:
            for d in w.camera_detections(r.u):
                for s in d.extra.get("sensors", []):
                    kind = d.extra.get("kind")
                    # 표적: 수광창은 면 중앙이라 지터 낀 bbox 안쪽. UGV: 가까우면 원근 때문에 bbox 가장자리를 조금 넘을 수 있다
                    mx = 6.0 if kind == "target" else 0.3 * (d.box[2] - d.box[0]) + 12
                    my = 6.0 if kind == "target" else 0.3 * (d.box[3] - d.box[1]) + 12
                    assert d.box[0] - mx <= s[0] <= d.box[2] + mx and d.box[1] - my <= s[1] <= d.box[3] + my, (kind, d.box, s)
                    n_t += kind == "target"
                    n_u += kind == "ugv"
    assert n_t > 0                                                        # 표적 면을 봤을 때 센서 위치가 함께 온다
