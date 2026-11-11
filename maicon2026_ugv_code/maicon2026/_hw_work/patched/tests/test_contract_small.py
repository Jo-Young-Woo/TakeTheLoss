"""하드웨어 문서 반영 3장: 작은 계약 4가지 — 시뮬 없이 돈다.

  1) 검출기 입력은 BGR 그대로(RGB 변환 금지)   2) 서버 주소 환경변수 MAICON_BASE 별칭
  3) 호출부호 형식 'R-2' (하드웨어 PDF p77 의 'R-UGV-2' 는 틀린 예시 — 2차 API 문서 기준)
  4) HP 가 0 이어도 발사 자체를 막는 조건은 없다 (10 초 피격 잠금과 주행 정지는 그대로)
"""
import importlib.util
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from hw_fakes import make_robot
from maicon26.config import Config
from maicon26.link.schema import callsign, m6_to_server
from maicon26.mission.aim import LaserManager
from maicon26.perception.detector import YoloDetector
from maicon26.types import Frame

ROOT = Path(__file__).resolve().parents[1]


# ------------------------------------------------------------------------------------------ 1) BGR 그대로
class _Boxes(list):
    pass


class _FakeModel:
    def __init__(self):
        self.seen = []

    def predict(self, image, **kw):
        self.seen.append((image, kw))
        return [SimpleNamespace(boxes=_Boxes(), names={})]


def test_detector_gets_the_bgr_array_unchanged():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:, :, 0], img[:, :, 1], img[:, :, 2] = 10, 120, 240          # B,G,R 가 서로 달라야 순서가 바뀌면 드러난다
    keep = img.copy()
    det = YoloDetector.__new__(YoloDetector)                          # ultralytics 없이 만든다
    det.model, det.conf, det.imgsz, det.device = _FakeModel(), 0.35, 640, None
    det.names_map, det.locate_sensors = {}, True
    det.detect(Frame(img, 0.0))
    (seen, kw), = det.model.seen
    assert seen is img                                                # 복사·변환 없이 같은 배열
    assert np.array_equal(seen, keep) and tuple(seen[0, 0]) == (10, 120, 240)
    assert kw["verbose"] is False


def test_no_rgb_conversion_in_detector_source():
    src = (ROOT / "maicon26" / "perception" / "detector.py").read_text(encoding="utf-8")
    code = "\n".join(l for l in src.splitlines() if not l.strip().startswith("#"))
    assert "COLOR_BGR2RGB" not in code and "[..., ::-1]" not in code and "[:, :, ::-1]" not in code


# ------------------------------------------------------------------------------------------ 2) MAICON_BASE
def _team_argv(env_pairs, argv_extra=()):
    spec = importlib.util.spec_from_file_location("contract_small_team_node", ROOT / "team" / "team_node.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    saved_env, saved_argv = dict(os.environ), list(sys.argv)
    try:
        for k in list(os.environ):
            if k.startswith("MAICON_") or k == "TEAM_LAUNCH_NAV":
                del os.environ[k]
        os.environ.update(env_pairs)
        captured = []
        mod.runpy.run_path = lambda *a, **k: captured.append(list(sys.argv))
        sys.argv = ["team_node.py", *argv_extra]
        rc = mod.main()
        return rc, (captured[-1] if captured else None)
    finally:
        os.environ.clear()
        os.environ.update(saved_env)
        sys.argv = saved_argv


def _server_of(argv):
    return argv[argv.index("--server") + 1]


def test_maicon_base_is_accepted_as_server_alias():
    rc, argv = _team_argv({"MAICON_BASE": "http://10.0.0.5:8000"})
    assert rc == 0 and _server_of(argv) == "http://10.0.0.5:8000"


def test_server_env_priority_url_then_server_then_base():
    both = {"MAICON_SERVER_URL": "http://u:1", "MAICON_SERVER": "http://s:2", "MAICON_BASE": "http://b:3"}
    assert _server_of(_team_argv(both)[1]) == "http://u:1"
    both.pop("MAICON_SERVER_URL")
    assert _server_of(_team_argv(both)[1]) == "http://s:2"
    both.pop("MAICON_SERVER")
    assert _server_of(_team_argv(both)[1]) == "http://b:3"


def test_no_server_anywhere_still_exits_2():
    rc, argv = _team_argv({})
    assert rc == 2 and argv is None


def test_explicit_server_argument_still_works_without_env():
    rc, argv = _team_argv({}, ["--server", "http://x:9"])
    assert rc == 0 and "http://x:9" in argv


# ------------------------------------------------------------------------------------------ 3) R-2
def test_callsign_is_r_dash_digit():
    # 하드웨어 PDF p77 의 'R-UGV-2' 는 틀린 예시 — 2차 API 문서 기준으로 서버가 받는 형식은 'R-2'.
    assert callsign("R-2") == "R-2"
    assert callsign("r2") == "R-2"
    assert callsign("R-UGV-2") == "R-2"           # 입력은 너그럽게 받되, 나가는 값은 항상 R-n
    assert callsign("B-UGV-3") == "B-3"
    assert callsign("R-4") is None and callsign("X-1") is None


def test_m6_payload_never_contains_ugv_in_callsign():
    out = m6_to_server({"ranking": ["R-UGV-2", "R-1", "r3", "B-1"]}, "BLUE")
    assert out == {"ranking": ["R-2", "R-1", "R-3"]}
    assert all("UGV" not in c for c in out["ranking"])


# ------------------------------------------------------------------------------------------ 4) HP 0 에서도 발사
def _mission_robot():
    robot, tr, clock = make_robot("mission", "start")
    robot.on_state(3, 3, 80.0)
    return robot, tr, clock


def test_hp_zero_does_not_by_itself_block_firing():
    robot, tr, clock = _mission_robot()
    clock.t += 1.0
    robot.on_state(0, 3, 80.0)                                        # 3번 맞아 HP 0
    st = robot.status()
    assert st.hp == 0 and st.hits_taken == 3
    assert robot.laser_fire() is False and tr.fire == []              # 피격 직후 10 초 잠금(p16)은 그대로
    assert st.disabled_left > 0
    clock.t += robot.cfg.laser.disabled_s + 0.5
    st = robot.status()
    assert st.disabled_left == 0 and st.hp == 0
    assert robot.laser_fire() is True and tr.fire == [1]              # 잠금이 풀리면 HP 0 이어도 쏜다 (p73)


def test_hp_zero_does_not_make_laser_manager_unready():
    robot, tr, clock = _mission_robot()
    robot.on_state(0, 3, 80.0)
    clock.t += 11.0
    assert LaserManager(Config(), robot).ready() is True


def test_drive_pause_for_accumulated_hits_is_kept():
    robot, tr, clock = _mission_robot()
    robot.on_state(0, 3, 80.0)
    assert robot.status().drive_paused_left > 0                       # 주행 정지는 이 장의 범위 밖 — 그대로
    clock.t += 11.0
    assert robot.status().drive_paused_left == 0


def test_no_mission_code_branches_on_hp_value():
    """미션 코드에서 hp 값으로 발사·조준을 거르는 분기가 없다 — hp 는 OLED 표시와 HAL 의 피격 계산에만 쓴다."""
    import re
    offenders = []
    for p in (ROOT / "maicon26" / "mission").rglob("*.py"):
        for n, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1):
            if line.strip().startswith("#"):
                continue
            if re.search(r"\.hp\s*(<=|<|==|>=|>)|\bhp\s*(<=|==)\s*0|hp_max\s*(<=|<|==)", line):
                offenders.append(f"{p.name}:{n}: {line.strip()}")
    assert offenders == []
