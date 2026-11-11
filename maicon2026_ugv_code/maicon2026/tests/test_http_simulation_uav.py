"""Actual saved Gazebo JPEG through HTTP, async delivery and Agent coverage.

No Gazebo/ROS/network process, layout truth or synthetic object detections.
Only the navigation pose is an inert fixture; no motion strategy is exercised.
"""
import datetime as dt
import io
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.link.async_link import AsyncServerLink
from maicon26.link.http import HttpServerLink
from maicon26.mission.manager import Agent
from maicon26.nav.grid import OccupancyGrid
from maicon26.perception.simulation_uav import load_simulation_uav_detector
from maicon26.sim.mockserver import FakeClock, MockServer
from maicon26.types import Pose2D


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT/"ros2/maicon_arena_gz"
CALIBRATION = PACKAGE/"config/uav_grid_calibration.json"
CODE = "UavTest8"


def measured_config():
    cfg = Config()
    cfg.arena = ArenaGeometry.from_spec(yaml.safe_load((PACKAGE/"config/arena_spec.yaml").read_text(encoding="utf-8")))
    cfg.team, cfg.slot = "BLUE", 2
    cfg.hal.restart_state_path = None
    return cfg


def connection(*, callback=True):
    pytest.importorskip("cv2")
    cfg = measured_config()
    detector = load_simulation_uav_detector(CALIBRATION, cfg.arena)
    clock = FakeClock(10.)
    mock = MockServer({CODE: "B-2"}, clock=clock)
    mock.set_phase("running")
    link = HttpServerLink("http://fixture.invalid", CODE, clock=clock, transport=mock,
                          uav_layout_detector=detector.detect if callback else None)
    assert link.me()["phase"] == "running"
    link._started_bg = True
    return cfg, clock, mock, link


def blank_jpeg():
    stream = io.BytesIO()
    Image.new("RGB", (1280, 720), "white").save(stream, "JPEG")
    return stream.getvalue()


@pytest.mark.spec("S4.1", "S4.5", "S4.6")
def test_actual_api_jpeg_reaches_agent_as_twenty_unclassified_footprints(monkeypatch):
    cfg, clock, mock, http = connection()
    jpeg = (PACKAGE/"validation/api_uav_live.jpg").read_bytes()
    # Exercise the actual worker body deterministically, without threads racing
    # FakeClock or requesting unrelated long-poll endpoints.
    monkeypatch.setattr(threading.Thread, "start", lambda _: None)
    transport = AsyncServerLink(http)
    robot = SimpleNamespace(get_scan=lambda: None, get_frame=lambda: None, get_depth=lambda: None,
                            get_odom=lambda: Pose2D(), now=lambda: clock()+90.)
    grid = OccupancyGrid(cfg.arena)
    navigation = SimpleNamespace(initialized=True, pose=Pose2D(.34, 2.4), grid=grid,
                                 update=lambda *args: None, clear_goal=lambda: None)
    agent = Agent(robot, transport, cfg=cfg, stack=navigation)
    observed = []
    agent.tracker = SimpleNamespace(update_uav=lambda frame, pose, **kwargs: observed.append(frame))
    before_grid = grid.L.copy()
    for index in range(2):
        if index:
            clock.advance(1.)
        mock.capture(jpeg)
        transport._uav_once()
        frame = transport.get_uav()
        assert frame.jpeg == jpeg and frame.image.shape == (720, 1280, 3)
        assert len(frame.boxes) == 20 and frame.detections == []
        assert frame.layout_source == "gazebo_uav_image" and frame.layout_stamp is None
        expected = (mock.epoch+dt.timedelta(seconds=clock())).timestamp()
        assert frame.stamp == expected and frame.age_s == pytest.approx(0.)
        assert agent._sense(robot.now(), .05)
    assert agent.coverage.uav_layout_state == "available"
    assert len(agent.coverage.boxes) == 20
    assert all(box.kind == "uav" and all(face.det_n == 0 for face in box.faces)
               for box in agent.coverage.boxes.values())
    assert agent.coverage.m3_counts(agent.omap) == {}
    assert np.array_equal(grid.L, before_grid), "Image hints must not write hidden obstacles into SLAM"
    assert 99. < agent.coverage._layout_stamp <= 101., "Coverage must use robot time, not HTTP epoch"
    assert observed and all(frame.stamp < 200 for frame in observed)


def test_blank_images_do_not_fabricate_boxes_and_default_http_has_no_simulation_detector():
    _, _, mock, link = connection()
    mock.capture(blank_jpeg())
    frame = link.get_uav()
    assert frame.boxes == frame.detections == []
    assert frame.layout_source == "gazebo_uav_image"
    _, _, mock, real_default = connection(callback=False)
    mock.capture((PACKAGE/"validation/api_uav_live.jpg").read_bytes())
    frame = real_default.get_uav()
    assert frame.image is not None and frame.boxes == [] and frame.layout_source is None


@pytest.mark.parametrize("age", [13., -3.])
def test_stale_or_substantially_future_capture_never_runs_layout_inference(age):
    _, _, mock, link = connection()
    mock.capture(blank_jpeg())
    original = link._transport
    called = []
    link.uav_layout_detector = lambda image: called.append(image) or []
    def shifted(request, timeout):
        status, body, headers = original(request, timeout)
        headers = dict(headers)
        captured = dt.datetime.fromisoformat(headers["X-Captured-At"])-dt.timedelta(seconds=age)
        headers["X-Captured-At"] = captured.isoformat()
        return status, body, headers
    link._transport = shifted
    frame = link.get_uav()
    assert frame is not None and frame.jpeg and not called
    assert frame.boxes == [] and frame.layout_source is None
    assert link.stats["drone_layout_rejected"] == 1


def test_decode_or_inference_error_is_safe_and_duplicate_is_not_new_geometry():
    _, clock, mock, link = connection()
    mock.capture(blank_jpeg())
    def failing(image):
        raise RuntimeError("unsafe detail "+CODE)
    link.uav_layout_detector = failing
    frame = link.get_uav()
    assert frame.layout_source is None and frame.boxes == []
    assert CODE not in link.last_error and "unsafe detail" not in link.last_error
    clock.advance(1.)
    assert link.get_uav() is None
    mock.capture(b"invalid JPEG")
    # Cadence is still enforced after the duplicate response above.
    clock.advance(1.)
    frame = link.get_uav()
    assert frame is not None and frame.image is None and frame.layout_source is None


def test_loader_requires_matching_measured_dimensions_and_independent_validation(tmp_path):
    cfg = measured_config()
    with pytest.raises(ValueError, match="measured arena"):
        load_simulation_uav_detector(CALIBRATION, ArenaGeometry())
    cfg.arena.W += .12
    with pytest.raises(ValueError, match="dimensions differ"):
        load_simulation_uav_detector(CALIBRATION, cfg.arena)
    data = json.loads(CALIBRATION.read_text(encoding="utf-8"))
    data["checkpoints"] = []
    unvalidated = tmp_path/"unvalidated.json"
    unvalidated.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="independent checkpoints"):
        load_simulation_uav_detector(unvalidated, measured_config().arena)
