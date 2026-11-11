"""M2 uses an observed designated opponent target, not a same-class ally."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.mission.manager import Agent
from maicon26.perception.camera import CameraModel
from maicon26.perception.object_map import Face, ObjectMap
from maicon26.sim.world import Cube, Item, World
from maicon26.types import Command, Detection, Pose2D


def agent_fixture(team="BLUE", measured=False):
    cfg = Config()
    if measured:
        source = Path(__file__).resolve().parents[1] / "ros2/maicon_arena_gz/config/arena_spec.yaml"
        cfg.arena = ArenaGeometry.from_spec(yaml.safe_load(source.read_text(encoding="utf-8")))
    agent = Agent.__new__(Agent)
    agent.cfg, agent.geo = cfg, cfg.arena
    agent.team, agent.slot = team, 2
    agent.cmd = Command("COMM_FACILITY", [], None)
    agent.omap, agent.cam = ObjectMap(cfg), CameraModel(cfg.camera)
    agent.target_done = agent.target_failed = False
    agent.stats, agent.log, agent.t_start = {"led_confirmed": 0}, [], 0.
    agent.robot = SimpleNamespace(now=lambda: 12.)
    agent.laser, agent._strike = SimpleNamespace(shots=1), {"shots0": 0}
    agent._scan, agent._dth, agent._det_ctx = None, 0., None
    return agent


def face_at(x, y, cls="COMM_FACILITY", led=False):
    return Face(x-.2, y, x, y, {cls: 3.}, n=3, kind="target", led_on=led, vx=-1.)


@pytest.mark.spec("S6.1", "S6.3", "S6.5")
@pytest.mark.parametrize("team", ["BLUE", "RED"])
@pytest.mark.parametrize("measured", [False, True])
def test_only_observed_opponent_designated_faces_are_eligible_in_both_frames(team, measured):
    a = agent_fixture(team, measured)
    own_x, enemy_x = (1.5, a.geo.W-1.5) if team == "BLUE" else (a.geo.W-1.5, 1.5)
    own, opponent = face_at(own_x, 2., led=True), face_at(enemy_x, 2.)
    midpoint = face_at(a.geo.W/2, 2., led=True)
    wrong = face_at(enemy_x, 3., "GUARD_POST", led=True)
    a.omap.faces = [own, midpoint, wrong, opponent]
    assert a._target_faces() == [opponent]
    assert not a.target_done and a.stats["led_confirmed"] == 0
    opponent.led_on = True
    assert a._target_faces() == []
    assert a.target_done and a.stats["led_confirmed"] == 1
    if measured:
        assert a.geo.world_to_nav(*a.geo.nav_to_world(opponent.cx, opponent.cy)) == pytest.approx((opponent.cx, opponent.cy))


@pytest.mark.spec("S6.1", "S6.3")
@pytest.mark.parametrize("changes", [
    {"camp": "BLUE"}, {"camp": None}, {"cls": "GUARD_POST"}, {"cls": None},
    {"by": "B-UGV-1"}, {"target": 23}, {"target_id": "other-observed-device"},
])
def test_device_hit_cannot_complete_wrong_camp_kind_shooter_or_selected_id(changes):
    a = agent_fixture()
    a._strike["face"] = face_at(4.5, 2.)
    a._strike["face"].target_id = "observed-device-1"
    event = {"type": "target_hit", "by": "B-UGV-2", "cls": "COMM_FACILITY", "camp": "RED"}
    event.update(changes)
    a._on_event(12., event)
    assert not a.target_done


@pytest.mark.spec("S6.1", "S6.3")
@pytest.mark.parametrize("team,kind,shooter,camp", [
    ("BLUE", "COMM_FACILITY", "B-UGV-2", "RED"), ("RED", "comms", "R-2", "B"),
])
def test_scoped_device_hit_matches_current_observed_selection(team, kind, shooter, camp):
    a = agent_fixture(team)
    a._strike["face"] = face_at(4.5 if team == "BLUE" else 1.5, 2.)
    a._strike["face"].target_id = "observed-device-1"
    a._on_event(12., {"type": "target_hit", "shooter": shooter, "kind": kind,
                      "camp": camp, "target_id": "observed-device-1"})
    assert a.target_done


@pytest.mark.spec("S6.1", "S6.3", "S6.5")
def test_event_without_active_engagement_is_ignored_but_confirmed_led_needs_no_event():
    a = agent_fixture()
    opponent = face_at(4.5, 2.)
    event = {"type": "target_hit", "by": "B-UGV-2", "cls": "COMM_FACILITY", "camp": "RED"}
    a._on_event(12., event)
    assert not a.target_done
    a._strike.update(face=opponent, shots0=1)
    a._on_event(12., event)
    assert not a.target_done, "A delayed event must not complete a new unengaged selection"
    # A teammate may already have destroyed the designated opponent target.
    a.omap.faces = [opponent]
    opponent.led_on = True
    a._target_faces()
    assert a.target_done


@pytest.mark.spec("S6.1", "S6.3")
def test_same_class_detection_uses_capture_pose_and_selected_face_not_image_centre():
    a = agent_fixture()
    # Camera stands in COMMON. The nearer centred image belongs to our side;
    # the selected opponent is offset left in the same captured frame.
    capture_pose = Pose2D(3., 2., math.pi/2)
    a._det_ctx = (10., capture_pose, 0., 0.)
    target = face_at(4.5, 3.)
    # Project physical observation points through the same pinhole geometry.
    def detection(x, y):
        dx, dy = x-capture_pose.x, y-capture_pose.y
        angle = math.atan2(dy, dx)-capture_pose.th
        u = a.cam.cx-a.cam.f*math.tan(angle)
        depth = math.hypot(dx, dy)*math.cos(angle)
        h = a.cam.f*.4/depth
        return Detection("COMM_FACILITY", .99, (u-20, 240-h/2, u+20, 240+h/2))
    own = detection(2.8, 3.)
    opponent = detection(target.x, target.y)
    a._dets = [own, opponent]
    assert a._best_det(lambda d: d.cls == "COMM_FACILITY") is own
    assert not a._target_detection_matches(own, target)
    assert a._target_detection_matches(opponent, target)
    assert a._best_det(lambda d: a._target_detection_matches(d, target)) is opponent
    a._det_ctx = None
    assert not a._target_detection_matches(opponent, target)


@pytest.mark.spec("S6.1", "S6.3", "S6.5")
@pytest.mark.parametrize("x,camp", [(1.5, "BLUE"), (4.5, "RED")])
def test_local_referee_event_identifies_actual_hit_camp_without_layout_identity(x, camp):
    cfg = Config()
    world = World.__new__(World)
    world.cfg, world.geo, world.t, world.events = cfg, cfg.arena, 2., []
    shooter = SimpleNamespace(id="B-UGV-2", contact={}, targets_hit=[])
    item = Item("target", "COMM_FACILITY")
    cube = Cube(x, 2., {0: item})
    world._contact(shooter, ("item", id(item)), 2., ("item", cube, 0))
    assert world.events == [{"t": 2., "type": "target_hit", "by": "B-UGV-2",
                             "cls": "COMM_FACILITY", "camp": camp}]
    assert item.led_on and shooter.targets_hit == ["COMM_FACILITY"]
