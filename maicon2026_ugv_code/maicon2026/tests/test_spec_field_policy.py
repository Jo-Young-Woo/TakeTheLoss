"""S9.7 -> requested 225 s own-base strategy, without simulator ground truth."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.config import Config
from maicon26.link.base import ServerLink
from maicon26.mission.field_policy import OwnBasePolicy
from maicon26.mission.manager import Agent
from maicon26.nav.grid import OccupancyGrid, line_free
from maicon26.nav.nav2_stack import Anchor, Nav2Client, Nav2Stack
from maicon26.nav.navigator import Navigator
from maicon26.perception.camera import CameraModel
from maicon26.perception.coverage import CoverageMap
from maicon26.sim.fake_nav2 import FakeNav2Client
from maicon26.types import Pose2D
from test_spec_conformance import NoSensorRobot
from maicon26.sim.mockserver import FakeClock

pytestmark = pytest.mark.spec("S9.7")
ROOT = Path(__file__).resolve().parents[1]


def setup_policy(team="BLUE", pose=None, elapsed=225.):
    cfg = Config()
    grid = OccupancyGrid(cfg.arena, cfg.nav.res)
    policy = OwnBasePolicy(cfg)
    pose = pose or Pose2D(3., 2.5, 0.)
    mask = policy.planning_mask(grid, pose, team, elapsed)
    grid.set_keepout(mask)
    return cfg, grid, policy, pose


@pytest.mark.spec("S9.7")
@pytest.mark.parametrize("team", ["BLUE", "RED"])
def test_activation_at_225_only_blocks_own_three_bases_and_can_reset(team):
    cfg, grid, policy, pose = setup_policy(team, elapsed=224.999)
    assert not policy.active and grid.keepout is None
    mask = policy.planning_mask(grid, pose, team, 225.)
    assert policy.active
    for name in cfg.arena.bases(team):
        assert mask[grid.to_cell(*cfg.arena.base_center(name))]
    for name in cfg.arena.bases(cfg.arena.opponent(team)):
        assert not mask[grid.to_cell(*cfg.arena.base_center(name))]
    assert policy.planning_mask(grid, pose, team, 0.) is None
    assert not policy.active


def test_measured_yaml_bases_use_nav_frame_translation_not_legacy_coordinates():
    cfg = Config()
    spec = yaml.safe_load((ROOT / "ros2/maicon_arena_gz/config/arena_spec.yaml").read_text(encoding="utf-8"))
    cfg.arena = ArenaGeometry.from_spec(spec)
    grid = OccupancyGrid(cfg.arena, cfg.nav.res)
    policy = OwnBasePolicy(cfg)
    mask = policy.planning_mask(grid, Pose2D(3, 2.4, 0), "BLUE", 225.)
    # Printed B-2 world[-2.97,-2.35] x [-.4,.4], explicitly translated.
    assert policy.rectangles["B-2"] == pytest.approx((.03, 2., .65, 2.8))
    assert mask[grid.to_cell(.34, 2.4)]
    assert not mask[grid.to_cell(5.66, 2.4)]


def test_policy_never_becomes_lidar_occupied_cells_or_erases_depth():
    cfg, grid, _, _ = setup_policy()
    cell = grid.to_cell(*cfg.arena.base_center("B-2"))
    before = grid.L.copy()
    assert not grid.occupied()[cell]
    blocked, _ = grid.cost_maps(.15, .3, 1.)
    assert blocked[cell]
    coverage = CoverageMap(cfg, CameraModel(cfg.camera))
    assert coverage.planning_cost_maps(grid)[0][cell]
    assert np.array_equal(grid.L, before)
    assert not coverage.visibility_occupancy(grid)[cell]
    depth = np.zeros_like(grid.static)
    depth[grid.to_cell(3., 3.)] = True
    grid.set_layer3d(depth)
    assert grid.occupied()[grid.to_cell(3., 3.)]
    grid.set_keepout(None)
    assert grid.occupied()[grid.to_cell(3., 3.)]


@pytest.mark.spec("S9.7")
@pytest.mark.parametrize("team,pose,inward,outward", [
    ("BLUE", Pose2D(.9, 2.5, math.pi), .2, -.2),
    ("RED", Pose2D(5.22, 2.5, 0.), .2, -.2),
])
def test_initial_footprint_overlap_can_leave_but_cannot_go_deeper(team, pose, inward, outward):
    _, _, policy, _ = setup_policy(team, pose)
    assert policy.limit_velocity(pose, inward, 0., .05) == (0., 0.)
    assert policy.limit_velocity(pose, outward, 0., .05) == (outward, 0.)


def test_fully_contained_robot_cannot_move_deeper_even_when_intersection_area_is_constant():
    _, _, policy, pose = setup_policy(pose=Pose2D(.65, 2.5, 0.))
    assert policy.limit_velocity(pose, -.1, 0., .05) == (0., 0.)
    assert policy.limit_velocity(pose, .1, 0., .05) == (.1, 0.)


def test_outer_wall_is_not_used_as_the_escape_edge():
    _, _, policy, pose = setup_policy(pose=Pose2D(.2, 2.5, 0.))
    assert policy.limit_velocity(pose, .15, 0., .05) == (.15, 0.)
    assert policy.limit_velocity(pose, -.15, 0., .05) == (0., 0.)


def test_near_side_edge_does_not_hide_deeper_inward_motion():
    _, _, policy, pose = setup_policy(pose=Pose2D(.65, 2.91, 0.))
    assert policy.limit_velocity(pose, -.1, 0., .05) == (0., 0.)


def test_rotating_corner_is_blocked_while_robot_center_remains_outside():
    _, _, policy, pose = setup_policy(pose=Pose2D(.96, 2.5, 0.))
    assert pose.x > .8
    assert policy.limit_velocity(pose, 0., 1.5, .05) == (0., 0.)


def test_diagonal_swept_footprint_cannot_cross_a_base_corner():
    cfg, _, policy, _ = setup_policy()
    _, _, x1, y1 = cfg.arena.base_rect("B-2")
    pose = Pose2D(x1+.2, y1+.2, -3*math.pi/4)
    assert policy.limit_velocity(pose, .5, 0., .05) == (0., 0.)


def test_safe_turn_remains_available_when_translation_would_enter():
    _, _, policy, pose = setup_policy(pose=Pose2D(.98, 2.5, math.pi))
    assert policy.limit_velocity(pose, .3, .1, .05) == (0., .1)


def test_nonfinite_active_command_fails_closed():
    _, _, policy, pose = setup_policy()
    assert policy.limit_velocity(pose, math.nan, 0., .05) == (0., 0.)


def test_escape_corridor_excludes_deeper_half_then_closes_after_exit():
    cfg, grid, policy, pose = setup_policy(pose=Pose2D(.6, 2.5, 0.))
    assert not grid.keepout[grid.to_cell(pose.x, pose.y)]
    assert grid.keepout[grid.to_cell(.3, 2.5)]
    assert policy.escape_goal(grid, pose)[0] > cfg.arena.base_rect("B-2")[2]
    assert grid.keepout[grid.to_cell(*cfg.arena.base_center("B-1"))]
    outside = Pose2D(1.1, 2.5, 0.)
    mask = policy.planning_mask(grid, outside, "BLUE", 226.)
    assert not policy.escape_cuts and policy.escape_goal(grid, outside) is None
    assert mask[grid.to_cell(.6, 2.5)]
    assert policy.limit_velocity(outside, -.6, 0., .05) == (0., 0.)


def test_builtin_replans_existing_path_around_keepout_without_changing_perception():
    cfg = Config()
    nav = Navigator(cfg)
    pose, goal = Pose2D(.95, 1.7, math.pi/2), (.95, 3.3)
    nav.set_goal(goal)
    assert nav._plan(pose, 224.)
    policy = OwnBasePolicy(cfg)
    mask = policy.planning_mask(nav.grid, pose, "BLUE", 225.)
    assert nav.grid.set_keepout(mask)
    before = nav.replans
    nav.step(pose, None, 225., .05)
    assert nav.replans > before
    assert nav.path and max(p[0] for p in nav.path) > 1.02
    assert all(line_free(nav.grid, mask, a, b) for a, b in zip(nav.path[:-1], nav.path[1:]))
    assert not nav.grid.occupied()[nav.grid.to_cell(.6, 2.5)]


class MaskClient(Nav2Client):
    def __init__(self, supported=True):
        self.supported = supported
        self.masks, self.goals, self.cancellations = [], [], 0

    def lookup_map_to_base(self):
        return 0., 0., 0.

    def latest_map(self):
        return None

    def latest_cmd(self, now):
        return .2, 0.

    def send_goal(self, x, y, yaw):
        self.goals.append((x, y, yaw))

    def cancel(self):
        self.cancellations += 1

    def goal_state(self):
        return "active"

    def set_keepout_mask(self, mask):
        self.masks.append(mask)
        return self.supported


@pytest.mark.parametrize("yaw", [0., math.pi/2, .37])
def test_nav2_mask_is_reprojected_to_map_axes_instead_of_ignored_rotated_origin(yaw):
    cfg = Config()
    client = MaskClient()
    stack = Nav2Stack(cfg, client)
    stack.anchor = Anchor(1.1, -.4, yaw)
    policy = OwnBasePolicy(cfg)
    stack.set_keepout_layer(policy.planning_mask(stack.grid, Pose2D(3, 2.5, 0), "BLUE", 225.))
    message = client.masks[-1]
    for name, blocked in (("B-1", True), ("B-2", True), ("B-3", True), ("R-2", False)):
        x, y, _ = stack.anchor.to_map(*cfg.arena.base_center(name))
        ix, iy = math.floor((x-message.origin_x)/message.resolution), math.floor((y-message.origin_y)/message.resolution)
        assert bool(message.data[iy, ix]) is blocked
    stack.set_keepout_layer(None)
    assert not client.masks[-1].data.any(), "A new match can clear the complete policy mask"


def test_unsupported_nav2_policy_transport_uses_constrained_local_planner():
    cfg = Config()
    client = MaskClient(supported=False)
    stack = Nav2Stack(cfg, client)
    stack.anchor, stack.pose = Anchor(0., 0., 0.), Pose2D(.95, 1.7, math.pi/2)
    policy = OwnBasePolicy(cfg)
    stack.set_keepout_layer(policy.planning_mask(stack.grid, stack.pose, "BLUE", 225.))
    stack.set_goal((.95, 3.3))
    out = stack.step(None, 225., .05)
    assert out.status == "moving" and not client.goals
    assert stack._fb.path and max(p[0] for p in stack._fb.path) > 1.02


def test_fake_nav2_receives_and_clears_same_policy_without_perception_contamination():
    cfg = Config()
    client = object.__new__(FakeNav2Client)
    client.T = Pose2D(.8, 2.5, math.pi/2)
    client._nav = Navigator(cfg)
    stack = Nav2Stack(cfg, client)
    stack.anchor = Anchor(client.T.x, client.T.y, client.T.th)
    policy = OwnBasePolicy(cfg)
    mask = policy.planning_mask(stack.grid, Pose2D(3, 2.5, 0), "BLUE", 225.)
    stack.grid.set_keepout(mask)
    assert client.set_keepout_mask(stack.keepout_map())
    cell = client._nav.grid.to_cell(.4, 2.5)
    assert client._nav.grid.keepout[cell] and not client._nav.grid.occupied()[cell]
    stack.grid.set_keepout(None)
    assert client.set_keepout_mask(stack.keepout_map())
    assert client._nav.grid.keepout is None


class EmptyServer(ServerLink):
    def poll(self):
        return []

    def submit(self, kind, payload):
        return True


@pytest.mark.spec("S9.7")
@pytest.mark.parametrize("backend", ["builtin", "nav2"])
def test_agent_final_output_guard_overrides_acceleration_ramp_for_both_backends(backend):
    cfg, grid, _, pose = setup_policy(pose=Pose2D(.96, 2.5, math.pi))
    cfg.nav.backend, cfg.team, cfg.slot = backend, "BLUE", 2

    class OutputRobot(NoSensorRobot):
        def set_pan_tilt(self, pan, tilt):
            pass

    clock = FakeClock(225.)
    robot = OutputRobot(clock)
    stack = SimpleNamespace(pose=pose, grid=grid, initialized=True,
                            set_keepout_layer=grid.set_keepout, clear_goal=lambda: None)
    agent = Agent(robot, EmptyServer(), cfg, stack=stack)
    agent.t_start = 0.
    agent._update_field_policy(clock())
    agent._v_cmd = .3  # A zero requested velocity alone still leaves a ramp tail.
    agent._apply(0., 0., 0., 0., .05)
    assert robot.velocities[-1][1:] == (0., 0.)
    assert agent._v_cmd == 0. and agent.stats["own_base_blocked_commands"] == 1


@pytest.mark.parametrize("filename", ["nav2_params.yaml", "nav2_params.calibrated.yaml"])
def test_both_nav2_costmaps_load_dedicated_keepout_filter(filename):
    doc = yaml.safe_load((ROOT / "ros2/config" / filename).read_text(encoding="utf-8"))
    cfg = Config()
    for name in ("local_costmap", "global_costmap"):
        params = doc[name][name]["ros__parameters"]
        assert "own_base_keepout" in params["filters"]
        assert params["own_base_keepout"]["plugin"] == "nav2_costmap_2d::KeepoutFilter"
        assert params["own_base_keepout"]["filter_info_topic"] == cfg.nav2.keepout_info_topic
