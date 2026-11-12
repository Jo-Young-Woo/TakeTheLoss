"""Metric coordinates must agree across printed markings, runtime and UAV."""
import argparse
import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

from maicon26.arena import ArenaGeometry
from maicon26.perception.simulation_uav import load_simulation_uav_detector

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2/maicon_arena_gz"
sys.path.insert(0, str(PACKAGE / "scripts"))
from arena_common import fixed_geometry, load_spec
from make_atlas import floor_obj


def test_resized_inner_walls_match_navigation_boundaries():
    spec = load_spec(PACKAGE / "config/arena_spec.yaml")
    geo = ArenaGeometry.from_spec(spec)
    walls = {g["name"]: g for g in fixed_geometry(spec) if g["kind"] == "wall"}
    assert walls["wall_east"]["center"][0] - walls["wall_east"]["size"][0]/2 == pytest.approx(3.06)
    assert walls["wall_north"]["center"][1] - walls["wall_north"]["size"][1]/2 == pytest.approx(2.50)
    assert geo.world_to_nav(-3.06, -2.5) == (0., 0.)
    assert geo.world_to_nav(3.06, 2.5) == (6.12, 5.)
    assert geo.measured_starts["B-2"] == pytest.approx((.4, 2.5, 0.))
    assert spec["cubes"]["size"] == .4


def test_floor_mesh_keeps_original_metric_print_inside_expanded_surface():
    mesh = floor_obj((6./6.12, 4.8/5.))
    vertices = [list(map(float, s.split()[1:])) for s in mesh.splitlines() if s.startswith("v ")]
    xs = sorted(set(round(v[0]*6.12, 7) for v in vertices))
    ys = sorted(set(round(v[1]*5., 7) for v in vertices))
    assert xs == [-3.06, -3., 3., 3.06]
    assert ys == [-2.5, -2.4, 2.4, 2.5]
    # Changing arena size must not stretch the original 20 cm print grid.
    assert xs[2] - xs[1] == 6.
    assert ys[2] - ys[1] == 4.8


def test_uav_calibration_agrees_with_new_navigation_frame():
    arena = ArenaGeometry.from_spec(yaml.safe_load((PACKAGE/"config/arena_spec.yaml").read_text()))
    detector = load_simulation_uav_detector(PACKAGE/"config/uav_grid_calibration.json", arena)
    assert detector.calibration.camera_center == pytest.approx((3.06, 2.5, 2.5))
    assert detector.calibration.validation_report()["status"] == "within_tolerance"
    for point in detector.calibration.measurements:
        px, py = point["reference_pixel"]
        assert point["arena"] == pytest.approx((.06+(px+.5)/2400*6., .1+4.8-(py+.5)/1920*4.8))


@pytest.mark.parametrize("environment,cli", [(None,None), ("field.yaml",None), ("field.yaml","override.yaml")])
def test_team_entry_passes_arena_spec_with_cli_precedence(monkeypatch, environment, cli):
    spec = importlib.util.spec_from_file_location("resize_team_entry", ROOT/"team/team_node.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for key in ("TEAM_LAUNCH_NAV", "MAICON_OAK", "MAICON_ARENA_SPEC"):
        monkeypatch.delenv(key, raising=False)
    if environment:
        monkeypatch.setenv("MAICON_ARENA_SPEC", environment)
    argv = ["team_node.py", "--preflight-only"]
    if cli:
        argv += ["--arena-spec", cli]
    monkeypatch.setattr(sys, "argv", argv)
    captured = []
    monkeypatch.setattr(module.runpy, "run_path", lambda *a, **k: captured.append(list(sys.argv[1:])))
    assert module.main() == 0
    parser = argparse.ArgumentParser()
    parser.add_argument("--arena-spec")
    args, _ = parser.parse_known_args(captured[0])
    assert args.arena_spec == (cli or environment or str(PACKAGE/"config/arena_spec.yaml"))
