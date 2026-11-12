"""The controller's raster frame is explicitly derived from the measured world."""
from pathlib import Path

import pytest
import yaml

from maicon26.arena import ArenaGeometry


def measured():
    root = Path(__file__).resolve().parents[1]
    return ArenaGeometry.from_spec(yaml.safe_load((root/"ros2/maicon_arena_gz/config/arena_spec.yaml").read_text()))


def test_measured_world_and_navigation_frames_roundtrip_and_share_sectors():
    geo = measured()
    assert (geo.W, geo.H) == (6.12, 5.)
    assert geo.world_to_nav(0., 0.) == (3.06, 2.5)
    assert geo.nav_to_world(*geo.world_to_nav(-2.66, 1.6)) == pytest.approx((-2.66, 1.6))
    assert geo.base_center("B-2") == pytest.approx((.40, 2.5))
    assert geo.measured_starts["B-2"] == pytest.approx((.40, 2.5, 0.))
    assert geo.measured_starts["R-1"] == pytest.approx((5.72, 4.1, 3.14159265))
    for name, point in (("B-S1", (-1.2, 1.6)), ("R-S3", (1.5, -1.3)), ("COMMON", (0., 1.2))):
        assert geo.sector_of(*geo.world_to_nav(*point)) == name
    assert geo.sector_of(*geo.world_to_nav(0., 2.2)) is None  # CM stops at the printed border.
    assert geo.sector_of(*geo.world_to_nav(-2.66, 0.)) is None  # Base strip never earns M3 points.


def test_original_approximate_arena_remains_explicit_legacy_fixture():
    geo = ArenaGeometry()
    assert (geo.W, geo.H) == (6.12, 5.)
    assert not geo.measured_sectors and geo.world_to_nav(1., 2.) == (1., 2.)
