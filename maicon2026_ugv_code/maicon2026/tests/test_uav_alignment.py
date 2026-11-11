from types import SimpleNamespace

import pytest

from maicon26.link.base import UavBox
from maicon26.perception.uav_alignment import estimate_uav_translation, transform_uav_boxes


def lidar(box, dx=0.0, dy=0.0, *, kind="full"):
    return SimpleNamespace(cx=box.cx + dx, cy=box.cy + dy, sx=box.sx, sy=box.sy, kind=kind)


def scene():
    return [UavBox(1.0, 1.0, .4, .8), UavBox(3.0, 2.5, .4, .4), UavBox(4.5, 4.0, .8, .4)]


def test_zero_offset_and_measured_translation_are_recovered():
    aerial = scene()
    zero = estimate_uav_translation(aerial, [lidar(box) for box in aerial])
    assert zero is not None and zero.matches == 3
    assert zero.dx == zero.dy == zero.rms == 0.0
    aligned = estimate_uav_translation(aerial, [lidar(box, -.09, .27 + .005 * i)
                                               for i, box in enumerate(aerial)])
    assert aligned is not None and aligned.matches == 3
    assert aligned.dx == pytest.approx(-.09)
    assert aligned.dy == pytest.approx(.275)
    assert aligned.rms < .01
    shifted = transform_uav_boxes(aerial, aligned)
    assert shifted[0].cy == pytest.approx(1.275)
    assert aerial[0].cy == 1.0  # preserve raw arena coordinates
    assert shifted[0].confidence == aerial[0].confidence


def test_two_consistent_well_separated_matches_can_align_but_one_cannot():
    aerial = scene()
    result = estimate_uav_translation(aerial, [lidar(box, .1, -.2) for box in aerial[:2]])
    assert result is not None and result.matches == 2
    assert estimate_uav_translation(aerial, [lidar(aerial[0], .1, -.2)]) is None
    assert estimate_uav_translation(aerial, [lidar(box, .1, -.2, kind="face") for box in aerial]) is None


def test_no_matches_large_offset_and_inconsistent_offsets_are_rejected():
    aerial = scene()
    assert estimate_uav_translation(aerial, []) is None
    assert estimate_uav_translation(aerial, [lidar(box, .6, .0) for box in aerial]) is None
    assert estimate_uav_translation(aerial, [lidar(aerial[0], -.25, .1),
                                             lidar(aerial[1], .25, .1)]) is None
    merged = [SimpleNamespace(cx=box.cx, cy=box.cy, sx=.8, sy=.8, kind="full") for box in aerial]
    assert estimate_uav_translation(aerial, merged) is None


def test_repeated_structure_ambiguity_is_rejected():
    aerial = [UavBox(x, 1.0, .4, .4) for x in (1.0, 3.0, 1.4, 3.4)]
    ground = [SimpleNamespace(cx=x, cy=1.1, sx=.4, sy=.4, kind="full") for x in (1.2, 3.2)]
    # Both (+.2,+.1) and (-.2,+.1) explain two well-separated boxes.
    assert estimate_uav_translation(aerial, ground) is None


def test_adjacent_or_duplicate_evidence_cannot_establish_alignment():
    aerial = [UavBox(1.0, 1.0, .4, .4), UavBox(1.4, 1.0, .4, .4)]
    assert estimate_uav_translation(aerial, [lidar(box, .1, .1) for box in aerial]) is None
    assert estimate_uav_translation(scene(), [lidar(scene()[0], .1, .1)] * 3) is None
