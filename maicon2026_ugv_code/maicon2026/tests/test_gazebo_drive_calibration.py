import pytest
from scripts.generate_gazebo_world import generate


def test_kinematic_calibration_preserves_cad_wheel_positions_and_radius():
    baseline, _ = generate(2, "B2")
    corrected, _ = generate(2, "B2", effective_track=.20473)
    a = baseline.find("world/model[@name='wingbot_ugv']")
    b = corrected.find("world/model[@name='wingbot_ugv']")
    assert [x.findtext("pose") for x in a.findall("link")] == [x.findtext("pose") for x in b.findall("link")]
    drive = b.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    original = a.find("plugin[@name='ignition::gazebo::systems::DiffDrive']")
    assert float(drive.findtext("wheel_separation")) == pytest.approx(.20473)
    assert drive.findtext("wheel_radius") == original.findtext("wheel_radius")


@pytest.mark.parametrize("track", [0., float("nan"), 1.])
def test_invalid_kinematic_calibration_is_rejected(track):
    with pytest.raises(ValueError, match="Effective wheel track"):
        generate(2, "B2", effective_track=track)
