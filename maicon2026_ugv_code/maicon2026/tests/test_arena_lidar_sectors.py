"""Three real120degree sensors: synchronization, rotation, seams and invalid rays."""
import copy
import math
import xml.etree.ElementTree as ET

import pytest

from test_arena_robot_runtime import inputs, module


MERGER = module("lidar_sector_merger")


def frames(samples=450):
    _, assumptions, _ = inputs()
    settings = assumptions["lidar"]
    count = math.ceil(samples/3)+1
    result = [{"stamp": 10., "frame_id": f"laser_sector_{i}",
               "angle_min": -settings["sector_span"]/2, "angle_max": settings["sector_span"]/2,
               "angle_increment": settings["sector_span"]/(count-1),
               "range_min": settings["range"][0], "range_max": settings["range"][1],
               "ranges": [5.]*count} for i in range(3)]
    return result, settings


@pytest.mark.parametrize("samples", [360, 450, 640])
def test_complete_circle_and_shared_seam_keep_nearest_actual_measurement(samples):
    scans, settings = frames(samples)
    scans[0]["ranges"][-1] = 3.
    scans[1]["ranges"][0] = 1.2
    result = MERGER.merge_scans(scans, samples, 10.1, settings)
    assert len(result["ranges"]) == samples and all(math.isfinite(v) for v in result["ranges"])
    seam = int(math.floor(((settings["sector_span"]/2+math.pi) % (2*math.pi))/(2*math.pi/samples)+.5)) % samples
    assert result["ranges"][seam] == 1.2
    assert result["stamp"] == 10. and result["frame_id"] == "laser_frame"
    assert result["angle_max"] == pytest.approx(result["angle_min"]+(samples-1)*result["angle_increment"])


def test_measured_front_and_rotated_sector_centres_have_correct_angles():
    scans, settings = frames(450)
    for index, scan in enumerate(scans):
        scan["ranges"] = [math.nan]*len(scan["ranges"])
        scan["ranges"][75] = index+1.
    result = MERGER.merge_scans(scans, 450, 10., settings)
    assert result["ranges"][225] == 1.
    assert result["ranges"][375] == 2.
    assert result["ranges"][75] == 3.
    assert sum(math.isfinite(v) for v in result["ranges"]) == 3


def test_unknown_and_out_of_range_rays_stay_unknown_without_interpolation_or_self_mask():
    scans, settings = frames()
    for scan in scans:
        scan["ranges"] = [math.nan]*len(scan["ranges"])
    scans[0]["ranges"][:4] = [math.inf, .01, 13., .08]
    result = MERGER.merge_scans(scans, 450, 10., settings)
    assert [v for v in result["ranges"] if math.isfinite(v)] == [.08]


@pytest.mark.parametrize("change,now,reason", [
    ("skew", 10.1, "skew"), ("stale", 10.6, "stale_or_future"),
    ("future", 9.9, "stale_or_future"), ("frame", 10., "geometry"),
    ("increment", 10., "geometry"), ("count", 10., "geometry"),
    ("range_min", 10., "range_min_collapse")])
def test_unusable_sector_set_is_rejected_instead_of_publishing_fake_obstacles(change, now, reason):
    scans, settings = frames()
    if change == "skew":
        scans[1]["stamp"] += .03
    elif change == "frame":
        scans[1]["frame_id"] = "wrong_frame"
    elif change == "increment":
        scans[1]["angle_increment"] *= 2
    elif change == "count":
        scans[1]["ranges"].pop()
    elif change == "range_min":
        scans[1]["ranges"] = [settings["range"][0]]*len(scans[1]["ranges"])
    with pytest.raises(MERGER.ScanRejected, match=reason):
        MERGER.merge_scans(scans, 450, now, settings)


@pytest.mark.parametrize("profile,rays,output", [("standard", 151, 450), ("lite", 121, 360), ("high", 215, 640)])
def test_ogre_one_generates_three_colocated_sectors_only_on_ego(profile, rays, output):
    spec, assumptions, config = inputs(profile)
    original = copy.deepcopy(spec)
    world = ET.Element("world", name="maicon_arena")
    def add_system(parent, key, simulator):
        return ET.SubElement(parent, "plugin", name=key)
    report = module("robot_models").add_ugvs(world, spec, config, 6, "B-2", system_adder=add_system,
                                           render_engine="ogre", robot_model="legacy")
    scans = world.findall(".//sensor[@type='gpu_lidar']")
    assert len(scans) == report["sensors"]["gpu_lidar"] == 3
    assert report["lidar_output_samples"] == output and report["lidar_mode"] == "sectors360"
    positions = [list(map(float, scan.findtext("pose").split())) for scan in scans]
    assert all(p[:3] == positions[0][:3] for p in positions)
    assert [p[-1] for p in positions] == assumptions["lidar"]["sector_yaws"]
    assert all(int(scan.findtext("lidar/scan/horizontal/samples")) == rays for scan in scans)
    assert all(float(scan.findtext("lidar/scan/horizontal/max_angle")) < math.pi/2 for scan in scans)
    assert spec == original
