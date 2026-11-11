"""Three measured sectors must merge without inventing observations or time."""
from dataclasses import replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from maicon26.config import Config
from maicon26.hal.gazebo_model import LIDAR_XYZ
from maicon26.hal.lidar_sectors import (LidarSectorMerger, SectorScan, SECTOR_HALF_FOV,
                                      SECTOR_YAWS, mask_rover_self_returns, sector_frame, sector_samples, sector_topic)
from scripts.generate_gazebo_world import robot_model

ROOT = Path(__file__).resolve().parents[1]


def packet(index, samples=360, prefix="", stamp=1., value=np.nan):
    n = sector_samples(samples)
    return SectorScan(stamp, sector_frame(prefix, index), -SECTOR_HALF_FOV,
                      SECTOR_HALF_FOV, 2 * SECTOR_HALF_FOV / (n - 1), .05, 8.,
                      np.full(n, value, dtype=np.float32))


def send(merger, packets, wall=10.):
    for index, p in enumerate(packets):
        merger.push(index, p, wall)


def test_rotation_wraparound_and_seams_keep_only_nearest_measured_return():
    merger = LidarSectorMerger()
    packets = [packet(i) for i in range(3)]
    for i, p in enumerate(packets):
        p.ranges[60] = i + 1.
    packets[0].ranges[-1] = 2.
    packets[1].ranges[0] = 1.2  # Same +60 degree direction, closer real return.
    packets[1].ranges[-1] = 1.1
    packets[2].ranges[0] = .8  # +pi and -pi are the same output bin.
    send(merger, packets)
    output = merger.merge(1.01, 10.01)
    assert output.frame_id == "gazebo_lidar" and output.stamp == 1.
    assert output.ranges[180] == 1.
    assert output.ranges[300] == 2.
    assert output.ranges[60] == 3.
    assert output.ranges[240] == pytest.approx(1.2)
    assert output.ranges[0] == pytest.approx(.8)
    assert np.isfinite(output.ranges).sum() == 5
    assert output.angle_max == pytest.approx(math.pi - 2 * math.pi / 360)


@pytest.mark.parametrize("samples", [360, 640])
def test_all_azimuth_bins_have_actual_measurement_support(samples):
    merger = LidarSectorMerger(samples, prefix="R1")
    send(merger, [packet(i, samples, "R1", value=2.) for i in range(3)])
    output = merger.merge(1., 10.)
    assert len(output.ranges) == samples
    assert np.all(output.ranges == 2.)
    assert output.frame_id == "R1/gazebo_lidar"
    assert output.angle_increment == pytest.approx(2 * math.pi / samples)


def test_invalid_returns_and_unmeasured_directions_remain_nan():
    merger = LidarSectorMerger()
    packets = [packet(i) for i in range(3)]
    packets[0].ranges[50:58] = [np.nan, np.inf, -1., 0., .01, 8.1, .5, 8.]
    send(merger, packets)
    output = merger.merge(1., 10.)
    assert np.isfinite(output.ranges).sum() == 2
    assert output.ranges[176] == .5 and output.ranges[177] == 8.
    assert np.isnan(output.ranges[170:176]).all()
    assert np.isnan(output.ranges[0])


@pytest.mark.parametrize("mode", ["missing", "sensor_stale", "receive_stale", "future", "skew"])
def test_missing_stale_or_unsynchronized_sector_never_produces_scan(mode):
    merger = LidarSectorMerger()
    packets = [packet(i, stamp=1.1 if mode == "skew" and i == 2 else 1.) for i in range(3)]
    send(merger, packets[:2] if mode == "missing" else packets)
    sim = 1.5 if mode == "sensor_stale" else .8 if mode == "future" else 1.1
    wall = 10.6 if mode == "receive_stale" else 10.1
    assert merger.merge(sim, wall) is None


def test_no_sector_replay_and_clock_reset_requires_three_new_measurements():
    merger = LidarSectorMerger()
    send(merger, [packet(i) for i in range(3)])
    assert merger.merge(1., 10.) is not None
    assert merger.merge(1., 10.) is None
    merger.push(0, packet(0, stamp=1.1), 10.1)
    assert merger.merge(1.1, 10.1) is None
    assert merger.merge(.2, 10.2) is None
    assert not merger.latest and merger.stats["clock_resets"] == 1
    send(merger, [packet(i, stamp=.2) for i in range(3)], 10.2)
    assert merger.merge(.2, 10.2).stamp == .2


@pytest.mark.parametrize("bad", ["frame", "angle", "count", "increment", "range"])
def test_sector_frame_and_optical_geometry_are_validated(bad):
    merger = LidarSectorMerger(prefix="B1")
    p = packet(1, prefix="B1")
    if bad == "frame": p = replace(p, frame_id=sector_frame("B2", 1))
    elif bad == "angle": p = replace(p, angle_min=-1.396263)
    elif bad == "count": p = replace(p, ranges=p.ranges[:-1])
    elif bad == "increment": p = replace(p, angle_increment=.01)
    elif bad == "range": p = replace(p, range_min=8.)
    with pytest.raises(ValueError):
        merger.push(1, p, 10.)
    assert merger.stats["rejected"] == 1 and 1 not in merger.latest


def test_disagreeing_range_metadata_is_not_merged():
    merger = LidarSectorMerger()
    packets = [packet(i) for i in range(3)]
    packets[1] = replace(packets[1], range_max=9.)
    send(merger, packets)
    assert merger.merge(1., 10.) is None and merger.status == "range_limits_disagree"


def test_observed_all_minimum_renderer_failure_is_rejected_but_real_near_return_survives():
    merger = LidarSectorMerger()
    with pytest.raises(ValueError, match="collapsed to range_min"):
        merger.push(0, packet(0, value=.05), 10.)
    assert merger.merge(1., 10.) is None
    assert merger.stats["rejected"] == 1
    packets = [packet(i, value=2.) for i in range(3)]
    packets[0].ranges[60] = .05
    send(merger, packets)
    assert merger.merge(1., 10.).ranges[180] == pytest.approx(.05)


@pytest.mark.parametrize("samples", [360, 640])
def test_sdf_sectors_share_origin_and_match_the_merger_geometry_for_every_robot(samples):
    cfg = Config()
    for prefix in ("", "/B1", "/B3", "/R1", "/R2", "/R3"):
        world = ET.Element("world", name="test")
        robot = robot_model(world, cfg, 1., 2., 0., topic_prefix=prefix,
                            lidar_samples=samples, lidar_mode="sectors360")
        sensors = robot.findall(".//sensor[@type='gpu_lidar']")
        assert len(sensors) == 3
        for index, sensor in enumerate(sensors):
            pose = tuple(map(float, sensor.findtext("pose").split()))
            assert pose[:3] == LIDAR_XYZ
            assert pose[3:5] == (0., 0.) and pose[5] == SECTOR_YAWS[index]
            assert sensor.findtext("topic") == sector_topic(prefix, index)
            assert sensor.findtext("ignition_frame_id") == sector_frame(prefix, index)
            assert int(sensor.findtext("ray/scan/horizontal/samples")) == sector_samples(samples)
            assert float(sensor.findtext("ray/scan/horizontal/min_angle")) == -SECTOR_HALF_FOV
            assert float(sensor.findtext("ray/scan/horizontal/max_angle")) == SECTOR_HALF_FOV


def test_default_160_sensor_is_unchanged_and_bridges_supply_all_optional_inputs():
    robot = robot_model(ET.Element("world"), Config(), 1., 2., 0.)
    sensors = robot.findall(".//sensor[@type='gpu_lidar']")
    assert len(sensors) == 1 and sensors[0].get("name") == "laser"
    assert sensors[0].findtext("topic") == "/scan"
    assert float(sensors[0].findtext("ray/scan/horizontal/min_angle")) == -1.396263
    for filename in ("bridge.yaml", "bridge_agent.yaml", "bridge_match.yaml"):
        rows = yaml.safe_load((ROOT / "ros2/gazebo" / filename).read_text())
        topics = {r["ros_topic_name"]: r for r in rows}
        prefixes = ("", "B1", "B3", "R1", "R2", "R3") if filename == "bridge_match.yaml" else ("",)
        for prefix in prefixes:
            for index in range(3):
                row = topics[sector_topic(prefix, index)]
                assert row["gz_topic_name"] == row["ros_topic_name"]
                assert row["direction"] == "GZ_TO_ROS"


def full_scan(ranges, frame="gazebo_lidar"):
    step = 2 * math.pi / len(ranges)
    return SectorScan(1., frame, -math.pi, math.pi-step, step, .05, 8., ranges)


def test_self_returns_use_mount_offset_and_preserve_unknown_and_near_external_obstacles():
    ranges = np.full(360, 2., dtype=np.float32)
    ranges[90] = ranges[270] = .058  # Side receiver domes, inside 250x230 mm body.
    ranges[0] = .15                # Rear endpoint x=-.110185: inside despite longer range.
    ranges[180] = .09              # Front endpoint x=.129815: OUTSIDE despite shorter range.
    ranges[5] = np.nan
    filtered, count = mask_rover_self_returns(full_scan(ranges, "R1/gazebo_lidar"))
    assert count == 3
    assert np.isnan(filtered.ranges[[0, 90, 270]]).all()
    assert filtered.ranges[180] == pytest.approx(.09)
    assert np.isnan(filtered.ranges[5]) and filtered.ranges[120] == 2.
    assert ranges[90] == pytest.approx(.058), "Input observations must not be overwritten"


def test_self_mask_preserves_inset_boundary_and_actual_body_surface_measurements():
    ranges = np.full(360, 2., dtype=np.float32)
    ranges[180] = .12 - LIDAR_XYZ[0]   # Exactly on the inset front boundary.
    ranges[270] = .11                 # Exactly on the inset left boundary.
    ranges[181] = (.125 - LIDAR_XYZ[0]) / math.cos(math.pi / 180)
    ranges[269] = .115 / math.sin(89 * math.pi / 180)
    filtered, count = mask_rover_self_returns(full_scan(ranges))
    assert count == 0
    assert np.array_equal(filtered.ranges, ranges)
