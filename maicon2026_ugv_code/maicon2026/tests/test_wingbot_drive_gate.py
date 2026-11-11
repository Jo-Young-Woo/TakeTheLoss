"""The validation tool cannot pass reversed travel or mismatched sample clocks."""
from collections import deque
import importlib.util
import io
import math
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

PATH = Path(__file__).resolve().parents[1]/"ros2/wingbot_description/scripts/drive_gate.py"
SPEC = importlib.util.spec_from_file_location("wingbot_drive_gate", PATH)
gate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(gate)


def rows(positions, duration=5., heading=0., twist=.3):
    return [{"odom_stamp": index*duration/(len(positions)-1),
             "truth": [position*math.cos(heading),position*math.sin(heading),heading],
             "odom": [position*math.cos(heading),position*math.sin(heading),heading],
             "odom_twist": [twist,0.]}
            for index,position in enumerate(positions)]


@pytest.mark.parametrize("heading", [0., .7, 2.9])
def test_forward_and_reverse_require_commanded_direction_not_distance_norm(heading):
    forward = gate.summarize(rows([i*.075 for i in range(21)],heading=heading),[.3,0],5.)
    backward = gate.summarize(rows([-i*.075 for i in range(21)],heading=heading,twist=-.3),[.3,0],5.)
    assert gate.assess_segment(forward)
    assert backward["truth_distance_m"] == pytest.approx(1.5)
    assert backward["truth_forward_m"] == pytest.approx(-1.5)
    assert not gate.assess_segment(backward)
    reverse_command = gate.summarize(rows([-i*.075 for i in range(21)],heading=heading,twist=-.3),[-.3,0],5.)
    assert gate.assess_segment(reverse_command)


def test_brief_overspeed_cannot_hide_behind_correct_average_displacement():
    # Same1.5m endpoint and same5s duration, but first.25s travelled at1.2m/s.
    positions = [0.,.3]+[.3+1.2*index/19 for index in range(1,20)]
    result = gate.summarize(rows(positions),[.3,0],5.)
    assert result["truth_error_ratio"] == pytest.approx(0.)
    assert result["truth_sampled_peak_linear_mps"] == pytest.approx(1.2)
    assert not gate.assess_segment(result)


def test_truth_is_interpolated_to_odom_stamp_including_yaw_wrap():
    history = deque([(1.,(1.,2.,math.radians(179)),.01,(0,0,0,1)),
                     (1.04,(1.04,2.08,math.radians(-179)),.03,(0,0,0,1))])
    pose,z,_,bracket = gate.align_truth(history,1.02)
    assert pose == pytest.approx((1.02,2.04,math.pi))
    assert z == pytest.approx(.02) and bracket == (1.,1.04)
    assert gate.align_truth(history,.99) is None
    assert gate.align_truth(history,1.05) is None
    assert gate.align_truth(history,1.02,max_bracket_s=.02) is None


def test_exact_truth_timestamp_does_not_require_an_unrelated_future_frame():
    history = deque([(1.,(2.,3.,.4),.01,(0,0,0,1))])
    aligned = gate.align_truth(history,1.)
    assert aligned[0] == [2.,3.,.4] and aligned[3] == (1.,1.)


def test_duplicate_measurement_timestamps_are_rejected():
    samples = rows([i*.075 for i in range(21)])
    samples[2]["odom_stamp"] = samples[1]["odom_stamp"]
    with pytest.raises(ValueError,match="strictly increase"):
        gate.summarize(samples,[.3,0],5.)


def test_cleanup_reaches_kill_and_closes_logs_when_child_ignores_int_and_term(monkeypatch):
    class Child:
        pid=101
        exited=False
        def poll(self):
            return 0 if self.exited else None
        def wait(self,timeout):
            if not self.exited:
                raise subprocess.TimeoutExpired("test child",timeout)
            return 0
    child = Child()
    signals=[]
    def kill(pid,sig):
        signals.append((pid,sig))
        if sig == signal.SIGKILL:
            child.exited=True
    monkeypatch.setattr(gate.os,"killpg",kill)
    log=io.StringIO()
    gate.stop_children([child],[log])
    assert signals == [(101,signal.SIGINT),(101,signal.SIGTERM),(101,signal.SIGKILL)]
    assert child.exited and log.closed


def test_startup_exception_still_runs_child_cleanup(monkeypatch):
    sentinel=object()
    cleaned=[]
    def fail(children,logs):
        children.append(sentinel)
        raise ImportError("ROS import failed after launch")
    monkeypatch.setattr(gate,"run",fail)
    monkeypatch.setattr(gate,"stop_children",lambda children,logs: cleaned.extend(children))
    with pytest.raises(ImportError):
        gate.main()
    assert cleaned == [sentinel]


def test_fortress_model_truth_publisher_preserves_stamps_and_root_model(monkeypatch,tmp_path):
    urdf=tmp_path/"input.urdf"
    urdf.write_text('<robot name="wingbot"><gazebo><plugin name="DiffDrive">'
                    '<topic>/test/cmd</topic><odom_topic>/test/odom</odom_topic>'
                    '</plugin></gazebo></robot>',encoding="utf-8")
    monkeypatch.setattr(gate.subprocess,"run",lambda *a,**kw: SimpleNamespace(
        stdout='<sdf version="1.9"><model name="wingbot"><link name="base_link"/></model></sdf>'))
    destination=tmp_path/"world.sdf"
    gate.make_world(urdf,destination)
    model=ET.parse(destination).find("world/model[@name='wingbot']")
    publisher=model.find("plugin[@name='ignition::gazebo::systems::PosePublisher']")
    # Gazebo Fortress PosePublisher.cc377 requires both flags even for the root.
    assert publisher.findtext("publish_model_pose") == "true"
    assert publisher.findtext("publish_nested_model_pose") == "true"
    assert publisher.findtext("publish_link_pose") == "false"
    assert publisher.findtext("use_pose_vector_msg") == "false", "TFMessage bridge loses Pose_V outer timestamps"
    assert not model.findall("model"), "No nested poses may mix into this single-model truth channel"
