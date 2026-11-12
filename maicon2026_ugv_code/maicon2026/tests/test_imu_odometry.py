"""Relative-only IMU yaw fusion, dropout continuity and real HAL callback contract."""
import math
import sys
from types import SimpleNamespace

import pytest

from maicon26.config import Config
from maicon26.hal.imu_odometry import RelativeImuOdometry,wrap
from maicon26.hal.ros_mil import RosMilTransport
from maicon26.types import Pose2D


def test_pure_wheel_spin_does_not_rotate_a_stationary_imu():
    f=RelativeImuOdometry()
    for i in range(31):
        t=1.+i*.05
        f.add_imu(t,2.3)
        p=f.update_wheel(t,Pose2D(2.,3.,.4+i*.1))
    assert (p.x,p.y,p.th)==pytest.approx((2.,3.,.4))
    assert f.stats["fused_steps"]==30


def test_actual_imu_90_degrees_replaces_wheel_180_without_absolute_heading():
    f=RelativeImuOdometry()
    for i in range(19):
        t=1.+i*.05
        f.add_imu(t,1.1+i*math.pi/36)
        p=f.update_wheel(t,Pose2D(0.,0.,i*math.pi/18))
    assert p.th==pytest.approx(math.pi/2)
    assert p.th!=pytest.approx(1.1+math.pi/2)


def test_imu_wrap_at_pi_uses_small_measured_delta():
    f=RelativeImuOdometry()
    f.add_imu(1.,math.radians(179));f.update_wheel(1.,Pose2D())
    f.add_imu(1.1,math.radians(-179))
    p=f.update_wheel(1.1,Pose2D(0.,0.,.3))
    assert p.th==pytest.approx(math.radians(2))


def test_dropout_reacquire_preserves_heading_continuity_and_uses_new_deltas():
    f=RelativeImuOdometry()
    f.add_imu(1.,1.7);f.update_wheel(1.,Pose2D())
    p=f.update_wheel(1.4,Pose2D(0.,0.,1.))
    assert p.th==pytest.approx(1.) and f.stats["mode"]=="wheel_fallback"
    f.add_imu(1.5,-2.8)
    p=f.update_wheel(1.5,Pose2D(0.,0.,1.1))
    assert p.th==pytest.approx(1.1) and f.stats["mode"]=="wheel_reanchor"
    f.add_imu(1.6,-2.7)
    p=f.update_wheel(1.6,Pose2D(0.,0.,1.4))
    assert p.th==pytest.approx(1.2)


def test_invalid_orientation_and_out_of_order_data_cannot_jump_heading():
    f=RelativeImuOdometry()
    f.add_imu(1.,.2);f.update_wheel(1.,Pose2D())
    assert not f.add_imu(.9,2.5)
    assert f.update_wheel(.9,Pose2D(99.,99.,2.)).th==0.
    f.add_imu(1.1,0.,valid=False,reason="orientation covariance unavailable")
    p=f.update_wheel(1.1,Pose2D(0.,0.,.2))
    assert p.th==pytest.approx(.2) and f.stats["mode"]=="wheel_fallback"
    f.add_imu(1.2,2.9)
    assert f.update_wheel(1.2,Pose2D(0.,0.,.3)).th==pytest.approx(.3)
    assert f.stats["imu_rejected"]==2 and f.stats["wheel_rejected"]==1


def test_translation_rotates_wheel_body_increment_in_fused_heading_frame():
    f=RelativeImuOdometry()
    f.add_imu(1.,-.7);f.update_wheel(1.,Pose2D(4.,7.,0.))
    for i in range(1,19):
        t=1.+i*.05
        f.add_imu(t,-.7+i*math.pi/36)
        f.update_wheel(t,Pose2D(4.,7.,i*math.pi/18))
    # Wheel odometry says west after its erroneous 180 degree turn; measured
    # yaw says north. Body-forward translation must follow the fused north.
    f.add_imu(2.,-.7+math.pi/2)
    p=f.update_wheel(2.,Pose2D(3.,7.,math.pi))
    assert (p.x,p.y,p.th)==pytest.approx((4.,8.,math.pi/2))


def test_timestamp_interpolation_and_association_budget():
    f=RelativeImuOdometry()
    f.add_imu(1.,0.);f.add_imu(1.1,1.)
    f.update_wheel(1.,Pose2D())
    p=f.update_wheel(1.05,Pose2D(0.,0.,2.))
    assert p.th==pytest.approx(.5)
    p=f.update_wheel(1.4,Pose2D(0.,0.,2.2))
    assert p.th==pytest.approx(.7)
    assert f.stats["imu_age_s"] is None


def quaternion(yaw=0.,roll=0.):
    return SimpleNamespace(x=math.sin(roll/2),y=0.,z=math.sin(yaw/2),w=math.cos(roll/2)*math.cos(yaw/2))


def header(stamp,frame):
    return SimpleNamespace(stamp=SimpleNamespace(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9)),frame_id=frame)


def fake_transport():
    t=RosMilTransport.__new__(RosMilTransport)
    t.cfg=Config()
    poses=[]
    t.robot=SimpleNamespace(stats={},on_imu=lambda *args:None,on_odom=lambda *args:poses.append(args))
    t.imu_odometry=RelativeImuOdometry()
    return t,poses


def imu_message(stamp,yaw,covariance=None,frame="base_link"):
    return SimpleNamespace(header=header(stamp,frame),orientation=quaternion(yaw),
        orientation_covariance=[0.]*9 if covariance is None else covariance,
        linear_acceleration=SimpleNamespace(x=0.,y=0.,z=9.80665))


def odom_message(stamp,yaw,frame="base_link"):
    return SimpleNamespace(header=header(stamp,"odom"),child_frame_id=frame,
        pose=SimpleNamespace(pose=SimpleNamespace(orientation=quaternion(yaw),position=SimpleNamespace(x=0.,y=0.))))


def test_hal_uses_only_valid_measured_imu_with_covariance_fallback():
    t,poses=fake_transport();base=t.cfg.nav2.base_frame
    t._on_imu(imu_message(1.,2.,frame=base));t._on_odom(odom_message(1.,0.,base))
    t._on_imu(imu_message(1.1,2.,frame=base));t._on_odom(odom_message(1.1,.3,base))
    assert poses[-1][2]==pytest.approx(0.)
    t._on_imu(imu_message(1.2,2.,[-1.]+[0.]*8,base));t._on_odom(odom_message(1.2,.5,base))
    assert poses[-1][2]==pytest.approx(.2)
    assert "covariance" in t.robot.stats["imu_heading_error"]


def test_unvalidated_tilted_sensor_mount_does_not_enable_fusion(monkeypatch):
    t,poses=fake_transport();base=t.cfg.nav2.base_frame
    monkeypatch.setitem(sys.modules,"rclpy.time",SimpleNamespace(Time=lambda:0))
    t._tf=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(transform=SimpleNamespace(rotation=quaternion(roll=.1))))
    t._on_imu(imu_message(1.,0.,frame="tilted_imu"))
    t._on_odom(odom_message(1.,0.,base));t._on_odom(odom_message(1.1,.4,base))
    assert poses[-1][2]==pytest.approx(.4)
    assert t.imu_odometry.stats["fused_steps"]==0
    assert "frame validation" in t.robot.stats["imu_heading_error"]


def test_raw_and_fused_audit_dump_is_bounded_and_contains_no_truth(tmp_path):
    f=RelativeImuOdometry(history_size=3)
    for i in range(3):
        f.add_imu(1.+i*.1,.5)
        f.update_wheel(1.+i*.1,Pose2D(0.,0.,i*.1))
    path=tmp_path/"fusion.jsonl";assert f.dump(path)==3
    text=path.read_text()
    assert '"raw"' in text and '"fused"' in text and "truth" not in text
    assert Config().hal.imu_heading_fusion is False

def test_invalid_missing_or_reordered_imu_timestamps_fall_back_until_fresh_sample():
    for invalid_stamp in (0.,float('nan'),.9):
        f=RelativeImuOdometry()
        f.add_imu(1.,2.);f.update_wheel(1.,Pose2D())
        assert not f.add_imu(invalid_stamp,2.2)
        p=f.update_wheel(1.05,Pose2D(0.,0.,.1))
        assert p.th==pytest.approx(.1) and f.stats['mode']=='wheel_fallback'
        f.add_imu(1.1,-1.);p=f.update_wheel(1.1,Pose2D(0.,0.,.2))
        assert p.th==pytest.approx(.2) and f.stats['mode']=='wheel_reanchor'


def test_missing_wheel_stamp_falls_back_without_resetting_order_highwater():
    f=RelativeImuOdometry()
    f.add_imu(1.,2.);f.update_wheel(1.,Pose2D())
    assert f.update_wheel(None,Pose2D(0.,0.,.2)).th==pytest.approx(.2)
    assert f.update_wheel(.9,Pose2D(99.,99.,2.)).th==pytest.approx(.2)
    with pytest.raises(ValueError):
        RelativeImuOdometry(.151)

def test_rigid_mount_validation_uses_latest_tf_and_caches_only_a_valid_frame(monkeypatch):
    t,poses=fake_transport();base=t.cfg.nav2.base_frame
    monkeypatch.setitem(sys.modules,'rclpy.time',SimpleNamespace(Time=lambda:0))
    lookups=[]
    def lookup(parent,child,stamp):
        assert stamp==0, 'Fixed IMU mounting does not need an extrapolated measurement-time TF'
        lookups.append((parent,child))
        return SimpleNamespace(transform=SimpleNamespace(rotation=quaternion()))
    t._tf=SimpleNamespace(lookup_transform=lookup)
    for i in range(4):
        stamp=1.+i*.05
        t._on_imu(imu_message(stamp,1.5,frame='base_imu_link'))
        t._on_odom(odom_message(stamp,i*.1,base))
    assert poses[-1][2]==pytest.approx(0.)
    assert t.imu_odometry.stats['fused_steps']==3
    assert lookups==[(base,'base_imu_link')]
    assert t.robot.stats['imu_heading_mount']['verified_rigid_mount']
    t._on_imu(imu_message(1.3,1.5,frame='different_imu_frame'))
    assert len(lookups)==2, 'Changing frame must be revalidated'
