"""Exact scan lineage and honest wall-time intervals; no simulator needed."""
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from maicon26.config import Config
from maicon26.hal.ros_mil import RosMilTransport
from maicon26.hal.wingbot import WingbotRobot
from maicon26.mission.env import speckle_filter
from maicon26.mission.risk_worker import RiskFieldWorker
from maicon26.nav.grid import OccupancyGrid
from maicon26.nav.selfmask import SelfMask
from maicon26.types import Scan


def wait_version(worker, version):
    until=time.perf_counter()+2.
    while worker.latest()[0]!=version and time.perf_counter()<until:
        time.sleep(.001)
    assert worker.latest()[0]==version


def test_scan_processing_preserves_receipt_without_changing_capture_clock():
    cfg=Config(); cfg.lidar.min_range=.1
    scan=Scan(-math.pi,math.pi/4,np.array([.08,4.,4.,.2,4.,4.,4.,4.]),
              stamp=52.,receipt_wall_s=1000.25,receipt_seq=19)
    masked=SelfMask(cfg.lidar).apply(scan)
    cleaned=speckle_filter(masked,.4,.05)
    assert cleaned is not scan and math.isnan(cleaned.ranges[0]) and math.isnan(cleaned.ranges[3])
    assert (cleaned.receipt_wall_s,cleaned.receipt_seq,cleaned.stamp)==(1000.25,19,52.)
    assert scan.ranges[0]==.08 and scan.ranges[3]==.2
    legacy=Scan(0.,.1,np.ones(3),10.,8.)
    assert legacy.receipt_wall_s is None and legacy.receipt_seq is None


def test_hal_keeps_provided_receipt_without_inventing_one_for_legacy_sources():
    robot=WingbotRobot(Config(),transport=SimpleNamespace(),clock=lambda:20.)
    robot.on_scan(0.,.1,[1.,2.],1.,8.,receipt_wall_s=123.4,receipt_seq=7)
    assert robot.get_scan().receipt_wall_s==123.4 and robot.get_scan().receipt_seq==7
    robot.on_scan(0.,.1,[1.,2.],2.,8.)
    assert robot.get_scan().receipt_wall_s is None


def test_ros_scan_receipt_precedes_rebinning_and_rejected_frames_leave_sequence_gap(monkeypatch):
    import maicon26.hal.ros_mil as module
    transport=RosMilTransport.__new__(RosMilTransport)
    transport.cfg=Config();transport._scan_receipt_seq=0
    calls=[]
    transport.robot=SimpleNamespace(stats={},on_scan=lambda *args,**kw:calls.append((args,kw)))
    m=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=51,nanosec=20),frame_id=''),
        angle_min=-math.pi,angle_increment=math.pi/2,ranges=[1.,1.,1.,1.],range_min=.05,range_max=8.)
    times=iter((100.,200.))
    monkeypatch.setattr(module.time,'perf_counter',lambda:next(times))
    transport._on_scan(m)
    assert not calls
    m.header.frame_id=transport.cfg.nav2.base_frame
    transport._on_scan(m)
    assert calls[0][1]=={'receipt_wall_s':200.,'receipt_seq':2}
    assert calls[0][0][3]==pytest.approx(51.00000002)
    assert transport.robot.stats['scan_callbacks_received']==2


def test_worker_pairs_built_scan_not_newest_scan_and_reports_coalescing(monkeypatch):
    import maicon26.mission.risk_worker as module
    entered=threading.Event();release=threading.Event()
    def build(*args,**kwargs):
        if args[6]==1:
            entered.set();assert release.wait(2.)
        return None
    monkeypatch.setattr(module,'build_field',build)
    cfg=Config();grid=OccupancyGrid(cfg.arena);worker=RiskFieldWorker(cfg)
    receipt=time.perf_counter()-.05
    try:
        worker.submit([],grid,1.,1,urgent=True,scan_receipt_wall_s=receipt,scan_receipt_seq=1,scan_stamp=10.)
        assert entered.wait(1.)
        worker.submit([],grid,2.,2,urgent=True,scan_receipt_wall_s=receipt+.01,scan_receipt_seq=2,scan_stamp=11.)
        worker.submit([],grid,3.,3,urgent=True,scan_receipt_wall_s=receipt+.02,scan_receipt_seq=4,scan_stamp=13.)
        release.set();wait_version(worker,3)
        snapshot=worker.lineage_snapshot();records=snapshot['records'];counts=snapshot['counts']
        assert [r['receipt_seq'] for r in records]==[1,4]
        assert [r['scan_stamp'] for r in records]==[10.,13.]
        assert records[0]['receipt_wall_s']==receipt
        assert records[0]['newer_scan_submitted_before_publish']
        assert not records[1]['newer_scan_submitted_before_publish']
        assert records[0]['scan_callback_to_publish_ms']>=records[0]['submit_to_publish_ms']
        assert counts['coalesced_pending_requests']==1
        assert counts['receipt_sequences_not_submitted']==1
        assert counts['distinct_scan_submissions']==3
        assert counts['published_requests']==2 and snapshot['truncated_records']==0
    finally:
        release.set();worker.close()


def test_worker_separates_timer_reuse_from_fresh_and_missing_lineage(monkeypatch):
    import maicon26.mission.risk_worker as module
    monkeypatch.setattr(module,'build_field',lambda *a,**kw:None)
    cfg=Config();grid=OccupancyGrid(cfg.arena);worker=RiskFieldWorker(cfg)
    receipt=time.perf_counter()-.01
    try:
        for version,urgent,seq in ((1,True,7),(2,False,7),(3,True,None)):
            worker.submit([],grid,float(version),version,urgent=urgent,
                scan_receipt_wall_s=receipt if seq else None,scan_receipt_seq=seq,scan_stamp=22.)
            wait_version(worker,version)
        records=worker.lineage_snapshot()['records']
        assert [r['category'] for r in records]==['urgent_fresh_scan','timer_same_scan','unmatched_scan']
        assert records[1]['receipt_wall_s']==records[0]['receipt_wall_s']
        assert records[1]['scan_callback_to_publish_ms']>records[0]['scan_callback_to_publish_ms']
        assert records[2]['scan_callback_to_publish_ms'] is None
        assert worker.lineage_snapshot()['retained_limit']>=10000
    finally:worker.close()
