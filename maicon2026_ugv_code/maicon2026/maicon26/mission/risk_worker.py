"""Latest-observation risk worker: 5 Hz timer plus immediate LiDAR wakeups.

No simulator API is imported. Inputs are snapshots of Agent sensor-derived
tracks and maps. A pending old request is replaced by the newest request;
publication records actual wall-clock latency instead of assuming a budget.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from collections import Counter, deque

from .threat_field import build_field


class RiskFieldWorker:
    def __init__(self, cfg):
        self.cfg = cfg
        self._condition = threading.Condition()
        self._pending = None
        self._result = None
        self._result_version = -1
        self._stop = False
        self._urgent = False
        # Covers a <=330-second 10 Hz trial with ample room; retained/total
        # counts expose any truncation in longer field sessions.
        self.latency_ms = deque(maxlen=10000)
        self.build_ms = deque(maxlen=10000)
        self.scan_lineage = deque(maxlen=10000)
        self.lineage_counts = Counter()
        self._latest_scan_seq = None
        self._latest_submitted_version = -1
        self.errors = deque(maxlen=20)
        self._thread = threading.Thread(target=self._run, name="maicon-risk", daemon=True)
        self._thread.start()

    def submit(self, tracks, grid, now, version, *, hit_history=(), static_boxes=(),
               ego_velocity=(0., 0.), urgent=False, scan_receipt_wall_s=None,
               scan_receipt_seq=None, scan_stamp=None):
        started = time.perf_counter()
        frozen_grid = copy.copy(grid)
        frozen_grid.L = grid.L.copy()
        frozen_grid.static = grid.static.copy()
        frozen_grid.layer3d = None if grid.layer3d is None else grid.layer3d.copy()
        request = (copy.deepcopy(tuple(tracks)), frozen_grid, float(now), int(version),
                   tuple(hit_history), copy.deepcopy(tuple(static_boxes)), tuple(ego_velocity), started)
        with self._condition:
            if self._stop:
                return
            valid = (isinstance(scan_receipt_seq,int) and not isinstance(scan_receipt_seq,bool)
                     and scan_receipt_seq > 0 and isinstance(scan_receipt_wall_s,(int,float))
                     and math.isfinite(scan_receipt_wall_s) and 0 <= scan_receipt_wall_s <= started)
            fresh = valid and scan_receipt_seq != self._latest_scan_seq
            category = (('urgent_' if urgent else 'timer_') + ('fresh_scan' if fresh else 'same_scan')) if valid else 'unmatched_scan'
            self.lineage_counts['submitted_requests'] += 1
            self.lineage_counts[category] += 1
            if self._pending is not None:
                self.lineage_counts['coalesced_pending_requests'] += 1
                old_lineage=self._pending[-1]
                if old_lineage['receipt_seq'] != scan_receipt_seq:
                    self.lineage_counts['coalesced_different_scan_requests'] += 1
            if fresh:
                if self._latest_scan_seq is not None and scan_receipt_seq > self._latest_scan_seq:
                    self.lineage_counts['receipt_sequences_not_submitted'] += max(0,scan_receipt_seq-self._latest_scan_seq-1)
                self._latest_scan_seq = scan_receipt_seq
                self.lineage_counts['distinct_scan_submissions'] += 1
            if not valid:
                self.lineage_counts['missing_or_invalid_lineage_submissions'] += 1
            self._latest_submitted_version = int(version)
            lineage=dict(receipt_wall_s=float(scan_receipt_wall_s) if valid else None,
                         receipt_seq=scan_receipt_seq if valid else None,scan_stamp=scan_stamp,
                         submit_wall_s=started,category=category,urgent=bool(urgent))
            self._pending = request + (lineage,)
            self._urgent |= bool(urgent)
            self._condition.notify()

    def latest(self):
        """Returns (version, field); field can be None when all tracks expire."""
        with self._condition:
            return self._result_version, self._result

    def close(self):
        with self._condition:
            if self._pending is not None:
                self.lineage_counts['pending_requests_discarded_on_close'] += 1
                self._pending = None
            self._stop = True
            self._condition.notify_all()
        self._thread.join(timeout=2.)

    def lineage_snapshot(self):
        """Diagnostic publication history, not a control or freshness signal."""
        with self._condition:
            return dict(counts=dict(self.lineage_counts),records=list(self.scan_lineage),
                        retained_limit=self.scan_lineage.maxlen,
                        retained_records=len(self.scan_lineage),
                        truncated_records=max(0,self.lineage_counts['published_requests']-len(self.scan_lineage)),
                        scope='Monotonic ROS scan-handler entry to publication of the request built from that exact scan. Includes queued Agent/map processing; excludes transport before callback and Agent consumption after publication. Receipt gaps are not DDS-loss evidence.')

    def _run(self):
        period = 1./max(.1,float(getattr(self.cfg.risk,"update_hz",5.)))
        due = 0.
        while True:
            with self._condition:
                while not self._stop:
                    now = time.perf_counter()
                    if self._pending is not None and (self._urgent or now >= due):
                        break
                    wait = max(.001,due-now) if self._pending is not None else None
                    self._condition.wait(timeout=wait)
                if self._stop:
                    return
                request, self._pending, dispatch_urgent, self._urgent = self._pending, None, self._urgent, False
                self.lineage_counts['builds_started'] += 1
            tracks, grid, stamp, version, hits, boxes, velocity, received, lineage = request
            lineage=dict(lineage,dispatch_urgent=dispatch_urgent)
            started = time.perf_counter()
            try:
                result = build_field(tracks,grid,self.cfg.avoid,self.cfg.laser.max_range,
                    stamp,self.cfg.nav.inflate_hard,version,
                    use_visibility=self.cfg.avoid.use_visibility,risk_cfg=self.cfg.risk,
                    min_clearance_m=self.cfg.engage.min_clearance_m,
                    hit_history=hits,static_boxes=boxes,ego_velocity=velocity,
                    body_radius_m=math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2.)
                with self._condition:
                    ended = time.perf_counter()
                    self._result_version, self._result = version,result
                    self.latency_ms.append(1000.*(ended-received))
                    self.build_ms.append(1000.*(ended-started))
                    self.lineage_counts['published_requests'] += 1
                    newer_scan=(lineage['receipt_seq'] is not None and
                                self._latest_scan_seq is not None and self._latest_scan_seq>lineage['receipt_seq'])
                    if newer_scan:
                        self.lineage_counts['publications_after_newer_scan_submitted'] += 1
                    self.scan_lineage.append(dict(lineage,version=version,build_start_wall_s=started,
                        publish_wall_s=ended,latest_submitted_version=self._latest_submitted_version,
                        newer_scan_submitted_before_publish=newer_scan,
                        scan_callback_to_publish_ms=(1000.*(ended-lineage['receipt_wall_s']) if lineage['receipt_wall_s'] is not None else None),
                        submit_to_publish_ms=1000.*(ended-received),build_ms=1000.*(ended-started)))
            except Exception as exc:
                self.errors.append(f"{type(exc).__name__}: {exc}")
                with self._condition:
                    self.lineage_counts['build_errors'] += 1
            due = time.perf_counter()+period
