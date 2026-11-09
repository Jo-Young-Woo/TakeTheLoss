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
from collections import deque

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
        self.latency_ms = deque(maxlen=2000)
        self.build_ms = deque(maxlen=2000)
        self.errors = deque(maxlen=20)
        self._thread = threading.Thread(target=self._run, name="maicon-risk", daemon=True)
        self._thread.start()

    def submit(self, tracks, grid, now, version, *, hit_history=(), static_boxes=(),
               ego_velocity=(0., 0.), urgent=False):
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
            self._pending = request
            self._urgent |= bool(urgent)
            self._condition.notify()

    def latest(self):
        """Returns (version, field); field can be None when all tracks expire."""
        with self._condition:
            return self._result_version, self._result

    def close(self):
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        self._thread.join(timeout=2.)

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
                request, self._pending, self._urgent = self._pending, None, False
            tracks, grid, stamp, version, hits, boxes, velocity, received = request
            started = time.perf_counter()
            try:
                result = build_field(tracks,grid,self.cfg.avoid,self.cfg.laser.max_range,
                    stamp,self.cfg.nav.inflate_hard,version,
                    use_visibility=self.cfg.avoid.use_visibility,risk_cfg=self.cfg.risk,
                    min_clearance_m=self.cfg.engage.min_clearance_m,
                    hit_history=hits,static_boxes=boxes,ego_velocity=velocity,
                    body_radius_m=math.hypot(self.cfg.robot.length,self.cfg.robot.width)/2.)
                ended = time.perf_counter()
                with self._condition:
                    self._result_version, self._result = version,result
                    self.latency_ms.append(1000.*(ended-received))
                    self.build_ms.append(1000.*(ended-started))
            except Exception as exc:
                self.errors.append(f"{type(exc).__name__}: {exc}")
            due = time.perf_counter()+period
