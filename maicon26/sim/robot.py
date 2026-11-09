"""시뮬레이터용 Robot/ServerLink 구현 + 서버가 보내는 자연어 명령 생성."""
from __future__ import annotations

import math
from copy import deepcopy
import zlib
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

import numpy as np

from ..hal.base import Robot
from ..link.base import ServerLink, ServerMsg, UavBox, UavDet, UavFrame
from ..link.schema import OBJECT_CODES, server_sector
from ..types import DepthFrame, Frame, Pose2D, RobotStatus, Scan
from ..vocab import BASES, DISPLAY_KR, NATO, TARGETS
from .world import SimUGV, World


def josa(word: str, with_final: str, without_final: str) -> str:
    ch = word[-1]
    if "가" <= ch <= "힣":
        return word + (with_final if (ord(ch) - 0xAC00) % 28 else without_final)
    return word + without_final


class SimRobot(Robot):
    """한 대의 SimUGV를 HAL 인터페이스로 감싼다. LiDAR 10 Hz, 카메라 15 Hz."""

    can_abort_laser = True

    def __init__(self, world: World, ugv: SimUGV, scan_hz: float = 10.0, cam_hz: float = 15.0) -> None:
        self.world, self.u = world, ugv
        self._scan: Optional[Scan] = None
        self._scan_q: Deque[Scan] = deque(maxlen=12)
        self._pose_off = [0.0, 0.0]                       # 위치추정 오차 시험용 OU 오프셋 (lidar_err.sim_pose_sigma_m)
        self._pose_rng = np.random.default_rng(zlib.crc32(ugv.id.encode()) + 5)
        self._frame: Optional[Frame] = None
        self._depth: Optional[DepthFrame] = None
        self._scan_dt, self._cam_dt = 1.0 / scan_hz, 1.0 / cam_hz
        self._t_scan = self._t_cam = self._t_depth = -1e9

    def now(self) -> float:
        return self.world.t

    def set_velocity(self, v: float, w: float) -> None:
        self.u.cmd_v, self.u.cmd_w = float(v), float(w)

    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        self.u.pan_cmd, self.u.tilt_cmd = float(pan_deg), float(tilt_deg)

    def get_pan_tilt(self) -> Tuple[float, float]:
        return self.u.pan, self.u.tilt

    def laser_fire(self) -> bool:
        return self.u.fire()

    def laser_abort(self) -> None:
        self.u.abort()

    def get_scan(self) -> Optional[Scan]:
        if self.world.t - self._t_scan >= self._scan_dt:
            self._scan_q.append(self.world.lidar(self.u))
            self._t_scan = self.world.t
        lat = self.world.cfg.lidar_err.sim_latency_s
        if lat <= 0:
            self._scan = self._scan_q[-1] if self._scan_q else None
        else:                                           # 전달 지연: lat 이전에 측정된 가장 최근 스캔을 준다(stamp 는 측정 시각)
            ready = [s for s in self._scan_q if s.stamp <= self.world.t - lat]
            self._scan = ready[-1] if ready else None
        return self._scan

    def get_frame(self) -> Optional[Frame]:
        if self.world.t - self._t_cam >= self._cam_dt:
            cam = self.world.cfg.camera
            dets = self.world.camera_detections(self.u)
            self._frame = Frame(image=None, stamp=self.world.t, pan=self.u.pan, tilt=self.u.tilt,
                                width=cam.width, height=cam.height, meta={"sim_detections": dets})
            self._t_cam = self.world.t
        return self._frame

    def get_depth(self) -> Optional[DepthFrame]:
        """깊이 카메라(config.depth.sim_hz, 0 이면 없음). 촬영 시각·팬틸트·K 를 같이 준다."""
        dc = self.world.cfg.depth
        if not dc.enable or dc.sim_hz <= 0:
            return None
        if self.world.t - self._t_depth >= 1.0 / dc.sim_hz:
            img = self.world.depth_image(self.u, dc)
            self._depth = DepthFrame(depth=img, stamp=self.world.t, pan=self.u.pan, tilt=self.u.tilt,
                                     K=tuple(dc.intrinsics(img.shape[1], img.shape[0]).K()))
            self._t_depth = self.world.t
        return self._depth

    def get_odom(self) -> Pose2D:
        od = self.u.odom.copy()
        sg = self.world.cfg.lidar_err.sim_pose_sigma_m
        if sg > 0:                                        # 자기 위치추정 오차(느리게 변하는 오프셋): τ=5 s 의 OU 과정
            a = math.exp(-0.05 / 5.0)
            for i in (0, 1):
                self._pose_off[i] = a * self._pose_off[i] + math.sqrt(1 - a * a) * sg * float(self._pose_rng.normal())
            od.x += self._pose_off[0]
            od.y += self._pose_off[1]
        return od

    def status(self) -> RobotStatus:
        return self.u.status()


class SimServer(ServerLink):
    LATENCY = 0.4

    def __init__(self, world: World, ugv: SimUGV, announce_identity: bool = True,
                 sector_style: str = "nato") -> None:
        self.world, self.u = world, ugv
        self.announce_identity = announce_identity
        self._sent_start = self._sent_base = self._sent_stop = False
        self._event_idx = 0
        self._hist: Deque[Tuple[float, UavFrame]] = deque(maxlen=20)
        # Gazebo supplies an image-derived frame here. None is the explicitly
        # synthetic Python simulator; a provider returning None never falls
        # back to hidden world geometry.
        self.uav_layout_provider: Optional[Callable[[], Optional[UavFrame]]] = None
        # Explicit Gazebo opt-in: only detections supplied by the image provider
        # may be exposed. The generic Python simulator keeps its legacy model.
        self.uav_image_only = False
        # Fixed M3 rehearsal opts into overhead geometry. Preserve the older
        # generic/random scenario's sensor contract unless explicitly enabled.
        self.uav_layout_enabled = world.scenario == "maicon_fixed"
        self._last_layout_stamp = -math.inf
        self.uav_layout_frames = 0
        self.uav_layout_boxes = 0
        self._last_uav_stamp = -1e9
        # Pure Python sim uses zero. Gazebo match sets its live /clock origin.
        self.uav_stamp_offset = 0.0
        self.uav_frames_generated = 0
        self.uav_frames_delivered = 0
        self._last_uav_delivered = -1e9
        self.style = sector_style
        self.truth = self._make_truth()
        # The second order is per alliance, not a separate random base per UGV.
        # The first scenario assignment selects it; later members share it.
        self.truth["m5_base"] = world.goal_base_types.setdefault(ugv.team, self.truth["m5_base"])
        world.assign[ugv.id] = self.truth

    # ------------------------------------------------------------------ 진실값/명령문
    def _make_truth(self) -> Dict[str, Any]:
        if self.world.scenario == "maicon_fixed":
            from .fixed_maicon import B2_MISSION
            truth = dict(B2_MISSION)
            opposite = "R" if self.u.team == "BLUE" else "B"
            truth["sectors"] = (f"{opposite}-S2", "COMMON")
            return truth
        rng = self.world.rng
        ttypes = list(TARGETS.keys())
        btypes = list(BASES.keys())
        side_secs = self.world.geo.sector_names("RED" if self.u.team == "BLUE" else "BLUE")
        if self.style == "nato":
            secs = rng.sample(NATO[:4], 2)
        else:
            secs = rng.sample(side_secs, 2)
        return {"target": ttypes[(self.u.slot - 1 + rng.randrange(3)) % 3],
                "sectors": secs,
                "goal_base": rng.choice(btypes),
                "m5_base": btypes[(self.u.slot - 1 + rng.randrange(3)) % 3]}

    def command_text(self) -> str:
        if self.world.scenario == "maicon_fixed":
            s1, s2 = self.truth["sectors"]
            return (f"우선 타격 표적은 {DISPLAY_KR[self.truth['target']]}. "
                    f"{s1}와 {s2} 구역을 정찰하라. "
                    f"후속 작전 거점은 {DISPLAY_KR[self.truth['goal_base']]}로 설정하라.")
        rng = self.world.rng
        t = DISPLAY_KR[self.truth["target"]]
        b = DISPLAY_KR[self.truth["goal_base"]]
        s1, s2 = self.truth["sectors"]
        k = rng.randrange(3)
        if k == 0:
            return (f"{josa(t, '을', '를')} 주요 타격대상으로 지정한다. 객체 탐색은 {s1}와 {s2} 구역에서 "
                    f"수행하도록 계획하며, 후속 작전 거점은 {josa(b, '으로', '로')} 설정하라.")
        if k == 1:
            return (f"작전 명령. 우선 타격 표적은 {t}. {s1}, {s2} 섹터를 정찰하라. "
                    f"이후 {josa(b, '으로', '로')} 이동 준비.")
        return (f"{s1} 및 {s2} 구역에서 전장 객체를 탐지하고, {josa(t, '을', '를')} 타격하라. "
                f"작전 거점은 {josa(b, '이다', '다')}.")

    def base_command_text(self) -> str:
        if self.world.scenario == "maicon_fixed":
            return f"목표거점은 {DISPLAY_KR[self.truth['m5_base']]}다. 즉시 진입하라."
        b = DISPLAY_KR[self.truth["m5_base"]]
        return f"목표거점은 {josa(b, '이다', '다')}. 즉시 진입하라." if self.world.rng.random() < 0.5 else \
            f"{josa(b, '을', '를')} 점령하라. 경로를 재계획하여 진입한다."

    # ------------------------------------------------------------------ ServerLink
    def identity(self):
        return (self.u.team, self.u.slot) if self.announce_identity else None

    def poll(self) -> List[ServerMsg]:
        w, out = self.world, []
        if not self._sent_start and w.t >= 0.0:
            self._sent_start = True
            out.append(ServerMsg("start", stamp=w.t))
            out.append(ServerMsg("command", text=self.command_text(), stamp=w.t))
        if (not self._sent_base and w.t >= w.cfg.mission.t_base_cmd) and not w.ended:
            self._sent_base = True
            opp = "RED" if self.u.team == "BLUE" else "BLUE"
            self.u.goal_base = w.base_of_type(opp, self.truth["m5_base"])
            out.append(ServerMsg("base_command", text=self.base_command_text(), stamp=w.t))
        if w.ended and not self._sent_stop:
            self._sent_stop = True
            out.append(ServerMsg("stop", stamp=w.t))
        # 이벤트(내 유효타격 / 피격)
        while self._event_idx < len(w.events):
            ev = w.events[self._event_idx]
            self._event_idx += 1
            if ev.get("by") == self.u.id or ev.get("victim") == self.u.id or ev.get("broadcast"):
                out.append(ServerMsg("event", data=dict(ev), stamp=w.t))
        return out

    def submit(self, kind: str, payload: Dict[str, Any]) -> bool:
        key = f"{self.u.id}"
        if getattr(self.world, "finished", False):
            self.last_submit_status = 409
            return False
        if kind in ("m1", "m3"):
            # Judge receipt time, including exact 5:00 and early match end.
            # M6's report-phase allowance never extends M3's deadline.
            if self.world.ended or self.world.t >= min(self.world.cfg.mission.t_total, 300.):
                self.last_submit_status = 409
                return False
        if kind == "m3":
            sectors = payload.get("sectors")
            if not isinstance(sectors, dict) or any(
                    server_sector(sec) is None or not isinstance(counts, dict) or any(
                        cls not in OBJECT_CODES or not isinstance(n, int) or isinstance(n, bool) or not 0 <= n <= 99
                        for cls, n in counts.items()) for sec, counts in sectors.items()):
                self.last_submit_status = 422
                return False
        if self.world.ended and self.world.end_time is not None and \
                self.world.t >= self.world.end_time + self.world.cfg.mission.t_report:
            self.last_submit_status = 409
            return False
        if kind == "m6" and not self.world.ended:
            self.last_submit_status = 409
            return False
        d = self.world.submissions.setdefault(key, {})
        d.setdefault(kind, {"payload": payload, "t": self.world.t, "n": 0})
        d[kind]["payload"] = deepcopy(payload)
        d[kind]["t"] = self.world.t
        d[kind]["n"] += 1
        self.last_submit_status = 200
        if kind == "m6" and len(getattr(self.world, "ugvs", ())) == 6 and all("m6" in self.world.submissions.get(u.id, {}) for u in self.world.ugvs):
            self.world.finished = True
        return True

    def get_uav(self) -> Optional[UavFrame]:
        w = self.world
        period = w.uav_frame_interval(self.u.team)
        if period is None:
            return None
        if w.t - self._last_uav_stamp >= period:
            self._last_uav_stamp = w.t
            dets = []
            if not self.uav_image_only:
                rng = np.random.default_rng(zlib.crc32(f"{self.u.id}{int(w.t)}".encode()))
                dets = [UavDet(x=o.pose.x + float(rng.normal(0, w.uav_sigma)), y=o.pose.y + float(rng.normal(0, w.uav_sigma)),
                               team=o.team) for o in w.ugvs]
            frame = UavFrame(stamp=w.t + self.uav_stamp_offset,
                             detections=dets, interval_s=period)
            if self.uav_layout_enabled:
                if self.uav_layout_provider is None and not self.uav_image_only:
                    # Geometry-only perception model for the Python simulator.
                    # Use a separate RNG so this does not change UGV detections.
                    box_rng = np.random.default_rng(zlib.crc32(f"layout{int(w.t)}".encode()))
                    frame.boxes = [UavBox(c.cx + float(box_rng.normal(0, .015)),
                                          c.cy + float(box_rng.normal(0, .015)),
                                          c.sx, c.sy, .9) for c in w.cubes]
                    frame.layout_source = "synthetic_overhead_footprints"
                    frame.layout_stamp = frame.stamp
                elif self.uav_layout_provider is not None:
                    observed = self.uav_layout_provider()
                    # Fixed camera runs at 1 Hz; accept only recent, new frames
                    # on the same Gazebo clock. Never relabel stale images.
                    if (observed is not None and observed.stamp > self._last_layout_stamp
                            and -.15 <= frame.stamp - observed.stamp <= 2.5):
                        self._last_layout_stamp = observed.stamp
                        frame.boxes = list(observed.boxes)
                        frame.layout_source = observed.layout_source
                        frame.layout_stamp = observed.stamp
                        if self.uav_image_only:
                            # The current top detector produces boxes only;
                            # absent pixel-based UGV detections stay empty.
                            # A future image robot detector can supply them
                            # without changing this transport's contract.
                            frame.detections = list(observed.detections)
                            frame.image = observed.image
                if frame.layout_source is not None:
                    self.uav_layout_frames += 1
                    self.uav_layout_boxes += len(frame.boxes)
            self._hist.append((w.t, frame))
            self.uav_frames_generated += 1
        for t0, frame in reversed(self._hist):
            if w.t - t0 >= w.uav_latency:
                if t0 > self._last_uav_delivered:
                    self._last_uav_delivered = t0
                    self.uav_frames_delivered += 1
                    frame.interval_s = period
                    if self.uav_image_only and frame.layout_stamp is not None:
                        # Keep the organizer's monotonic delivery sequence, but
                        # let Agent restore actual capture time before tracking.
                        # Stamping the entire packet with an older image time
                        # would hide a valid image behind an earlier empty packet.
                        frame.age_s = max(0., w.t + self.uav_stamp_offset - frame.layout_stamp)
                    return frame
                return None
        return None
