"""M3 전장객체 집계 + 지정표적/거점표지 위치 기억.

핵심 아이디어(2025 강의의 '구간 Max / 다수결'을 위치 기반으로 확장):
  1) 검출 하나 → (월드 방위, 거리) → 월드 좌표의 '면 위 한 점'
  2) 같은 점 근처(merge_dist) 관측은 한 '면'으로 합치고 클래스는 신뢰도 가중 투표
  3) 면의 섹터는 '박스 중심 추정(면 점에서 시선 방향으로 20cm 안쪽)'의 섹터
  4) 보고: min_votes 이상 관측된 면만, 섹터별·클래스별 개수
같은 객체를 여러 프레임에서 봐도 한 번만 세고, 한 면에는 객체가 1개뿐이라는 규칙을 이용한다.
"""
from __future__ import annotations

import math
from collections import defaultdict, deque

import numpy as np
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from ..arena import ArenaGeometry
from ..config import Config
from ..types import Detection, Frame, Pose2D, Scan
from .camera import CameraModel, ObsCamera, estimate_range, lidar_range_at

DISAGREE_MIN_OBS = 2          # 팬틸트·OAK 가 각각 이만큼 본 면에서만 '클래스가 엇갈린다'고 본다(한 장의 오검출로 법석을 떨지 않는다)


@dataclass
class Face:
    x: float
    y: float
    cx: float          # 박스 중심 추정
    cy: float
    votes: Dict[str, float] = field(default_factory=lambda: defaultdict(float))
    n: int = 0
    kind: str = "object"
    led_on: bool = False
    led_votes: int = 0   # 시그널 LED '점등' 관측 연속 확인용(event.led_confirm_frames 이상이면 led_on). 한 프레임 오검출로 M2 를 끝내지 않는다
    last_t: float = 0.0
    vx: float = 0.0     # 면에서 로봇 쪽으로 향하는 단위벡터(관측 방향 평균) — 정면 접근 지점 계산용
    vy: float = 0.0
    geometry_key: Optional[Tuple[int, int]] = None  # Observed box id / face index, never a truth id.
    # ---- 6장(OAK 보조 탐지): 출처별 관측 수·득표. 팬틸트만 쓰는 기존 경로에서는 {"pantilt": n} 하나뿐이다.
    src_n: Dict[str, float] = field(default_factory=dict)
    src_votes: Dict[str, Dict[str, float]] = field(default_factory=dict)
    # 아래 두 필드는 최종 보고서(run_wingbot_autonomous_trial.json_value)·체크포인트가 그대로 JSON 으로 쓰므로 list 로 둔다
    # (deque·set 은 직렬화가 안 된다). Face(**저장된_dict) 로 되살려도 같은 모양이어야 해서 튜플도 쓰지 않는다.
    oak_obs: list = field(default_factory=list)         # vote_window 가 있을 때만 쓰는 최근 OAK 관측 [클래스, 가중 득표, 가중치] (오래된 것이 앞)
    disagreed: list = field(default_factory=list)       # 이미 알린 엇갈림 "팬틸트클래스>OAK클래스" 문자열

    @property
    def cls(self) -> str:
        return max(self.votes.items(), key=lambda kv: kv[1])[0]

    @property
    def mean_conf(self) -> float:
        return self.votes[self.cls] / max(self.n, 1)


@dataclass(frozen=True)
class GeometryAnchor:
    key: Tuple[int, int]
    x: float
    y: float
    cx: float
    cy: float
    nx: float
    ny: float
    distance: float


def surface_normal(scan: Optional[Scan], pose_s: Pose2D, world_bearing: float, r: float,
                   win: int = 6) -> Optional[Tuple[float, float]]:
    """LiDAR 점들의 국소 직선 맞춤으로 면의 법선(로봇 쪽을 향함). 코너/잡음으로 직선이 아니면 None.
    인접 면(법선이 90° 다름)을 구분하는 데 쓴다 — 위치만으로는 인접 면 간격(0.28 m)이 오차와 비슷하다."""
    if scan is None:
        return None
    n = len(scan.ranges)
    x = ((world_bearing - pose_s.th - scan.angle_min) / scan.angle_inc) % n
    i0 = int(round(x))
    idx = [(i0 + k) % n for k in range(-win, win + 1)]
    rr = scan.ranges[idx]
    ang = scan.angles[idx] + pose_s.th
    ok = np.isfinite(rr) & (np.abs(rr - r) < 0.18) & (rr > 0.05)
    if ok.sum() < 4:
        return None
    pts = np.stack([pose_s.x + rr[ok] * np.cos(ang[ok]), pose_s.y + rr[ok] * np.sin(ang[ok])], axis=1)
    c = pts.mean(axis=0)
    _, sv, vt = np.linalg.svd(pts - c, full_matrices=False)
    m = len(pts)
    if sv[0] / math.sqrt(m) < 0.025 or sv[1] / math.sqrt(m) > 0.02:     # 퇴화(점이 한 곳) 또는 직선 아님
        return None
    d = vt[0]
    nrm = np.array([-d[1], d[0]])
    if nrm @ (np.array([pose_s.x, pose_s.y]) - c) < 0:
        nrm = -nrm
    return float(nrm[0]), float(nrm[1])


class ObjectMap:
    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.geo: ArenaGeometry = cfg.arena
        self.cam = CameraModel(cfg.camera)
        self.faces: List[Face] = []
        self.base_votes: Dict[str, Dict[str, float]] = defaultdict(lambda: defaultdict(float))  # 거점 → 표지클래스 → 가중표
        self.base_n: Dict[str, Dict[str, int]] = defaultdict(lambda: defaultdict(int))
        self._target_cls = set(cfg.perception.target_classes)
        self._object_cls = set(cfg.perception.object_classes)
        self._base_cls = {"COMMAND_POST", "MAINT_DEPOT", "SUPPLY_BASE"}
        self._nonobject_cls = set(cfg.perception.nonobject_classes)
        self.nonobject_seen = 0                  # 비인식용 박스(도시 나무·차단벽·전진지·건물)로 분류돼 무시한 검출 수
        self.association_stats = defaultdict(int)
        self.identify_mode = False               # 매니저가 식별 단계(미식별 면을 OAK 로 다시 보는 중)에만 켠다 — 그때 OAK 관측에 'oak_id' 출처를 따로 남긴다
        self.disagreements: deque = deque(maxlen=16)   # 팬틸트와 OAK 의 클래스가 엇갈린 면(6장). Agent 가 pop_disagreements() 로 가져간다

    # ------------------------------------------------------------------ 월드 좌표화
    def on_anchor_change(self, pivot: Tuple[float, float], trim: Tuple[float, float, float]) -> int:
        """SLAM→경기장 앵커가 바뀌었다(PoseGuard 앵커 재설정·원복). 로봇 자세로 투영해 쌓은 면 위치·접근 방향을 같은 변환 p' = R(dth)(p−pivot)+pivot+d 로 옮긴다
        (득표·시각은 그대로). 옮기지 않으면 CoverageMap 이 위치로 득표를 맞추는 곳에서 식별한 면이 새 박스와 어긋난다. 반환: 옮긴 면 수."""
        dx, dy, dth = trim
        c, s = math.cos(dth), math.sin(dth)
        for f in self.faces:
            for ax, ay in (("x", "y"), ("cx", "cy")):
                px, py = getattr(f, ax) - pivot[0], getattr(f, ay) - pivot[1]
                setattr(f, ax, pivot[0] + c * px - s * py + dx)
                setattr(f, ay, pivot[1] + s * px + c * py + dy)
            f.vx, f.vy = c * f.vx - s * f.vy, s * f.vx + c * f.vy
        return len(self.faces)

    def _cam_and_pan(self, frame: Frame, cam: Optional[ObsCamera]) -> Tuple[CameraModel, float]:
        """관측에 쓸 카메라 모델과 '팬' 자리의 각도. cam 을 안 넘기면 기존처럼 팬틸트 카메라와 frame.pan."""
        if cam is None:
            return self.cam, frame.pan
        return cam.model, frame.pan + cam.pan_equiv_deg

    def locate(self, det: Detection, pose: Pose2D, frame: Frame, scan: Optional[Scan],
               real_h: float = 0.40, dth: float = 0.0, cam: Optional[ObsCamera] = None) -> Optional[Tuple[float, float, float, float]]:
        """(x, y, 거리, 월드방위) — 실패하면 None. pose는 '프레임 촬영 시각'의 자세여야 한다.
        cam: 팬틸트가 아닌 카메라(OAK)의 모델·장착. None 이면 지금처럼 팬틸트 카메라."""
        cm, pan = self._cam_and_pan(frame, cam)
        r, how = estimate_range(cm, det, real_h, scan, pan, dth)
        if r is None:
            return None
        psi = cm.world_ray(pose, pan, det.cx, det.cy)
        ox, oy = (pose.x, pose.y) if cam is None else cam.origin(pose, how)
        return (ox + r * math.cos(psi), oy + r * math.sin(psi), r, psi)

    def _edge_ok(self, det: Detection, frame: Frame, margin: float = 6.0) -> bool:
        x1, y1, x2, y2 = det.box
        return x1 > margin and y1 > margin and x2 < frame.width - margin and y2 < frame.height - margin

    def _geometry_anchor(self, boxes, pose, psi, measured_range, normal):
        """Associate one image ray to a unique, repeatedly observed cube plane.

        Only local LiDAR-resolved 40cm footprints qualify. The live LiDAR must
        agree with the ray/plane intersection; image size cannot authorize it.
        """
        ux, uy = math.cos(psi), math.sin(psi)
        candidates, angular_support = [], False
        for box in boxes:
            if (box.kind != "full" or box.n_seen < 2 or box.full_seen < 2 or not box.cells
                    or abs(box.sx-.4) > .08 or abs(box.sy-.4) > .08):
                continue
            for face in box.faces:
                dx, dy = face.cx-pose.x, face.cy-pose.y
                distance = math.hypot(dx, dy)
                if distance < .2 or -(dx*face.nx+dy*face.ny)/distance < .35:
                    continue
                denom = ux*face.nx+uy*face.ny
                if denom >= -.1:
                    continue
                along = (dx*face.nx+dy*face.ny)/denom
                px, py = pose.x+along*ux, pose.y+along*uy
                tangent = abs((px-face.cx)*face.ny-(py-face.cy)*face.nx)
                half = (box.sy if abs(face.nx) > .5 else box.sx)/2
                if along <= 0. or tangent > half+.04:
                    continue
                angular_support = True
                if measured_range is None or abs(along-measured_range) > .10+.03*measured_range:
                    continue
                if normal is not None and (normal[0]*face.nx+normal[1]*face.ny) < .65:
                    continue
                anchor = GeometryAnchor((box.bid, face.idx), face.cx, face.cy, box.cx, box.cy,
                                        face.nx, face.ny, along)
                score = abs(along-measured_range)/.08 + .15*tangent/max(half, .1)
                candidates.append((score, anchor))
        if not candidates:
            return None, "range_rejected" if angular_support else "no_geometry"
        candidates.sort(key=lambda item: item[0])
        if len(candidates) > 1 and candidates[1][0]-candidates[0][0] < .25:
            return None, "ambiguous"
        return candidates[0][1], "anchored"

    # ------------------------------------------------------------------ 관측 입력
    def observe(self, dets: List[Detection], pose: Pose2D, frame: Frame, scan: Optional[Scan],
                dth: float = 0.0, scan_pose: Optional[Pose2D] = None, max_range: Optional[float] = None,
                boxes=None, cam: Optional[ObsCamera] = None, weight: float = 1.0, source: str = "pantilt",
                pt_gate: Optional[Callable[[Optional[Tuple[int, int]]], bool]] = None) -> None:
        """max_range: 집계에 쓰는 최대 거리(스모그 중 먼 객체는 가려져 오분류하기 쉬워 줄인다). None 이면 perception.max_obj_range.
        cam / weight / source (6장): 팬틸트 카메라가 아닌 관측(OAK)의 카메라 모델·장착, 투표 가중치, 출처 이름. 안 넘기면 기존과 같다.
        pt_gate (oak_det.identify_primary): 팬틸트 영상의 '표지 물체(면)' 득표를 받을지 정하는 함수. 인자는 LiDAR 면 키(bid, idx) — 위치 닻이 없으면 None.
        False 면 그 득표를 버린다(OAK 가 맡은 면을 팬틸트가 먼저 채우지 않게). 표적(M2)·거점 표지는 대상이 아니다. 안 넘기면 기존과 같다."""
        pc = self.cfg.perception
        cm, pan = self._cam_and_pan(frame, cam)
        rmax = pc.max_obj_range if max_range is None else min(pc.max_obj_range, max_range)
        for d in dets:
            if d.conf < pc.conf_min:
                continue
            if d.cls in self._nonobject_cls:           # 비인식용 박스는 세지 않는다(장애물일 뿐)
                self.nonobject_seen += 1
                continue
            if d.cls in self._base_cls:
                if source != "pantilt":                 # 거점 표지는 팬틸트로만 센다(OAK 는 보조 — 6장)
                    continue
                loc = self.locate(d, pose, frame, scan, real_h=0.30, dth=dth)
                if loc and loc[2] < 4.0:
                    # 같은 종류의 표지가 양 진영에 하나씩 있으므로, 관측 위치에서 가장 가까운 거점에 귀속시킨다
                    names = ("B-1", "B-2", "B-3", "R-1", "R-2", "R-3")
                    b = min(names, key=lambda n: math.hypot(self.geo.base_center(n)[0] - loc[0],
                                                            self.geo.base_center(n)[1] - loc[1]))
                    if math.hypot(self.geo.base_center(b)[0] - loc[0], self.geo.base_center(b)[1] - loc[1]) < 1.0:
                        self.base_votes[b][d.cls] += d.conf
                        self.base_n[b][d.cls] += 1
                continue
            if d.cls not in self._object_cls and d.cls not in self._target_cls:
                continue
            if not self._edge_ok(d, frame):          # 잘린 bbox는 분류가 흔들리므로 제외
                continue
            if boxes is not None and d.cls in self._object_cls:
                psi = cm.world_ray(pose, pan, d.cx, d.cy)
                measured = lidar_range_at(scan, math.radians(pan)+cm.bearing(d.cx, d.cy)+dth)
                normal = surface_normal(scan, scan_pose or pose, psi, measured) if measured is not None else None
                anchor, reason = self._geometry_anchor(boxes, pose, psi, measured, normal)
                if anchor is not None:
                    if source == "pantilt" and pt_gate is not None and not pt_gate(anchor.key):
                        self.association_stats["pt_gated"] += 1
                        continue
                    if anchor.distance <= rmax:
                        self.association_stats["anchored"] += 1
                        self._add(d, anchor.x, anchor.y, psi, frame.stamp, pose,
                                  (anchor.nx, anchor.ny), anchor=anchor, weight=weight, source=source)
                    else:
                        self.association_stats["beyond_max_range"] += 1
                    continue
                self.association_stats[reason] += 1
                if reason in ("range_rejected", "ambiguous"):
                    continue  # A size-only guess must not overrule known geometry.
                _, ranging = estimate_range(cm, d, .40, scan, pan, dth)
                if measured is not None and ranging == "size":
                    self.association_stats["size_range_conflict"] += 1
                    continue  # Wait for geometry; do not seed a false distant face.
            if source == "pantilt" and pt_gate is not None and d.cls in self._object_cls and not pt_gate(None):
                self.association_stats["pt_gated"] += 1      # 닻 없는 팬틸트 표지 득표 — OAK 우선 모드에서는 폴백 면을 보는 중에만 받는다
                continue
            loc = self.locate(d, pose, frame, scan, dth=dth, cam=cam)
            if loc is None or loc[2] > rmax:
                if d.cls in self._object_cls:
                    self.association_stats["no_range" if loc is None else "beyond_max_range"] += 1
                continue
            x, y, r, psi = loc
            nrm = surface_normal(scan, scan_pose or pose, psi, r)
            self._add(d, x, y, psi, frame.stamp, pose, nrm, weight=weight, source=source)

    def _add(self, d: Detection, x: float, y: float, psi: float, t: float, pose: Pose2D,
             nrm: Optional[Tuple[float, float]] = None, anchor: Optional[GeometryAnchor] = None,
             weight: float = 1.0, source: str = "pantilt") -> None:
        pc = self.cfg.perception
        w = float(weight)
        unit = (w == 1.0)                                # 가중치 1(팬틸트)은 기존 식을 그대로 쓴다 — 수치까지 같게
        kind = "target" if d.cls in self._target_cls else "object"
        nx, ny = nrm if nrm is not None and kind == "object" else (-math.cos(psi), -math.sin(psi))
        nn = math.hypot(nx, ny) or 1.
        cx, cy = (anchor.cx, anchor.cy) if anchor else (x-.2*nx/nn, y-.2*ny/nn)
        best, bd = None, pc.object_merge_dist
        for f in self.faces:
            if f.kind != kind:
                continue
            if anchor is not None and f.geometry_key == anchor.key:
                best = f
                break
            if f.geometry_key is not None:
                continue  # Distinct planes and size-only guesses never share an anchor.
            dd = math.hypot(f.x - x, f.y - y)
            if dd < bd:
                if nrm is not None and math.hypot(f.vx, f.vy) > 1e-6:
                    nf = math.hypot(f.vx, f.vy) or 1.0
                    if (f.vx * nrm[0] + f.vy * nrm[1]) / (nf * (math.hypot(*nrm) or 1.0)) < 0.5:
                        continue
                best, bd = f, dd
        if best is None:
            best = Face(x, y, cx, cy, kind=kind)
            self.faces.append(best)
        else:   # 위치를 이동평균으로 갱신
            k = 1.0 / (best.n + 1) if unit else w / (best.n + w)
            best.x += (x - best.x) * k
            best.y += (y - best.y) * k
            best.cx += (cx - best.cx) * k
            best.cy += (cy - best.cy) * k
        ux, uy = nrm if nrm is not None else (pose.x - x, pose.y - y)    # 면 법선(없으면 관측 방향)
        un = math.hypot(ux, uy) or 1.0
        k2 = 1.0 / (best.n + 1) if unit else w / (best.n + w)
        best.vx += (ux / un - best.vx) * k2
        best.vy += (uy / un - best.vy) * k2
        if anchor is not None:
            best.geometry_key = anchor.key
            best.x, best.y, best.cx, best.cy = anchor.x, anchor.y, anchor.cx, anchor.cy
            best.vx, best.vy = anchor.nx, anchor.ny
        conf_w = d.conf if unit else d.conf * w
        best.votes[d.cls] += conf_w
        best.n += 1 if unit else w
        best.last_t = t
        best.kind = kind
        best.src_n[source] = best.src_n.get(source, 0.0) + 1.0
        if source == "oak" and self.identify_mode:
            best.src_n["oak_id"] = best.src_n.get("oak_id", 0.0) + 1.0       # 식별 단계(미식별 면을 OAK 로 다시 봄)에서 쌓은 관측
        sv = best.src_votes.setdefault(source, {})
        sv[d.cls] = sv.get(d.cls, 0.0) + d.conf
        if source == "oak":
            self._trim_oak_window(best, d.cls, conf_w, w)
        self._note_disagreement(best, t)
        if source != "pantilt":                         # 시그널 LED 점등(M2 완료 판정)은 팬틸트 영상으로만 — OAK 영상으로 '타격 완료'를 만들지 않는다
            pass
        elif d.extra.get("led_on"):                       # E5: 점등은 연속 프레임으로 확인(오검출 한 번으로 '타격 완료'가 되면 재사격을 놓친다)
            best.led_votes += 1
            if best.led_votes >= self.cfg.event.led_confirm_frames:
                best.led_on = True
        elif "led_on" in d.extra and not best.led_on and best.led_votes > 0:
            best.led_votes -= 1
        if anchor is not None:
            self._adopt_unanchored_history(best)

    def _trim_oak_window(self, f: Face, cls: str, conf_w: float, w: float) -> None:
        """oak_det.vote_window(프레임)가 있으면 이 면에서 가장 최근 N개의 OAK 관측만 득표에 남긴다. None 이면 창이 없다(팬틸트와 같다)."""
        win = self.cfg.oak_det.vote_window
        if not win:
            return
        f.oak_obs.append([cls, conf_w, w])
        while len(f.oak_obs) > int(win):
            c0, v0, w0 = f.oak_obs.pop(0)
            f.votes[c0] = f.votes.get(c0, 0.0) - v0
            if f.votes[c0] <= 1e-12:
                f.votes.pop(c0, None)
            f.n -= w0

    def _note_disagreement(self, f: Face, t: float) -> None:
        """팬틸트와 OAK 가 각각 충분히 본 면인데 1위 클래스가 다르면 한 번만 기록한다(6-4: 팬틸트로 다시 본다 / 링 버퍼 저장 조건)."""
        pt, oak = f.src_votes.get("pantilt"), f.src_votes.get("oak")
        if not pt or not oak or f.src_n.get("pantilt", 0.0) < DISAGREE_MIN_OBS or f.src_n.get("oak", 0.0) < DISAGREE_MIN_OBS:
            return
        pc, oc = max(pt, key=pt.get), max(oak, key=oak.get)
        key = f"{pc}>{oc}"
        if pc == oc or key in f.disagreed:
            return
        f.disagreed.append(key)
        self.disagreements.append({"x": f.x, "y": f.y, "pantilt": pc, "oak": oc, "t": t})
        self.association_stats["oak_pantilt_disagree"] += 1

    def is_object_cls(self, cls: str) -> bool:
        """M3 인식 대상(표지 물체) 클래스인가 — 표적·거점 표지·비인식용 그림은 아니다."""
        return cls in self._object_cls

    def pop_disagreements(self) -> List[dict]:
        out = list(self.disagreements)
        self.disagreements.clear()
        return out

    def submittable(self, f: Face) -> bool:
        """M3 로 내보낼 수 있는 면인가: OAK 가 본 면은 팬틸트로도 한 번 이상 봤어야 한다. oak_det.submit_alone 이면 OAK 단독도 허용."""
        return (f.src_n.get("oak", 0.0) == 0.0 or f.src_n.get("pantilt", 0.0) >= 1.0
                or bool(self.cfg.oak_det.submit_alone)
                or ((bool(self.cfg.oak_det.identify_submit) or bool(self.cfg.oak_det.identify_primary))
                    and f.src_n.get("oak_id", 0.0) >= 1.0))

    def _adopt_unanchored_history(self, host: Face) -> None:
        """Move compatible pre-geometry votes into their measured plane once.

        Orthogonal faces and distant independent photos remain separate. A
        nearest-box/radius deletion would discard legitimate ground evidence.
        """
        keep = []
        for other in self.faces:
            if (other is host or other.geometry_key is not None or other.kind != host.kind
                    or math.hypot(other.x-host.x, other.y-host.y) >= .18
                    or math.hypot(other.cx-host.cx, other.cy-host.cy) >= .25
                    or (other.vx*host.vx+other.vy*host.vy)/(math.hypot(other.vx, other.vy) or 1.) <= .8):
                keep.append(other)
                continue
            for cls, weight in other.votes.items():
                host.votes[cls] += weight
            for src, cnt in other.src_n.items():
                host.src_n[src] = host.src_n.get(src, 0.0) + cnt
            for src, vv in other.src_votes.items():
                hv = host.src_votes.setdefault(src, {})
                for cls, weight in vv.items():
                    hv[cls] = hv.get(cls, 0.0) + weight
            host.n += other.n
            host.last_t = max(host.last_t, other.last_t)
            host.led_on |= other.led_on
            host.led_votes = max(host.led_votes, other.led_votes)
            self.association_stats["history_migrated"] += 1
        self.faces = keep

    def drop_near(self, cx: float, cy: float, r: float) -> int:
        """박스가 옮겨졌다고 판단했을 때 그 박스의 옛 면 관측(박스 중심 추정이 r 안)을 지운다. 지운 수를 돌려준다."""
        keep = [f for f in self.faces if math.hypot(f.cx - cx, f.cy - cy) >= r]
        n = len(self.faces) - len(keep)
        self.faces = keep
        return n

    def drop_box_faces(self, faces) -> int:
        """Invalidate only observed faces associated with a verified moved box.

        A radius around its estimated centre also erased neighbouring cubes.
        Use the same point/normal association as the coverage ledger instead.
        """
        keep = [of for of in self.faces if not any(
            math.hypot(of.x - f.cx, of.y - f.cy) < .25
            and (of.vx * f.nx + of.vy * f.ny) / (math.hypot(of.vx, of.vy) or 1.0) > .5
            for f in faces)]
        n = len(self.faces) - len(keep)
        self.faces = keep
        return n

    # ------------------------------------------------------------------ 질의
    def confirmed_faces(self) -> List[Face]:
        """관측이 충분한 면만, 그리고 '같은 면의 위치 이탈 관측'을 가장 강한 면에 흡수한 뒤 반환.
        흡수 조건: 거리 < 0.32 m 이고 관측 방향(면→로봇)이 비슷(내적 > 0.5).
        같은 박스의 인접 면은 중심 간격 0.28 m 이지만 법선이 90° 달라 내적≈0 이라 합쳐지지 않는다."""
        pc = self.cfg.perception
        cand = sorted([f for f in self.faces if f.n >= pc.min_votes and self.submittable(f)], key=lambda f: -f.n)
        kept: List[Face] = []
        for f in cand:
            nf = math.hypot(f.vx, f.vy) or 1.0
            host = None
            for k in kept:
                if f.kind != k.kind or (f.geometry_key is not None and k.geometry_key is not None
                                       and f.geometry_key != k.geometry_key):
                    continue
                nk = math.hypot(k.vx, k.vy) or 1.0
                if (math.hypot(f.x - k.x, f.y - k.y) < 0.32 and
                        (f.vx * k.vx + f.vy * k.vy) / (nf * nk) > 0.5):
                    host = k
                    break
            if host is None:
                kept.append(Face(f.x, f.y, f.cx, f.cy, votes=defaultdict(float, f.votes), n=f.n,
                                 kind=f.kind, led_on=f.led_on, led_votes=f.led_votes,
                                 last_t=f.last_t, vx=f.vx, vy=f.vy, geometry_key=f.geometry_key,
                                 src_n=dict(f.src_n), src_votes={k: dict(v) for k, v in f.src_votes.items()}))
            else:
                for c, v in f.votes.items():
                    host.votes[c] += v
                for src, cnt in f.src_n.items():
                    host.src_n[src] = host.src_n.get(src, 0.0) + cnt
                host.n += f.n
                host.led_on = host.led_on or f.led_on
        return [f for f in kept if f.mean_conf >= 0.45]

    def counts_by_sector(self) -> Dict[str, Dict[str, int]]:
        out: Dict[str, Dict[str, int]] = {}
        for f in self.confirmed_faces():
            if f.kind != "object":
                continue
            sec = self.geo.sector_of(f.cx, f.cy)
            if sec is None:
                continue
            out.setdefault(sec, {})
            out[sec][f.cls] = out[sec].get(f.cls, 0) + 1
        return out

    def find_target(self, cls: str, avoid_own_base: Optional[str] = None) -> List[Face]:
        return [f for f in self.faces if f.kind == "target" and f.cls == cls and f.n >= 2]

    def seen_bases(self, min_n: int = 3) -> Dict[str, str]:
        """{거점 이름: 표지 클래스} — 거점별 다수결(관측 min_n회 이상)."""
        out: Dict[str, str] = {}
        for b, votes in self.base_votes.items():
            cls = max(votes.items(), key=lambda kv: kv[1])[0]
            if self.base_n[b][cls] >= min_n:
                out[b] = cls
        return out
