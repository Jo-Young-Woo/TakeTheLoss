"""경기장 기하. 1차 사전교육 자료(p23~24) 도면에서 읽은 근사값이다.

!! 수치는 모두 '도면 근사'다. 사전 테스트 경기장에서 실측해 config의 arena 항목으로 덮어쓸 것.

좌표계: 원점 = 경기장 왼쪽 아래 모서리, x = 오른쪽(0..W), y = 위쪽(0..H), θ 반시계 +.
BLUE 진영 = 왼쪽(x 작은 쪽), RED 진영 = 오른쪽. 도면 위쪽이 y 큰 쪽이다(B-1이 위, B-3이 아래).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Rect = Tuple[float, float, float, float]  # x0, y0, x1, y1


@dataclass
class ArenaGeometry:
    W: float = 6.12
    H: float = 5.00
    wall_h: float = 0.40
    box_size: float = 0.40
    base_depth: float = 0.80       # 거점 영역의 x 방향 깊이(도면 근사)
    base_height: float = 0.90      # 거점 영역의 y 방향 길이(도면 근사; 규격서는 1200x400)
    common_half: float = 0.615     # COMMON 띠의 반폭(도면 근사: x 2.46~3.69)
    base_cy: Tuple[float, float, float] = (4.16, 2.50, 0.83)  # 1,2,3번 거점 중심 y
    min_gap: float = 0.40          # [PDF p24] 장애물(박스) 사이 최소 이격
    validate_layout: bool = True   # 무작위 배치를 만든 직후 정적 유효성을 검사하고, 통과 못 하면 다시 만든다(sim/arena_check.py)
    max_layout_tries: int = 30     # 재생성 상한. 끝내 못 만들면 ArenaGenerationError(제외 사유 포함)
    # YAML-driven arena uses a floor-centred Gazebo world. Navigation keeps its
    # existing nonnegative raster frame through this explicit, reversible offset.
    world_offset: Tuple[float, float] = (0.0, 0.0)
    measured_sectors: Dict[str, Rect] = field(default_factory=dict)
    measured_bases: Dict[str, Rect] = field(default_factory=dict)
    measured_starts: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)

    @classmethod
    def from_spec(cls, spec):
        """Build every measured bound from arena_spec.yaml; no layout truth input."""
        width, height = map(float, spec["floor"]["size"])
        def translated(rect):
            xmin, xmax, ymin, ymax = map(float, rect)
            return (xmin+width/2, ymin+height/2, xmax+width/2, ymax+height/2)
        sectors = {("COMMON" if key == "CM" else key): translated(value)
                   for key, value in spec["sectors"].items()}
        bases = {key: translated(value) for key, value in spec["bases"]["printed_outline"].items()}
        starts = {key: (float(value[0])+width/2, float(value[1])+height/2, float(value[2]))
                  for key, value in spec["bases"]["start_pose"].items()}
        blue_band = spec["non_sector"]["base_band_blue"]
        return cls(W=width, H=height, wall_h=float(spec["walls"]["height"]),
                   box_size=float(spec["cubes"]["size"]),
                   min_gap=float(spec["cubes"]["min_clearance"]),
                   base_depth=float(blue_band[1]-blue_band[0]),
                   base_height=bases["B-2"][3]-bases["B-2"][1],
                   base_cy=tuple(starts[f"B-{row}"][1] for row in (1, 2, 3)),
                   common_half=(sectors["COMMON"][2]-sectors["COMMON"][0])/2,
                   world_offset=(-width/2, -height/2), measured_sectors=sectors,
                   measured_bases=bases, measured_starts=starts)

    def world_to_nav(self, x, y):
        return x-self.world_offset[0], y-self.world_offset[1]

    def nav_to_world(self, x, y):
        return x+self.world_offset[0], y+self.world_offset[1]

    # ---------------------------------------------------------------- 거점
    def base_rect(self, name: str) -> Rect:
        """name: 'B-1'..'B-3' / 'R-1'..'R-3'."""
        if name in self.measured_bases:
            return tuple(self.measured_bases[name])
        side, idx = name.split("-")
        cy = self.base_cy[int(idx) - 1]
        hh = self.base_height / 2
        if side == "B":
            return (0.0, cy - hh, self.base_depth, cy + hh)
        return (self.W - self.base_depth, cy - hh, self.W, cy + hh)

    def base_center(self, name: str) -> Tuple[float, float]:
        x0, y0, x1, y1 = self.base_rect(name)
        return (0.5 * (x0 + x1), 0.5 * (y0 + y1))

    def base_entry(self, name: str) -> Tuple[float, float]:
        """거점 앞 진입 지점(경기장 안쪽 방향으로 0.35 m)."""
        x0, y0, x1, y1 = self.base_rect(name)
        cy = 0.5 * (y0 + y1)
        return ((x1 + 0.35) if name.startswith("B") else (x0 - 0.35), cy)

    def bases(self, side: str) -> List[str]:
        p = "B" if side.upper() == "BLUE" else "R"
        return [f"{p}-{i}" for i in (1, 2, 3)]

    # ---------------------------------------------------------------- 섹터
    def sector_rect(self, name: str) -> Rect:
        """'B-S1'..'B-S3', 'R-S1'..'R-S3', 'COMMON'."""
        if name == "CM":
            name = "COMMON"
        if name in self.measured_sectors:
            return tuple(self.measured_sectors[name])
        W, H = self.W, self.H
        c0, c1 = W / 2 - self.common_half, W / 2 + self.common_half
        if name == "COMMON":
            return (c0, 0.0, c1, H)
        side, s = name.split("-S")
        row = int(s) - 1                      # 0 = 위(y 큰 쪽)
        y1 = H - row * H / 3
        y0 = y1 - H / 3
        if side == "B":
            return (self.base_depth, y0, c0, y1)
        return (c1, y0, W - self.base_depth, y1)

    def sector_of(self, x: float, y: float) -> Optional[str]:
        for n in ("COMMON", "B-S1", "B-S2", "B-S3", "R-S1", "R-S2", "R-S3"):
            x0, y0, x1, y1 = self.sector_rect(n)
            if x0 <= x < x1 and y0 <= y < y1:
                return n
        return None  # 거점 영역 등

    def sector_names(self, side: Optional[str] = None) -> List[str]:
        if side is None:
            return ["B-S1", "B-S2", "B-S3", "COMMON", "R-S1", "R-S2", "R-S3"]
        p = "B" if side.upper() == "BLUE" else "R"
        return [f"{p}-S{i}" for i in (1, 2, 3)]

    def sector_center(self, name: str) -> Tuple[float, float]:
        x0, y0, x1, y1 = self.sector_rect(name)
        return (0.5 * (x0 + x1), 0.5 * (y0 + y1))

    # ---------------------------------------------------------------- 진영
    @staticmethod
    def opponent(team: str) -> str:
        return "RED" if team.upper() == "BLUE" else "BLUE"

    def in_base(self, x: float, y: float, name: str, margin: float = 0.0) -> bool:
        x0, y0, x1, y1 = self.base_rect(name)
        return (x0 - margin) <= x <= (x1 + margin) and (y0 - margin) <= y <= (y1 + margin)

    def wall_distance(self, x: float, y: float) -> float:
        """가장 가까운 외벽까지 거리(박스 무시)."""
        return min(x, self.W - x, y, self.H - y)

    def inside(self, x: float, y: float, margin: float = 0.0) -> bool:
        return margin <= x <= self.W - margin and margin <= y <= self.H - margin


def mirror_for_team(geo: ArenaGeometry, team: str, name: str) -> str:
    """내 진영 기준 이름(예: 'own-1')을 팀에 맞는 실제 이름으로 바꾸는 도우미."""
    p = "B" if team.upper() == "BLUE" else "R"
    return f"{p}-{name.split('-')[-1]}"
