"""경기 중 구조물 변경 시나리오(시뮬). run(..., events=[(t, shuffle_structures)]) 로 쓴다."""
from __future__ import annotations

import random

from .world import World


def shuffle_structures(world: World, seed: int = 7, n_blank: int = 2) -> None:
    """중앙부(x 2~W-2 m) 박스를 새 위치로 옮기고, 비인식용 박스를 n_blank 개 더 놓는다.
    박스 간 최소 간격 0.8 m(규정: 이격 40 cm 이상 + 박스 40 cm)를 지키고, 로봇 위에는 놓지 않는다."""
    rng = random.Random(seed + int(world.t))
    g = world.geo

    def free(x: float, y: float, skip: int = -1) -> bool:
        if any(max(abs(x - o.cx), abs(y - o.cy)) < 0.8 for j, o in enumerate(world.cubes) if j != skip):
            return False
        return all((x - u.pose.x) ** 2 + (y - u.pose.y) ** 2 > 0.6 ** 2 for u in world.ugvs)

    def draw() -> tuple:
        return (round(rng.uniform(2.2, g.W - 2.2) / 0.2) * 0.2, round(rng.uniform(0.6, g.H - 0.6) / 0.2) * 0.2)

    for idx, c in enumerate(list(world.cubes)):
        if 2.0 < c.cx < g.W - 2.0:
            for _ in range(200):
                x, y = draw()
                if free(x, y, skip=idx):
                    world.move_cube(idx, x, y)
                    break
    for _ in range(n_blank):
        for _ in range(200):
            x, y = draw()
            if free(x, y):
                world.add_blank_cube(x, y)
                break
