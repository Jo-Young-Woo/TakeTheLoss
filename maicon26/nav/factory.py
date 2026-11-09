"""config.nav.backend 에 맞는 항법 스택을 만든다."""
from __future__ import annotations

from typing import Optional

from ..config import Config
from ..hal.base import Robot
from .stack import NavStack


def make_stack(cfg: Config, robot: Optional[Robot] = None, seed: int = 0) -> NavStack:
    b = cfg.nav.backend
    if b == "builtin":
        from .local_stack import LocalStack
        return LocalStack(cfg, seed=seed)
    if b == "nav2":
        from .nav2_stack import Nav2Stack
        from .ros_nav2 import RosNav2Client           # rclpy 는 여기서만(로봇 측) import 된다
        return Nav2Stack(cfg, RosNav2Client(cfg), seed=seed)
    if b == "nav2-fake":
        from ..sim.fake_nav2 import FakeNav2Client
        from ..sim.robot import SimRobot
        from .nav2_stack import Nav2Stack
        if not isinstance(robot, SimRobot):
            raise ValueError("nav2-fake 는 시뮬 로봇(SimRobot)에서만 쓴다")
        return Nav2Stack(cfg, FakeNav2Client(robot, cfg, seed=seed), seed=seed)
    raise ValueError(f"알 수 없는 nav.backend: {b!r} (builtin | nav2 | nav2-fake)")
