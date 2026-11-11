"""rclpy 한 번만 초기화하고 하나의 실행기(스레드)로 여러 노드를 돌린다.

RosWingbot(센서/구동)과 RosNav2Client(SLAM/Nav2)가 둘 다 노드를 만들기 때문에 init()이 두 번 불리면
오류가 난다 — 그래서 공용으로 둔다. ROS가 없는 PC(시뮬)에서는 이 모듈을 import 해도 rclpy를 건드리지 않는다.
"""
from __future__ import annotations

import threading
from typing import Any, Optional, Tuple

_lock = threading.Lock()
_executor: Optional[Any] = None
_thread: Optional[threading.Thread] = None


def ensure_ros() -> Tuple[Any, Any]:
    """(rclpy 모듈, MultiThreadedExecutor). 처음 부를 때 init + 스레드 시작."""
    global _executor, _thread
    import rclpy  # type: ignore
    from rclpy.executors import MultiThreadedExecutor  # type: ignore
    with _lock:
        if not rclpy.ok():
            rclpy.init()
        if _executor is None:
            _executor = MultiThreadedExecutor(num_threads=4)
            _thread = threading.Thread(target=_executor.spin, name="rclpy-spin", daemon=True)
            _thread.start()
    return rclpy, _executor


def add_node(node: Any) -> None:
    _, ex = ensure_ros()
    ex.add_node(node)


def shutdown() -> None:
    """실행기 → 스레드 join → rclpy 순서로 끈다(순서를 어기면 종료 시 'rclpy-spin' 스레드 예외/abort 가 난다)."""
    global _executor, _thread
    with _lock:
        try:
            import rclpy  # type: ignore
            if _executor is not None:
                _executor.shutdown()
            if _thread is not None:
                _thread.join(timeout=3.0)
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        _executor = _thread = None
