"""하드웨어 추상화(HAL). 임무 코드는 이 인터페이스만 본다.

실기(WINGBOT-UGV)는 hal/wingbot.py, 시뮬레이터는 sim/robot.py 가 구현한다.
실기 ROS2 API(2차 사전교육 PDF)는 wingbot.py(로직) + ros_mil.py(rclpy 전송)로 구현했다 — 로봇에서는 미검증.

단위: 거리 m, 속도 m/s, 각도 rad(차체) / deg(팬·틸트), 시간 s.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Tuple

from ..types import DepthFrame, Frame, Pose2D, RobotStatus, Scan


class Robot(ABC):
    # 진행 중인 5초 발광을 즉시 끌 수 있는가. 실기 API 확인 전에는 False — 그러면 아군 사선 판단을 더 보수적으로 한다.
    can_abort_laser: bool = False

    # ------------------------------------------------------------------ 시간
    @abstractmethod
    def now(self) -> float:
        """단조 증가 시각(s). 실기는 time.monotonic(), 시뮬은 시뮬 시계."""

    # ------------------------------------------------------------------ 구동
    @abstractmethod
    def set_velocity(self, v: float, w: float) -> None:
        """차체 속도 명령. v: 전진 +, w: 반시계(좌회전) +. 다음 호출 전까지 유지."""

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    @abstractmethod
    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        """팬(좌 +)/틸트(위 +) 절대각 명령."""

    @abstractmethod
    def get_pan_tilt(self) -> Tuple[float, float]:
        """현재 팬/틸트 (deg). 가능하면 서보 피드백, 없으면 마지막 명령값."""

    # ------------------------------------------------------------------ 레이저
    @abstractmethod
    def laser_fire(self) -> bool:
        """발사 요청(5초 발광). 탄약 없음/재장전 중/피격 정지 중이면 False."""

    def laser_abort(self) -> None:
        """진행 중인 발광을 즉시 끈다(지원하지 않으면 무시)."""

    # ------------------------------------------------------------------ 센서
    @abstractmethod
    def get_scan(self) -> Optional[Scan]:
        """가장 최근 LiDAR 스캔(없으면 None). 같은 스캔을 다시 줄 수 있으니 stamp로 구분.
        **stamp 는 now() 와 같은 시계여야 한다**(촬영 시각의 자세와 짝짓는 데 쓴다). ROS header.stamp 가 에포크/센서 자체 시계면
        hal/clock.py 의 StampSync 로 변환해서 채운다 — 그냥 넣으면 Agent 가 감지해 변환하지만 최소 지연만큼 오차가 남는다."""

    @abstractmethod
    def get_frame(self) -> Optional[Frame]:
        """가장 최근 카메라 프레임. stamp(촬영 시각)와 pan/tilt(촬영 시점 값)는 get_scan 과 같은 시계 계약을 따른다."""

    def get_depth(self) -> Optional[DepthFrame]:
        """가장 최근 깊이 영상(없거나 지원하지 않으면 None). stamp 와 pan/tilt 는 get_frame 과 같은 계약.
        3D 지도는 이게 None 이어도 정상 동작한다(LiDAR 2D 만 쓴다) — 선택 센서다."""
        return None

    @abstractmethod
    def get_odom(self) -> Pose2D:
        """오도메트리 자세. 시작 시점 기준(0,0,0)에서 누적. 월드 좌표가 아님에 주의."""

    @abstractmethod
    def status(self) -> RobotStatus:
        """탄약·재장전·피격 상태 등."""

    # ------------------------------------------------------------------ 저하 모드(선택, 실기 HAL 만 구현)
    def get_oak_frame(self) -> Optional[Frame]:
        """OAK-D Lite 컬러 카메라의 가장 최근 프레임(6장 보조 탐지). pan/tilt 는 0 — 차체에 고정된 카메라다. 없으면 None(기본)."""
        return None

    def get_oak_info(self) -> Optional[tuple]:
        """OAK 컬러 CameraInfo 에서 읽은 (K 9개, 폭, 높이). 없으면 None(기본)."""
        return None

    def oak_power(self, on: bool) -> str:
        """OAK 카메라를 켜고(True)/끈다(False) — 드라이버의 start/stop 서비스(oak.power_gating). 호출은 막지 않는다(비동기).
        돌려주는 값: "ok"=이 명령이 확인됐다 · "pending"=응답을 기다리는 중(다음 틱에 같은 인자로 다시 부르면 결과를 준다) · "fail"=실패 · "none"=지원 안 함/서비스가 안 보임(기본). 안 보이면 호출자는 OAK 를 켜 둔 채로 쓴다."""
        return "none"

    def is_degraded(self, flag: str) -> bool:
        """preflight 저하 플래그(no_drive·no_vision·no_fire)가 걸려 있는가. 시뮬 로봇은 늘 False."""
        return False

    def degraded_flags(self) -> frozenset:
        return frozenset()

    # ------------------------------------------------------------------ 출력(선택)
    def show(self, text: str) -> None:
        """OLED 등 표시(선택)."""

    def close(self) -> None:
        self.stop()
