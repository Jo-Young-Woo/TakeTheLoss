"""관제 서버 연결 추상화.

사전교육 자료 기준 흐름(p6~8):
  서버 → 팀: 경기 시작 + 자연어 전술명령(0:00), 목표거점 전술명령(4:00), 정지(5:00)
  팀 → 서버: M1 해석 JSON, M3 객체인식 JSON, M6 위험도 JSON
  서버 → 팀: 고정 UAV 영상(기본 1 FPS, 임무 달성에 따라 2s/5s당 1프레임 → 중단)

실제 서버(「참가팀 단계별 API 안내」)에는 START/STOP 푸시가 없다 — link/http.py 가 /me phase 와 긴 폴링(/order, /target)을
이 인터페이스의 메시지로 바꿔 준다. JSON 변환은 link/schema.py. 임무 코드는 이 인터페이스만 쓴다.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class ServerMsg:
    kind: str                       # start | command | base_command | stop | event
    text: str = ""                  # 자연어 명령 본문(command/base_command)
    data: Dict[str, Any] = field(default_factory=dict)
    stamp: float = 0.0


@dataclass
class UavDet:
    """UAV 영상에서 검출된 UGV 하나(경기장 좌표 m)."""
    x: float
    y: float
    team: Optional[str] = None      # 안테나 색으로 구분되면 'RED'/'BLUE'
    ident: Optional[str] = None     # 식별 가능하면(예: 'R-UGV-2')


@dataclass
class UavBox:
    """Overhead footprint prior in arena metres, never an object classification.

    A top view can suggest where to inspect. Only UGV side observations may
    populate M3 counts. Dimensions include rectangular fixed obstacles.
    """
    cx: float
    cy: float
    sx: float
    sy: float
    confidence: float = 1.0


@dataclass
class UavFrame:
    stamp: float
    image: Optional[np.ndarray] = None            # 실기: BGR 영상
    detections: List[UavDet] = field(default_factory=list)  # 시뮬/후처리 결과
    interval_s: Optional[float] = None            # 실기: 서버가 알리는 새 장 간격 X-Frame-Interval-S (1.0 · 2.0 · 5.0)
    destroyed: Optional[int] = None               # 실기: 우리 진영 시설물 파괴 수 X-Destroyed (0~2; 3이면 403 으로 더는 못 받는다)
    jpeg: Optional[bytes] = None                  # 원본 JPEG. 지상 좌표·검출 결과로 간주하지 않는다.
    age_s: Optional[float] = None                 # 실기: 받은 순간의 촬영 후 경과(s). stamp 는 서버 벽시계 epoch 라 로봇 시계와 다르다 → Agent 가 age 로 로봇 시계 시각을 만든다
    recv_mono: Optional[float] = None             # 실기: 받은 순간의 time.monotonic() (Agent 가 받은 뒤 지난 시간을 age 에 더한다)
    camera: Optional[str] = None                  # X-Camera: main | test1 | test2
    test_mode: bool = False                       # X-Test-Mode == on
    camp: Optional[str] = None                    # Metadata only; /me remains the assignment authority.
    boxes: List[UavBox] = field(default_factory=list)
    layout_source: Optional[str] = None           # image-derived or explicitly synthetic footprint priors
    layout_stamp: Optional[float] = None          # original geometry capture time; can differ from UGV detections


class ServerLink(ABC):
    @abstractmethod
    def poll(self) -> List[ServerMsg]:
        """새로 도착한 서버 메시지들(없으면 [])."""

    @abstractmethod
    def submit(self, kind: str, payload: Dict[str, Any]) -> bool:
        """결과 제출. kind: 'm1' | 'm3' | 'm6'. 성공 시 True."""

    def submit_async(self, kind: str, payload: Dict[str, Any], token: str) -> None:
        """제어 루프를 막지 않는 제출 요청. 기본 구현은 submit()을 그 자리에서 호출하고 결과를 쌓아 둔다.
        네트워크가 느린 실기는 link/async_link.py 의 AsyncServerLink 로 감싸 별도 스레드에서 보낸다.
        token 은 같은 작업의 재전송에서 변하지 않는다(멱등 키로 쓸 수 있다)."""
        try:
            ok = bool(self.submit(kind, payload))
        except Exception:
            ok = False
        self.__dict__.setdefault("_async_results", []).append((token, ok))

    def submit_results(self) -> List[Tuple[str, bool]]:
        """submit_async 의 (token, 성공 여부) 중 아직 못 읽은 것."""
        out = self.__dict__.get("_async_results", [])
        self.__dict__["_async_results"] = []
        return out

    def submission_allowed(self, kind: str) -> Optional[bool]:
        """Known server-phase gate; None means the transport has no phase feed."""
        return None

    def close_submission(self, kind: str) -> None:
        """Cancel unsent transport work for a permanently closed mission kind."""
        return None

    def get_uav(self) -> Optional[UavFrame]:
        """가장 최근 UAV 프레임(없으면 None)."""
        return None

    def identity(self) -> Optional[Tuple[str, int]]:
        """(team 'BLUE'/'RED', slot 1..3). 서버가 알려주지 않으면 None."""
        return None

    def health(self) -> Tuple[bool, str]:
        """(연결 정상 여부, 문제 설명). 비동기 링크는 백그라운드 스레드의 오류를 여기로 알려 Agent 가 로그·통계로 남기게 한다.
        동기 링크의 오류는 호출 자리에서 예외로 드러나므로 기본은 항상 정상."""
        return True, ""
