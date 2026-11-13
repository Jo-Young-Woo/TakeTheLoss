"""느린 서버 호출을 제어 루프 밖으로 빼는 래퍼 (R02).

세 스레드를 따로 둬서 한쪽이 막혀도 다른 쪽이 영향받지 않게 한다.
  poller    inner.poll()      — STOP·명령·이벤트 경로. 제출/영상이 느려도 STOP 수신이 늦어지지 않는다.
  uav       inner.get_uav()   — 영상 수신
  submitter inner.submit()    — 제출. 결과는 submit_results() 로 돌려준다.
inner 가 예외를 던져도 스레드는 죽지 않는다(last_error 에 남기고 잠시 쉰 뒤 계속).
"""
from __future__ import annotations

import queue
import copy
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from .base import ServerLink, ServerMsg, UavFrame
from .secrets import REDACT


class AsyncServerLink(ServerLink):
    def __init__(self, inner: ServerLink, poll_hz: float = 20.0, uav_hz: float = 5.0) -> None:
        self.inner = inner
        self._msgs: "queue.Queue[ServerMsg]" = queue.Queue()
        self._jobs: "queue.Queue[Tuple[str, Dict[str, Any], str]]" = queue.Queue()
        self._res: "queue.Queue[Tuple[str, bool]]" = queue.Queue()
        self._lock = threading.Lock()
        self._closed_submissions: set = set()
        self._queued_tokens: set = set()
        self._uav: Optional[UavFrame] = None
        self._ident: Optional[Tuple[str, int]] = None
        self.last_error: Optional[str] = None
        self._t_create = time.monotonic()
        self._last_poll_ok: Optional[float] = None       # 마지막으로 poll 이 예외 없이 끝난 시각
        self._poll_error: Optional[str] = None           # 가장 최근 poll 예외(성공하면 지움)
        self._submit_error: Optional[str] = None
        self.stale_s = 3.0                               # 이 시간 넘게 poll 성공이 없으면 연결 이상으로 본다
        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._loop, args=(self._poll_once, 1.0 / poll_hz), daemon=True),
            threading.Thread(target=self._loop, args=(self._uav_once, 1.0 / uav_hz), daemon=True),
            threading.Thread(target=self._submit_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    # ------------------------------------------------------------------ 스레드 본체
    def _loop(self, fn, period: float) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                fn()
            except Exception as e:                       # 예외가 경로를 영구히 죽이면 안 된다
                self.last_error = REDACT(f"{type(e).__name__}: {e}")
                self._stop.wait(0.5)
            self._stop.wait(max(period - (time.monotonic() - t0), 0.0))

    def _poll_once(self) -> None:
        try:
            msgs = self.inner.poll()
        except Exception as e:
            self._poll_error = REDACT(f"poll {type(e).__name__}: {e}")
            raise
        for m in msgs:
            self._msgs.put(m)
        self._last_poll_ok, self._poll_error = time.monotonic(), None

    def _uav_once(self) -> None:
        f = self.inner.get_uav()
        if f is not None:
            with self._lock:
                self._uav = f

    def _submit_loop(self) -> None:
        while not self._stop.is_set():
            try:
                kind, payload, token = self._jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            with self._lock:
                closed = kind in self._closed_submissions
            try:
                ok = False if closed else bool(self.inner.submit(kind, payload))
                self._submit_error = None
            except Exception as e:
                self.last_error = self._submit_error = REDACT(f"submit {type(e).__name__}: {e}")
                ok = False
            with self._lock:
                self._queued_tokens.discard(token)
            self._res.put((token, ok))

    # ------------------------------------------------------------------ ServerLink
    def poll(self) -> List[ServerMsg]:
        out: List[ServerMsg] = []
        while True:
            try:
                out.append(self._msgs.get_nowait())
            except queue.Empty:
                return out

    def submit(self, kind: str, payload: Dict[str, Any]) -> bool:
        """동기 제출(호환용). 제어 루프에서는 submit_async 를 쓴다."""
        self.submit_async(kind, payload, f"sync-{time.monotonic()}")
        return True

    def submit_async(self, kind: str, payload: Dict[str, Any], token: str) -> None:
        with self._lock:
            if kind in self._closed_submissions:
                self._res.put((token, False))
                return
            if token in self._queued_tokens:
                return  # An ACK timeout must not pile up duplicate HTTP requests.
            self._queued_tokens.add(token)
        self._jobs.put((kind, copy.deepcopy(payload), token))

    def submission_allowed(self, kind: str) -> Optional[bool]:
        with self._lock:
            if kind in self._closed_submissions:
                return False
        return self.inner.submission_allowed(kind)

    def close_submission(self, kind: str) -> None:
        with self._lock:
            self._closed_submissions.add(kind)
        self.inner.close_submission(kind)

    def submit_results(self) -> List[Tuple[str, bool]]:
        out: List[Tuple[str, bool]] = []
        while True:
            try:
                out.append(self._res.get_nowait())
            except queue.Empty:
                return out

    def get_uav(self) -> Optional[UavFrame]:
        with self._lock:
            return self._uav

    def identity(self) -> Optional[Tuple[str, int]]:
        self._ident = self.inner.identity()            # 편성이 철회되면 이전 팀/자리를 유지하지 않는다
        return self._ident

    def health(self) -> Tuple[bool, str]:
        """poll(STOP·명령 경로)과 제출 스레드의 상태. 예외 문구를 그대로 올려 원인을 알 수 있게 한다."""
        now = time.monotonic()
        if self._poll_error is not None and (self._last_poll_ok is None or now - self._last_poll_ok > self.stale_s):
            return False, self._poll_error
        if self._last_poll_ok is None and now - self._t_create > self.stale_s:
            return False, "poll 응답 없음(시작 후 %.0f s)" % (now - self._t_create)
        if self._last_poll_ok is not None and now - self._last_poll_ok > self.stale_s:
            return False, "poll 이 %.0f s 동안 성공하지 못함" % (now - self._last_poll_ok)
        if self._submit_error is not None:
            return False, self._submit_error
        try:
            return self.inner.health()                   # 안쪽 링크가 따로 스레드를 돌리면(HttpServerLink) 그 상태도 반영한다
        except Exception:
            return True, ""

    def ready_handshake(self, *a, **k) -> bool:
        """준비완료 핸드셰이크는 안쪽 링크가 지원할 때만(HttpServerLink). 지원하지 않으면(시뮬) 바로 True."""
        fn = getattr(self.inner, "ready_handshake", None)
        return bool(fn(*a, **k)) if fn is not None else True

    def close(self) -> None:
        self._stop.set()
        try:
            getattr(self.inner, "close", lambda: None)()
        except Exception:
            pass
        for t in self._threads:
            t.join(timeout=1.0)
