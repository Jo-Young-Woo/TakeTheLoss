"""확인용 표본 링 버퍼 (6-5): 사람이 볼 영상만 조금 남긴다. 탐지는 메모리에서 끝난다 — 디스크에는 최대 save_max(0–10)장.

  - 최대 장수 save_max: 0–10. 10 을 넘으면 10 으로 자르고 경고, 음수는 0. 0 이면 저장하지 않는다.
  - 저장할 때마다 오래된 것부터 지워 장수를 지키고, 그와 별도로 cleanup_every_s 마다 정리한다. 시작할 때 이전 실행의 파일(우리 이름 패턴)을 지운다.
  - 같은 장면이 연달아 쌓이지 않게 최소 간격 save_every_s (운영자 요청 force=True 는 예외).
  - 형식: 폭 640 JPEG(품질 80) + 같은 이름의 JSON(상자·클래스·신뢰도·출처·촬영 시각·로봇 자세, 상자는 저장된 JPEG 픽셀 좌표).
  - 쓰기·지우기·JPEG 인코딩은 별도 스레드. submit() 은 큐에 넣고 바로 돌아온다 — 제어 루프를 막지 않는다. 디스크 오류는 경고만(예외 없음).
  - 저장 위치는 기본적으로 저장소 밖 실행 로그 폴더(validate.run_log_dir()/oak_samples). 저장소 안이면 ensure_gitignored 가 .gitignore 에 넣는다.
field/experiments.py 의 현장 실험 저장(일부러 모으는 데이터)과는 별개다 — 그쪽은 바꾸지 않는다.
"""
from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

import numpy as np

SAVE_MAX_LIMIT = 10
NAME_RE = re.compile(r"^oak_(\d{6})_(\d+)\.(jpg|json)$")      # 우리가 만든 파일만 지우고 세는 이름 패턴
TRIGGER_NAME = "TRIGGER"                                     # 운영자 요청: 이 이름의 파일을 만들면 다음 OAK 프레임을 저장한다


def clamp_save_max(value, report: Callable[[str], None] = print) -> int:
    try:
        n = int(value)
    except (TypeError, ValueError):
        report(f"[ring] WARN oak_det.save_max={value!r} 를 읽을 수 없어 0(저장 안 함)으로 둔다")
        return 0
    if n > SAVE_MAX_LIMIT:
        report(f"[ring] WARN oak_det.save_max={n} 는 최대 {SAVE_MAX_LIMIT} 를 넘어 {SAVE_MAX_LIMIT} 로 자른다(Jetson 저장 공간)")
        return SAVE_MAX_LIMIT
    if n < 0:
        report(f"[ring] WARN oak_det.save_max={n} 는 음수라 0(저장 안 함)으로 둔다")
        return 0
    return n


def ensure_gitignored(save_dir, root, report: Callable[[str], None] = print) -> bool:
    """save_dir 이 저장소(root) 안이면 root/.gitignore 에 그 상대 경로를 넣는다. 넣었으면 True. 어떤 실패도 경고만."""
    try:
        d, r = Path(save_dir).expanduser().resolve(), Path(root).resolve()
        rel = d.relative_to(r).as_posix()
    except (ValueError, OSError):
        return False                                           # 저장소 밖 — 할 일이 없다
    line = rel.rstrip("/") + "/"
    try:
        gi = r / ".gitignore"
        text = gi.read_text(encoding="utf-8") if gi.exists() else ""
        if line in {l.strip() for l in text.splitlines()}:
            return False
        gi.write_text(text + ("" if not text or text.endswith("\n") else "\n") + line + "\n", encoding="utf-8")
        report(f"[ring] 저장 폴더가 저장소 안이라 .gitignore 에 {line} 를 넣었다(경기장 영상은 저장소에 남기지 않는다)")
        return True
    except OSError as e:
        report(f"[ring] WARN 저장 폴더가 저장소 안인데 .gitignore 를 고치지 못했다({type(e).__name__}) — 직접 {line} 를 넣을 것")
        return False


class RingStore:
    def __init__(self, save_dir, save_max=8, save_every_s: float = 2.0, cleanup_every_s: float = 30.0, *,
                 clock: Callable[[], float] = time.monotonic, wall: Callable[[], float] = time.time,
                 report: Callable[[str], None] = print, start_thread: bool = True,
                 jpeg_width: int = 640, jpeg_quality: int = 80) -> None:
        self.dir = Path(save_dir).expanduser()
        self.report = report
        self.max = clamp_save_max(save_max, report)
        self.every = max(float(save_every_s), 0.0)
        self.cleanup_every = max(float(cleanup_every_s), 0.1)
        self._clock, self._wall = clock, wall
        self.jpeg_width, self.jpeg_quality = int(jpeg_width), int(jpeg_quality)
        self._last_save = -1e9
        self._last_cleanup = clock()
        self._last_warn = -1e9
        self._seq = 0
        self._q: "queue.Queue" = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._th: Optional[threading.Thread] = None
        self.stats: Dict[str, Any] = {"submitted": 0, "saved": 0, "deleted": 0, "skipped_interval": 0, "skipped_busy": 0,
                                      "errors": 0, "last_error": None}
        if self.max <= 0:
            return
        self.clear_previous()
        if start_thread:
            self._th = threading.Thread(target=self._run, name="ring_store", daemon=True)
            self._th.start()

    @property
    def enabled(self) -> bool:
        return self.max > 0

    # ------------------------------------------------------------------ 제어 루프 쪽
    def submit(self, image: Optional[np.ndarray], meta: Dict[str, Any], *, force: bool = False) -> bool:
        """표본 하나를 큐에 넣는다(JPEG 인코딩·쓰기는 쓰기 스레드가 한다). 저장 대상이면 True."""
        if not self.enabled or image is None:
            return False
        now = self._clock()
        if not force and now - self._last_save < self.every:
            self.stats["skipped_interval"] += 1
            return False
        self._seq += 1
        item = (self._seq, int(self._wall() * 1000), np.asarray(image), dict(meta))
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self.stats["skipped_busy"] += 1                   # 디스크가 느리다 — 제어 루프는 기다리지 않는다
            return False
        self._last_save = now
        self.stats["submitted"] += 1
        return True

    def flush(self) -> None:
        """(시험·종료용) 큐에 쌓인 표본을 지금 이 스레드에서 쓰고 정리한다. 쓰기 스레드가 없을 때만 의미가 있다."""
        while True:
            try:
                item = self._q.get_nowait()
            except queue.Empty:
                break
            self._write(*item)
        self.cleanup()

    def close(self) -> None:
        self._stop.set()
        if self._th is not None and self._th.is_alive() and threading.current_thread() is not self._th:
            self._th.join(timeout=5.0)
        try:
            self.flush()
        except Exception:                                      # pragma: no cover
            pass

    # ------------------------------------------------------------------ 쓰기 스레드
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                item = None
            if item is not None:
                self._write(*item)
            if self._clock() - self._last_cleanup >= self.cleanup_every:
                self.cleanup()

    def _warn(self, what: str, e: BaseException) -> None:
        self.stats["errors"] += 1
        self.stats["last_error"] = f"{what}: {type(e).__name__}: {e}"
        now = self._clock()
        if now - self._last_warn >= 5.0:
            self._last_warn = now
            self.report(f"[ring] WARN {what} 실패({type(e).__name__}: {e}) — 표본 저장을 건너뛴다(탐지·제어에는 영향 없음)")

    def _write(self, seq: int, ms: int, image: np.ndarray, meta: Dict[str, Any]) -> None:
        try:
            import cv2
            h, w = image.shape[:2]
            scale = self.jpeg_width / float(w) if w > self.jpeg_width else 1.0
            small = cv2.resize(image, (int(round(w * scale)), int(round(h * scale))), interpolation=cv2.INTER_AREA) if scale != 1.0 else image
            ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                raise RuntimeError("JPEG 인코딩 실패")
            doc = dict(meta)
            doc["image_size"] = [int(small.shape[1]), int(small.shape[0])]
            doc["scale"] = scale
            doc["boxes"] = [{**b, "box": [float(v) * scale for v in b["box"]]} if "box" in b else b for b in doc.get("boxes", [])]
            doc["jpeg_quality"] = self.jpeg_quality
            stem = f"oak_{seq:06d}_{ms}"
            self.dir.mkdir(parents=True, exist_ok=True)
            tmp = self.dir / (stem + ".jpg.tmp")
            tmp.write_bytes(buf.tobytes())
            os.replace(tmp, self.dir / (stem + ".jpg"))
            (self.dir / (stem + ".json")).write_text(json.dumps(doc, ensure_ascii=False, default=float), encoding="utf-8")
            self.stats["saved"] += 1
        except Exception as e:                                 # 디스크 가득 참·권한·인코딩 오류 — 경고만
            self._warn("표본 쓰기", e)
            return
        self.enforce_max()

    # ------------------------------------------------------------------ 정리
    def _stems(self) -> List[str]:
        """우리 이름 패턴의 표본 묶음(stem)을 오래된 순서(일련번호)로."""
        stems = set()
        try:
            for p in self.dir.iterdir():
                m = NAME_RE.match(p.name)
                if m:
                    stems.add(p.name.rsplit(".", 1)[0])
        except OSError:
            return []
        return sorted(stems)

    def _remove(self, stem: str) -> None:
        for ext in (".jpg", ".json"):
            try:
                (self.dir / (stem + ext)).unlink()
            except FileNotFoundError:
                pass
            except OSError as e:
                self._warn("표본 삭제", e)

    def enforce_max(self) -> int:
        """장수가 save_max 를 넘으면 오래된 것부터 지운다. 지운 수."""
        stems = self._stems()
        extra = max(len(stems) - self.max, 0)
        for stem in stems[:extra]:
            self._remove(stem)
        self.stats["deleted"] += extra
        return extra

    def cleanup(self) -> None:
        """주기 정리: 장수 유지 + 짝이 없는 파일·임시 파일 제거."""
        self._last_cleanup = self._clock()
        try:
            for p in list(self.dir.iterdir()):
                if p.name.endswith(".tmp") and p.name.startswith("oak_"):
                    p.unlink()
            for stem in self._stems():
                if not ((self.dir / (stem + ".jpg")).exists() and (self.dir / (stem + ".json")).exists()):
                    self._remove(stem)                          # 쓰다 만 표본
        except FileNotFoundError:
            pass
        except OSError as e:
            self._warn("표본 폴더 정리", e)
        self.enforce_max()

    def clear_previous(self) -> int:
        """시작할 때 이전 실행의 우리 파일을 지운다(다른 이름의 파일은 건드리지 않는다)."""
        n = 0
        try:
            for p in list(self.dir.iterdir()):
                if NAME_RE.match(p.name) or (p.name.startswith("oak_") and p.name.endswith(".tmp")):
                    p.unlink()
                    n += 1
        except FileNotFoundError:
            pass
        except OSError as e:
            self._warn("이전 표본 삭제", e)
        return n

    # ------------------------------------------------------------------ 운영자 요청
    def consume_trigger(self) -> bool:
        """저장 폴더의 TRIGGER 파일이 있으면 지우고 True — 다음 표본을 최소 간격과 상관없이 저장하라는 요청."""
        if not self.enabled:
            return False
        p = self.dir / TRIGGER_NAME
        try:
            if p.exists():
                p.unlink()
                return True
        except OSError:
            pass
        return False
