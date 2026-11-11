"""6장 6-5: 확인용 표본 링 버퍼. 디스크에는 최대 save_max(0–10)장. 쓰기 스레드는 쓰지 않고(start_thread=False) flush 로 직접 쓴다."""
import json
from pathlib import Path

import numpy as np
import pytest

from maicon26.ring_store import NAME_RE, SAVE_MAX_LIMIT, TRIGGER_NAME, RingStore, clamp_save_max, ensure_gitignored


class Tick:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def img(w=1280, h=720, v=0):
    return np.full((h, w, 3), v, np.uint8)


def meta(i=0):
    return {"why": "new_class", "source": "oak", "stamp": float(i), "pose": {"x": 1.0, "y": 2.0, "th": 0.5},
            "boxes": [{"cls": "TANK", "conf": 0.8, "box": [100.0, 200.0, 300.0, 400.0], "source": "oak"}]}


def make(tmp_path, **kw):
    msgs = []
    clk = Tick()
    args = dict(save_max=8, save_every_s=2.0, cleanup_every_s=30.0, clock=clk, report=msgs.append, start_thread=False)
    args.update(kw)
    rs = RingStore(tmp_path / "ring", **args)
    return rs, clk, msgs


def files(tmp_path):
    d = tmp_path / "ring"
    return sorted(p.name for p in d.iterdir()) if d.exists() else []


def save_many(rs, clk, n, step=3.0):
    for i in range(n):
        clk.t += step
        assert rs.submit(img(v=i % 250), meta(i))
        rs.flush()


# ---------------------------------------------------------------------- 장수 한도
def test_fifty_saves_never_leave_more_than_save_max_files(tmp_path):
    rs, clk, msgs = make(tmp_path, save_max=8)
    save_many(rs, clk, 50)
    names = files(tmp_path)
    assert len([n for n in names if n.endswith(".jpg")]) == 8 and len([n for n in names if n.endswith(".json")]) == 8
    assert all(NAME_RE.match(n) for n in names)
    seqs = sorted(int(NAME_RE.match(n).group(1)) for n in names if n.endswith(".jpg"))
    assert seqs == list(range(43, 51))                                        # 오래된 것부터 지워 최근 8장이 남는다


def test_limit_is_checked_on_every_write_not_only_by_the_periodic_cleanup(tmp_path):
    rs, clk, _ = make(tmp_path, save_max=3, cleanup_every_s=10_000.0)
    for i in range(10):
        clk.t += 3.0
        rs.submit(img(), meta(i))
        rs.flush()
        assert len([n for n in files(tmp_path) if n.endswith(".jpg")]) <= 3


def test_save_max_above_ten_is_clamped_with_a_warning(tmp_path):
    rs, clk, msgs = make(tmp_path, save_max=30)
    assert rs.max == SAVE_MAX_LIMIT == 10
    assert any("10" in m and "WARN" in m for m in msgs)
    save_many(rs, clk, 25)
    assert len([n for n in files(tmp_path) if n.endswith(".jpg")]) == 10


def test_save_max_zero_saves_nothing_and_creates_nothing(tmp_path):
    rs, clk, _ = make(tmp_path, save_max=0)
    assert not rs.enabled
    assert rs.submit(img(), meta()) is False
    rs.flush()
    assert files(tmp_path) == []


@pytest.mark.parametrize("value,expect", [(-3, 0), (0, 0), (5, 5), (10, 10), (11, 10), ("7", 7), (None, 0), ("abc", 0)])
def test_clamp_save_max(value, expect):
    assert clamp_save_max(value, lambda m: None) == expect


# ---------------------------------------------------------------------- 간격·형식
def test_minimum_interval_blocks_back_to_back_scenes_but_force_does_not(tmp_path):
    rs, clk, _ = make(tmp_path, save_every_s=2.0)
    clk.t = 10.0
    assert rs.submit(img(), meta())
    clk.t = 10.5
    assert rs.submit(img(), meta()) is False and rs.stats["skipped_interval"] == 1
    assert rs.submit(img(), meta(), force=True)                                # 운영자 요청은 간격과 상관없다
    clk.t = 12.6
    assert rs.submit(img(), meta())


def test_saved_pair_has_640_wide_jpeg_and_matching_json_with_scaled_boxes(tmp_path):
    import cv2
    rs, clk, _ = make(tmp_path)
    clk.t = 5.0
    rs.submit(img(1280, 720, 90), meta(7))
    rs.flush()
    names = files(tmp_path)
    jpg = next(n for n in names if n.endswith(".jpg"))
    js = jpg[:-4] + ".json"
    assert js in names
    im = cv2.imread(str(tmp_path / "ring" / jpg))
    assert im.shape[1] == 640 and im.shape[0] == 360
    doc = json.loads((tmp_path / "ring" / js).read_text(encoding="utf-8"))
    assert doc["image_size"] == [640, 360] and doc["scale"] == pytest.approx(0.5) and doc["jpeg_quality"] == 80
    assert doc["boxes"][0]["box"] == pytest.approx([50.0, 100.0, 150.0, 200.0])     # 저장된 JPEG 픽셀 좌표
    assert doc["boxes"][0]["cls"] == "TANK" and doc["boxes"][0]["source"] == "oak"
    assert doc["source"] == "oak" and doc["pose"] == {"x": 1.0, "y": 2.0, "th": 0.5} and doc["stamp"] == 7.0


def test_images_narrower_than_640_are_not_upscaled(tmp_path):
    rs, clk, _ = make(tmp_path)
    clk.t = 5.0
    rs.submit(img(320, 240), meta())
    rs.flush()
    doc = json.loads(next((tmp_path / "ring").glob("*.json")).read_text(encoding="utf-8"))
    assert doc["image_size"] == [320, 240] and doc["scale"] == 1.0


def test_no_partial_files_are_left_behind(tmp_path):
    rs, clk, _ = make(tmp_path)
    save_many(rs, clk, 5)
    assert not [n for n in files(tmp_path) if n.endswith(".tmp")]


# ---------------------------------------------------------------------- 시작 시 정리·우리 파일만
def test_start_removes_previous_run_files_but_only_ours(tmp_path):
    d = tmp_path / "ring"
    d.mkdir()
    for n in ("oak_000001_111.jpg", "oak_000001_111.json", "oak_000002_222.jpg"):
        (d / n).write_bytes(b"x")
    keep = d / "notes.txt"
    keep.write_text("mine")
    other = d / "oak_samples_backup.jpg"
    other.write_bytes(b"x")
    rs, clk, _ = make(tmp_path)
    assert files(tmp_path) == ["notes.txt", "oak_samples_backup.jpg"]


def test_periodic_cleanup_enforces_the_limit_even_if_files_appear_outside_writes(tmp_path):
    rs, clk, _ = make(tmp_path, save_max=2, cleanup_every_s=30.0)
    d = tmp_path / "ring"
    d.mkdir(exist_ok=True)
    for i in range(1, 7):
        (d / f"oak_{i:06d}_{i}.jpg").write_bytes(b"x")
        (d / f"oak_{i:06d}_{i}.json").write_bytes(b"{}")
    rs.cleanup()
    assert len([n for n in files(tmp_path) if n.endswith(".jpg")]) == 2


# ---------------------------------------------------------------------- 운영자 요청(TRIGGER 파일)
def test_trigger_file_is_consumed_once(tmp_path):
    rs, clk, _ = make(tmp_path)
    d = tmp_path / "ring"
    d.mkdir(exist_ok=True)
    assert rs.consume_trigger() is False
    (d / TRIGGER_NAME).write_text("")
    assert rs.consume_trigger() is True
    assert not (d / TRIGGER_NAME).exists() and rs.consume_trigger() is False


def test_trigger_is_ignored_when_saving_is_off(tmp_path):
    rs, clk, _ = make(tmp_path, save_max=0)
    d = tmp_path / "ring"
    d.mkdir()
    (d / TRIGGER_NAME).write_text("")
    assert rs.consume_trigger() is False and (d / TRIGGER_NAME).exists()


# ---------------------------------------------------------------------- 디스크 오류는 경고만
def test_disk_error_is_a_warning_not_an_exception(tmp_path, monkeypatch):
    rs, clk, msgs = make(tmp_path)

    def boom(*a, **k):
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(Path, "write_bytes", boom)
    clk.t = 5.0
    assert rs.submit(img(), meta())
    rs.flush()                                                                 # 예외 없음
    assert rs.stats["errors"] == 1 and "No space" in rs.stats["last_error"]
    assert any("WARN" in m for m in msgs)
    assert rs.stats["saved"] == 0


def test_unwritable_directory_is_a_warning(tmp_path):
    blocker = tmp_path / "ring"
    blocker.write_text("a file, not a directory")
    msgs = []
    rs = RingStore(blocker, 8, 0.0, 30.0, clock=Tick(), report=msgs.append, start_thread=False)
    rs.submit(img(), meta())
    rs.flush()
    assert rs.stats["errors"] >= 1


def test_encode_failure_is_a_warning(tmp_path):
    rs, clk, msgs = make(tmp_path)
    clk.t = 5.0
    rs.submit(np.zeros((0, 0, 3), np.uint8), meta())
    rs.flush()
    assert rs.stats["saved"] == 0


def test_a_full_queue_never_blocks_the_caller(tmp_path):
    rs, clk, _ = make(tmp_path, save_every_s=0.0)
    results = [rs.submit(img(64, 48), meta(i)) for i in range(10)]            # 쓰기 스레드가 없으니 큐(4)가 찬다
    assert results[:4] == [True] * 4 and not any(results[4:]) and rs.stats["skipped_busy"] == 6


def test_submit_without_image_is_a_noop(tmp_path):
    rs, clk, _ = make(tmp_path)
    assert rs.submit(None, meta()) is False


# ---------------------------------------------------------------------- 쓰기 스레드
def test_background_thread_writes_and_close_flushes(tmp_path):
    msgs = []
    rs = RingStore(tmp_path / "ring", 5, 0.0, 30.0, report=msgs.append)         # 진짜 스레드
    for i in range(12):
        rs.submit(img(320, 240, i), meta(i))
    rs.close()
    assert 1 <= len([n for n in files(tmp_path) if n.endswith(".jpg")]) <= 5


# ---------------------------------------------------------------------- 저장소 안이면 .gitignore
def test_inside_repo_adds_gitignore_line_once(tmp_path):
    msgs = []
    save = tmp_path / "logs" / "oak"
    assert ensure_gitignored(save, tmp_path, msgs.append) is True
    assert (tmp_path / ".gitignore").read_text().splitlines() == ["logs/oak/"]
    assert ensure_gitignored(save, tmp_path, msgs.append) is False             # 이미 있다
    assert (tmp_path / ".gitignore").read_text().count("logs/oak/") == 1


def test_gitignore_keeps_existing_lines(tmp_path):
    (tmp_path / ".gitignore").write_text("*.pyc")
    ensure_gitignored(tmp_path / "x", tmp_path, lambda m: None)
    assert (tmp_path / ".gitignore").read_text().splitlines() == ["*.pyc", "x/"]


def test_outside_repo_changes_nothing(tmp_path):
    repo = tmp_path / "repo"
    repo.mkdir()
    assert ensure_gitignored(tmp_path / "elsewhere", repo, lambda m: None) is False
    assert not (repo / ".gitignore").exists()


def test_unwritable_gitignore_is_only_a_warning(tmp_path, monkeypatch):
    msgs = []
    monkeypatch.setattr(Path, "write_text", lambda *a, **k: (_ for _ in ()).throw(OSError("read-only")))
    assert ensure_gitignored(tmp_path / "x", tmp_path, msgs.append) is False
    assert any("WARN" in m for m in msgs)


def test_default_save_dir_is_outside_the_repository():
    from maicon26.validate import run_log_dir
    repo = Path(__file__).resolve().parents[1]
    assert repo not in run_log_dir().resolve().parents                          # 기본 저장 위치는 저장소 밖이다
