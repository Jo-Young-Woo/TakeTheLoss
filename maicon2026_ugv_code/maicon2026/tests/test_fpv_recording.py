"""Actual camera recording must preserve timestamps and expose missing imagery."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from maicon26.hal.fpv_recording import SimulationVideoWriter, decode_bgr


class MemoryWriter:
    def __init__(self, *_args):
        self.frames = []
        self.released = False

    def isOpened(self):
        return True

    def write(self, frame):
        self.frames.append(frame.copy())

    def release(self):
        self.released = True


def test_padded_actual_ros_rgb_is_decoded_without_swapping_rows():
    msg = SimpleNamespace(width=2, height=2, step=8, encoding="rgb8",
                          data=bytes([255, 0, 0, 0, 255, 0, 99, 99,
                                      0, 0, 255, 255, 255, 255, 77, 77]))
    assert decode_bgr(msg).tolist() == [[[0, 0, 255], [0, 255, 0]],
                                       [[255, 0, 0], [255, 255, 255]]]
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="stride"):
        decode_bgr(msg)


def test_timeline_holds_only_observed_images_and_marks_long_camera_gaps(tmp_path):
    memory = MemoryWriter()
    recorder = SimulationVideoWriter(tmp_path / "real.mp4", fps=10, max_hold_s=.2,
                                     writer_factory=lambda *_: memory)
    before = np.full((64, 128, 3), 50, dtype=np.uint8)
    after = np.full_like(before, 180)
    recorder.add(before, 100.0)
    recorder.add(after, 100.5)
    summary = recorder.close()
    assert summary["source_frames"] == 2
    assert summary["encoded_frames"] == 6
    assert summary["held_frames"] == 2
    assert summary["missing_frames"] == 2
    assert np.array_equal(memory.frames[0], before)
    assert np.array_equal(memory.frames[1], before)
    assert not np.array_equal(memory.frames[3], before)
    assert np.array_equal(memory.frames[5], after)
    frames = [json.loads(line) for line in recorder.timestamps.read_text().splitlines()]
    assert [r["video_s"] for r in frames] == [0, .1, .2, .3, .4, .5]
    assert frames[4]["missing"] and frames[4]["image_sim_s"] == 100
    assert not frames[5]["held"] and frames[5]["image_sim_s"] == 100.5
    assert memory.released


def test_duplicate_reset_resolution_and_output_overwrite_are_not_silent(tmp_path):
    path = tmp_path / "real.mp4"
    rec = SimulationVideoWriter(path, writer_factory=MemoryWriter)
    frame = np.zeros((64, 128, 3), dtype=np.uint8)
    assert rec.add(frame, 10)
    assert not rec.add(frame, 10)
    with pytest.raises(ValueError, match="clock moved"):
        rec.add(frame, 9)
    with pytest.raises(ValueError, match="resolution changed"):
        rec.add(frame[:, :64], 11)
    result = rec.close()
    assert result["duplicate_stamps"] == 1
    with pytest.raises(FileExistsError):
        SimulationVideoWriter(path)


def test_h264_output_decodes_at_recorded_size_and_simulation_duration(tmp_path):
    path = tmp_path / "camera.mp4"
    recorder = SimulationVideoWriter(path)
    for i in range(10):
        frame = np.zeros((64, 128, 3), np.uint8)
        frame[:, i * 10:i * 10 + 20, 1] = 220
        try:
            recorder.add(frame, 20 + i / 10)
        except RuntimeError as exc:
            recorder.close()
            pytest.skip(str(exc))
    summary = recorder.close()
    video = cv2.VideoCapture(str(path))
    assert video.isOpened()
    assert video.get(cv2.CAP_PROP_FPS) == pytest.approx(10)
    count, last = 0, None
    while True:
        ok, frame = video.read()
        if not ok:
            break
        count += 1
        last = frame
    video.release()
    assert count == summary["encoded_frames"] == 10
    assert last.shape == (64, 128, 3)
    assert summary["source_span_s"] == pytest.approx(.9)
    assert summary["held_frames"] == summary["missing_frames"] == 0


def test_rejected_long_gap_does_not_inflate_accepted_source_statistics(tmp_path):
    recorder = SimulationVideoWriter(tmp_path / "gap.mp4", writer_factory=MemoryWriter)
    frame = np.zeros((64, 128, 3), np.uint8)
    recorder.add(frame, 100)
    with pytest.raises(ValueError, match="gap exceeded"):
        recorder.add(frame, 161)
    summary = recorder.close()
    assert summary["source_frames"] == summary["encoded_frames"] == 1
    assert summary["first_image_sim_s"] == summary["last_image_sim_s"] == 100
    assert summary["source_span_s"] == summary["source_fps"] == 0
    assert summary["max_source_gap_s"] == 0
