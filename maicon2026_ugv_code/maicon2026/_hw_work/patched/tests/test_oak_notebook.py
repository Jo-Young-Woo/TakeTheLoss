"""6장 6-6: tools/notebooks/oak_field_check.ipynb — 구조·안전 규칙·(가짜 파일로) 코드 셀 실행. ROS·시뮬 없이."""
import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
NB = ROOT / "tools" / "notebooks" / "oak_field_check.ipynb"


@pytest.fixture(scope="module")
def nb():
    return json.loads(NB.read_text(encoding="utf-8"))


def cell_src(c):
    return "".join(c["source"])


def all_src(nb):
    return "\n".join(cell_src(c) for c in nb["cells"])


def code_cells(nb):
    return [c for c in nb["cells"] if c["cell_type"] == "code"]


# ---------------------------------------------------------------------- 형식·출력
def test_valid_notebook_with_all_outputs_cleared(nb):
    assert nb["nbformat"] == 4
    for c in code_cells(nb):
        assert c["outputs"] == [] and c["execution_count"] is None
    assert "image/png" not in NB.read_text(encoding="utf-8")                   # 경기장 이미지가 저장소에 남지 않는다


def test_cells_follow_the_chapter_layout(nb):
    heads = [cell_src(c).splitlines()[0] for c in nb["cells"] if c["cell_type"] == "markdown"]
    want = ("설정", "토픽", "링 버퍼", "유효성 통계", "깊이", "정리 확인")
    for w in want:
        assert any(w in h for h in heads), w


# ---------------------------------------------------------------------- 안전: 읽기만 한다
@pytest.mark.parametrize("bad", ["/cmd_vel", "cmd_vel", "/mil/gimbal", "/mil/fire", "set_velocity", "laser_fire", "set_pan_tilt",
                                 "publish(", "call_gimbal", "call_fire", "ros2 topic pub", "ros2 service call"])
def test_notebook_never_commands_the_robot(nb, bad):
    assert bad not in all_src(nb)


def test_ros_cli_cells_are_the_read_only_ones_the_chapter_lists(nb):
    bang = [l.strip() for c in code_cells(nb) for l in cell_src(c).splitlines() if l.strip().startswith("!")]
    assert bang == ["!ros2 topic list | grep oak", "!timeout 10 ros2 topic hz {rgb_topic}",
                    "!ros2 topic echo --once {info_topic}", "!ros2 param dump {OAK_NODE}"]


def test_topic_names_come_from_the_current_settings(nb):
    src = all_src(nb)
    assert "rgb_topic, info_topic = cfg.oak.rgb_topic, cfg.oak.rgb_info_topic" in src
    assert "[실기 확인]" in src


def test_bgr_to_rgb_conversion_only_for_display_in_the_ring_buffer_cell(nb):
    holders = [cell_src(c) for c in nb["cells"] if "COLOR_BGR2RGB" in cell_src(c) or "[..., ::-1]" in cell_src(c)]
    assert len(holders) == 1 and "표시 전용" in holders[0] and "oak_*.jpg" in holders[0]


def test_notebook_writes_no_files(nb):
    src = "\n".join(cell_src(c) for c in code_cells(nb))
    for w in ("write_text", "write_bytes", "imwrite", "open(", ".unlink(", "os.remove", "shutil", "savefig", "to_csv"):
        assert w not in src, w


# ---------------------------------------------------------------------- 코드 셀 실행(가짜 파일)
def run_cells(nb, tmp_path, monkeypatch, files=10, save_max=None):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    import cv2
    ring = tmp_path / "logs" / "oak_samples"
    ring.mkdir(parents=True)
    for i in range(1, files + 1):
        img = np.full((360, 640, 3), (10, 120, 240), np.uint8)
        cv2.imwrite(str(ring / f"oak_{i:06d}_{1000 + i}.jpg"), img)
        (ring / f"oak_{i:06d}_{1000 + i}.json").write_text(json.dumps(
            {"why": "new_class", "source": "oak", "boxes": [{"cls": "TANK", "conf": 0.8, "box": [50, 60, 200, 220]}]}), encoding="utf-8")
    (tmp_path / "logs" / "validation_status.json").write_text(json.dumps(
        {"schema": 1, "written_wall": 1.0, "uptime_s": 12.0, "sources": {
            "image": {"dropped": 3, "reasons": {"size": 3}, "last_reason": "size", "valid_count": 40, "last_valid_age_s": 0.5}}}), encoding="utf-8")
    monkeypatch.setenv("MAICON_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.chdir(ROOT / "tools" / "notebooks")
    ns = {"display": lambda x: print(x)}
    import matplotlib.pyplot as plt
    monkeypatch.setattr(plt, "show", lambda *a, **k: None)
    out = []
    for c in code_cells(nb):
        src = "\n".join(l for l in cell_src(c).splitlines() if not l.strip().startswith("!"))
        if "cfg, applied" in src and save_max is not None:
            src += f"\ncfg.oak_det.save_max = {save_max}\n"
        exec(compile(src, "nb-cell", "exec"), ns)
    return ns


def test_code_cells_run_against_fake_files_and_report_ok(nb, tmp_path, monkeypatch, capsys):
    ns = run_cells(nb, tmp_path, monkeypatch, files=8, save_max=8)
    out = capsys.readouterr().out
    assert "표본 폴더" in out and "장수: 8" in out
    assert "jpg 8장 / 한도 8장" in out and "OK" in out
    assert "image" in out                                                      # validation_status 표의 출처
    assert ns["cfg"].oak.rgb_topic in ns["rgb_topic"]
    assert not (ROOT / "tools" / "notebooks" / ".ipynb_checkpoints").exists()


def test_over_limit_is_flagged(nb, tmp_path, monkeypatch, capsys):
    run_cells(nb, tmp_path, monkeypatch, files=12, save_max=10)
    out = capsys.readouterr().out
    assert "jpg 12장 / 한도 10장" in out and "WARN" in out


def test_save_max_above_ten_is_shown_as_ten(nb, tmp_path, monkeypatch, capsys):
    run_cells(nb, tmp_path, monkeypatch, files=3, save_max=30)
    assert "한도 10장" in capsys.readouterr().out


def test_missing_status_file_is_explained_not_an_error(nb, tmp_path, monkeypatch, capsys):
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    monkeypatch.setenv("MAICON_LOG_DIR", str(tmp_path / "empty"))
    monkeypatch.chdir(ROOT / "tools" / "notebooks")
    ns = {"display": print}
    for c in code_cells(nb):
        src = "\n".join(l for l in cell_src(c).splitlines() if not l.strip().startswith("!"))
        exec(compile(src, "nb-cell", "exec"), ns)
    out = capsys.readouterr().out
    assert "아직 없다" in out and "표본 폴더" in out and "장수: 0" in out
