#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""가짜 ROS(tests/fake_ros, tests/fake_bin/ros2)로 스크립트 전체 흐름을 돌려 본다. 로봇·ROS 없이 돈다(리눅스).

  python3 tests/test_integration.py

실제 rclpy·rosbag2 와의 연결은 여기서 확인되지 않는다. 그 부분은 로봇에서 README 의 첫 실행 점검표로 본다.
"""
import contextlib
import fcntl
import glob
import http.server
import io
import json
import os
import pty
import select
import signal
import subprocess
import sys
import tempfile
import termios
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FAKE_CODE = "K7M2Q9XA"
UP = b"\x1b[A"


def env(**extra):
    e = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", MIN_FREE_GB="0", STOP_FREE_GB="0")
    e["PYTHONPATH"] = os.path.join(HERE, "fake_ros") + os.pathsep + e.get("PYTHONPATH", "")
    e["PATH"] = os.path.join(HERE, "fake_bin") + os.pathsep + e["PATH"]
    for k in ("MAICON_CODE", "MAICON_MISSION_CODE", "FAKE_ROS_OTHERS", "FAKE_ROS_MODE", "FAKE_TOPICS", "FAKE_NO_METADATA", "FAKE_HANDLE_SIGINT"):
        e.pop(k, None)
    e.update(extra)
    return e


class Term:
    """pty 안에서 명령을 돌리고 키를 넣는다."""

    def __init__(self, argv, **extra):
        self.master, slave = pty.openpty()
        self.before = termios.tcgetattr(slave)
        self.slave = slave
        def own_terminal():                                  # 실제 SSH 처럼 이 pty 를 제어 터미널로 삼는다(Ctrl-C 가 신호로 간다)
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        self.p = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=slave, env=env(**extra), preexec_fn=own_terminal, cwd=ROOT)
        self.out = b""

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            r, _, _ = select.select([self.master], [], [], 0.02)
            if r:
                try:
                    self.out += os.read(self.master, 4096)
                except OSError:
                    break

    def keys(self, data, secs=0.0, every=0.05):
        if secs <= 0:
            os.write(self.master, data)
            self.pump(0.05)
            return
        end = time.time() + secs
        while time.time() < end:
            os.write(self.master, data)
            self.pump(every)

    def wait(self, timeout=30):
        end = time.time() + timeout
        while self.p.poll() is None and time.time() < end:
            self.pump(0.1)
        self.pump(0.2)
        assert self.p.poll() is not None, "프로세스가 끝나지 않았다:\n" + self.text()[-800:]
        return self.p.returncode

    def text(self):
        return self.out.decode("utf-8", "replace")

    def restored(self):
        return termios.tcgetattr(self.slave)[3] == self.before[3]              # lflag(ICANON·ECHO)가 원래대로


def read_log(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def cmds(log):
    return [(d["v"], d["w"]) for d in log if d.get("topic") == "/cmd_vel"]


# ---------------------------------------------------------------- 텔레옵
def test_teleop_drive_stop_quit():
    log = tempfile.mktemp()
    t = Term([sys.executable, "teleop_keys.py"], FAKE_ROS_LOG=log)
    t.pump(1.0)
    t.keys(UP, secs=1.2)
    t.pump(0.8)                                                                # 손을 뗐다
    t.keys(b"d")
    t.keys(b"m")
    t.pump(0.4)
    t.keys(b"q")
    assert t.wait() == 0 and t.restored()
    data = read_log(log)
    c = cmds(data)
    assert max(v for v, _ in c) > 0.1 and max(v for v, _ in c) <= 0.12 + 1e-9, c
    assert all(x == (0.0, 0.0) for x in c[-5:]) and "정지 명령 5회" in t.text()
    ev = [json.loads(d["data"]) for d in data if d.get("topic") == "/teleop/events"]
    assert [e["type"] for e in ev][0] == "start" and ev[-1]["type"] == "end" and any(e["type"] == "marker" for e in ev)
    assert any(d.get("service") == "/mil/gimbal" and d["req"]["pan"] == 5.0 for d in data)
    assert any(e["type"] == "gimbal" and e["resp"] == "ok" for e in ev)
    assert any(d.get("topic") == "/mil/oled" and d.get("text") == "TELEOP" for d in data)


def test_teleop_sighup_and_ctrl_c_send_zero():
    for how in ("hup", "ctrl_c", "term"):
        log = tempfile.mktemp()
        t = Term([sys.executable, "teleop_keys.py"], FAKE_ROS_LOG=log)
        t.pump(1.0)
        t.keys(UP, secs=0.8)
        if how == "ctrl_c":
            os.write(t.master, b"\x03")
        else:
            t.p.send_signal(signal.SIGHUP if how == "hup" else signal.SIGTERM)
        assert t.wait() == 0, how
        c = cmds(read_log(log))
        assert max(v for v, _ in c) > 0.05 and all(x == (0.0, 0.0) for x in c[-5:]), (how, c[-8:])
        assert t.restored(), how


def test_teleop_other_publisher_exits_without_sending():
    log = tempfile.mktemp()
    t = Term([sys.executable, "teleop_keys.py"], FAKE_ROS_LOG=log, FAKE_ROS_OTHERS="1")
    t0 = time.time()
    t.keys(UP, secs=2.0)
    rc = t.wait()
    assert rc == 3 and 2.5 <= time.time() - t0 <= 6.0
    assert cmds(read_log(log)) == [] and "정지 명령 안 보냄" in t.text() and t.restored()


def test_teleop_mission_mode_exits_without_sending():
    log = tempfile.mktemp()
    t = Term([sys.executable, "teleop_keys.py"], FAKE_ROS_LOG=log, FAKE_ROS_MODE="mission")
    t.keys(UP, secs=1.5)
    t.wait()
    assert all(x == (0.0, 0.0) for x in cmds(read_log(log))) and "경기 모드" in t.text()
    assert not any(d.get("service") for d in read_log(log))


def test_teleop_unknown_mode_needs_flag():
    log = tempfile.mktemp()
    t = Term([sys.executable, "teleop_keys.py"], FAKE_ROS_LOG=log, FAKE_ROS_MODE="")
    t.pump(0.5)
    t.keys(UP, secs=1.0)
    t.keys(b"q")
    t.wait()
    c = cmds(read_log(log))
    assert c and all(x == (0.0, 0.0) for x in c)
    log = tempfile.mktemp()
    t = Term([sys.executable, "teleop_keys.py", "--no-state-guard"], FAKE_ROS_LOG=log, FAKE_ROS_MODE="")
    t.pump(0.5)
    t.keys(UP, secs=1.0)
    t.keys(b"q")
    t.wait()
    assert max(v for v, _ in cmds(read_log(log))) > 0.05


def test_teleop_fire_only_with_flag():
    for flag, want in ((False, 0), (True, 1)):
        log = tempfile.mktemp()
        t = Term([sys.executable, "teleop_keys.py"] + (["--allow-fire"] if flag else []), FAKE_ROS_LOG=log)
        t.pump(1.0)
        t.keys(b"f")
        t.pump(0.2)
        t.keys(b"f")
        t.pump(0.5)
        t.keys(b"q")
        t.wait()
        assert sum(1 for d in read_log(log) if d.get("service") == "/mil/fire") == want, flag


# ---------------------------------------------------------------- 기록
def run_record(args, secs=3.5, sig=signal.SIGINT, **extra):
    root = tempfile.mkdtemp()
    p = subprocess.Popen(["bash", os.path.join(ROOT, "record.sh"), "-o", root] + args, env=env(**extra), cwd=ROOT,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(secs)
    if p.poll() is None:
        p.send_signal(sig)
    out = p.communicate(timeout=60)[0].decode("utf-8", "replace")
    sess = glob.glob(os.path.join(root, "*"))
    return p.returncode, out, (sess[0] if sess else None)


def no_leftover():
    time.sleep(0.3)
    out = subprocess.run(["pgrep", "-f", "fake_bin/ros2 bag record"], capture_output=True, text=True).stdout.strip()
    return out == ""


def test_record_normal_stop():
    rc, out, s = run_record(["-t", "cal_cam"])
    assert rc == 0, out
    assert os.path.basename(s).split("_", 2)[2].startswith("cal_cam_")
    for f in ("bag/metadata.yaml", "notes.txt", "info.txt", "record.log", "rec_status"):
        assert os.path.exists(os.path.join(s, f)), f
    args = open(os.path.join(s, "bag", "args.txt")).read()
    assert "bag record -a -s mcap -o" in args and "--max-bag-duration 60" in args and "--storage-preset-profile fastwrite" in args
    assert "stdin_tty=False sid_leader=True" in args
    assert open(os.path.join(s, "rec_status")).read().startswith("STOPPED") and "결과: 정상" in out
    notes = open(os.path.join(s, "notes.txt")).read()
    assert "태그: cal_cam" in notes and "끝:" in notes and "ROS_DOMAIN_ID" in notes
    assert no_leftover()


def test_record_term_and_default_sigint_handler():
    rc, out, s = run_record(["-t", "a"], sig=signal.SIGTERM)
    assert rc == 0 and os.path.exists(os.path.join(s, "bag", "metadata.yaml")), out
    rc, out, s = run_record(["-t", "b"], FAKE_HANDLE_SIGINT="0")                # 기록기가 SIGINT 를 무시 상태로 물려받지 않았는지
    assert rc == 0 and os.path.exists(os.path.join(s, "bag", "metadata.yaml")), out
    assert no_leftover()


def test_record_reindex_when_metadata_missing():
    rc, out, s = run_record(["-t", "c"], FAKE_NO_METADATA="1")
    assert os.path.exists(os.path.join(s, "bag", "metadata.yaml")) and "reindex" in out, out


def test_record_missing_topic_fails_check_and_exclude():
    rc, out, s = run_record(["-t", "d", "-x", "/oak/.*"], FAKE_TOPICS="/cmd_vel /odom /image_raw /tf")
    assert rc == 1 and "실패: /scan 없음" in out, out
    assert "--exclude /oak/.*" in open(os.path.join(s, "bag", "args.txt")).read()


def test_record_refuses_low_disk_and_bad_tag_and_no_topics():
    rc, out, s = run_record(["-t", "e"], secs=1.0, MIN_FREE_GB="999999")
    assert rc == 2 and "시작하지 않는다" in out and not os.path.exists(os.path.join(s, "bag"))
    rc, out, s = run_record(["-t", "bad tag"], secs=0.5)
    assert rc == 2 and s is None
    rc, out, s = run_record(["-t", "f"], secs=1.0, FAKE_TOPICS=" ")
    assert rc == 2 and "토픽이 하나도 없다" in out


def test_record_does_not_leak_code():
    rc, out, s = run_record(["-t", "g"], MAICON_CODE=FAKE_CODE)
    assert rc == 0, out
    for d, _, files in os.walk(s):
        for f in files:
            assert FAKE_CODE.encode() not in open(os.path.join(d, f), "rb").read(), f
    assert FAKE_CODE not in out and "임무코드를 찾지 못했다" in open(os.path.join(s, "record.log")).read()


def test_teleop_record_together():
    root = tempfile.mkdtemp()
    t = Term(["bash", "teleop_record.sh", "-t", "drive", "-o", root, "--", "--level", "2"], FAKE_ROS_LOG=tempfile.mktemp())
    t.pump(5.0)
    assert "기록 중" in t.text(), t.text()
    t.keys(UP, secs=1.0)
    t.keys(b"m")
    t.pump(0.5)
    assert "REC" in t.text()
    t.keys(b"q")
    assert t.wait(40) == 0, t.text()[-600:]
    s = glob.glob(os.path.join(root, "*"))[0]
    assert os.path.exists(os.path.join(s, "bag", "metadata.yaml"))
    ev = read_log(os.path.join(s, "teleop_events.jsonl"))
    assert any(e["type"] == "marker" for e in ev) and ev[0]["level"] == 2
    assert open(os.path.join(s, "rec_status")).read().startswith("STOPPED") and no_leftover()


def test_teleop_record_ctrl_c_and_hup_close_bag():
    for how in ("ctrl_c", "hup"):
        root = tempfile.mkdtemp()
        t = Term(["bash", "teleop_record.sh", "-t", "x", "-o", root], FAKE_ROS_LOG=tempfile.mktemp())
        t.pump(5.0)
        t.keys(UP, secs=0.5)
        if how == "ctrl_c":
            os.write(t.master, b"\x03")
        else:
            os.killpg(t.p.pid, signal.SIGHUP)
        t.wait(40)
        s = glob.glob(os.path.join(root, "*"))[0]
        end = time.time() + 25
        while time.time() < end and not open(os.path.join(s, "rec_status")).read().startswith("STOPPED"):
            time.sleep(0.5)
        assert os.path.exists(os.path.join(s, "bag", "metadata.yaml")), how
        assert open(os.path.join(s, "rec_status")).read().startswith("STOPPED"), how
        assert no_leftover(), how


def test_teleop_record_aborts_when_record_fails():
    root = tempfile.mkdtemp()
    t = Term(["bash", "teleop_record.sh", "-t", "y", "-o", root], FAKE_ROS_LOG=tempfile.mktemp(), FAKE_TOPICS=" ")
    assert t.wait(40) == 2 and "텔레옵 시작" not in t.text()


# ---------------------------------------------------------------- UAV 수집
class _Drone(http.server.BaseHTTPRequestHandler):
    n = 0
    status = 200

    def do_GET(self):
        type(self).n += 1
        ok = self.path.startswith("/api/report/drone") and ("mission_code=" + FAKE_CODE) in self.path and "camera=main" in self.path
        if not ok or type(self).status != 200:
            self.send_response(403)
            self.end_headers()
            return
        body = b"\xff\xd8fakejpeg\xff\xd9"
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("X-Camera", "main")
        self.send_header("X-Captured-At", str(1000 + (type(self).n + 1) // 2))   # 두 번에 한 번만 새 프레임
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def test_uav_collect():
    sys.path.insert(0, ROOT)
    import uav_collect
    srv = http.server.HTTPServer(("127.0.0.1", 0), _Drone)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{srv.server_port}"
    old = os.environ.get("MAICON_CODE")
    os.environ["MAICON_CODE"] = FAKE_CODE
    try:
        out_dir = tempfile.mkdtemp()
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = uav_collect.main(["--server", url, "--count", "3", "--interval", "0.05", "--out", out_dir])
        assert rc == 0 and sorted(os.listdir(out_dir)) == ["f_0001.jpg", "f_0002.jpg", "f_0003.jpg", "frames.csv"]
        csv_text = open(os.path.join(out_dir, "frames.csv")).read()
        assert csv_text.count("\n") == 4 and "main" in csv_text and _Drone.n >= 5      # 같은 촬영 시각은 건너뛰었다
        assert FAKE_CODE not in buf.getvalue() and FAKE_CODE not in csv_text
        _Drone.status = 403
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = uav_collect.main(["--server", url, "--count", "1", "--interval", "0.01", "--out", tempfile.mkdtemp()])
        assert rc == 1 and "HTTPError 403" in buf.getvalue()
        assert FAKE_CODE not in buf.getvalue() and "127.0.0.1" not in buf.getvalue().split("\n", 1)[1]
    finally:
        srv.shutdown()
        if old is None:
            os.environ.pop("MAICON_CODE", None)
        else:
            os.environ["MAICON_CODE"] = old


if __name__ == "__main__":
    only = sys.argv[1:]
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f) and (not only or n in only)]
    failed = 0
    for name, fn in tests:
        t0 = time.time()
        try:
            fn()
            print(f"통과 {name} ({time.time() - t0:.0f}s)", flush=True)
        except Exception as e:                                                # noqa: BLE001
            failed += 1
            print(f"실패 {name} {type(e).__name__}: {str(e)[:900]}", flush=True)
    print(f"{len(tests) - failed}/{len(tests)} 통과")
    sys.exit(1 if failed else 0)
