"""Record one explicit Gazebo HWND, then run its isolated match.

Windows dependencies: windows-capture>=2.0.1, OpenCV, NumPy. WGC captures WSLg's
GPU window; Pillow/PrintWindow may return black. No desktop capture, title search,
focus changes, or window resizing. Start Gazebo/Nav2 separately in the same domain.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shlex
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


class WindowCapture:
    """WGC bound to one HWND. Callbacks copy only that window's pixels."""
    def __init__(self, hwnd: int, fps: int = 10):
        from windows_capture import WindowsCapture
        if hwnd <= 0:
            raise ValueError("--hwnd must be a positive Windows window handle")
        self._lock = threading.Lock()
        self._changed = threading.Event()
        self._latest = None
        self._error = None
        self._stopping = False
        self._sequence = 0
        self.capture = WindowsCapture(window_hwnd=hwnd, cursor_capture=False,
                                      minimum_update_interval=max(1, round(1000 / fps)))

        @self.capture.event
        def on_frame_arrived(frame, capture_control):
            try:
                pixels = np.ascontiguousarray(frame.frame_buffer[:, :, :3]).copy()
                with self._lock:
                    self._sequence += 1
                    self._latest = (pixels, time.monotonic(), time.time(), int(frame.timespan), self._sequence)
                self._changed.set()
            except Exception as exc:
                self._error = f"WGC frame callback failed: {exc}"
                self._changed.set()
                capture_control.stop()

        @self.capture.event
        def on_closed():
            if not self._stopping:
                self._error = "The selected WGC window closed or capture ended"
            self._changed.set()

        self.control = self.capture.start_free_threaded()

    def snapshot(self, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while True:
            if self._error:
                raise RuntimeError(self._error)
            with self._lock:
                latest = self._latest
            if latest is not None:
                if time.monotonic() - latest[1] > 10:
                    raise RuntimeError("WGC has delivered no new window frame for 10 seconds")
                return latest
            left = deadline - time.monotonic()
            if left <= 0:
                raise RuntimeError("WGC produced no frame for this HWND; no desktop fallback is permitted")
            self._changed.wait(min(left, .2))
            self._changed.clear()

    def initial_frame(self):
        deadline = time.monotonic() + 10
        while True:
            data = self.snapshot()
            if min(data[0].shape[:2]) > 16 and np.any(data[0]):
                return data
            if time.monotonic() >= deadline:
                raise RuntimeError("Selected HWND produced only black/empty WGC frames")
            time.sleep(.1)

    def close(self):
        self._stopping = True
        self.control.stop()


def wsl_path(path: Path, distro: str) -> str:
    """Pass Windows paths as argv, never shell-interpolated text."""
    result = subprocess.run(["wsl.exe", "-d", distro, "--", "wslpath", "-a", "-u", str(path.resolve())],
                            capture_output=True, text=True, encoding="utf-8", check=True, timeout=30)
    value = result.stdout.strip()
    if not value.startswith("/") or "\n" in value:
        raise RuntimeError(f"wslpath did not return one absolute Linux path: {value!r}")
    return value


def match_command(args, root_linux: str, report_linux: str, pid_linux: str, run_id: str):
    argv = ["python3", "-u", f"{root_linux}/scripts/run_gazebo_match.py",
            "--layout", args.layout, "--seed", str(args.seed), "--physical-robots", args.physical_robots,
            "--duration", str(args.duration), "--base-at", str(args.base_at),
            "--smog-at", str(args.smog_at), "--report", report_linux,
            "--m3-count", str(args.m3_count), "--uav-layout", args.uav_layout]
    if args.uav_schedule:
        argv.extend(("--uav-schedule", args.uav_schedule))
    # Register the session leader before ROS setup, so cancellation covers setup too.
    child = ("set -e\numask 077\n"
             f"printf '%s %s\\n' \"$$\" {shlex.quote(run_id)} > {shlex.quote(pid_linux)}\n"
             "source /opt/ros/humble/setup.bash\n"
             f"cd -- {shlex.quote(root_linux)}\nexec {shlex.join(argv)}")
    env = [f"ROS_DOMAIN_ID={args.ros_domain_id}", "ROS_LOCALHOST_ONLY=1",
           f"IGN_PARTITION={args.partition}", f"GZ_PARTITION={args.partition}", f"MAICON_RECORD_RUN_ID={run_id}"]
    shell = "exec " + shlex.join(["env", *env, "setsid", "--wait", "bash", "-c", child])
    return ["wsl.exe", "-d", args.distro, "--", "bash", "-lc", shell]


def cleanup_match(distro: str, pid_linux: str, run_id: str):
    """Signal only our recorded, token-verified Linux session. Never pkill Gazebo."""
    script = f"""set -eu
pidfile={shlex.quote(pid_linux)}
token={shlex.quote(run_id)}
for i in {{1..20}}; do [[ -s "$pidfile" ]] && break; sleep .1; done
[[ -s "$pidfile" ]] || {{ echo 'Owned match PID file was not created' >&2; exit 3; }}
read -r pid saved < "$pidfile"
[[ "$saved" == "$token" && "$pid" =~ ^[1-9][0-9]*$ ]] || exit 4
[[ -r "/proc/$pid/environ" ]] || exit 0
tr '\\0' '\\n' < "/proc/$pid/environ" | grep -Fxq -- "MAICON_RECORD_RUN_ID=$token" || exit 5
pgid=$(ps -o pgid= -p "$pid")
pgid=${{pgid//[[:space:]]/}}
[[ "$pgid" == "$pid" ]] || exit 6
kill -s INT -- "-$pid" 2>/dev/null || true
for i in {{1..30}}; do kill -0 -- "-$pid" 2>/dev/null || exit 0; sleep .1; done
kill -s TERM -- "-$pid" 2>/dev/null || true
for i in {{1..20}}; do kill -0 -- "-$pid" 2>/dev/null || exit 0; sleep .1; done
kill -s KILL -- "-$pid" 2>/dev/null || true
"""
    subprocess.run(["wsl.exe", "-d", distro, "--", "bash", "-lc", script],
                   capture_output=True, text=True, encoding="utf-8", check=True, timeout=15)


def crop_frame(pixels, crop, initial_size):
    height, width = pixels.shape[:2]
    if (width, height) != initial_size:
        raise RuntimeError(f"Selected window resized from {initial_size} to {(width, height)}; restart recording")
    x, y, w, h = crop
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
        raise ValueError("--crop must fit inside the selected window (x y width height)")
    result = np.ascontiguousarray(pixels[y:y+h, x:x+w])
    # Preserve all pixels; pad at most 1 black pixel to meet MP4 even dimensions.
    return cv2.copyMakeBorder(result, 0, h % 2, 0, w % 2, cv2.BORDER_CONSTANT, value=(0, 0, 0))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hwnd", type=lambda v: int(v, 0), required=True)
    parser.add_argument("--crop", type=int, nargs=4, metavar=("X", "Y", "WIDTH", "HEIGHT"))
    parser.add_argument("--duration", type=float, default=300.0, help="match simulation seconds")
    parser.add_argument("--base-at", type=float, default=240.0)
    parser.add_argument("--smog-at", type=float, default=120.0)
    parser.add_argument("--uav-schedule", default="", help="e.g. 90:2,160:5,220:off")
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--m3-count", type=int, default=4, help="Must match the generated world")
    parser.add_argument("--uav-layout", choices=("image", "off"), default="image")
    parser.add_argument("--layout", choices=("fixed", "random"), default="fixed")
    parser.add_argument("--physical-robots", choices=("one", "six"), default="one")
    parser.add_argument("--ros-domain-id", type=int, default=188)
    parser.add_argument("--partition", default="maicon_record_20261107_01")
    parser.add_argument("--distro", default="Ubuntu-22.04")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--timeout", type=float, help="wall-time limit; default max(180, duration * 5)")
    parser.add_argument("--output", type=Path, default=ROOT / "ros2/gazebo/recordings/maicon_rover_match.mp4")
    args = parser.parse_args()
    if args.hwnd <= 0 or args.fps <= 0:
        parser.error("--hwnd and --fps must be positive")
    if not all(math.isfinite(v) for v in (args.duration, args.base_at, args.smog_at)):
        parser.error("mission timing must be finite")
    if not 0 < args.base_at < args.duration or not 0 <= args.smog_at < args.duration:
        parser.error("require 0 < --base-at < --duration and 0 <= --smog-at < --duration")
    if not 0 <= args.ros_domain_id <= 232 or not args.partition.strip():
        parser.error("require ROS domain 0..232 and nonempty Gazebo partition")
    timeout = args.timeout if args.timeout is not None else max(180.0, args.duration * 5)
    if not math.isfinite(timeout) or timeout <= 0:
        parser.error("--timeout must be finite and positive")

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    report, log = output.with_suffix(".json"), output.with_suffix(".log")
    stamps, metadata = output.with_suffix(".frames.jsonl"), output.with_suffix(".recording.json")
    run_id = uuid.uuid4().hex
    pidfile = output.with_name(f"{output.stem}.{run_id}.pid")
    for path in (output, report, log, stamps, metadata):
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite recording artifact: {path}; choose a new --output")
    root_linux, report_linux, pid_linux = (wsl_path(p, args.distro) for p in (ROOT, report, pidfile))
    command = match_command(args, root_linux, report_linux, pid_linux, run_id)
    capture = WindowCapture(args.hwnd, args.fps)
    writer = process = None
    frames = 0
    wall_start = time.monotonic()
    summary = {"output": str(output), "report": str(report), "log": str(log), "timestamps": str(stamps),
               "run_id": run_id, "pidfile": str(pidfile), "hwnd": args.hwnd, "capture_backend": "WGC window_hwnd",
               "ros_domain_id": args.ros_domain_id, "ros_localhost_only": 1, "partition": args.partition,
               "physical_robots": args.physical_robots, "fps": args.fps, "duration_sim_s": args.duration,
               "base_at_sim_s": args.base_at, "smog_at_sim_s": args.smog_at,
               "uav_schedule": args.uav_schedule, "wall_timeout_s": timeout,
               "status": "initializing", "exit_code": None}
    error = None
    try:
        initial = capture.initial_frame()
        h, w = initial[0].shape[:2]
        initial_size = (w, h)
        crop = tuple(args.crop) if args.crop else (0, 0, w, h)
        encoded = crop_frame(initial[0], crop, initial_size)
        video_size = (encoded.shape[1], encoded.shape[0])
        summary.update(window_size=list(initial_size), crop=list(crop), video_size=list(video_size))
        writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, video_size)
        if not writer.isOpened():
            raise RuntimeError("OpenCV could not open an MP4 encoder")
        with log.open("x", encoding="utf-8") as stream, stamps.open("x", encoding="utf-8") as timing:
            wall_start = time.monotonic()
            summary["started_utc"] = datetime.now(timezone.utc).isoformat()
            print(f"Recording HWND {args.hwnd} to {output}; report={report}", flush=True)
            process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
            next_frame = wall_start
            while process.poll() is None:
                if time.monotonic() - wall_start > timeout:
                    raise TimeoutError(f"Match exceeded {timeout:.1f}s recording wall-time limit")
                pixels, captured_mono, captured_utc, source_stamp, sequence = capture.snapshot()
                writer.write(crop_frame(pixels, crop, initial_size))
                timing.write(json.dumps({"frame": frames, "video_time_s": frames / args.fps,
                                         "write_wall_elapsed_s": time.monotonic() - wall_start,
                                         "capture_wall_elapsed_s": captured_mono - wall_start,
                                         "capture_utc_unix_s": captured_utc,
                                         "wgc_timespan_raw": source_stamp, "source_sequence": sequence}) + "\n")
                timing.flush()
                frames += 1
                next_frame += 1 / args.fps
                time.sleep(max(0, next_frame - time.monotonic()))
            summary["exit_code"] = process.returncode
            if process.returncode:
                raise RuntimeError(f"Match exited with code {process.returncode}; see {log}")
            if not report.exists():
                raise RuntimeError(f"Match exited without writing report: {report}")
            summary["status"] = "complete"
    except BaseException as exc:
        error = exc
        summary["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        summary["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if process is not None and process.poll() is None:
            try:
                cleanup_match(args.distro, pid_linux, run_id)
            except Exception as exc:
                summary["cleanup_error"] = f"Owned Linux match cleanup failed: {exc}"
                print(summary["cleanup_error"], flush=True)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()  # Only our WSL client, never another WSL/Gazebo process.
                process.wait(timeout=5)
            summary["exit_code"] = process.returncode
        if writer is not None:
            writer.release()
        capture.close()
        wall_elapsed = time.monotonic() - wall_start
        summary.update(frames=frames, video_duration_s=frames / args.fps, wall_elapsed_s=wall_elapsed,
                       video_wall_ratio=(frames / args.fps / wall_elapsed if wall_elapsed else None),
                       ended_utc=datetime.now(timezone.utc).isoformat())
        metadata.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    if error is not None:
        raise error
    print(f"Recorded {frames} frames ({frames / args.fps:.1f}s video / {wall_elapsed:.1f}s wall); metadata={metadata}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
