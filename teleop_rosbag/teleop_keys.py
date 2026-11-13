#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""방향키 텔레옵 (연습·데이터 수집용. 경기 중에는 쓰지 않는다).

  python3 teleop_keys.py                       # 주행 + 팬틸트
  python3 teleop_keys.py --level 2             # 2단계(0.24 m/s)로 시작
  python3 teleop_keys.py --allow-fire          # 발사 허용(F 두 번). 사람 눈 방향 발사 금지

로봇(젯슨)의 ROS 2 Humble 환경에서 SSH 터미널로 실행한다. 팀 노드(team_node.py)는 먼저 끈다.
명령 규칙과 안전장치는 teleop_core.py 에 있고, 이 파일은 ROS 와 터미널을 잇는 부분이다.
메시지·서비스 타입(/mil/state, /mil/match, /mil/oled, /mil/gimbal, /mil/fire)은 패키지 이름을 모르므로
maicon26/hal/ros_mil.py 와 같은 방법으로 실행 중인 그래프에서 알아낸다. 안 되면 --type 으로 직접 적는다.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import shutil
import signal
import sys
import time
import unicodedata

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_core import HELP, OTHER_PUBLISHER, Options, Teleop, V_CAP, W_CAP  # noqa: E402


def response_text(res):
    """서비스 응답에서 ok/locked/fired… 문자열을 꺼낸다(필드 이름이 자료에 없다)."""
    for name in ("message", "result", "status", "msg", "state"):
        v = getattr(res, name, None)
        if isinstance(v, str) and v:
            return v.strip().lower()
    return None


class RosIO:
    def __init__(self, node, a) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.node, self.a = node, a
        self._Twist, self._String = Twist, String
        ns = ("/" + a.ns.strip("/")) if a.ns.strip("/") else ""
        self.names = {"cmd": ns + a.cmd_topic, "odom": ns + a.odom_topic, "state": ns + "/mil/state", "match": ns + "/mil/match",
                      "oled": ns + "/mil/oled", "gimbal": ns + "/mil/gimbal", "fire": ns + "/mil/fire"}
        self.type_override = dict(t.split("=", 1) for t in a.type)
        self.mode = self.cmd = ""
        self.battery = self.hp = None
        self.odom_v = None
        self._ready = {k: False for k in ("state", "match", "oled", "gimbal", "fire")}
        self._pub_oled = self._oled_cls = self._cli_gimbal = self._gimbal_cls = self._cli_fire = self._fire_cls = None
        self._t_oled = 0.0
        self._log = open(a.event_log, "a", encoding="utf-8") if a.event_log else None
        self.pub_cmd = node.create_publisher(Twist, self.names["cmd"], 10)
        self.pub_evt = node.create_publisher(String, "/teleop/events", 10)
        node.create_subscription(Odometry, self.names["odom"], self._on_odom, qos_profile_sensor_data)

    # ------------------------------------------------------------------ 타입 찾기
    def _type_of(self, key, services=False):
        tstr = self.type_override.get(key)
        if tstr is None:
            pairs = dict(self.node.get_service_names_and_types() if services else self.node.get_topic_names_and_types())
            tl = pairs.get(self.names[key])
            tstr = tl[0] if tl else None
        if not tstr:
            return None
        try:
            from rosidl_runtime_py.utilities import get_message, get_service
            return get_service(tstr) if services else get_message(tstr)
        except Exception:
            return None

    def _state_qos(self, name):
        """송신자가 내는 QoS 에 맞춘다. 자료대로면 depth 1 · TRANSIENT_LOCAL · RELIABLE 이다."""
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        dur, rel = QoSDurabilityPolicy.TRANSIENT_LOCAL, QoSReliabilityPolicy.RELIABLE
        try:
            for info in self.node.get_publishers_info_by_topic(name):
                if info.qos_profile.durability == QoSDurabilityPolicy.VOLATILE:
                    dur = QoSDurabilityPolicy.VOLATILE
                if info.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT:
                    rel = QoSReliabilityPolicy.BEST_EFFORT
        except Exception:
            pass
        return QoSProfile(depth=1, durability=dur, reliability=rel)

    def discover(self) -> None:
        for key, cb in (("state", self._on_state), ("match", self._on_match)):
            if not self._ready[key]:
                cls = self._type_of(key)
                if cls is not None:
                    self.node.create_subscription(cls, self.names[key], cb, self._state_qos(self.names[key]))
                    self._ready[key] = True
        if not self._ready["oled"] and not self.a.no_oled:
            cls = self._type_of("oled")
            if cls is not None:
                self._pub_oled, self._oled_cls, self._ready["oled"] = self.node.create_publisher(cls, self.names["oled"], 10), cls, True
        for key in ("gimbal", "fire"):
            if not self._ready[key]:
                cls = self._type_of(key, services=True)
                if cls is not None:
                    setattr(self, "_cli_" + key, self.node.create_client(cls, self.names[key]))
                    setattr(self, "_" + key + "_cls", cls)
                    self._ready[key] = True

    # ------------------------------------------------------------------ 수신
    @staticmethod
    def _get(m, *names):
        for n in names:
            v = getattr(m, n, None)
            if v is not None:
                return v
        return None

    def _on_odom(self, m) -> None:
        self.odom_v = float(m.twist.twist.linear.x)

    def _on_state(self, m) -> None:
        self.hp, self.battery = self._get(m, "hp"), self._get(m, "battery_pct", "battery")
        mode, cmd = self._get(m, "mode"), self._get(m, "cmd")     # mode/cmd 가 /mil/state 에 같이 오는 구현도 받는다
        if mode is not None or cmd is not None:
            self.mode, self.cmd = str(mode or ""), str(cmd or "")

    def _on_match(self, m) -> None:
        self.mode, self.cmd = str(self._get(m, "mode") or ""), str(self._get(m, "cmd") or "")

    # ------------------------------------------------------------------ 송신
    def other_cmd_publishers(self) -> int:
        return max(0, self.node.count_publishers(self.names["cmd"]) - 1)

    def publish_cmd(self, v: float, w: float) -> None:
        t = self._Twist()
        t.linear.x, t.angular.z = float(max(-V_CAP, min(V_CAP, v))), float(max(-W_CAP, min(W_CAP, w)))
        self.pub_cmd.publish(t)

    def event(self, d: dict) -> None:
        """rosbag2(Humble)는 서비스를 기록하지 않는다. 그래서 요청·응답·마커를 토픽과 파일로도 남긴다."""
        line = json.dumps(dict({"t": round(time.time(), 3)}, **d), ensure_ascii=False)
        try:
            self.pub_evt.publish(self._String(data=line))
        except Exception:
            pass
        if self._log is not None:
            self._log.write(line + "\n")
            self._log.flush()

    def call_gimbal(self, pan, tilt, speed, acc, cb) -> bool:
        cli = self._cli_gimbal
        if cli is None or not cli.service_is_ready():
            return False
        req = self._gimbal_cls.Request()
        req.pan, req.tilt, req.speed, req.acc = float(pan), float(tilt), float(speed), float(acc)
        cli.call_async(req).add_done_callback(
            lambda f: cb(response_text(f.result()) if f.exception() is None and f.result() is not None else None))
        return True

    def call_fire(self, cb) -> bool:
        cli = self._cli_fire
        if cli is None or not cli.service_is_ready():
            return False

        def done(f) -> None:
            if f.exception() is not None or f.result() is None:
                cb(None, None)
                return
            left = getattr(f.result(), "left_ms", None)
            try:
                left = float(left) if left is not None else None
            except (TypeError, ValueError, OverflowError):
                left = None
            cb(response_text(f.result()), left)

        cli.call_async(self._fire_cls.Request()).add_done_callback(done)
        return True

    def oled(self, line: int, text: str, now: float) -> None:
        if self._pub_oled is None or now - self._t_oled < 0.6:      # 자료: 전송 간격 0.5 s 이상
            return
        self._t_oled = now
        try:
            self._pub_oled.publish(self._oled_cls(line=int(line), text=str(text)[:21]))
        except Exception:
            pass

    def close(self) -> None:
        if self._log is not None:
            self._log.close()


# ============================================================================ 화면
def clip(s: str, cols: int) -> str:
    out, w = [], 0
    for ch in s:
        cw = 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
        if w + cw > cols:
            break
        out.append(ch)
        w += cw
    return "".join(out)


class Screen:
    def __init__(self) -> None:
        self.line = ""

    def say(self, text: str) -> None:
        sys.stdout.write("\r\x1b[2K" + text + "\n")
        self.draw(self.line)

    def draw(self, line: str) -> None:
        self.line = line
        cols = shutil.get_terminal_size((100, 24)).columns
        if cols < 20:                                          # 크기를 모르는 터미널은 0 을 돌려준다(파이썬 3.10)
            cols = 100
        sys.stdout.write("\r\x1b[2K" + clip(line, cols - 1))
        sys.stdout.flush()


def rec_label(path: str) -> str:
    if not path:
        return ""
    try:
        with open(path, encoding="utf-8") as f:
            s = f.read().strip()
    except OSError:
        return "REC ?"
    return "● REC" if s.startswith("RECORDING") else "■ 기록 멈춤(" + s[:40] + ")"


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="방향키 텔레옵 (연습·데이터 수집용)")
    ap.add_argument("--level", type=int, default=1, choices=[1, 2, 3, 4, 5], help="시작 속도 단계 (1 = 0.12 m/s, 0.6 rad/s)")
    ap.add_argument("--max-lin", type=float, default=V_CAP, help="5단계 직진 속도 [m/s]. 0.6 보다 크게는 안 된다")
    ap.add_argument("--max-ang", type=float, default=W_CAP, help="5단계 회전 속도 [rad/s]. 3.0 보다 크게는 안 된다")
    ap.add_argument("--first-hold", type=float, default=0.6, help="첫 입력 뒤 키 반복을 기다리는 시간 [s]. 터미널의 반복 지연보다 길게")
    ap.add_argument("--release", type=float, default=0.2, help="반복이 이만큼 끊기면 손을 뗀 것으로 본다 [s]")
    ap.add_argument("--allow-fire", action="store_true", help="F 두 번으로 레이저 발사 허용. 사람 눈 방향 발사 금지")
    ap.add_argument("--no-state-guard", action="store_true", help="모드(practice/mission)를 못 받아도 주행한다. 상태 토픽이 없는 환경에서만")
    ap.add_argument("--idle-exit", type=float, default=120.0, help="이 시간 동안 키가 없으면 끝낸다 [s]")
    ap.add_argument("--ns", default="", help="로봇 네임스페이스(시뮬 다중 로봇)")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--type", action="append", default=[], metavar="키=pkg/msg/Name",
                    help="타입을 직접 지정. 키: state | match | oled | gimbal | fire (서비스는 pkg/srv/Name)")
    ap.add_argument("--no-oled", action="store_true", help="OLED 에 TELEOP 표시를 하지 않는다")
    ap.add_argument("--event-log", default="", help="이벤트(JSON 한 줄씩)를 이 파일에도 남긴다")
    ap.add_argument("--rec-status-file", default="", help="record.sh 의 상태 파일. 화면에 기록 여부를 보여 준다")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    if not sys.stdin.isatty():
        print("터미널에서 실행해야 한다(키 입력을 읽는다).")
        return 2
    import termios
    import tty
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)     # 종료할 때 0 을 보낼 수 있게 신호는 직접 다룬다
    except Exception:
        rclpy.init()
    node = rclpy.create_node("teleop_keys")
    ros = RosIO(node, a)
    scr = Screen()
    opt = Options(max_lin=min(abs(a.max_lin), V_CAP), max_ang=min(abs(a.max_ang), W_CAP), level=a.level, first_hold_s=a.first_hold,
                  release_s=a.release, state_guard=not a.no_state_guard, allow_fire=a.allow_fire, idle_exit_s=a.idle_exit)
    tele = Teleop(ros, opt, time.monotonic(), say=scr.say)
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda s, f: tele.request_quit("신호 " + signal.Signals(s).name))
    print("텔레옵 시작. 팀 노드가 꺼져 있어야 한다. 충전 케이블을 빼고 넓은 곳에서 1단계로 시작한다.")
    for line in HELP:
        print("  " + line)
    if a.allow_fire:
        print("  ※ 발사가 켜져 있다. 사람 눈 방향으로 쏘지 않는다. 주변을 확인한다.")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    t_draw = t_oled = 0.0
    try:
        tty.setcbreak(fd)
        ros.event({"type": "start", "level": opt.level, "allow_fire": opt.allow_fire})
        while tele.quit is None:
            ready, _, _ = select.select([fd], [], [], 0.02)
            now = time.monotonic()
            if ready:
                data = os.read(fd, 64)
                if not data:
                    tele.request_quit("입력이 끊겼다")
                    break
                tele.on_bytes(data, now)
            for _ in range(3):                                # 오도메트리 20 Hz · 상태 2 Hz · 서비스 응답을 받기에 충분하다
                rclpy.spin_once(node, timeout_sec=0.0)
            tele.tick(now)
            if now - t_draw >= 0.2:
                t_draw = now
                scr.draw(tele.status(rec_label(a.rec_status_file)))
            if now - t_oled >= 1.0:
                t_oled = now
                ros.oled(0, "TELEOP", now)
    except KeyboardInterrupt:
        tele.request_quit("Ctrl-C")
    except Exception as e:                                    # 어떤 예외든 아래에서 정지 명령을 보낸다
        tele.request_quit("오류 " + type(e).__name__ + ": " + str(e)[:80])
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sent = 0
        if tele.send_zero_on_exit():
            for _ in range(5):                                # 정지 명령 5번. 못 보내도 로봇은 0.5 s 뒤 스스로 선다
                try:
                    ros.publish_cmd(0.0, 0.0)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    sent += 1
                except Exception:
                    pass
                time.sleep(0.1)
        try:
            ros.event({"type": "end", "why": tele.quit or "", "zero_sent": sent})
            ros._t_oled = 0.0
            ros.oled(0, "TELEOP END", time.monotonic())
            rclpy.spin_once(node, timeout_sec=0.05)
        except Exception:
            pass
        ros.close()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        print("\r\x1b[2K텔레옵 종료: " + (tele.quit or "") + (f" (정지 명령 {sent}회)" if sent else " (정지 명령 안 보냄)"))
    return 3 if tele.quit_kind == OTHER_PUBLISHER else 0


if __name__ == "__main__":
    sys.exit(main())
