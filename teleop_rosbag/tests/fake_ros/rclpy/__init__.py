"""시험·연습용 가짜 rclpy. 로봇 없이 teleop_keys.py 의 키 처리와 종료 동작을 본다.

  PYTHONPATH=tests/fake_ros python3 teleop_keys.py --no-state-guard

FAKE_ROS_LOG 에 파일 경로를 주면 발행한 메시지를 JSON 한 줄씩 남긴다.
FAKE_ROS_MODE(practice|mission), FAKE_ROS_OTHERS(다른 /cmd_vel 송신자 수)로 상황을 바꾼다.
"""
import json
import os
import time

_LOG = os.environ.get("FAKE_ROS_LOG", "")


def _log(d):
    if _LOG:
        with open(_LOG, "a", encoding="utf-8") as f:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")


class _Pub:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        d = {"t": round(time.time(), 3), "topic": self.topic}
        if hasattr(msg, "linear"):
            d.update(v=msg.linear.x, w=msg.angular.z)
        elif hasattr(msg, "data"):
            d.update(data=msg.data)
        else:
            d.update(line=getattr(msg, "line", None), text=getattr(msg, "text", None))
        _log(d)


class _Future:
    def __init__(self, result):
        self._r, self._cbs = result, []

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        _PENDING.append(self)

    def exception(self):
        return None

    def result(self):
        return self._r


_PENDING = []


class _Client:
    def __init__(self, cls, name):
        self.cls, self.name = cls, name

    def service_is_ready(self):
        return True

    def call_async(self, req):
        _log({"t": round(time.time(), 3), "service": self.name, "req": {k: v for k, v in vars(req).items()}})
        return _Future(self.cls.Response())


class Node:
    def __init__(self, name):
        self.name, self._subs = name, {}

    def create_publisher(self, cls, topic, qos):
        return _Pub(topic)

    def create_subscription(self, cls, topic, cb, qos):
        self._subs[topic] = (cls, cb)

    def create_client(self, cls, name):
        return _Client(cls, name)

    def count_publishers(self, topic):
        return 1 + int(os.environ.get("FAKE_ROS_OTHERS", "0"))

    def get_topic_names_and_types(self):
        return [("/mil/match", ["fake_mil/msg/Match"]), ("/mil/state", ["fake_mil/msg/State"]), ("/mil/oled", ["fake_mil/msg/OledLine"])]

    def get_service_names_and_types(self):
        return [("/mil/gimbal", ["fake_mil/srv/Gimbal"]), ("/mil/fire", ["fake_mil/srv/Fire"])]

    def get_publishers_info_by_topic(self, name):
        return []

    def destroy_node(self):
        pass


_NODE = None


def init(*a, **k):
    pass


def create_node(name):
    global _NODE
    _NODE = Node(name)
    return _NODE


def spin_once(node, timeout_sec=0.0):
    if _PENDING:
        f = _PENDING.pop(0)
        for cb in f._cbs:
            cb(f)
    mode = os.environ.get("FAKE_ROS_MODE", "practice")
    sub = node._subs.get("/mil/match")
    if sub and mode:
        cls, cb = sub
        m = cls()
        m.mode, m.cmd = mode, "idle"
        cb(m)


def shutdown():
    pass
