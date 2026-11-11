"""하드웨어 반영 시험이 같이 쓰는 가짜 시계·transport·robot. 시뮬을 쓰지 않는다."""
from maicon26.config import Config
from maicon26.hal.wingbot import Transport, WingbotRobot


class Clock:
    t = 0.0
    calls = 0

    def __call__(self):
        self.calls += 1
        return self.t

    def sleep(self, dt):
        self.t += dt


class FakeTransport(Transport):
    """fire_replies: 호출마다 (message, left_ms) 를 하나씩 쓴다. 비면 ('fired', 10000)."""

    def __init__(self, fire_replies=None, gimbal_reply="ok"):
        self.cmds, self.gimbal, self.fire, self.oled = [], [], [], []
        self.fire_replies = list(fire_replies or [])
        self.gimbal_reply = gimbal_reply
        self.gimbal_pending = None            # 설정하면 응답을 바로 주지 않고 여기에 콜백을 쌓는다

    def publish_cmd(self, v, w):
        self.cmds.append((v, w))

    def publish_oled(self, line, text):
        self.oled.append((line, text))

    def call_gimbal(self, pan, tilt, speed, acc, cb):
        self.gimbal.append((pan, tilt))
        if self.gimbal_pending is not None:
            self.gimbal_pending.append(cb)
        else:
            cb(self.gimbal_reply)

    def call_fire(self, cb):
        self.fire.append(1)
        msg, left = self.fire_replies.pop(0) if self.fire_replies else ("fired", 10000)
        cb(msg, left)


def make_robot(mode="practice", cmd="idle", flags=(), cfg=None, transport=None):
    clock = Clock()
    tr = transport or FakeTransport()
    robot = WingbotRobot(cfg or Config(), transport=tr, clock=clock)
    robot.on_match(mode, cmd)
    if flags:
        robot.set_degraded(flags)
    return robot, tr, clock
