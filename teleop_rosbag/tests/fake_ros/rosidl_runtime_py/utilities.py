class _Msg:
    def __init__(self, **k):
        self.__dict__.update(k)


class _Gimbal:
    class Request:
        def __init__(self):
            self.pan = self.tilt = self.speed = self.acc = 0.0

    class Response:
        message = "ok"


class _Fire:
    class Request:
        pass

    class Response:
        message, left_ms = "fired", 0


def get_message(tstr):
    return _Msg


def get_service(tstr):
    return _Gimbal if tstr.endswith("Gimbal") else _Fire
