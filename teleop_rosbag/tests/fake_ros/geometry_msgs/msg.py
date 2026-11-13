class _V:
    def __init__(self):
        self.x = self.y = self.z = 0.0


class Twist:
    def __init__(self):
        self.linear, self.angular = _V(), _V()
