class QoSDurabilityPolicy:
    TRANSIENT_LOCAL, VOLATILE = 1, 2


class QoSReliabilityPolicy:
    RELIABLE, BEST_EFFORT = 1, 2


class QoSProfile:
    def __init__(self, **k):
        self.__dict__.update(k)


qos_profile_sensor_data = QoSProfile(depth=5)
