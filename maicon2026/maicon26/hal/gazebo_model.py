"""Frame geometry of the local Fortress SDF, not measured robot calibration."""

BASE_FRAME = "base_link"
LIDAR_FRAME = "gazebo_lidar"
LIDAR_XYZ = (0.039815, 0.0, 0.16)
LIDAR_RPY = (0.0, 0.0, 0.0)


def lidar_frame(topic_prefix: str = "") -> str:
    prefix = topic_prefix.strip("/")
    return f"{prefix}/{LIDAR_FRAME}" if prefix else LIDAR_FRAME
