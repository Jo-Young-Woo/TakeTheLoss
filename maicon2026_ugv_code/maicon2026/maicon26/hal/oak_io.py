"""OAK 컬러 메시지 → BGR 배열 (6-3). ROS 를 import 하지 않는 순수 함수라 시험이 쉽다.

  - sensor_msgs/Image: 기존 hal.ros_mil.image_to_bgr (bgr8 은 그대로, rgb8 은 뒤집는다 — 결과는 항상 BGR).
  - sensor_msgs/CompressedImage: cv2.imdecode (결과가 BGR).
YOLO 입력에 RGB 변환을 넣지 않는다 — 여기서 BGR 로 끝난다. (화면 표시용 BGR→RGB 는 노트북에서만.)
"""
from __future__ import annotations

from typing import Any

import numpy as np


def compressed_to_bgr(msg: Any) -> np.ndarray:
    import cv2
    buf = np.frombuffer(bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray, np.ndarray)) else msg.data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("압축 영상을 해석하지 못했다")
    return img


def oak_msg_to_bgr(msg: Any, compressed: bool) -> np.ndarray:
    if compressed:
        return compressed_to_bgr(msg)
    from .ros_mil import image_to_bgr
    return image_to_bgr(msg)
