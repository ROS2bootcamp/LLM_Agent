"""Subscribes to /vision/detection_results and buffers recent YOLO frames.

ROBOT_VISION 발행 포맷 (std_msgs/String, JSON):
    {
      "timestamp_ns": 1718001234567890,
      "num_detections": 1,
      "objects": [
        {
          "class_name": "cup",
          "confidence": 0.892,
          "center_2d": {"u": 320, "v": 240},
          "distance_m": 0.452,
          "position_3d_camera_frame": {"X": 0.12, "Y": -0.04, "Z": 0.45},
          "position_3d_base_frame":   {"X": 0.40, "Y": 0.10,  "Z": 0.06}
        }
      ]
    }
좌표 키는 대문자 X/Y/Z. 타겟 좌표는 position_3d_base_frame 사용
(YOLO가 camera_link->base_link TF 변환 완료한 값).
"""

import json
import threading
import time
from collections import deque
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String


class YoloSubscriber:
    """
    Maintains a rolling buffer of recent YOLO detection frames.

    Thread-safe. Phase loops query detections without blocking the
    ROS2 spin thread.
    """

    DEFAULT_TOPIC = '/vision/detection_results'

    def __init__(self, node: Node, topic: str = DEFAULT_TOPIC, buffer_size: int = 60):
        self._lock = threading.Lock()
        self._buffer: deque[dict] = deque(maxlen=buffer_size)
        self._event = threading.Event()

        node.create_subscription(String, topic, self._callback, 10)

    def _callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict) or 'objects' not in data:
            return
        with self._lock:
            self._buffer.append(data)
            self._event.set()

    # ------------------------------------------------------------------
    # Frame access
    # ------------------------------------------------------------------

    def latest(self) -> Optional[dict]:
        """Return the most recent full frame (wrapper dict), or None."""
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def recent(self, n: int = 10) -> list[dict]:
        """Return the last n full frames (wrapper dicts) — for P3 LLM input."""
        with self._lock:
            return list(self._buffer)[-n:]

    # ------------------------------------------------------------------
    # Detection query
    # ------------------------------------------------------------------

    SELECTION_POLICIES = ('highest_conf', 'nearest')

    def wait_for_detection(
        self,
        class_name: str,
        confidence_threshold: float,
        timeout_sec: float,
        max_distance_m: Optional[float] = None,
        selection_policy: str = 'highest_conf',
    ) -> Optional[dict]:
        """
        Block until an object matching class_name (and optional distance bound)
        is seen, or timeout. Returns the matching object dict (single detection,
        with 'timestamp_ns' injected), or None on timeout.

        같은 class 가 한 프레임에 여러 개면 selection_policy 로 하나를 결정한다:
        'highest_conf'(기본, 최고 confidence) | 'nearest'(min distance_m).
        """
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self._event.wait(timeout=min(remaining, 0.1))
            self._event.clear()
            with self._lock:
                frames = list(self._buffer)
            for frame in reversed(frames):
                obj = self._match(
                    frame, class_name, confidence_threshold,
                    max_distance_m, selection_policy,
                )
                if obj is not None:
                    return obj
        return None

    @staticmethod
    def _match(
        frame: dict,
        class_name: str,
        confidence_threshold: float,
        max_distance_m: Optional[float],
        selection_policy: str = 'highest_conf',
    ) -> Optional[dict]:
        candidates = []
        for obj in frame.get('objects', []):
            if obj.get('class_name') != class_name:
                continue
            if obj.get('confidence', 0.0) < confidence_threshold:
                continue
            distance = obj.get('distance_m', -1.0)
            if distance is None or distance <= 0:
                continue
            if max_distance_m is not None and distance > max_distance_m:
                continue
            # D9: reject objects whose base_link coordinates could not be computed
            # (TF lookup failed in ROBOT_VISION — guard before P2 uses them).
            base = obj.get('position_3d_base_frame') or {}
            if base.get('X') is None:
                continue
            candidates.append(obj)
        if not candidates:
            return None

        if selection_policy == 'nearest':
            best = min(candidates, key=lambda o: o.get('distance_m', float('inf')))
        else:  # 'highest_conf' (default / 미지원 정책 fallback)
            best = max(candidates, key=lambda o: o.get('confidence', 0.0))

        result = dict(best)
        result['timestamp_ns'] = frame.get('timestamp_ns')
        return result

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()
            self._event.clear()
