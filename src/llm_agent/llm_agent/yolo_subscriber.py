"""Subscribes to /obj_data and buffers recent YOLO detection frames."""

import json
import threading
from collections import deque
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String


class YoloSubscriber:
    """
    Maintains a rolling buffer of recent YOLO detection frames.

    Thread-safe. Used by phase loops to query detections without
    blocking the ROS2 spin thread.
    """

    TOPIC = '/obj_data'

    def __init__(self, node: Node, buffer_size: int = 30):
        self._lock = threading.Lock()
        self._buffer: deque[dict] = deque(maxlen=buffer_size)
        self._event = threading.Event()

        node.create_subscription(String, self.TOPIC, self._callback, 10)

    def _callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._buffer.append(data)
            self._event.set()

    def latest(self) -> Optional[dict]:
        with self._lock:
            return self._buffer[-1] if self._buffer else None

    def recent(self, n: int = 10) -> list[dict]:
        with self._lock:
            return list(self._buffer)[-n:]

    def wait_for_detection(
        self,
        class_name: str,
        confidence_threshold: float,
        timeout_sec: float,
    ) -> Optional[dict]:
        """
        Block until a detection matching class_name is received or timeout.
        Returns the matching frame dict, or None on timeout.
        """
        import time
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            self._event.wait(timeout=min(remaining, 0.1))
            self._event.clear()
            with self._lock:
                frames = list(self._buffer)
            for frame in reversed(frames):
                if (frame.get('class_name') == class_name
                        and frame.get('confidence', 0.0) >= confidence_threshold):
                    return frame
        return None

    def clear(self) -> None:
        with self._lock:
            self._buffer.clear()
            self._event.clear()
