"""Publishes /moveit_command JSON and waits for /moveit_status response."""

import json
import threading
import time
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String


class MoveItClient:
    """
    Topic-based interface to the external MoveIt Module.

    Publishes JSON commands to /moveit_command.
    Subscribes to /moveit_status for completion feedback.

    send_and_wait() is a blocking call designed to run inside
    the ThreadPoolExecutor (not on the ROS2 spin thread).
    """

    CMD_TOPIC = '/moveit_command'
    STATUS_TOPIC = '/moveit_status'

    def __init__(self, node: Node):
        self._pub = node.create_publisher(String, self.CMD_TOPIC, 10)
        self._last_status: Optional[dict] = None
        self._status_event = threading.Event()
        self._lock = threading.Lock()

        node.create_subscription(String, self.STATUS_TOPIC, self._status_callback, 10)

    def _status_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._last_status = data
            self._status_event.set()

    def send_and_wait(self, command: dict, timeout_sec: float = 10.0) -> dict:
        """
        Publish command and block until /moveit_status received or timeout.

        Returns:
            {'success': bool, 'error_message': str, ...}
        """
        self._status_event.clear()
        with self._lock:
            self._last_status = None

        msg = String()
        msg.data = json.dumps(command)
        self._pub.publish(msg)

        fired = self._status_event.wait(timeout=timeout_sec)
        if not fired:
            return {'success': False, 'error_message': 'moveit_status timeout'}

        with self._lock:
            return self._last_status or {'success': False, 'error_message': 'empty status'}

    def send(self, command: dict) -> None:
        """Fire-and-forget publish (used for HOME after PLACE)."""
        msg = String()
        msg.data = json.dumps(command)
        self._pub.publish(msg)
