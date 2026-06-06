"""Service client for /moveit/execute (llm_agent_msgs/MoveItExecute).

agent=client, MoveIt Module=server. 명령별 파라미터는 params_json 에
JSON 직렬화하여 전달한다. call_and_wait()은 ThreadPoolExecutor 워커
스레드에서 호출되며(스핀 스레드 아님), call_async + future 대기로 동작한다.
"""

import json
import threading
from typing import Optional

from rclpy.node import Node

from llm_agent_msgs.srv import MoveItExecute


class MoveItClient:
    """Synchronous wrapper over the /moveit/execute service."""

    DEFAULT_SERVICE = '/moveit/execute'

    def __init__(self, node: Node, service_name: str = DEFAULT_SERVICE):
        self._node = node
        self._client = node.create_client(MoveItExecute, service_name)
        self._service_name = service_name

    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def call_and_wait(
        self,
        cmd: str,
        params: Optional[dict] = None,
        timeout_sec: float = 10.0,
    ) -> dict:
        """
        Call /moveit/execute and block until the response or timeout.

        Returns:
            {'success': bool, 'error_code': int, 'error_message': str}
        """
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=min(timeout_sec, 5.0)):
                return {
                    'success': False,
                    'error_code': -1,
                    'error_message': f'service {self._service_name} unavailable',
                }

        request = MoveItExecute.Request()
        request.cmd = cmd
        request.params_json = json.dumps(params or {})

        future = self._client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())

        if not done.wait(timeout=timeout_sec):
            return {
                'success': False,
                'error_code': -1,
                'error_message': f'service call timeout ({cmd})',
            }

        if future.exception() is not None:
            return {
                'success': False,
                'error_code': -1,
                'error_message': f'service exception: {future.exception()}',
            }

        resp = future.result()
        return {
            'success': resp.success,
            'error_code': resp.error_code,
            'error_message': resp.error_message,
        }
