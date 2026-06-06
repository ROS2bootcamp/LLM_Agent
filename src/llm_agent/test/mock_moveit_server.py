"""Mock MoveIt service server for E2E testing — /moveit/execute.

실제 MoveIt 모듈 없이 에이전트 전체 Phase 흐름을 검증하기 위한 mock 서버.
모든 cmd 에 success 응답을 주며, 파라미터 fail_cmds 로 특정 cmd 를 실패시킬 수 있다.

실행 (워크스페이스 source 후):
    ros2 run llm_agent mock_moveit_server
    # 특정 명령 실패 주입:
    ros2 run llm_agent mock_moveit_server --ros-args -p fail_cmds:="['place']"
"""

import json

import rclpy
from rclpy.node import Node

from llm_agent_msgs.srv import MoveItExecute


class MockMoveItServer(Node):
    def __init__(self):
        super().__init__('mock_moveit_server')
        self.declare_parameter('fail_cmds', [''])  # 실패시킬 cmd 목록
        self._srv = self.create_service(MoveItExecute, '/moveit/execute', self._cb)
        self.get_logger().info('Mock MoveIt server ready on /moveit/execute')

    def _cb(self, req: MoveItExecute.Request, resp: MoveItExecute.Response):
        fail_cmds = [c for c in (self.get_parameter('fail_cmds').value or []) if c]
        try:
            params = json.loads(req.params_json or '{}')
        except json.JSONDecodeError:
            params = {}
        self.get_logger().info(f'recv cmd="{req.cmd}" param_keys={list(params)}')

        if req.cmd in fail_cmds:
            resp.success = False
            resp.error_code = -1
            resp.error_message = f'mock injected failure for "{req.cmd}"'
            self.get_logger().warn(resp.error_message)
        else:
            resp.success = True
            resp.error_code = 0
            resp.error_message = ''
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = MockMoveItServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
