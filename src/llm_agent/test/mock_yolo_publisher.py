"""Mock YOLO publisher for E2E testing — /vision/detection_results.

ROBOT_VISION 노드 없이 에이전트의 P1 탐지 / P2 grip 검증 / P3 수집을 돌리기 위한
mock 발행기. ROBOT_VISION 포맷(JSON, 대문자 X/Y/Z, objects[]) 그대로 발행한다.

distance_m 기본값은 grip 임계값(0.15)보다 작은 0.1 로 두어 P1 탐지와 P2 grip
검증이 모두 통과하도록 한다.

실행 (워크스페이스 source 후):
    ros2 run llm_agent mock_yolo_publisher
    ros2 run llm_agent mock_yolo_publisher --ros-args -p class_name:=bottle -p distance_m:=0.1
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockYoloPublisher(Node):
    def __init__(self):
        super().__init__('mock_yolo_publisher')
        self.declare_parameter('class_name', 'cup')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('distance_m', 0.1)
        self.declare_parameter('confidence', 0.92)
        self.declare_parameter('base_xyz', [0.4, 0.1, 0.06])

        self._pub = self.create_publisher(String, '/vision/detection_results', 10)
        rate = float(self.get_parameter('rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Mock YOLO publishing /vision/detection_results @ {rate}Hz '
            f'(class={self.get_parameter("class_name").value})'
        )

    def _tick(self):
        cls = self.get_parameter('class_name').value
        dist = float(self.get_parameter('distance_m').value)
        conf = float(self.get_parameter('confidence').value)
        bx, by, bz = [float(v) for v in self.get_parameter('base_xyz').value]

        payload = {
            'timestamp_ns': self.get_clock().now().nanoseconds,
            'num_detections': 1,
            'objects': [{
                'class_name': cls,
                'confidence': conf,
                'center_2d': {'u': 320, 'v': 240},
                'distance_m': dist,
                'position_3d_camera_frame': {'X': 0.1, 'Y': 0.0, 'Z': dist},
                'position_3d_base_frame': {'X': bx, 'Y': by, 'Z': bz},
            }],
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockYoloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
