"""Transforms 3D points from base_link to world frame via tf2.

YOLO 노드(ROBOT_VISION)가 이미 camera_link->base_link 변환을 끝내
position_3d_base_frame 를 제공하므로, 에이전트는 base_link->world
정적 변환만 수행한다(보통 identity, UR3 URDF에서 확인 필요).
"""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers do_transform_point


class TFTransformer:
    """Wraps tf2_ros.Buffer to convert base_frame 3D points to world frame."""

    def __init__(self, node: Node, base_frame: str = 'base_link', world_frame: str = 'world'):
        self._base_frame = base_frame
        self._world_frame = world_frame
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def to_world(
        self,
        x: float,
        y: float,
        z: float,
        timeout_sec: float = 3.0,
    ) -> tuple[float, float, float]:
        """
        Transform (x, y, z) in base_frame to world_frame coordinates.
        Raises tf2_ros.TransformException on failure.
        """
        point = PointStamped()
        point.header.frame_id = self._base_frame
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        transform = self._tf_buffer.lookup_transform(
            self._world_frame,        # target frame
            self._base_frame,         # source frame
            rclpy.time.Time(),        # latest available
            timeout=Duration(seconds=timeout_sec),
        )
        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        return (transformed.point.x, transformed.point.y, transformed.point.z)
