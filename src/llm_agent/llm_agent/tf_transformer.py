"""Transforms 3D points from camera frame to world frame via tf2."""

from geometry_msgs.msg import PointStamped
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers do_transform_point


class TFTransformer:
    """
    Wraps tf2_ros.Buffer to convert camera-frame 3D points to world frame.

    camera_frame: name of the camera link frame (confirm with YOLO team)
    world_frame:  MoveIt planning frame (default: 'world')
    """

    def __init__(self, node: Node, camera_frame: str, world_frame: str = 'world'):
        self._camera_frame = camera_frame
        self._world_frame = world_frame
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

    def to_world(
        self,
        x: float,
        y: float,
        z: float,
        timeout_sec: float = 1.0,
    ) -> tuple[float, float, float]:
        """
        Transform (x, y, z) in camera_frame to world_frame coordinates.
        Raises tf2_ros.LookupException / ConnectivityException on failure.
        """
        from rclpy.duration import Duration
        point = PointStamped()
        point.header.frame_id = self._camera_frame
        point.point.x = x
        point.point.y = y
        point.point.z = z

        transform = self._tf_buffer.lookup_transform(
            self._world_frame,
            self._camera_frame,
            rclpy_time=tf2_ros.Time(),
            timeout=Duration(seconds=timeout_sec),
        )
        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        return (
            transformed.point.x,
            transformed.point.y,
            transformed.point.z,
        )
