"""ROS 2 node publishing a conservative 20 cm binary traversability grid."""

import json
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .grid_core import BinaryGridBuilder, GridConfig


def _stamp_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


class BinaryGridNode(Node):
    def __init__(self):
        super().__init__('binary_grid_node')
        defaults = GridConfig()
        self.declare_parameter('vision_topic', '/vision/result')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('binary_grid_topic', '/perception/local_binary_grid')
        self.declare_parameter('occupancy_grid_topic', '/perception/local_occupancy_grid')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('sync_max_ms', 100.0)
        self.declare_parameter('resolution_m', defaults.resolution_m)
        self.declare_parameter('forward_min_m', defaults.forward_min_m)
        self.declare_parameter('forward_max_m', defaults.forward_max_m)
        self.declare_parameter('lateral_min_m', defaults.lateral_min_m)
        self.declare_parameter('lateral_max_m', defaults.lateral_max_m)
        self.declare_parameter('depth_min_m', defaults.depth_min_m)
        self.declare_parameter('depth_max_m', defaults.depth_max_m)
        self.declare_parameter('sample_stride', defaults.sample_stride)
        self.declare_parameter('camera_forward_m', defaults.camera_forward_m)
        self.declare_parameter('camera_left_m', defaults.camera_left_m)
        self.declare_parameter('camera_height_m', defaults.camera_height_m)
        self.declare_parameter('camera_roll_deg', defaults.camera_roll_deg)
        self.declare_parameter('camera_pitch_down_deg', defaults.camera_pitch_down_deg)
        self.declare_parameter('camera_yaw_left_deg', defaults.camera_yaw_left_deg)
        self.declare_parameter('obstacle_min_height_m', defaults.obstacle_min_height_m)
        self.declare_parameter('mowable_grass_tolerance_m', defaults.mowable_grass_tolerance_m)
        self.declare_parameter('ground_below_tolerance_m', defaults.ground_below_tolerance_m)
        self.declare_parameter('min_obstacle_points_per_cell', defaults.min_obstacle_points_per_cell)
        self.declare_parameter('detection_score_threshold', defaults.detection_score_threshold)
        self.declare_parameter(
            'detection_contact_band_fraction', defaults.detection_contact_band_fraction)
        self.declare_parameter('horizon_y_ratio', defaults.horizon_y_ratio)

        config_names = GridConfig.__dataclass_fields__.keys()
        config = GridConfig(**{name: self.get_parameter(name).value for name in config_names})
        if config.resolution_m <= 0.0 or config.forward_max_m <= config.forward_min_m:
            raise ValueError('invalid grid dimensions')
        if config.lateral_max_m <= config.lateral_min_m:
            raise ValueError('invalid lateral grid dimensions')
        self.builder = BinaryGridBuilder(config)
        self.config = config
        self.bridge = CvBridge()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.sync_max_ns = int(float(self.get_parameter('sync_max_ms').value) * 1_000_000)
        self.depth_buffer = deque(maxlen=15)
        self.camera_info: Optional[CameraInfo] = None

        self.binary_pub = self.create_publisher(
            String, str(self.get_parameter('binary_grid_topic').value), 10)
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter('occupancy_grid_topic').value), 10)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._depth_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('vision_topic').value), self._vision_cb, 10)
        self.get_logger().info(
            f'이진 격자 준비: {config.height}x{config.width}, '
            f'{config.resolution_m:.2f}m/cell, 0=통과 가능, 1=통과 불가능')

    def _depth_cb(self, msg: Image):
        self.depth_buffer.append((_stamp_ns(msg), msg))

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _nearest_depth(self, timestamp_ns: int) -> Optional[Tuple[int, Image]]:
        if not self.depth_buffer:
            return None
        nearest = min(self.depth_buffer, key=lambda item: abs(item[0] - timestamp_ns))
        if abs(nearest[0] - timestamp_ns) > self.sync_max_ns:
            return None
        return nearest

    def _depth_m(self, msg: Image) -> np.ndarray:
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding in ('16UC1', 'mono16'):
            return raw.astype(np.float32) * 0.001
        if msg.encoding == '32FC1':
            return raw.astype(np.float32, copy=False)
        raise ValueError(f'지원하지 않는 깊이 인코딩: {msg.encoding}')

    def _vision_cb(self, msg: String):
        try:
            result = json.loads(msg.data)
            if result.get('status') != 'ok':
                return
            capture_ns = int(result['capture_timestamp_ns'])
            depth_item = self._nearest_depth(capture_ns)
            if depth_item is None or self.camera_info is None:
                self.get_logger().warn('동기화된 Depth 또는 CameraInfo 없음', throttle_duration_sec=5.0)
                return
            depth_ns, depth_msg = depth_item
            depth_m = self._depth_m(depth_msg)
            info = self.camera_info
            if depth_m.shape != (info.height, info.width):
                raise ValueError(
                    f'Depth {depth_m.shape}와 CameraInfo {(info.height, info.width)} 크기 불일치')
            grid = self.builder.build(
                depth_m=depth_m,
                fx=float(info.k[0]), fy=float(info.k[4]),
                cx=float(info.k[2]), cy=float(info.k[5]),
                state_grid=result.get('state_grid', []),
                detections=result.get('detections', []),
            )
            self._publish(grid, depth_msg, capture_ns, depth_ns)
        except Exception as exc:  # keep the perception pipeline alive on one bad frame
            self.get_logger().error(f'이진 격자 생성 실패: {exc}', throttle_duration_sec=2.0)

    def _publish(self, grid, depth_msg: Image, capture_ns: int, depth_ns: int):
        payload = {
            'frame_id': self.frame_id,
            'capture_timestamp_ns': capture_ns,
            'depth_gap_ms': abs(depth_ns - capture_ns) / 1_000_000.0,
            'resolution_m': self.config.resolution_m,
            'width': self.config.width,
            'height': self.config.height,
            'origin': {
                'forward_m': self.config.forward_min_m,
                'left_m': self.config.lateral_min_m,
            },
            'value_contract': {'free': 0, 'blocked': 1},
            'valid_depth_points': grid.valid_depth_points,
            'free_cells': grid.free_cells,
            'blocked_cells': grid.blocked_cells,
            'data': grid.binary_grid.tolist(),
        }
        binary_msg = String()
        binary_msg.data = json.dumps(payload, separators=(',', ':'))
        self.binary_pub.publish(binary_msg)

        occupancy = OccupancyGrid()
        occupancy.header.stamp = depth_msg.header.stamp
        occupancy.header.frame_id = self.frame_id
        occupancy.info.resolution = float(self.config.resolution_m)
        occupancy.info.width = self.config.width
        occupancy.info.height = self.config.height
        occupancy.info.origin.position.x = float(self.config.forward_min_m)
        occupancy.info.origin.position.y = float(self.config.lateral_min_m)
        occupancy.info.origin.orientation = Quaternion(w=1.0)
        occupancy.data = (grid.binary_grid.astype(np.int8) * 100).reshape(-1).tolist()
        self.occupancy_pub.publish(occupancy)


def main(args=None):
    rclpy.init(args=args)
    node = BinaryGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

