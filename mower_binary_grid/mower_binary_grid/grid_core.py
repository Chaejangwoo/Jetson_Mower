"""ROS-independent depth/semantic fusion for a local binary grid.

Coordinate convention:
  * base X: forward
  * base Y: left
  * base Z: up
  * grid rows: lateral (right to left)
  * grid columns: forward (near to far)

The public output intentionally stays binary: 0 is traversable and 1 is
blocked.  Unobserved cells remain blocked, so loss of depth never creates a
false free corridor.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class GridConfig:
    resolution_m: float = 0.20
    forward_min_m: float = 0.0
    forward_max_m: float = 6.0
    lateral_min_m: float = -3.0
    lateral_max_m: float = 3.0
    depth_min_m: float = 0.30
    depth_max_m: float = 6.0
    sample_stride: int = 4
    camera_forward_m: float = 0.0
    camera_left_m: float = 0.0
    camera_height_m: float = 0.60
    camera_roll_deg: float = 0.0
    camera_pitch_down_deg: float = 0.0
    camera_yaw_left_deg: float = 0.0
    obstacle_min_height_m: float = 0.12
    mowable_grass_tolerance_m: float = 0.30
    ground_below_tolerance_m: float = 0.20
    min_obstacle_points_per_cell: int = 3
    detection_score_threshold: float = 0.35
    detection_contact_band_fraction: float = 0.25
    horizon_y_ratio: float = 0.40

    @property
    def width(self) -> int:
        return int(np.ceil((self.forward_max_m - self.forward_min_m) / self.resolution_m))

    @property
    def height(self) -> int:
        return int(np.ceil((self.lateral_max_m - self.lateral_min_m) / self.resolution_m))


@dataclass
class GridResult:
    binary_grid: np.ndarray
    observed_grid: np.ndarray
    obstacle_grid: np.ndarray
    valid_depth_points: int
    free_cells: int
    blocked_cells: int


def rotation_base_from_camera_axes(config: GridConfig) -> np.ndarray:
    """Return Rz(yaw) @ Ry(pitch-down) @ Rx(roll), in base-axis convention."""
    roll = np.deg2rad(config.camera_roll_deg)
    pitch = np.deg2rad(config.camera_pitch_down_deg)
    yaw = np.deg2rad(config.camera_yaw_left_deg)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _bresenham(col0: int, row0: int, col1: int, row1: int) -> Iterable[Tuple[int, int]]:
    """Yield (row, col) cells on a 2-D integer line, including both ends."""
    dc = abs(col1 - col0)
    dr = abs(row1 - row0)
    sc = 1 if col0 < col1 else -1
    sr = 1 if row0 < row1 else -1
    err = dc - dr
    col, row = col0, row0
    while True:
        yield row, col
        if col == col1 and row == row1:
            break
        twice = 2 * err
        if twice > -dr:
            err -= dr
            col += sc
        if twice < dc:
            err += dc
            row += sr


class BinaryGridBuilder:
    """Fuse one aligned depth frame and one AFR semantic result."""

    def __init__(self, config: GridConfig):
        self.config = config
        self._rotation = rotation_base_from_camera_axes(config)
        self._translation = np.array(
            [config.camera_forward_m, config.camera_left_m, config.camera_height_m],
            dtype=np.float32,
        )

    def _metric_to_cell(self, forward_m: float, left_m: float) -> Tuple[int, int]:
        col = int(np.floor((forward_m - self.config.forward_min_m) / self.config.resolution_m))
        row = int(np.floor((left_m - self.config.lateral_min_m) / self.config.resolution_m))
        return row, col

    def _inside(self, row: int, col: int) -> bool:
        return 0 <= row < self.config.height and 0 <= col < self.config.width

    def _state_for_pixels(self, u: np.ndarray, v: np.ndarray, image_w: int, image_h: int,
                          state_grid: np.ndarray) -> np.ndarray:
        states = np.full(u.shape, 4, dtype=np.uint8)
        if state_grid.ndim != 2 or state_grid.size == 0:
            return states
        roi_y = self.config.horizon_y_ratio * image_h
        below = v >= roi_y
        if not np.any(below):
            return states
        cols = np.floor(u[below] / image_w * state_grid.shape[1]).astype(np.int32)
        rows = np.floor((v[below] - roi_y) / max(1.0, image_h - roi_y) * state_grid.shape[0]).astype(np.int32)
        cols = np.clip(cols, 0, state_grid.shape[1] - 1)
        rows = np.clip(rows, 0, state_grid.shape[0] - 1)
        states[below] = state_grid[rows, cols]
        return states

    def _detection_mask(self, u: np.ndarray, v: np.ndarray, image_w: int, image_h: int,
                        detections: Sequence[Mapping]) -> np.ndarray:
        mask = np.zeros(u.shape, dtype=bool)
        for detection in detections:
            if float(detection.get('score', 0.0)) < self.config.detection_score_threshold:
                continue
            box = detection.get('xyxy_normalized')
            if not isinstance(box, (list, tuple)) or len(box) != 4:
                continue
            x0, y0, x1, y1 = [float(value) for value in box]
            x0, x1 = sorted((np.clip(x0, 0.0, 1.0), np.clip(x1, 0.0, 1.0)))
            y0, y1 = sorted((np.clip(y0, 0.0, 1.0), np.clip(y1, 0.0, 1.0)))
            # A 2-D box contains background and ground behind the object.  Only
            # use its lower contact band to estimate the occupied map cells;
            # otherwise one oversized/duplicate detector box blocks most of
            # the local map at several unrelated depths.
            band = np.clip(self.config.detection_contact_band_fraction, 0.05, 1.0)
            contact_y0 = y1 - (y1 - y0) * band
            mask |= ((u >= x0 * image_w) & (u <= x1 * image_w) &
                     (v >= contact_y0 * image_h) & (v <= y1 * image_h))
        return mask

    def build(self, depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
              state_grid: Sequence[Sequence[int]], detections: Sequence[Mapping]) -> GridResult:
        if depth_m.ndim != 2:
            raise ValueError('depth_m must be a 2-D array')
        image_h, image_w = depth_m.shape
        stride = max(1, int(self.config.sample_stride))
        vv, uu = np.mgrid[0:image_h:stride, 0:image_w:stride]
        zz = depth_m[::stride, ::stride].astype(np.float32, copy=False)
        u = uu.reshape(-1).astype(np.float32)
        v = vv.reshape(-1).astype(np.float32)
        z = zz.reshape(-1)
        valid = (np.isfinite(z) & (z >= self.config.depth_min_m) &
                 (z <= self.config.depth_max_m))
        u, v, z = u[valid], v[valid], z[valid]

        binary = np.ones((self.config.height, self.config.width), dtype=np.uint8)
        observed = np.zeros_like(binary, dtype=bool)
        obstacles = np.zeros_like(binary, dtype=bool)
        if z.size == 0:
            return GridResult(binary, observed, obstacles, 0, 0, int(binary.size))

        # Optical axes (right, down, forward) -> base-oriented camera axes
        # (forward, left, up), then configurable mounting rotation/translation.
        optical_right = (u - cx) * z / fx
        optical_down = (v - cy) * z / fy
        camera_base_axes = np.vstack((z, -optical_right, -optical_down))
        points = (self._rotation @ camera_base_axes).T + self._translation
        forward, left, height = points[:, 0], points[:, 1], points[:, 2]
        in_map = ((forward >= self.config.forward_min_m) &
                  (forward < self.config.forward_max_m) &
                  (left >= self.config.lateral_min_m) &
                  (left < self.config.lateral_max_m) &
                  (height >= -self.config.ground_below_tolerance_m))
        u, v = u[in_map], v[in_map]
        forward, left, height = forward[in_map], left[in_map], height[in_map]

        if forward.size == 0:
            return GridResult(binary, observed, obstacles, int(z.size), 0, int(binary.size))

        states = self._state_for_pixels(
            u, v, image_w, image_h, np.asarray(state_grid, dtype=np.uint8))
        detected = self._detection_mask(u, v, image_w, image_h, detections)

        # AFR dangerous/deferred/unknown are conservative blocks.  mow_now may
        # contain flexible grass, so it gets a larger height tolerance.  A
        # confident object detection always overrides the grass exception.
        semantic_block = np.isin(states, (0, 1, 4))
        height_limit = np.where(states == 2,
                                self.config.mowable_grass_tolerance_m,
                                self.config.obstacle_min_height_m)
        obstacle_point = detected | semantic_block | (height > height_limit)

        free_counts: Dict[Tuple[int, int], int] = {}
        obstacle_counts: Dict[Tuple[int, int], int] = {}
        for x_value, y_value, is_obstacle in zip(forward, left, obstacle_point):
            cell = self._metric_to_cell(float(x_value), float(y_value))
            target = obstacle_counts if bool(is_obstacle) else free_counts
            target[cell] = target.get(cell, 0) + 1

        origin_row, origin_col = self._metric_to_cell(0.0, 0.0)
        # When forward_min is zero, the sensor origin lies on the lower edge.
        origin_col = int(np.clip(origin_col, 0, self.config.width - 1))
        origin_row = int(np.clip(origin_row, 0, self.config.height - 1))

        endpoint_cells = set(free_counts) | set(obstacle_counts)
        for row, col in endpoint_cells:
            if not self._inside(row, col):
                continue
            is_obstacle = obstacle_counts.get((row, col), 0) >= self.config.min_obstacle_points_per_cell
            ray = list(_bresenham(origin_col, origin_row, col, row))
            clear_ray = ray[:-1] if is_obstacle else ray
            for ray_row, ray_col in clear_ray:
                if self._inside(ray_row, ray_col):
                    observed[ray_row, ray_col] = True
                    binary[ray_row, ray_col] = 0
            if is_obstacle:
                observed[row, col] = True
                obstacles[row, col] = True

        # Occupied evidence wins over every free ray crossing that cell.
        binary[obstacles] = 1
        return GridResult(
            binary_grid=binary,
            observed_grid=observed,
            obstacle_grid=obstacles,
            valid_depth_points=int(z.size),
            free_cells=int(np.count_nonzero(binary == 0)),
            blocked_cells=int(np.count_nonzero(binary == 1)),
        )

