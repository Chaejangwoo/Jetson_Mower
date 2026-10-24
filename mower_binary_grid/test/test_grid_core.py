import numpy as np

from mower_binary_grid.grid_core import BinaryGridBuilder, GridConfig


def _config(**overrides):
    values = dict(
        resolution_m=0.2,
        forward_max_m=4.0,
        lateral_min_m=-2.0,
        lateral_max_m=2.0,
        sample_stride=1,
        camera_height_m=0.6,
        obstacle_min_height_m=0.12,
        mowable_grass_tolerance_m=0.30,
        min_obstacle_points_per_cell=1,
        horizon_y_ratio=0.0,
    )
    values.update(overrides)
    return GridConfig(**values)


def test_empty_depth_is_all_blocked():
    builder = BinaryGridBuilder(_config())
    depth = np.zeros((4, 4), dtype=np.float32)
    result = builder.build(depth, 2.0, 2.0, 1.5, 1.5, [[2]], [])
    assert np.all(result.binary_grid == 1)
    assert result.free_cells == 0


def test_mowable_low_surface_clears_ray():
    # Lower image pixels project below the camera. At this geometry the point
    # is close to ground level and should clear a corridor.
    builder = BinaryGridBuilder(_config())
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[4, 2] = 1.2
    result = builder.build(depth, 4.0, 4.0, 2.0, 2.0, [[2]], [])
    assert result.free_cells > 0
    assert not np.any(result.obstacle_grid)


def test_detection_overrides_mowable_grass_tolerance():
    builder = BinaryGridBuilder(_config())
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[2, 2] = 1.0  # base height remains camera_height_m
    detection = {'score': 0.9, 'xyxy_normalized': [0.3, 0.3, 0.7, 0.7]}
    result = builder.build(depth, 4.0, 4.0, 2.0, 2.0, [[2]], [detection])
    assert np.any(result.obstacle_grid)


def test_unknown_semantic_is_blocked():
    builder = BinaryGridBuilder(_config())
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[4, 2] = 1.2
    result = builder.build(depth, 4.0, 4.0, 2.0, 2.0, [[4]], [])
    assert np.any(result.obstacle_grid)

