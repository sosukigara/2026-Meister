"""Test 共通の OccupancyGrid ビルダ。ROS グラフは不要 (rclpy.init もしない)。"""
import array

import pytest
from nav_msgs.msg import OccupancyGrid


def make_grid(size: int = 32, fill: int = 0, wall_row: int | None = 10) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.info.width = size
    grid.info.height = size
    grid.info.resolution = 0.05
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -2.0
    data = array.array('b', [fill]) * (size * size)
    if wall_row is not None:
        for c in range(size):
            data[wall_row * size + c] = 100
    grid.data = data
    return grid


@pytest.fixture
def grid_factory():
    return make_grid
