import numpy as np
import pytest

from adaptive_lcp.core.types import CostGrid, GeoTransform


@pytest.fixture
def simple_grid():
    """10x10 平原，成本 1.0，第 5 列 3~6 行一堵墙。"""
    cost = np.ones((10, 10), dtype=np.float32)
    cost[3:7, 5] = np.inf
    gt = GeoTransform(x0=0.0, dx=1.0, y0=10.0, dy=-1.0)
    return CostGrid(cost=cost, gt=gt)
