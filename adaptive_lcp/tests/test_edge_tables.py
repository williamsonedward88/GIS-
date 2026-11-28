"""直接对比 build_edge_tables 和 segment_cost 的逐边结果。"""
import numpy as np
import pytest

from adaptive_lcp.core.types import CostGrid, GeoTransform
from adaptive_lcp.core.cost.segment_cost import segment_cost
from adaptive_lcp.core.solver.astar import build_edge_tables, NEIGHBORS


@pytest.fixture
def random_grid():
    rng = np.random.default_rng(42)
    H, W = 12, 12
    cost = rng.uniform(1.0, 3.0, size=(H, W)).astype(np.float32)
    cost[3:7, 5] = np.inf
    gt = GeoTransform(x0=500000.0, dx=10.0, y0=4000000.0, dy=-12.0)
    return CostGrid(cost=cost, gt=gt)


def test_tables_match_segment_cost(random_grid):
    grid = random_grid
    tables = build_edge_tables(grid, None)
    assert tables is not None

    H, W = grid.cost.shape
    worst = 0.0
    worst_case = None
    bad = 0

    for r in range(H - 1):
        for c in range(W - 1):
            x0, y0 = grid.gt.pixel_to_world(c, r)
            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < H and 0 <= nc < W):
                    continue
                x1, y1 = grid.gt.pixel_to_world(nc, nr)
                slow = segment_cost((x0, y0), (x1, y1), grid)
                fast = float(tables[(dr, dc)][r, c])
                assert np.isfinite(slow) == np.isfinite(fast)
                if not np.isfinite(slow):
                    continue
                rel = abs(fast - slow) / max(abs(slow), 1e-12)
                if rel > worst:
                    worst = rel
                    worst_case = (r, c, dr, dc, slow, fast)
                if rel > 1e-6:
                    bad += 1

    print(f"\n  最大相对误差 = {worst:.3e}")
    print(f"  误差 > 1e-6 的边数 = {bad}")
    if worst_case:
        r, c, dr, dc, slow, fast = worst_case
        print(f"  最差边: r={r} c={c} dir=({dr},{dc})")
        print(f"    slow={slow:.8f}  fast={fast:.8f}")
    assert worst < 1e-6, f"边成本不等价，最大相对误差 {worst:.3e}"
