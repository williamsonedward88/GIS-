from adaptive_lcp.core.types import PathProblem
from adaptive_lcp.core.solver.astar import AStarSolver


def test_simple_grid_reachable(simple_grid):
    """起终点在同一行且墙不挡住，但这里故意放同一行验证能到。"""
    p = PathProblem(grid=simple_grid, entries=[(0.5, 0.5)], exits=[(9.5, 0.5)])
    r = AStarSolver().solve(p)
    assert r.ok
    assert r.total_cost > 0
    assert r.visited > 0


def test_go_around_wall(simple_grid):
    """起点左侧、终点右侧，必须绕过墙。"""
    p = PathProblem(grid=simple_grid, entries=[(1.5, 4.5)], exits=[(8.5, 4.5)])
    r = AStarSolver().solve(p)
    assert r.ok
    assert r.length > 7.0   # 直线 7，绕墙一定更长


def test_unreachable():
    import numpy as np
    from adaptive_lcp.core.types import CostGrid, GeoTransform
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, :] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    p = PathProblem(grid=grid, entries=[(0.5, 0.5)], exits=[(0.5, 4.5)])
    r = AStarSolver().solve(p)
    assert not r.ok
