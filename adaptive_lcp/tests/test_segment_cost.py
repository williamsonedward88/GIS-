import numpy as np

from adaptive_lcp.core.cost.segment_cost import segment_cost, line_of_sight
from adaptive_lcp.core.solver.registry import get_solver
from adaptive_lcp.core.types import CostGrid, GeoTransform, PathProblem


def test_cost_along_axis():
    cost = np.full((5, 5), 2.0, dtype=np.float32)
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    c = segment_cost((0.5, 0.5), (4.5, 0.5), grid)
    assert abs(c - 2.0 * 4.0) < 1e-6


def test_cost_blocked():
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, 2] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    assert segment_cost((0.5, 0.5), (4.5, 4.5), grid) == np.inf


def test_line_of_sight_false_through_wall():
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, 2] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    assert not line_of_sight((0.5, 0.5), (4.5, 4.5), grid)

def test_cost_long_segment():
    """20 像元长的直线，成本 3.0，结果必须精确等于 60.0"""
    cost = np.full((25, 25), 3.0, dtype=np.float32)
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 25.0, -1.0))
    c = segment_cost((0.5, 0.5), (20.5, 0.5), grid)
    assert abs(c - 60.0) < 1e-6


def test_corner_obstacle_blocks_both_directions():
    cost = np.ones((8, 4), dtype=np.float32)
    cost[3, 2] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 8.0, -1.0))
    start, end = (0.5, 7.5), (3.5, 0.5)

    for a, b in ((start, end), (end, start)):
        assert not line_of_sight(a, b, grid)
        result = get_solver("any_angle").solve(PathProblem(grid, [a], [b]))
        assert result.ok
        assert all(line_of_sight(p, q, grid) for p, q in zip(result.path, result.path[1:]))


def test_two_solvers_charge_same_cost_for_same_segment():
    grid = CostGrid(np.array([[1.0, 3.0]], dtype=np.float32), GeoTransform(0.0, 1.0, 1.0, -1.0))
    problem = PathProblem(grid, [(0.5, 0.5)], [(1.5, 0.5)])

    for name in ("astar", "any_angle"):
        result = get_solver(name).solve(problem)
        assert result.ok
        assert result.total_cost == 2.0
