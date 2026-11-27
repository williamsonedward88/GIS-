from math import isclose, sqrt

import numpy as np

from adaptive_lcp.core.solver.any_angle_astar import AnyAngleAStarSolver
from adaptive_lcp.core.types import CostGrid, GeoTransform, PathProblem


def test_weighted_grid_compares_step_and_shortcut_costs():
    # The direct shortcut cuts through an expensive cell. The bend avoids it.
    grid = CostGrid(
        cost=np.array([[1, 8, 8], [1, 1, 8]], dtype=np.float32),
        gt=GeoTransform(0, 1, 2, -1),
    )
    problem = PathProblem(grid=grid, entries=[(0.5, 1.5)], exits=[(2.5, 0.5)])

    result = AnyAngleAStarSolver().solve(problem)

    assert result.ok, result.message
    assert (1.5, 0.5) in result.path
    assert isclose(result.total_cost, 4.5 + sqrt(2))
