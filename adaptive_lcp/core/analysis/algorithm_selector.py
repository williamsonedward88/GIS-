from __future__ import annotations


def select_solver(problem) -> str:
    pref = problem.preference
    grid = problem.grid
    n = grid.n_valid

    if pref == "fast":
        return "astar"
    if pref == "precise":
        return "any_angle"
    if pref == "balanced" and len(problem.entries) > 1:
        return "any_angle"
    if grid.cost_cv > 1.5 and n < 2_000_000:
        return "any_angle"
    return "astar"
