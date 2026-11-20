from .astar import AStarSolver
from .any_angle_astar import AnyAngleAStarSolver

REGISTRY = {s.name: s for s in (AStarSolver(), AnyAngleAStarSolver())}


def get_solver(name: str):
    return REGISTRY[name]
