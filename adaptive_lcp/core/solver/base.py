from __future__ import annotations

from abc import ABC, abstractmethod

from ..types import PathProblem, PathResult


class Solver(ABC):
    name = "base"
    supports_any_angle = False
    supports_direction = True

    @abstractmethod
    def solve(self, problem: PathProblem) -> PathResult:
        ...
