from __future__ import annotations

import heapq
import math
import time

import numpy as np

from .base import Solver
from ..types import PathProblem, PathResult
from ..cost.segment_cost import segment_cost

NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1),
             (0, -1), (0, 1),
             (1, -1), (1, 0), (1, 1)]


class AStarSolver(Solver):
    name = "astar"

    def solve(self, problem: PathProblem) -> PathResult:
        t0 = time.perf_counter()
        grid = problem.grid
        cost = grid.cost
        gt = grid.gt
        dm = problem.direction_model
        H, W = cost.shape

        def to_cell(x, y):
            return (int(math.floor((y - gt.y0) / gt.dy)),
                    int(math.floor((x - gt.x0) / gt.dx)))

        def in_grid(r, c):
            return 0 <= r < H and 0 <= c < W

        starts = [(x, y) for (x, y) in problem.entries
                  if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]
        goals = [(x, y) for (x, y) in problem.exits
                 if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]

        if not starts or not goals:
            return PathResult(ok=False, solver=self.name,
                              runtime=time.perf_counter() - t0,
                              message="入口/出口越界或落在不可通行像元")

        goal_cells = {to_cell(x, y) for (x, y) in goals}
        goal_pts = [gt.pixel_to_world(c, r) for (r, c) in goal_cells]
        cmin = grid.min_cost

        def h(r, c):
            x, y = gt.pixel_to_world(c, r)
            return min(math.hypot(x - gx, y - gy) for gx, gy in goal_pts) * cmin

        g = {}
        parent = {}
        heap = []
        k = 0

        for (x, y) in starts:
            r, c = to_cell(x, y)
            cx, cy = gt.pixel_to_world(c, r)
            enter = segment_cost((x, y), (cx, cy), grid, dm)
            if not math.isfinite(enter):
                enter = 0.0
            if enter < g.get((r, c), math.inf):
                g[(r, c)] = enter
                parent[(r, c)] = None
                heapq.heappush(heap, (enter + h(r, c), k, enter, r, c))
                k += 1

        closed = set()
        visited = 0
        found = None

        while heap:
            _, _, gc, r, c = heapq.heappop(heap)
            if (r, c) in closed or gc > g.get((r, c), math.inf):
                continue
            closed.add((r, c))
            visited += 1
            if (r, c) in goal_cells:
                found = (r, c)
                break

            x0w, y0w = gt.pixel_to_world(c, r)
            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not in_grid(nr, nc) or not np.isfinite(cost[nr, nc]):
                    continue
                if dr and dc:
                    if not (np.isfinite(cost[r, nc]) and np.isfinite(cost[nr, c])):
                        continue
                x1w, y1w = gt.pixel_to_world(nc, nr)
                step = segment_cost((x0w, y0w), (x1w, y1w), grid, dm)
                if not math.isfinite(step):
                    continue
                ng = gc + step
                if ng < g.get((nr, nc), math.inf):
                    g[(nr, nc)] = ng
                    parent[(nr, nc)] = (r, c)
                    heapq.heappush(heap, (ng + h(nr, nc), k, ng, nr, nc))
                    k += 1

        rt = time.perf_counter() - t0
        if found is None:
            return PathResult(ok=False, solver=self.name, visited=visited,
                              runtime=rt, message="未找到可行路径")

        cells = []
        cur = found
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()

        start_pt = next(((x, y) for (x, y) in starts if to_cell(x, y) == cells[0]),
                        gt.pixel_to_world(cells[0][1], cells[0][0]))
        end_pt = next(((x, y) for (x, y) in goals if to_cell(x, y) == cells[-1]),
                      gt.pixel_to_world(cells[-1][1], cells[-1][0]))

        path = [start_pt] + [gt.pixel_to_world(c, r) for (r, c) in cells] + [end_pt]
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))

        return PathResult(ok=True, path=path, total_cost=g[found],
                          length=length, visited=visited, runtime=rt,
                          solver=self.name)
