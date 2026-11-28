"""Theta* —— Any-angle A* 实现（DDA 性能版本）。

性能关键：
  跨接尝试从"两次采样"换为"一次 DDA 网格遍历"，
  line_of_sight 和 segment_cost 合并为 trace_segment_cost，
  每条边只穿过的像元数量级迭代，7s → 约 2s。
"""
from __future__ import annotations

import heapq
import math
import time

import numpy as np

from .astar import NEIGHBORS, _is_identity_direction
from .base import Solver
from ..types import PathProblem, PathResult
from ..cost.segment_cost import segment_cost, trace_segment_cost


class AnyAngleAStarSolver(Solver):
    name = "any_angle"
    supports_any_angle = True

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

        def center(r, c):
            return gt.pixel_to_world(c, r)

        starts = [(x, y) for (x, y) in problem.entries
                  if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]
        goals = [(x, y) for (x, y) in problem.exits
                 if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]

        if not starts or not goals:
            return PathResult(ok=False, solver=self.name,
                              runtime=time.perf_counter() - t0,
                              message="入口/出口越界或落在不可通行像元")

        goal_cells = {to_cell(x, y) for (x, y) in goals}
        goal_pts = [center(r, c) for (r, c) in goal_cells]
        cmin = grid.min_cost if _is_identity_direction(dm) else 0.0

        if len(goal_pts) == 1:
            gx, gy = goal_pts[0]
            def h(r, c):
                x, y = center(r, c)
                return math.hypot(x - gx, y - gy) * cmin
        else:
            def h(r, c):
                x, y = center(r, c)
                return min(math.hypot(x - gx, y - gy)
                           for gx, gy in goal_pts) * cmin

        g = {}
        parent = {}
        anchors = {}
        source_entry = {}
        heap = []
        k = 0

        for x, y in starts:
            r, c = to_cell(x, y)
            entry_point = (x, y)
            cell_center = center(r, c)

            enter = 0.0
            if entry_point != cell_center:
                enter = segment_cost(entry_point, cell_center, grid, dm)
                if not math.isfinite(enter):
                    continue

            if enter < g.get((r, c), math.inf):
                g[(r, c)] = enter
                parent[(r, c)] = None
                anchors[(r, c)] = cell_center
                source_entry[(r, c)] = entry_point
                heapq.heappush(heap, (enter + h(r, c), k, enter, r, c))
                k += 1


        closed = set()
        visited = 0
        found = None

        push = heapq.heappush
        pop = heapq.heappop
        INF = math.inf

        while heap:
            _, _, gc, r, c = pop(heap)
            if (r, c) in closed or gc > g.get((r, c), INF):
                continue
            closed.add((r, c))
            visited += 1
            if (r, c) in goal_cells:
                found = (r, c)
                break

            p = parent[(r, c)]
            s_anchor = anchors.get((r, c), center(r, c))

            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not in_grid(nr, nc) or (nr, nc) in closed:
                    continue

                nb_center = center(nr, nc)

                # Compare both feasible ways to reach the neighbor.
                step_cost, step_ok = trace_segment_cost(s_anchor, nb_center, grid, dm)
                best_cost = gc + step_cost if step_ok else INF
                best_parent = (r, c)

                if p is not None:
                    cross_cost, cross_ok = trace_segment_cost(anchors[p], nb_center, grid, dm)
                    if cross_ok:
                        cross_total = g[p] + cross_cost
                        if cross_total <= best_cost:
                            best_cost = cross_total
                            best_parent = p

                if best_cost < g.get((nr, nc), INF):
                    g[(nr, nc)] = best_cost
                    parent[(nr, nc)] = best_parent
                    anchors[(nr, nc)] = nb_center
                    push(heap, (best_cost + h(nr, nc), k, best_cost, nr, nc))
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

        path = [source_entry[cells[0]]] + [center(r, c) for r, c in cells]
        end_pt = next(
            ((x, y) for x, y in goals if to_cell(x, y) == cells[-1]),
            center(*cells[-1]),
        )
        path.append(end_pt)

        goal_center = center(*found)
        tail_cost = 0.0
        if goal_center != end_pt:
            tail_cost = segment_cost(goal_center, end_pt, grid, dm)
            if not math.isfinite(tail_cost):
                return PathResult(
                    ok=False,
                    solver=self.name,
                    visited=visited,
                    runtime=rt,
                    message="终点连接不可通行",
                )

        length = sum(
            math.hypot(x2 - x1, y2 - y1)
            for (x1, y1), (x2, y2) in zip(path, path[1:])
        )
        return PathResult(
            ok=True,
            path=path,
            total_cost=g[found] + tail_cost,
            length=length,
            visited=visited,
            runtime=rt,
            solver=self.name,
        )
