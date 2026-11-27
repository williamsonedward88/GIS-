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

from .astar import NEIGHBORS
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
        cmin = grid.min_cost

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

                # === Theta* 跨接（DDA 一次搞定视线 + 成本） ===
                if p is not None:
                    p_anchor = anchors.get(p, center(p[0], p[1]))
                    cross, ok = trace_segment_cost(p_anchor, nb_center, grid, dm)
                    if ok:
                        ng = g[p] + cross
                        if ng < g.get((nr, nc), INF):
                            g[(nr, nc)] = ng
                            parent[(nr, nc)] = p
                            anchors[(nr, nc)] = nb_center
                            push(heap, (ng + h(nr, nc), k, ng, nr, nc))
                            k += 1
                            continue

                # === 标准 8 邻域回退（同样 DDA） ===
                cur_anchor = anchors.get((r, c), center(r, c))
                step, ok2 = trace_segment_cost(cur_anchor, nb_center, grid, dm)
                if not ok2:
                    continue
                ng = gc + step
                if ng < g.get((nr, nc), INF):
                    g[(nr, nc)] = ng
                    parent[(nr, nc)] = (r, c)
                    anchors[(nr, nc)] = nb_center
                    push(heap, (ng + h(nr, nc), k, ng, nr, nc))
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