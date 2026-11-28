"""A* 路径规划。普通成本栅格使用预计算边表加速。"""
from __future__ import annotations

import heapq
import math
import time

import numpy as np

from .base import Solver
from ..types import PathProblem, PathResult
from ..cost.segment_cost import segment_cost

NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1),
             (0, -1),           (0, 1),
             (1, -1),  (1, 0),  (1, 1)]

# 预计算边表的内存上限（超过则逐边计算）
_MAX_TABLE_BYTES = 400 * 1024 * 1024


def _is_identity_direction(dm) -> bool:
    if dm is None:
        return True
    try:
        return bool(dm.is_identity())
    except Exception:
        return False


def build_edge_tables(grid, direction_model=None):
    """预计算相邻像元中心之间的成本，每端各占线长的一半。"""
    if not _is_identity_direction(direction_model):
        return None

    H, W = grid.cost.shape
    if H * W * 8 * 8 > _MAX_TABLE_BYTES:
        return None

    gt = grid.gt
    dx = gt.dx
    dy = gt.dy
    if dx <= 0 or dy >= 0:
        return None
    cost = grid.cost.astype(np.float64)
    cost[~np.isfinite(cost)] = np.inf
    pad = np.pad(cost, 1, constant_values=np.inf)
    tables = {}

    for dr, dc in NEIGHBORS:
        half_length = 0.5 * math.hypot(dc * dx, dr * dy)
        neighbor = pad[1 + dr: 1 + dr + H, 1 + dc: 1 + dc + W]
        tbl = half_length * (cost + neighbor)

        if dr != 0 and dc != 0:
            n1 = pad[1 + dr: 1 + dr + H, 1: 1 + W]
            n2 = pad[1: 1 + H, 1 + dc: 1 + dc + W]
            tbl[(n1 == np.inf) | (n2 == np.inf)] = np.inf

        tables[(dr, dc)] = tbl

    return tables

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

        # === 预计算 8 方向边成本表 ===
        tables = build_edge_tables(grid, dm)
        goal_cells = {to_cell(x, y) for (x, y) in goals}
        goal_pts = [gt.pixel_to_world(c, r) for (r, c) in goal_cells]
        cmin = grid.min_cost if _is_identity_direction(dm) else 0.0

        if len(goal_pts) == 1:
            gx, gy = goal_pts[0]
            def h(r, c):
                x, y = gt.pixel_to_world(c, r)
                return math.hypot(x - gx, y - gy) * cmin
        else:
            def h(r, c):
                x, y = gt.pixel_to_world(c, r)
                return min(math.hypot(x - gx, y - gy)
                           for gx, gy in goal_pts) * cmin

        g = {}
        parent = {}
        source_entry = {}
        heap = []
        k = 0

        # 多源初始化：支持多点/连续入口
        for x, y in starts:
            r, c = to_cell(x, y)
            entry_point = (x, y)
            cell_center = gt.pixel_to_world(c, r)

            enter = 0.0
            if entry_point != cell_center:
                enter = segment_cost(entry_point, cell_center, grid, dm)
                if not math.isfinite(enter):
                    continue

            if enter < g.get((r, c), math.inf):
                g[(r, c)] = enter
                parent[(r, c)] = None
                source_entry[(r, c)] = entry_point
                heapq.heappush(heap, (enter + h(r, c), k, enter, r, c))
                k += 1

        closed = set()
        visited = 0
        found = None

        # 局部变量能减少 20% 的属性查找开销
        push, pop = heapq.heappush, heapq.heappop
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

            if tables is not None:
                # —— 快路径：查表 ——
                for dr, dc in NEIGHBORS:
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= H or nc < 0 or nc >= W:
                        continue
                    step = tables[(dr, dc)][r, c]
                    if not (step < INF):   # 同时过滤 inf 和 nan
                        continue
                    ng = gc + step
                    if ng < g.get((nr, nc), INF):
                        g[(nr, nc)] = ng
                        parent[(nr, nc)] = (r, c)
                        push(heap, (ng + h(nr, nc), k, ng, nr, nc))
                        k += 1
            else:
                # —— 慢路径：用共享的线段计价逐边计算 ——
                x0w, y0w = gt.pixel_to_world(c, r)
                for dr, dc in NEIGHBORS:
                    nr, nc = r + dr, c + dc
                    if nr < 0 or nr >= H or nc < 0 or nc >= W:
                        continue
                    if not np.isfinite(cost[nr, nc]):
                        continue
                    if dr and dc:
                        if not (np.isfinite(cost[r, nc]) and np.isfinite(cost[nr, c])):
                            continue
                    x1w, y1w = gt.pixel_to_world(nc, nr)
                    step = segment_cost((x0w, y0w), (x1w, y1w), grid, dm)
                    if not math.isfinite(step):
                        continue
                    ng = gc + step
                    if ng < g.get((nr, nc), INF):
                        g[(nr, nc)] = ng
                        parent[(nr, nc)] = (r, c)
                        push(heap, (ng + h(nr, nc), k, ng, nr, nc))
                        k += 1

        rt = time.perf_counter() - t0
        if found is None:
            return PathResult(ok=False, solver=self.name, visited=visited,
                              runtime=rt, message="未找到可行路径")

        # —— 回溯 ——
        cells = []
        cur = found
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()

        start_pt = source_entry[cells[0]]
        end_pt = next(((x, y) for (x, y) in goals if to_cell(x, y) == cells[-1]),
                      gt.pixel_to_world(cells[-1][1], cells[-1][0]))

        path = [start_pt] + [gt.pixel_to_world(c, r) for (r, c) in cells] + [end_pt]
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))

        goal_row, goal_col = found
        goal_center = gt.pixel_to_world(goal_col, goal_row)

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

        return PathResult(
            ok=True,
            path=path,
            total_cost=g[found] + tail_cost,
            length=length,
            visited=visited,
            runtime=rt,
            solver=self.name,
        )
