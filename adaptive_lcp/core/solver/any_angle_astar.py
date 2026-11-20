"""Theta* 风格 Any-angle A*。

实现思路：在 A* 的松弛阶段，如果当前节点 s 与其祖父节点 ps 之间视线可达，
直接把 ps 设为 (nr,nc) 的父节点，从而"跨接"父指针，让路径脱离 8 邻域。

当前为占位实现（暂用标准 A* 行为），同学 B 按 TODO 完成即可。
"""
from __future__ import annotations

from .astar import AStarSolver


class AnyAngleAStarSolver(AStarSolver):
    name = "any_angle"
    supports_any_angle = True

    # TODO(B): 重写 solve 的松弛循环
    #   ps = parent[s]
    #   if ps is not None and line_of_sight(cell_world(ps),
    #                                       cell_world((nr, nc)), grid):
    #       ng = g[ps] + segment_cost(cell_world(ps), cell_world((nr, nc)), grid, dm)
    #       if ng < g.get((nr, nc), inf):
    #           g[(nr, nc)] = ng
    #           parent[(nr, nc)] = ps        # 跨接
    #           heappush(...)
    #   注意弹堆时必须检查 gc > g[(r,c)] 跳过失效条目。
