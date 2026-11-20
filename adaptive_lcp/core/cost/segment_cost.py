"""线积分边成本：Cost = Σ C_k · P_k(θ) · L_k

这是 A* 和 Any-angle A* 共用的唯一成本口径。改动前必须双方确认。
"""
from __future__ import annotations

import math

import numpy as np


def sample_cells(p0, p1, grid, step_frac: float = 0.25):
    """沿 p0->p1 采样，返回 (rows, cols, ds, L)。

    若任一点越界则返回 None。该函数也被 Any-angle A* 的视线判断复用。
    """
    x0, y0 = p0
    x1, y1 = p1
    length = math.hypot(x1 - x0, y1 - y0)
    if length == 0.0:
        return None

    gt = grid.gt
    px = min(abs(gt.dx), abs(gt.dy))
    n = max(2, int(math.ceil(length / (px * step_frac))) + 1)

    t = np.linspace(0.0, 1.0, n)
    xs = x0 + (x1 - x0) * t
    ys = y0 + (y1 - y0) * t

    cols = np.floor((xs - gt.x0) / gt.dx).astype(np.int64)
    rows = np.floor((ys - gt.y0) / gt.dy).astype(np.int64)

    H, W = grid.cost.shape
    if (rows < 0).any() or (rows >= H).any() or (cols < 0).any() or (cols >= W).any():
        return None

    return rows, cols, length / (n - 1), length


def segment_cost(p0, p1, grid, direction_model=None, step_frac: float = 0.25) -> float:
    res = sample_cells(p0, p1, grid, step_frac)
    if res is None:
        return math.inf

    rows, cols, ds, length = res
    c = grid.cost[rows, cols].astype(np.float64)
    if not np.all(np.isfinite(c)):
        return math.inf

    if direction_model is not None:
        theta = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        c = c * direction_model.multiplier(theta, rows, cols)

    return float(np.sum((c[:-1] + c[1:]) * 0.5) * ds)


def line_of_sight(p0, p1, grid, step_frac: float = 0.5) -> bool:
    """Any-angle A* 用：路径经过的所有像元是否全部可通行。"""
    res = sample_cells(p0, p1, grid, step_frac)
    if res is None:
        return False
    rows, cols, _, _ = res
    return bool(np.all(np.isfinite(grid.cost[rows, cols])))
