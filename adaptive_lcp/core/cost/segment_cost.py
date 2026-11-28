"""线积分边成本：Cost = Σ C_k · P_k(θ) · L_k

这是 A* 和 Any-angle A* 共用的唯一成本口径。改动前必须双方确认。
"""
from __future__ import annotations

import math

import numpy as np


def sample_cells(p0, p1, grid, step_frac: float = 0.25):
    """沿 p0->p1 采样，返回 (rows, cols, ds, L)。

    若任一点越界则返回 None。求解器使用 trace_segment_cost 精确遍历。
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
    """返回穿过各像元的长度加权成本。step_frac 保留用于兼容旧调用。"""
    return trace_segment_cost(p0, p1, grid, direction_model)[0]


def line_of_sight(p0, p1, grid, step_frac: float = 0.5) -> bool:
    """按计价时的障碍规则检查线段。step_frac 保留用于兼容旧调用。"""
    return trace_segment_cost(p0, p1, grid)[1]


def trace_segment_cost(p0, p1, grid, direction_model=None):
    """遍历线段触及的像元，返回 (累计代价, 是否可通行)。"""
    x0, y0 = p0
    x1, y1 = p1
    gt = grid.gt
    costs = grid.cost
    H, W = costs.shape
    fc0, fr0 = gt.world_to_pixel(x0, y0)
    fc1, fr1 = gt.world_to_pixel(x1, y1)
    inf = math.inf
    if not all(map(math.isfinite, (fc0, fr0, fc1, fr1))):
        return inf, False

    c, r = math.floor(fc0), math.floor(fr0)
    end_c, end_r = math.floor(fc1), math.floor(fr1)
    if not (0 <= r < H and 0 <= c < W and 0 <= end_r < H and 0 <= end_c < W):
        return inf, False
    if not (math.isfinite(costs[r, c]) and math.isfinite(costs[end_r, end_c])):
        return inf, False

    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0, True

    dc, dr = fc1 - fc0, fr1 - fr0
    step_c = int(dc > 0) - int(dc < 0)
    step_r = int(dr > 0) - int(dr < 0)
    c_offset, r_offset = int(dc > 0), int(dr > 0)

    # 角点允许浮点舍入误差。大坐标、小像元时也计入坐标转换误差。
    corner_tol = 1e-12
    if step_c and step_r:
        corner_tol += 4 * max(math.ulp(x0), math.ulp(x1)) / abs(dx)
        corner_tol += 4 * max(math.ulp(y0), math.ulp(y1)) / abs(dy)
        corner_tol = min(corner_tol, 1e-8)

    rows, cols, fractions = [], [], []
    total = 0.0
    t = 0.0
    while True:
        # 直接由格边位置算交点，避免逐格叠加引入误差。
        t_c = (c + c_offset - fc0) / dc if step_c else inf
        t_r = (r + r_offset - fr0) / dr if step_r else inf
        t_next = min(t_c, t_r, 1.0)
        value = float(costs[r, c])
        if not math.isfinite(value):
            return inf, False

        fraction = t_next - t
        if direction_model is None:
            total += value * fraction
        elif fraction > 0:
            rows.append(r)
            cols.append(c)
            fractions.append(fraction)

        if t_next == 1.0:
            # 终点恰在角点时，也用相同规则校验两侧。
            if r != end_r and c != end_c:
                if not (math.isfinite(costs[end_r, c]) and math.isfinite(costs[r, end_c])):
                    return inf, False
            break

        t = t_next
        if abs(t_c - t_r) <= corner_tol:
            next_r, next_c = r + step_r, c + step_c
            if not (0 <= next_r < H and 0 <= next_c < W):
                return inf, False
            if not (math.isfinite(costs[next_r, c]) and math.isfinite(costs[r, next_c])):
                return inf, False
            r, c = next_r, next_c
        elif t_r < t_c:
            r += step_r
        else:
            c += step_c
        if not (0 <= r < H and 0 <= c < W):
            return inf, False

    if direction_model is not None:
        row_indices, col_indices = np.asarray(rows), np.asarray(cols)
        factors = direction_model.multiplier(math.atan2(dy, dx), row_indices, col_indices)
        rates = costs[row_indices, col_indices].astype(np.float64) * factors
        total = float(np.dot(rates, fractions))

    total *= length
    return (total, True) if math.isfinite(total) else (inf, False)
