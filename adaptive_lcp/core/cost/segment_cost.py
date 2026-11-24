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
def trace_segment_cost(p0, p1, grid, direction_model=None):
    """Amanatides-Woo DDA 网格遍历：一次遍历同时得成本与通行性。

    返回 (cost, ok)。
      ok=False  → 穿越不可通行像元或越界，cost 为 inf
      ok=True   → cost 是沿线段积分得到的精确边成本

    与 segment_cost 的差异：
      - segment_cost 用 0.25 像元步长采样，边界处有 ≤0.125 像元误差
      - 本函数精确计算每段在像元内的穿越长度，误差 0
      - 方向模型非恒等时退回 segment_cost（DDA 不处理方向惩罚）
    """
    if direction_model is not None:
        try:
            if not direction_model.is_identity():
                c = segment_cost(p0, p1, grid, direction_model)
                return (c, math.isfinite(c))
        except Exception:
            pass

    x0, y0 = p0
    x1, y1 = p1
    gt = grid.gt
    H, W = grid.cost.shape
    px_w = abs(gt.dx)
    px_h = abs(gt.dy)

    dx = x1 - x0
    dy = y1 - y0
    L = math.hypot(dx, dy)
    if L == 0.0:
        return 0.0, True

    # 连续像元坐标（0.0 = 左上角点，0.5 = 首像元中心）
    fc0 = (x0 - gt.x0) / gt.dx
    fr0 = (y0 - gt.y0) / gt.dy
    fc1 = (x1 - gt.x0) / gt.dx
    fr1 = (y1 - gt.y0) / gt.dy

    c = int(math.floor(fc0))
    r = int(math.floor(fr0))
    if not (0 <= r < H and 0 <= c < W):
        return math.inf, False
    if not np.isfinite(grid.cost[r, c]):
        return math.inf, False

    step_c = 1 if fc1 > fc0 else (-1 if fc1 < fc0 else 0)
    step_r = 1 if fr1 > fr0 else (-1 if fr1 < fr0 else 0)

    INF = math.inf
    if step_c > 0:
        t_max_c = (c + 1 - fc0) * gt.dx / dx
    elif step_c < 0:
        t_max_c = (c - fc0) * gt.dx / dx
    else:
        t_max_c = INF
    t_delta_c = abs(gt.dx / dx) if step_c != 0 else INF

    if step_r > 0:
        t_max_r = (r + 1 - fr0) * gt.dy / dy
    elif step_r < 0:
        t_max_r = (r - fr0) * gt.dy / dy
    else:
        t_max_r = INF
    t_delta_r = abs(gt.dy / dy) if step_r != 0 else INF

    total = 0.0
    t_cur = 0.0
    guard = 2 * (H + W) + 32

    while guard > 0:
        guard -= 1
        t_next = t_max_c if t_max_c < t_max_r else t_max_r
        if t_next > 1.0:
            t_next = 1.0

        cval = grid.cost[r, c]
        if not (cval < INF):
            return INF, False
        total += float(cval) * (t_next - t_cur) * L

        if t_next >= 1.0:
            return total, True

        t_cur = t_next
        if t_max_c < t_max_r:
            c += step_c
            t_max_c += t_delta_c
        elif t_max_r < t_max_c:
            r += step_r
            t_max_r += t_delta_r
        else:
            # 恰好从角点穿过：保守判定——两正交邻居任一不可通行则视为不可走
            r1, c1 = r + step_r, c
            r2, c2 = r, c + step_c
            if not (0 <= r1 < H and 0 <= c1 < W
                    and 0 <= r2 < H and 0 <= c2 < W):
                return INF, False
            if not (grid.cost[r1, c1] < INF) or not (grid.cost[r2, c2] < INF):
                return INF, False
            r += step_r
            c += step_c
            t_max_c += t_delta_c
            t_max_r += t_delta_r

        if not (0 <= r < H and 0 <= c < W):
            return INF, False

    return INF, False