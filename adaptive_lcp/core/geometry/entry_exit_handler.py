"""把点 / 多点 / 线段 转成真实世界坐标候选集。

核心：不做像元中心吸附，真实坐标一路保留到 solver。
"""
from __future__ import annotations

import math


def points_to_candidates(points):
    return [tuple(map(float, p)) for p in points]


def boundary_to_candidates(p0, p1, step: float):
    """沿线段按 step 世界单位均匀取点，包含两端。"""
    x0, y0 = p0
    x1, y1 = p1
    L = math.hypot(x1 - x0, y1 - y0)
    if L == 0 or step <= 0:
        return [(x0, y0)]
    n = max(2, int(math.ceil(L / step)) + 1)
    return [(x0 + (x1 - x0) * i / (n - 1),
             y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]
