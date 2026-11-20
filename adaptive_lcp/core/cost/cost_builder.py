"""从坡度/土地利用/道路合成基础成本 C0。

第一版先实现坡度 + 归一化加权，土地利用与道路留接口。
"""
from __future__ import annotations

import numpy as np

from ..types import CostGrid, GeoTransform


def normalize(arr: np.ndarray, lo: float = 0.0, hi: float = 1.0) -> np.ndarray:
    v = np.isfinite(arr)
    if not v.any():
        return np.zeros_like(arr, dtype=np.float32)
    a = arr.copy().astype(np.float64)
    mn, mx = a[v].min(), a[v].max()
    if mx - mn < 1e-12:
        a[v] = lo
    else:
        a[v] = lo + (a[v] - mn) / (mx - mn) * (hi - lo)
    a[~v] = np.inf
    return a.astype(np.float32)


def build_c0(slope_deg: np.ndarray,
             landuse: np.ndarray | None = None,
             road: np.ndarray | None = None,
             w_slope: float = 0.6,
             w_landuse: float = 0.3,
             w_road: float = 0.1,
             gt: GeoTransform | None = None) -> CostGrid:
    """坡度越陡成本越高，返回范围约 [1.0, 1.0 + 权重和] 的成本栅格。"""
    c = 1.0 + w_slope * normalize(slope_deg, 0.0, 1.0).astype(np.float64)
    if landuse is not None:
        c = c + w_landuse * normalize(landuse, 0.0, 1.0)
    if road is not None:
        # 道路降低成本的语义，这里先按"距离越小越好"处理
        c = c - w_road * normalize(road, 0.0, 1.0)
    c = np.clip(c, 0.1, None).astype(np.float32)
    c[~np.isfinite(slope_deg)] = np.inf
    if gt is None:
        gt = GeoTransform(0.0, 1.0, 0.0, -1.0)
    return CostGrid(cost=c, gt=gt, name="c0")
