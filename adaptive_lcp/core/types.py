"""全局数据结构。模块之间只通过这些类型传数据。"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class GeoTransform:
    """像元(col,row) <-> 世界坐标(x,y)，语义与 GDAL geotransform 一致。"""
    x0: float
    dx: float          # 像元宽（正）
    y0: float
    dy: float          # 像元高（一般负）

    def pixel_to_world(self, col: float, row: float) -> tuple[float, float]:
        return (self.x0 + (col + 0.5) * self.dx,
                self.y0 + (row + 0.5) * self.dy)

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        return ((x - self.x0) / self.dx,
                (y - self.y0) / self.dy)


@dataclass
class CostGrid:
    cost: np.ndarray            # float32 (H, W)，np.inf 表示不可通行
    gt: GeoTransform
    crs_wkt: str = ""
    name: str = "cost"

    @property
    def shape(self) -> tuple[int, int]:
        return self.cost.shape

    @property
    def n_valid(self) -> int:
        return int(np.isfinite(self.cost).sum())

    @property
    def min_cost(self) -> float:
        v = self.cost[np.isfinite(self.cost)]
        return float(v.min()) if v.size else 1.0

    @property
    def cost_cv(self) -> float:
        v = self.cost[np.isfinite(self.cost)]
        if v.size == 0 or v.mean() <= 0:
            return 0.0
        return float(v.std() / v.mean())


@dataclass
class PathProblem:
    grid: CostGrid
    entries: list[tuple[float, float]]
    exits: list[tuple[float, float]]
    direction_model: Optional[object] = None
    preference: str = "auto"   # auto | fast | balanced | precise


@dataclass
class PathResult:
    ok: bool
    path: list[tuple[float, float]] = field(default_factory=list)
    total_cost: float = math.inf
    length: float = 0.0
    visited: int = 0
    runtime: float = 0.0
    solver: str = ""
    message: str = ""
