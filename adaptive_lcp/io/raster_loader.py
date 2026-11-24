"""QgsRasterLayer -> CostGrid。

关键变化：改用 GDAL 读取，自动适配 Int16/Float32，不再假定 float32。
"""
from __future__ import annotations

import numpy as np

from ..core.types import CostGrid, GeoTransform
from .terrain import read_raster_via_gdal


def load_cost_layer(layer) -> CostGrid:
    """把任意栅格当成本栅格。

    - NoData / nan / 非正数 → inf（不可通行）
    - 其余保持原值
    """
    if layer is None:
        raise ValueError("传入的图层是 None")

    arr, gt, crs_wkt = read_raster_via_gdal(layer)
    cost = arr.astype(np.float32, copy=True)

    cost[~np.isfinite(cost)] = np.inf
    cost[cost <= 0] = np.inf

    n_valid = int(np.isfinite(cost).sum())
    if n_valid == 0:
        raise ValueError("成本栅格全部不可通行（全部 NoData 或 <= 0）")

    return CostGrid(cost=cost, gt=gt, crs_wkt=crs_wkt, name=layer.name())


def load_dem_layer(layer):
    """读 DEM，返回 (arr, gt, crs_wkt)。NaN 表示无效高程。"""
    return read_raster_via_gdal(layer)