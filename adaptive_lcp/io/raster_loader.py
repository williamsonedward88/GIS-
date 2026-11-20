"""QgsRasterLayer -> CostGrid。只有这个文件允许 import qgis。"""
from __future__ import annotations

import numpy as np

from ..core.types import CostGrid, GeoTransform


def load_cost_layer(layer) -> CostGrid:
    from qgis.core import QgsRasterLayer, QgsCoordinateReferenceSystem

    if not isinstance(layer, QgsRasterLayer):
        raise TypeError("需要 QgsRasterLayer")

    provider = layer.dataProvider()
    block = provider.block(1, layer.extent(), layer.width(), layer.height())
    H, W = layer.height(), layer.width()

    arr = np.full((H, W), np.nan, dtype=np.float32)
    for r in range(H):
        for c in range(W):
            v = block.value(r, c)
            arr[r, c] = v if block.isNoData(r, c) is False else np.nan

    # 统一：NoData -> inf（不可通行）
    cost = np.where(np.isfinite(arr), arr, np.inf).astype(np.float32)

    ext = layer.extent()
    dx = (ext.xMaximum() - ext.xMinimum()) / W
    dy = (ext.yMaximum() - ext.yMinimum()) / H
    gt = GeoTransform(ext.xMinimum(), dx, ext.yMaximum(), -abs(dy))

    return CostGrid(cost=cost, gt=gt,
                    crs_wkt=layer.crs().toWkt(), name=layer.name())
