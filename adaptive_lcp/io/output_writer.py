"""结果输出：栅格 GeoTIFF + 矢量路径图层。

关键变化：
1. 矢量图层 CRS 用 setCrs，不再拼 URI（避免 WKT 泄漏导致图层无效）
2. write_raster 防 inf（GDAL 不认 inf），并校验 CRS
"""
from __future__ import annotations

import os

import numpy as np
from osgeo import gdal, osr

from ..core.types import CostGrid


def result_to_layer(result, crs_wkt: str, name: str = "best_path"):
    """PathResult → QGIS 内存矢量线图层，并加入地图。"""
    from qgis.core import (
        QgsVectorLayer, QgsFeature, QgsGeometry, QgsPointXY,
        QgsProject, QgsField, QgsFields, QgsCoordinateReferenceSystem,
    )
    from qgis.PyQt.QtCore import QVariant

    layer = QgsVectorLayer("LineString", name, "memory")
    if crs_wkt:
        try:
            crs = QgsCoordinateReferenceSystem.fromWkt(crs_wkt)
            if crs.isValid():
                layer.setCrs(crs)
        except Exception:
            pass  # crs 为空也能画出线，只是没有坐标参考

    fields = QgsFields()
    fields.append(QgsField("total_cost", QVariant.Double))
    fields.append(QgsField("length", QVariant.Double))
    fields.append(QgsField("solver", QVariant.String))
    fields.append(QgsField("runtime_s", QVariant.Double))
    layer.dataProvider().addAttributes(fields)
    layer.updateFields()

    if result.ok and len(result.path) >= 2:
        feat = QgsFeature()
        feat.setGeometry(QgsGeometry.fromPolylineXY(
            [QgsPointXY(x, y) for (x, y) in result.path]
        ))
        feat.setAttributes([
            float(result.total_cost), float(result.length),
            str(result.solver), float(result.runtime),
        ])
        layer.dataProvider().addFeature(feat)
        layer.updateExtents()

    QgsProject.instance().addMapLayer(layer)
    return layer


def write_raster(arr: np.ndarray, gt, crs_wkt: str, out_path: str,
                 nodata=np.nan, name: str = "out"):
    """numpy 数组 → GeoTIFF。

    - inf 会被替换为 nan（GDAL 不认 inf）
    - 无 CRS 时跳过投影设置
    """
    if arr.ndim != 2:
        raise ValueError(f"数组必须是 2D，实际是 {arr.shape}")

    H, W = arr.shape
    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    drv = gdal.GetDriverByName("GTiff")
    if drv is None:
        raise RuntimeError("GDAL 没有 GTiff 驱动")

    ds = drv.Create(
        out_path, W, H, 1, gdal.GDT_Float32,
        options=["COMPRESS=LZW", "TILED=YES"],
    )
    if ds is None:
        raise RuntimeError(f"无法创建 GeoTIFF：{out_path}")

    ds.SetGeoTransform([gt.x0, gt.dx, 0.0, gt.y0, 0.0, gt.dy])

    if crs_wkt:
        try:
            srs = osr.SpatialReference()
            srs.ImportFromWkt(crs_wkt)
            ds.SetProjection(srs.ExportToWkt())
        except Exception:
            pass

    band = ds.GetRasterBand(1)
    band.SetDescription(name)
    band.SetNoDataValue(float(nodata))

    out = arr.astype(np.float32)
    out[np.isinf(out)] = np.nan
    band.WriteArray(out)
    band.FlushCache()
    ds = None
    return out_path


def load_raster_to_qgis(out_path: str, layer_name: str | None = None):
    """把写出的 tif 加入 QGIS 图层树。"""
    from qgis.core import QgsRasterLayer, QgsProject

    name = layer_name or os.path.splitext(os.path.basename(out_path))[0]
    rlayer = QgsRasterLayer(out_path, name)
    if not rlayer.isValid():
        raise RuntimeError(f"QGIS 载入栅格失败：{out_path}")
    QgsProject.instance().addMapLayer(rlayer)
    return rlayer


def write_cost_grid_as_tif(cost_grid: CostGrid, out_path: str):
    return write_raster(
        cost_grid.cost, cost_grid.gt, cost_grid.crs_wkt,
        out_path, name="cost",
    )