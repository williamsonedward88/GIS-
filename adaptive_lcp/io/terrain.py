"""QGIS Processing 计算坡度/坡向。

关键修复：
- read_raster_via_gdal 同时接受 QgsRasterLayer 和 str（Processing 可能返回路径）
- slope. / aspect_as_array 显式给出临时 .tif 路径，保证返回值一定是可用路径
"""
from __future__ import annotations

import os
import tempfile

import numpy as np
from osgeo import gdal

from ..core.types import GeoTransform


def _ensure_processing_ready():
    from qgis.core import QgsApplication

    reg = QgsApplication.processingRegistry()
    if reg.providerById("native") is None:
        from qgis.analysis import QgsNativeAlgorithms
        reg.addProvider(QgsNativeAlgorithms())
    if reg.providerById("native") is None:
        raise RuntimeError("无法注册 QGIS native provider")

    try:
        import processing  # noqa: F401
    except ImportError as e:
        raise RuntimeError(
            "Processing 未启用。请在 QGIS 插件管理器里勾选 Process。"
        ) from e


def _resolve_source(layer_or_path):
    """把 QgsRasterLayer / 路径 / id 统一解析成 GDAL 能打开的 path。

    返回 (path, display_name)。
    """
    # 情况 A：QgsMapLayer 对象
    if hasattr(layer_or_path, "source"):
        src = layer_or_path.source()
        name = getattr(layer_or_path, "name", lambda: os.path.basename(src))()
        if isinstance(src, str) and src.startswith("file://"):
            src = src[7:]
        # memory 图层没法给 GDAL 用，给出明确错误
        if isinstance(src, str) and ":" in src.split(".")[0] and not os.path.exists(src):
            raise RuntimeError(
                f"图层是内存图层，GDAL 读不到：{name}；"
                f"请先“图层 → 另存为”写成 GeoTIFF 再选。"
            )
        return src, name

    # 情况 B：Processing 返回的字符串（路径 或 图层 id）
    s = str(layer_or_path)
    # id 形式："output_xxx_xxxx"
    if ":" in s or (s.count(".") == 0 and not os.path.exists(s)):
        from qgis.core import QgsProject, QgsRasterLayer
        lyr = QgsProject.instance().mapLayer(s)
        if lyr is not None and hasattr(lyr, "source"):
            return _resolve_source(lyr)
        # 可能是 dataProvider 的 uri（如 gdal：xxx）
        if os.path.exists(s):
            return s, os.path.basename(s)
        try:
            tmp = QgsRasterLayer(s, os.path.basename(s))
            if tmp.isValid():
                return tmp.source(), tmp.name()
        except Exception:
            pass
        raise RuntimeError(f"无法解析栅格 source：{s}")

    if os.path.exists(s):
        return s, os.path.basename(s)

    raise RuntimeError(f"路径不存在：{s}")


def read_raster_via_gdal(layer_or_path):
    """读取栅格 → (arr: float32 HxW, GeoTransform, crs_wkt)。

    - 支持 QgsRasterLayer 和 str 路径
    - 把 NoData 和掩膜里的无效像元置为 NaN
    - 支持 Int16 / UInt16 / Float32 / Float64
    """
    src, name = _resolve_source(layer_or_path)

    ds = gdal.Open(src)
    if ds is None:
        raise RuntimeError(f"GDAL 无法打开栅格：{name}（path={src}）")

    try:
        band = ds.GetRasterBand(1)
        arr = band.ReadAsArray()
        if arr is None:
            raise RuntimeError(f"栅格读取失败：{name}")

        invalid = ~np.isfinite(arr)
        nodata = band.GetNoDataValue()
        if nodata is not None:
            invalid |= arr == nodata

        flags = band.GetMaskFlags()
        if flags not in (gdal.GMF_ALL_VALID, gdal.GMF_NODATA):
            # 单独保存的掩膜可与 NoData 同时存在。0 表示不可用。
            validity = band.GetMaskBand().ReadAsArray()
            if validity is None:
                raise RuntimeError(f"栅格有效性掩膜读取失败：{name}")
            invalid |= validity == 0

        arr = arr.astype(np.float32)
        arr[invalid] = np.nan

        g = ds.GetGeoTransform()
        if g is None or g[0] is None:
            raise RuntimeError(f"栅格缺少 geotransform：{name}")

        gt = GeoTransform(x0=g[0], dx=g[1], y0=g[3], dy=g[5])
        crs_wkt = ds.GetProjection() or ""
    finally:
        ds = None

    return arr, gt, crs_wkt


def _run_native(algo_id: str, dem_layer, z_factor: float):
    """跑 native 算法并返回生成的临时 tif 路径。"""
    _ensure_processing_ready()
    import processing

    fd, out_path = tempfile.mkstemp(
        suffix=".tif", prefix=f"lcp_{algo_id.split(':')[-1]}_"
    )
    os.close(fd)
    try:
        os.remove(out_path)          # GDAL 不想覆盖已存在的空文件
    except OSError:
        pass

    result = processing.run(algo_id, {
        "INPUT": dem_layer,
        "Z_FACTOR": z_factor,
        "OUTPUT": out_path,
    })

    out = result.get("OUTPUT")
    if out is None:
        raise RuntimeError(f"{algo_id} 未返回 OUTPUT")

    # 3.x 中通常返回路径；保险起见再解析一次
    path, _ = _resolve_source(out)
    if not os.path.exists(path):
        raise RuntimeError(f"{algo_id} 输出文件不存在：{path}")
    return path


def slope_as_array(dem_layer, z_factor: float = 1.0):
    """返回 (slope_deg, gt, crs_wkt)。单位：度。"""
    dem, dem_gt, _ = read_raster_via_gdal(dem_layer)
    out_path = _run_native("native:slope", dem_layer, z_factor)
    slope, slope_gt, crs_wkt = read_raster_via_gdal(out_path)

    # QGIS 的坡度算法可能忽略输入栅格独立保存的有效性掩膜。
    invalid = ~np.isfinite(dem)
    if invalid.any():
        if slope.shape != dem.shape or slope_gt != dem_gt:
            raise ValueError("坡度与原 DEM 网格不一致，无法应用原始无效区。")
        slope[invalid] = np.nan
    return slope, slope_gt, crs_wkt


def aspect_as_array(dem_layer, z_factor: float = 1.0):
    """返回 (aspect_deg, gt, crs_wkt)。范围 0~360，0=正北。"""
    out_path = _run_native("native:aspect", dem_layer, z_factor)
    try:
        return read_raster_via_gdal(out_path)
    finally:
        pass
