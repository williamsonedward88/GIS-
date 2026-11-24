"""生成一个测试 DEM（UTM 50N，300x300，10m 像元）。
在 QGIS Python Console 里执行：
    exec(open(r"D:/PKU/gis_homework2/adaptive_lcp/scripts/make_demo_dem.py").read())
"""
import numpy as np
from osgeo import gdal, osr

OUT = "D:/PKU/gis_homework2/demo_dem.tif"
W, H = 300, 300


def _gauss(x, y, cx, cy, sx, sy, h):
    return h * np.exp(-(((x - cx) / sx) ** 2 + ((y - cy) / sy) ** 2))


x, y = np.meshgrid(np.arange(W), np.arange(H))
elev = np.full((H, W), 100.0, dtype=np.float32)
elev += _gauss(x, y,  80,  80, 40, 40, 200).astype(np.float32)
elev += _gauss(x, y, 220, 220, 50, 50, 300).astype(np.float32)
elev += _gauss(x, y, 150, 200, 30, 60, 150).astype(np.float32)
elev += _gauss(x, y, 150, 150, 12, 90, 500).astype(np.float32)
elev += (np.random.randn(H, W) * 5).astype(np.float32)

drv = gdal.GetDriverByName("GTiff")
ds = drv.Create(OUT, W, H, 1, gdal.GDT_Float32, options=["COMPRESS=LZW"])
ds.SetGeoTransform([500000.0, 10.0, 0.0, 4000000.0, 0.0, -10.0])

srs = osr.SpatialReference()
srs.ImportFromEPSG(32650)          # UTM 50N
ds.SetProjection(srs.ExportToWkt())

band = ds.GetRasterBand(1)
band.SetNoDataValue(-9999.0)
band.WriteArray(elev)
band.FlushCache()
ds = None

print(f"DEM 已生成：{OUT}")
print(f"  尺寸 {W}x{H}，像元 10m，高程范围 {elev.min():.1f} ~ {elev.max():.1f}")