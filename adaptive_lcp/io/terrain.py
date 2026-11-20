"""调用 QGIS Processing 计算坡度/坡向，返回 numpy 数组。"""


def slope_as_array(dem_layer):
    # TODO(A): 使用 processing.run("qgis:slope", {...}) 或 gdal:slope
    # 结果图层 -> io.raster_loader 复用读数组逻辑
    raise NotImplementedError
