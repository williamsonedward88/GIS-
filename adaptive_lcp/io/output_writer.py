"""PathResult -> QgsVectorLayer，并加入地图。"""


def result_to_layer(result, crs_wkt: str, name: str = "best_path"):
    from qgis.core import (
        QgsVectorLayer, QgsFeature, QgsGeometry, QgsPointXY,
        QgsProject, QgsCoordinateReferenceSystem, QgsField, QgsFields,
    )
    from qgis.PyQt.QtCore import QVariant

    fields = QgsFields()
    fields.append(QgsField("total_cost", QVariant.Double))
    fields.append(QgsField("length", QVariant.Double))
    fields.append(QgsField("solver", QVariant.String))
    fields.append(QgsField("runtime_s", QVariant.Double))

    layer = QgsVectorLayer("LineString?crs=" + crs_wkt, name, "memory")
    layer.dataProvider().addAttributes(fields)
    layer.updateFields()

    if not result.ok or len(result.path) < 2:
        QgsProject.instance().addMapLayer(layer)
        return layer

    feat = QgsFeature()
    feat.setGeometry(QgsGeometry.fromPolylineXY(
        [QgsPointXY(x, y) for (x, y) in result.path]))
    feat.setAttributes([result.total_cost, result.length,
                        result.solver, result.runtime])
    layer.dataProvider().addFeature(feat)
    layer.updateExtents()
    QgsProject.instance().addMapLayer(layer)
    return layer
