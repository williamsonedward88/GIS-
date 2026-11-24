"""A* vs Any-angle A* 对比脚本。

执行：
    exec(open(r"D:/PKU/gis_homework2/adaptive_lcp/scripts/compare_algos.py").read())
"""
import math
import os
import traceback

import numpy as np

from qgis.core import QgsProject, QgsRasterLayer

from adaptive_lcp.io.terrain import slope_as_array
from adaptive_lcp.core.cost.cost_builder import build_c0_from_dem
from adaptive_lcp.core.types import PathProblem
from adaptive_lcp.core.solver.registry import get_solver
from adaptive_lcp.io.output_writer import result_to_layer


# ============ 改这里：你的 DEM 文件路径 ============
DEM_PATH = r"D:\PKU\gis_homework2\demo_dem.tif"
# ==================================================


def count_turns(path, angle_thresh_deg=5.0):
    if len(path) < 3:
        return 0
    turns = 0
    for i in range(1, len(path) - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        a1 = math.atan2(y1 - y0, x1 - x0)
        a2 = math.atan2(y2 - y1, x2 - x1)
        d = abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi)
        if math.degrees(d) > angle_thresh_deg:
            turns += 1
    return turns


def _find_or_load_dem():
    """项目里有栅格就用第一个；没有就从 DEM_PATH 加载。"""
    rasters = [lyr for lyr in QgsProject.instance().mapLayers().values()
               if isinstance(lyr, QgsRasterLayer)]
    if rasters:
        dem = next((l for l in rasters if "demo" in l.name().lower()), rasters[0])
        print(f"从项目图层取 DEM: {dem.name()}")
        return dem

    print(f"项目里没有栅格图层，尝试从文件加载: {DEM_PATH}")
    if not os.path.exists(DEM_PATH):
        raise FileNotFoundError(
            f"DEM 文件不存在: {DEM_PATH}\n"
            f"请修改脚本顶部的 DEM_PATH 常量，改成你实际的 DEM 路径。"
        )

    lyr = QgsRasterLayer(DEM_PATH, os.path.splitext(os.path.basename(DEM_PATH))[0])
    if not lyr.isValid():
        raise RuntimeError(f"QGIS 无法识别该 DEM: {DEM_PATH}")

    QgsProject.instance().addMapLayer(lyr)
    print(f"已加载 DEM: {lyr.name()}  {lyr.width()}x{lyr.height()}")
    return lyr


def run():
    # ---- 1. DEM ----
    try:
        dem = _find_or_load_dem()
    except Exception as e:
        print(f"!! {e}")
        return

    ext = dem.extent()
    print(f"DEM: {dem.name()}  {dem.width()}x{dem.height()}")
    print(f"范围 X=[{ext.xMinimum():.1f}, {ext.xMaximum():.1f}]  "
          f"Y=[{ext.yMinimum():.1f}, {ext.yMaximum():.1f}]")

    # ---- 2. 坡度 + 成本 ----
    print("计算坡度…")
    slope, gt, crs = slope_as_array(dem)
    v = np.isfinite(slope)
    if not v.any():
        print("!! 坡度结果全部无效，DEM 可能全是 NoData")
        return
    print(f"  坡度 min={slope[v].min():.2f}  max={slope[v].max():.2f}")

    grid = build_c0_from_dem(slope, gt, crs_wkt=crs, w_slope=1.5)
    f = np.isfinite(grid.cost)
    print(f"  成本 min={grid.cost[f].min():.2f}  "
          f"max={grid.cost[f].max():.2f}  有效像元={int(f.sum())}")

    # ---- 3. 端点 ----
    ENTRY = (500900.0, 3999100.0)
    EXIT = (502100.0, 3997900.0)

    def _inside(x, y):
        return (ext.xMinimum() <= x <= ext.xMaximum()
                and ext.yMinimum() <= y <= ext.yMaximum())

    if _inside(*ENTRY) and _inside(*EXIT):
        print(f"端点（固定）: {ENTRY} -> {EXIT}")
    else:
        xmin, xmax = ext.xMinimum(), ext.xMaximum()
        ymin, ymax = ext.yMinimum(), ext.yMaximum()
        ENTRY = (xmin + (xmax - xmin) * 0.30, ymin + (ymax - ymin) * 0.30)
        EXIT = (xmin + (xmax - xmin) * 0.70, ymin + (ymax - ymin) * 0.70)
        print(f"端点（自动 30%~70%）: {ENTRY} -> {EXIT}")

    problem = PathProblem(grid=grid, entries=[ENTRY], exits=[EXIT])

    # ---- 4. 跑两种算法 ----
    print("\n" + "=" * 62)
    results = {}
    for name in ("astar", "any_angle"):
        solver = get_solver(name)
        try:
            r = solver.solve(problem)
        except Exception as e:
            print(f"[{name}] 异常: {e}")
            traceback.print_exc()
            continue

        results[name] = r
        turns = count_turns(r.path)
        print(f"[{name}]")
        print(f"  ok         = {r.ok}")
        print(f"  总成本     = {r.total_cost:.3f}")
        print(f"  路径长度   = {r.length:.3f}")
        print(f"  访问节点   = {r.visited:,}")
        print(f"  运行时间   = {r.runtime:.3f} s")
        print(f"  路径点数   = {len(r.path)}")
        print(f"  实际拐点   = {turns}")

        if r.ok:
            result_to_layer(r, crs, name=f"path_{name}")
            print(f"  已加入图层 = path_{name}")
    print("=" * 62)

    # ---- 5. 结论 ----
    if "astar" in results and "any_angle" in results \
            and results["astar"].ok and results["any_angle"].ok:
        a, t = results["astar"], results["any_angle"]
        dc = (1 - t.total_cost / a.total_cost) * 100 if a.total_cost else 0
        dl = (1 - t.length / a.length) * 100 if a.length else 0
        ta, tt = count_turns(a.path), count_turns(t.path)
        print("\n【对比结论】")
        print(f"  总成本降幅   {dc:+.2f}%   ({a.total_cost:.1f} -> {t.total_cost:.1f})")
        print(f"  路径长度降幅 {dl:+.2f}%   ({a.length:.1f} -> {t.length:.1f})")
        print(f"  拐点变化     {ta} -> {tt}   ({tt - ta:+d})")
        print(f"  耗时对比     {a.runtime:.2f}s -> {t.runtime:.2f}s")


run()