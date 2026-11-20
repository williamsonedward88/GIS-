"""
Adaptive LCP 项目脚手架生成器
用法：
    python bootstrap.py
默认生成到当前目录下的 adaptive_lcp_project/，可在下面 TARGET 修改。
"""
from pathlib import Path
import textwrap

# ============ 改这里 ============
TARGET = Path("./adaptive_lcp_project")
# ===============================

PKG = TARGET / "adaptive_lcp"

FILES: dict[str, str] = {}

# ---------- metadata.txt ----------
FILES["adaptive_lcp/metadata.txt"] = """[general]
name=Adaptive Least-Cost Path
qgisMinimumVersion=3.28
description=成本栅格 + 连续入口 + 自适应最小成本路径规划
version=0.1.0
author=A,B
email=a@example.com
about=课程作业
experimental=True
"""

# ---------- __init__.py ----------
FILES["adaptive_lcp/__init__.py"] = """def classFactory(iface):
    from .plugin import AdaptiveLCPPlugin
    return AdaptiveLCPPlugin(iface)
"""

# ---------- plugin.py ----------
FILES["adaptive_lcp/plugin.py"] = """from qgis.PyQt.QtWidgets import QAction

from .dialog import AdaptiveLCPDialog


class AdaptiveLCPPlugin:
    def __init__(self, iface):
        self.iface = iface
        self.action = None
        self.dialog = None

    def initGui(self):
        self.action = QAction('自适应最小成本路径', self.iface.mainWindow())
        self.action.triggered.connect(self.run)
        self.iface.addToolBarIcon(self.action)
        self.iface.addPluginToMenu('Adaptive LCP', self.action)

    def unload(self):
        if self.action is not None:
            self.iface.removeToolBarIcon(self.action)
            self.iface.removePluginMenu('Adaptive LCP', self.action)

    def run(self):
        if self.dialog is None:
            self.dialog = AdaptiveLCPDialog(self.iface)
        self.dialog.show()
        self.dialog.raise_()
"""

# ---------- dialog.py ----------
FILES["adaptive_lcp/dialog.py"] = '''from qgis.PyQt.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QPushButton, QMessageBox, QFormLayout,
)

# 注意：界面只负责收集参数，真正的计算全部交给 core/ 层。
# 这样同学 B 可以完全脱离 QGIS 开发算法。


class AdaptiveLCPDialog(QDialog):
    def __init__(self, iface, parent=None):
        super().__init__(parent or iface.mainWindow())
        self.iface = iface
        self.setWindowTitle('Adaptive Least-Cost Path')
        self.resize(520, 420)
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.cmb_cost = QComboBox()
        self.cmb_cost.addItems(['来自 DEM 自动生成', '已有成本栅格'])
        form.addRow('成本来源：', self.cmb_cost)

        self.cmb_pref = QComboBox()
        self.cmb_pref.addItems(['自动', '快速', '平衡', '高精度'])
        form.addRow('求解模式：', self.cmb_pref)

        layout.addLayout(form)
        layout.addStretch(1)

        btns = QHBoxLayout()
        self.btn_run = QPushButton('开始分析')
        self.btn_run.clicked.connect(self.on_run)
        self.btn_close = QPushButton('关闭')
        self.btn_close.clicked.connect(self.reject)
        btns.addStretch(1)
        btns.addWidget(self.btn_run)
        btns.addWidget(self.btn_close)
        layout.addLayout(btns)

    def on_run(self):
        # MVP：先接 core/solver，后面再接 io/ 读真实 DEM
        QMessageBox.information(
            self, '提示',
            '界面骨架已就绪。下一步接入 io/raster_loader 和 core/solver。'
        )
'''

# ---------- core/__init__.py ----------
FILES["adaptive_lcp/core/__init__.py"] = ""

# ---------- core/types.py ----------
FILES["adaptive_lcp/core/types.py"] = '''"""全局数据结构。模块之间只通过这些类型传数据。"""
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
'''

# ---------- core/cost/__init__.py ----------
FILES["adaptive_lcp/core/cost/__init__.py"] = ""

# ---------- core/cost/base.py ----------
FILES["adaptive_lcp/core/cost/base.py"] = '''"""方向惩罚模型。第一版返回 1.0，第二版接入坡向数组。"""


class DirectionModel:
    def __init__(self, k_up: float = 0.0, k_down: float = 0.0):
        self.k_up = k_up
        self.k_down = k_down

    def multiplier(self, theta: float, rows, cols):
        return 1.0

    def is_identity(self) -> bool:
        return self.k_up == 0.0 and self.k_down == 0.0
'''

# ---------- core/cost/segment_cost.py ----------
FILES["adaptive_lcp/core/cost/segment_cost.py"] = '''"""线积分边成本：Cost = Σ C_k · P_k(θ) · L_k

这是 A* 和 Any-angle A* 共用的唯一成本口径。改动前必须双方确认。
"""
from __future__ import annotations

import math

import numpy as np


def sample_cells(p0, p1, grid, step_frac: float = 0.25):
    """沿 p0->p1 采样，返回 (rows, cols, ds, L)。

    若任一点越界则返回 None。该函数也被 Any-angle A* 的视线判断复用。
    """
    x0, y0 = p0
    x1, y1 = p1
    length = math.hypot(x1 - x0, y1 - y0)
    if length == 0.0:
        return None

    gt = grid.gt
    px = min(abs(gt.dx), abs(gt.dy))
    n = max(2, int(math.ceil(length / (px * step_frac))) + 1)

    t = np.linspace(0.0, 1.0, n)
    xs = x0 + (x1 - x0) * t
    ys = y0 + (y1 - y0) * t

    cols = np.floor((xs - gt.x0) / gt.dx).astype(np.int64)
    rows = np.floor((ys - gt.y0) / gt.dy).astype(np.int64)

    H, W = grid.cost.shape
    if (rows < 0).any() or (rows >= H).any() or (cols < 0).any() or (cols >= W).any():
        return None

    return rows, cols, length / (n - 1), length


def segment_cost(p0, p1, grid, direction_model=None, step_frac: float = 0.25) -> float:
    res = sample_cells(p0, p1, grid, step_frac)
    if res is None:
        return math.inf

    rows, cols, ds, length = res
    c = grid.cost[rows, cols].astype(np.float64)
    if not np.all(np.isfinite(c)):
        return math.inf

    if direction_model is not None:
        theta = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        c = c * direction_model.multiplier(theta, rows, cols)

    return float((c * ds).sum())


def line_of_sight(p0, p1, grid, step_frac: float = 0.5) -> bool:
    """Any-angle A* 用：路径经过的所有像元是否全部可通行。"""
    res = sample_cells(p0, p1, grid, step_frac)
    if res is None:
        return False
    rows, cols, _, _ = res
    return bool(np.all(np.isfinite(grid.cost[rows, cols])))
'''

# ---------- core/cost/cost_builder.py ----------
FILES["adaptive_lcp/core/cost/cost_builder.py"] = '''"""从坡度/土地利用/道路合成基础成本 C0。

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
'''

# ---------- core/geometry/__init__.py ----------
FILES["adaptive_lcp/core/geometry/__init__.py"] = ""

# ---------- core/geometry/entry_exit_handler.py ----------
FILES["adaptive_lcp/core/geometry/entry_exit_handler.py"] = '''"""把点 / 多点 / 线段 转成真实世界坐标候选集。

核心：不做像元中心吸附，真实坐标一路保留到 solver。
"""
from __future__ import annotations

import math


def points_to_candidates(points):
    return [tuple(map(float, p)) for p in points]


def boundary_to_candidates(p0, p1, step: float):
    """沿线段按 step 世界单位均匀取点，包含两端。"""
    x0, y0 = p0
    x1, y1 = p1
    L = math.hypot(x1 - x0, y1 - y0)
    if L == 0 or step <= 0:
        return [(x0, y0)]
    n = max(2, int(math.ceil(L / step)) + 1)
    return [(x0 + (x1 - x0) * i / (n - 1),
             y0 + (y1 - y0) * i / (n - 1)) for i in range(n)]
'''

# ---------- core/analysis/__init__.py ----------
FILES["adaptive_lcp/core/analysis/__init__.py"] = ""

# ---------- core/analysis/algorithm_selector.py ----------
FILES["adaptive_lcp/core/analysis/algorithm_selector.py"] = '''from __future__ import annotations


def select_solver(problem) -> str:
    pref = problem.preference
    grid = problem.grid
    n = grid.n_valid

    if pref == "fast":
        return "astar"
    if pref == "precise":
        return "any_angle"
    if pref == "balanced" and len(problem.entries) > 1:
        return "any_angle"
    if grid.cost_cv > 1.5 and n < 2_000_000:
        return "any_angle"
    return "astar"
'''

# ---------- core/solver/__init__.py ----------
FILES["adaptive_lcp/core/solver/__init__.py"] = ""

# ---------- core/solver/base.py ----------
FILES["adaptive_lcp/core/solver/base.py"] = '''from __future__ import annotations

from abc import ABC, abstractmethod

from ..types import PathProblem, PathResult


class Solver(ABC):
    name = "base"
    supports_any_angle = False
    supports_direction = True

    @abstractmethod
    def solve(self, problem: PathProblem) -> PathResult:
        ...
'''

# ---------- core/solver/astar.py ----------
FILES["adaptive_lcp/core/solver/astar.py"] = '''from __future__ import annotations

import heapq
import math
import time

import numpy as np

from .base import Solver
from ..types import PathProblem, PathResult
from ..cost.segment_cost import segment_cost

NEIGHBORS = [(-1, -1), (-1, 0), (-1, 1),
             (0, -1), (0, 1),
             (1, -1), (1, 0), (1, 1)]


class AStarSolver(Solver):
    name = "astar"

    def solve(self, problem: PathProblem) -> PathResult:
        t0 = time.perf_counter()
        grid = problem.grid
        cost = grid.cost
        gt = grid.gt
        dm = problem.direction_model
        H, W = cost.shape

        def to_cell(x, y):
            return (int(math.floor((y - gt.y0) / gt.dy)),
                    int(math.floor((x - gt.x0) / gt.dx)))

        def in_grid(r, c):
            return 0 <= r < H and 0 <= c < W

        starts = [(x, y) for (x, y) in problem.entries
                  if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]
        goals = [(x, y) for (x, y) in problem.exits
                 if in_grid(*to_cell(x, y)) and np.isfinite(cost[to_cell(x, y)])]

        if not starts or not goals:
            return PathResult(ok=False, solver=self.name,
                              runtime=time.perf_counter() - t0,
                              message="入口/出口越界或落在不可通行像元")

        goal_cells = {to_cell(x, y) for (x, y) in goals}
        goal_pts = [gt.pixel_to_world(c, r) for (r, c) in goal_cells]
        cmin = grid.min_cost

        def h(r, c):
            x, y = gt.pixel_to_world(c, r)
            return min(math.hypot(x - gx, y - gy) for gx, gy in goal_pts) * cmin

        g = {}
        parent = {}
        heap = []
        k = 0

        for (x, y) in starts:
            r, c = to_cell(x, y)
            cx, cy = gt.pixel_to_world(c, r)
            enter = segment_cost((x, y), (cx, cy), grid, dm)
            if not math.isfinite(enter):
                enter = 0.0
            if enter < g.get((r, c), math.inf):
                g[(r, c)] = enter
                parent[(r, c)] = None
                heapq.heappush(heap, (enter + h(r, c), k, enter, r, c))
                k += 1

        closed = set()
        visited = 0
        found = None

        while heap:
            _, _, gc, r, c = heapq.heappop(heap)
            if (r, c) in closed or gc > g.get((r, c), math.inf):
                continue
            closed.add((r, c))
            visited += 1
            if (r, c) in goal_cells:
                found = (r, c)
                break

            x0w, y0w = gt.pixel_to_world(c, r)
            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if not in_grid(nr, nc) or not np.isfinite(cost[nr, nc]):
                    continue
                if dr and dc:
                    if not (np.isfinite(cost[r, nc]) and np.isfinite(cost[nr, c])):
                        continue
                x1w, y1w = gt.pixel_to_world(nc, nr)
                step = segment_cost((x0w, y0w), (x1w, y1w), grid, dm)
                if not math.isfinite(step):
                    continue
                ng = gc + step
                if ng < g.get((nr, nc), math.inf):
                    g[(nr, nc)] = ng
                    parent[(nr, nc)] = (r, c)
                    heapq.heappush(heap, (ng + h(nr, nc), k, ng, nr, nc))
                    k += 1

        rt = time.perf_counter() - t0
        if found is None:
            return PathResult(ok=False, solver=self.name, visited=visited,
                              runtime=rt, message="未找到可行路径")

        cells = []
        cur = found
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()

        start_pt = next(((x, y) for (x, y) in starts if to_cell(x, y) == cells[0]),
                        gt.pixel_to_world(cells[0][1], cells[0][0]))
        end_pt = next(((x, y) for (x, y) in goals if to_cell(x, y) == cells[-1]),
                      gt.pixel_to_world(cells[-1][1], cells[-1][0]))

        path = [start_pt] + [gt.pixel_to_world(c, r) for (r, c) in cells] + [end_pt]
        length = sum(math.hypot(path[i + 1][0] - path[i][0],
                                path[i + 1][1] - path[i][1])
                     for i in range(len(path) - 1))

        return PathResult(ok=True, path=path, total_cost=g[found],
                          length=length, visited=visited, runtime=rt,
                          solver=self.name)
'''

# ---------- core/solver/any_angle_astar.py ----------
FILES["adaptive_lcp/core/solver/any_angle_astar.py"] = '''"""Theta* 风格 Any-angle A*。

实现思路：在 A* 的松弛阶段，如果当前节点 s 与其祖父节点 ps 之间视线可达，
直接把 ps 设为 (nr,nc) 的父节点，从而"跨接"父指针，让路径脱离 8 邻域。

当前为占位实现（暂用标准 A* 行为），同学 B 按 TODO 完成即可。
"""
from __future__ import annotations

from .astar import AStarSolver


class AnyAngleAStarSolver(AStarSolver):
    name = "any_angle"
    supports_any_angle = True

    # TODO(B): 重写 solve 的松弛循环
    #   ps = parent[s]
    #   if ps is not None and line_of_sight(cell_world(ps),
    #                                       cell_world((nr, nc)), grid):
    #       ng = g[ps] + segment_cost(cell_world(ps), cell_world((nr, nc)), grid, dm)
    #       if ng < g.get((nr, nc), inf):
    #           g[(nr, nc)] = ng
    #           parent[(nr, nc)] = ps        # 跨接
    #           heappush(...)
    #   注意弹堆时必须检查 gc > g[(r,c)] 跳过失效条目。
'''

# ---------- core/solver/registry.py ----------
FILES["adaptive_lcp/core/solver/registry.py"] = '''from .astar import AStarSolver
from .any_angle_astar import AnyAngleAStarSolver

REGISTRY = {s.name: s for s in (AStarSolver(), AnyAngleAStarSolver())}


def get_solver(name: str):
    return REGISTRY[name]
'''

# ---------- io/__init__.py ----------
FILES["adaptive_lcp/io/__init__.py"] = ""

# ---------- io/raster_loader.py ----------
FILES["adaptive_lcp/io/raster_loader.py"] = '''"""QgsRasterLayer -> CostGrid。只有这个文件允许 import qgis。"""
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
'''

# ---------- io/terrain.py ----------
FILES["adaptive_lcp/io/terrain.py"] = '''"""调用 QGIS Processing 计算坡度/坡向，返回 numpy 数组。"""


def slope_as_array(dem_layer):
    # TODO(A): 使用 processing.run("qgis:slope", {...}) 或 gdal:slope
    # 结果图层 -> io.raster_loader 复用读数组逻辑
    raise NotImplementedError
'''

# ---------- io/output_writer.py ----------
FILES["adaptive_lcp/io/output_writer.py"] = '''"""PathResult -> QgsVectorLayer，并加入地图。"""


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
'''

# ---------- tests ----------
FILES["adaptive_lcp/tests/__init__.py"] = ""

FILES["adaptive_lcp/tests/conftest.py"] = '''import numpy as np
import pytest

from adaptive_lcp.core.types import CostGrid, GeoTransform


@pytest.fixture
def simple_grid():
    """10x10 平原，成本 1.0，第 5 列 3~6 行一堵墙。"""
    cost = np.ones((10, 10), dtype=np.float32)
    cost[3:7, 5] = np.inf
    gt = GeoTransform(x0=0.0, dx=1.0, y0=10.0, dy=-1.0)
    return CostGrid(cost=cost, gt=gt)
'''

FILES["adaptive_lcp/tests/test_astar.py"] = '''from adaptive_lcp.core.types import PathProblem
from adaptive_lcp.core.solver.astar import AStarSolver


def test_simple_grid_reachable(simple_grid):
    """起终点在同一行且墙不挡住，但这里故意放同一行验证能到。"""
    p = PathProblem(grid=simple_grid, entries=[(0.5, 0.5)], exits=[(9.5, 0.5)])
    r = AStarSolver().solve(p)
    assert r.ok
    assert r.total_cost > 0
    assert r.visited > 0


def test_go_around_wall(simple_grid):
    """起点左侧、终点右侧，必须绕过墙。"""
    p = PathProblem(grid=simple_grid, entries=[(1.5, 4.5)], exits=[(8.5, 4.5)])
    r = AStarSolver().solve(p)
    assert r.ok
    assert r.length > 7.0   # 直线 7，绕墙一定更长


def test_unreachable():
    import numpy as np
    from adaptive_lcp.core.types import CostGrid, GeoTransform
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, :] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    p = PathProblem(grid=grid, entries=[(0.5, 0.5)], exits=[(0.5, 4.5)])
    r = AStarSolver().solve(p)
    assert not r.ok
'''

FILES["adaptive_lcp/tests/test_segment_cost.py"] = '''import numpy as np

from adaptive_lcp.core.cost.segment_cost import segment_cost, line_of_sight
from adaptive_lcp.core.types import CostGrid, GeoTransform


def test_cost_along_axis():
    cost = np.full((5, 5), 2.0, dtype=np.float32)
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    c = segment_cost((0.5, 0.5), (4.5, 0.5), grid)
    assert abs(c - 2.0 * 4.0) < 1e-6


def test_cost_blocked():
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, 2] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    assert segment_cost((0.5, 0.5), (4.5, 4.5), grid) == np.inf


def test_line_of_sight_false_through_wall():
    cost = np.ones((5, 5), dtype=np.float32)
    cost[2, 2] = np.inf
    grid = CostGrid(cost=cost, gt=GeoTransform(0.0, 1.0, 5.0, -1.0))
    assert not line_of_sight((0.5, 0.5), (4.5, 4.5), grid)
'''

# ---------- 根目录杂项 ----------
FILES[".gitignore"] = """__pycache__/
*.pyc
.idea/
.pytest_cache/
*.tif
*.gpkg
*.qgz
dist/
build/
.env
"""

FILES["requirements-dev.txt"] = """numpy>=1.24
pytest>=7.0
pytest-cov
"""

FILES["pytest.ini"] = """[pytest]
testpaths = adaptive_lcp/tests
pythonpath = .
"""

FILES["scripts/run_tests.bat"] = """@echo off
cd /d %~dp0\\..
python -m pytest -q
pause
"""

FILES["scripts/make_dev_link.bat"] = """@echo off
REM 用法：修改下方 QGIS_PLUGINS 为你本机 QGIS 插件目录后运行
set QGIS_PLUGINS=%APPDATA%\\QGIS\\QGIS3\\profiles\\default\\python\\plugins
set PROJECT=%~dp0..\\adaptive_lcp
if not exist "%QGIS_PLUGINS%" mkdir "%QGIS_PLUGINS%"
mklink /J "%QGIS_PLUGINS%\\adaptive_lcp" "%PROJECT%"
echo 已建立软链接到 %QGIS_PLUGINS%
pause
"""

FILES["CONTRACT.md"] = """# 接口契约（改动需双方确认）

## 冻结文件
- `core/types.py`          —— 数据结构
- `core/cost/segment_cost.py` —— 唯一成本口径
- `core/solver/base.py`    —— Solver 抽象
- `core/cost/base.py`      —— DirectionModel

## 铁律
1. 所有 `(x, y)` 参数 = 世界坐标；`(r, c)` = 数组索引。禁止混用。
2. `core/` 下禁止 `import qgis` / `import processing`。
3. `np.inf` 表示不可通行。
4. 对角移动禁止切角。

## 责任人
- A：`plugin.py` `dialog.py` `io/*` `scripts/*`
- B：`core/solver/*` `core/geometry/*` `core/analysis/*` `tests/*`
- 共用：以上冻结文件
"""


def _write(rel: str, content: str):
    path = TARGET / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    print(f"  + {rel}")


def main():
    print(f"生成目标：{TARGET.resolve()}")
    PKG.mkdir(parents=True, exist_ok=True)

    # 空目录（git 不追踪空目录，放 .gitkeep）
    for d in ["adaptive_lcp/docs", "adaptive_lcp/scripts"]:
        p = TARGET / d
        p.mkdir(parents=True, exist_ok=True)
        (p / ".gitkeep").touch()

    for rel, content in FILES.items():
        _write(rel, textwrap.dedent(content))

    print()
    print(f"完成。共 {len(FILES)} 个文件。")
    print()
    print("下一步：")
    print(f"  cd {TARGET}")
    print("  python -m pytest -q        # 应该全部通过")
    print("  git init && git add . && git commit -m 'M0: 契约与骨架'")


if __name__ == "__main__":
    main()