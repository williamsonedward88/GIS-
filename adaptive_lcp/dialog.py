"""Adaptive LCP 对话框 —— 文件浏览 + 进度条 + 分阶段计时。

数据来源支持：
  ① QGIS 项目已加载图层
  ② 本地文件浏览（选后自动加入项目，方便地图点选）
"""
import os
import time

import numpy as np

from qgis.PyQt.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QMessageBox, QFormLayout, QDoubleSpinBox,
    QGroupBox, QProgressBar, QRadioButton, QButtonGroup,
    QLineEdit, QFileDialog, QApplication, QWidget, QFrame,
)
from qgis.PyQt.QtCore import Qt
from qgis.core import QgsProject, QgsRasterLayer, QgsMessageLog
from qgis.gui import QgsMapToolEmitPoint

from .core.types import PathProblem
from .core.cost.cost_builder import build_c0_from_dem
from .core.solver.registry import get_solver
from .core.analysis.algorithm_selector import select_solver
from .io.raster_loader import load_cost_layer
from .io.terrain import slope_as_array
from .io.output_writer import (
    write_raster, load_raster_to_qgis, result_to_layer,
)

LOG_TAG = "AdaptiveLCP"


def _log(msg):
    QgsMessageLog.logMessage(str(msg), LOG_TAG)


def _fmt(sec: float) -> str:
    if sec < 1.0:
        return f"{sec * 1000:.0f} ms"
    return f"{sec:.2f} s"


class AdaptiveLCPDialog(QDialog):
    def __init__(self, iface, parent=None):
        super().__init__(parent or iface.mainWindow())
        self.iface = iface

        self._pending_tool = None
        self._pending_target = None
        self._pending_layer = None
        self._t0 = None
        self._stage_times: dict[str, float] = {}
        self._last_dir = os.path.expanduser("~")

        self.setWindowTitle("Adaptive Least-Cost Path")
        self.resize(640, 720)
        self._build_ui()
        self._refresh_layers()

    # ==================== UI ====================
    def _build_ui(self):
        layout = QVBoxLayout(self)

        # ---------- ① 数据 ----------
        g_data = QGroupBox("① 数据")
        v_data = QVBoxLayout(g_data)

        # 来源切换
        row_mode = QHBoxLayout()
        self.radio_project = QRadioButton("QGIS 项目图层")
        self.radio_file = QRadioButton("本地文件")
        self.radio_project.setChecked(True)
        grp = QButtonGroup(self)
        grp.addButton(self.radio_project)
        grp.addButton(self.radio_file)
        row_mode.addWidget(QLabel("来源："))
        row_mode.addWidget(self.radio_project)
        row_mode.addWidget(self.radio_file)
        row_mode.addStretch(1)
        v_data.addLayout(row_mode)

        # 项目图层行
        self.w_project = QWidget()
        r = QHBoxLayout(self.w_project)
        r.setContentsMargins(0, 0, 0, 0)
        self.cmb_dem = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self._refresh_layers)
        r.addWidget(QLabel("图层："))
        r.addWidget(self.cmb_dem, 1)
        r.addWidget(self.btn_refresh)
        v_data.addWidget(self.w_project)

        # 文件行
        self.w_file = QWidget()
        r2 = QHBoxLayout(self.w_file)
        r2.setContentsMargins(0, 0, 0, 0)
        self.edit_path = QLineEdit()
        self.edit_path.setPlaceholderText("选择 .tif / .img / .vrt 文件")
        self.btn_browse = QPushButton("浏览…")
        self.btn_browse.clicked.connect(self._browse_file)
        r2.addWidget(QLabel("文件："))
        r2.addWidget(self.edit_path, 1)
        r2.addWidget(self.btn_browse)
        v_data.addWidget(self.w_file)
        self.w_file.setVisible(False)

        self.radio_project.toggled.connect(self._on_mode_changed)

        # 数据类型
        self.cmb_source = QComboBox()
        self.cmb_source.addItems([
            "来自 DEM 自动生成成本",
            "DEM 本身就是成本栅格",
        ])
        fr = QFormLayout()
        fr.setContentsMargins(0, 6, 0, 0)
        fr.addRow("数据类型：", self.cmb_source)
        v_data.addLayout(fr)

        layout.addWidget(g_data)

        # ---------- ② 入口 / 出口 ----------
        g_pts = QGroupBox("② 入口 / 出口（世界坐标）")
        f_pts = QFormLayout(g_pts)

        row_e = QHBoxLayout()
        self.spin_ex = self._mk_spin()
        self.spin_ey = self._mk_spin()
        self.btn_pick_entry = QPushButton("地图点击")
        self.btn_pick_entry.clicked.connect(lambda: self._pick_from_canvas("entry"))
        row_e.addWidget(QLabel("X")); row_e.addWidget(self.spin_ex)
        row_e.addWidget(QLabel("Y")); row_e.addWidget(self.spin_ey)
        row_e.addWidget(self.btn_pick_entry)
        f_pts.addRow("入口：", row_e)

        row_x = QHBoxLayout()
        self.spin_xx = self._mk_spin()
        self.spin_xy = self._mk_spin()
        self.btn_pick_exit = QPushButton("地图点击")
        self.btn_pick_exit.clicked.connect(lambda: self._pick_from_canvas("exit"))
        row_x.addWidget(QLabel("X")); row_x.addWidget(self.spin_xx)
        row_x.addWidget(QLabel("Y")); row_x.addWidget(self.spin_xy)
        row_x.addWidget(self.btn_pick_exit)
        f_pts.addRow("出口：", row_x)

        self.btn_auto_ends = QPushButton("用栅格范围自动填一对端点（测试用）")
        self.btn_auto_ends.clicked.connect(self._auto_fill_ends)
        f_pts.addRow("", self.btn_auto_ends)

        layout.addWidget(g_pts)

        # ---------- ③ 求解 ----------
        g_solve = QGroupBox("③ 求解")
        f_solve = QFormLayout(g_solve)
        self.cmb_pref = QComboBox()
        self.cmb_pref.addItems(["自动", "快速", "平衡", "高精度"])
        f_solve.addRow("模式：", self.cmb_pref)
        layout.addWidget(g_solve)

        # ---------- ④ 进度 ----------
        g_prog = QGroupBox("④ 进度")
        v_prog = QVBoxLayout(g_prog)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.lbl_stage = QLabel("就绪")
        self.lbl_stage.setStyleSheet("color:#555;")
        v_prog.addWidget(self.progress)
        v_prog.addWidget(self.lbl_stage)
        layout.addWidget(g_prog)

        # ---------- ⑤ 结果面板 ----------
        g_res = QGroupBox("⑤ 上次运行")
        v_res = QVBoxLayout(g_res)
        self.lbl_result = QLabel("（尚未运行）")
        self.lbl_result.setTextFormat(Qt.RichText)
        self.lbl_result.setWordWrap(True)
        self.lbl_result.setFrameShape(QFrame.StyledPanel)
        self.lbl_result.setStyleSheet(
            "QLabel { background:#fafafa; border:1px solid #ddd; padding:8px;"
            " font-family:Consolas,monospace; font-size:12px; }"
        )
        v_res.addWidget(self.lbl_result)
        layout.addWidget(g_res)

        # ---------- 按钮 ----------
        btn_row = QHBoxLayout()
        self.btn_run = QPushButton("开始分析")
        self.btn_run.clicked.connect(self.on_run)
        self.btn_close = QPushButton("关闭")
        self.btn_close.clicked.connect(self.reject)
        btn_row.addStretch(1)
        btn_row.addWidget(self.btn_run)
        btn_row.addWidget(self.btn_close)
        layout.addLayout(btn_row)

    def _mk_spin(self):
        s = QDoubleSpinBox()
        s.setRange(-1e9, 1e9)
        s.setDecimals(6)
        s.setMinimumWidth(110)
        s.setButtonSymbols(QDoubleSpinBox.NoButtons)
        return s

    # ==================== 数据来源 ====================
    def _on_mode_changed(self):
        proj = self.radio_project.isChecked()
        self.w_project.setVisible(proj)
        self.w_file.setVisible(not proj)

    def _browse_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "选择 DEM / 成本栅格",
            self._last_dir,
            "栅格文件 (*.tif *.tiff *.img *.vrt *.asc);;所有文件 (*.*)",
        )
        if not path:
            return
        self._last_dir = os.path.dirname(path)
        self.edit_path.setText(path)
        try:
            lyr = self._ensure_layer_in_project(path)
            _log(f"已加载栅格：{lyr.name()}，尺寸 {lyr.width()}x{lyr.height()}")
            self._auto_fill_ends()
        except Exception as e:
            QMessageBox.warning(self, "加载失败", str(e))
            _log(f"加载失败：{e}")

    def _ensure_layer_in_project(self, path: str):
        """若项目里已有同路径图层就复用，否则新建并加入项目。"""
        apath = os.path.abspath(path)
        for lyr in QgsProject.instance().mapLayers().values():
            if isinstance(lyr, QgsRasterLayer):
                try:
                    if os.path.abspath(lyr.source()) == apath:
                        return lyr
                except Exception:
                    pass

        lyr = QgsRasterLayer(path, os.path.splitext(os.path.basename(path))[0])
        if not lyr.isValid():
            raise ValueError(
                f"QGIS 无法识别该栅格：{os.path.basename(path)}\n"
                f"可能不是 GeoTIFF/IMG/VRT，或者文件损坏。"
            )
        QgsProject.instance().addMapLayer(lyr)
        return lyr

    # ==================== 图层下拉 ====================
    def _refresh_layers(self):
        keep = self.cmb_dem.currentData()
        self.cmb_dem.clear()
        n = 0
        for lyr in QgsProject.instance().mapLayers().values():
            if isinstance(lyr, QgsRasterLayer):
                self.cmb_dem.addItem(lyr.name(), lyr.id())
                n += 1
        if n == 0:
            self.cmb_dem.addItem("（项目中无栅格图层）", None)
        elif keep:
            idx = self.cmb_dem.findData(keep)
            if idx >= 0:
                self.cmb_dem.setCurrentIndex(idx)

    def showEvent(self, ev):
        super().showEvent(ev)
        self._refresh_layers()

    def _resolve_current_raster(self):
        """返回 QgsRasterLayer，失败抛 ValueError。"""
        if self.radio_project.isChecked():
            lid = self.cmb_dem.currentData()
            if not lid:
                raise ValueError("项目中还没有栅格图层，请切换到「本地文件」浏览选择")
            lyr = QgsProject.instance().mapLayer(lid)
            if lyr is None:
                raise ValueError("图层 id 已失效，请点「刷新」重新选择")
            return lyr

        path = self.edit_path.text().strip()
        if not path:
            raise ValueError("请先点「浏览…」选择 DEM 文件")
        if not os.path.exists(path):
            raise ValueError(f"文件不存在：{path}")
        return self._ensure_layer_in_project(path)

    # ==================== 地图点选 ====================
    def _pick_from_canvas(self, target: str):
        try:
            dem = self._resolve_current_raster()
        except ValueError as e:
            QMessageBox.warning(self, "无栅格", str(e))
            return

        self.hide()
        canvas = self.iface.mapCanvas()
        tool = QgsMapToolEmitPoint(canvas)
        self._pending_tool = tool
        self._pending_target = target
        self._pending_layer = dem
        tool.canvasClicked.connect(self._on_canvas_clicked)
        canvas.setMapTool(tool)

        QMessageBox.information(self, "点选", "请在地图上点击一个点，然后窗口会自动返回")

    def _on_canvas_clicked(self, point, button):
        lyr = self._pending_layer
        try:
            pt = self.iface.mapCanvas().mapSettings().mapToLayerCoordinates(lyr, point)
            x, y = pt.x(), pt.y()
        except Exception as e:
            _log(f"坐标转换失败，退回画布坐标：{e}")
            x, y = point.x(), point.y()

        if self._pending_target == "entry":
            self.spin_ex.setValue(x); self.spin_ey.setValue(y)
            _log(f"入口 = ({x:.4f}, {y:.4f})")
        else:
            self.spin_xx.setValue(x); self.spin_xy.setValue(y)
            _log(f"出口 = ({x:.4f}, {y:.4f})")

        try:
            self.iface.mapCanvas().unsetMapTool(self._pending_tool)
        except Exception:
            pass
        self._pending_tool = None
        self.show()
        self.raise_()

    def _auto_fill_ends(self):
        try:
            dem = self._resolve_current_raster()
        except ValueError as e:
            QMessageBox.warning(self, "无栅格", str(e))
            return
        ext = dem.extent()
        xmin, xmax = ext.xMinimum(), ext.xMaximum()
        ymin, ymax = ext.yMinimum(), ext.yMaximum()
        x0 = xmin + (xmax - xmin) * 0.30
        y0 = ymin + (ymax - ymin) * 0.30
        x1 = xmin + (xmax - xmin) * 0.70
        y1 = ymin + (ymax - ymin) * 0.70
        self.spin_ex.setValue(x0); self.spin_ey.setValue(y0)
        self.spin_xx.setValue(x1); self.spin_xy.setValue(y1)
        _log(f"自动端点：({x0:.3f}, {y0:.3f}) → ({x1:.3f}, {y1:.3f})")

    # ==================== 进度 ====================
    def _progress_set(self, value: int, text: str = ""):
        self.progress.setRange(0, 100)
        self.progress.setValue(int(value))
        if text:
            self.lbl_stage.setText(text)
        QApplication.processEvents()

    def _progress_busy(self, text: str):
        self.progress.setRange(0, 0)   # 不确定模式：来回滚动
        self.lbl_stage.setText(text)
        QApplication.processEvents()

    def _stage(self, name: str, start: int, end: int, fn):
        """执行 fn，记录耗时，推进进度条。"""
        self._progress_set(start, f"{name}…")
        t = time.perf_counter()
        try:
            out = fn()
        finally:
            self._stage_times[name] = time.perf_counter() - t
        self._progress_set(end, f"{name} 完成（{_fmt(self._stage_times[name])}）")
        return out

    # ==================== 执行 ====================
    def on_run(self):
        if self._t0 is not None:
            return  # 正在运行，忽略
        self.btn_run.setEnabled(False)
        self.btn_close.setEnabled(False)
        self._stage_times = {}
        self._t0 = time.perf_counter()
        try:
            self._do_run()
        except Exception as e:
            import traceback
            tb = traceback.format_exc()
            _log(tb)
            self._progress_set(0, f"失败：{e}")
            QMessageBox.critical(
                self, "运行出错",
                f"{e}\n\n详见 视图 → 面板 → 日志消息 → AdaptiveLCP"
            )
        finally:
            self._t0 = None
            self.btn_run.setEnabled(True)
            self.btn_close.setEnabled(True)

    def _do_run(self):
        # ---- 0. 参数 ----
        dem = self._resolve_current_raster()
        pref_map = {"自动": "auto", "快速": "fast", "平衡": "balanced", "高精度": "precise"}
        pref = pref_map[self.cmb_pref.currentText()]
        direct_cost = (self.cmb_source.currentText() == "DEM 本身就是成本栅格")
        _log(f"=== 开始 ===\nDEM={dem.name()} 尺寸={dem.width()}x{dem.height()} "
             f"模式={pref} 直读成本={direct_cost}")
        self._progress_set(0, "初始化…")

        entry = (self.spin_ex.value(), self.spin_ey.value())
        exit_ = (self.spin_xx.value(), self.spin_xy.value())
        self._last_entry = entry
        self._last_exit = exit_
        _log(f"入口={entry} 出口={exit_}")

        # ---- 1. 数据准备（坡度 / 直读） ----
        if direct_cost:
            grid = self._stage("读取成本栅格", 0, 25,
                               lambda: load_cost_layer(dem))
            slope = gt = crs_wkt = None
        else:
            def _do_slope():
                s, g, c = slope_as_array(dem)
                v = np.isfinite(s)
                if not v.any():
                    raise ValueError("坡度结果全部无效，DEM 可能全是 NoData")
                _log(f"坡度 min={float(s[v].min()):.3f} "
                     f"max={float(s[v].max()):.3f} mean={float(s[v].mean()):.3f}")
                return s, g, c

            slope, gt, crs_wkt = self._stage("坡度计算", 0, 35, _do_slope)
            grid = self._stage("成本构建", 38, 48,
                               lambda: build_c0_from_dem(slope, gt, crs_wkt=crs_wkt))
            finite = np.isfinite(grid.cost)
            if not finite.any():
                raise ValueError("成本栅格全部不可通行")
            _log(f"成本 min={float(grid.cost[finite].min()):.3f} "
                 f"max={float(grid.cost[finite].max()):.3f} "
                 f"有效像元={int(finite.sum())}")

        # ---- 2. 求解 ----
        problem = PathProblem(grid=grid, entries=[entry], exits=[exit_], preference=pref)
        solver_name = select_solver(problem)
        _log(f"求解器={solver_name}")
        solver = get_solver(solver_name)

        self._progress_busy(f"路径求解中（{solver_name}）…")
        t_solve = time.perf_counter()
        try:
            result = solver.solve(problem)
        finally:
            self._stage_times["路径求解"] = time.perf_counter() - t_solve

        if not result.ok:
            self._progress_set(48, f"求解失败：{result.message}")
            raise RuntimeError(result.message or "未找到可行路径")
        self._progress_set(85, f"路径求解 完成（{_fmt(self._stage_times['路径求解'])}）")

        # ---- 3. 输出 ----
        def _write_all():
            if not direct_cost:
                import tempfile
                out_dir = tempfile.mkdtemp(prefix="adaptive_lcp_")
                sp = os.path.join(out_dir, "slope_deg.tif")
                cp = os.path.join(out_dir, "cost.tif")
                write_raster(slope, gt, crs_wkt, sp, name="slope_deg")
                write_raster(grid.cost, grid.gt, crs_wkt, cp, name="cost")
                load_raster_to_qgis(sp, "slope_deg")
                load_raster_to_qgis(cp, "cost")
                _log(f"已写出 {sp} / {cp}")
            result_to_layer(result, grid.crs_wkt, name=f"path_{result.solver}")
            _log(f"已添加路径图层 path_{result.solver}")

        self._stage("结果输出", 85, 100, _write_all)

        # ---- 4. 汇总 ----
        total = time.perf_counter() - self._t0
        self._show_summary(result, total, direct_cost)

    def _show_summary(self, result, total: float, direct_cost: bool):
        st = self._stage_times
        rows = []
        if not direct_cost:
            rows.append(("坡度计算", st.get("坡度计算", 0.0)))
            rows.append(("成本构建", st.get("成本构建", 0.0)))
        else:
            rows.append(("读取成本", st.get("读取成本栅格", 0.0)))
        rows.append(("路径求解", st.get("路径求解", 0.0)))
        rows.append(("结果输出", st.get("结果输出", 0.0)))

        detail = " &nbsp;|&nbsp; ".join(
            f"{k} <b>{_fmt(v)}</b>" for k, v in rows
        )
        e = getattr(self, "_last_entry", (0.0, 0.0))
        x = getattr(self, "_last_exit", (0.0, 0.0))
        html = f"""
        <table style="border-collapse:collapse;">
        <tr><td style="padding-right:12px;">入口</td>
            <td><b>({e[0]:.1f}, {e[1]:.1f})</b></td></tr>
        <tr><td>出口</td><td><b>({x[0]:.1f}, {x[1]:.1f})</b></td></tr>
        <tr><td>算法</td><td><b>{result.solver}</b></td></tr>
        <tr><td>总成本</td><td><b>{result.total_cost:.3f}</b></td></tr>
        <tr><td>路径长度</td><td><b>{result.length:.3f}</b></td></tr>
        <tr><td>访问节点</td><td><b>{result.visited:,}</b></td></tr>
        <tr><td>总耗时</td><td><b style="color:#c0392b;">{_fmt(total)}</b></td></tr>
        </table>
        <div style="margin-top:6px; color:#555; font-size:11px;">
        阶段：{detail}
        </div>
        """
        self.lbl_result.setText(html)

        QMessageBox.information(
            self, "完成",
            f"算法：{result.solver}\n"
            f"总成本：{result.total_cost:.3f}\n"
            f"路径长度：{result.length:.3f}\n"
            f"访问节点：{result.visited}\n"
            f"总耗时：{_fmt(total)}"
        )