from qgis.PyQt.QtWidgets import (
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
