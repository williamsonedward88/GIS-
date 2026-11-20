from qgis.PyQt.QtWidgets import QAction

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
