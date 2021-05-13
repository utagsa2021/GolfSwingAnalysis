# ------------------------------------------------- -----
# -------------------- mplwidget.py --------------------
# -------------------------------------------------- ----
from PyQt5.QtWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class MplWidget3D(QWidget):
    def __init__(self, parent=None):

        QWidget.__init__(self, parent)

        self.canvas = FigureCanvas(Figure())

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        fig = plt.figure(figsize=(4, 4))

        self.canvas.axes = self.canvas.figure.add_subplot(111, projection="3d")
        self.setLayout(vertical_layout)


class MplWidget(QWidget):
    def __init__(self, parent=None):

        QWidget.__init__(self, parent)

        self.canvas = FigureCanvas(Figure())

        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)

        self.canvas.axes = self.canvas.figure.add_subplot(111)
        self.setLayout(vertical_layout)
