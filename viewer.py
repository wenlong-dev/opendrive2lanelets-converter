
import math
import signal
import sys
import os

from lxml import etree

import numpy as np

import matplotlib
matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.path import Path
from matplotlib.patches import PathPatch, Polygon

from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtWidgets import QApplication, QWidget, QTableWidget, QPushButton, QLineEdit, QFileDialog, QLabel, QMessageBox
from PyQt5.QtWidgets import QSizePolicy, QHBoxLayout, QVBoxLayout, QTableWidgetItem, QAbstractItemView

from opendrive2lanelet.commonroad import Lanelet, LaneletNetwork, Scenario, ScenarioError


class Canvas(FigureCanvas):
    """Ultimately, this is a QWidget """

    def __init__(self, parent=None, width=5, height=5, dpi=100):

        self.ax = None

        self.fig = Figure(figsize=(width, height), dpi=dpi)

        super(Canvas, self).__init__(self.fig)
        self.setParent(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.clear_axes()

    def clear_axes(self):
        if self.ax is not None:
            self.ax.clear()
        else:

            self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect('equal', 'datalim')
        self.ax.set_axis_off()

        self.draw()

    def get_axes(self):
        return self.ax

    def update_plot(self):
        self.draw()

class MainWindow(QWidget):

    def __init__(self, parent=None, path=None):
        super().__init__(parent)

        self.current_scenario = None
        self.selected_lanelet_id = None

        self._initUserInterface()
        self.show()

        if path is not None:
            self.openPath(path)

    def _initUserInterface(self):

        self.setWindowTitle("CommonRoad XML Viewer")

        self.setMinimumSize(1000, 600)

        self.testButton = QPushButton('test', self)
        self.testButton.clicked.connect(self.testCmd)

        self.loadButton = QPushButton('Load CommonRoad', self)
        self.loadButton.setToolTip('Load a CommonRoad scenario within a *.xml file')
        self.loadButton.clicked.connect(self.openCommonRoadFile)

        self.inputCommonRoadFile = QLineEdit(self)
        self.inputCommonRoadFile.setReadOnly(True)

        hbox = QHBoxLayout()
        hbox.addWidget(self.loadButton)
        hbox.addWidget(self.inputCommonRoadFile)

        self.dynamic = Canvas(self, width=5, height=10, dpi=100)

        vbox = QVBoxLayout()
        vbox.addLayout(hbox, 0)
        vbox.addWidget(self.dynamic, 1)

        vbox.setAlignment(Qt.AlignTop)

        self.laneletsList = QTableWidget(self)
        self.laneletsList.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.laneletsList.clicked.connect(self.onClickLanelet)

        hbox2 = QHBoxLayout(self)
        hbox2.addLayout(vbox, 1)
        hbox2.addWidget(self.laneletsList, 0)

        self.setLayout(hbox2)

    def openCommonRoadFile(self):

        path, _ = QFileDialog.getOpenFileName(
            self,
            "QFileDialog.getOpenFileName()",
            "",
            "CommonRoad scenario files *.xml (*.xml)",
            options=QFileDialog.Options()
        )

        if not path:
            return

        self.openPath(path)

    def openPath(self, path):

        filename = os.path.basename(path)
        self.inputCommonRoadFile.setText(filename)

        try:
            fh = open(path, 'r')
            data = fh.read()
            fh.close()

            scenario = Scenario.read_from_string(data)
        except etree.XMLSyntaxError as e:
            errorMsg = 'Syntax Error: {}'.format(e)
            QMessageBox.warning(self, 'CommonRoad XML error', 'There was an error during the loading of the selected CommonRoad file.\n\n{}'.format(errorMsg), QMessageBox.Ok)
            return
        except Exception as e:
            errorMsg = 'Unknown Error: {}'.format(e)
            QMessageBox.warning(self, 'CommonRoad XML error', 'There was an error during the loading of the selected CommonRoad file.\n\n{}'.format(errorMsg), QMessageBox.Ok)
            return

        self.openScenario(scenario)

    def openScenario(self, scenario):
        self.current_scenario = scenario
        self.update_plot()

    @pyqtSlot()
    def onClickLanelet(self):
        self.dynamic.clear_axes()

        selectedLanelets = self.laneletsList.selectedItems()

        if not selectedLanelets:
            self.selected_lanelet_id = None
            return

        self.selected_lanelet_id = int(selectedLanelets[0].text())
        self.update_plot()

    def update_plot(self):

        if self.current_scenario is None:
            self.dynamic.clear_axes()
            return

        try:
            selected_lanelet = self.current_scenario.lanelet_network.find_lanelet_by_id(self.selected_lanelet_id)

        except ScenarioError:
            selected_lanelet = None

        ax = self.dynamic.get_axes()

        xlim1 = float('Inf')
        xlim2 = -float('Inf')

        ylim1 = float('Inf')
        ylim2 = -float('Inf')

        for lanelet in self.current_scenario.lanelet_network.lanelets:

            # Selected lanelet
            if selected_lanelet is None:
                color = 'gray'
                alpha = 0.3
                zorder = 0
                label = None
            elif lanelet.lanelet_id == selected_lanelet.lanelet_id:
                color = 'red'
                alpha = 0.7
                zorder = 10
                label = '{} selected'.format(lanelet.lanelet_id)
            elif lanelet.lanelet_id in selected_lanelet.predecessor:
                color = 'blue'
                alpha = 0.5
                zorder = 5
                label = '{} predecessor of {}'.format(lanelet.lanelet_id, selected_lanelet.lanelet_id)
            elif lanelet.lanelet_id in selected_lanelet.successor:
                color = 'green'
                alpha = 0.5
                zorder = 5
                label = '{} successor of {}'.format(lanelet.lanelet_id, selected_lanelet.lanelet_id)
            else:
                color = 'gray'
                alpha = 0.3
                zorder = 0
                label = None


            verts = []
            codes = [Path.MOVETO]

            # TODO efficiency
            for x, y in np.vstack([lanelet.left_vertices, lanelet.right_vertices[::-1]]):
                verts.append([x, y])
                codes.append(Path.LINETO)

                xlim1 = min(xlim1, x)
                xlim2 = max(xlim2, x)

                ylim1 = min(ylim1, y)
                ylim2 = max(ylim2, y)

            verts.append(verts[0])
            codes[-1] = Path.CLOSEPOLY

            path = Path(verts, codes)

            ax.add_patch(PathPatch(path, facecolor=color, edgecolor="black", lw=0.0, alpha=alpha, zorder=zorder, label=label))

            ax.plot([x for x, y in lanelet.left_vertices], [y for x, y in lanelet.left_vertices], color="black", lw=0.1)
            ax.plot([x for x, y in lanelet.right_vertices], [y for x, y in lanelet.right_vertices], color="black", lw=0.1)

        handles, labels = self.dynamic.get_axes().get_legend_handles_labels()
        self.dynamic.get_axes().legend(handles, labels)

        self.dynamic.get_axes().set_xlim([xlim1, xlim2])
        self.dynamic.get_axes().set_ylim([ylim1, ylim2])

        self.dynamic.update_plot()


        self.laneletsList.setRowCount(len(self.current_scenario.lanelet_network.lanelets))
        self.laneletsList.setColumnCount(2)

        for idx, lanelet in enumerate(self.current_scenario.lanelet_network.lanelets):
            self.laneletsList.setItem(idx - 1, 0, QTableWidgetItem("{}".format(lanelet.lanelet_id)))
            self.laneletsList.setItem(idx - 1, 1, QTableWidgetItem("{}".format(lanelet.description)))

    def testCmd(self):
        self.dynamic.fig.savefig("foo.pdf", bbox_inches='tight')



if __name__ == '__main__':
    # Make it possible to exit application with ctrl+c on console
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Startup application
    app = QApplication(sys.argv)

    if len(sys.argv) >= 2:
        ex = MainWindow(path=sys.argv[1])
    else:
        ex = MainWindow()

    sys.exit(app.exec_())
