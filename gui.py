
import os
import signal
import sys

from lxml import etree

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QWidget, QLineEdit, QFileDialog
from PyQt5.QtWidgets import QPushButton, QMessageBox, QLabel

from opendriveparser import parse_opendrive
from opendrive2lanelet import Network

class MainWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.loadedRoadNetwork = None

        self._initUserInterface()

    def _initUserInterface(self):

        self.setWindowTitle("OpenDRIVE 2 Lanelets Converter")

        self.setFixedSize(560, 345)

        self.loadButton = QPushButton('Load OpenDRIVE', self)
        self.loadButton.setToolTip('Load a OpenDRIVE scenario within a *.xodr file')
        self.loadButton.move(10, 10)
        self.loadButton.resize(130, 35)
        self.loadButton.clicked.connect(self.openOpenDriveFile)

        self.inputOpenDriveFile = QLineEdit(self)
        self.inputOpenDriveFile.move(150, 10)
        self.inputOpenDriveFile.resize(400, 35)
        self.inputOpenDriveFile.setReadOnly(True)

        self.statsText = QLabel(self)
        self.statsText.move(10, 55)
        self.statsText.resize(540, 235)
        self.statsText.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.statsText.setTextFormat(Qt.RichText)

        self.exportCommonRoadButton = QPushButton('Export as CommonRoad', self)
        self.exportCommonRoadButton.move(10, 300)
        self.exportCommonRoadButton.resize(170, 35)
        self.exportCommonRoadButton.setDisabled(True)
        self.exportCommonRoadButton.clicked.connect(self.exportAsCommonRoad)

        self.viewOutputButton = QPushButton('View Road Network', self)
        self.viewOutputButton.move(190, 300)
        self.viewOutputButton.resize(170, 35)
        self.viewOutputButton.setDisabled(True)
        self.viewOutputButton.clicked.connect(self.viewLaneletNetwork)

        self.show()

    def resetOutputElements(self):
        self.exportCommonRoadButton.setDisabled(True)
        self.viewOutputButton.setDisabled(True)

    def openOpenDriveFile(self):
        self.resetOutputElements()

        path, _ = QFileDialog.getOpenFileName(
            self,
            "QFileDialog.getOpenFileName()",
            "",
            "OpenDRIVE files *.xodr (*.xodr)",
            options=QFileDialog.Options()
        )

        if not path:
            return

        filename = os.path.basename(path)
        self.inputOpenDriveFile.setText(filename)

        # Load road network and print some statistics
        try:
            fh = open(path, 'r')
            openDriveXml = parse_opendrive(etree.parse(fh).getroot())
            fh.close()
        except (etree.XMLSyntaxError) as e:
            errorMsg = 'XML Syntax Error: {}'.format(e)
            QMessageBox.warning(self, 'OpenDRIVE error', 'There was an error during the loading of the selected OpenDRIVE file.\n\n{}'.format(errorMsg), QMessageBox.Ok)
            return
        except (TypeError, AttributeError, ValueError) as e:
            errorMsg = 'Value Error: {}'.format(e)
            QMessageBox.warning(self, 'OpenDRIVE error', 'There was an error during the loading of the selected OpenDRIVE file.\n\n{}'.format(errorMsg), QMessageBox.Ok)
            return

        self.loadedRoadNetwork = Network()
        self.loadedRoadNetwork.loadOpenDrive(openDriveXml)

        self.statsText.setText("Name: {}<br>Version: {}<br>Date: {}<br><br>OpenDRIVE Version {}.{}<br><br>Number of roads: {}<br>Total length of road network: {:.2f} meters".format(
            openDriveXml.header.name if openDriveXml.header.name else "<i>unset</i>",
            openDriveXml.header.version,
            openDriveXml.header.date,
            openDriveXml.header.revMajor,
            openDriveXml.header.revMinor,
            len(openDriveXml.roads),
            sum([road.length for road in openDriveXml.roads])
        ))

        self.exportCommonRoadButton.setDisabled(False)
        self.viewOutputButton.setDisabled(False)

    def exportAsCommonRoad(self):

        if not self.loadedRoadNetwork:
            return

        path, _ = QFileDialog.getSaveFileName(
            self,
            "QFileDialog.getSaveFileName()",
            "",
            "CommonRoad files *.xml (*.xml)",
            options=QFileDialog.Options()
        )

        if not path:
            return

        try:
            fh = open(path, "wb")
            fh.write(self.loadedRoadNetwork.exportCommonRoadScenario().export_to_string())
            fh.close()
        except (IOError) as e:
            QMessageBox.critical(self, 'CommonRoad file not created!', 'The CommonRoad file was not exported due to an error.\n\n{}'.format(e), QMessageBox.Ok)
            return

        QMessageBox.information(self, 'CommonRoad file created!', 'The CommonRoad file was successfully exported.', QMessageBox.Ok)

    def viewLaneletNetwork(self):
        import matplotlib.pyplot as plt
        from fvks.visualization.draw_dispatch import draw_object
        from fvks.scenario.lanelet import Lanelet as FvksLanelet

        def convert_to_fvks_lanelet(ll):
            return FvksLanelet(
                left_vertices=ll.left_vertices,
                center_vertices=ll.center_vertices,
                right_vertices=ll.right_vertices,
                lanelet_id=ll.lanelet_id
            )

        scenario = self.loadedRoadNetwork.exportCommonRoadScenario(filterTypes=[
            'driving',
            'onRamp',
            'offRamp',
            'stop',
            'parking',
            'special1',
            'special2',
            'special3',
            'entry',
            'exit',
        ])

        # Visualization
        fig = plt.figure()
        ax = fig.add_subplot(111)

        for lanelet in scenario.lanelet_network.lanelets:
            draw_object(convert_to_fvks_lanelet(lanelet), ax=ax)

        ax.set_aspect('equal', 'datalim')
        plt.axis('off')

        plt.show()


if __name__ == '__main__':
    # Make it possible to exit application with ctrl+c on console
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Startup application
    app = QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())
