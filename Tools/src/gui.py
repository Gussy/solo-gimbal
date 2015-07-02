import sys, argparse
from PySide import QtCore, QtGui
from PySide.QtCore import QThread, Slot
from qtasync import AsyncTask, coroutine
import gui_ui, gui_utils, gui_connection, gui_validation, gui_firmware, gui_calibration, gui_tests
from gui_utils import waitCursor

class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = gui_ui.Ui_MainWindow()
        self.ui.setupUi(self)

        # Public
        self.autoUpdate = False
        self.autoMotorCal = False

        # Disable the tabwidget
        self.ui.tabWidget.setEnabled(False)

        # Setup connection UI
        self.connectionUI = gui_connection.connectionUI(self)

        # Setup validation UI
        self.validationUI = gui_validation.validationUI(self)

        # Setup firmware UI
        self.firmwareUI = gui_firmware.firmwareUI(self)

        # Setup calibration UI
        self.calibrationUI = gui_calibration.calibrationUI(self)

        # Setup tests UI
        self.testsUI = gui_tests.testsUI(self)

    def resetUI(self, isCycling):
        self.validationUI.clearValidationResults()
        self.firmwareUI.resetDidBootload()
        if not isCycling:
            self.calibrationUI.resetCalibrationAttempted()
            self.calibrationUI.resetCalibrationTable()

    def setFirmwareFile(self, filename):
        self.firmwareUI.loadFirmwareFile(filename)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("file",  nargs='?', help="parameter or firmware file to be loaded into the gimbal", default=None)
    parser.add_argument("--autoupdate", help="Automatically bootload firmware onto the gimbal", action='store_true')
    parser.add_argument("--automotorcal", help="Automatically run motor commutation the gimbal", action='store_true')
    args = parser.parse_args()

    app = QtGui.QApplication(sys.argv)
    gui = ControlMainWindow()
    gui.show()
    
    if args.file:
        gui.setFirmwareFile(args.file)

    if args.autoupdate:
        gui.autoUpdate = True

    if args.automotorcal:
        gui.autoMotorCal = True

    sys.exit(app.exec_())
