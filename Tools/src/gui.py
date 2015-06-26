import sys
from PySide import QtCore, QtGui
from PySide.QtCore import QThread, Slot
from qtasync import AsyncTask, coroutine
import gui_ui, gui_utils, gui_connection, gui_validation, gui_firmware, gui_calibration
from gui_utils import waitCursor

class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = gui_ui.Ui_MainWindow()
        self.ui.setupUi(self)

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

    def resetUI(self, isCycling):
        self.validationUI.clearValidationResults()
        self.firmwareUI.clearFirmwareInfoUI()
        if not isCycling:
            self.calibrationUI.resetCalibrationTable()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = ControlMainWindow()
    gui.show()
    sys.exit(app.exec_())
