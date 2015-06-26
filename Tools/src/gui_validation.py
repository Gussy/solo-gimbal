from PySide.QtCore import Slot
from qtasync import AsyncTask, coroutine
import setup_validate
import gui_utils

class validationUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

    def setupUI(self):
        self.ui.btnValidationRunTests.clicked.connect(self.handleRunValidation)
        self.ui.btnValidationSetDefaults.clicked.connect(self.handleSetDefaults)

    @Slot()
    def handleRunValidation(self):
        if self.connection.isConnected():
            self.runAsyncValidation()

    @Slot()
    def handleSetDefaults(self):
        if self.connection.isConnected():
            self.runAsyncSetDefaults()

    @gui_utils.waitCursor
    def validateVersion(self):
        return setup_validate.validate_version(self.connection.getLink())

    @gui_utils.waitCursor
    def validateCommutation(self):
        return setup_validate.validate_comutation(self.connection.getLink())
    
    @gui_utils.waitCursor
    def validateJoints(self):
        return setup_validate.validate_joints(self.connection.getLink())
    
    @gui_utils.waitCursor
    def validateGyros(self):
        return setup_validate.validate_gyros(self.connection.getLink())
    
    @gui_utils.waitCursor
    def validateAccelerometers(self):
        return setup_validate.validate_accelerometers(self.connection.getLink())
    
    @gui_utils.waitCursor
    def validateGains(self):
        return setup_validate.validate_gains(self.connection.getLink())

    @gui_utils.waitCursor
    def setDefaults(self):
        return setup_validate.restore_defaults(self.connection.getLink())

    def updateValidation(self, result, uiLabel, failureMessage=""):
        if result == setup_validate.Results.Pass:
            uiLabel.setText("PASS")
            uiLabel.setStyleSheet("color: rgb(0, 0, 0);\n"
                "background-color: rgb(0, 255, 0);")
        elif result == setup_validate.Results.Fail:
            uiLabel.setText("FAIL (%s)" % failureMessage)
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")
        elif result == setup_validate.Results.Error:
            uiLabel.setText("ERROR")
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")
        else:
            uiLabel.setText("")
            uiLabel.setStyleSheet("color: rgb(0, 0, 0);\n"
                "background-color: rgb(255, 255, 255);")

    def clearValidationResults(self):
        self.updateValidation(None, self.ui.lblValidationVersion)
        self.updateValidation(None, self.ui.lblValidationSerialNumber)
        self.updateValidation(None, self.ui.lblValidationAssemblyTime)
        self.updateValidation(None, self.ui.lblValidationCommutation)
        self.updateValidation(None, self.ui.lblValidationJoints)
        self.updateValidation(None, self.ui.lblValidationGyros)
        self.updateValidation(None, self.ui.lblValidationAccelerometer)
        self.updateValidation(None, self.ui.lblValidationGains)

    @coroutine
    def runAsyncValidation(self):
        self.ui.btnValidationRunTests.setEnabled(False)
        self.ui.btnValidationSetDefaults.setEnabled(False)
        self.clearValidationResults()

        valid = yield AsyncTask(self.validateVersion)
        self.updateValidation(valid, self.ui.lblValidationVersion,
            "Update to latest gimbal software")

        valid = yield AsyncTask(self.validateVersion)
        self.updateValidation(valid, self.ui.lblValidationSerialNumber,
            "Serial number not set")

        valid = yield AsyncTask(self.validateVersion)
        self.updateValidation(valid, self.ui.lblValidationAssemblyTime,
            "Assembly time not set")

        valid = yield AsyncTask(self.validateCommutation)
        self.updateValidation(valid, self.ui.lblValidationCommutation,
            "Run comutation calibration")

        valid = yield AsyncTask(self.validateJoints)
        self.updateValidation(valid, self.ui.lblValidationJoints,
            "Run joint calibration")

        valid = yield AsyncTask(self.validateGyros)
        self.updateValidation(valid, self.ui.lblValidationGyros,
            "Run gyro calibration")

        valid = yield AsyncTask(self.validateAccelerometers)
        self.updateValidation(valid, self.ui.lblValidationAccelerometer,
            "Run accelerometer calibration")

        valid = yield AsyncTask(self.validateGains)
        self.updateValidation(valid, self.ui.lblValidationGains,
            "Reset parameters to default values")

        self.ui.btnValidationRunTests.setEnabled(True)
        self.ui.btnValidationSetDefaults.setEnabled(True)

    @coroutine
    def runAsyncSetDefaults(self):
        self.ui.btnValidationRunTests.setEnabled(False)
        self.ui.btnValidationSetDefaults.setEnabled(False)
        self.clearValidationResults()

        result = yield AsyncTask(self.setDefaults)

        self.ui.btnValidationRunTests.setEnabled(True)
        self.ui.btnValidationSetDefaults.setEnabled(True)
