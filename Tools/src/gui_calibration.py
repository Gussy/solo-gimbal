from PySide.QtCore import Slot
from qtasync import AsyncTask, coroutine
import setup_validate, setup_comutation, setup_home
import gui_utils

class calibrationUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

    def setupUI(self):
        self.ui.btnGetCalibration.clicked.connect(self.handleGetCalibration)
        self.ui.btnEraseCalibration.clicked.connect(self.handleEraseCalibration)
        self.ui.btnRunStaticCalibration.clicked.connect(self.handleRunStaticCalibration)

    @Slot()
    def handleGetCalibration(self):
        if self.connection.isConnected():
            self.runAsyncGetCalibration()

    @Slot()
    def handleEraseCalibration(self):
        if self.connection.isConnected():
            self.runAsyncEraseCalibration()

    @Slot()
    def handleRunStaticCalibration(self):
        if self.connection.isConnected():
            self.resetCalibrationTable()
            self.runAsyncStaticCalibration()

    @gui_utils.waitCursor
    def getAllParams(self):
        return setup_validate.show(self.connection.getLink())

    @gui_utils.waitCursor
    def eraseCalibration(self):
        return setup_comutation.resetCalibration(self.connection.getLink())

    @gui_utils.waitCursor
    def staticCalibration(self):
        joints = setup_home.calibrate_joints(self.connection.getLink())
        gyros = setup_home.calibrate_gyro(self.connection.getLink())
        return joints, gyros

    @coroutine
    def runAsyncGetCalibration(self):
        self.enableCalibrationButtons(False)
        allParams = yield AsyncTask(self.getAllParams)
        self.updateCalibrationTable(allParams)
        self.enableCalibrationButtons(True)

    @coroutine
    def runAsyncEraseCalibration(self):
        self.enableCalibrationButtons(False)
        result = yield AsyncTask(self.eraseCalibration)
        self.resetCalibrationTable()
        self.enableCalibrationButtons(True)

    @coroutine
    def runAsyncStaticCalibration(self):
        self.enableCalibrationButtons(False)
        joints, gyros = yield AsyncTask(self.staticCalibration)
        if joints == None:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, False)
        if gyros == None:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, False)
        self.enableCalibrationButtons(True)

    def enableCalibrationButtons(self, enabled):
        self.ui.btnGetCalibration.setEnabled(enabled)
        self.ui.btnRunMotorCalibration.setEnabled(enabled)
        self.ui.btnRunStaticCalibration.setEnabled(enabled)
        self.ui.btnEraseCalibration.setEnabled(enabled)

    def isCalibrated(self, params):
        if 'icept' in params.keys():
            if params['icept'] == 0 or params['slope'] == 0:
                return None
            return True
        elif 'x' in params.keys():
            if params['x'] == 0 or params['y'] == 0 or params['z'] == 0:
                return None
            return True
        return None

    def setCalibrationStatusLabel(self, uiLabel, isCalibrated):
        if isCalibrated == True:
            uiLabel.setText("")
            uiLabel.setStyleSheet("color: rgb(0, 0, 0);\n"
                "background-color: rgb(255, 255, 255);")
        elif isCalibrated == False:
            uiLabel.setText("Failed")
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")
        elif isCalibrated == None:
            uiLabel.setText("Missing")
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")

    def updateCalibrationTable(self, params):
        # Axis statuses
        self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, self.isCalibrated(params['pitch']))
        self.setCalibrationStatusLabel(self.ui.lblCalibrationRollStatus, self.isCalibrated(params['roll']))
        self.setCalibrationStatusLabel(self.ui.lblCalibrationYawStatus, self.isCalibrated(params['yaw']))

        # Static statuses
        self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, self.isCalibrated(params['joint']))
        self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, self.isCalibrated(params['gyro']))
        self.setCalibrationStatusLabel(self.ui.lblCalibrationAccelStatus, self.isCalibrated(params['accel']))

        # Pitch values
        self.ui.lblCalibrationPitchIntercept.setText(str(params['pitch']['icept']))
        self.ui.lblCalibrationPitchSlope.setText(str(params['pitch']['slope']))

        # Roll values
        self.ui.lblCalibrationRollIntercept.setText(str(params['roll']['icept']))
        self.ui.lblCalibrationRollSlope.setText(str(params['roll']['slope']))

        # Yaw values
        self.ui.lblCalibrationYawIntercept.setText(str(params['yaw']['icept']))
        self.ui.lblCalibrationYawSlope.setText(str(params['yaw']['slope']))

        # Joint values
        self.ui.lblCalibrationJointX.setText(str(params['joint']['x']))
        self.ui.lblCalibrationJointY.setText(str(params['joint']['y']))
        self.ui.lblCalibrationJointZ.setText(str(params['joint']['z']))

        # Gyro values
        self.ui.lblCalibrationGyroX.setText(str(params['gyro']['x']))
        self.ui.lblCalibrationGyroY.setText(str(params['gyro']['y']))
        self.ui.lblCalibrationGyroZ.setText(str(params['gyro']['z']))

        # Accel values
        self.ui.lblCalibrationAccelX.setText(str(params['accel']['x']))
        self.ui.lblCalibrationAccelY.setText(str(params['accel']['y']))
        self.ui.lblCalibrationAccelZ.setText(str(params['accel']['z']))

    def resetCalibrationTable(self):
        # Axis statuses
        self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, True)
        self.setCalibrationStatusLabel(self.ui.lblCalibrationRollStatus, True)
        self.setCalibrationStatusLabel(self.ui.lblCalibrationYawStatus, True)

        # Static statuses
        self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, True)
        self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, True)
        self.setCalibrationStatusLabel(self.ui.lblCalibrationAccelStatus, True)

        # Pitch values
        self.ui.lblCalibrationPitchIntercept.setText("")
        self.ui.lblCalibrationPitchSlope.setText("")

        # Roll values
        self.ui.lblCalibrationRollIntercept.setText("")
        self.ui.lblCalibrationRollSlope.setText("")

        # Yaw values
        self.ui.lblCalibrationYawIntercept.setText("")
        self.ui.lblCalibrationYawSlope.setText("")

        # Joint values
        self.ui.lblCalibrationJointX.setText("")
        self.ui.lblCalibrationJointY.setText("")
        self.ui.lblCalibrationJointZ.setText("")

        # Gyro values
        self.ui.lblCalibrationGyroX.setText("")
        self.ui.lblCalibrationGyroY.setText("")
        self.ui.lblCalibrationGyroZ.setText("")

        # Accel values
        self.ui.lblCalibrationAccelX.setText("")
        self.ui.lblCalibrationAccelY.setText("")
        self.ui.lblCalibrationAccelZ.setText("")