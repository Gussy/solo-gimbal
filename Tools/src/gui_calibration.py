from PySide.QtCore import Slot
from qtasync import AsyncTask, coroutine
import setup_mavlink, setup_validate, setup_comutation, setup_home
import gui_utils

class calibrationUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

        self.ui.btnGetCalibration.clicked.connect(self.handleGetCalibration)
        self.ui.btnEraseCalibration.clicked.connect(self.handleEraseCalibration)
        self.ui.btnRunStaticCalibration.clicked.connect(self.handleRunStaticCalibration)
        self.ui.btnRunMotorCalibration.clicked.connect(self.handleRunMotorCalibration)

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

    @Slot()
    def handleRunMotorCalibration(self):
        if self.connection.isConnected():
            self.resetCalibrationTable()
            self.runAsyncMotorCalibration()

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

    @gui_utils.waitCursor
    def runMotorCalibration(self):
        def calibrationProgressCallback(axis, progress, status):
            self.ui.pbarCalibration.setValue(progress)
            self.setCalibrationStatus(axis)
        return setup_comutation.calibrate(self.connection.getLink(), calibrationProgressCallback)

    @gui_utils.waitCursor
    def resetGimbal(self):
        return setup_mavlink.reset_gimbal(self.connection.getLink())

    @coroutine
    def runAsyncGetCalibration(self):
        self.setButtonsEnabled(False)
        allParams = yield AsyncTask(self.getAllParams)
        self.updateCalibrationTable(allParams)
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncEraseCalibration(self):
        self.setButtonsEnabled(False)
        result = yield AsyncTask(self.eraseCalibration)
        self.resetCalibrationTable()
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncStaticCalibration(self):
        self.setButtonsEnabled(False)
        joints, gyros = yield AsyncTask(self.staticCalibration)
        if joints == None:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, False)
        if gyros == None:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, False)
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncMotorCalibration(self):
        self.setButtonsEnabled(False)
        self.ui.pbarCalibration.setValue(0)

        result = yield AsyncTask(self.runMotorCalibration)

        if result == setup_comutation.Results.ParamFetchFailed:
            self.setCalibrationStatus("Failed to get calibration parameters from the gimbal")
        elif result == setup_comutation.Results.CommsFailed:
            self.setCalibrationStatus("Gimbal failed to communicate calibration progress")
        else:
            # These results require a reset of the Gimbal
            if result == setup_comutation.Results.Success:
                self.setCalibrationStatus("Calibration successful!")
                # Get all the parameters if the calibration was successful
                allParams = yield AsyncTask(self.getAllParams)
                self.updateCalibrationTable(allParams)
            elif result == setup_comutation.Results.CalibrationExists:
                self.setCalibrationStatus("A calibration already exists, erase current calibration first (-e)")
            elif result == setup_comutation.Results.PitchFailed:
                self.setCalibrationStatus("Pitch calibration failed")
                self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, False)
            elif result == setup_comutation.Results.RollFailed:
                self.setCalibrationStatus("Roll calibration failed")
                self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, True)
                self.setCalibrationStatusLabel(self.ui.lblCalibrationRollStatus, False)
            elif result == setup_comutation.Results.YawFailed:
                self.setCalibrationStatus("Yaw calibration failed")
                self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, True)
                self.setCalibrationStatusLabel(self.ui.lblCalibrationRollStatus, True)
                self.setCalibrationStatusLabel(self.ui.lblCalibrationYawStatus, False)

            # Reset the gimbal
            self.setCalibrationStatus("Rebooting Gimbal")
            result = yield AsyncTask(self.resetGimbal)

        self.setCalibrationStatus()
        self.ui.pbarCalibration.setValue(0)
        self.setButtonsEnabled(True)

    def setCalibrationStatus(self, msg=''):
        self.ui.lblCalibrationStatus.setText(msg)

    def setButtonsEnabled(self, enabled):
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
            uiLabel.setText("Calibrated")
            uiLabel.setStyleSheet("color: rgb(0, 0, 0);\n"
                "background-color: rgb(0, 255, 0);")
        elif isCalibrated == False:
            uiLabel.setText("Failed")
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")
        elif isCalibrated == None:
            uiLabel.setText("Missing")
            uiLabel.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);")
        else:
            uiLabel.setText("")
            uiLabel.setStyleSheet("color: rgb(0, 0, 0);\n"
                "background-color: rgb(255, 255, 255);")

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
        self.setCalibrationStatusLabel(self.ui.lblCalibrationPitchStatus, 'Blank')
        self.setCalibrationStatusLabel(self.ui.lblCalibrationRollStatus, 'Blank')
        self.setCalibrationStatusLabel(self.ui.lblCalibrationYawStatus, 'Blank')

        # Static statuses
        self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, 'Blank')
        self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, 'Blank')
        self.setCalibrationStatusLabel(self.ui.lblCalibrationAccelStatus, 'Blank')

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

        # Calibration status
        self.setCalibrationStatus()
