from PySide.QtCore import Slot, QTimer
from qtasync import AsyncTask, coroutine
import setup_mavlink, setup_validate, setup_comutation, setup_home
import gui_utils

class calibrationUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

        # Public
        self.calibrationAttempted = False

        # Private
        self.progress = -1
        self.status = ''

        self.ui.btnGetCalibration.clicked.connect(self.handleGetCalibration)
        self.ui.btnEraseCalibration.clicked.connect(self.handleEraseCalibration)
        self.ui.btnRunJointCalibration.clicked.connect(self.handleRunJointCalibration)
        self.ui.btnRunGyroCalibration.clicked.connect(self.handleRunGyroCalibration)
        self.ui.btnRunMotorCalibration.clicked.connect(self.handleRunMotorCalibration)

        self.timer = QTimer()
        self.timer.timeout.connect(self.timerUpdate)

    @Slot()
    def handleGetCalibration(self):
        if self.connection.isConnected():
            self.runAsyncGetCalibration()

    @Slot()
    def handleEraseCalibration(self):
        if self.connection.isConnected():
            self.runAsyncEraseCalibration()

    @Slot()
    def handleRunJointCalibration(self):
        if self.connection.isConnected():
            self.resetCalibrationTable()
            self.runAsyncJointCalibration()

    @Slot()
    def handleRunGyroCalibration(self):
        if self.connection.isConnected():
            self.resetCalibrationTable()
            self.runAsyncGyroCalibration()

    @Slot()
    def handleRunMotorCalibration(self):
        if self.connection.isConnected():
            self.resetCalibrationTable()
            self.runAsyncMotorCalibration()

    def getCalibrationAttempted(self):
        return self.calibrationAttempted

    def resetCalibrationAttempted(self):
        self.calibrationAttempted = False

    @gui_utils.waitCursor
    def getAllParams(self):
        return setup_validate.show(self.connection.getLink())

    @gui_utils.waitCursor
    def eraseCalibration(self):
        return setup_comutation.resetCalibration(self.connection.getLink())

    @gui_utils.waitCursor
    def jointCalibration(self):
        return setup_home.calibrate_joints(self.connection.getLink())

    @gui_utils.waitCursor
    def gyroCalibration(self):
        return setup_home.calibrate_gyro(self.connection.getLink())

    @gui_utils.waitCursor
    def runMotorCalibration(self):
        result = None
        try:
            def calibrationProgressCallback(axis, progress, status):
                self.progress = progress
                self.status = "Calibrating %s" % axis.title()
            result = setup_comutation.calibrate(self.connection.getLink(), calibrationProgressCallback)
        except Exception as e:
            print e
        finally:
            return result

    @gui_utils.waitCursor
    def resetGimbal(self):
        return setup_mavlink.reset_gimbal(self.connection.getLink())

    @coroutine
    def runAsyncGetCalibration(self):
        self.setButtonsEnabled(False)
        allParams = yield AsyncTask(self.getAllParams)
        if allParams != None:
            self.updateCalibrationTable(allParams)
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncGetCalibration(self):
        self.setButtonsEnabled(False)
        allParams = yield AsyncTask(self.getAllParams)
        if allParams != None:
            self.updateCalibrationTable(allParams)
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncEraseCalibration(self):
        self.setButtonsEnabled(False)
        result = yield AsyncTask(self.eraseCalibration)
        self.resetCalibrationTable()
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncJointCalibration(self):
        self.setButtonsEnabled(False)

        joints = yield AsyncTask(self.jointCalibration)
        
        # Show joints results
        if joints:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, True)
            self.ui.lblCalibrationJointX.setText('%0.6f' % joints.x)
            self.ui.lblCalibrationJointY.setText('%0.6f' % joints.y)
            self.ui.lblCalibrationJointZ.setText('%0.6f' % joints.z)
        else:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationJointStatus, False)
        
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncGyroCalibration(self):
        self.setButtonsEnabled(False)
        
        gyros = yield AsyncTask(self.gyroCalibration)

        # Show joints results
        if gyros:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, True)
            self.ui.lblCalibrationGyroX.setText('%0.6f' % gyros.x)
            self.ui.lblCalibrationGyroY.setText('%0.6f' % gyros.y)
            self.ui.lblCalibrationGyroZ.setText('%0.6f' % gyros.z)
        else:
            self.setCalibrationStatusLabel(self.ui.lblCalibrationGyroStatus, False)
        self.setButtonsEnabled(True)

    @coroutine
    def runAsyncMotorCalibration(self):
        self.setButtonsEnabled(False)
        self.calibrationAttempted = True

        # Run calibration
        self.timerStart()
        result = yield AsyncTask(self.runMotorCalibration)

        if result == setup_comutation.Results.ParamFetchFailed:
            self.setCalibrationStatus("Failed to get calibration parameters from the gimbal")
        elif result == setup_comutation.Results.CommsFailed:
            self.setCalibrationStatus("Gimbal failed to communicate calibration progress")
        elif result == setup_comutation.Results.CalibrationExists:
            self.setCalibrationStatus("A calibration already exists, erase current calibration first")
        elif result != None:
            # These results require a reset of the Gimbal
            if result == setup_comutation.Results.Success:
                pass
            # Below assumes that the calibration runs in the order: pitch->roll->yaw
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
            reset = yield AsyncTask(self.resetGimbal)

            if result == setup_comutation.Results.Success:
                self.setCalibrationStatus("Calibration successful!")
                # Get all the parameters if the calibration was successful
                allParams = yield AsyncTask(self.getAllParams)
                self.updateCalibrationTable(allParams)

        self.timerStop()
        self.setButtonsEnabled(True)

    def setCalibrationStatus(self, msg=''):
        self.ui.lblCalibrationStatus.setText(msg)

    def setButtonsEnabled(self, enabled):
        self.ui.btnGetCalibration.setEnabled(enabled)
        self.ui.btnRunMotorCalibration.setEnabled(enabled)
        self.ui.btnRunJointCalibration.setEnabled(enabled)
        self.ui.btnRunGyroCalibration.setEnabled(enabled)
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
        self.ui.lblCalibrationPitchIntercept.setText('%0.6f' % params['pitch']['icept'])
        self.ui.lblCalibrationPitchSlope.setText('%0.6f' % params['pitch']['slope'])

        # Roll values
        self.ui.lblCalibrationRollIntercept.setText('%0.6f' % params['roll']['icept'])
        self.ui.lblCalibrationRollSlope.setText('%0.6f' % params['roll']['slope'])

        # Yaw values
        self.ui.lblCalibrationYawIntercept.setText('%0.6f' % params['yaw']['icept'])
        self.ui.lblCalibrationYawSlope.setText('%0.6f' % params['yaw']['slope'])

        # Joint values
        self.ui.lblCalibrationJointX.setText('%0.6f' % params['joint']['x'])
        self.ui.lblCalibrationJointY.setText('%0.6f' % params['joint']['y'])
        self.ui.lblCalibrationJointZ.setText('%0.6f' % params['joint']['z'])

        # Gyro values
        self.ui.lblCalibrationGyroX.setText('%0.6f' % params['gyro']['x'])
        self.ui.lblCalibrationGyroY.setText('%0.6f' % params['gyro']['y'])
        self.ui.lblCalibrationGyroZ.setText('%0.6f' % params['gyro']['z'])

        # Accel values
        self.ui.lblCalibrationAccelX.setText('%0.6f' % params['accel']['x'])
        self.ui.lblCalibrationAccelY.setText('%0.6f' % params['accel']['y'])
        self.ui.lblCalibrationAccelZ.setText('%0.6f' % params['accel']['z'])

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

    def timerStart(self, interval=100):
        self.ui.pbarCalibration.setValue(0)
        self.progress = -1
        self.status = ''
        self.timer.start()

    def timerStop(self):
        self.ui.pbarCalibration.setValue(0)
        self.timer.stop()

    def timerUpdate(self):
        self.ui.pbarCalibration.setValue(self.progress)
        self.setCalibrationStatus(self.status)
