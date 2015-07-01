import time, datetime
from PySide.QtCore import Slot, QTimer
from qtasync import AsyncTask, coroutine
import setup_run
import gui_utils

class testsUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

        # Private
        self.stopTests = False
        self.startTime = 0

        self.ui.btnTestsRun.clicked.connect(self.handleTestsRun)
        self.ui.btnTestsAlign.clicked.connect(self.handleTestsAlign)
        self.ui.btnTestsWobble.clicked.connect(self.handleTestsWobble)
        self.ui.btnTestsStop.clicked.connect(self.handleTestsStop)

        self.timer = QTimer()
        self.timer.timeout.connect(self.timerUpdate)

    @Slot()
    def handleTestsRun(self):
        if self.connection.isConnected():
            self.runAsyncTestsRun()

    @Slot()
    def handleTestsAlign(self):
        if self.connection.isConnected():
            self.runAsyncTestsAlign()

    @Slot()
    def handleTestsWobble(self):
        if self.connection.isConnected():
            self.runAsyncTestsWobble()

    @Slot()
    def handleTestsStop(self):
        if self.connection.isConnected():
            self.stopTests = True
            self.ui.btnTestsRun.setEnabled(True)
            self.ui.btnTestsStop.setEnabled(False)

    @coroutine
    def runAsyncTestsRun(self):
        self.enableUI(False)
        self.stopTests = False
        self.timerStart()
        result = yield AsyncTask(self.testsRun)
        self.timerStop()
        self.stopTests = True
        self.enableUI(True)

    @coroutine
    def runAsyncTestsAlign(self):
        self.enableUI(False)
        self.stopTests = False
        self.timerStart()
        result = yield AsyncTask(self.testsAlign)
        self.timerStop()
        self.stopTests = True
        self.enableUI(True)

    @coroutine
    def handleTestsWobble(self):
        self.enableUI(False)
        self.stopTests = False
        self.timerStart()
        result = yield AsyncTask(self.testsWobble)
        self.timerStop()
        self.stopTests = True
        self.enableUI(True)

    def stopTestsCallback(self):
        return self.stopTests

    def testsRun(self):
        return setup_run.run(self.connection.getLink(), self.stopTestsCallback)

    def testsAlign(self):
        return setup_run.align(self.connection.getLink(), self.stopTestsCallback)

    def testsWobble(self):
        return setup_run.wobble(self.connection.getLink(), self.stopTestsCallback)

    def enableUI(self, enabled):
        self.ui.btnTestsRun.setEnabled(enabled)
        self.ui.btnTestsAlign.setEnabled(enabled)
        self.ui.btnTestsWobble.setEnabled(enabled)

        self.ui.btnTestsStop.setEnabled(not enabled)

        self.ui.tabValidate.setEnabled(enabled)
        self.ui.tabFirmware.setEnabled(enabled)
        self.ui.tabCalibration.setEnabled(enabled)

    def timerStart(self, interval=500):
        self.ui.lblTestsRunTime.setText("0:00:00")
        self.startTime = time.time()
        self.timer.start(interval)

    def timerStop(self):
        self.timer.stop()

    def timerUpdate(self):
        time_delta = time.time() - self.startTime
        m, s = divmod(time_delta, 60)
        h, m = divmod(m, 60)
        self.ui.lblTestsRunTime.setText("%d:%02d:%02d" % (h, m, s))
