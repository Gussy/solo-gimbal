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
        self.faults = 0
        self.logMessages = list()

        self.graph = None
        self.pitchGraph = None
        self.rollGraph = None
        self.yawGraph = None
        self.pitchValues = list()
        self.rollValues = list()
        self.yawValues = list()

        self.ui.btnTestsRun.clicked.connect(self.handleTestsRun)
        self.ui.btnTestsAlign.clicked.connect(self.handleTestsAlign)
        self.ui.btnTestsWobble.clicked.connect(self.handleTestsWobble)
        self.ui.btnTestsLifeTest.clicked.connect(self.handleTestsLifeTest)
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
    def handleTestsLifeTest(self):
        if self.connection.isConnected():
            self.runAsyncTestsLifeTest()

    @Slot()
    def handleTestsStop(self):
        if self.connection.isConnected():
            self.stopTests = True
            self.ui.btnTestsStop.setEnabled(False)

    @coroutine
    def runAsyncTestsRun(self):
        self.testStart()
        result = yield AsyncTask(self.testsRun)
        self.testStop()

    @coroutine
    def runAsyncTestsWobble(self):
        self.testStart(delay=setup_run.WOBBLE_TEST_ALIGNMENT_TIME)

        # self.graph = gui_graph.GraphWindow()
        # self.pitchGraph = self.graph.newGraph('Pitch gyro')
        # self.rollGraph = self.graph.newGraph('Roll gyro')
        # self.yawGraph = self.graph.newGraph('Yaw gyro')

        result = yield AsyncTask(self.testsWobble)

        if result != True and isinstance(result, str):
            self.ui.lblTestsLogfile.setText(result)

        # self.graph = None

        self.testStop()

    @coroutine
    def runAsyncTestsLifeTest(self):
        self.testStart(delay=setup_run.WOBBLE_TEST_ALIGNMENT_TIME)
        result = yield AsyncTask(self.testsLifeTest)
        if result != True and isinstance(result, str):
            self.ui.lblTestsLogfile.setText(result)
        self.testStop()

    @coroutine
    def runAsyncTestsAlign(self):
        self.testStart()
        result = yield AsyncTask(self.testsAlign)
        self.testStop()

    def stopTestsCallback(self):
        return self.stopTests

    def testEventCallback(self, msg, fault=False):
        if fault:
            self.faults += 1
        elapsedTime = int(time.time() - self.startTime)
        message = "%is - %s" % (elapsedTime, msg)
        self.logMessages.append(message)

    def reportCallback(self, a, b, c):
        now = time.time()
        self.pitchValues.append((now, a))
        self.rollValues.append((now, b))
        self.yawValues.append((now, c))

    def testsRun(self):
        return setup_run.runTest(
            self.connection.getLink(),
            'run',
            stopTestsCallback=self.stopTestsCallback,
            eventCallback=self.testEventCallback,
            timeout=self.getTestTimeout()
        )

    def testsAlign(self):
        return setup_run.runTest(
            self.connection.getLink(),
            'align',
            stopTestsCallback=self.stopTestsCallback,
            eventCallback=self.testEventCallback,
            timeout=self.getTestTimeout()
        )

    def testsWobble(self):
        return setup_run.runTestLoop(
            self.connection.getLink(),
            'wobble',
            stopTestsCallback=self.stopTestsCallback,
            eventCallback=self.testEventCallback,
            reportCallback=self.reportCallback,
            timeout=self.getTestTimeout(),
            wobbleport=self.parent.wobblePort
        )

    def testsLifeTest(self):
        return setup_run.runLifeTest(
            self.connection.getLink(),
            stopTestsCallback=self.stopTestsCallback,
            eventCallback=self.testEventCallback,
            wobbleport=self.parent.wobblePort
        )

    def enableUI(self, enabled):
        self.ui.btnTestsRun.setEnabled(enabled)
        self.ui.btnTestsAlign.setEnabled(enabled)
        self.ui.btnTestsWobble.setEnabled(enabled)
        self.ui.btnTestsLifeTest.setEnabled(enabled)
        self.ui.btnTestsStop.setEnabled(not enabled)
        self.ui.tabValidate.setEnabled(enabled)
        self.ui.tabFirmware.setEnabled(enabled)
        self.ui.tabCalibration.setEnabled(enabled)

    def testStart(self, interval=100, delay=0):
        self.enableUI(False)
        self.stopTests = False
        self.pitchValues = list()
        self.rollValues = list()
        self.yawValues = list()
        self.faults = 0
        self.ui.lblTestsRunTime.setText("0:00:00")
        self.ui.lblTestsFaults.setText("0")
        self.ui.txtTestsLog.setPlainText("")
        self.startTime = time.time() + delay
        self.timer.start(interval)

    def testStop(self):
        self.timerUpdate()
        self.timer.stop()
        self.stopTests = True
        self.enableUI(True)

    def timerUpdate(self):
        time_delta = time.time() - self.startTime
        m, s = divmod(time_delta, 60)
        h, m = divmod(m, 60)
        if time_delta < 0:
            h, m, s = 0, 0, abs(time_delta)
        self.ui.lblTestsRunTime.setText("%d:%02d:%02d" % (h, m, s))
        for i in range(len(self.logMessages)):
            self.ui.txtTestsLog.appendPlainText(self.logMessages.pop(0))
        self.ui.lblTestsFaults.setText(str(self.faults))
        self.dequeueValues(self.pitchValues, self.pitchGraph)
        self.dequeueValues(self.rollValues, self.rollGraph)
        self.dequeueValues(self.yawValues, self.yawGraph)

    def dequeueValues(self, values, graph):
        for i in range(len(values)):
            t, v = values.pop(0)
            if self.graph:
                self.graph.updateGraph(graph, t, v)

    def getTestTimeout(self):
        timeout = self.ui.sbxTestsTimeout.value()
        if timeout == 0:
            return None
        return timeout
