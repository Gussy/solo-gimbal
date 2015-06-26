import datetime
from PySide import QtCore, QtGui
from PySide.QtCore import Slot
from qtasync import AsyncTask, coroutine
import setup_mavlink, setup_factory
import gui_utils

class connectionUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent

        # Public
        self.connected = False
        self.link = None

        # Private
        self.mavport = None
        self.connectWithAuto = True
        self.isCycling = False

        self.addBaudrates()
        self.ui.rbAuto.clicked.connect(self.handleConnectionTypeAuto)
        self.ui.rbSerial.clicked.connect(self.handleConnectionTypeSerial)
        self.ui.btnConnect.clicked.connect(self.handleConnectButton)

    @Slot()
    def handleConnectionTypeAuto(self):
        self.connectWithAuto = True
        self.ui.cbSerialPort.setEnabled(False)
        self.ui.cbBaudrate.setEnabled(False)

    @Slot()
    def handleConnectionTypeSerial(self):
        self.connectWithAuto = False
        ports = [str(x).split(':')[0].strip() for x in setup_mavlink.getSerialPorts()]
        self.updateSerialPorts(ports)
        self.ui.cbSerialPort.setEnabled(True)
        self.ui.cbBaudrate.setEnabled(True)

    @Slot()
    def handleConnectButton(self):
        if not self.connected:
            self.setConnectionState(True)
            self.attemptConnection()
        else:
            self.closeConnection()
            self.setConnectionState(False)

    def getLink(self):
        return self.link

    def isConnected(self):
        return self.connected

    def setConnectionStatusBanner(self, state):
        if state == 'disconnected':
            self.ui.lblConnectionStatus.setText("Disconnected")
            self.ui.lblConnectionStatus.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(255, 0, 0);\n"
                "border-radius: 20px;")
        elif state == 'connecting':
            self.ui.lblConnectionStatus.setText("Connecting...")
            self.ui.lblConnectionStatus.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(0, 0, 255);\n"
                "border-radius: 20px;")
        elif state == 'connected':
            self.ui.lblConnectionStatus.setText("Connected")
            self.ui.lblConnectionStatus.setStyleSheet("color: rgb(255, 255, 255);\n"
                "background-color: rgb(0, 255, 0);\n"
                "border-radius: 20px;")

    def setStatusInfo(self, softwareVersion=None, serialNumber=None, assemblyTime=None):
        if softwareVersion != None:
            self.ui.lblSoftwareVersion.setText("v%i.%i.%i" % (softwareVersion[0], softwareVersion[1], softwareVersion[2]))
        elif softwareVersion != '':
            self.ui.lblSoftwareVersion.setText("Unknown")

        if serialNumber != None:
            if serialNumber == '':
                self.ui.lblSerialNumber.setText("Not set")
            else:
                self.ui.lblSerialNumber.setText(str(serialNumber))
        else:
            self.ui.lblSerialNumber.setText("Unknown")

        if assemblyTime != None:
            if assemblyTime == 0:
                self.ui.lblAssemblyTime.setText("Not set")
            else:
                strTime = datetime.datetime.fromtimestamp(assemblyTime).strftime('%d/%m/%Y %H:%M:%S')
                self.ui.lblAssemblyTime.setText(strTime)
        else:
            self.ui.lblAssemblyTime.setText("Unknown")

    def setConnectionState(self, connected):
        if connected:
            self.connected = True
            self.setConnectionStatusBanner('connecting')
            self.ui.cbSerialPort.setEnabled(False)
            self.ui.cbBaudrate.setEnabled(False)
            self.ui.rbAuto.setEnabled(False)
            self.ui.rbSerial.setEnabled(False)
            self.ui.btnConnect.setText("Disconnect")
        else:
            self.connected = False
            self.setConnectionStatusBanner('disconnected')
            self.ui.cbSerialPort.setEnabled(True)
            self.ui.cbBaudrate.setEnabled(True)
            self.ui.rbAuto.setEnabled(True)
            self.ui.rbSerial.setEnabled(True)
            self.ui.btnConnect.setText("Connect")
            self.ui.tabWidget.setEnabled(False)

    def updateSerialPorts(self, ports=None):
        self.ui.cbSerialPort.clear()
        if ports:
            for port in ports:
                self.ui.cbSerialPort.addItem(str(port))

    def addBaudrates(self, bauds=['230400', '115200']):
        self.ui.cbBaudrate.clear()
        if bauds:
            for baud in bauds:
                self.ui.cbBaudrate.addItem(str(baud))

    @gui_utils.waitCursor
    def autoConnectionWorker(self):
        return setup_mavlink.open_comm()

    @gui_utils.waitCursor
    def serialConnectionWorker(self, port, baudrate):
        return setup_mavlink.open_comm(port, baudrate)

    @gui_utils.waitCursor
    def waitForHeartbeatWorker(self):
        return setup_mavlink.wait_for_heartbeat(self.link)

    @gui_utils.waitCursor
    def getGimbalParameters(self):
        version = setup_factory.readSWver(self.link)
        serialNumber = setup_factory.get_serial_number(self.link)
        assemblyTime = setup_factory.get_assembly_time(self.link)
        return version, serialNumber, assemblyTime

    @gui_utils.waitCursor
    def writeSerialNumber(self, serialNumber):
        serialNumber = setup_factory.set_serial_number(self.link, serialNumber)
        assemblyTime = setup_factory.set_assembly_date(self.link)
        return serialNumber, assemblyTime

    @coroutine
    def attemptConnection(self):
        self.ui.btnConnect.setEnabled(False)

        # Open the mavlink connection
        if self.connectWithAuto:
            self.mavport, self.link = yield AsyncTask(self.autoConnectionWorker)
        else:
            # Get the current serial port if connecting with serial
            port = gui_utils.getComboboxSelection(self.ui.cbSerialPort)
            baudrate = gui_utils.getComboboxSelection(self.ui.cbBaudrate)
            self.mavport, self.link = yield AsyncTask(self.serialConnectionWorker, port, baudrate)

        heartbeat = yield AsyncTask(self.waitForHeartbeatWorker)
        if heartbeat == None:
            self.setConnectionState(False)
            self.ui.tabWidget.setEnabled(False)
        else:
            self.setConnectionStatusBanner('connected')
            softwareVersion, serialNumber, assemblyTime = yield AsyncTask(self.getGimbalParameters)

            # Prompt a for the serial number if the gimbal is factory fresh
            if serialNumber == '' and assemblyTime == 0:
                text, ok = QtGui.QInputDialog.getText(self.parent, '3DR Gimbal', 'Serial Number:')
                if ok and text != '':
                    serialNumber, assemblyTime = yield AsyncTask(self.writeSerialNumber, text)

            # Update the status display
            self.setStatusInfo(
                softwareVersion=softwareVersion,
                serialNumber=serialNumber,
                assemblyTime=assemblyTime
            )

            self.ui.tabWidget.setEnabled(True)
        self.ui.btnConnect.setEnabled(True)

    def closeConnection(self):
        self.link.file.close()
        self.parent.resetUI(self.isCycling)
        self.setStatusInfo()

    def cycleConnection(self):
        self.isCycling = True
        self.ui.btnConnect.clicked.emit()
        self.ui.btnConnect.clicked.emit()
        self.isCycling = False
