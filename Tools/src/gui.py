import sys, time
from PySide import QtCore, QtGui
from PySide.QtCore import QThread, Slot
from qtasync import AsyncTask, coroutine
import setup_mavlink, setup_read_sw_version

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(600, 500)
        MainWindow.setMinimumSize(QtCore.QSize(600, 500))
        MainWindow.setMaximumSize(QtCore.QSize(600, 500))
        MainWindow.setWindowOpacity(1.0)
        MainWindow.setTabShape(QtGui.QTabWidget.Rounded)
        MainWindow.setUnifiedTitleAndToolBarOnMac(False)
        self.centralWidget = QtGui.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.tabWidget = QtGui.QTabWidget(self.centralWidget)
        self.tabWidget.setGeometry(QtCore.QRect(10, 149, 581, 341))
        self.tabWidget.setTabPosition(QtGui.QTabWidget.North)
        self.tabWidget.setTabShape(QtGui.QTabWidget.Rounded)
        self.tabWidget.setElideMode(QtCore.Qt.ElideNone)
        self.tabWidget.setDocumentMode(False)
        self.tabWidget.setTabsClosable(False)
        self.tabWidget.setMovable(False)
        self.tabWidget.setObjectName("tabWidget")
        self.tabWidget.setEnabled(False)
        self.tabTests = QtGui.QWidget()
        self.tabTests.setEnabled(True)
        self.tabTests.setObjectName("tabTests")
        self.tabWidget.addTab(self.tabTests, "")
        self.tabFirmware = QtGui.QWidget()
        self.tabFirmware.setObjectName("tabFirmware")
        self.tabWidget.addTab(self.tabFirmware, "")
        self.tabCalibration = QtGui.QWidget()
        self.tabCalibration.setObjectName("tabCalibration")
        self.tabWidget.addTab(self.tabCalibration, "")
        self.formLayoutWidget = QtGui.QWidget(self.centralWidget)
        self.formLayoutWidget.setGeometry(QtCore.QRect(9, 10, 255, 101))
        self.formLayoutWidget.setObjectName("formLayoutWidget")
        self.frmlytConnection = QtGui.QFormLayout(self.formLayoutWidget)
        self.frmlytConnection.setContentsMargins(0, 0, 0, 0)
        self.frmlytConnection.setObjectName("frmlytConnection")
        self.lblConnectionLabel = QtGui.QLabel(self.formLayoutWidget)
        font = QtGui.QFont()
        font.setWeight(50)
        font.setBold(False)
        self.lblConnectionLabel.setFont(font)
        self.lblConnectionLabel.setObjectName("lblConnectionLabel")
        self.frmlytConnection.setWidget(0, QtGui.QFormLayout.LabelRole, self.lblConnectionLabel)
        self.lblSerialPortLabel = QtGui.QLabel(self.formLayoutWidget)
        self.lblSerialPortLabel.setObjectName("lblSerialPortLabel")
        self.frmlytConnection.setWidget(1, QtGui.QFormLayout.LabelRole, self.lblSerialPortLabel)
        self.lblBaurdateLabel = QtGui.QLabel(self.formLayoutWidget)
        self.lblBaurdateLabel.setObjectName("lblBaurdateLabel")
        self.frmlytConnection.setWidget(2, QtGui.QFormLayout.LabelRole, self.lblBaurdateLabel)
        self.frmlytConnectionType = QtGui.QFormLayout()
        self.frmlytConnectionType.setObjectName("frmlytConnectionType")
        self.rbAuto = QtGui.QRadioButton(self.formLayoutWidget)
        self.rbAuto.setChecked(True)
        self.rbAuto.setObjectName("rbAuto")
        self.frmlytConnectionType.setWidget(0, QtGui.QFormLayout.LabelRole, self.rbAuto)
        self.rbSerial = QtGui.QRadioButton(self.formLayoutWidget)
        self.rbSerial.setChecked(False)
        self.rbSerial.setObjectName("rbSerial")
        self.frmlytConnectionType.setWidget(0, QtGui.QFormLayout.FieldRole, self.rbSerial)
        self.frmlytConnection.setLayout(0, QtGui.QFormLayout.FieldRole, self.frmlytConnectionType)
        self.cbSerialPort = QtGui.QComboBox(self.formLayoutWidget)
        self.cbSerialPort.setEnabled(False)
        self.cbSerialPort.setObjectName("cbSerialPort")
        self.frmlytConnection.setWidget(1, QtGui.QFormLayout.FieldRole, self.cbSerialPort)
        self.cbBaudrate = QtGui.QComboBox(self.formLayoutWidget)
        self.cbBaudrate.setEnabled(False)
        self.cbBaudrate.setObjectName("cbBaudrate")
        self.frmlytConnection.setWidget(2, QtGui.QFormLayout.FieldRole, self.cbBaudrate)
        self.btnConnect = QtGui.QPushButton(self.centralWidget)
        self.btnConnect.setGeometry(QtCore.QRect(10, 110, 251, 34))
        self.btnConnect.setObjectName("btnConnect")
        self.lblConnectionStatus = QtGui.QLabel(self.centralWidget)
        self.lblConnectionStatus.setGeometry(QtCore.QRect(270, 10, 321, 41))
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setWeight(75)
        font.setBold(True)
        self.lblConnectionStatus.setFont(font)
        self.lblConnectionStatus.setStyleSheet("color: rgb(255, 255, 255);\n"
"background-color: rgb(255, 0, 0);\n"
"border-radius: 20px;")
        self.lblConnectionStatus.setAlignment(QtCore.Qt.AlignCenter)
        self.lblConnectionStatus.setObjectName("lblConnectionStatus")
        self.formLayoutWidget_3 = QtGui.QWidget(self.centralWidget)
        self.formLayoutWidget_3.setGeometry(QtCore.QRect(270, 60, 324, 81))
        self.formLayoutWidget_3.setObjectName("formLayoutWidget_3")
        self.frmlytParameters = QtGui.QFormLayout(self.formLayoutWidget_3)
        self.frmlytParameters.setContentsMargins(0, 0, 0, 0)
        self.frmlytParameters.setObjectName("frmlytParameters")
        self.lblSoftwareVersionLabel = QtGui.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblSoftwareVersionLabel.setFont(font)
        self.lblSoftwareVersionLabel.setObjectName("lblSoftwareVersionLabel")
        self.frmlytParameters.setWidget(0, QtGui.QFormLayout.LabelRole, self.lblSoftwareVersionLabel)
        self.lblSoftwareVersion = QtGui.QLabel(self.formLayoutWidget_3)
        self.lblSoftwareVersion.setText("")
        self.lblSoftwareVersion.setObjectName("lblSoftwareVersion")
        self.frmlytParameters.setWidget(0, QtGui.QFormLayout.FieldRole, self.lblSoftwareVersion)
        self.lblSerialNumberLabel = QtGui.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblSerialNumberLabel.setFont(font)
        self.lblSerialNumberLabel.setObjectName("lblSerialNumberLabel")
        self.frmlytParameters.setWidget(1, QtGui.QFormLayout.LabelRole, self.lblSerialNumberLabel)
        self.lblAssemblyDateLabel = QtGui.QLabel(self.formLayoutWidget_3)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblAssemblyDateLabel.setFont(font)
        self.lblAssemblyDateLabel.setObjectName("lblAssemblyDateLabel")
        self.frmlytParameters.setWidget(2, QtGui.QFormLayout.LabelRole, self.lblAssemblyDateLabel)
        self.lblSerialNumber = QtGui.QLabel(self.formLayoutWidget_3)
        self.lblSerialNumber.setText("")
        self.lblSerialNumber.setObjectName("lblSerialNumber")
        self.frmlytParameters.setWidget(1, QtGui.QFormLayout.FieldRole, self.lblSerialNumber)
        self.lblAssemblyDate = QtGui.QLabel(self.formLayoutWidget_3)
        self.lblAssemblyDate.setText("")
        self.lblAssemblyDate.setObjectName("lblAssemblyDate")
        self.frmlytParameters.setWidget(2, QtGui.QFormLayout.FieldRole, self.lblAssemblyDate)
        MainWindow.setCentralWidget(self.centralWidget)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "3DR Gimbal Tools", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabTests), QtGui.QApplication.translate("MainWindow", "Tests", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabFirmware), QtGui.QApplication.translate("MainWindow", "Firmware", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabCalibration), QtGui.QApplication.translate("MainWindow", "Calibration", None, QtGui.QApplication.UnicodeUTF8))
        self.lblConnectionLabel.setText(QtGui.QApplication.translate("MainWindow", "Connection:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblSerialPortLabel.setText(QtGui.QApplication.translate("MainWindow", "Serial Port:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblBaurdateLabel.setText(QtGui.QApplication.translate("MainWindow", "Baudrate:", None, QtGui.QApplication.UnicodeUTF8))
        self.rbAuto.setText(QtGui.QApplication.translate("MainWindow", "Auto", None, QtGui.QApplication.UnicodeUTF8))
        self.rbSerial.setText(QtGui.QApplication.translate("MainWindow", "Serial", None, QtGui.QApplication.UnicodeUTF8))
        self.btnConnect.setText(QtGui.QApplication.translate("MainWindow", "Connect", None, QtGui.QApplication.UnicodeUTF8))
        self.lblConnectionStatus.setText(QtGui.QApplication.translate("MainWindow", "Disconnected", None, QtGui.QApplication.UnicodeUTF8))
        self.lblSoftwareVersionLabel.setText(QtGui.QApplication.translate("MainWindow", "Software Version:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblSerialNumberLabel.setText(QtGui.QApplication.translate("MainWindow", "Serial Number:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblAssemblyDateLabel.setText(QtGui.QApplication.translate("MainWindow", "Assembly Date:", None, QtGui.QApplication.UnicodeUTF8))

class UiUtils():
    def getComboboxSelection(self, combobox):
        return combobox.itemText(combobox.currentIndex())

class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Setup connection UI
        self.addBaudrates()
        self.ui.rbAuto.clicked.connect(self.handleConnectionTypeAuto)
        self.ui.rbSerial.clicked.connect(self.handleConnectionTypeSerial)
        self.ui.btnConnect.clicked.connect(self.handleConnectButton)

        # MAVLink
        self.mavport = None
        self.link = None

        # Setup states
        self.connectWithAuto = True
        self.connected = False

        self.utils = UiUtils()

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

    def setStatusInfo(self, version):
        if version:
            self.ui.lblSoftwareVersion.setText("v%i.%i.%i" % (version[0], version[1], version[2]))
        else:
            self.ui.lblSoftwareVersion.setText("")

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

    def autoConnectionWorker(self):
        return setup_mavlink.open_comm()

    def serialConnectionWorker(self, port, baudrate):
        return setup_mavlink.open_comm(port, baudrate)

    def waitForHeartbeatWorker(self):
        return setup_mavlink.wait_for_heartbeat(self.link)

    def getGimbalParameters(self):
        return setup_read_sw_version.readSWver(self.link)

    @coroutine
    def attemptConnection(self):
        self.ui.btnConnect.setEnabled(False)

        # Open the mavlink connection
        if self.connectWithAuto:
            self.mavport, self.link = yield AsyncTask(self.autoConnectionWorker)
        else:
            # Get the current serial port if connecting with serial
            port = self.utils.getComboboxSelection(self.ui.cbSerialPort)
            baudrate = self.utils.getComboboxSelection(self.ui.cbBaudrate)
            self.mavport, self.link = yield AsyncTask(self.serialConnectionWorker, port, baudrate)

        heartbeat = yield AsyncTask(self.waitForHeartbeatWorker)
        if heartbeat == None:
            self.setConnectionState(False)
            self.ui.tabWidget.setEnabled(False)
        else:
            self.setConnectionStatusBanner('connected')
            softwareVersion = yield AsyncTask(self.getGimbalParameters)
            self.setStatusInfo(softwareVersion)
            self.ui.tabWidget.setEnabled(True)
        self.ui.btnConnect.setEnabled(True)

    def closeConnection(self):
        self.link.file.close()
        self.setStatusInfo(None)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = ControlMainWindow()
    gui.show()
    sys.exit(app.exec_())
