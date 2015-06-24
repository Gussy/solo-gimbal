import sys, time, datetime
from PySide import QtCore, QtGui
from PySide.QtCore import QThread, Slot
from qtasync import AsyncTask, coroutine
import setup_mavlink, setup_read_sw_version, setup_validate
import firmware_helper, firmware_loader

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
        self.tabWidget.setEnabled(True)
        self.tabWidget.setGeometry(QtCore.QRect(10, 150, 581, 341))
        self.tabWidget.setTabPosition(QtGui.QTabWidget.North)
        self.tabWidget.setTabShape(QtGui.QTabWidget.Rounded)
        self.tabWidget.setElideMode(QtCore.Qt.ElideNone)
        self.tabWidget.setDocumentMode(False)
        self.tabWidget.setTabsClosable(False)
        self.tabWidget.setMovable(False)
        self.tabWidget.setObjectName("tabWidget")
        self.tabTests = QtGui.QWidget()
        self.tabTests.setEnabled(True)
        self.tabTests.setObjectName("tabTests")
        self.formLayoutWidget_4 = QtGui.QWidget(self.tabTests)
        self.formLayoutWidget_4.setGeometry(QtCore.QRect(10, 80, 551, 221))
        self.formLayoutWidget_4.setObjectName("formLayoutWidget_4")
        self.frmlytValidationResults = QtGui.QFormLayout(self.formLayoutWidget_4)
        self.frmlytValidationResults.setContentsMargins(0, 0, 0, 0)
        self.frmlytValidationResults.setHorizontalSpacing(20)
        self.frmlytValidationResults.setObjectName("frmlytValidationResults")
        self.lblVersionLabel = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblVersionLabel.setEnabled(True)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblVersionLabel.setFont(font)
        self.lblVersionLabel.setObjectName("lblVersionLabel")
        self.frmlytValidationResults.setWidget(0, QtGui.QFormLayout.LabelRole, self.lblVersionLabel)
        self.lblValidationCommutationLabel = QtGui.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblValidationCommutationLabel.setFont(font)
        self.lblValidationCommutationLabel.setObjectName("lblValidationCommutationLabel")
        self.frmlytValidationResults.setWidget(1, QtGui.QFormLayout.LabelRole, self.lblValidationCommutationLabel)
        self.lblValidationJointsLabel = QtGui.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblValidationJointsLabel.setFont(font)
        self.lblValidationJointsLabel.setObjectName("lblValidationJointsLabel")
        self.frmlytValidationResults.setWidget(2, QtGui.QFormLayout.LabelRole, self.lblValidationJointsLabel)
        self.lblValidationGyrosLabel = QtGui.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblValidationGyrosLabel.setFont(font)
        self.lblValidationGyrosLabel.setObjectName("lblValidationGyrosLabel")
        self.frmlytValidationResults.setWidget(3, QtGui.QFormLayout.LabelRole, self.lblValidationGyrosLabel)
        self.lblValidationAccelerometerLabel = QtGui.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblValidationAccelerometerLabel.setFont(font)
        self.lblValidationAccelerometerLabel.setObjectName("lblValidationAccelerometerLabel")
        self.frmlytValidationResults.setWidget(4, QtGui.QFormLayout.LabelRole, self.lblValidationAccelerometerLabel)
        self.lblValidationGainsLabel = QtGui.QLabel(self.formLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(8)
        font.setWeight(75)
        font.setBold(True)
        self.lblValidationGainsLabel.setFont(font)
        self.lblValidationGainsLabel.setObjectName("lblValidationGainsLabel")
        self.frmlytValidationResults.setWidget(5, QtGui.QFormLayout.LabelRole, self.lblValidationGainsLabel)
        self.lblValidationVersion = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationVersion.setText("")
        self.lblValidationVersion.setObjectName("lblValidationVersion")
        self.frmlytValidationResults.setWidget(0, QtGui.QFormLayout.FieldRole, self.lblValidationVersion)
        self.lblValidationCommutation = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationCommutation.setEnabled(True)
        self.lblValidationCommutation.setText("")
        self.lblValidationCommutation.setObjectName("lblValidationCommutation")
        self.frmlytValidationResults.setWidget(1, QtGui.QFormLayout.FieldRole, self.lblValidationCommutation)
        self.lblValidationJoints = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationJoints.setText("")
        self.lblValidationJoints.setObjectName("lblValidationJoints")
        self.frmlytValidationResults.setWidget(2, QtGui.QFormLayout.FieldRole, self.lblValidationJoints)
        self.lblValidationGyros = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationGyros.setText("")
        self.lblValidationGyros.setObjectName("lblValidationGyros")
        self.frmlytValidationResults.setWidget(3, QtGui.QFormLayout.FieldRole, self.lblValidationGyros)
        self.lblValidationAccelerometer = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationAccelerometer.setEnabled(True)
        self.lblValidationAccelerometer.setText("")
        self.lblValidationAccelerometer.setObjectName("lblValidationAccelerometer")
        self.frmlytValidationResults.setWidget(4, QtGui.QFormLayout.FieldRole, self.lblValidationAccelerometer)
        self.lblValidationGains = QtGui.QLabel(self.formLayoutWidget_4)
        self.lblValidationGains.setText("")
        self.lblValidationGains.setObjectName("lblValidationGains")
        self.frmlytValidationResults.setWidget(5, QtGui.QFormLayout.FieldRole, self.lblValidationGains)
        self.btnValidationRunTests = QtGui.QPushButton(self.tabTests)
        self.btnValidationRunTests.setEnabled(True)
        self.btnValidationRunTests.setGeometry(QtCore.QRect(10, 10, 271, 51))
        self.btnValidationRunTests.setObjectName("btnValidationRunTests")
        self.btnValidationSetDefaults = QtGui.QPushButton(self.tabTests)
        self.btnValidationSetDefaults.setEnabled(True)
        self.btnValidationSetDefaults.setGeometry(QtCore.QRect(290, 10, 271, 51))
        self.btnValidationSetDefaults.setObjectName("btnValidationSetDefaults")
        self.tabWidget.addTab(self.tabTests, "")
        self.tabFirmware = QtGui.QWidget()
        self.tabFirmware.setObjectName("tabFirmware")
        self.btnFirmwareFileDialog = QtGui.QPushButton(self.tabFirmware)
        self.btnFirmwareFileDialog.setGeometry(QtCore.QRect(440, 10, 121, 31))
        self.btnFirmwareFileDialog.setObjectName("btnFirmwareFileDialog")
        self.txtFirmwareFilename = QtGui.QLineEdit(self.tabFirmware)
        self.txtFirmwareFilename.setEnabled(True)
        self.txtFirmwareFilename.setGeometry(QtCore.QRect(10, 10, 421, 31))
        self.txtFirmwareFilename.setFrame(True)
        self.txtFirmwareFilename.setEchoMode(QtGui.QLineEdit.Normal)
        self.txtFirmwareFilename.setReadOnly(True)
        self.txtFirmwareFilename.setObjectName("txtFirmwareFilename")
        self.formLayoutWidget_5 = QtGui.QWidget(self.tabFirmware)
        self.formLayoutWidget_5.setGeometry(QtCore.QRect(10, 50, 551, 191))
        self.formLayoutWidget_5.setObjectName("formLayoutWidget_5")
        self.formLayout = QtGui.QFormLayout(self.formLayoutWidget_5)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setHorizontalSpacing(20)
        self.formLayout.setObjectName("formLayout")
        self.lblFirmwareVersionLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareVersionLabel.setFont(font)
        self.lblFirmwareVersionLabel.setObjectName("lblFirmwareVersionLabel")
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.lblFirmwareVersionLabel)
        self.lblFirmwareBuildTimeLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareBuildTimeLabel.setFont(font)
        self.lblFirmwareBuildTimeLabel.setObjectName("lblFirmwareBuildTimeLabel")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.lblFirmwareBuildTimeLabel)
        self.lblFirmwareReleaseNameLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareReleaseNameLabel.setFont(font)
        self.lblFirmwareReleaseNameLabel.setObjectName("lblFirmwareReleaseNameLabel")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.lblFirmwareReleaseNameLabel)
        self.lblFirmwareGitIdentityLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareGitIdentityLabel.setFont(font)
        self.lblFirmwareGitIdentityLabel.setObjectName("lblFirmwareGitIdentityLabel")
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.lblFirmwareGitIdentityLabel)
        self.lblFirmwareSizeLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareSizeLabel.setFont(font)
        self.lblFirmwareSizeLabel.setObjectName("lblFirmwareSizeLabel")
        self.formLayout.setWidget(4, QtGui.QFormLayout.LabelRole, self.lblFirmwareSizeLabel)
        self.lblFirmwareVersion = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareVersion.setText("")
        self.lblFirmwareVersion.setObjectName("lblFirmwareVersion")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.lblFirmwareVersion)
        self.lblFirmwareBuildTime = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareBuildTime.setText("")
        self.lblFirmwareBuildTime.setObjectName("lblFirmwareBuildTime")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.lblFirmwareBuildTime)
        self.lblFirmwareReleaseName = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareReleaseName.setText("")
        self.lblFirmwareReleaseName.setObjectName("lblFirmwareReleaseName")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.lblFirmwareReleaseName)
        self.lblFirmwareGitIdentity = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareGitIdentity.setText("")
        self.lblFirmwareGitIdentity.setObjectName("lblFirmwareGitIdentity")
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.lblFirmwareGitIdentity)
        self.lblFirmwareSize = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareSize.setText("")
        self.lblFirmwareSize.setObjectName("lblFirmwareSize")
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.lblFirmwareSize)
        self.lblFirmwareChecksumLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareChecksumLabel.setFont(font)
        self.lblFirmwareChecksumLabel.setObjectName("lblFirmwareChecksumLabel")
        self.formLayout.setWidget(5, QtGui.QFormLayout.LabelRole, self.lblFirmwareChecksumLabel)
        self.lblFirmwareChecksum = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareChecksum.setText("")
        self.lblFirmwareChecksum.setObjectName("lblFirmwareChecksum")
        self.formLayout.setWidget(5, QtGui.QFormLayout.FieldRole, self.lblFirmwareChecksum)
        self.lblFirmwareStatusLabel = QtGui.QLabel(self.formLayoutWidget_5)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.lblFirmwareStatusLabel.setFont(font)
        self.lblFirmwareStatusLabel.setObjectName("lblFirmwareStatusLabel")
        self.formLayout.setWidget(6, QtGui.QFormLayout.LabelRole, self.lblFirmwareStatusLabel)
        self.lblFirmwareStatus = QtGui.QLabel(self.formLayoutWidget_5)
        self.lblFirmwareStatus.setText("")
        self.lblFirmwareStatus.setObjectName("lblFirmwareStatus")
        self.formLayout.setWidget(6, QtGui.QFormLayout.FieldRole, self.lblFirmwareStatus)
        self.btnLoadFirmware = QtGui.QPushButton(self.tabFirmware)
        self.btnLoadFirmware.setEnabled(False)
        self.btnLoadFirmware.setGeometry(QtCore.QRect(441, 260, 121, 41))
        self.btnLoadFirmware.setObjectName("btnLoadFirmware")
        self.pbarFirmwareUpdate = QtGui.QProgressBar(self.tabFirmware)
        self.pbarFirmwareUpdate.setGeometry(QtCore.QRect(10, 260, 421, 41))
        self.pbarFirmwareUpdate.setProperty("value", 0)
        self.pbarFirmwareUpdate.setTextVisible(False)
        self.pbarFirmwareUpdate.setObjectName("pbarFirmwareUpdate")
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
        self.tabWidget.setCurrentIndex(1)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "3DR Gimbal Tools", None, QtGui.QApplication.UnicodeUTF8))
        self.lblVersionLabel.setText(QtGui.QApplication.translate("MainWindow", "Version:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblValidationCommutationLabel.setText(QtGui.QApplication.translate("MainWindow", "Commutation:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblValidationJointsLabel.setText(QtGui.QApplication.translate("MainWindow", "Joints:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblValidationGyrosLabel.setText(QtGui.QApplication.translate("MainWindow", "Gyros:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblValidationAccelerometerLabel.setText(QtGui.QApplication.translate("MainWindow", "Accelerometer:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblValidationGainsLabel.setText(QtGui.QApplication.translate("MainWindow", "Gains:", None, QtGui.QApplication.UnicodeUTF8))
        self.btnValidationRunTests.setText(QtGui.QApplication.translate("MainWindow", "Run Tests", None, QtGui.QApplication.UnicodeUTF8))
        self.btnValidationSetDefaults.setText(QtGui.QApplication.translate("MainWindow", "Set Defaults", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabTests), QtGui.QApplication.translate("MainWindow", "Tests", None, QtGui.QApplication.UnicodeUTF8))
        self.btnFirmwareFileDialog.setText(QtGui.QApplication.translate("MainWindow", "Open Firmware", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareVersionLabel.setText(QtGui.QApplication.translate("MainWindow", "Version:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareBuildTimeLabel.setText(QtGui.QApplication.translate("MainWindow", "Build Time:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareReleaseNameLabel.setText(QtGui.QApplication.translate("MainWindow", "Release Name:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareGitIdentityLabel.setText(QtGui.QApplication.translate("MainWindow", "Git Identity:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareSizeLabel.setText(QtGui.QApplication.translate("MainWindow", "Firmware Size:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareChecksumLabel.setText(QtGui.QApplication.translate("MainWindow", "Checksum:", None, QtGui.QApplication.UnicodeUTF8))
        self.lblFirmwareStatusLabel.setText(QtGui.QApplication.translate("MainWindow", "Status:", None, QtGui.QApplication.UnicodeUTF8))
        self.btnLoadFirmware.setText(QtGui.QApplication.translate("MainWindow", "Load Firmware", None, QtGui.QApplication.UnicodeUTF8))
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

        # Disable the tabwidget
        self.ui.tabWidget.setEnabled(False)

        # Setup connection UI
        self.addBaudrates()
        self.ui.rbAuto.clicked.connect(self.handleConnectionTypeAuto)
        self.ui.rbSerial.clicked.connect(self.handleConnectionTypeSerial)
        self.ui.btnConnect.clicked.connect(self.handleConnectButton)

        # Setup validation UI
        self.ui.btnValidationRunTests.clicked.connect(self.handleRunValidation)
        self.ui.btnValidationSetDefaults.clicked.connect(self.handleSetDefaults)

        # Setup firmware UI
        self.ui.btnFirmwareFileDialog.clicked.connect(self.handleFirmwareDialog)
        self.ui.btnLoadFirmware.clicked.connect(self.handleFirmwareLoad)

        # MAVLink
        self.mavport = None
        self.link = None

        # Setup states
        self.connectWithAuto = True
        self.connected = False

        # Firmware
        self.firmwareBinary = None

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

    def resetUI(self):
        self.clearValidationResults()
        self.clearFirmwareInfoUI()

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

    @Slot()
    def handleRunValidation(self):
        if self.connected:
            self.runAsyncValidation()

    @Slot()
    def handleSetDefaults(self):
        if self.connected:
            self.runAsyncSetDefaults()

    @Slot()
    def handleFirmwareDialog(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self, 'Open Firmware', '')
        fileExtension = str(filename).split(".")[-1].lower()
        if filename and fileExtension == "ax":
            self.ui.txtFirmwareFilename.setText(filename)
            self.parseFirmware(filename)
            self.ui.btnLoadFirmware.setEnabled(True)

    @Slot()
    def handleFirmwareLoad(self):
        if self.connected and self.firmwareBinary != None:
            self.runAsyncFirmwareLoad()

    def parseFirmware(self, filename):
        firmware = firmware_helper.load_firmware(filename)
        self.firmwareBinary, checksum = firmware_helper.append_checksum(firmware['binary'])
        self.updateFirmwareInfoUI(
            "v%s" % firmware['version'],
            datetime.datetime.fromtimestamp(firmware['build_time']).strftime('%d/%m/%Y @ %H:%M:%S'),
            firmware['release'],
            firmware['git_identity'],
            "%i kB" % int(firmware['image_size'] / 1024),
            "%04X" % checksum
        )

    def updateFirmwareInfoUI(self, version, buildTime, releaseName, gitIdentity, firmwareSize, checksum):
        self.ui.lblFirmwareVersion.setText(version)
        self.ui.lblFirmwareBuildTime.setText(buildTime)
        self.ui.lblFirmwareReleaseName.setText(releaseName)
        self.ui.lblFirmwareGitIdentity.setText(gitIdentity)
        self.ui.lblFirmwareSize.setText(firmwareSize)
        self.ui.lblFirmwareChecksum.setText(checksum)

    def clearFirmwareInfoUI(self):
        self.updateFirmwareInfoUI("", "", "", "", "", "")

    def firmwareStartBootloader(self):
        return firmware_loader.start_bootloader(self.link)

    def firmwareLoadBinary(self):
        def loaderProgressCallback(uploaded_kb, total_kb, percentage):
            self.ui.pbarFirmwareUpdate.setValue(percentage)
        return firmware_loader.load_binary(self.firmwareBinary, self.link, progressCallback=loaderProgressCallback)

    @coroutine
    def runAsyncFirmwareLoad(self):
        self.ui.pbarFirmwareUpdate.setValue(0)
        self.ui.btnFirmwareFileDialog.setEnabled(False)
        self.ui.btnLoadFirmware.setEnabled(False)
        self.ui.lblFirmwareStatus.setText("")

        # Start the bootloader
        self.ui.lblFirmwareStatus.setText("Starting firmware update")
        bootloader = yield AsyncTask(self.firmwareStartBootloader)
        if bootloader == firmware_loader.Results.NoResponse:
            self.ui.lblFirmwareStatus.setText("No response from gimbal")
        elif bootloader == firmware_loader.Results.InBoot:
            self.ui.lblFirmwareStatus.setText('Target already in bootloader mode')
        elif bootloader == firmware_loader.Results.Restarting:
            self.ui.lblFirmwareStatus.setText("Target in bootloader mode, restarting")

        # Load the binary using the bootloader
        if bootloader != firmware_loader.Results.NoResponse:
            self.ui.lblFirmwareStatus.setText("Loading firmware")
            load = yield AsyncTask(self.firmwareLoadBinary)
            if load == firmware_loader.Results.Success:
                self.ui.lblFirmwareStatus.setText("Upload successful")
            elif load == firmware_loader.Results.NoResponse:
                self.ui.lblFirmwareStatus.setText("No response from gimbal, exiting.")
            elif load == firmware_loader.Results.Timeout:
                self.ui.lblFirmwareStatus.setText("Timeout")
            else:
                self.ui.lblFirmwareStatus.setText("Unknown error while finishing bootloading")

        self.ui.pbarFirmwareUpdate.setValue(0)
        self.ui.btnFirmwareFileDialog.setEnabled(True)
        self.ui.btnLoadFirmware.setEnabled(True)
        self.ui.lblFirmwareStatus.setText("")

    def validateVersion(self):
        return setup_validate.validate_version(self.link)

    def validateCommutation(self):
        return setup_validate.validate_comutation(self.link)
    
    def validateJoints(self):
        return setup_validate.validate_joints(self.link)
    
    def validateGyros(self):
        return setup_validate.validate_gyros(self.link)
    
    def validateAccelerometers(self):
        return setup_validate.validate_accelerometers(self.link)
    
    def validateGains(self):
        return setup_validate.validate_gains(self.link)

    def setDefaults(self):
        return setup_validate.restore_defaults(self.link)

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
        self.resetUI()

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = ControlMainWindow()
    gui.show()
    sys.exit(app.exec_())
