import datetime
from PySide import QtCore, QtGui
from PySide.QtCore import Slot
from qtasync import AsyncTask, coroutine
import firmware_helper, firmware_loader
import gui_utils

class firmwareUI(object):
    def __init__(self, parent):
        self.ui = parent.ui
        self.parent = parent
        self.connection = parent.connectionUI

        # Private
        self.firmwareBinary = None

    def setupUI(self):
        self.ui.btnFirmwareFileDialog.clicked.connect(self.handleFirmwareDialog)
        self.ui.btnLoadFirmware.clicked.connect(self.handleFirmwareLoad)

    @Slot()
    def handleFirmwareDialog(self):
        filename, _ = QtGui.QFileDialog.getOpenFileName(self.parent, 'Open Firmware', '')
        fileExtension = str(filename).split(".")[-1].lower()
        if filename and fileExtension == "ax":
            self.ui.txtFirmwareFilename.setText(filename)
            self.parseFirmware(filename)
            self.ui.btnLoadFirmware.setEnabled(True)

    @Slot()
    def handleFirmwareLoad(self):
        if self.connection.isConnected() and self.firmwareBinary != None:
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

    @gui_utils.waitCursor
    def firmwareStartBootloader(self):
        return firmware_loader.start_bootloader(self.connection.getLink())

    @gui_utils.waitCursor
    def firmwareLoadBinary(self):
        def loaderProgressCallback(uploaded_kb, total_kb, percentage):
            self.ui.pbarFirmwareUpdate.setValue(percentage)
        return firmware_loader.load_binary(self.firmwareBinary, self.connection.getLink(), progressCallback=loaderProgressCallback)

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
