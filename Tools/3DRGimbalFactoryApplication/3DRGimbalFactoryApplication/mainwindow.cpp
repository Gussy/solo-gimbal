#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "load_firmware_dialog.h"
#include "calibrate_axes_dialog.h"
#include "home_offset_calibration_result_dialog.h"
#include "enter_factory_parameters_dialog.h"
#include "axis_range_test_dialog.h"
#include "axis_calibration_status_dialog.h"
#include "choose_axes_to_calibrate_dialog.h"
#include "version.h"
#include "wobble_test_dialog.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QList>
#include <QMessageBox>
#include <QFileDialog>
#include <QFile>
#include <QDialog>
#include <QTime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_gimbalMessagesVisible(false)
{
    ui->setupUi(this);

    foreach (QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comPort->addItem(port.portName());
    }

    ui->baudRate->addItem("9600", 9600);
    ui->baudRate->addItem("19200", 19200);
    ui->baudRate->addItem("57600", 57600);
    ui->baudRate->addItem("115200", 115200);
    ui->baudRate->addItem("230400", 230400);

    connect(&m_connectionTimeoutTimer, SIGNAL(timeout()), this, SLOT(connectionTimeout()));

    // Populate the version string
    ui->version_label->setText(QString(GitVersionString) + QString("-") + QString(GitBranch));

    // Hide the gimbal messages console by default
    ui->gimbalMessagesConsole->setVisible(false);
    this->adjustSize();

    // Set up gimbal messages window stylesheet
    ui->gimbalMessagesConsole->document()->setDefaultStyleSheet(".info_msg { \
                                                                    color: #00529B; \
                                                                    background-color: #BDE5F8; \
                                                                } \
                                                                .warning_msg { \
                                                                    color: #9F6000; \
                                                                    background-color: #FEEFB3; \
                                                                } \
                                                                .error_msg { \
                                                                    color: #D8000C; \
                                                                    background-color: #FFBABA; \
                                                                }");
    // Set max number of gimbal messages at 1000
    ui->gimbalMessagesConsole->document()->setMaximumBlockCount(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_connectButton_clicked()
{
    // Initialize and start the serial interface thread
    m_serialThreadObj = new SerialInterfaceThread(ui->comPort->currentText(), ui->baudRate->currentData().toInt());
    m_serialThreadObj->moveToThread(&m_serialThread);
    connect(&m_serialThread, SIGNAL(started()), m_serialThreadObj, SLOT(run()));
    connect(&m_serialThread, SIGNAL(finished()), m_serialThreadObj, SLOT(deleteLater()));
    connect(m_serialThreadObj, SIGNAL(receivedHeartbeat()), this, SLOT(receivedGimbalHeartbeat()));
    connect(m_serialThreadObj, SIGNAL(receivedDataTransmissionHandshake()), this, SLOT(receivedGimbalDataTransmissionHandshake()));
    connect(this, SIGNAL(closeSerialInterface()), m_serialThreadObj, SLOT(requestStop()));
    connect(this, SIGNAL(requestFirmwareDownload(QString)), m_serialThreadObj, SLOT(requestLoadFirmware(QString)));
    //connect(m_serialThreadObj, SIGNAL(firmwareLoadError(QString)), this, SLOT(debugFirmwareLoadError(QString)));
    //connect(m_serialThreadObj, SIGNAL(firmwareLoadProgress(double)), this, SLOT(debugFirmwareLoadProgress(double)));
    connect(m_serialThreadObj, SIGNAL(firmwareLoadError(QString)), this, SIGNAL(firmwareLoadError(QString)));
    connect(m_serialThreadObj, SIGNAL(firmwareLoadProgress(double)), this, SIGNAL(firmwareLoadProgress(double)));
    connect(m_serialThreadObj, SIGNAL(axisCalibrationStarted(int)), this, SIGNAL(axisCalibrationStarted(int)));
    connect(m_serialThreadObj, SIGNAL(axisCalibrationFinished(int,bool)), this, SIGNAL(axisCalibrationFinished(int,bool)));
    connect(this, SIGNAL(requestFirmwareVersion()), m_serialThreadObj, SLOT(requestFirmwareVersion()));
    connect(m_serialThreadObj, SIGNAL(sendFirmwareVersion(QString)), this, SLOT(receiveFirmwareVersion(QString)));
    connect(m_serialThreadObj, SIGNAL(serialPortOpenError(QString)), this, SLOT(receiveSerialPortOpenError(QString)));
    connect(this, SIGNAL(retryAxesCalibration()), m_serialThreadObj, SLOT(retryAxesCalibration()));
    connect(this, SIGNAL(requestResetGimbal()), m_serialThreadObj, SLOT(requestResetGimbal()));
    connect(this, SIGNAL(requestHomeOffsetCalibration()), m_serialThreadObj, SLOT(requestCalibrateHomeOffsets()));
    connect(m_serialThreadObj, SIGNAL(homeOffsetCalibrationFinished(bool)), this, SLOT(receiveHomeOffsetCalibrationStatus(bool)));
    connect(m_serialThreadObj, SIGNAL(sendNewHomeOffsets(int,int,int)), this, SIGNAL(sendNewHomeOffsets(int,int,int)));
    connect(this, SIGNAL(requestFactoryParameters()), m_serialThreadObj, SLOT(requestFactoryParameters()));
    connect(m_serialThreadObj, SIGNAL(sendFactoryParameters(QString,QString)), this, SLOT(receiveFactoryParameters(QString,QString)));
    connect(this, SIGNAL(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)), m_serialThreadObj, SLOT(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)));
    connect(m_serialThreadObj, SIGNAL(factoryParametersLoaded()), this, SIGNAL(factoryParametersLoaded()));
    connect(this, SIGNAL(requestEraseGimbalFlash()), m_serialThreadObj, SLOT(requestGimbalEraseFlash()));
    connect(this, SIGNAL(requestStartFactoryTests(unsigned char, unsigned char)), m_serialThreadObj, SLOT(requestGimbalFactoryTests(unsigned char, unsigned char)));
    connect(m_serialThreadObj, SIGNAL(factoryTestsStatus(int,int,int,int)), this, SIGNAL(factoryTestsStatus(int,int,int,int)));
    connect(m_serialThreadObj, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)), this, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)));
    connect(this, SIGNAL(requestAxisCalibrationStatus()), m_serialThreadObj, SLOT(requestAxisCalibrationStatus()));
    connect(this, SIGNAL(requestCalibrateAxesSetup(bool,bool,bool)), m_serialThreadObj, SLOT(requestCalibrateAxesSetup(bool,bool,bool)));
    connect(this, SIGNAL(requestCalibrateAxesPerform()), m_serialThreadObj, SLOT(requestCalibrateAxesPerform()));
    connect(m_serialThreadObj, SIGNAL(gimbalStatusMessage(uint,QString)), this, SLOT(receiveGimbalStatusMessage(uint,QString)));
    connect(m_serialThreadObj, SIGNAL(receivedTestStatus(unsigned char,float)), this, SIGNAL(receivedTestStatus(unsigned char,float)));
    connect(m_serialThreadObj, SIGNAL(receivedGimbalReport(float, float, float)), this, SIGNAL(receivedGimbalReport(float, float, float)));
    m_serialThread.start();

    // Disable the connect button, enable the disconnect button
    ui->connectButton->setEnabled(false);
    ui->disconnectButton->setEnabled(true);

    // Change connection status to connecting
    ui->connectionStatus->setText("Connecting...");

    // Start the connection timeout timer
    // (this will timeout if we don't get either a heartbeat or a data transmission handshake from the gimbal)
    m_connectionTimeoutTimer.setSingleShot(true);
    m_connectionTimeoutTimer.start(5000);
}

void MainWindow::receiveSerialPortOpenError(QString errorMsg)
{
    // Disable the disconnect button, enable the connect button
    ui->disconnectButton->setEnabled(false);
    ui->connectButton->setEnabled(true);

    // Change connection status to disconnected
    ui->connectionStatus->setText("Disconnected");

    // If there's a pending connection timeout, cancel it
    m_connectionTimeoutTimer.stop();

    QMessageBox msg;
    msg.setText("Error Opening Serial Port");
    msg.setInformativeText("Encountered error \"" + errorMsg + "\" while trying to open the serial port.  Please check your settings and try again");
    msg.setIcon(QMessageBox::Warning);
    msg.setStandardButtons(QMessageBox::Ok);
    msg.setDefaultButton(QMessageBox::Ok);
    msg.exec();
}

void MainWindow::connectionTimeout()
{
    QMessageBox msg;
    msg.setText("Gimbal Connection Timeout");
    msg.setInformativeText("Didn't receive a message from the gimbal in 5 seconds.  Please check gimbal power and connection and try again");
    msg.setIcon(QMessageBox::Warning);
    msg.setStandardButtons(QMessageBox::Ok);
    msg.setDefaultButton(QMessageBox::Ok);
    msg.exec();

    // Ask the serial interface thread to stop
    emit closeSerialInterface();

    // Disable the disconnect button, enable the connect button
    ui->disconnectButton->setEnabled(false);
    ui->connectButton->setEnabled(true);

    // Change connection status to disconnected
    ui->connectionStatus->setText("Disconnected");
}

void MainWindow::receivedGimbalHeartbeat()
{
    // If we receive a gimbal heartbeat when we connect, it means there is firmware loaded on the gimbal

    // Cancel the pending connection timeout
    m_connectionTimeoutTimer.stop();

    // Set connection status to connected
    ui->connectionStatus->setText("Connected");

    // Since we don't know whether the gimbal needs calibration yet, only enable
    // the subset of controls that are valid when the gimbal is uncalibrated
    setUIGimbalConnectedNeedsCalibration();

    // Request gimbal firmware version
    emit requestFirmwareVersion();

    // Request the factory parameters (assembly date, serial number)
    emit requestFactoryParameters();

    // Wait for the gimbal axis calibration status
    AxisCalibrationStatusDialog dialog;
    connect(this, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)), &dialog, SLOT(receiveCalibrationStatus(bool,bool,bool)));
    connect(&dialog, SIGNAL(gimbalRequiresCalibration(bool)), this, SLOT(gimbalRequiresCalibration(bool)));

    // Request the gimbal axis calibration status
    emit requestAxisCalibrationStatus();

    dialog.exec();
}

void MainWindow::receivedGimbalDataTransmissionHandshake()
{
    // If we receive a data transmission handshake when we connect, it means there is no firmware loaded on the gimbal

    // Cancel the pending connection timeout
    m_connectionTimeoutTimer.stop();

    // Set connection status to connected
    ui->connectionStatus->setText("Connected");

    setUIGimbalConnectedNoFirmware();

    // Show the user a message indicating that they need to load firmware
    QMessageBox msg;
    msg.setText("No firmware loaded on gimbal");
    msg.setInformativeText("Before calibrating the gimbal, firmware must be loaded on it");
    msg.setIcon(QMessageBox::Information);
    msg.setStandardButtons(QMessageBox::Ok);
    msg.setDefaultButton(QMessageBox::Ok);
    msg.exec();
}

void MainWindow::on_disconnectButton_clicked()
{
    // Ask the serial interface thread to stop
    emit closeSerialInterface();

    // Disable the disconnect button, enable the connect button
    ui->disconnectButton->setEnabled(false);
    ui->connectButton->setEnabled(true);

    setUIGimbalDisconnected();

    // Change connection status to disconnected
    ui->connectionStatus->setText("Disconnected");

    // Clear out firmware version
    ui->firmwareLoaded->setText("None");

    // Clear out assembly date/time and serial number
    ui->assemblyDateTime->setText("None");
    ui->serialNumber->setText("None");

    // If there's a pending connection timeout, cancel it
    m_connectionTimeoutTimer.stop();
}

void MainWindow::on_firmwareImageBrowseButton_clicked()
{
    QString firmwareFileName = QFileDialog::getOpenFileName(this, "Select Firmware Image", QString(), "Firmware Image Files (*.hex);;All Files (*.*)");
    ui->firmwareImage->setText(firmwareFileName);
}

void MainWindow::on_loadFirmwareButton_clicked()
{
    // Make sure the file path in the firmware image textbox exists
    if (!QFile(ui->firmwareImage->text()).exists()) {
        QMessageBox msg;
        msg.setText("Invalid Firmware File");
        msg.setInformativeText("The file \"" + ui->firmwareImage->text() + "\" does not appear to exist.  Please pick a valid file and try again");
        msg.setIcon(QMessageBox::Warning);
        msg.setStandardButtons(QMessageBox::Ok);
        msg.setDefaultButton(QMessageBox::Ok);
        msg.exec();
    } else {
        LoadFirmwareDialog firmwareDialog;
        connect(this, SIGNAL(firmwareLoadError(QString)), &firmwareDialog, SLOT(firmwareUpdateError(QString)));
        connect(this, SIGNAL(firmwareLoadProgress(double)), &firmwareDialog, SLOT(updateFirmwareProgress(double)));
        emit requestFirmwareDownload(ui->firmwareImage->text());
        if (firmwareDialog.exec() == QDialog::Accepted) {
            // Once the firmware load has completed, enable the parts of the UI that are valid if the gimbal is not calibrated
            setUIGimbalConnectedNeedsCalibration();

            // Bring up the axis calibration status dialog
            AxisCalibrationStatusDialog calibrationStatusDialog;
            connect(this, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)), &calibrationStatusDialog, SLOT(receiveCalibrationStatus(bool,bool,bool)));
            connect(&calibrationStatusDialog, SIGNAL(gimbalRequiresCalibration(bool)), this, SLOT(gimbalRequiresCalibration(bool)));
            calibrationStatusDialog.exec();

            // Once the user has closed the calibration status dialog,
            // bring up the factory parameters dialog to set assembly date/time and serial number
            EnterFactoryParametersDialog factoryParmsDialog;
            connect(&factoryParmsDialog, SIGNAL(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)), this, SIGNAL(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)));
            connect(this, SIGNAL(factoryParametersLoaded()), &factoryParmsDialog, SLOT(factoryParametersLoaded()));
            factoryParmsDialog.exec();

            // Reload the firmware version, since it has probably changed after the update
            emit requestFirmwareVersion();
            // Also reload the factory parameters, since the user has entered new ones
            emit requestFactoryParameters();
        }
    }
}

void MainWindow::receiveFirmwareVersion(QString versionString)
{
    ui->firmwareLoaded->setText(versionString);
}

void MainWindow::receiveGimbalStatusMessage(unsigned int severity, QString message)
{
    QString newMessage = "";

    // Set the text color appropriate to the message severity
    switch (severity) {
        case MAV_SEVERITY_EMERGENCY:
        case MAV_SEVERITY_ALERT:
        case MAV_SEVERITY_CRITICAL:
        case MAV_SEVERITY_ERROR:
            newMessage += "<span class='error_msg'>";
            break;

        case MAV_SEVERITY_WARNING:
            newMessage += "<span class='warning_msg'>";
            break;

        case MAV_SEVERITY_NOTICE:
        case MAV_SEVERITY_INFO:
        case MAV_SEVERITY_DEBUG:
            newMessage += "<span class='info_msg'>";
            break;
    }

    // Add the current time to the message
    newMessage += QTime::currentTime().toString("HH:mm:ss ");

    // Add the severity level to the message
    switch (severity) {
        case MAV_SEVERITY_EMERGENCY:
            newMessage += QString("EMERGENCY: ");
            break;

        case MAV_SEVERITY_ALERT:
            newMessage += QString("ALERT: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_CRITICAL:
            newMessage += QString("CRITICAL: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_ERROR:
            newMessage += QString("ERROR: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_WARNING:
            newMessage += QString("WARNING: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_NOTICE:
            newMessage += QString("NOTICE: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_INFO:
            newMessage += QString("INFO: ").leftJustified(11, ' ');
            break;

        case MAV_SEVERITY_DEBUG:
            newMessage += QString("DEBUG: ").leftJustified(11, ' ');
            break;
    }

    // Add the actual message
    newMessage += message;

    // Close the color span tag
    newMessage += "</span>";

    // Append the new message to the log window
    ui->gimbalMessagesConsole->textCursor().insertBlock();
    ui->gimbalMessagesConsole->textCursor().insertHtml(newMessage);
    ui->gimbalMessagesConsole->ensureCursorVisible();
}

void MainWindow::on_runAxisCalibrationButton_clicked()
{
    // First, make sure the user really wants to run the axis calibration
    QMessageBox msg;
    msg.setText("Confirm Axis Calibration");
    msg.setInformativeText("The axis calibration takes ~5 minutes, and can't be canceled once started.  Are you sure you want to proceed?");
    msg.setIcon(QMessageBox::Information);
    msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg.setDefaultButton(QMessageBox::No);
    if (msg.exec() == QMessageBox::Yes) {
        ChooseAxesToCalibrateDialog chooseAxesDialog;
        connect(&chooseAxesDialog, SIGNAL(requestAxisCalibrationSetup(bool,bool,bool)), this, SIGNAL(requestCalibrateAxesSetup(bool,bool,bool)));
        connect(&chooseAxesDialog, SIGNAL(requestAxisCalibrationStart()), this, SIGNAL(requestCalibrateAxesPerform()));
        connect(this, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)), &chooseAxesDialog, SLOT(receiveGimbalAxisCalibrationStatus(bool,bool,bool)));

        CalibrateAxesDialog calibrateAxesDialog;
        connect(this, SIGNAL(axisCalibrationStarted(int)), &calibrateAxesDialog, SLOT(axisCalibrationStarted(int)));
        connect(this, SIGNAL(axisCalibrationFinished(int,bool)), &calibrateAxesDialog, SLOT(axisCalibrationFinished(int,bool)));
        connect(&calibrateAxesDialog, SIGNAL(retryAxesCalibration()), this, SIGNAL(retryAxesCalibration()));
        connect(this, SIGNAL(gimbalAxisCalibrationStatus(bool,bool,bool)), &calibrateAxesDialog, SLOT(receiveAxisCalibrationStatus(bool,bool,bool)));
        connect(&calibrateAxesDialog, SIGNAL(requestStartAxisCalibration()), this, SIGNAL(requestCalibrateAxesPerform()));

        chooseAxesDialog.exec();

        calibrateAxesDialog.exec();
    }
}

void MainWindow::debugFirmwareLoadError(QString errorMsg)
{
    qDebug("Firmware Load Error: %s", errorMsg);
}

void MainWindow::debugFirmwareLoadProgress(double progress)
{
    qDebug("Firmware Load Progress: %f%%", progress);
}

void MainWindow::on_resetGimbalButton_clicked()
{
    emit requestResetGimbal();
    CalibrateAxesDialog calibrateAxesDialog;
    connect(this, SIGNAL(axisCalibrationStarted(int)), &calibrateAxesDialog, SLOT(axisCalibrationStarted(int)));
    connect(this, SIGNAL(axisCalibrationFinished(int,bool)), &calibrateAxesDialog, SLOT(axisCalibrationFinished(int,bool)));
    connect(&calibrateAxesDialog, SIGNAL(retryAxesCalibration()), this, SIGNAL(retryAxesCalibration()));
    calibrateAxesDialog.exec();
}

void MainWindow::on_setHomePositionsButton_clicked()
{
    QMessageBox msg;
    msg.setText("Confirm Home Offset Calibration");
    msg.setInformativeText("This will set the gimbal's current position to its new \"home\" position.  Make sure that the gimbal is immobilized in an appropriate fixture before proceding.  Do you want to proceed?");
    msg.setIcon(QMessageBox::Information);
    msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg.setDefaultButton(QMessageBox::No);
    if (msg.exec() == QMessageBox::Yes) {
        HomeOffsetCalibrationResultDialog dialog;
        connect(this, SIGNAL(sendNewHomeOffsets(int,int,int)), &dialog, SLOT(receiveNewHomeOffsets(int,int,int)));
        emit requestHomeOffsetCalibration();
        dialog.exec();
    }
}

void MainWindow::receiveHomeOffsetCalibrationStatus(bool successful)
{
    qDebug("Home offset calibration finished with result: %d", successful);
}

void MainWindow::receiveFactoryParameters(QString assemblyDateTime, QString serialNumber)
{
    ui->assemblyDateTime->setText(assemblyDateTime);
    m_serialNum = serialNumber;
    ui->serialNumber->setText(serialNumber);
}

void MainWindow::on_setUnitParametersButton_clicked()
{
    EnterFactoryParametersDialog dialog;
    connect(&dialog, SIGNAL(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)), this, SIGNAL(setGimbalFactoryParameters(unsigned short,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,ulong,ulong,ulong)));
    connect(this, SIGNAL(factoryParametersLoaded()), &dialog, SLOT(factoryParametersLoaded()));
    dialog.exec();//todo
    // After the user sets the new factory parameters, request the new parameters from the gimbal to update the UI
    emit requestFactoryParameters();
}

void MainWindow::on_eraseGimbalFlashButton_clicked()
{
    QMessageBox msg;
    msg.setText("Confirm flash erase");
    msg.setInformativeText("WARNING: This will erase gimbal flash, including firmware and calibration values.  This will render the gimbal inoperable until new firmware is loaded, and will also require the gimbal to re-calibrate itself after new firmware is loaded.  Are you sure you want to proceed?");
    msg.setIcon(QMessageBox::Warning);
    msg.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msg.setDefaultButton(QMessageBox::No);
    if (msg.exec() == QMessageBox::Yes) {
        emit requestEraseGimbalFlash();

        // Disable the load firmware and calibration controls
        setUIGimbalDisconnected();

        // Change connection status to disconnected
        ui->connectionStatus->setText("Erasing flash...");

        // Clear out firmware version
        ui->firmwareLoaded->setText("None");

        // Clear out assembly date/time and serial number
        ui->assemblyDateTime->setText("None");
        ui->serialNumber->setText("None");
    }
}

void MainWindow::on_factoryTestsButton_clicked()
{
    AxisRangeTestDialog dialog;
    connect(this, SIGNAL(factoryTestsStatus(int,int,int,int)), &dialog, SLOT(receiveTestProgress(int,int,int,int)));
    connect(&dialog, SIGNAL(requestTestRetry(unsigned char, unsigned char)), this, SIGNAL(requestStartFactoryTests(unsigned char, unsigned char)));
    connect(this, SIGNAL(receivedTestStatus(unsigned char,float)), &dialog, SLOT(receiveTestStatus(unsigned char,float)));
    emit requestStartFactoryTests(TEST_AXIS_RANGE_LIMITS, 0);
    dialog.exec();
}

void MainWindow::on_runWobbleTestButton_clicked()
{
    WobbleTestDialog dialog(m_serialNum);

    connect(this, SIGNAL(receivedGimbalReport(float, float, float)), &dialog, SLOT(receivedGimbalReport(float, float, float)));
    dialog.exec();
}


void MainWindow::on_showHideGimbalMessagesButton_clicked()
{
    if (m_gimbalMessagesVisible) {
        ui->gimbalMessagesConsole->setVisible(false);
        this->resize(ui->connectionInfoFrame->width() - 500, this->height());
        ui->showHideGimbalMessagesButton->setText("Show Gimbal Messages");
        m_gimbalMessagesVisible = false;
    } else {
        ui->gimbalMessagesConsole->setVisible(true);
        this->resize(ui->connectionInfoFrame->width() + 500, this->height());
        ui->showHideGimbalMessagesButton->setText("Hide Gimbal Messages");
        m_gimbalMessagesVisible = true;
    }
}

void MainWindow::on_startGyroHealthTestButton_clicked()
{
    emit requestStartFactoryTests(TEST_GYRO_HEALTH, START_TEST);
}

void MainWindow::on_stopGyroHealthTestButton_clicked()
{
    emit requestStartFactoryTests(TEST_GYRO_HEALTH, STOP_TEST);
}

void MainWindow::gimbalRequiresCalibration(bool requiresCalibration)
{
    // If the gimbal does not require calibration, enable the entire UI
    // Else, the UI has already been put into the connected but needs calibration state,
    // so don't do anything
    if (!requiresCalibration) {
        setUIGimbalConnected();
    }
}

void MainWindow::setUIGimbalConnectedNoFirmware()
{
    // Enable only the load firmware controls
    ui->loadFirmwareButton->setEnabled(true);
    ui->firmwareImage->setEnabled(true);
    ui->firmwareImageBrowseButton->setEnabled(true);
}

void MainWindow::setUIGimbalConnectedNeedsCalibration()
{
    // Enable the load firmware and calibration controls
    ui->loadFirmwareButton->setEnabled(true);
    ui->firmwareImage->setEnabled(true);
    ui->firmwareImageBrowseButton->setEnabled(true);
    ui->runAxisCalibrationButton->setEnabled(true);
    ui->runWobbleTestButton->setEnabled(true);
    ui->setHomePositionsButton->setEnabled(true);
    ui->setUnitParametersButton->setEnabled(true);
    ui->eraseGimbalFlashButton->setEnabled(true);
}

void MainWindow::setUIGimbalConnected()
{
    // Enable the load firmware, calibration, and factory tests controls
    ui->loadFirmwareButton->setEnabled(true);
    ui->firmwareImage->setEnabled(true);
    ui->firmwareImageBrowseButton->setEnabled(true);
    ui->runAxisCalibrationButton->setEnabled(true);
    ui->runWobbleTestButton->setEnabled(true);
    ui->setHomePositionsButton->setEnabled(true);
    ui->setUnitParametersButton->setEnabled(true);
    ui->eraseGimbalFlashButton->setEnabled(true);
    ui->factoryTestsButton->setEnabled(true);
    ui->startGyroHealthTestButton->setEnabled(true);
    ui->stopGyroHealthTestButton->setEnabled(true);
}

void MainWindow::setUIGimbalDisconnected()
{
    // Disable all controls except connection settings
    ui->loadFirmwareButton->setEnabled(false);
    ui->firmwareImage->setEnabled(false);
    ui->firmwareImageBrowseButton->setEnabled(false);
    ui->runAxisCalibrationButton->setEnabled(false);
    ui->runWobbleTestButton->setEnabled(false);
    ui->setHomePositionsButton->setEnabled(false);
    ui->setUnitParametersButton->setEnabled(false);
    ui->eraseGimbalFlashButton->setEnabled(false);
    ui->factoryTestsButton->setEnabled(false);
    ui->startGyroHealthTestButton->setEnabled(false);
    ui->stopGyroHealthTestButton->setEnabled(false);
}

void MainWindow::closeEvent(QCloseEvent *)
{
    // Ask the serial interface thread to stop
    emit closeSerialInterface();
}

