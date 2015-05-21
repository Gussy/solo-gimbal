#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "serial_interface_thread.h"

#include "MAVLink/ardupilotmega/mavlink.h"

#include <QMainWindow>
#include <QThread>
#include <QSerialPort>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void receivedGimbalHeartbeat();
    void receivedGimbalDataTransmissionHandshake();
    void debugFirmwareLoadError(QString errorMsg);
    void debugFirmwareLoadProgress(double progress);
    void receiveFirmwareVersion(QString versionString);
    void receiveSerialPortOpenError(QString errorMsg);
    void receiveHomeOffsetCalibrationStatus(bool successful);
    void receiveFactoryParameters(QString assemblyDateTime, QString serialNumber);
    void receiveGimbalStatusMessage(unsigned int severity, QString message);
    void closeEvent(QCloseEvent *);

signals:
    void closeSerialInterface();
    void requestFirmwareDownload(QString firmwareFileName);
    void firmwareLoadError(QString errorMsg);
    void firmwareLoadProgress(double progress);
    void axisCalibrationStarted(int axis);
    void axisCalibrationFinished(int axis, bool successful);
    void requestFirmwareVersion();
    void requestCalibrateAxesSetup(bool calibrateYaw, bool calibratePitch, bool calibrateRoll);
    void requestCalibrateAxesPerform();
    void retryAxesCalibration();
    void requestResetGimbal();
    void requestHomeOffsetCalibration();
    void sendNewHomeOffsets(int yawOffset, int pitchOffset, int rollOffset);
    void requestFactoryParameters();
    void setGimbalFactoryParameters(unsigned short assyYear,
                                    unsigned char assyMonth,
                                    unsigned char assyDay,
                                    unsigned char assyHour,
                                    unsigned char assyMinute,
                                    unsigned char assySecond,
                                    unsigned long serialNumber1,
                                    unsigned long serialNumber2,
                                    unsigned long serialNumber3);
    void factoryParametersLoaded();
    void requestEraseGimbalFlash();
    void requestStartFactoryTests(unsigned char test_id, unsigned char test_arg);
    void factoryTestsStatus(int test, int test_section, int test_progress, int test_status);
    void gimbalAxisCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration);
    void requestAxisCalibrationStatus();
    void receivedTestStatus(unsigned char result_id, float result);

private:
    Ui::MainWindow *ui;

    QThread m_serialThread;
    SerialInterfaceThread* m_serialThreadObj;
    QTimer m_connectionTimeoutTimer;
    bool m_gimbalMessagesVisible;

    void setUIGimbalConnectedNoFirmware();
    void setUIGimbalConnectedNeedsCalibration();
    void setUIGimbalConnected();
    void setUIGimbalDisconnected();

private slots:
    void connectionTimeout();
    void gimbalRequiresCalibration(bool requiresCalibration);

    void on_connectButton_clicked();
    void on_disconnectButton_clicked();
    void on_firmwareImageBrowseButton_clicked();
    void on_loadFirmwareButton_clicked();
    void on_runAxisCalibrationButton_clicked();
    void on_resetGimbalButton_clicked();
    void on_setHomePositionsButton_clicked();
    void on_setUnitParametersButton_clicked();
    void on_eraseGimbalFlashButton_clicked();
    void on_factoryTestsButton_clicked();
    void on_showHideGimbalMessagesButton_clicked();
};

#endif // MAINWINDOW_H
