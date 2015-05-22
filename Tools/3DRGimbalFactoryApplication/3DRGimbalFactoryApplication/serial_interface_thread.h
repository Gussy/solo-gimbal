#ifndef SERIAL_INTERFACE_THREAD_H
#define SERIAL_INTERFACE_THREAD_H

#include "MAVLink/ardupilotmega/mavlink.h"

#include <QObject>
#include <QSerialPort>
#include <QByteArray>
#include <QTimer>
#include <cstdint>

#define MAVLINK_GIMBAL_SYSTEM_ID 50

#define FACTORY_PARAM_CHECK_MAGIC_1 189496049
#define FACTORY_PARAM_CHECK_MAGIC_2 775861598
#define FACTORY_PARAM_CHECK_MAGIC_3 950903575
#define GIMBAL_FIRMWARE_ERASE_KNOCK 484383340

enum GimbalInterfaceState {
    INTERFACE_INITIALIZED,
    INTERFACE_INDICATED_CODE_LOADED,
    INTERFACE_INDICATED_NO_CODE_LOADED,
    INTERFACE_LOADING_CODE,
    INTERFACE_CALIBRATING_GIMBAL_EL,
    INTERFACE_CALIBRATING_GIMBAL_RL,
    INTERFACE_CALIBRATING_GIMBAL_AZ
};

enum TestType {
    TEST_AXIS_RANGE_LIMITS = 0,
    TEST_GYRO_HEALTH
};

enum TestArg {
    START_TEST = 0,
    STOP_TEST
};

enum  AxisRangeLimitsTestStatus {
    AXIS_RANGE_TEST_STATUS_IN_PROGRESS = 0,
    AXIS_RANGE_TEST_STATUS_SUCCEEDED,
    AXIS_RANGE_TEST_STATUS_FAILED_NEGATIVE,
    AXIS_RANGE_TEST_STATUS_FAILED_POSITIVE,
    AXIS_RANGE_TEST_STATUS_FAILED_HOME
};

enum  AxisRangeLimitsTestSection {
    AXIS_RANGE_TEST_SECTION_EL_CHECK_NEG = 0,
    AXIS_RANGE_TEST_SECTION_EL_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_EL_RETURN_HOME,
    AXIS_RANGE_TEST_SECTION_RL_CHECK_NEG,
    AXIS_RANGE_TEST_SECTION_RL_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_RL_RETURN_HOME,
    AXIS_RANGE_TEST_SECTION_AZ_CHECK_NEG,
    AXIS_RANGE_TEST_SECTION_AZ_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_AZ_RETURN_HOME
};

enum TestResult {
    TEST_RESULT_NEG_RANGE_AZ = 0,
    TEST_RESULT_POS_RANGE_AZ,
    TEST_RESULT_NEG_MAX_TORQUE_AZ,
    TEST_RESULT_POS_MAX_TORQUE_AZ,
    TEST_RESULT_NEG_RANGE_EL,
    TEST_RESULT_POS_RANGE_EL,
    TEST_RESULT_NEG_MAX_TORQUE_EL,
    TEST_RESULT_POS_MAX_TORQUE_EL,
    TEST_RESULT_NEG_RANGE_RL,
    TEST_RESULT_POS_RANGE_RL,
    TEST_RESULT_NEG_MAX_TORQUE_RL,
    TEST_RESULT_POS_MAX_TORQUE_RL
};

typedef union {
    uint32_t uint32_value;
    float float_value;
} IntOrFloat;

class SerialInterfaceThread : public QObject
{
    Q_OBJECT
public:
    explicit SerialInterfaceThread(QString serialPortName, int serialPortBaudRate, QObject *parent = 0);
    virtual ~SerialInterfaceThread();

public slots:
    void run();
    void handleInput();
    void requestStop();
    void requestLoadFirmware(QString firmwareImageFileName);
    void requestCalibrateAxesSetup(bool calibrateYaw, bool calibratePitch, bool calibrateRoll);
    void requestCalibrateAxesPerform();
    void requestResetGimbal();
    void requestFirmwareVersion();
    void retryAxesCalibration();
    void requestCalibrateHomeOffsets();
    void setGimbalFactoryParameters(unsigned short assyYear,
                                    unsigned char assyMonth,
                                    unsigned char assyDay,
                                    unsigned char assyHour,
                                    unsigned char assyMinute,
                                    unsigned char assySecond,
                                    unsigned long serialNumber1,
                                    unsigned long serialNumber2,
                                    unsigned long serialNumber3);
    void requestFactoryParameters();
    void requestGimbalEraseFlash();
    void requestGimbalFactoryTests(unsigned char test_id, unsigned char test_arg);
    void requestCalibrationParameters();
    void requestAxisCalibrationStatus();

signals:
    void receivedHeartbeat();
    void receivedDataTransmissionHandshake();
    void firmwareLoadError(QString errorMsg);
    void firmwareLoadProgress(double progress);
    void axisCalibrationStarted(int axis);
    void axisCalibrationFinished(int axis, bool successful);
    void sendFirmwareVersion(QString versionString);
    void serialPortOpenError(QString errorMsg);
    void homeOffsetCalibrationFinished(bool successful);
    void sendNewHomeOffsets(int yawOffset, int pitchOffset, int rollOffset);
    void sendFactoryParameters(QString assyDateTime, QString serialNumber);
    void factoryParametersLoaded();
    void factoryTestsStatus(int test, int test_section, int test_progress, int test_status);
    void gimbalAxisCalibrationStatus(bool yawAxisNeedsCalibration, bool pitchAxisNeedsCalibration, bool rollAxisNeedsCalibration);
    void gimbalStatusMessage(unsigned int severity, QString message);
    void receivedTestStatus(unsigned char result_id, float result);

private:
    QSerialPort* m_serialPort;
    QString m_serialPortName;
    QTimer* m_periodicRateCmdsTimer;
    mavlink_message_t m_zeroRateCmd;
    int m_serialPortBaudRate;
    GimbalInterfaceState m_interfaceState;
    bool m_stopRequested;
    uint16_t* m_firmwareImageData;
    int m_firmwareImageDataSize;
    char m_messageBuffer[MAVLINK_MAX_PACKET_LEN];
    int m_lastYawHomeOffset;
    int m_lastPitchHomeOffset;
    int m_lastRollHomeOffset;
    uint32_t m_lastAssyDate;
    uint32_t m_lastAssyTime;
    uint32_t m_lastSerialNumber1;
    uint32_t m_lastSerialNumber2;
    uint32_t m_lastSerialNumber3;

    void sendMavlinkMessage(mavlink_message_t* msg);
    void handleDataTransmissionHandshake(mavlink_message_t* msg);
    void sendFirmwareImageBlock(int seqNum, int packetSize);
    void handleGimbalAxisCalibrationProgress(mavlink_message_t* msg);
    void mavlinkSetParam(const char* param_name, float param_value);
    void resetGimbal();
    void handleParamValue(mavlink_message_t* msg);
    void handleStatusText(mavlink_message_t* msg);
    void handleHomeOffsetCalibrationResult(mavlink_message_t* msg);
    void handleFactoryTestsProgress(mavlink_message_t* msg);
    void handleReportAxisCalibrationStatus(mavlink_message_t* msg);
    void handleDebug(mavlink_message_t* msg);
    void sendLoadFirmwareStart(int numPackets);
    void sendCalibrateHomeOffsets();
    void sendParamRequest(QString paramName);

private slots:
    void sendPeriodicRateCmd();
};

#endif // SERIAL_INTERFACE_THREAD_H
