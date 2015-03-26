#ifndef CALIBRATE_AXES_DIALOG_H
#define CALIBRATE_AXES_DIALOG_H

#include <QDialog>

#include "MAVLink/ardupilotmega/mavlink.h"

namespace Ui {
class CalibrateAxesDialog;
}

class CalibrateAxesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibrateAxesDialog(QWidget *parent = 0);
    ~CalibrateAxesDialog();

public slots:
    void axisCalibrationStarted(int axis);
    void axisCalibrationFinished(int axis, bool successful);
    void receiveAxisCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration);
    void reject();

signals:
    void retryAxesCalibration();
    void requestStartAxisCalibration();

private:
    Ui::CalibrateAxesDialog *ui;
    bool m_axisCalibrationStatusReceived;

    void resetForCalibrationRetry();

private slots:
    void on_cancelButton_clicked();
};

#endif // CALIBRATE_AXES_DIALOG_H
