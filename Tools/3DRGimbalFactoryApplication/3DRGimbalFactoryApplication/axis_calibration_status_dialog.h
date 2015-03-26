#ifndef AXIS_CALIBRATION_STATUS_DIALOG_H
#define AXIS_CALIBRATION_STATUS_DIALOG_H

#include <QDialog>

namespace Ui {
class AxisCalibrationStatusDialog;
}

class AxisCalibrationStatusDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AxisCalibrationStatusDialog(QWidget *parent = 0);
    ~AxisCalibrationStatusDialog();

public slots:
    void receiveCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration);
    void reject();

signals:
    void gimbalRequiresCalibration(bool requiresCalibration);

private:
    Ui::AxisCalibrationStatusDialog *ui;

    bool m_calibrationStatusReceived;
};

#endif // AXIS_CALIBRATION_STATUS_DIALOG_H
