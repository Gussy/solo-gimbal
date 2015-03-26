#ifndef CHOOSE_AXES_TO_CALIBRATE_DIALOG_H
#define CHOOSE_AXES_TO_CALIBRATE_DIALOG_H

#include <QDialog>

namespace Ui {
class ChooseAxesToCalibrateDialog;
}

class ChooseAxesToCalibrateDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ChooseAxesToCalibrateDialog(QWidget *parent = 0);
    ~ChooseAxesToCalibrateDialog();

public slots:
    void receiveGimbalAxisCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration);
    void reject();

signals:
    void requestAxisCalibrationSetup(bool calibrateYaw, bool calibratePitch, bool calibrateRoll);
    void requestAxisCalibrationStart();

private:
    Ui::ChooseAxesToCalibrateDialog *ui;

    bool m_axisCalibrationStatusReceived;
    bool m_calibrationSetupCmdSent;

private slots:
    void on_setupCalibrationButton_clicked();
    void on_performCalibrationButton_clicked();
};

#endif // CHOOSE_AXES_TO_CALIBRATE_DIALOG_H
