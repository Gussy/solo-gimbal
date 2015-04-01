#include "axis_calibration_status_dialog.h"
#include "ui_axis_calibration_status_dialog.h"

#include <QMessageBox>

AxisCalibrationStatusDialog::AxisCalibrationStatusDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AxisCalibrationStatusDialog),
    m_calibrationStatusReceived(false)
{
    ui->setupUi(this);

    // Disable all of the title bar buttons (so the user can't close the dialog from the title bar)
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
}

AxisCalibrationStatusDialog::~AxisCalibrationStatusDialog()
{
    delete ui;
}

void AxisCalibrationStatusDialog::receiveCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration)
{
    // The gimbal repeats the calibration status message every second, so there's a possibility this slot could get
    // called multiple times.  Only execute the body once
    if (!m_calibrationStatusReceived) {
        m_calibrationStatusReceived = true;

        bool needsCalibration = false;

        if (yawNeedsCalibration) {
            ui->yawCalibrationStatus->setText("Calibration Required");
            needsCalibration = true;
        } else {
            ui->yawCalibrationStatus->setText("Calibrated");
        }

        if (pitchNeedsCalibration) {
            ui->pitchCalibrationStatus->setText("Calibration Required");
            needsCalibration = true;
        } else {
            ui->pitchCalibrationStatus->setText("Calibrated");
        }

        if (rollNeedsCalibration) {
            ui->rollCalibrationStatus->setText("Calibration Required");
            needsCalibration = true;
        } else {
            ui->rollCalibrationStatus->setText("Calibrated");
        }

        ui->axisCalibrationStatusProgress_label->setText("Waiting for Axis Calibration Status...done");

        if (needsCalibration) {
            emit gimbalRequiresCalibration(true);

            QMessageBox msg;
            msg.setText("Axes Require Calibration");
            msg.setInformativeText("One or more of the axes on this gimbal require calibration before the gimbal can be used.  Some functions of this application will be disabled until all gimbal axes are properly calibrated");
            msg.setStandardButtons(QMessageBox::Ok);
            msg.setDefaultButton(QMessageBox::Ok);
            msg.setIcon(QMessageBox::Information);
            msg.exec();
        } else {
            emit gimbalRequiresCalibration(false);
        }

        ui->buttonBox->setEnabled(true);
    }
}

/*
void AxisCalibrationStatusDialog::reject()
{
    // Overriding this to prevent default behavior of escape key causing dialog to exit
}
*/
