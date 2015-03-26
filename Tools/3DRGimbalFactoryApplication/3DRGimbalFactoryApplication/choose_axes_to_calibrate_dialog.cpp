#include "choose_axes_to_calibrate_dialog.h"
#include "ui_choose_axes_to_calibrate_dialog.h"

#include <QMessageBox>

ChooseAxesToCalibrateDialog::ChooseAxesToCalibrateDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ChooseAxesToCalibrateDialog),
    m_axisCalibrationStatusReceived(false),
    m_calibrationSetupCmdSent(false)
{
    ui->setupUi(this);

    // Disable all of the title bar buttons (so the user can't close the dialog from the title bar)
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);

    // Set setting up calibration and continue messages to invisible to start
    ui->settingUpCalibration_label->setVisible(false);
    ui->continueMessage_label->setVisible(false);
}

ChooseAxesToCalibrateDialog::~ChooseAxesToCalibrateDialog()
{
    delete ui;
}

void ChooseAxesToCalibrateDialog::receiveGimbalAxisCalibrationStatus(bool yawNeedsCalibration, bool pitchNeedsCalibration, bool rollNeedsCalibration)
{
    // We only want to react to this message after we've sent the command to set up calibration
    if (m_calibrationSetupCmdSent) {
        // It's possible to receive this message more than once, so only run this the first time
        if (!m_axisCalibrationStatusReceived) {
            m_axisCalibrationStatusReceived = true;
            // Once we've received the calibration status, we know the gimbal has rebooted and is ready to calibrate
            ui->settingUpCalibration_label->setText("Setting up calibration...done");
            ui->continueMessage_label->setVisible(true);
            ui->performCalibrationButton->setEnabled(true);
        }
    }
}

void ChooseAxesToCalibrateDialog::on_setupCalibrationButton_clicked()
{
    // Make sure at least one axis is selected
    if (!(ui->calibrateYaw->isChecked() || ui->calibratePitch->isChecked() || ui->calibrateRoll->isChecked())) {
        QMessageBox msg;
        msg.setText("No axes selected");
        msg.setInformativeText("At least one axis must be selected to calibrate");
        msg.setStandardButtons(QMessageBox::Ok);
        msg.setDefaultButton(QMessageBox::Ok);
        msg.setIcon(QMessageBox::Information);
        msg.exec();
    } else {
        emit requestAxisCalibrationSetup(ui->calibrateYaw->isChecked(), ui->calibratePitch->isChecked(), ui->calibrateRoll->isChecked());
        ui->setupCalibrationButton->setEnabled(false);
        ui->settingUpCalibration_label->setVisible(true);
        m_calibrationSetupCmdSent = true;
    }
}

void ChooseAxesToCalibrateDialog::on_performCalibrationButton_clicked()
{
    emit requestAxisCalibrationStart();
    this->accept();
}

void ChooseAxesToCalibrateDialog::reject()
{
    // Overriding this to prevent default behavior of escape key causing dialog to exit
}
