#ifndef AXIS_RANGE_TEST_DIALOG_H
#define AXIS_RANGE_TEST_DIALOG_H

#include "serial_interface_thread.h"

#include <QDialog>
#include <QPixmap>
#include <QLabel>

namespace Ui {
class AxisRangeTestDialog;
}

class AxisRangeTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AxisRangeTestDialog(QWidget *parent = 0);
    ~AxisRangeTestDialog();

public slots:
    void receiveTestProgress(int, int test_section, int test_progress, int test_status);
    void receiveTestStatus(unsigned char result_id, float result);
    void reject();

signals:
    void requestTestRetry(unsigned char test_id, unsigned char test_arg);

private:
    Ui::AxisRangeTestDialog *ui;

    QPixmap m_inProgressIcon;
    QPixmap m_successIcon;
    QPixmap m_failureIcon;

    QString m_pitchMechRange;
    QString m_pitchMechSymm;
    QString m_pitchMaxNegTorq;
    QString m_pitchMaxNegTorqLoc;
    QString m_pitchAvgNegTorq;
    QString m_pitchMaxPosTorq;
    QString m_pitchMaxPosTorqLoc;
    QString m_pitchAvgPosTorq;

    QString m_rollMechRange;
    QString m_rollMechSymm;
    QString m_rollMaxNegTorq;
    QString m_rollMaxNegTorqLoc;
    QString m_rollAvgNegTorq;
    QString m_rollMaxPosTorq;
    QString m_rollMaxPosTorqLoc;
    QString m_rollAvgPosTorq;

    QString m_yawMechRange;
    QString m_yawMechSymm;
    QString m_yawMaxNegTorq;
    QString m_yawMaxNegTorqLoc;
    QString m_yawAvgNegTorq;
    QString m_yawMaxPosTorq;
    QString m_yawMaxPosTorqLoc;
    QString m_yawAvgPosTorq;


    const double ENCODER_COUNTS_PER_DEG;
    const double TORQUE_HALF_SCALE;
    const double MAX_CURRENT_HALF_SCALE;
    const double MOTOR_NM_PER_A;
    const double OZ_IN_PER_NM;
    const QString TAB = \t;


    void setStepStatus(QLabel *statusLabel, int status);
    void resetTestUI();
    double gimbalTorqueToOzIn(double gimbalTorque);

private slots:
    void on_retryButton_clicked();
    void on_okButton_clicked();
};

#endif // AXIS_RANGE_TEST_DIALOG_H
