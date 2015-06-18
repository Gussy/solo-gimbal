#ifndef WOBBLE_TEST_DIALOG_H
#define WOBBLE_TEST_DIALOG_H

#include "serial_interface_thread.h"
#include "qcustomplot.h"
#include "mainwindow.h"


#include <QDialog>
#include <QPixmap>
#include <QLabel>
#include <QTime>


namespace Ui {
class WobbleTestDialog;
}

class WobbleTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WobbleTestDialog(const QString serNum, const QString comPort, QWidget *parent = 0);
    ~WobbleTestDialog();


public slots:
    void receivedGimbalReport(float, float, float);


private:
    Ui::WobbleTestDialog *ui;

    const int COUNTS;
    const int RANGE;
    const QString GIM_SERIAL_NUM;
    const QString COMM_PORT;
    const QString MAX_ALLOWED_ALL_AXES;

    bool m_pause;
    QElapsedTimer m_elapsedTime;
    double m_timerTemp;
    QTime m_totalElapsed;

    QString m_xMaxGyroAllowed;
    QString m_xGyro;
    QString m_xMaxGyro;
    QString m_xMaxGyroTime;
    QString m_xMinGyro;
    QString m_xNumFails;
    bool m_xFailOccurred;
    int m_xCount;
//    const float X_MAX_GYRO_ALLOWED;

    QString m_yMaxGyroAllowed;
    QString m_yGyro;
    QString m_yMaxGyro;
    QString m_yMaxGyroTime;
    QString m_yMinGyro;
    QString m_yNumFails;
    bool m_yFailOccurred;
    int m_yCount;
//    const float Y_MAX_GYRO_ALLOWED;

    QString m_zMaxGyroAllowed;
    QString m_zGyro;
    QString m_zMaxGyro;
    QString m_zMaxGyroTime;
    QString m_zMinGyro;
    QString m_zNumFails;
    bool m_zFailOccurred;
    int m_zCount;
//    const float Z_MAX_GYRO_ALLOWED;


    void setupPlot(QCustomPlot *customPlot);

private slots:
    void on_closeButton_clicked();
    void on_pauseButton_clicked();
    void on_resumeButton_clicked();
    void on_refreshSetpointsButton_clicked();
};

#endif // WOBBLE_TEST_DIALOG_H
