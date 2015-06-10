#ifndef WOBBLE_TEST_DIALOG_H
#define WOBBLE_TEST_DIALOG_H

#include "serial_interface_thread.h"
#include "qcustomplot.h"

#include <QDialog>
#include <QPixmap>
#include <QLabel>



namespace Ui {
class WobbleTestDialog;
}

class WobbleTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit WobbleTestDialog(QWidget *parent = 0);
    ~WobbleTestDialog();


public slots:
    void receivedGimbalReport(float, float, float);


private:
    Ui::WobbleTestDialog *ui;

    const int COUNTS;
    const int RANGE;

    QString m_xDelta;
    QString m_xMaxDelta;
    QString m_xMaxDeltaTime;
    QString m_xMinDelta;
    QString m_xNumFails;
    bool m_xFailOccurred;
    int m_xCount;
    const float X_MAX_DELTA_ALLOWED;

    QString m_yDelta;
    QString m_yMaxDelta;
    QString m_yMaxDeltaTime;
    QString m_yMinDelta;
    QString m_yNumFails;
    bool m_yFailOccurred;
    int m_yCount;
    const float Y_MAX_DELTA_ALLOWED;

    QString m_zDelta;
    QString m_zMaxDelta;
    QString m_zMaxDeltaTime;
    QString m_zMinDelta;
    QString m_zNumFails;
    bool m_zFailOccurred;
    int m_zCount;
    const float Z_MAX_DELTA_ALLOWED;

    void setupPlot(QCustomPlot *customPlot);

private slots:
    void on_closeButton_clicked();
};

#endif // WOBBLE_TEST_DIALOG_H
