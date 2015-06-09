#ifndef WOBBLE_TEST_DIALOG_H
#define WOBBLE_TEST_DIALOG_H

#include "serial_interface_thread.h"

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

    QString m_xDelta;
    QString m_yDelta;
    QString m_zDelta;

private slots:
    void on_okButton_clicked();
};

#endif // WOBBLE_TEST_DIALOG_H
