#include "wobble_test_dialog.h"
#include "ui_wobble_test_dialog.h"

#include "serial_interface_thread.h"

WobbleTestDialog::WobbleTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WobbleTestDialog)
{
    // Disable all of the title bar buttons (so the user can't close the dialog from the title bar)
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
    ui->setupUi(this);
}

WobbleTestDialog::~WobbleTestDialog()
{
    delete ui;
}

void WobbleTestDialog::receivedGimbalReport(float deltaX, float deltaY, float deltaZ)
{
    m_xDelta = QString::number(deltaX);
    ui->xDelta->setText(m_xDelta);//TODO

    m_yDelta = QString::number(deltaY);
    ui->yDelta->setText(m_yDelta);

    m_zDelta = QString::number(deltaZ);
    ui->zDelta->setText(m_zDelta);

}

void WobbleTestDialog::on_okButton_clicked()
{
    accept();
}



