#include "wobble_test_dialog.h"
#include "ui_wobble_test_dialog.h"

#include "serial_interface_thread.h"

WobbleTestDialog::WobbleTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WobbleTestDialog),
    X_MAX_DELTA_ALLOWED(0.01),
    Y_MAX_DELTA_ALLOWED(0.01),
    Z_MAX_DELTA_ALLOWED(0.01),
    COUNTS(10)
{
    // Disable all of the title bar buttons (so the user can't close the dialog from the title bar)
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
    ui->setupUi(this);
    setupPlot(ui->customPlot);
    setupPlot(ui->customPlot_2);
    setupPlot(ui->customPlot_3);
    m_xCount = COUNTS;
    m_yCount = COUNTS;
    m_zCount = COUNTS;
}

WobbleTestDialog::~WobbleTestDialog()
{
    delete ui;
}

void WobbleTestDialog::receivedGimbalReport(float deltaX, float deltaY, float deltaZ)
{
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    if(deltaX > X_MAX_DELTA_ALLOWED || m_xCount == COUNTS){
        if(m_xCount == 10){
            m_xCount = 0;
        }
        //deal with logging and replotting for x axis
        m_xDelta = QString::number(deltaX);
        ui->xDelta->setText(m_xDelta);

        if(deltaX > X_MAX_DELTA_ALLOWED){
            m_xNumFails = QString::number(m_xNumFails.toInt() + 1);
            m_xFailOccurred = true;//TODO, make background red if fail occurred
            ui->customPlot->setBackground(Qt::red);
            ui->customPlot->axisRect()->setBackground(Qt::white);
            ui->xNumFails->setText(m_xNumFails);
            m_xCount = 0;
        }
        else{
            if(m_xFailOccurred){
                ui->customPlot->setBackground(Qt::yellow);
            }
            else{
                ui->customPlot->setBackground(Qt::white);
            }
        }

        if(deltaX > m_xMaxDelta.toDouble()){
            m_xMaxDelta = m_xDelta;
            m_xMaxDeltaTime = QString::number(key);
            ui->xFailTime->setText(m_xMaxDeltaTime);
        }

        // add data to lines:
        ui->customPlot->graph(0)->addData(key, deltaX);
        // remove data of lines that's outside visible range:
        ui->customPlot->graph(0)->removeDataBefore(key-4);
        // rescale value (vertical) axis to fit the current data:
        ui->customPlot->graph(0)->rescaleValueAxis();
        // make key axis range scroll with the data (at a constant range size of 16):
        ui->customPlot->xAxis->setRange(key+0.25, 4, Qt::AlignRight);
        ui->customPlot->replot();
        m_xCount++;
    }
    else{
        m_xCount++;
    }

    if(deltaY > Y_MAX_DELTA_ALLOWED || m_yCount == COUNTS){
        if(m_yCount == 10){
            m_yCount = 0;
        }
        //deal with logging and replotting for y axis
        m_yDelta = QString::number(deltaY);
        ui->yDelta->setText(m_yDelta);

        if(deltaY > Y_MAX_DELTA_ALLOWED){
            m_yNumFails = QString::number(m_yNumFails.toInt() + 1);
            m_yFailOccurred = true;
            ui->customPlot_2->setBackground(Qt::red);
            ui->customPlot_2->axisRect()->setBackground(Qt::white);
            ui->yNumFails->setText(m_yNumFails);
            m_yCount = 0;
        }
        else{
            if(m_yFailOccurred){
                ui->customPlot_2->setBackground(Qt::yellow);
            }
            else{
                ui->customPlot_2->setBackground(Qt::white);
            }
        }

        if(deltaY > m_yMaxDelta.toDouble()){
            m_yMaxDelta = m_yDelta;
            m_yMaxDeltaTime = QString::number(key);
            ui->yFailTime->setText(m_yMaxDeltaTime);
        }

        // add data to lines:
        ui->customPlot_2->graph(0)->addData(key, deltaY);
        // remove data of lines that's outside visible range:
        ui->customPlot_2->graph(0)->removeDataBefore(key-4);
        // rescale value (vertical) axis to fit the current data:
        ui->customPlot_2->graph(0)->rescaleValueAxis();
        // make key axis range scroll with the data (at a constant range size of 16):
        ui->customPlot_2->xAxis->setRange(key+0.25, 4, Qt::AlignRight);
        ui->customPlot_2->replot();
        m_yCount++;
    }
    else{
        m_yCount++;
    }

    if(deltaZ > Z_MAX_DELTA_ALLOWED || m_zCount == COUNTS){
        if(m_zCount == 10){
            m_zCount = 0;
        }
        //deal with logging and replotting for z axis
        m_zDelta = QString::number(deltaZ);
        ui->zDelta->setText(m_zDelta);

        if(deltaZ > Z_MAX_DELTA_ALLOWED){
            m_zNumFails = QString::number(m_zNumFails.toInt() + 1);
            m_zFailOccurred = true;//TODO, make background red if fail occurred
            ui->customPlot_3->setBackground(Qt::red);
            ui->customPlot_3->axisRect()->setBackground(Qt::white);
            ui->zNumFails->setText(m_zNumFails);
            m_zCount = 0;
        }
        else{
            if(m_zFailOccurred){
                ui->customPlot_3->setBackground(Qt::yellow);
            }
            else{
                ui->customPlot_3->setBackground(Qt::white);
            }
        }

        if(deltaZ > m_zMaxDelta.toDouble()){
            m_zMaxDelta = m_zDelta;
            m_zMaxDeltaTime = QString::number(key);
            ui->zFailTime->setText(m_zMaxDeltaTime);
        }

        // add data to lines:
        ui->customPlot_3->graph(0)->addData(key, deltaZ);
        // remove data of lines that's outside visible range:
        ui->customPlot_3->graph(0)->removeDataBefore(key-4);
        // rescale value (vertical) axis to fit the current data:
        ui->customPlot_3->graph(0)->rescaleValueAxis();
        // make key axis range scroll with the data (at a constant range size of 16):
        ui->customPlot_3->xAxis->setRange(key+0.25, 4, Qt::AlignRight);
        ui->customPlot_3->replot();
        m_zCount++;
    }
    else{
        m_zCount++;
    }

}

void WobbleTestDialog::setupPlot(QCustomPlot *customPlot)
{
#if QT_VERSION < QT_VERSION_CHECK(4, 7, 0)
  QMessageBox::critical(this, "", "You're using Qt < 4.7, the realtime data demo needs functions that are available with Qt 4.7 to work properly");
#endif
  // include this section to fully disable antialiasing for higher performance:

  customPlot->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  customPlot->xAxis->setTickLabelFont(font);
  customPlot->yAxis->setTickLabelFont(font);
  customPlot->legend->setFont(font);
  //

  customPlot->addGraph(); // blue line
  customPlot->graph(0)->setPen(QPen(Qt::blue));
  //customPlot->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
  //customPlot->graph(0)->setAntialiasedFill(false);
//  customPlot_2->addGraph(); // red line
//  customPlot_2->graph(0)->setPen(QPen(Qt::red));
//  customPlot_3->addGraph(); // green line
//  customPlot_3->graph(0)->setPen(QPen(Qt::green));

//  customPlot->addGraph(); // blue dot
//  customPlot->graph(3)->setPen(QPen(Qt::blue));
//  customPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
//  customPlot->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
//  customPlot->addGraph(); // red dot
//  customPlot->graph(4)->setPen(QPen(Qt::red));
//  customPlot->graph(4)->setLineStyle(QCPGraph::lsNone);
//  customPlot->graph(4)->setScatterStyle(QCPScatterStyle::ssDisc);

//  customPlot->addGraph(); // green dot
//  customPlot->graph(5)->setPen(QPen(Qt::green));
//  customPlot->graph(5)->setLineStyle(QCPGraph::lsNone);
//  customPlot->graph(5)->setScatterStyle(QCPScatterStyle::ssDisc);

  customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
  customPlot->xAxis->setAutoTickStep(false);
  customPlot->xAxis->setTickStep(2);
  customPlot->axisRect()->setupFullAxesBox();

//  customPlot_2->xAxis->setTickLabelType(QCPAxis::ltDateTime);
//  customPlot_2->xAxis->setDateTimeFormat("hh:mm:ss");
//  customPlot_2->xAxis->setAutoTickStep(false);
//  customPlot_2->xAxis->setTickStep(2);
//  customPlot_2->axisRect()->setupFullAxesBox();

//  customPlot_3->xAxis->setTickLabelType(QCPAxis::ltDateTime);
//  customPlot_3->xAxis->setDateTimeFormat("hh:mm:ss");
//  customPlot_3->xAxis->setAutoTickStep(false);
//  customPlot_3->xAxis->setTickStep(2);
//  customPlot_3->axisRect()->setupFullAxesBox();


  // make left and bottom axes transfer their ranges to right and top axes:
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

//  connect(customPlot_2->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_2->xAxis2, SLOT(setRange(QCPRange)));
//  connect(customPlot_2->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_2->yAxis2, SLOT(setRange(QCPRange)));

//  connect(customPlot_3->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_3->xAxis2, SLOT(setRange(QCPRange)));
//  connect(customPlot_3->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot_3->yAxis2, SLOT(setRange(QCPRange)));
}

void WobbleTestDialog::on_okButton_clicked()
{
    accept();
}



