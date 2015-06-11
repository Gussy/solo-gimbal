#include "wobble_test_dialog.h"
#include "ui_wobble_test_dialog.h"
#include "mainwindow.h"
#include "serial_interface_thread.h"

#include <QTime>



WobbleTestDialog::WobbleTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WobbleTestDialog),
//    X_MAX_DELTA_ALLOWED(0.01),
//    Y_MAX_DELTA_ALLOWED(0.01),
//    Z_MAX_DELTA_ALLOWED(0.01),
    COUNTS(10),
    RANGE(30)
{
    // Disable all of the title bar buttons (so the user can't close the dialog from the title bar)
    setWindowFlags(Qt::Dialog | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
    ui->setupUi(this);
    setupPlot(ui->customPlot);
    setupPlot(ui->customPlot_2);
    setupPlot(ui->customPlot_3);

    m_xCount = 0;
    m_xFailOccurred = false;
    m_xDelta = "0";
    m_xMaxDelta = "0";
    m_xMaxDeltaTime = "00:00:00";
    m_xMinDelta = "0";
    m_xNumFails = "0";
    m_xMaxDeltaAllowed = "0.001";

    m_yCount = 0;
    m_yFailOccurred = false;
    m_yDelta = "0";
    m_yMaxDelta = "0";
    m_yMaxDeltaTime = "00:00:00";
    m_yMinDelta = "0";
    m_yNumFails = "0";
    m_yMaxDeltaAllowed = "0.001";

    m_zCount = 0;
    m_zFailOccurred = false;
    m_zDelta = "0";
    m_zMaxDelta = "0";
    m_zMaxDeltaTime = "00:00:00";
    m_zMinDelta = "0";
    m_zNumFails = "0";
    m_zMaxDeltaAllowed = "0.001";

    m_pause = false;
    ui->resumeButton->setEnabled(false);
    ui->pauseButton->setEnabled(true);

    m_timerTemp = 0;
}

WobbleTestDialog::~WobbleTestDialog()
{
    delete ui;
}

void WobbleTestDialog::receivedGimbalReport(float deltaX, float deltaY, float deltaZ)
{
    if(m_pause == false)
    {
        QDateTime logTime = QDateTime::currentDateTime();
        double key = logTime.toMSecsSinceEpoch()/1000.0;
        double max = 0;

        /*
         *
         *handle everything for roll
         *
         */
        //adjust watermarks if necessary
        if(deltaX > m_xMaxDelta.toFloat()){
            m_xMaxDelta = QString::number(deltaX);
        }

        if(deltaX < m_xMinDelta.toFloat()){
            m_xMinDelta = QString::number(deltaX);
        }

        max = qMax(qAbs(m_xMaxDelta.toDouble()),qAbs(m_xMinDelta.toDouble()));
        ui->customPlot->yAxis->setRange(1.2 * max, -1.2 * max);

        //plot watermarks
        ui->customPlot->graph(1)->addData(key, m_xMaxDelta.toFloat());
        ui->customPlot->graph(2)->addData(key, m_xMinDelta.toFloat());

        //if we've failed this run or it's been 10 counts since adding point to plot, refresh plot
        if(deltaX > m_xMaxDeltaAllowed.toFloat() || m_xCount == COUNTS){
            if(m_xCount == 10){
                m_xCount = 0;
            }
            //deal with logging and replotting for x axis
            m_xDelta = QString::number(deltaX);
            ui->xDelta->setText(m_xDelta);

            if(deltaX > m_xMaxDeltaAllowed.toFloat()){
                m_xNumFails = QString::number(m_xNumFails.toInt() + 1);
                m_xFailOccurred = true;
                ui->customPlot->setBackground(Qt::red);
                ui->customPlot->axisRect()->setBackground(Qt::white);
                ui->xNumFails->setText(m_xNumFails);
                m_xCount = 0;
                m_xMaxDeltaTime = logTime.toString("hh:mm:ss.zzz");
                ui->xFailTime->setText(m_xMaxDeltaTime);
            }
            else{
                if(m_xFailOccurred){
                    ui->customPlot->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot->setBackground(Qt::green);
                }
            }

            if(deltaX > m_xMaxDelta.toFloat()){
                m_xMaxDelta = m_xDelta;
            }

            // add data to lines:
            ui->customPlot->graph(0)->addData(key, deltaX);
            // remove data of lines that's outside visible range:
            ui->customPlot->graph(0)->removeDataBefore(key-RANGE);
            ui->customPlot->graph(1)->removeDataBefore(key-RANGE);
            ui->customPlot->graph(2)->removeDataBefore(key-RANGE);
            // rescale value (vertical) axis to fit the current data:
//            ui->customPlot->graph(0)->rescaleValueAxis(true);
//            ui->customPlot->graph(1)->rescaleValueAxis(true);
//            ui->customPlot->graph(2)->rescaleValueAxis(true);
            // make key axis range scroll with the data (at a constant range size of 16):
            ui->customPlot->xAxis->setRange(key+0.25, RANGE, Qt::AlignRight);
            ui->customPlot->replot();
            m_xCount++;
        }
        else{
            m_xCount++;
        }



        /*
         *
         *handle everything for pitch
         *
         */
        if(deltaY > m_yMaxDelta.toFloat()){
            m_yMaxDelta = QString::number(deltaY);
        }

        if(deltaY < m_yMinDelta.toFloat()){
            m_yMinDelta = QString::number(deltaY);
        }

        max = qMax(qAbs(m_yMaxDelta.toDouble()),qAbs(m_yMinDelta.toDouble()));
        ui->customPlot_2->yAxis->setRange(1.2 * max, -1.2 * max);

        ui->customPlot_2->graph(1)->addData(key, m_yMaxDelta.toFloat());
        ui->customPlot_2->graph(2)->addData(key, m_yMinDelta.toFloat());

        if(deltaY > m_yMaxDeltaAllowed.toFloat() || m_yCount == COUNTS){
            if(m_yCount == 10){
                m_yCount = 0;
            }
            //deal with logging and replotting for y axis
            m_yDelta = QString::number(deltaY);
            ui->yDelta->setText(m_yDelta);

            if(deltaY > m_yMaxDeltaAllowed.toFloat()){
                m_yNumFails = QString::number(m_yNumFails.toInt() + 1);
                m_yFailOccurred = true;
                ui->customPlot_2->setBackground(Qt::red);
                ui->customPlot_2->axisRect()->setBackground(Qt::white);
                ui->yNumFails->setText(m_yNumFails);
                m_yCount = 0;
                m_yMaxDeltaTime = logTime.toString("hh:mm:ss.zzz");
                ui->yFailTime->setText(m_yMaxDeltaTime);
            }
            else{
                if(m_yFailOccurred){
                    ui->customPlot_2->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot_2->setBackground(Qt::green);
                }
            }

            if(deltaY > m_yMaxDelta.toFloat()){
                m_yMaxDelta = m_yDelta;
            }

            // add data to lines:
            ui->customPlot_2->graph(0)->addData(key, deltaY);
            // remove data of lines that's outside visible range:
            ui->customPlot_2->graph(0)->removeDataBefore(key-RANGE);
            ui->customPlot_2->graph(1)->removeDataBefore(key-RANGE);
            ui->customPlot_2->graph(2)->removeDataBefore(key-RANGE);
            // rescale value (vertical) axis to fit the current data:
//            ui->customPlot_2->graph(0)->rescaleValueAxis(true);
//            ui->customPlot_2->graph(1)->rescaleValueAxis(true);
//            ui->customPlot_2->graph(2)->rescaleValueAxis(true);
            // make key axis range scroll with the data (at a constant range size of 16):
            ui->customPlot_2->xAxis->setRange(key+0.25, RANGE, Qt::AlignRight);
            ui->customPlot_2->replot();
            m_yCount++;
        }
        else{
            m_yCount++;
        }


        /*
         *
         *handle everything for yaw
         *
         */
        if(deltaZ > m_zMaxDelta.toFloat()){
            m_zMaxDelta = QString::number(deltaZ);
        }

        if(deltaZ < m_zMinDelta.toFloat()){
            m_zMinDelta = QString::number(deltaZ);
        }

        max = qMax(qAbs(m_zMaxDelta.toDouble()),qAbs(m_zMinDelta.toDouble()));
        ui->customPlot_3->yAxis->setRange(1.2 * max, -1.2 * max);

        ui->customPlot_3->graph(1)->addData(key, m_zMaxDelta.toFloat());
        ui->customPlot_3->graph(2)->addData(key, m_zMinDelta.toFloat());

        if(deltaZ > m_zMaxDeltaAllowed.toFloat() || m_zCount == COUNTS){
            if(m_zCount == 10){
                m_zCount = 0;
            }
            //deal with logging and replotting for z axis
            m_zDelta = QString::number(deltaZ);
            ui->zDelta->setText(m_zDelta);

            if(deltaZ >  m_zMaxDeltaAllowed.toFloat()){
                m_zNumFails = QString::number(m_zNumFails.toInt() + 1);
                m_zFailOccurred = true;//TODO, make background red if fail occurred
                ui->customPlot_3->setBackground(Qt::red);
                ui->customPlot_3->axisRect()->setBackground(Qt::white);
                ui->zNumFails->setText(m_zNumFails);
                m_zCount = 0;
                m_zMaxDeltaTime = logTime.toString("hh:mm:ss.zzz");
                ui->zFailTime->setText(m_zMaxDeltaTime);
            }
            else{
                if(m_zFailOccurred){
                    ui->customPlot_3->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot_3->setBackground(Qt::green);
                }
            }

            if(deltaZ > m_zMaxDelta.toFloat()){
                m_zMaxDelta = m_zDelta;
            }

            // add data to lines:
            ui->customPlot_3->graph(0)->addData(key, deltaZ);
            // remove data of lines that's outside visible range:
            ui->customPlot_3->graph(0)->removeDataBefore(key-RANGE);
            ui->customPlot_3->graph(1)->removeDataBefore(key-RANGE);
            ui->customPlot_3->graph(2)->removeDataBefore(key-RANGE);
            // rescale value (vertical) axis to fit the current data:
//            ui->customPlot_3->graph(0)->rescaleValueAxis(true);
//            ui->customPlot_3->graph(1)->rescaleValueAxis(true);
//            ui->customPlot_3->graph(2)->rescaleValueAxis(true);
            // make key axis range scroll with the data (at a constant range size of 16):
            ui->customPlot_3->xAxis->setRange(key+0.25, RANGE, Qt::AlignRight);
            ui->customPlot_3->replot();
            m_zCount++;
        }
        else{
            m_zCount++;
        }

        QTime displayTime = QTime::QTime(0, 0, 0, 0);
        displayTime = displayTime.addMSecs((int)(m_timerTemp + m_elapsedTime.elapsed()));
        ui->displayTimer->setText(displayTime.toString("hh:mm:ss.zzz"));

    }

}

void WobbleTestDialog::setupPlot(QCustomPlot *customPlot)
{
  // include this section to fully disable antialiasing for higher performance:

  customPlot->setNotAntialiasedElements(QCP::aeAll);
  QFont font;
  font.setStyleStrategy(QFont::NoAntialias);
  customPlot->xAxis->setTickLabelFont(font);
  customPlot->yAxis->setTickLabelFont(font);
  customPlot->legend->setFont(font);
  customPlot->axisRect()->setBackground(Qt::white);
  customPlot->setBackground(Qt::green);
  customPlot->yAxis->setRangeLower(-0.045);
  customPlot->yAxis->setRangeUpper(0.045);
  customPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
  customPlot->xAxis->setDateTimeFormat("hh:mm:ss");
  customPlot->xAxis->setAutoTickStep(false);
  customPlot->xAxis->setTickStep(5);
  customPlot->axisRect()->setupFullAxesBox();
  customPlot->setBackground(Qt::red);
  customPlot->axisRect()->setBackground(Qt::white);
  //

  customPlot->addGraph(); // blue line (data)
  customPlot->graph(0)->setPen(QPen(Qt::blue));

  customPlot->addGraph(); //max watermark
  customPlot->graph(1)->setPen(QPen(Qt::red));

  customPlot->addGraph(); //min watermark
  customPlot->graph(2)->setPen(QPen(Qt::green));
  m_elapsedTime.start();


  // make left and bottom axes transfer their ranges to right and top axes:
//  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
//  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));


}

void WobbleTestDialog::on_closeButton_clicked()
{
    accept();
}

void WobbleTestDialog::on_pauseButton_clicked()
{
    m_pause = true;
    ui->pauseButton->setEnabled(false);
    ui->resumeButton->setEnabled(true);
    m_timerTemp = m_timerTemp + m_elapsedTime.elapsed();
}

void WobbleTestDialog::on_resumeButton_clicked()
{
    m_pause = false;
    ui->pauseButton->setEnabled(true);
    ui->resumeButton->setEnabled(false);
    m_elapsedTime.restart();
}

void WobbleTestDialog::on_refreshSetpointsButton_clicked()
{
    m_xMaxDeltaAllowed = ui->xMaxDeltaAllowed->toPlainText();
    m_yMaxDeltaAllowed = ui->yMaxDeltaAllowed->toPlainText();
    m_zMaxDeltaAllowed = ui->zMaxDeltaAllowed->toPlainText();

    setupPlot(ui->customPlot);
    setupPlot(ui->customPlot_2);
    setupPlot(ui->customPlot_3);

    m_xCount = 0;
    m_xFailOccurred = false;
    m_xDelta = "0";
    m_xMaxDelta = "0";
    m_xMaxDeltaTime = "00:00:00";
    m_xMinDelta = "0";
    m_xNumFails = "0";

    m_yCount = 0;
    m_yFailOccurred = false;
    m_yDelta = "0";
    m_yMaxDelta = "0";
    m_yMaxDeltaTime = "00:00:00";
    m_yMinDelta = "0";
    m_yNumFails = "0";

    m_zCount = 0;
    m_zFailOccurred = false;
    m_zDelta = "0";
    m_zMaxDelta = "0";
    m_zMaxDeltaTime = "00:00:00";
    m_zMinDelta = "0";
    m_zNumFails = "0";

    m_elapsedTime.restart();
    m_timerTemp = 0;
    m_pause = false;
    ui->xDelta->setText("null");
    ui->xFailTime->setText("null");
    ui->xNumFails->setText("null");
    ui->yDelta->setText("null");
    ui->yFailTime->setText("null");
    ui->yNumFails->setText("null");
    ui->zDelta->setText("null");
    ui->zFailTime->setText("null");
    ui->zNumFails->setText("null");
    ui->resumeButton->setEnabled(false);
    ui->pauseButton->setEnabled(true);
}

