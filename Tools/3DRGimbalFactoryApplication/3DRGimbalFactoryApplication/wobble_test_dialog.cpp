#include "wobble_test_dialog.h"
#include "ui_wobble_test_dialog.h"
#include "mainwindow.h"
#include "serial_interface_thread.h"

#include <QTime>



WobbleTestDialog::WobbleTestDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::WobbleTestDialog),
//    X_MAX_GYRO_ALLOWED(0.01),
//    Y_MAX_GYRO_ALLOWED(0.01),
//    Z_MAX_GYRO_ALLOWED(0.01),
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
    m_xGyro = "0";
    m_xMaxGyro = "0";
    m_xMaxGyroTime = "00:00:00";
    m_xMinGyro = "0";
    m_xNumFails = "0";
    m_xMaxGyroAllowed = "0.001";

    m_yCount = 0;
    m_yFailOccurred = false;
    m_yGyro = "0";
    m_yMaxGyro = "0";
    m_yMaxGyroTime = "00:00:00";
    m_yMinGyro = "0";
    m_yNumFails = "0";
    m_yMaxGyroAllowed = "0.001";

    m_zCount = 0;
    m_zFailOccurred = false;
    m_zGyro = "0";
    m_zMaxGyro = "0";
    m_zMaxGyroTime = "00:00:00";
    m_zMinGyro = "0";
    m_zNumFails = "0";
    m_zMaxGyroAllowed = "0.001";

    m_pause = false;
    ui->resumeButton->setEnabled(false);
    ui->pauseButton->setEnabled(true);

    m_timerTemp = 0;
}

WobbleTestDialog::~WobbleTestDialog()
{
    delete ui;
}

void WobbleTestDialog::receivedGimbalReport(float gyroX, float gyroY, float gyroZ)
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
        if(gyroX > m_xMaxGyro.toFloat()){
            m_xMaxGyro = QString::number(gyroX);
            ui->xMaxGyro->setText(m_xMaxGyro);
        }

        if(gyroX < m_xMinGyro.toFloat()){
            m_xMinGyro = QString::number(gyroX);
            ui->xMinGyro->setText(m_xMinGyro);
        }

        max = qMax(qAbs(m_xMaxGyro.toDouble()),qAbs(m_xMinGyro.toDouble()));
        ui->customPlot->yAxis->setRange(1.2 * max, -1.2 * max);

        //plot watermarks
        ui->customPlot->graph(1)->addData(key, m_xMaxGyro.toFloat());
        ui->customPlot->graph(2)->addData(key, m_xMinGyro.toFloat());

        //if we've failed this run or it's been 10 counts since adding point to plot, refresh plot
        if(gyroX > m_xMaxGyroAllowed.toFloat() || m_xCount == COUNTS){
            if(m_xCount == 10){
                m_xCount = 0;
            }
            //deal with logging and replotting for x axis
            m_xGyro = QString::number(gyroX);
            ui->xGyro->setText(m_xGyro);

            if(gyroX > m_xMaxGyroAllowed.toFloat()){
                m_xNumFails = QString::number(m_xNumFails.toInt() + 1);
                m_xFailOccurred = true;
                ui->customPlot->setBackground(Qt::red);
                ui->customPlot->axisRect()->setBackground(Qt::white);
                ui->xNumFails->setText(m_xNumFails);
                m_xCount = 0;
                m_xMaxGyroTime = logTime.toString("hh:mm:ss.zzz");
                ui->xFailTime->setText(m_xMaxGyroTime);
            }
            else{
                if(m_xFailOccurred){
                    ui->customPlot->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot->setBackground(Qt::green);
                }
            }

            if(gyroX > m_xMaxGyro.toFloat()){
                m_xMaxGyro = m_xGyro;
            }

            // add data to lines:
            ui->customPlot->graph(0)->addData(key, gyroX);
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
        if(gyroY > m_yMaxGyro.toFloat()){
            m_yMaxGyro = QString::number(gyroY);
            ui->yMaxGyro->setText(m_yMaxGyro);
        }

        if(gyroY < m_yMinGyro.toFloat()){
            m_yMinGyro = QString::number(gyroY);
            ui->yMinGyro->setText(m_yMinGyro);
        }

        max = qMax(qAbs(m_yMaxGyro.toDouble()),qAbs(m_yMinGyro.toDouble()));
        ui->customPlot_2->yAxis->setRange(1.2 * max, -1.2 * max);

        ui->customPlot_2->graph(1)->addData(key, m_yMaxGyro.toFloat());
        ui->customPlot_2->graph(2)->addData(key, m_yMinGyro.toFloat());

        if(gyroY > m_yMaxGyroAllowed.toFloat() || m_yCount == COUNTS){
            if(m_yCount == 10){
                m_yCount = 0;
            }
            //deal with logging and replotting for y axis
            m_yGyro = QString::number(gyroY);
            ui->yGyro->setText(m_yGyro);

            if(gyroY > m_yMaxGyroAllowed.toFloat()){
                m_yNumFails = QString::number(m_yNumFails.toInt() + 1);
                m_yFailOccurred = true;
                ui->customPlot_2->setBackground(Qt::red);
                ui->customPlot_2->axisRect()->setBackground(Qt::white);
                ui->yNumFails->setText(m_yNumFails);
                m_yCount = 0;
                m_yMaxGyroTime = logTime.toString("hh:mm:ss.zzz");
                ui->yFailTime->setText(m_yMaxGyroTime);
            }
            else{
                if(m_yFailOccurred){
                    ui->customPlot_2->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot_2->setBackground(Qt::green);
                }
            }

            if(gyroY > m_yMaxGyro.toFloat()){
                m_yMaxGyro = m_yGyro;
            }

            // add data to lines:
            ui->customPlot_2->graph(0)->addData(key, gyroY);
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
        if(gyroZ > m_zMaxGyro.toFloat()){
            m_zMaxGyro = QString::number(gyroZ);
            ui->zMaxGyro->setText(m_zMaxGyro);
        }

        if(gyroZ < m_zMinGyro.toFloat()){
            m_zMinGyro = QString::number(gyroZ);
            ui->zMinGyro->setText(m_zMinGyro);
        }

        max = qMax(qAbs(m_zMaxGyro.toDouble()),qAbs(m_zMinGyro.toDouble()));
        ui->customPlot_3->yAxis->setRange(1.2 * max, -1.2 * max);

        ui->customPlot_3->graph(1)->addData(key, m_zMaxGyro.toFloat());
        ui->customPlot_3->graph(2)->addData(key, m_zMinGyro.toFloat());

        if(gyroZ > m_zMaxGyroAllowed.toFloat() || m_zCount == COUNTS){
            if(m_zCount == 10){
                m_zCount = 0;
            }
            //deal with logging and replotting for z axis
            m_zGyro = QString::number(gyroZ);
            ui->zGyro->setText(m_zGyro);

            if(gyroZ >  m_zMaxGyroAllowed.toFloat()){
                m_zNumFails = QString::number(m_zNumFails.toInt() + 1);
                m_zFailOccurred = true;//TODO, make background red if fail occurred
                ui->customPlot_3->setBackground(Qt::red);
                ui->customPlot_3->axisRect()->setBackground(Qt::white);
                ui->zNumFails->setText(m_zNumFails);
                m_zCount = 0;
                m_zMaxGyroTime = logTime.toString("hh:mm:ss.zzz");
                ui->zFailTime->setText(m_zMaxGyroTime);
            }
            else{
                if(m_zFailOccurred){
                    ui->customPlot_3->setBackground(Qt::yellow);
                }
                else{
                    ui->customPlot_3->setBackground(Qt::green);
                }
            }

            if(gyroZ > m_zMaxGyro.toFloat()){
                m_zMaxGyro = m_zGyro;
            }

            // add data to lines:
            ui->customPlot_3->graph(0)->addData(key, gyroZ);
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
  connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
  connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));


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
    m_xMaxGyroAllowed = ui->xMaxGyroAllowed->toPlainText();
    m_yMaxGyroAllowed = ui->yMaxGyroAllowed->toPlainText();
    m_zMaxGyroAllowed = ui->zMaxGyroAllowed->toPlainText();

    setupPlot(ui->customPlot);
    setupPlot(ui->customPlot_2);
    setupPlot(ui->customPlot_3);

    m_xCount = 0;
    m_xFailOccurred = false;
    m_xGyro = "0";
    m_xMaxGyro = "0";
    m_xMaxGyroTime = "00:00:00";
    m_xMinGyro = "0";
    m_xNumFails = "0";

    m_yCount = 0;
    m_yFailOccurred = false;
    m_yGyro = "0";
    m_yMaxGyro = "0";
    m_yMaxGyroTime = "00:00:00";
    m_yMinGyro = "0";
    m_yNumFails = "0";

    m_zCount = 0;
    m_zFailOccurred = false;
    m_zGyro = "0";
    m_zMaxGyro = "0";
    m_zMaxGyroTime = "00:00:00";
    m_zMinGyro = "0";
    m_zNumFails = "0";

    m_elapsedTime.restart();
    m_timerTemp = 0;
    m_pause = false;
    ui->xGyro->setText("null");
    ui->xFailTime->setText("null");
    ui->xNumFails->setText("null");
    ui->xMaxGyro->setText("null");
    ui->xMinGyro->setText("null");
    ui->yGyro->setText("null");
    ui->yFailTime->setText("null");
    ui->yNumFails->setText("null");
    ui->yMaxGyro->setText("null");
    ui->yMinGyro->setText("null");
    ui->zGyro->setText("null");
    ui->zFailTime->setText("null");
    ui->zNumFails->setText("null");
    ui->zMaxGyro->setText("null");
    ui->zMinGyro->setText("null");
    ui->resumeButton->setEnabled(false);
    ui->pauseButton->setEnabled(true);
}

