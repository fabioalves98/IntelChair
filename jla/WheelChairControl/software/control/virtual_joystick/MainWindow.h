#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QTimer>

#include "ui_dialog.h"
#include "CommSerial.h"

class MainWindow : public QDialog{
  Q_OBJECT
  QTimer timerS, timerR; //, timerDC; // Experimental - TIMERDC
  CommSerial comm;

  QDialog view;
  Ui::Dialog myDialog;
  
  char buttonPressed;
  char connectOption;
  int xval, yval;
  int connectStatus;

  public slots:
    void decreaseMaxWheelSpeed();
    void increaseMaxWheelSpeed();
    void connect_disconnect();
    void sendFrame();
    void readResponse();

  public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
};

#endif  // MAINWINDOW_H
