#include <QtGui>
#include <QTimer>

#include "MainWindow.h"
#include "ui_dialog.h"

#define DEBUG 1
#define DEBUG1 0
#define DEVICE "/dev/ttyUSB3"

// Constructor
MainWindow::MainWindow(QWidget *parent) : QDialog(parent){
   // Max Wheel Speed initial value
   buttonPressed = 0x00;

   // Status of Connection
   connectOption = 0x00;

   // Initial position value
   xval = yval = 0;

   // Preperation of dialog window
   myDialog.setupUi(&view);
   connect(myDialog.decreaseMaxSpeedButton, SIGNAL(clicked()), this, SLOT(decreaseMaxWheelSpeed()));
   connect(myDialog.increaseMaxSpeedButton, SIGNAL(clicked()), this, SLOT(increaseMaxWheelSpeed()));
   connect(myDialog.connectButton, SIGNAL(clicked()), this, SLOT(connect_disconnect()));

   // Timer for sendFrame()
   timerS.setInterval(50);         	// time in msec
   timerS.start();   
   // Timer for readResponse()
   timerR.setInterval(8);          	// time in msec
   timerR.start();
   // Timer for timeout of connection to wheel chair
   //timerDC.setInterval(60000);     // time in msec - Experimental - TIMERDC

   // Assigning timers to functions
   connect(&timerS, SIGNAL(timeout()), this, SLOT(sendFrame()));  
   connect(&timerR, SIGNAL(timeout()), this, SLOT(readResponse()));  
   //connect(&timerDC, SIGNAL(timeout()), this, SLOT(connect_disconnect()));  // Experimental - TIMERDC

   // Start the commSerial connection
   comm.openSerial(DEVICE, 115200, 8, PARITY_NONE, 1);

   // Display the Window Form
   view.show();
}

// Deconstructor
MainWindow::~MainWindow(){
   comm.closeSerial();
}

void MainWindow::sendFrame(){
   char aux[1024 * 4];       // Variable to hold instructions to be sent

   if (connectStatus != 0){
      sprintf(aux, "#+000+00000");

      // Send aux variable
      if(comm.serialTx(aux, strlen(aux)) != 0)
         fprintf(stderr, "ERRO ENVIO %s %d\n", aux, (int)strlen(aux));
      return;
   }

   // GET of the position of Virtual Joystick
   xval = myDialog.graphicsView->xval;
   yval = myDialog.graphicsView->yval;

   if ((xval != 0) || (yval != 0))
      //timerDC.start();              // Experimental - TIMERDC

      // DEBUG MESSAGE
      if (DEBUG1)
         fprintf(stderr, "xval: %03d yval: %03d buttonPressed: %1d connectOption: %1d connectionStatus %1d", xval, yval, buttonPressed, connectOption, connectStatus);

   // Saving position of Virtual Joystick in aux variable
   sprintf(aux, "#%c%03d%c%03d%1d%1d", (xval > 0 ? '+' : '-'), abs(xval), (yval > 0 ? '+' : '-'), abs(yval), buttonPressed, connectOption);

   // DEBUG MESSAGE
   if (DEBUG1)
      fprintf(stderr, " aux: %s, count: %d\n", aux, (int)strlen(aux));

   // Send aux variable
   if(comm.serialTx(aux, strlen(aux)) != 0)
      fprintf(stderr, "ERRO ENVIO %s %d\n", aux, (int)strlen(aux));

   // Reset of Button pressed
   if (buttonPressed != 0x00)
      buttonPressed = 0x00;

   // Reset of Button pressed
   if (connectOption != 0x00)
      connectOption = 0x00;

   // update of LCD's on Window Form
   myDialog.straightSpeedLCD->display(xval);
   myDialog.rotationalSpeedLCD->display(yval);
}

void MainWindow::decreaseMaxWheelSpeed(){
   buttonPressed = 0x02;   // Decrease Max Wheel Speed
}

void MainWindow::increaseMaxWheelSpeed(){
   buttonPressed = 0x04;   // Increase Max Wheel Speed
}

void MainWindow::connect_disconnect(){
   if (connectStatus){
      connectStatus = 0;
      connectOption = 0x01;   // Send information to connect
      //timerDC.start(); // Experimental - TIMERDC
      fprintf(stderr, "Connect command sent!\n");
   }else{
      //timerDC.stop();         // Experimental - TIMERDC
      connectOption = 0x02;   // Send information to disconnect
      connectStatus = 1;
      myDialog.connectionStatusLabel->setText("Not Connected");
      myDialog.connectButton->setText("Push to Connect");
      fprintf(stderr, "Disconnect command sent!\n");
   }
}

void MainWindow::readResponse(){
   char ch;
   char response[1024*16] = {0};
   int count = 0;

   int fd = comm.fileDescriptor();
   fd_set read_mask;
   struct timeval time_out;

   FD_ZERO(&read_mask);
   FD_SET(fd, &read_mask);
   time_out.tv_sec = 0;  
   time_out.tv_usec = 0;
   select(fd+1, &read_mask, NULL, NULL, &time_out);

   if (connectStatus != 0)
      return;

   while(FD_ISSET(fd, &read_mask) || count < 14){
      comm.serialRxByte(&ch);
      response[count] = ch;

      FD_ZERO(&read_mask);
      FD_SET(fd, &read_mask);
      time_out.tv_sec = 0;
      time_out.tv_usec = 0;
      select(fd+1, &read_mask, NULL, NULL, &time_out);

      count++;

      if (((response[0] != 'F') || (response[1] != 'E')) && (count == 2)){
         count = 0;
         continue;
      }

      if (((response[2] != '5') || (response[3] != '4')) && (count == 4)){
         count = 0;
         continue;
      }
   }

   // Clean buffer in case there's still more information on it to be read
   while(FD_ISSET(fd, &read_mask)){
      comm.serialRxByte(&ch);

      FD_ZERO(&read_mask);
      FD_SET(fd, &read_mask);
      time_out.tv_sec = 0;
      time_out.tv_usec = 0;
      select(fd+1, &read_mask, NULL, NULL, &time_out);
   }

   if (DEBUG){
      for (count = 0; count < 14 ; count++)
         fprintf(stderr, "%c",response[count]);
      fprintf(stderr, "\n");
   }

   // Read of values to update on Form LCD's'
   int velocity = response[10]-'0';                      // conversion to ascii -'0'
   int battery = response[8]-'0';                        // conversion to ascii -'0'
   battery = battery >= 17 ? battery - 7 : battery;      // conversion to decimal
   velocity = velocity >= 17 ? velocity - 7 : velocity;  // conversion to decimal
   connectStatus = response[4]-'0';
   connectStatus = connectStatus >= 17 ? connectStatus - 7 : connectStatus;

   // Update connection status
   if (connectStatus != 0){
      connectStatus = 0;
      connectOption = 0x02;   // Send information to disconnect
      //myDialog.connectionStatusLabel->setText("Not Connected");
      //myDialog.connectButton->setText("Push to Connect");
   }else{
      myDialog.connectionStatusLabel->setText("Connected");
      myDialog.connectButton->setText("Push to Disconnect");
   }

   // update of battery Level LCD
   myDialog.batteryLevelLCD->display(battery);

   // Update of Max Wheel Speed LCD
   if (velocity == 3){
      myDialog.maxWheelSpeedLCD->display(1);
   }else if(velocity == 5){
      myDialog.maxWheelSpeedLCD->display(2);
   }else if(velocity == 7){
      myDialog.maxWheelSpeedLCD->display(3);
   }else if(velocity == 9){
      myDialog.maxWheelSpeedLCD->display(4);
   }else
      myDialog.maxWheelSpeedLCD->display(5);
}
