#include "CommHandler.h"
#include "CommSerial.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEVICE "/dev/ttyUSB0"

CommHandler::CommHandler(void)
{
    comm.openSerial(DEVICE, 115200, 8, PARITY_NONE, 1);
    connectInfo = 0x00;
    velocityInfo = 0x00;
    connectStatus = 0x00;
}

CommHandler::~CommHandler(void){
    comm.~CommSerial();
}

void CommHandler::buildFrame(Coordinate joystick, int buttonPressed, int connectOption){
    velocityInfo = buttonPressed;
    connectInfo  = connectOption;

    if(connectInfo == 0x01)
    {
        printf("CONNECT\n");
        connectStatus = 0x01;
    }

    sprintf(aux, "#%c%03d%c%03d%1d%1d", (joystick.x > 0 ? '+' : '-'), abs(joystick.x), 
           (joystick.y > 0 ? '+' : '-'), abs(joystick.y), velocityInfo, connectInfo);

    // printf("FRAME: %s\n", aux);

}

void CommHandler::sendFrame(Coordinate joystick, int buttonPressed, int connectOption){
    CommHandler::buildFrame(joystick, buttonPressed, connectOption);

    if(comm.serialTx(aux, strlen(aux)) != 0)
        fprintf(stderr, "ERRO ENVIO %s %d\n", aux, (int)strlen(aux));
}

ChairInfo CommHandler::receiveFrame(){

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

    if (connectStatus == 0)
      return parseFrame(response);

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
    printFrame(response);
    
    return parseFrame(response);

}


ChairInfo CommHandler::parseFrame(char* response)
{
    ChairInfo chair;
    int VELOCITY_INDEX = 10;
    int BATTERY_INDEX = 8;
    int CONNECT_INDEX = 4;

    char bat = response[BATTERY_INDEX];
    char vel = response[VELOCITY_INDEX];

    chair.connected = response[CONNECT_INDEX] == 0x0 ? 0 : 1; 
    chair.velocity = (((vel >= 'A') ? (vel - 'A' + 10) : (vel - '0')) - 1) / 2;
    chair.battery = (bat >= 'A') ? (bat - 'A' + 10) : (bat - '0');

    return chair;

}

void CommHandler::printFrame(char* response)
{
    for (int count = 0; count < 14 ; count++)
    {
        fprintf(stderr, "%c",response[count]);
        if ((count + 1) % 2 == 0)
        {
        fprintf(stderr, " ");
        }
    }
    fprintf(stderr, "\n");
}

