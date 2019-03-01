// pic32_uart.h

#ifndef __PIC32_UART_H
#define __PIC32_UART_H

void configUart(int channel, unsigned int baudrate, unsigned char parity, int);
void txChar(int channel, char txChar);
char rxChar(int channel, int *errFlags);
void enableUart(int channel);
void disableUart(int channel);

void enableUartTx(int channel);
void disableUartTx(int channel);
#endif

