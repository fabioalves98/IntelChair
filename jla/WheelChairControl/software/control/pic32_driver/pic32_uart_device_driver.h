// ****************************************************************************
// pic32_uart_device_driver.h
//
// J.L.Azevedo
// 06/02/2013
// ****************************************************************************

#ifndef PIC32_UART_DEVICE_DRIVER_H
#define PIC32_UART_DEVICE_DRIVER_H


#define	DisableUartTxInterrupt()	IECbits.UTXIE = 0
#define	EnableUartTxInterrupt()		IECbits.UTXIE = 1

#define	DisableUartRxInterrupt()	IECbits.URXIE = 0
#define	EnableUartRxInterrupt()		IECbits.URXIE = 1

#define	EnableUartInterrupts()		IECbits.UTXIE = IECbits.URXIE = 1

#define	DisableUartInterrupts()		TXIE_ORI = IECbits.UTXIE; \
					IECbits.UTXIE = IECbits.URXIE = 0

#define UART	6

void commDrv_config(unsigned int baudrate, char parity, int stopBits);
char commDrv_getChar(char *ch);
char commDrv_putChar(char ch);
void commDrv_putStr(char *s);

#endif

