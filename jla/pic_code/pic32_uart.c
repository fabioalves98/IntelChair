#include <detpic32.h>
#include "pic32_uart.h"

#define UART1_ADDRESS	U1MODE
int uart_offsets[] = {0x0000, 0x0800, 0x0400, 0x0200, 0x0A00, 0x0600}; 

// ****************************************************************************
char rxChar(int channel, int *errFlags)
{
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);
	int *urxreg = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x30);

	char ch;
	while((*usta & 0x0001) == 0);	// wait until URXDA == 1
	
	ch = *urxreg; 	//get data from UART RX FIFO
	if( (*usta & 0x0002))
		*usta &= 0xFFFD;	// Clear OERR bit

	*errFlags = (*usta & 0x000C) >> 2;	// bit 0 is FERR, bit is PERR
	return ch;
}

// ****************************************************************************
void txChar(int channel, char ch)
{
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);
	int *utxreg = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x20);

	while( (*usta & 0x200) == 1);	// Test UTXBUF bit (Transmitter buffer is full)
	*utxreg = ch;
	while( (*usta & 0x100) == 0);	// Test TRMT bit (Transmitter is not empty)
}

// ****************************************************************************
// default settings assumed:
//  - baudrate factor = 16 (if the core frequency is 10 MHz, then the BRGH 
//  						factor should be 4)
//  - 1 stop bit
//  - parity N
//  - Auto Baud = OFF
void configUart(int channel, unsigned int baudrate, unsigned char parity, int stopBits)
{
	int *umode = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x00);
	int *ubrg = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x40);

	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);

	if(baudrate < 300 || baudrate > 115200)
		baudrate = 115200;

	*ubrg = (PBCLK + 8 * baudrate) / (16 * baudrate) - 1;	

	*usta = *usta | 0x1400;		// Enable receiver section
	// Enable transmiter sction
	if(parity == 'E' || parity == 'e')
		*umode = (*umode & 0xFFF9) | 0x0002; // 01x
	else if(parity == 'O' || parity == 'o')
		*umode = (*umode & 0xFFF9) | 0x0004; // 10x
	if(stopBits == 2)
		*umode |= 0x0001;

	*umode = *umode | 0x8000;	// UxMODEbits.ON = 1; 	// UART ON
}

// ****************************************************************************
void enableUart(int channel)
{
	int *umode = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x00);
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);

	*umode = *umode | 0x8000;	// UxMODEbits.ON = 1; 	// UART ON
	*usta = *usta | 0x1400;		// Activate RXEN and TXEN
}

// ****************************************************************************
void disableUart(int channel)
{
	int *umode = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x00);
	*umode = *umode & 0x7FFF;	// UxMODEbits.ON = 0; 	// UART OFF
}

// ****************************************************************************
void enableUartTx(int channel)
{
	//int *umode = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x00);
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);

	//*umode = *umode | 0x8000;	// UxMODEbits.ON = 1; 	// UART ON
	*usta = *usta | 0x0400;		// Activate RXEN and TXEN
}

// ****************************************************************************
void disableUartTx(int channel)
{
	volatile int *usta = (int *)((char *)&UART1_ADDRESS + uart_offsets[channel-1] + 0x10);

	*usta = *usta & 0xFBFF;		// Activate RXEN and TXEN
}

#if 0
int main(void)
{
	int errFlags;
	char ch;

	configUart(2, 38400, 'E', 1);

	while(1)
	{
		ch = rxChar(2, &errFlags);
		if(errFlags != 0)
			printf("ERR%X  ", errFlags);
		if(ch == 0x4A)
			putChar('\n');
		printInt( ch & 0xFF, 2 << 16 | 16 );
		putChar(' ');
	}
}
#endif
