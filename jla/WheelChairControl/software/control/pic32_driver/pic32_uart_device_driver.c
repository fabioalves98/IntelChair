// ****************************************************************************
// pic32_uart_device_driver.c
//
// J.L.Azevedo
// 
// 03/04/2011
// ****************************************************************************

//#include <p32xxxx.h>
#include "detpic32.h"
#include "pic32_uart_device_driver.h"

//#define _int_(v)  __attribute__((vector(v))) __attribute__((interrupt))

//#define PBCLK	20000000

#define TRUE 1
#define FALSE 0


#define UART_BUF_SIZE	256	// must be 2's power
//#define UART_BUF_SIZE	32768	// must be 2's power
#define INDEX_MASK	(UART_BUF_SIZE-1)


#if(UART == 1)
   #define UMODE     U1MODE
   #define UMODEbits U1MODEbits

   #define IFSbits   IFS0bits
   #define IECbits   IEC0bits
   #define IPCbits   IPC6bits

   #define USTA	   U1STA
   #define USTAbits  U1STAbits
   #define UBRG	   U1BRG
   #define URXREG    U1RXREG
   #define UTXREG    U1TXREG

   #define URXIF     U1RXIF
   #define UTXIF     U1TXIF
   #define UEIF	   U1EIF

   #define UEIE	   U1EIE
   #define URXIE     U1RXIE
   #define UTXIE     U1TXIE

   #define UIP       U1IP

   #define UART_VECTOR  24		

#elif(UART == 2)
   #define UMODE     U2MODE
   #define UMODEbits U2MODEbits

   #define IFSbits   IFS1bits
   #define IECbits   IEC1bits
   #define IPCbits   IPC8bits

   #define USTA	   U2STA
   #define USTAbits  U2STAbits
   #define UBRG	   U2BRG
   #define URXREG    U2RXREG
   #define UTXREG    U2TXREG

   #define URXIF     U2RXIF
   #define UTXIF     U2TXIF
   #define UEIF      U2EIF

   #define UEIE	   U2EIE
   #define URXIE     U2RXIE
   #define UTXIE     U2TXIE

   #define UIP       U2IP

   #define UART_VECTOR  32	

#elif(UART == 3)
   #define UMODE     U3MODE
   #define UMODEbits U3MODEbits

   #define IFSbits   IFS1bits
   #define IECbits   IEC1bits
   #define IPCbits   IPC7bits

   #define USTA	   U3STA
   #define USTAbits  U3STAbits
   #define UBRG	   U3BRG
   #define URXREG    U3RXREG
   #define UTXREG    U3TXREG

   #define URXIF     U3RXIF
   #define UTXIF     U3TXIF
   #define UEIF      U3EIF

   #define UEIE	   U3EIE
   #define URXIE     U3RXIE
   #define UTXIE     U3TXIE

   #define UIP       U3IP

   #define UART_VECTOR  31	

#elif(UART == 4)
   #define UMODE     U4MODE
   #define UMODEbits U4MODEbits

   #define IFSbits   IFS2bits
   #define IECbits   IEC2bits
   #define IPCbits   IPC12bits

   #define USTA	   U4STA
   #define USTAbits  U4STAbits
   #define UBRG	   U4BRG
   #define URXREG    U4RXREG
   #define UTXREG    U4TXREG

   #define URXIF     U4RXIF
   #define UTXIF     U4TXIF
   #define UEIF      U4EIF

   #define UEIE      U4EIE
   #define URXIE     U4RXIE
   #define UTXIE     U4TXIE

   #define UIP       U4IP

   #define UART_VECTOR  49	

#elif(UART == 5)
   #define UMODE     U5MODE
   #define UMODEbits U5MODEbits

   #define IFSbits   IFS2bits
   #define IECbits   IEC2bits
   #define IPCbits   IPC12bits

   #define USTA      U5STA
   #define USTAbits  U5STAbits
   #define UBRG      U5BRG
   #define URXREG    U5RXREG
   #define UTXREG    U5TXREG

   #define URXIF     U5RXIF
   #define UTXIF     U5TXIF
   #define UEIF	   U5EIF

   #define UEIE      U5EIE
   #define URXIE     U5RXIE
   #define UTXIE     U5TXIE

   #define UIP       U5IP

   #define UART_VECTOR  51

#elif(UART == 6)
   #define UMODE     U6MODE
   #define UMODEbits U6MODEbits

   #define IFSbits   IFS2bits
   #define IECbits   IEC2bits
   #define IPCbits   IPC12bits

   #define USTA      U6STA
   #define USTAbits  U6STAbits
   #define UBRG      U6BRG
   #define URXREG    U6RXREG
   #define UTXREG    U6TXREG

   #define URXIF     U6RXIF
   #define UTXIF     U6TXIF
   #define UEIF      U6EIF

   #define UEIE      U6EIE
   #define URXIE     U6RXIE
   #define UTXIE     U6TXIE

   #define UIP       U6IP

   #define UART_VECTOR  50	
#else
	#error "Invalid UART definition"
#endif


typedef struct
{
   unsigned char data[UART_BUF_SIZE];
   unsigned int head;
   unsigned int tail;
   unsigned int count;
} SerialBuf;

volatile static SerialBuf rxb;
volatile static SerialBuf txb;



// ****************************************************************************
char commDrv_getChar(char *ch)
{
   if(rxb.count == 0)
      return FALSE;

   DisableUartRxInterrupt();
   *ch = rxb.data[rxb.head];
   rxb.count--;
   rxb.head = (rxb.head + 1) & INDEX_MASK;
   EnableUartRxInterrupt();
   return TRUE;
} 

// ****************************************************************************
char commDrv_putChar(char ch)
{
   while(txb.count == UART_BUF_SIZE);	// Buffer is full

   DisableUartTxInterrupt();
   txb.data[txb.tail] = ch;			// Critical section
   txb.count++;							
   txb.tail = (txb.tail + 1) & INDEX_MASK;
   EnableUartTxInterrupt();
   return ch;
}

// ****************************************************************************
void commDrv_putStr(char *s)
{
   while(*s != 0)
      commDrv_putChar(*s++);
}


// ****************************************************************************
// UART reception interrupt
void _int_(UART_VECTOR) uart_isr(void)
{
   unsigned char trash;

   if(IFSbits.UEIF == 1)
   {
      USTAbits.OERR = 0; 	// OERR flag must be cleared
      trash = URXREG;		// Discard value
      IFSbits.UEIF = 0;
      IFSbits.URXIF = 0;	// Prevents void reading
   }

   if(IFSbits.URXIF == 1)
   {
      rxb.data[rxb.tail] = URXREG;
      rxb.tail = (rxb.tail + 1) & INDEX_MASK;
      if(rxb.count < UART_BUF_SIZE)
         rxb.count++;
      else
         rxb.head = (rxb.head + 1) & INDEX_MASK;   // Discard oldest character
      IFSbits.URXIF = 0;
   }

   if(IFSbits.UTXIF == 1)
   {
      while(txb.count > 0 && USTAbits.UTXBF == 0)  // Transmit buffer is not full 
      {						   //(at least one more character can be written)
         UTXREG = txb.data[txb.head];
         txb.head = (txb.head + 1) & INDEX_MASK;
         txb.count--;
      }
      if(txb.count == 0)   // This test (instead of an "else") avoids an 
         // aditional interrupt on end of transmission
         DisableUartTxInterrupt();
      IFSbits.UTXIF = 0;
   }
}


// ****************************************************************************
void commDrv_config(unsigned int baudrate, char parity, int stopBits)
{
   stopBits = (stopBits >> 1) & 0x01;

   if(baudrate < 300 || baudrate > 460800)
      baudrate = 115200;

   if(baudrate <= 115200)
   {
      UBRG = (PBCLK + 8 * baudrate) / (16 * baudrate) - 1;
      UMODEbits.BRGH = 0;		// baudrate factor = 16
   }
   else
   {
      UBRG = (PBCLK + 2 * baudrate) / (4 * baudrate) - 1;
      UMODEbits.BRGH = 1;		// baudrate factor = 4
   }

   UMODEbits.STSEL = stopBits;	// 0 -> 1 stop bit / 1 -> 2 stop bits
   switch (parity)
   {
      case 'e':
      case 'E':
         UMODEbits.PDSEL = 1;
         break;
      case 'o':
      case 'O':
         UMODEbits.PDSEL = 2;
         break;
      default:
         UMODEbits.PDSEL = 0;
         break;
   }
   USTAbits.URXEN = 1;	// Enable receiver section
   USTAbits.UTXEN = 1;	// Enable transmiter section
   USTAbits.URXISEL = 0;	// Interrupt flag is set when a character is received
   U1STAbits.UTXSEL = 0;	// Interrupt is generated while de transmit buffer contains 
   // at least one empty space 

   IFSbits.URXIF = 0;
   IFSbits.UTXIF = 0;
   IFSbits.UEIF = 0;

   IECbits.UEIE = 1;
   IECbits.URXIE = 1;
   IECbits.UTXIE = 0;
   IPCbits.UIP = 2;

   rxb.head=rxb.tail=rxb.count=0;
   txb.head=txb.tail=txb.count=0;

   UMODEbits.ON = 1;
}
 
// This allows the usage of printf() function
void _mon_putc (char c)
{
   while(USTAbits.UTXBF);
   UTXREG = c;
}

#if 0
// ****************************************************************************
int main(void)
{
	char ch;
	int i;

	commDrv_config(115200, 'n', 1);
	EnableInterrupts();

	commDrv_putStr("PIC32 UART Device Driver - 2011-04-03\n");
	
	while(1)
	{
		while(commDrv_getChar(&ch) == 0);
		if(ch == 'X')
		{
			for(i=0; i < 100; i++)
				commDrv_putStr("Aqui vai uma string grande como o caracas, sem grande imaginacao, diga-se de passagem, para por esta treta em MEGA teste com uma filinha pirilau 'a  espera de haver lugar no buffer para as interrupcoes despacharem todos os caracteres - como se ve esta em uma string que nao acaba mais, mas tem que ser assim, pois caso contrario o teste parece um daqueles testes CAMBADA em que basta testar uma vez e ja esta tudo bem :-)\n");
//			commDrv_putStr("Aqui vai uma string piquena:-)\n");
		}
		else if(ch == 'Y')
		{
			printStr("\n");
			printInt(txb.head, 16 + (2 << 16));
			printStr(" - ");
			printInt(txb.tail, 16 + (2 << 16));
			printStr("\n");
			printInt(rxb.head, 16 + (2 << 16));
			printStr(" - ");
			printInt(rxb.tail, 16 + (2 << 16));


		}
		else
			commDrv_putChar(ch);
	}
}
#endif


