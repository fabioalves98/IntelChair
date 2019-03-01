#include <detpic32.h>
#include "pic32_uart.c"


int main(void)
{
	int errFlags;
	unsigned char ch;

//	configUart(2, 38400, 'E', 1);
	configUart(2, 38800, 'E', 1);

	while(1)
	{
		ch = rxChar(2, &errFlags);
		if(errFlags != 0)
			printf("ERR%X  ", errFlags);
		if(ch == 0x4A)
			putChar('\n');
		else if(ch == 0xFE)
			putChar('-');

		printInt( ch & 0xFF, 2 << 16 | 16 );
		putChar(' ');
	}
}

