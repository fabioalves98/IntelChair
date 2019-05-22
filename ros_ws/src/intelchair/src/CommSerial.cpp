/**
 * \file commSerial.cpp
 *
 * Communications with the robot base
 *
 * \author Artur Pereira <artur@ua.pt>
 */

#include "CommSerial.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>


/** \brief The constructor
 *
 */
CommSerial::CommSerial(void)
{
	devname = NULL;
	baudrate = 115200;
	databits = 8;
	stopbits = 1;
	parity = PARITY_NONE;
	fd = -1;
}


/** 
 * \brief Opens the communication channel 
 */
int CommSerial::openSerial(const char* device, int br, int db, int par, int sb)
{
	/* open device */
	devname = device;
	baudrate = br;
	databits = db;
	parity = par;
	stopbits = sb;

	if ((fd = open(devname, O_RDWR | /* O_NOCTTY | */ O_NDELAY)) < 0)
	{
		return -1;
	}
	/* added by TOS */
	if(flock(fd,LOCK_EX | LOCK_NB) < 0)
	{
		fprintf(stderr,"%s is already being used\n",device);
		return -10;
	}
	// flushing is to be done after opening. This prevents first read and write to be spam'ish. (ACP: from cutecom)
	tcflush(fd, TCIOFLUSH);

	/* ACP: estava em cutecom */
	/** \todo Clarify the purpose of the two following lines */
	int n = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, n & ~O_NDELAY);

	/* save old settings */
	if ((tcgetattr(fd,&oldtio)) == -1)
	{
		return -2;
	}

	/* apply new settings */
	if (setNewSettings(baudrate, databits, parity, stopbits, false, false) == -1)
	{
		return -3;
	}
	return 1;
}


/** 
 *\brief Configure channel with new settings
 *
 * \param baudrate
 *      the baudrate
 * \param databits
 *      the number of date bits
 * \param parity
 *      required parity
 * \param stopbits
 *      the number of stop bits
 * \param softwareHandshake
 *      is software handshake active ?
 * \param hardwareHandshake
 *      is hardware handshake active ?
 * \return
 *      0 on success and -1 on error, in which case \c errno is set appropriately.
 */
int CommSerial::setNewSettings(int baudrate, int databits, const int parity, const int stopbits,
        bool softwareHandshake, bool hardwareHandshake)
{
	/* get previous settings */
	struct termios newtio;
	if (tcgetattr(fd, &newtio)!=0)
	{
		fprintf(stderr, "CommChannel::setNewSettings(): tcgetattr()\n");
		memset(&newtio, 0, sizeof(newtio));
	}

	/* change input and output baudrates */
	speed_t _baud=0;
	switch (baudrate)
	{
		case 600:    _baud=B600;    break;
		case 1200:   _baud=B1200;   break;
		case 2400:   _baud=B2400;   break;
		case 4800:   _baud=B4800;   break;
		case 9600:   _baud=B9600;   break;
		case 19200:  _baud=B19200;  break;
		case 38400:  _baud=B38400;  break;
		case 57600:  _baud=B57600;  break;
		case 115200: _baud=B115200; break;
		case 230400: _baud=B230400; break;
		case 460800: _baud=B460800; break;
		case 576000: _baud=B576000; break;
		case 921600: _baud=B921600; break;
			     // case 128000: _baud=B128000; break;
			     // case 256000: _baud=B256000; break;
		default: _baud=B115200; break;
	}
	cfsetospeed(&newtio, (speed_t)_baud);
	cfsetispeed(&newtio, (speed_t)_baud);

	/* We generate mark and space parity ourself. */
	if (databits == 7 && (parity==PARITY_MARK || parity == PARITY_SPACE))
		databits = 8;

	switch (databits)
	{
		case 5:
			newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
			break;
		case 6:
			newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
			break;
		case 7:
			newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
			break;
		case 8:
		default:
			newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
			break;
	}
	newtio.c_cflag |= CLOCAL | CREAD;

	// parity
	newtio.c_cflag &= ~(PARENB | PARODD);
	if (parity == PARITY_EVEN)
		newtio.c_cflag |= PARENB;
	else if (parity== PARITY_ODD)
		newtio.c_cflag |= (PARENB | PARODD);

	//hardware handshake
	/*  
	    if (hardwareHandshake)
	    newtio.c_cflag |= CRTSCTS;
	    else
	    newtio.c_cflag &= ~CRTSCTS;
	 */
	newtio.c_cflag &= ~CRTSCTS;

	// stopbits
	if (stopbits==2)
		newtio.c_cflag |= CSTOPB;
	else
		newtio.c_cflag &= ~CSTOPB;

	/* set input flags */
	//   newtio.c_iflag=IGNPAR | IGNBRK;
	newtio.c_iflag=IGNBRK;
	//   newtio.c_iflag=IGNPAR;

	//software handshake
	if (softwareHandshake)
		newtio.c_iflag |= IXON | IXOFF;
	else
		newtio.c_iflag &= ~(IXON|IXOFF|IXANY);

	/* set local flags */
	newtio.c_lflag=0;

	/* set output flags */
	newtio.c_oflag=0;

	/* ??? */
	newtio.c_cc[VTIME]=1;
	// newtio.c_cc[VMIN]=60; // ACP: no original
	newtio.c_cc[VMIN]=1;

	/* ACP: aplica os novos settings */
	//   tcflush(fd, TCIFLUSH);
	if (tcsetattr(fd, TCSANOW, &newtio)!=0)
	{
		fprintf(stderr, "CommChannel::setNewSettings(): tcsetattr()\n");
		/* ACP: no original não aborta. Fará sentido? */
	}

	/* ACP: ??? */
	int mcs=0;
	//   ioctl(fd, TIOCMODG, &mcs);
	ioctl(fd, TIOCMGET, &mcs);
//	mcs |= TIOCM_RTS;  // JLA

// JLA:
// 	 force RTS to OFF
//	ioctl(fd, TIOCMGET, &controlbits); 
//	if(state == 1) 
//		mcs |= TIOCM_RTS; 
//	else 
		mcs &= ~TIOCM_RTS; 
//	ioctl(fd, TIOCMSET, &controlbits); 

	ioctl(fd, TIOCMSET, &mcs);

	/* ACP: suponho que o ioctl anterior possa alterar os settings (??) */
	if (tcgetattr(fd, &newtio)!=0)
	{
		fprintf(stderr, "CommChannel::setNewSettings(): tcgetattr()\n");
		/* ACP: no original não aborta. Fará sentido? */
	}

	//  hardware handshake
	if (hardwareHandshake)
		newtio.c_cflag |= CRTSCTS;
	else
		newtio.c_cflag &= ~CRTSCTS;
	/* ACP: volta a aplicar os settings (??) */
	if (tcsetattr(fd, TCSANOW, &newtio)!=0)
	{
		fprintf(stderr, "CommChannel::setNewSettings(): tcsetattr()\n");
		/* ACP: no original não aborta. Fará sentido? */
	}
	return 0;
}


/** \brief The destructor
 *
 * Closes the communication channel
 */
CommSerial::~CommSerial(void)
{
	if (fd != -1)
	{
		closeSerial();
	}
}


/**
 * \brief Close the communication channel 
 */
void CommSerial::closeSerial(void)
{
	/* Repor configuracao inicial da porta serie */
	if ((tcsetattr(fd, TCSANOW, &oldtio)) == -1)
	{
		fprintf(stderr, "CommSerial::~CommSerial(): tcsetattr()\n");
		return;
	}
	/* added by TOS */
	flock(fd,LOCK_UN);
	/* Fechar o fd associado a porta serie */
	if (close(fd) == -1)
	{
		fprintf(stderr, "CommSerial::~CommSerial(): close()\n");
		return;
	}
	fd = -1;
}

int CommSerial::serialTx( char *data, int size )
{
	if( write( fd, data, size ) != size )
	{	// Error
		// fprintf(stderr, "serialTx Error\n");
		return 1;
	}
	return 0;
}

int CommSerial::serialTxByte( char byte )
{
	return serialTx( &byte, 1);
}

int CommSerial::serialRxByte( char *data ) 
{
	if( read( fd, data, sizeof(char)) != sizeof(char) )
	{	// Error
		fprintf(stderr, "serialRx Error\n");
		return 1;
	}
	return 0;
}

int CommSerial::fileDescriptor( void ) 
{
	return fd;
}


// Added by JLA
int CommSerial::getRTS(void)
{
	int controlbits;
 
	ioctl(fd, TIOCMGET, &controlbits);

	return controlbits & TIOCM_RTS;
}

void CommSerial::setRTS(int state) 
{ 
	int controlbits; 

	ioctl(fd, TIOCMGET, &controlbits); 
	if(state == 1) 
		controlbits |= TIOCM_RTS; 
	else 
		controlbits &= ~TIOCM_RTS; 
	ioctl(fd, TIOCMSET, &controlbits); 
} 




#if 0
int main(void)
{
	const char *devname = "/dev/ttyUSB0";
	CommSerial *cs;
	char buffer[100];
	unsigned int i = 0;

	cs = new CommSerial;

	if (cs->openSerial(devname, 115200) < 0)
	{
		fprintf(stderr, "ERROR: opening (\"%s\")\n", devname);
		return 1;
	}	
	while(1)
	{
		cs->serialRxByte( buffer );
		printf("%c", buffer[0]);
		fflush(stdout);
	}
	return 1;
}
#endif

#if 0 
int main(void)
{
	const char *devname = "/dev/ttyUSB0";
	CommSerial *cs;
	char buffer[100];
	char *p = buffer;
	unsigned int i = 0;

	cs = new CommSerial;

	if (cs->openSerial(devname, 115200) < 0)
	{
		fprintf(stderr, "ERROR: opening (\"%s\")\n", devname);
		return 1;
	}	
	while( cs->serialRxByte( p ) == 0)
	{
		if( *p == '#' )
			p = buffer;
		if( *p++ == '$' )
		{
			*p = '\0';
			printf("PIC32 Bootloader Vrs%c.%c\nDETI, J.L.Azevedo, Nov/2010\n", buffer[7], buffer[9]);
			cs->serialTx(buffer, strlen(buffer));	
		}
	}
	return 1;
}
#endif

