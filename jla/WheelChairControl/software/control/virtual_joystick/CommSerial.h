/**
 * \file CommSerial.h
 *
 * Communications with the car platform
 *
 * \author Artur Pereira <artur@ua.pt>
 */

/**
 * \brief The ROTA car hardware
 *
 * \author Artur Pereira <artur@ua.pt>
 *
 * \date Jan/2010
 */
#ifndef _ROTA_COMM_SERIAL_
#define _ROTA_COMM_SERIAL_

#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>


enum { PARITY_NONE, PARITY_ODD, PARITY_EVEN, PARITY_MARK, PARITY_SPACE };


class CommSerial
{
	private:	// private attributes

		int fd;                         /**< channel file descriptor */
		struct termios newtio;          /**< the new channel settings */
		struct termios oldtio;          /**< the previous channel settings */
		const char* devname;            /**< the device name */
		int baudrate;                   /**< the baudrate */
		int databits;                   /**< number of databits */
		int stopbits;                   /**< number of stopbits */
		int parity;                     /**< the parity */

	private:	// private methods
		int setNewSettings(int baudrate, int databits, const int parity, const int stopbits,
				bool softwareHandshake, bool hardwareHandshake);

	public:		// public methods
		CommSerial();
		int openSerial(const char* device, int br=115200, int db=8, int par=PARITY_NONE, int sb=1);


		~CommSerial();
		void closeSerial();
		int serialTx( char *data, int size );
		int serialTxByte( char data );
		int serialRxByte( char *data );
		int getRTS(void);
		void setRTS(int state);
		int fileDescriptor( void );
};

#endif
