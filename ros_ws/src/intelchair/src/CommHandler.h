#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "CommSerial.h"


struct Coordinate{
    int x;
    int y;
};

class CommHandler
{
	private:
        CommSerial comm;
        int velocityInfo;
        int connectInfo;
        char aux[1024 * 4];

    // Methods
	private:
		void buildFrame(Coordinate joystick, int buttonPressed, int connectOption);

	public:		
		CommHandler();
		void sendFrame(Coordinate joystick, int buttonPressed, int connectOption);
        void receiveFrame();

		~CommHandler();
		
};

