#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "CommSerial.h"


struct Coordinate{
    int x;
    int y;
};

struct ChairInfo{
    int connected;
    int velocity;
    int battery;
};

class CommHandler
{
	private:
        CommSerial comm;
        int velocityInfo;
        char aux[1024 * 4];

    // Methods
	private:
		void buildFrame(Coordinate joystick, int buttonPressed);
        void printFrame(char* response);

	public:		
		CommHandler();
		void sendFrame(Coordinate joystick, int buttonPressed);
        ChairInfo receiveFrame();
        ChairInfo parseFrame(char* response);

		~CommHandler();
		
};

