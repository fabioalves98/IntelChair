//*************************************************************************
//
// wheel_chair_driver.c
//
//*************************************************************************
//
#include "pic32_uart.c"
#include "pic32_uart_device_driver.c"

#define  TRUE        1
#define  FALSE       0
#define  ON          1
#define  OFF         0

#define DEBUG        0  // 1: debug mode

#define BUF_SIZE     64

#define UART_WCHAIR  3

// UART Ports
#if (UART_WCHAIR == 3)
   #define TX_TRIS_PORT TRISGbits.TRISG8
   #define TX_LAT_PORT  LATGbits.LATG8
#elif (UART_WCHAIR == 2)
   #define TX_TRIS_PORT TRISFbits.TRISF5
   #define TX_LAT_PORT  LATFbits.LATF5
#else
   #error "Wrong UART definition"
#endif

// LED Ports
#define LED0_TRIS TRISEbits.TRISE0
#define LED0      LATEbits.LATE0
#define LED1_TRIS TRISEbits.TRISE1
#define LED1      LATEbits.LATE1
#define LED2_TRIS TRISEbits.TRISE2
#define LED2      LATEbits.LATE2

// Macros
#define	asc2bin(a) ((a) - '0')
#define BlinkLED() (LED2 = ((loopCounter % 50 < 5)))
	
// Structure for Joystick position
typedef struct 
{
   char x;
   char y;
   char buttonPressed;
   char connectOption;
} virtualJoystick;

//*************************************************************************
// Global variables
#if (DEBUG == 1)
   int velocity = 1;	                  // Variable to store velocity. Varies from 1 to 5
   int battery = 0;                    // Variable to store battery level
#endif

static int powerStatus = OFF;
static int loopCounter = 0;

/*************************************************************************/
/*************************************************************************/
void delay(unsigned int tenth_ms)
{
   tenth_ms = tenth_ms > 500000 ? 500000 : tenth_ms;

   resetCoreTimer();
   while(readCoreTimer() <= (2000 * tenth_ms));
}

//*************************************************************************
// Function to calculate the frame checksum
unsigned char calcCs(unsigned char *data, int n)
{
   int i;
   unsigned char cs = 0;      // Variable to hold checksum of message

   for(i = 0; i < n; i++)     // Sum of all the data to be sent in the frame 
      cs += data[i];
   return 0xFF - cs;          // Calculation of final checksum to be appended to frame
}

//*************************************************************************
void buildFrame(char *data, unsigned char straightMovement, unsigned char sidewaysMovement, unsigned char buttonPressed)
{
   data[0] = 0x4A;             // Marker to signal beggining of new input frame (on motor side)
   data[1] = buttonPressed;    // Buttons pressed
   data[2] = 0x00;             // Has no actual use in the type of wheel chair we use
   data[3] = straightMovement; // Speed to be aplied in the y axis
   data[4] = sidewaysMovement; // Speed to be aplied in rotational form
   data[5] = calcCs(data, 5);  // frame checksum
}

//*************************************************************************
void sendFrame(int uart_nr, unsigned char *frame, int n, char firstFrame)
{
   static char tag[7];		                  // Last correct response frame
   static char tagTest[7];	                  // Frame validating the response we are seeing
   int i, errFlags;
   char ch;

   for(i = 0; i < n; i++)
   {
      txChar(uart_nr, frame[i]);
      ch = rxChar(uart_nr, &errFlags); // Flush the character from PIC32 buffer (we dont use this information)
   }

   delay(5);                  // Required by message pattern
   disableUartTx(uart_nr);    // The next 3 lines are used to free the uart_nr in order

   TX_LAT_PORT = 0;           // for the motor to respond to the frame sent
   TX_TRIS_PORT = 0;          // make the port an output
   resetCoreTimer();          // Again required by message pattern
   while(readCoreTimer() < 10);	  
   TX_TRIS_PORT = 1;          // Definition of port as input 

   if(firstFrame == TRUE)     // In case it is the startup frame we need to wait a bit because the 
      delay(20);              // message is smaller and the time is required by the message pattern
   else
   {
      // Read message from motor
      for(i = 0; i < 7; i++)
      {
         tagTest[i] = rxChar(uart_nr, &errFlags);
         if((tagTest[0] & 0xFF) != 0xFE)
            i = 7;
      }
      // Validation of message received
      if((tagTest[0] & 0xFF) == 0xFE)
      {
         for(i = 0; i < 7; i++)
            tag[i] = tagTest[i];
      }
      // Print message received - Necessary for MainWWindow.cpp program to readResponse functions
      for(i = 0; i < 7; i++)
         printInt(tag[i] & 0xFF, 2 << 16 | 16);
      printf("\n");

#if(DEBUG == 1)
      if(DEBUG)
      {
         // Store of Max wheel velocity received from motor
         velocity = ((tag[5] & 0xF0) >> 4);  

         // Conversion of velocity value to scale from joystick (lights)
         if(velocity == 3){
            velocity = 1;
         }else if(velocity == 5){
            velocity = 2;
         }else if(velocity == 7){
            velocity = 3;
         }else if(velocity == 9){
            velocity = 4;
         }else{
            velocity = 5;
         }
         // Store of Battery level
         battery = ((tag[4] & 0xF0) >> 4);
         printf(" - ");
      } else
         printf("\n");
#endif
      delay(5);
   }
   enableUartTx(uart_nr);     // Enable Tx in Uart n
   delay(50);                 // Required by message pattern
}

//*************************************************************************
int readCommand(virtualJoystick *js)
{
   static char frame[BUF_SIZE];
   int count = 0;
   char ch;

   // Aquisition of string sent from the pc, with the instructions to move the chair
   while ((commDrv_getChar(&ch) != 0) && (count <= 11))
   {
      if(ch == '#')
         count = 0;
      frame[count] = ch;
      count++;
   }
   // Data aquisition
   if ((count == 11)  && (frame[0] == '#'))
   {
      js->connectOption = asc2bin(frame[10]);
      js->buttonPressed = asc2bin(frame[9]);
      js->x = asc2bin(frame[2]) * 100 + asc2bin(frame[3]) * 10 + asc2bin(frame[4]);

      if (frame[1] == '-')
         js->x = -js->x;

      js->y = asc2bin(frame[6]) * 100 + asc2bin(frame[7]) * 10 + asc2bin(frame[8]);
      
      if (frame[5] == '-')
         js->y = -js->y;
      return TRUE;   // Valid read of data
   }
   return FALSE;     // No valid read of data
}

//*************************************************************************
void powerOffSequence(int uart_nr)
{
   static char frame[10];        // Variable to hold the instruction frame to send to the wheel chair motor
   char pressPower = 0xC0;       // Char representing the press of POWER button from physical command
   char releasePower = 0x80;     // Char representing the release of POWER button from physical command

   // Send frame with press POWER button
   buildFrame(frame, 0, 0, pressPower);
   sendFrame(uart_nr, frame, 6, FALSE);
   // Send frame with release POWER button
   buildFrame(frame, 0, 0, releasePower);
   sendFrame(uart_nr, frame, 6, FALSE);

   powerStatus = OFF;      		// Set powerStatus off
}

//*************************************************************************
void powerOnSequence(int uart_nr)
{
   static char frame[10];                                // Variable to hold the command frame to send to 
                                                         // the wheel chair motor controller
   static unsigned char startupFrame[] = {0x53, 0xAC};   // Frame to start the wheel chair
   int i;

   // Send START UP frame
   sendFrame(uart_nr, startupFrame, 2, TRUE);
 
   // Wait time in stop mode
   for (i = 0; i < 100; i++)
   {
      buildFrame(frame, 0, 0, 0);
      sendFrame(uart_nr, frame, 6, TRUE);
   }
   powerStatus = ON;         // Set powerStatus on
}

/*************************************************************************/
void initPorts(void)
{
   LED0 = LED1 = LED2 = 0;
   LED0_TRIS = LED1_TRIS = LED2_TRIS = 0;
}

/*************************************************************************/
int main(void)
{
   int buttonPressed = 0;     // Variable to hold the information of a button being pressed
   virtualJoystick js;        // Declaration of struture to hold the jss
   int straightMovement = 0;  // Control the movement forward and backwards
   int sidewaysMovement = 0;	// Control the movement to the right and left
   int connect_disconnect = 0;// Variable to store the input to connect and disconnect the wheelchair
   char frame[25];            // Variable to hold the instruction frame to send to the wheel chair motor


   initPorts();
   // Configuration of UART to communicate with Wheel Chair
   delay(1000);	                     
   configUart(UART_WCHAIR, 38700, 'E', 1);

   // Configuration of USB UART to communicate with PC
   commDrv_config(115200, 'N', 1);
   EnableInterrupts();              // Enable Interrupts

   // Start Up Sequence
   powerOnSequence(UART_WCHAIR);

   // Main cycle (aquisition of data string and forwarding to wheelchair motor)
   while(TRUE)
   {
      if (readCommand(&js))                  // In case there was a valid read
      {
         loopCounter++;
         BlinkLED();
         straightMovement = js.x;            // Assign straight Movement
         sidewaysMovement = js.y;            // Assign Rotational Movement
         buttonPressed = js.buttonPressed;       // Assign Button Pressed
         connect_disconnect = js.connectOption;  // Assign Connect/Disconnect option
      }
      // Connect/Disconnect Cycle
      if(connect_disconnect == 1)            // If it was received the instruction to Connect to wheel chair
      {
         powerOffSequence(UART_WCHAIR);      // Send power off sequence to assure the connection is not set
         delay(100);                         // Had a small delay to insure the wheel chair responds to the last instruction
         powerOnSequence(UART_WCHAIR);       // Send the power on sequence
         connect_disconnect = 0;
      }
      else if(connect_disconnect == 2)       // If it was received the instruction to Disconnect the Wheel Chair
      {                                      // JLA: nunca recebe 2...
         LED1 = 1;
         powerOffSequence(UART_WCHAIR);      // Send the power off sequence
         connect_disconnect = 0;
      }

      if (powerStatus == ON)     
      {   
#if (DEBUG == 1)
         // Show message to be sent to motor (velocity and battery from the response and are only updated on the next sendFrame)
         printf("\nSTM: %3d RTM: %3d VEL: %1d BAT: %2d, BUT: %1d, CON: %1d - ", 
                     straightMovement, sidewaysMovement, velocity, battery, buttonPressed, connect_disconnect);
#endif
         // Build frame and sent it to the motor controller
         buildFrame(frame, straightMovement, sidewaysMovement, buttonPressed);
         sendFrame(UART_WCHAIR, frame, 6, FALSE);
      }
      // Reset of buttonPressed variable to not spam the motor with reduce/increase speed when we just press one time
      if (buttonPressed != 0)
         buttonPressed = 0;
   }
}

