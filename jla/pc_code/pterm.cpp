//
// pterm, initial version
//
// TOS, January 2011
//

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include "CommSerial.h"

#define MAJOR_VERSION 0
#define MINOR_VERSION 1

#define default_dev_name "/dev/ttyUSB0"

static volatile int must_exit;

static void exit_request(int dummy)
{
  signal(SIGINT,exit_request);
  signal(SIGTERM,exit_request);
  must_exit = 1;
}

static int parse_com_spec(char *com_spec,int *d,const char *where)
{
  char *s;

  for(s = com_spec;*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r';s++)
    ;
  if(*s < '1' || *s > '9')
  {
x:  fprintf(stderr,"bad com_spec in the %s (%s)\n",where,com_spec);
    exit(1);
  }
  for(d[0] = (int)(*s++ - '0');*s >= '0' && *s <= '9';s++)
  {
    d[0] = 10 * d[0] + (int)(*s - '0');
    if(d[0] > 100000000)
      goto x;
  }
  if(*s != ',')
  { // baud_rate only
    while(*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r')
      s++;
    if(*s != '\0')
      goto x;
    return 0;
  }
  else
  { // full spec
    s++;
    switch((int)*s++)
    {
      case 'n': case 'N': d[1] = PARITY_NONE; break;
      case 'e': case 'E': d[1] = PARITY_EVEN; break;
      case 'o': case 'O': d[1] = PARITY_ODD;  break;
      default: goto x;
    }
    if(*s++ != ',')
      goto x;
    switch((int)*s++)
    {
      case '7': d[2] = 7; break;
      case '8': d[2] = 8; break;
      default: goto x;
    }
    if(*s++ != ',')
      goto x;
    switch((int)*s++)
    {
      case '1': d[3] = 1; break;
      case '2': d[3] = 2; break;
      default: goto x;
    }
    while(*s == ' ' || *s == '\t' || *s == '\n' || *s == '\r')
      s++;
    if(*s != '\0')
      goto x;
    return 1;
  }
}

static void banner(void)
{
  printf("DETPIC32 terminal emulator, V%d.%d (October, 2011)\n", MAJOR_VERSION, MINOR_VERSION);
  printf("Universidade de Aveiro, DETI\n");
  printf("TOS, 2011\n");
  printf("Send bug reports to tos@ua.pt\n");
}

static void help(void)
{
  banner();
  fprintf(stderr,"\n");
  fprintf(stderr,"Usage: pterm [-p device_name] [com_spec]\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"com_spec is either\n");
  fprintf(stderr,"    baud_rate\n");
  fprintf(stderr,"or\n");
  fprintf(stderr,"    baud_rate,parity,data_bits,stop_bits\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"baud_rate is one of 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600,\n");
  fprintf(stderr,"                    115200, 230400, 460800, 576000, 921600\n");
  fprintf(stderr,"parity is one of N (none), E (even), O (odd)\n");
  fprintf(stderr,"data_bits is one of 7, 8\n");
  fprintf(stderr,"stop_bits is one of 1, 2\n");
  fprintf(stderr,"\n");
  fprintf(stderr,"the default com_spec is 115200,N,8,1\n");
  fprintf(stderr,"the default device_name is " default_dev_name "\n");
  exit(1);
}

int main(int argc,char **argv)
{
  char c,*dev_name,*home,dev_name_buffer[256],file_name[256];
  int baud_rate,data_bits,stop_bits,parity;
  struct termios term_data;
  struct timeval time_out;
  int i,fd1,fd2,d[4];
  fd_set read_mask;
  CommSerial cs;
  FILE *fp;

  // get device name
  dev_name = (char *)default_dev_name;
  for(i = 1;i < argc - 1;i++)
    if(argv[i] != NULL && argv[i + 1] != NULL && argv[i][0] == '-' && argv[i][1] == 'p')
    {
      if(strlen(argv[i + 1]) >= sizeof(dev_name_buffer))
      {
        fprintf(stderr,"ERROR: device name (%s) too long\n",argv[i + 1]);
        return 1;
      }
      strcpy(dev_name_buffer,argv[i + 1]);
      dev_name = dev_name_buffer;
      argv[i] = NULL;
      argv[i + 1] = NULL;
    }
  // get serial line configuration parameters
  baud_rate = 115200;
  data_bits = 8;
  stop_bits = 1;
  parity = PARITY_NONE;
  home = getenv("HOME");
  if(home != NULL && strlen(home) + 20 < sizeof(file_name))
  { // try configuration file
    sprintf(file_name,"%s/.ptermrc",home);
    fp = fopen(file_name,"r");
    if(fp != NULL)
    {
      if(fgets(file_name,sizeof(file_name),fp) != NULL)
      {
        if(parse_com_spec(file_name,d,"configuration file") != 0)
        {
          parity = d[1];
          data_bits = d[2];
          stop_bits = d[3];
        }
        baud_rate = d[0];
      }
      fclose(fp);
    }
  }
  for(i = 1;i < argc && (argv[i] == NULL || argv[i][0] == '-');i++)
    ;
  if(i < argc)
  { // try command line
    if(parse_com_spec(argv[i],d,"command line") != 0)
    {
      parity = d[1];
      data_bits = d[2];
      stop_bits = d[3];
    }
    baud_rate = d[0];
    argv[i] = NULL;
  }
  // deal with unused arguments
  for(i = 1;i < argc;i++)
    if(argv[i] != NULL)
      help();
 // display configuration
#if 0
  printf("dev_name .... %s\n",dev_name);
  printf("baud_rate ... %d\n",baud_rate);
  printf("parity ...... %d\n",parity);
  printf("data_bits ... %d\n",data_bits);
  printf("stop_bits ... %d\n",stop_bits);
#endif
  // sanity check
  if(isatty(fileno(stdin)) == 0)
  {
    fprintf(stderr,"ERROR: standard input is not a tty device\n");
    return 1;
  }
  if(isatty(fileno(stdout)) == 0)
  {
    fprintf(stderr,"ERROR: standard output is not a tty device\n");
    return 1;
  }
  // banner
  banner();
  // initialization
  if(cs.openSerial(dev_name,baud_rate,data_bits,parity,stop_bits) < 0)
  {
    fprintf(stderr,"ERROR: unable to open %s\n",dev_name);
    return 1;
  }
  fd1 = cs.fileDescriptor();
  // disable terminal echo
  fd2 = fileno(stdin);
  tcgetattr(fd2,&term_data);
  term_data.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(fd2,TCSANOW,&term_data);
  // main loop
  signal(SIGINT,exit_request);
  signal(SIGTERM,exit_request);
  for(must_exit = 0;must_exit == 0;)
  {
    FD_ZERO(&read_mask);
    FD_SET(fd1,&read_mask);
    FD_SET(fd2,&read_mask);
    time_out.tv_sec = 1;  // wake-up the program every second
    time_out.tv_usec = 0; // (currently, not used to do anything useful)
    if(select(1 + (fd1 > fd2 ? fd1 : fd2),&read_mask,NULL,NULL,&time_out) > 0)
    {
      if(FD_ISSET(fd1,&read_mask))
      {
        if(cs.serialRxByte(&c) != 0)
          printf("\007[SERIAL PORT READ ERROR]");
        else
        {
          printf("%c",c);
          if(c == '\n')
            printf("\r");
        }
        fflush(stdout);
      }
      if(FD_ISSET(fd2,&read_mask))
      {
        if(read(fd2,&c,1) != 1)
        {
          printf("\007[TERMINAL READ ERROR]");
          fflush(stdout);
        }
	if(c == 18)
	{
		cs.setRTS(1);
		usleep(1000);
		cs.setRTS(0);
	}
        if(c == 127)
          c = '\b'; // replace DEL by BS
        cs.serialTxByte(c);
      }
    }
  }
  // clean up
  tcgetattr(fd2,&term_data);
  term_data.c_lflag |= ICANON | ECHO;
  tcsetattr(fd2,TCSANOW,&term_data);
  cs.closeSerial();
  printf("\nGoodbye!\n");
  return 0;
}


