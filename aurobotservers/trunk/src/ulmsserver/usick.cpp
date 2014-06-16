/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <termios.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <pthread.h>
//#include <sys/ioctl.h>   /* _SGIAPI FIO* functions  */

//#include <unocv4/ucommon.h>
#include <ugen4/ucommon.h>

#include "usick.h"
/** every packet is started by STX*/
#define STX     0x02
/**every packet is acknowledged with 0x06 if ok*/
#define ACKSTX  0x06
/** The packet is not acknowledged */
#define NACK    0x15
/** Address of scanner on line (just one) */
#define LMSADR  0x00
/**
Macro to extract telegram size from telegram definition */
#define TSIZE(telegram) (telegram[2]+telegram[3]*256+6)
/**
CRC check generator polynomie */
#define CRC16_GEN_POL 0x8005
/**
Macro to convert to short */
#define MKSHORT(a,b) ((unsigned short)(a) | ((unsigned)(b) << 8))


USick::USick()
: ULaserDevice()
{
  //running = false;
  LMS_fd = -1;
  //verbose = false;
  packsNew = 0;
  //threadStop = true;
  //threadRunning = false;
  //datalog = NULL;
  angleResolution = 1.0;
  modeAngleScan = 180;
  modeSimulated = false;
  lostBytes = 0;
  wastedData = 0;
  byteCnt = 0;
  //statGoodCnt = 0;
  //statBadCnt = 0;
  //statMsgRate = 0.0;
  // set default device
  setDeviceName("/dev/ttyS4");
  strncpy(name, "SICK", MAX_NAME_LNG);
  //setDeviceName("/dev/ttyS0");
  maxValidRange = 8.05;
  portSpeed = SLOWSERIAL_BAUDRATE;
  serialspeed = SLOWSERIAL;
}

///////////////////////////////////////////////////


USick::~USick()
{ // stop thread and close serial port
  stop(false);
}

///////////////////////////////////////////////////


/* Lists differents telegrams. The last two bytes will be replaced with CRC16
upon time of transmission, the ones are inserted to maintain the correct length. Bytes
three and four represents the length of the telegram and equals the total length take 6, since the
CRC16 bytes and the first four are not included in the total length*/

unsigned char LMSreset[] = {STX,LMSADR,0x01,0x00,0x10,1,1};
unsigned char LMSstatus[] = {STX,LMSADR,0x01,0x00,0x31,1,1};
unsigned char LMSinstallation_mode[] = {STX,LMSADR,0x0A,0x00,0x20,0x00,'S','I','C','K','_','L','M','S',1,1};
unsigned char LMSset_500k_baud[] = {STX,LMSADR,0x02,0x00,0x20,0x48,1,1};
unsigned char LMSset_9k6_baud[]   ={STX,LMSADR,0x02,0x00,0x20,0x42,1,1};
unsigned char LMSrequestfieldA[] = {0x02,0x00,0x03,0x00,0x45,0x01,0x00,0xa6,0xd2};
unsigned char LMSfieldActive[] = {0x02,0x00,0x02,0x00,0x41,0x00,0xD2,0x69};
unsigned char LMScontinous_mode[] = {STX,LMSADR,0x02,0x00,0x20,0x24,1,1};
unsigned char LMSrequest_scan[] = {STX,LMSADR,0x02,0x00,0x30,0x01,1,1};
unsigned char LMScontinous_mode_stop[] = {STX,LMSADR,0x02,0x00,0x20,0x25,1,1};


unsigned char LMSset_configuration[]=
{
  STX,
  LMSADR,
  0x21,0x00,  /* Length bytes, Low byte,high byte*/
  0x77,       /* Command byte, configuration mode */
  0x00,0x00,  /* Minimum size of objects to be detected*/
  0x46,0x00,  /* Threshold in mV*/
  0x00,       /* Availabillity level*/
  0x02,       /* Measurement mode, field A,B and C is chosen*/
  0x01,       /* Unit mode: 0 for cm and 1 for mm and */
  0x00,       /* Temporary field: 0 */
  0x00,       /* Treat field as subtractive, no: 0 */
  0x02,       /* Multiple evaluation: 2 */
  0x02,       /* Restart: No restart block 0 */
  0x02,       /* Restart time: 2*/
  0x00,       /* Object evaluation: 0 */
  0x00,       /* Contour A as reference, not active: 0*/
  0x0A,0x0A,0x50,0x64,  /*Setup conditions for contour, not relevant if not active*/
  0x00,       /* Contour B as reference, not active: 0*/
  0x0a,0x0a,0x50,0x64,  /*Setup conditions for contour, not relevant if not active*/
  0x00,       /* Contour C as reference, not active: 0*/
  0x0a,0x0a,0x50,0x64,  /*Setup conditions for contour, not relevant if not active*/
  0x00,       /* Pixel oriented evaluation: 0*/
  0x00,       /* Mode for single measured value evaluation */
  0x00,0x00,  /* Reserved */
  1,1         /* CRC16 bytes*/
};


unsigned char LMSfieldA_radial[]=
{
  STX,LMSADR,
  0x10,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x40,       /* Field A i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x01,       /* Field type: Radial*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0xfe,0x00,  /* Radius of circle i mm, lowbyte, highbyte: 160 mm  */
  1,1         /* CRC16 bytes*/
};
unsigned char LMSfieldB_radial[]=
{
  STX,LMSADR,
  0x10,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x41,       /* Field B i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x01,       /* Field type: Radial*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0xA0,0x00,  /* Radius of circle i mm, lowbyte, highbyte: 160 mm  */
  1,1         /* CRC16 bytes*/
};
unsigned char LMSfieldC_radial[]=
{
  STX,LMSADR,
  0x10,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x42,       /* Field C i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x01,       /* Field type: Radial*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0xE8,0x03,  /* Radius of circle i mm, lowbyte, highbyte: 1000 mm  */
  1,1         /* CRC16 bytes*/
};
unsigned char LMSfieldA_rect[]=
{
  STX,LMSADR,
  0x14,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x40,       /* Field A i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x00,       /* Field type: rectangular*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0xE1,0x00,  /* Left distance from sensor i mm, lowbyte, highbyte: 225 mm  */
  0xE1,0x00,  /* Right distance from sensor i mm, lowbyte, highbyte: 225 mm  */
  0xb4,0x00,  /* front distance from sensor i mm, lowbyte, highbyte: 180 mm  */
  1,1         /* CRC16 bytes*/
};
unsigned char LMSfieldB_rect[]=
{
  STX,LMSADR,
  0x14,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x41,       /* Field B i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x00,       /* Field type: rectangular*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0x45,0x01,  /* Left distance from sensor i mm, lowbyte, highbyte: 325 mm  */
  0x45,0x01,  /* Right distance from sensor i mm, lowbyte, highbyte: 325 mm  */
  0xFA,0x00,  /* front distance from sensor i mm, lowbyte, highbyte: 250 mm  */
  1,1         /* CRC16 bytes*/
};

unsigned char LMSfieldC_rect[]=
{
  STX,LMSADR,
  0x14,0x00,  /* Length bytes, Low byte,high byte*/
  0x40,       /* Command byte, field configuration*/
  0x01,       /* Fieldset 1*/
  0x42,       /* Field C i mm*/
  0xB4,0x00,  /* Scanning angle 180 or 100*/
  0x32,0x00,  /* Resolution: 100 = 1�, 50 = 0.5�, 25 = 0.25�*/
  0x00,       /* Field type: rectangular*/
  0x00,       /* Minute: 0  - not used*/
  0x00,       /* Hour: 0 - not used*/
  0x01,       /* Day: 1 - not used*/
  0x01,       /* Month: 1 - not used*/
  0x68,0x00,  /* Year - not used*/
  0x36,0x01,  /* Left distance from sensor i mm, lowbyte, highbyte: 310 mm  */
  0x36,0x01,  /* Right distance from sensor i mm, lowbyte, highbyte: 310 mm  */
  0x36,0x01,  /* front distance from sensor i mm, lowbyte, highbyte: 310 mm  */
  1,1         /* CRC16 bytes*/
};


/////////////////////////////////////////////////////

// void * threadRunSick(void * obj)
// { // call the hadling function in provided object
//   USick * ce = (USick *)obj;
//   ce->threadRunLoop();
//   pthread_exit((void*)NULL);
//   return NULL;
// }

///////////////////////////////////////////////////

// bool USick::start()
// {
//   bool result;
//   pthread_attr_t  thAttr;
//   const int MLN = 100;
//   char logname[MLN];
//   //
//   //
//   running = isRunning();
//   if (not running)
//   {
//     if (modeSimulated)
//       running = true;
//     else
//     { // open port
//       if (LMS_fd < 0)
//       {
//         LMS_fd = open_port();
//         printf("Opening port - result=%s\n",
//             bool2str(LMS_fd >= 0));
//       }
//       result = LMS_fd >= 0;
//       if (result)
//          changeMode(modeAngleScan, modeAngleResolution);
//       /*
//       if (result)
//       { // set low speed
//         set_serial(LMS_fd, SLOWSERIAL_BAUDRATE);
//         // flush unread data
//         tcflush(LMS_fd, TCIFLUSH);
//
//         result = set_slow_speed();
//       }
//       if (result)
//         result = stop_continous_mode();
//       if (result)
//       { // set default mode
//         //result = set_resolution(180, 50, 5); // 180 0.5 deg mode
//         //result = set_resolution(100, 50, 5); // 100 0.5 deg mode
//         result = set_resolution(180, 100, 5);  // 1 deg mode
//         //result = set_resolution(100, 100, 5); // 100 1.0 deg mode
//         //NB! result = set_resolution(180, 25, 5);  // 180 0.25 deg mode - not working in Sick
//         //result = set_resolution(100, 25, 5); // 100 0.25 deg mode
//       }
//       if (result)
//         result = set_fast_speed();
//       if (result)
//         result = enter_continous_mode();
//       if (not result)
//         close_port();
//       */
//       running = result;
//     }
//   }
//   //
//   // debug
//   if (false)
//   {
//     snprintf(logname, MLN, "%s/laserrxlog.txt", dataPath);
//     datalog = fopen(logname, "w");
//   }
//   // debug end
//   //
//   if (running and not threadRunning);
//   {
//     pthread_attr_init(&thAttr);
//     //
//     threadStop = false;
//     // create socket server thread
//     result = (pthread_create(&threadHandle, &thAttr,
//                   &threadRunSick, (void *)this) == 0);
//   }
//
//   return result;
// }


///////////////////////////////////////////////////

bool USick::changeMode(int scanangle, double resolution)
{
  bool result = true;
  const int MRT = 15; // max retry loops
  int i;
  //
  if (resolution < 0.4 and scanangle > 100)
  {
    printf("resolution 0.25 deg is supported in 100 deg mode only\n");
    result = false;
  }
  else
  {
    //
    if (LMS_fd >= 0)
    { // set low speed
      printf("USick::changeMode: setting to %d b/s"
          " (dev='%s' LMS_fd=%d)\n", SLOWSERIAL_BAUDRATE, devName, LMS_fd);
      // lock message receive before change of mode
      lock();
      // set serial port
      // result = set_serial(LMS_fd, SLOWSERIAL_BAUDRATE);
      printf("Setting serial speed to slow (38400)\n");
      set_serial(LMS_fd, 38400);
      for (i = 0; i < MRT; i++)
      {
        if (LMS_fd < 0)
          break;
        if (true)
        { // flush unread data
          tcflush(LMS_fd, TCIFLUSH);
          // send low-speed to SICK and then change port speed
          printf("Setting to low speed i=%d\n", i);
          result = set_slow_speed();
          if (not result)
          { // try to set serial port to anouther baudrate, as the sick-device
            // can have another default setting than 9600 bit/sec
            //printf("set serial i=%d (%d)\n", i, (i/2) %4);
            switch ((i/3) % 5)
            {
              case 1:
                printf("Setting serial speed to slow (19200)\n");
                set_serial(LMS_fd, 19200);
                break;
              case 2:
                printf("Setting serial speed to fast (500000)\n");
                set_serial(LMS_fd, 500000);
                break;
              //case 3: set_serial(LMS_fd, 115200); break;
              case 4:
                printf("Setting serial speed to slow (9600)\n");
                set_serial(LMS_fd, 9600);
                break;
              default:
                printf("Setting serial speed to slow (38400)\n");
                set_serial(LMS_fd, 38400);
                break;
            }
          }
        }
        if (result)
          result = stop_continous_mode();
        if (result)
        { // set default mode (angle in centi-degrees)
          result = set_resolution(scanangle,
                        roundi(resolution * 100.0), 5);
          //result = set_resolution(180, 50, 5); // 180 0.5 deg mode
          //result = set_resolution(100, 50, 5); // 100 0.5 deg mode
          //result = set_resolution(180, 100, 5);  // 1 deg mode
          //result = set_resolution(100, 100, 5); // 100 1.0 deg mode
          //NB! result = set_resolution(180, 25, 5);  // 180 0.25 deg mode - not working in Sick
          //result = set_resolution(100, 25, 5); // 100 0.25 deg mode
        }
        if (result)
          result = set_fast_speed();
        if (result)
          result = enter_continous_mode();
        if (result)
          // all is OK
          break;
      }
      // allow receive of data
      unlock();
    }
    else
    { // port is closed
      angleResolution = resolution;
      modeAngleScan = scanangle;
      result = true;
    }
  }
  return result;
}

/////////////////////////////////////////////////////////////////

// void USick::stop(bool justClosePort)
// {
//   if (LMS_fd >= 0)
//   { // stop contiious mode and close serial port
//     stop_continous_mode();
//     set_slow_speed();
//     close_port();
//   }
//   if (threadRunning and not justClosePort)
//   {
//     threadStop = true;
//     pthread_join(threadHandle, NULL);
//   }
//   if (datalog != NULL)
//     fclose(datalog);
// }

///////////////////////////////////////////////////


bool USick::set_fast_speed()
{
  bool result = true;
  const int repeats = 7;
  USickData * data; // reply

  if (serialspeed == SLOWSERIAL)
  {
    data = LMS_send_receive(LMSset_500k_baud, repeats);
    if(data != NULL)
      printf("Speed set to %d bps\n", FASTSERIAL_BAUDRATE);
    else
    { // try set speed on port
      result = set_serial(LMS_fd, FASTSERIAL_BAUDRATE);
      if (result)
      {
        data = LMS_send_receive(LMSset_500k_baud, repeats);
        if(data != NULL)
          printf("Speed set to %d bps\n", FASTSERIAL_BAUDRATE);
        else
        {
          printf("Speed not set successfully\n");
          result = false;
        }
      }
    }
  }
  else if (serialspeed == FASTSERIAL)
  {
    data = LMS_send_receive(LMSset_500k_baud, repeats);
    if(data != NULL)
      printf("Speed set to 500kbps\n");
    else
    { // try slow speed on port
      result = set_serial(LMS_fd, SLOWSERIAL_BAUDRATE);
      if (result)
      {
        data = LMS_send_receive(LMSset_500k_baud, repeats);
        if (data != NULL)
          printf("Speed set to 500kbps\n");
        else
        { // no good reply
          printf("Speed not set successfully.\n");
          result = false;
        }
      }
    }
  }
  if (result)
    Wait(0.002);  /*The LMS must have some rest*/
  //
  return result;
}

//////////////////////////////////////////////////////////////////////

bool USick::set_slow_speed()
{
  bool result = true;
  int repeats = 7;
  USickData * data;
  //
  if (serialspeed == FASTSERIAL)
  {
    data = LMS_send_receive(LMSset_9k6_baud, repeats);
    if(data != NULL)
      printf("Speed set to 9k6bps\n");
    else
    {
      result = set_serial(LMS_fd, SLOWSERIAL_BAUDRATE);
      if (result)
      {
        data = LMS_send_receive(LMSset_9k6_baud, repeats);
        if(data != NULL)
          printf("Speed set to 9k6bps\n");
        else
        {
          printf("Speed not set successfully.\n");
          result = false;
        }
      }
    }
  }
  else if (serialspeed == SLOWSERIAL)
  {
    data = LMS_send_receive(LMSset_9k6_baud, repeats);
    if(data != NULL)
      printf("Speed set to 9k6bps\n");
    else
    {
      result = set_serial(LMS_fd, FASTSERIAL_BAUDRATE);
      if (result)
      {
        data = LMS_send_receive(LMSset_9k6_baud, repeats);
        if(data != NULL)
          printf("Speed set to 9k6bps\n");
        else
        {
          printf("Speed not set successfully.\n");
          result = false;
        }
      }
    }
  }
  if (result)
     Wait(0.001);  /*The LMS must have some rest*/
  return result;
}

////////////////////////////////////////////////////////////////////////

bool USick::set_serial(int fd, int speed)
{ // set speed of serial port on this computer
  bool result = true;
  struct termios options;

  if (tcgetattr(fd, &options) == -1)
  {
    perror("USick::set_serial: tcgetattr(fd, &options) ");
    //exit(-1);
    result = false;
  }
  if (result)
  {
    if (speed != portSpeed)
    {
      printf("Setting SICK serial port to %d (from %d) bit/sec\n", speed, portSpeed);
      portSpeed = speed;
    }
    if (speed == 9600)
    {
      cfsetispeed(&options, B9600);  /*/Set input baudrate */
      cfsetospeed(&options, B9600);  /*/Set output baudrate */
      serialspeed = SLOWSERIAL;
    }
    else if (speed == 19200)
    {
      cfsetispeed(&options, B19200);  /*/Set input baudrate */
      cfsetospeed(&options, B19200);  /*/Set output baudrate */
      serialspeed = SLOWSERIAL;
    }
    else if (speed == 38400)
    {
      cfsetispeed(&options, B38400);  /*/Set input baudrate */
      cfsetospeed(&options, B38400);  /*/Set output baudrate */
      serialspeed = SLOWSERIAL;
    }
    else if (speed == 115200)
    {
      cfsetispeed(&options, B115200);  /*/Set input baudrate */
      cfsetospeed(&options, B115200);  /*/Set output baudrate */
    }
    else if (speed == 500000)
    {
      cfsetispeed(&options, B500000);  /*/Set input baudrate */
      cfsetospeed(&options, B500000);  /*/Set output baudrate */
      serialspeed = FASTSERIAL;
    }
    else
      fprintf(stderr, "Trying to set speed of serial port to unsupported %d bit/sec\n", speed);

    /* Enable the receiver and set local mode... */
    options.c_cflag |= (CLOCAL | CREAD);

    /*/ Set to no parity 8 bit,1 stop, 8N1
    //options.c_cflag &= ~PARENB;
    //options.c_cflag &= ~CSTOPB;
    //options.c_cflag &= ~CSIZE; */
    options.c_cflag |= CS8;
    /*
    //options.c_cflag = 0;
    //options.c_cflag |= (CLOCAL | CREAD | CS8);


    //Using raw input mode
    //options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); */
    options.c_lflag = 0;
    /*
    //ignore parity errors
    //options.c_iflag |= IGNPAR; */
    options.c_iflag =0;  /*/Must be zero, change it at own risk */
    /*
    //Set output to raw mode
    //options.c_oflag &= ~OPOST; */
    options.c_oflag = 0;

    /*/Set the new options for the port... */
    if (tcsetattr(fd, TCSANOW, &options) == -1)
    {
      perror("can not set serial port parameters\n");
      //exit(-1);
      result = false;
    }
  }
  if (result)
    /* flush unread data */
    tcflush(fd, TCIFLUSH);
  //
  return result;
}

////////////////////////////////////////////////////////////////////////

int USick::open_port(void)
{
  int fd;   /* File descriptor for the port */
  /* fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); */
  printf("open_port: Trying to open port %s\n", devName);
  fd = open(devName, O_RDWR | O_NOCTTY );
  if (fd == -1)
  { /* Could not open the port. */
    perror("open_port:: Unable to open port\n");
  }
  else
  { /* fcntl(fd, F_SETFL, FNDELAY);   non blocking by using FNDELAY */
    fcntl(fd, F_SETFL, 0);      /* Blocking */
    printf("open_port:: Sucessfully opened serial port\n");
  }
  tcflush(fd, TCIFLUSH); /* flush unread data */
  return (fd);
}

////////////////////////////////////////////////////////////////////////

void USick::close_port()
{
  int err;
  if (LMS_fd >= 0)
  {
    err = close(LMS_fd);
    if (err != 0)
      perror("Serial laser port close error:");
    LMS_fd = -1;
  }
}

////////////////////////////////////////////////////////////////////////

bool USick::enter_continous_mode()
{
  int repeats = 7;
  USickData * data;

  data = LMS_send_receive(LMScontinous_mode, repeats);
  if(data != NULL)
    printf("Continous mode has been started, prepare yourself...\n");
  else
  {
    printf("Continous mode has NOT started\n");
  }
  return (data != NULL);
}

////////////////////////////////////////////////////////////////////////

bool USick::stop_continous_mode()
{
  int repeats = 7;
  USickData * data;

  data = LMS_send_receive(LMScontinous_mode_stop, repeats);
  if (data != NULL)
    printf("Continous mode has been STOPPED...\n");
  else
  {
    printf("Continous mode has NOT been stopped.\n");
  }
  return data != NULL;
}

////////////////////////////////////////////////////////////////////////

USickData * USick::LMS_send_receive(unsigned char* telegram, int repeats)
{
  USickData * data = NULL;
  int j;
  //
  for (j = 0; j < repeats; j++)
  {
    data = LMS_send_receive(telegram);
    if (data != NULL)
      break;
  }
  return data;
}

////////////////////////////////////////////////////////////////////////

USickData * USick::LMS_send_receive(unsigned char* telegram)
{
  bool result = true;
  unsigned int len;
  unsigned int response_len, i, n;
  unsigned char buf;
  struct pollfd sickstruct;
  USickData * data = NULL;
  unsigned char * buff = NULL;
  //
  sickstruct.fd = LMS_fd;
  sickstruct.events = POLLIN;

  len = TSIZE(telegram);
  addCRC16(telegram);

  /*Sends the telegram on the specified file descripter*/
  if (toLMS(LMS_fd, telegram,len) == -1)
  {
    printf("Communication failed\n");
    result = false;
  } /* /else printf ("Successful transmission of command\n"); */
  if (result)
  {
    i=0;  /* /index to keep track to received packet */
    if (poll(&sickstruct,1,80))
    {
      n = read(LMS_fd,&buf,1);
    }
    if (buf == NACK)
      result = false;
    if (buf != ACKSTX)
      result = false;
    if (result and verbose)
      printf("ACK received\n");
  }
  if (result)
  { /*receiving the normal response packets..*/
    data = getNextBuffLocked();
    result = data != NULL;
  }
  if (result)
  {
    data->setValid(false);
    buff = data->getData();
    i = 0;
    while(i < 5)
    {
      n = poll(&sickstruct,1,50);
      if (n == 1)
      {
        n = read(LMS_fd, &buff[i], 5 - i);
        if (n > 0)
          i += n;
        if (n < 0)
        {
          result = false;
          break;
        }
      }
      else
      { // timeout - stop
        result = false;
        break;
      }
    }
  }
  if (result)
  { // get length
    response_len = buff[2]+ buff[3]*256 +6;
    if(buff[4] != (unsigned char)(telegram[4]+0x80))
    {
      printf("Telegram: 0x%x\n",telegram[4]);
      printf("Response: 0x%x\n",buff[4]);
      printf("Invalid response from SICK, perhaps unknown command\n");
      result = false;
    }
  }
  if (result)
  {  /*Timeout setup edit value at tstop to alter timeout condition*/
    i = 5;
    /*receiving rest of message*/
    while((i < response_len))
    {
      n = poll(&sickstruct, 1, 200);
      if (n == 1)
      {
        n = read(LMS_fd, &buff[i], response_len - i);
        if (n > 0)
          i += n;
        /*Timeout is restartet everytime a new character is received, remember to alter tstop here also*/
        /*/if(i>700)  printf("I: %d  Value: 0x%x\n",i,buf);  */
        i++;
      }
      else
      { // poll timeout - not enough data
        result = false;
        printf("LMS_send_receive:: TIMEOUT at i=%d/%d\n",
                i, response_len);
        break;
      }
    }
  }
  if (result)
  { /* If the command was set serial then change the serial settings of the serial port */
    if((telegram[4]==0x20) and (telegram[5] == 0x42))
      result = set_serial(LMS_fd, SLOWSERIAL_BAUDRATE);
    else if((telegram[4]==0x20) and (telegram[5] == 0x48))
      result = set_serial(LMS_fd, FASTSERIAL_BAUDRATE);
  }
  if (data != NULL)
    data->unlock();
    //setNextBuffUnLocked(data, result);
  // debug
  printf("USick::receiveData returns\n");
  // debug end
  if (result)
    return data;
  else
    return NULL;
}

/////////////////////////////////////////////////////////

void USick::addCRC16(unsigned char* CommData)
{

  unsigned int crc,ulen;

  ulen = TSIZE(CommData);
  crc = getCCRC(CommData);

  CommData[ulen - 2] = crc & 0x00ff;
  CommData[ulen - 1] = crc >> 8;
}

//////////////////////////////////////////////////////////

/*Function writes the received message to the delivered file descripter */

int USick::toLMS(int fd,unsigned char* msg, unsigned int datalen)
{
  int t;
  unsigned int i;
  int result = -1;

  if (fd >= 0)
  {
    tcflush(fd, TCIFLUSH);        /*Empty input buffer, not sure if this is a good idea*/
    t = write(fd, msg, datalen);  /* Write to filedescripter */
    //
    printf("Message send: ");
    for(i = 0; i < datalen; i++) printf("0x%x ",msg[i]);
    printf("\n");
    //
    if (t < 0)
      printf("USick::toLMS: Write failed\n");
    else
      result = 1;
  }
  return result;
}

//////////////////////////////////////////////////////////

USickData * USick::getNextBuffLocked()
{
  USickData * result;
  int i, j;
  //
  j = packsNew;
  for (i = 0; i < PACK_BUF_SIZE; i++)
  {
    j = (j + 1) % PACK_BUF_SIZE;
    result = &packs[j];
    if (result->tryLock())
      break;
    else
      result = NULL;
  }
  if (result)
    packsNew = j;
  return result;
}

//////////////////////////////////////////////////////////

/* Test af data pakkerne der modtages fra SICK sensoren (CRC)
 *  Der unders�ges om der er fejl p� data pakken
 *  Denne funktion er taget fra SICK manualen */
unsigned int USick::getCCRC(unsigned char *CommData)
{
  unsigned int uCrc16;
  unsigned char abData[2];
  unsigned int uLen;

  uLen = TSIZE(CommData)-2;
  uCrc16 = 0;
  abData[0] = 0;

  while(uLen--)
  {
    abData[1] = abData[0];
    abData[0] = *CommData++;
    if (uCrc16 & 0x8000)
    {
      uCrc16 = (uCrc16 & 0x7fff)<<1;
      uCrc16 ^= CRC16_GEN_POL;
    }
    else
    {
      uCrc16 <<=1;
    }
    uCrc16 ^= MKSHORT(abData[0],abData[1]);
  }
  return(uCrc16);
}

////////////////////////////////////////////////////////////////

// void USick::setNextBuffUnLocked(USickData * data, bool valid)
// {
//   USickData * dp;
//   int i;
//   //
//   data->setValid(valid);
//   if (valid)
//   { // setting index to newest data
//     dp = packs;
//     for (i = 0; i < PACK_BUF_SIZE; i++)
//       if (dp == data)
//       {
//         packsNew = i;
//         break;
//       }
//   }
//   data->unlock();
// }

////////////////////////////////////////////////////////////////

USickData * USick::getNewestLocked()
{
  USickData * result = NULL;
  int i;
  //
  for (i = PACK_BUF_SIZE; i > 0 ; i--)
  { // start with 'packsNew' and go backwards until
    // valid data is found
    result = &packs[(packsNew + i) %  PACK_BUF_SIZE];
    if (result->tryLock())
    {
      if (result->isValid())
        break;
      else
      result->unlock();
    }
    result = NULL;
  }
  // debug
  //printf("USick::getNewestLocked: found %d and newest = %d (%s)\n",
  //       (packsNew + i) %  PACK_BUF_SIZE, packsNew, bool2str(result));
  // debug end
  return result;
}

////////////////////////////////////////////////////////////////

bool USick::receiveSimulatedData(int * length)
{
  unsigned char * ps;
  unsigned char * pd;
  int i;
  unsigned int v;
  unsigned char sim361[] = "02 80 d6 02 b0 69 41 cb 08 cb 08 c2 08 c2 08 c3 08 ba 08 b9 08 b0 08 9e 08 89 08 62 08 44 08 21 08 13 08 f8 07 e9 07 e9 07 e2 07 e7 07 e8 07 e5 07 f0 07 e8 07 e8 07 ea 07 e3 07 e3 07 e5 07 d0 07 c2 07 b8 07 c3 07 44 07 32 07 20 07 1f 07 17 07 01 07 2b 07 55 07 5f 07 57 07 43 07 58 07 15 08 b8 08 c2 08 ab 08 44 08 ed 07 a7 07 5e 07 1c 07 d4 06 86 06 82 06 cb 06 11 09 1b 09 24 09 2d 09 36 09 36 09 3f 09 "
    "3f 09 d5 08 34 08 f3 07 07 08 05 08 15 08 1d 08 1a 08 2b 08 15 08 16 08 15 08 09 08 fb 07 d8 07*02 08 ec 07 c7 07 7f 07 90 07 a1 12 75 12 37 10 20 0f e9 0e a5 0e 65 0e 30 0e fc 0d cb 0d 9b 0d 72 0d 3e 0d 11 0d e0 0c b3 0c 93 0c 67 0c 42 0c 25 0c 3d 0c 3e 0c 06 0c ec 0b d9 0b f6 0b f2 0b 0a 0c 17 0c 32 0c 43 0c 5e 0c 79 0c 6d 0c 51 0c 3c 0c 30 0c 31 0c 3f 0c 75 0c cc 0c 0f 0d 1c 0d 89 0d d6 0d 0a 0c c6 0b c9 0d d3 0d cf 0d be 0d a3 0d 93 0d 87 0d 10 0d 6e 0d 92 0d a5 0d ac 0d ac 0d a9 0d 73 0d 6c 0d 60 0d 63 0d 5c 0d 4d 0d 45 0d 36 0d 1c 0d 13 0d d6 0c d5 0c 18 0d 1c 0d 16 0d 1c 0d f1 0c e9 0c e2 0c dc 0c e5 0c d2 0c cd 0c e6 0c d2 0c d8 0c d0 0c db 0c d7 0c e1 0c d5 0c d7 0c dc 0c db 0c d9 0c e4 0c e4 0c e6 0c e2 0c e8 0c e8 0c ec 0c fb 0c ed 0c f1 0c f6 0c fd 0c 03 0d 07 0d 0a 0d 1d 0d 1e 0d 27 0d 36 0d 44 0d 3c 0d 37 0d 41 0d 4a 0d 54 0d 62 0d 5b 0d 62 0d 77 0d 76 0a 71 0a 83 0a 82 0a 81 0a 80 0a 81 0a 82 0a 72 0a 92 0a 98 0d b1 0c bc 0c 26 0c 89 0b 20 0b 3d 0b a8 0b af 0b b4 0b c0 0b c6 0b d0 0b da 0b e2 0b e8 0b ed 0b f8 0b 0a 0c 03 0c 0f 0c 35 0c 41 0c 39 0c 28 0c f2 0b b7 0b 5a 0e cf 0d 92 0b b9 08 45 "
    "08 43 08 ab 08 26 0b 4f 0b 83 0b 30 0c 7f 0d 37 11 68 0f 70 0b 41 0b 7a 22 65 22 63 22 58 22 49 22 43 22 3e 22 33 22 2c 22 25 22 1d 22 15 22 15 22 18 22 07 22 05 22 00 22 ff 21 fd 21 f8 21 f1 21 e3 21 e0 21 d9 21 d2 21 d7 21 cb 21 d0 21 da 21 cf 21 cf 21 d0 21 cb 21 ca 21 c2 21 c2 21 b9 21 b9 21 b4 21 b0 21 af 21 9e 21 97 21 98 21 a4 21 a1 21 9c 21 c2 21 c3 21 bf 21 bf 21 c1 21 b4 21 b8 21 bf 21 c3 21 bd 21 c8 21 ca 21 c0 21 d0 21 d1 21 c5 21 ca 21 d1 21 f0 21 32 22 2f 22 2b 22 2f 22 2a 22 2f 22 2f 22 4f 22 5d 22 5e 22 70 22 86 22 b5 22 cc 22 1c 23 3c 05 48 05 d4 21 2f 23 fe 22 c0 22 2b 23 3c 05 3a 05 33 05 33 05 28 05 1e 05 1b 23 b1 22 d7 22 18 05 10 64 47";
  //
  if (modeSimulated)
  {
    ps = sim361;
    pd = LMS_response_buffer;
    for (i = 0; i < 5; i++)
    {
      sscanf((char *)ps, "%x", &v);
      ps += 3;
      *pd++ = (unsigned char)v;
    }
    *length = LMS_response_buffer[3] * 256 + LMS_response_buffer[2];
    *length += 6; // uncounted overhad.
    for (i = 5; i < *length; i++)
    {
      sscanf((char *)ps, "%x", &v);
      ps += 3;
      *pd++ = (unsigned char)v;
    }
  }
  // wait 1/40 sec as is the laser scan rate
  Wait(1.0/40.0);
  return true;
}

////////////////////////////////////////////////////////////////

// void USick::threadRunLoop()
// {
//   USickData * data;
//   int STAT_INTERVAL = 1000;
//   struct timeval t1, t2;
//   double dt;
//   bool gotData;
//   int length = 0;
//   //
//   threadRunning = true;
//   lostBytes = 0;
//   wastedData = 0;
//   gettimeofday(&t1,NULL);
//   while (not threadStop)
//   {
//     if ((LMS_fd < 0) and not modeSimulated)
//       Wait(0.5);
//     else
//     { // get data from line - one message at a time
//       //
//       // debug
//       // printf("In thread before read (length=%d)\n", length);
//       // debug end
//       //
//       if (modeSimulated)
//         gotData = receiveSimulatedData(&length);
//       else
//         gotData =(&length);
//       //
//       // debug
//       //printf("In thread after (%s) read (length=%d) simulated (%s)\n",
//       //        bool2str(gotData), length, bool2str(modeSimulated));
//       // debug end
//       //
//       if (gotData)
//       {
//         statGoodCnt++;
//         data = getNextBuffLocked();
//         if (data != NULL)
//         {
//           memcpy(data->getData(), LMS_response_buffer, length);
//           data->setTime();
//           setNextBuffUnLocked(data, true);
//         }
//         else
//         {
//           printf("all buffers are locked - this is bad!\n");
//           Wait(1.0);
//         }
//       }
//       else
//       {
//         statBadCnt++;
//       }
//       if(int(statGoodCnt + statBadCnt) == STAT_INTERVAL )
//       {
//         printf("------------------- read staistics -\n");
//         gettimeofday(&t2,NULL);
//         dt = (double)(t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec) * 1e-6;
//         printf("Good packets: %d Bad packets: %d Total: %d (%3.1f%% good)\n",
//                 statGoodCnt, statBadCnt, statGoodCnt + statBadCnt,
//                 (float)(statGoodCnt) / (float)(statGoodCnt + statBadCnt) * 100.0);
//         printf("time used: %f sec  good/sec: %2.1f\n",
//                 dt, (float)(statGoodCnt)/dt);
//         gettimeofday(&t1,NULL);
//         /* debug */
//         printf("Read bytes: %d used, %d lost (packet too short)\n",
//               statGoodCnt * length, lostBytes);
//         printf("            %d wasted (%4.2f%% wasted)\n",
//               wastedData,
//               100.0 * (double)wastedData / (double)(statGoodCnt * length + wastedData));
//         //
//         statMsgRate = double(statGoodCnt)/dt;
//         statGoodCnt = 0;
//         statBadCnt = 0;
//         //
//         lostBytes = 0;
//         wastedData = 0;
//       }
//     }
//   }
//   threadRunning = false;
// }

/////////////////////////////////////////////////////////////////////////

bool USick::receiveData()
{
  bool gotData;
  int length = 0;
  USickData * data;
  // get data from line - one message at a time
  //
  if (modeSimulated)
  {
    gotData = receiveSimulatedData(&length);
    Wait(0.1);
  }
  else
  { // lock TX/RX to serial line to avoid
    // mode change while receiving
    lock();
    gotData = receive_continous_data(&length);
    unlock();
    //
  }
  //
  if (gotData)
  {
    statGoodCnt++;
    data = getNextBuffLocked();
    if (data != NULL)
    { //
      memcpy(data->getData(), LMS_response_buffer, length);
      // assume 500.000 bit/sec and 10 bit/byte
      // so subtract communication time for bytes in buffer plus
      // one scantime at 75 scans per sec
      //data->setTime(double(byteCnt)/(500000.0 / 10.0) + 1.0/75.0);
      data->setTime(var.scanDelay->getValued());
      //setNextBuffUnLocked(data, true);
//      data->setDeviceNum(deviceNum);
      data->setValid(true);
      data->unlock();
      // do any push commands, pending for the new data
      gotNewScan(NULL);
    }
    else
    {
      printf("all buffers are locked - this is bad - skipping new data!\n");
      Wait(2.0);
    }
  }
  // remove used bytes from rx buffer -- error or not
  if ((byteCnt >= length) and (length > 0))
  { // remaining length of buffer
    byteCnt -= length;
    if (byteCnt > 0)
      memmove(LMS_response_buffer,
              &LMS_response_buffer[length],
              byteCnt);
  }
  return gotData;
}

///////////////////////////////////////////////////

bool USick::receive_continous_data(int * length)
{ // get one new message in start of buffer
  int l;
  unsigned int CRCcontrol;
  bool headerfound;
  /*                        |start  80 |-length--|reply| points |
                                         732             361    */
  //unsigned char header[] = {0x02,0x80,0xd6,0x02,0xb0,0x69,0x41};
  const int HDR_LNG = 7;
  // poll wait time
  const int pollTime = 100; // ms
  unsigned char * bp;
  unsigned char * hp; /* header start pointer */
  int n, j;
  int err = -1;
  struct pollfd sickstruct;
  //
  sickstruct.fd = LMS_fd;
  sickstruct.events = POLLIN;
  headerfound = false;
  bool timeout;
  //
  timeout = false;
  while (not timeout)
  { // look for header in remaining message
    if ((byteCnt > HDR_LNG) and (not headerfound))
    { /* look for header from start of message */
      hp = LMS_response_buffer;
      for(l = 0; l < (byteCnt - HDR_LNG); l++)
      { /* check for header */
        headerfound = (hp[0] == 0x02) and // STX
                      (hp[1] == 0x80) and // LMS Adress + 0x80
                      (hp[4] == 0xb0);    // message type 0xb0
        if (headerfound)
        { // calculate message length
          j = hp[3] * 256 + hp[2] + 6;
          headerfound = (j == 732) or // 361 measurements
                (j == 1452) or // 721 measurements
                (j ==  372) or // 181 measurements
                (j == 212) or // 101 measurements
                (j == 412) or // 201 measurements
                (j == 812);   // 401 measurements
        }
        if (headerfound)
        {
          *length = j;
          // debug
          // printf("Found header at %d bytes from start (lng=%d) (%d byteCnt)\n",
          //      hp - LMS_response_buffer, j, byteCnt);
          // debug end
          break;
        }
        hp++;
      }
      if (headerfound and (hp > LMS_response_buffer))
      { /* did not find header at first byte, so move to start of buffer */
        n = hp - LMS_response_buffer; /*/ skipped bytes */
        byteCnt -= n;
        memmove(LMS_response_buffer, hp, byteCnt);
        wastedData += n;
      }
    }
    if (headerfound and byteCnt >= *length)
      // all is fine use data
      break;
    //
    if ((not headerfound) and (byteCnt >= RECEIVE_BUFFER_SIZE))
    { // no reason to get mor garbage - drop data and try a fresh
      printf("*** no SICK header found in %d bytes!!\n", byteCnt);
      wastedData += byteCnt;
      byteCnt = 0;
      statBadCnt++;
    }
    // else get some more data
    bp = &LMS_response_buffer[byteCnt];
    /* Wait for data in up to (100) ms */
    if (poll(&sickstruct, 1, pollTime) != POLLIN)
    { // timeout or other error - return
      timeout = true;
    }
    else
    { // read up to a full buffer
      n = read(LMS_fd, bp, RECEIVE_BUFFER_SIZE - byteCnt);
      if (n == -1)
      {
        perror("Error in read from SICK");
        byteCnt = 0;
        break;
      }
      else
        byteCnt += n;
    }
  }
  // got a message - is it OK?
  if (not timeout and byteCnt > 0)
  {
    CRCcontrol = LMS_response_buffer[*length-2] +
                (LMS_response_buffer[*length-1] << 8);
    if ((CRCcontrol == specialCCRC(LMS_response_buffer,*length - 2)))
    {
      err = 0;
      //printf("USick::receive_continous_data Pakken (%d bytes) klarede CRC check\n", *length);
    }
    else
    {
      wastedData += *length;
      statBadCnt++;
    }
  }
  if (err == 0)
    serial++;
  //
  return err == 0;
}

//////////////////////////////////////////////////////////////////

unsigned short USick::specialCCRC(unsigned char *CommData,unsigned int uLen)
{

  unsigned short uCrc16;
  unsigned char abData[2];

  uCrc16 = 0;
  abData[0] = 0;


  while(uLen--)
  {
    abData[1] = abData[0];
    abData[0] = *CommData++;
    if (uCrc16 & 0x8000)
    {
      uCrc16 = (uCrc16 & 0x7fff)<<1;
      uCrc16 ^= CRC16_GEN_POL;
    }
      else
  {
    uCrc16 <<=1;
  }
      uCrc16 ^= MKSHORT(abData[0],abData[1]);
    }
  return(uCrc16);
}

//////////////////////////////////////////////////////////////////

void USick::enter_installation_mode()
{

  if (LMS_send_receive(LMSinstallation_mode) != NULL)
    printf("Installation mode has been entered, do your thing..\n");
  else
    printf("Installation mode NOT achieved\n");
}

//////////////////////////////////////////////////////////////////

void USick::set_configuration()
{
  if (LMS_send_receive(LMSset_configuration) != NULL)
    printf("Configuration is set\n");
  else
    printf("Configuration is NOT set\n");
  usleep(1000);   /*/Just to be sure, this is not intented to be fast anyway */
}

//////////////////////////////////////////////////////////////////

void USick::request_status()
{
  if((LMS_send_receive(LMSstatus)) != NULL)
    printf("LMS status received..\n");
  else
    printf("LMS status not received\n");
}

//////////////////////////////////////////////////////////////////

bool USick::set_resolution(int scanAngleDeg, int resolutionCdeg, int repeats)
{
  USickData * reply = NULL;
  int i;
  unsigned char LMSresolution[]=
  { /* 180 deg 1 grad => 181 measurements */
    /* 180 deg 0.5 grad => 361 measurements */
    /* 180 deg 0.25 grad => not legal */
    STX,
    LMSADR,
    0x05,0x00,  /* Length bytes, Low byte,high byte*/
    0x3b,       /* Command byte, configuration mode */
    180, 0x00,  /* Scanning angle 180 (0xb4) or 100 (0x64)*/
    100, 0x00,  /* Resolution: 100 (0x64) = 1�, 50 (0x32) = 0.5�, 25 (0x19) = 0.25�*/
    1,1         /* CRC16 bytes*/
  };
  // scan width
  if ((scanAngleDeg == 100) or (scanAngleDeg == 180))
    LMSresolution[5] = (unsigned char)scanAngleDeg;
  else
    printf("set_resolution: not valid scan, must be 180 or 100 - not %d\n",
           scanAngleDeg);
  // resolution
  if ((resolutionCdeg == 25) or (resolutionCdeg == 50) or (resolutionCdeg == 100))
    LMSresolution[7] = (unsigned char)resolutionCdeg;
  else
    printf("set_resolution: not valid resolution, must be 25, 50 or 100 - not %d\n",
           resolutionCdeg);
  // send
  for (i = 0; i < repeats; i++)
  {
    reply = LMS_send_receive(LMSresolution);
    if (reply != NULL)
      break;
  }
  if (reply != NULL)
  {
    printf("Configuration is set (%d) to %d deg scan and %f deg res\n",
          i, scanAngleDeg, double(resolutionCdeg) / 100.0);
    modeAngleScan = scanAngleDeg;
    angleResolution = double(resolutionCdeg) / 100.0;
  }
  else
    printf("Configuration is NOT set\n");
  // usleep(1000);   /*/Just to be sure, this is not intented to be fast anyway */
  return reply != NULL;
}

//////////////////////////////////////////////////////////////////////////

// void USick::print(char * preString)
// {
//   printf("%s running (%s), port %s, mode=%d deg, res=%4.2f deg\n",
//       preString, bool2str(threadRunning), devName,
//       modeAngleScan, double(modeAngleResolution)/100.0);
//   printf("   - Statistics good packs %d bad %d dataRate %6.2f\n",
//       statGoodCnt, statBadCnt, statMsgRate);
// }

//////////////////////////////////////////////////////////////////////////

bool USick::openPort()
{
  bool result;
  //
  if (LMS_fd < 0)
  {
    LMS_fd = open_port();
    printf("Opening port - result=%s\n",
        bool2str(LMS_fd >= 0));
  }
  result = LMS_fd >= 0;
  if (result)
  { // port is open set working mode
    changeMode(modeAngleScan, angleResolution);
    strncpy(name, "SICK", MAX_NAME_LNG);
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////

void USick::closePort()
{
  if (LMS_fd >= 0)
  {
    stop_continous_mode();
    set_slow_speed();
    close_port();
  }
}

//////////////////////////////////////////////////////////////////////////

bool USick::isPortOpen()
{
  return (LMS_fd >= 0);
}

///////////////////////////////////////////////////

bool USick::getNewestData(ULaserData * dest,
                          unsigned long lastSerial,
                          int fake)
{
  USickData * sd;
  bool result;
//  UPose p;
  //
  result = dest != NULL;
  if (result)
  {
    if (fake > 0)
      getFakeScan( dest, lastSerial, fake);
    else
    {
      if (not isRunning())
      {
        start();
        Wait(0.1);
      }
      //
      sd = getNewestLocked();
      result = (sd != NULL) and (dest != NULL);
      if (result)
        result = sd->getDataTo(dest);
      if (result)
      {
        dest->setSerial(serial);
        dest->setMaxValidRange(maxValidRange);
        dest->setAngleResAndStart(-double(modeAngleScan)/2.0,
                                  angleResolution);
        dest->setMirror(mirrorData);
        // save to logfile (if open)
        if (datalogUsedScans)
          logThisScan(dest);
      }
      else
        // else data is not valid
        dest->setValid(false);
      if (sd != NULL)
      {
        sd->unlock();
      }
    }
    if (result)
      dest->setDeviceNum( deviceNum);
  }
  return result;
}

