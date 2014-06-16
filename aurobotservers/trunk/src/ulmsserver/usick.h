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
#ifndef USICK_H
#define USICK_H

#include <string.h>
#include "ulaserdevice.h"
#include "usickdata.h"

/**
Flag for serial speed to SICK scanner */
#define SLOWSERIAL 0x01
/**
Flag for serial speed to SICK scanner */
#define FASTSERIAL 0x02
/**
Number of receive buffers */
#define PACK_BUF_SIZE 20
/**
Size of receive buffer - should hold >= 2 receive messages */
#define RECEIVE_BUFFER_SIZE (2 * MAX_MSG_LNG)
/**
Define slow baudrate */
#define SLOWSERIAL_BAUDRATE 9600
//#define SLOWSERIAL_BAUDRATE 38400
/**
Define fast baudrate */
#define FASTSERIAL_BAUDRATE 500000


/**
Holds basic laser scanner functions

@author Christian Andersen
*/
class USick : public ULaserDevice, protected ULock
{
public:
  /**
  Constructor */
  USick();
  /**
  Destructor */
  ~USick();
  /**
  Is sick scanner running */
/*  inline bool isRunning()
    { return running and ((LMS_fd >= 0) or modeSimulated); };*/
  /**
  Set device name */
/*  inline void setDeviceName(const char * device)
    { strncpy(devName, device, MAX_DEVICE_NAME_LNG); } ;*/
  /**
  Start sick scanner and grap the data */
/*  bool start();*/
  /**
  Stop sick scanner port.
  Stop also thread - if not 'justClosePort'. */
/*  void stop(bool justClosePort);*/
  /**
  Get newest data packet */
  USickData * getNewestLocked();
  /**
  Start the receive loop for continous messages */
/*  void threadRunLoop();*/
  /**
  Set simulated mode */
/*  inline void setSimulatedMode(bool value)
    { modeSimulated = value; };*/
  /**
  Set scan angle mode (180 or 100 deg) */
/*  inline void setScanAngleMode(int angleInDeg)
    { modeAngleScan = angleInDeg; };*/
  /**
  Set scan resolution mode (100, 50 or 25 centi degrees) */
/*  inline void setScanResolution(int angleInCentiDeg)
    { modeAngleResolution = angleInCentiDeg; };*/
  /**
  Get actual scan resolution in centi degrees */
/*  inline int getScanResolution()
    { return modeAngleResolution; };*/
  /**
  Get actual scan angle in degrees */
/*  inline int getScanAngle()
    { return modeAngleScan; };*/
  /**
  Get statistics - good count */
/*  inline unsigned int getGood()
    { return statGoodCnt; };*/
  /**
  Get statistics - bad count */
/*  inline unsigned int getBad()
    { return statBadCnt; };*/
  /**
  Get statistics - number of good messages per second */
/*  inline double getMsgRate()
    { return statMsgRate; };*/
  /**
  Get serial device name */
  inline char * getDeviceName()
    { return devName;} ;
  /**
  Get number of measurements in scan-mode */
/*  int getMaxMeasurements()
    { return modeAngleScan * (100 / modeAngleResolution) + 1; };*/
  /**
  Get angle from measurement number - result is in degrees */
/*  double getScanAngle(int measurement)
    { return double(modeAngleResolution)/100 * double(measurement) - modeAngleScan/2.0; };*/
  /**
  Print status */
/*  void print(char * preString);*/
  /**
  Change scanner resolution mode.
  Returns true if new resolution is set */
  virtual bool changeMode(int scanangle, double resolution);
  /**
  Is the port to the device open */
  virtual bool isPortOpen();
  /**
  Get the newest data unpacked to this structure.
  Returns true if valid. */
  virtual bool getNewestData(ULaserData * dest, 
                             unsigned long lastSerial,
                             int fake);
  /**
  Is laserscanner a SICK scanner */
  virtual inline bool isSick()
  { return true; };

protected:
  /**
  Set serial speed on communication port */
  bool set_serial(int fd, int speed);
  /**
  change communication speed to slow (9600 bps)
  NB! this may take some time (< 1 sec) */
  bool set_slow_speed();
  /**
  change communication speed to fast (500 kbps)
  NB! this may take some time (< 1 sec) */
  bool set_fast_speed();
  /**
  Open serial post.
  Return true if open(ed) */
  virtual bool openPort();
  /**
  Close serial port */
  virtual void closePort();
  /**
  Receive data from device -- called from
  device loop, should return as fast as possible
  after dooing the job, i.e. no blocking read.
  Should add number of good and bad blocks of data
  to statBadCnt and statGootcnt.
  Returns true if data received. */
  virtual bool receiveData();
  /**
  Open serial post */
  int open_port(void);
  /**
  Close serial port */
  void close_port();
  /**
  Stop continoious mode operation from SICK scanner */
  bool stop_continous_mode();
  /**
  Start continoious mode operation from SICK scanner */
  bool enter_continous_mode();
  /**
  Send data to SICK */
  USickData * LMS_send_receive(unsigned char* telegram);
  /**
  Send data to sick, and try up to 'repeat' times for
  a successfull reply */
  USickData * LMS_send_receive(unsigned char* telegram, int repeats);
  /**
  Add CRC to this message */
  void addCRC16(unsigned char* CommData);
  /**
  Function writes the received message to the delivered file descripter */
  int toLMS(int fd,unsigned char* msg, unsigned int datalen);
  /**
  Get next buffer that can be locked and return pointer to this */
  USickData * getNextBuffLocked();
  /**
  Set next buffer unlocked and if the result is true
  advance packsNew to this buffer */
//  void setNextBuffUnLocked(USickData * data, bool valid);
  /**
  Test af data pakkerne der modtages fra SICK sensoren (CRC)
  Der undersoeges om der er fejl paa data pakken
  Denne funktion er taget fra SICK manualen */
  unsigned int getCCRC(unsigned char *CommData);
  /**
  Get next full package from serial device.
  Returnes the data length at the given adress 'length'.
  NB! on entry the length of the previous message must be
  maintained  in length, to discard used data properly.
  Returns true if data has passed the CRC check. */
  bool receive_continous_data(int * length);
  /**
  Calculate expected CRC on received data */
  unsigned short specialCCRC(unsigned char *CommData,unsigned int uLen);
  /**
  Change to installation mode */
  void enter_installation_mode();
  /**
  Set scanner configuration */
  void set_configuration();
  /**
  Request status */
  void request_status();
  /**
  Set resolution to either 1 deg or 0.5 deg.
  The command is repeated up to 'repeats',waiting for a
  positive reply.
  Returns true if successful,
  otherwise false. */
  virtual bool set_resolution(int scanAngleDeg,
                      int resolutionCdeg,
                      int repeats);
  /**
  Make a simulated reply - old recording in place of real Sick data
  Returnes the data length at the given adress 'length'.
  Returns true if message is valid. */
  bool receiveSimulatedData(int * length);
  /**
  Get default delat estimate for the device type
  - here expected to be a bit more than half a scantime (at 75 scans per second) */
  inline virtual double getDefaultDelay()
  { return 0.01; };

protected:
  /**
  Is sick scanner running */
  //bool running;
  /**
  Serial device, where laser scanner is attached */
  //char devName[MAX_DEVICE_NAME_LNG];
  /**
  File descriptor for connection to laser scanner */
  int LMS_fd;
  /**
  Serial speed (1 (slow (typically 9600)) or 2 (fast (500000))) */
  int serialspeed;
  /**
  Speed of serial port */
  int portSpeed;
  /**
  Print more (debug) messages */
  bool verbose;
  /**
  Buffers for received data */
  USickData packs[PACK_BUF_SIZE];
  /**
  Newest received package buffer */
  int packsNew;
  /**
  Thread handle for frame read thread. */
  //pthread_t threadHandle;
  /**
  Is thread actually running */
  //bool threadRunning;
  /**
  Should thread stop - terminate */
  //bool threadStop;
  /**
  Buffer for received data */
  unsigned char LMS_response_buffer[RECEIVE_BUFFER_SIZE];
  /**
  Used data in LMS_response_buffer */
  int byteCnt;
  /**
  Statistics of lost bytes, counts, when packages are
  too short */
  int lostBytes;
  /**
  Data left unused before a header is detected */
  int wastedData;
  /**
  Data logfile for received data - debug */
  //FILE * datalog;
  /**
  The scanning angle of the SICK laser scanner,
  it can be either 100 or 180 deg.
  Unit is degrees. */
  //int modeAngleScan;
  /**
  Angle resolution, this can be 1.0 0.5 or 0.25 degrees.
  NB! natural resolution is 1.0 deg, the others take
  2 or 4 resolutions to generate.
  Unit is 1/100 degree. */
  //int modeAngleResolution;
  /**
  Use simulated data and not real data from Sick */
  //bool modeSimulated;
  /**
  Read statistics total good messages */
  //unsigned int statGoodCnt;
  /**
  Read statistics total bad messages */
  //unsigned int statBadCnt;
  /**
  Good messages per second. */
  //double statMsgRate;
  /**
  Serial number for scan */
//  unsigned long serial;
};

#endif
