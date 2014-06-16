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
#ifndef UHOKUYO_H
#define UHOKUYO_H

#include <ugen4/ulock.h>
#include "ulaserdevice.h"

/**
Length of a full message with 768 measurements (or is it 769?).
header =                                10 bytes
24 blocks (each 32points) = 65 * 24 = 1560
 1 block (with 1 point?)  =              3
end LF                    =              1
total                                 1574 bytes

if 3 char encoding:
  header                            10 chars
  769*3 values                    2307 chars
  36 blocks adds 36 \n's            36
  endlf (x2)                         2
total                             2355 chars
*/

#define MAX_URG_MSG_LNG 2355
/**
Buffer to hold two full data frames plus a terminating zero. */
#define MAX_DATA_LNG (MAX_URG_MSG_LNG * 2 + 1)

/**
Interface to Hokuyo laserscanner

@author Christian Andersen
*/
class UHokuyo : public ULaserDevice, protected ULock
{
public:
  /**
  Constructor */
  UHokuyo();
  /**
  Destructor */
  ~UHokuyo();
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
  Is laserscanner an URG (Hokuyo) scanner */
  virtual inline bool isUrg()
  { return true; };
  /**
  Create entry infor the globas variable database for this device */
  virtual void createBaseVars();

protected:
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
  Send data to device. Sends 'lng' bytes from 'msg'.
  Data must be a zero-terminated string. */
  virtual bool sendToDevice(const char * msg, int lng);
  /**
  Receive data from device, at maximum
  'maxLng' of bytes.
  Return if timeout ('timeoutSec' sec) or
  when a '\n\n' is received - there may be
  additional data after '\n\n'.
  Returns when 'maxLng' is reached too  */
  int receiveFromDevice(char * start, int maxLng, double timeoutSec);
  /**
  Decode mesaurement data message.
  \param msg is the mesaage to decode.
  \param msgCnt is the length of the message
  \param dest is the destination buffer for the scan
  \returns true if scan is valid */
  bool decodeData(char * msg, const int msgCnt, ULaserData * dest);
  /**
  Decode name message */
  bool decodeName(char * msg);
  /**
  Get name of device */
  virtual const char * getNameFromDevice();
  /**
  Decode this 'msg' data to the 'dest' structure
  \param msg is the mesaage to decode.
  \param msgCnt is the length of the message
  \param dest is the destination buffer for the scan
  \returns true if successful (scan is valid) */
  bool getDataTo(char * msg, const int msgCnt, ULaserData * dest);
  /**
  Make the get-data command, e.g. "\nG04572501\n"
  for data element 45 to 725 with 1 measurement in each value.
  An extra "\n" is added in front as a workaround for an interface problem,
  where some data are repeated - an error will be received before tha valid data.
  Takes a string buffer 'cmdStr' of length 'cmdStrCnt'.
  Returns 'cmdStr'.
  Nothing is changed if no string is provided or string is
  too short (at least 11 characters is needed). */
  char * makeGetDataString(char * cmdStr, const int cmdStrCnt);
  /**
  Request data according to spec
  in angleResolution and modeAngleScan */
  bool requestMoreData();
  /**
  Send data to log, replacing whitespace with '<' */
  void toLog(const char * data, int length, const char * pre, UTime ts);
  /**
  Get default delat estimate for the device type
  - here expected to be a bit more than half a scantime (at 10 scans per second) */
  inline virtual double getDefaultDelay()
  { return 0.07; };

protected:
  /**
  File device handle for scanner */
  int hfd;
  /**
  Data buffer for received data */
  char dataBuf[MAX_DATA_LNG];
  /**
  Number of unused characters in buffer */
  int dataCnt;
  /**
  Time stamp of received data.
  Timestamp is taken, when first data is received
  (first chunk of data) */
  UTime dataRxTime;
  /**
  Time of last received data */
  UTime dataTime;
  /**
  Decoded range data */
  ULaserData * lasData;
  /**
  Logfile */
  ULogFile laslog;
  /**
  reply since last OK - for error detection */
  int badSeries;
  /**
  Version info suppress after 5 prints */
  int versionInfoCnt;
};

#endif
