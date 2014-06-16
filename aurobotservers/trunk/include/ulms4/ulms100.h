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
#ifndef ULMS100_H
#define ULMS100_H

#include <ugen4/ulock.h>
#include "ulaserdevice.h"

#define MAX_LMS100_MSG_LNG 64000
/**
Buffer to hold two full data frames plus a terminating zero. */
#define MAX_LMS100_LNG (MAX_LMS100_MSG_LNG * 2 + 1)

/**
Interface to LMS100 laser scanner

@author Christian Andersen
*/
class ULms100 : public ULaserDevice, protected UClientPort
{
public:
  /**
  Constructor */
  ULms100();
  /**
  Destructor */
  ~ULms100();
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
  Is laserscanner a LMS100 scanner */
  virtual inline bool isLms100()
  { return true; };

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
  after doing the job, i.e. no blocking read.
  Should add number of good and bad blocks of data
  to statBadCnt and statGootcnt.
  Returns true if data received. */
  virtual bool receiveData();
  /**
  Send data to device. Sends 'lng' bytes from 'msg'.
  Data must be a zero-terminated string. */
  virtual bool sendToDevice(const char * msg, int lng);
  /**
   * \brief Decode  message from laserscanner.
   * Handles all received messagetypes, and
   * stores result as appropriate.
   * \param msg is a 0 terminated string (but may have binary parts).
   * Intial STX and terminating ETX is removed.
   * \param rxTime is time-stamp just after start of receive
   * \returns pointer to the first unused character. This may be the same
   * as 'msg', as 'msg' may be part of a message only. */
  char * decodeData(char * msg, UTime rxTime);
  /**
  Decode name message */
  bool decodeName(char * msg);
  /**
  Get name of device */
  virtual const char * getNameFromDevice();
  /**
  Decode this 'msg' data to the 'dest' structure.
  \param \msg is location of current position in buffer
  \Return trur if limited data values are legal. */
  bool getDataTo(char * msg, ULaserData * dest);
  /**
  Send data to log, replacing whitespace with '<' */
  void toLog(const char * data, int length, const char * pre);
  /**
  Get default delay estimate for the device type
  - here expected to be a bit more than half a scan-time (at 50 scans per second) */
  inline virtual double getDefaultDelay()
  { return 0.019; };

protected:
  /**
  Data buffer for received data */
  char dataBuf[MAX_LMS100_LNG];
  /**
  Number of unused characters in buffer */
  int dataCnt;
  /**
  Time stamp of received data.
  Time stamp is taken, when first data is received
  (first chunk of data) */
  UTime dataRxTime;
  /**
  Time of last received data */
  UTime dataTime;
  /**
  Decoded range data */
  ULaserData * lasData;
  /**
  Log file */
  ULogFile laslog;

  static const char STX = 2;
  static const char ETX = 3;

private:
    bool finishedScan;
};

#endif
