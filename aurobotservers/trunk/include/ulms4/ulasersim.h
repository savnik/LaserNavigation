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
#ifndef ULASERSIM_H
#define ULASERSIM_H

#include <urob4/uclienthandler.h>
#include "uclientfuncsimscan.h"
#include "ulaserdevice.h"

/**
Class to connect to a simulated laserscanner

@author Christian Andersen
*/
class ULaserSim : public ULaserDevice, public UClientHandler
{
public:
  /**
  Constructor */
  ULaserSim();
  /**
  Destructor */
  ~ULaserSim();
  /**
  Is the port to the device open */
  inline virtual bool isPortOpen()
  { return isConnected(); };
  /**
  Get newest available data
  to this destination structure.
  Returns true if data is available. */
  virtual bool getNewestData(ULaserData * dest,
                             unsigned long lastSerial,
                             int fake);
  virtual bool changeMode(int scanangle, double resolution);

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
  Get name of device */
  virtual const char * getNameFromDevice();
  /**
   * set server core pointer - just a debug feature - I think) */
  virtual void setCore(UCmdExe * pCore)
  { core = pCore; };

protected:
  /**
  latest data received from device */
  ULaserData * scan;
  /**
  Client data decoder for simulated laser scanner connection */
  UClientFuncSimScan * scanf;
  /**
  New data received */
  bool newData;
  /**
  Time of last received data */
  UTime dataTime;
  /**
  Automatic repeat get scan */
  bool repeatGetScan;
  /**
  Last received serial from simulator */
  unsigned long rxSerial;
  ///server core pointer
  UCmdExe * core;
};

#endif
