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
#ifndef UFAKEDEVICE_H
#define UFAKEDEVICE_H

#include "ulaserdevice.h"
#include <ugen4/ulock.h>

/**
Device to deliver some simple fake scandata for test without real scanner or simulator.

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UFakeDevice : public ULaserDevice, public ULock
{
public:
  /**
  Constructor */
  UFakeDevice();
  /**
  Destructor */
  ~UFakeDevice();
  /**
  Change scanner resolution mode.
  Returns true if new resolution is set */
  virtual bool changeMode(int scanangle, double resolution);
  /**
  Is the port to the device open */
  virtual bool isPortOpen()
  { return true; };
  /**
  Get fake data to this destination and advance fake position
  if scan number is used before, otherwise maintain position
    \param dest is where to load the scan.
    \param lastSerial is last used serial number - set next number in scan
    \param int fake is fake number - 0 is live
    \param double fakeDt is update time for fake position - default is 0.2 sec */
  virtual bool getNewestData(ULaserData * dest,
                              unsigned long lastSerial,
                              int fake);
  /**
  Is laserscanner an URG (Hokuyo) scanner */
  virtual inline bool isFake()
  { return open; };
  /**
  Print device status to a buffer string */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Device name determines max range for device.
  Decode the range from the name. */
  double maxRange();
  /**
  Set device name */
  virtual void setDeviceName(const char * device);
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

protected:
  /**
  Is connection open - used to reset coordinates */
  bool open;
  /**
  Default fake pattern */
  int fakeMode;
  /// link to range SD value
  UVariable * varRsd;
  /// add fakw world to polygon plugin
  UVariable * varAddToPoly;
  /// add fakw world to polygon plugin
  UVariable * varAddToPose;
  /// pose error value
  UVariable* varPoseErr;
  /// fake timing
  UVariable * varFakeDt;
};

#endif
