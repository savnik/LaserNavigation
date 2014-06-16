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
#ifndef UFUNCLASERBASE_H
#define UFUNCLASERBASE_H

#include <urob4/ufuncplugbase.h>
#include <ulms4/ulaserdata.h>
#include <ulms4/ulaserdevice.h>

/**
Base function with a few extra laser scanner support functions

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UFuncLaserBase : public UFuncPlugBase
{
public:
  /**
   * Constructor */
  UFuncLaserBase();
  /**
   * Destructor */
  virtual ~UFuncLaserBase();

protected:
  /**
   * Get a laser device pointer to the device indicated in current message or pushdata
   * \param msg this may hold additional tags that requires a specific device
   * \param pushData if this pointer is not NULL, then it is assumed to hold a laser scan
   *            of the ULaserScan data type - this is then used to specify the device.
   * \returns a pointer to the laser device or a NULL if no device is specified (and no default). */
  ULaserDevice * getDevice(UServerInMsg * msg,
                           ULaserData * pushData);
  /**
   * Get a new laserscan from default (or specified in msg) device.
   * \param msg this may hold additional tags that requires a specific
   *            device (device=N) or a fake solution (fake=F)
   * \param pushData if this pointer is not NULL, then it is assumed to hold a laser scan
   *            of the ULaserScan data type.
   * \param any if this is false, then the new scan may be the same as last time getScan() were called. This can be usefull for some test purposes where the function were called with different options. Default is false.
   * \param dev adress of a device pointer - if device is needed.
   * \returns a pointer to a laserscan buffer. The data is marked as not valid if
              no new scan is available */
  ULaserData * getScan(UServerInMsg * msg, ULaserData * pushData,
                       bool any = false, ULaserDevice ** dev = NULL);

protected:
    /**
  Last used scan number */
  unsigned long lastSerial;
  /**
  Last device used */
  int lastDevice;
  /**
   * Data buffer for a laserscan, and information
   * about the last used scan, i.e. serial number and device */
  ULaserData dataBuff;
};

#endif
