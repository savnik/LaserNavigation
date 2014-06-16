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
#ifndef USICKDATA_H
#define USICKDATA_H

//#include <unocv4/utime.h>
#include <ugen4/utime.h>

#include "ulaserdata.h"

/**
Max telegram values */
#define MAX_VALUE_CNT 721
/**
Overhead for telegrams - STX, CRC, length etc */
#define TELEG_OH_ALL 7
/**
Overhead in type BO (measured data) telegram */
#define TELEG_OH_B0  3
/**
Max length for any message from SICK LMS 200 */
#define MAX_MSG_LNG (TELEG_OH_ALL + TELEG_OH_B0 + MAX_VALUE_CNT * 2)
//732

/**
Data scan from SICK

@author Christian Andersen
*/
class USickData : public ULock
{
public:
  /**
  Constructor */
  USickData();
  /**
  Destructor */
  ~USickData();
  /**
  Get pointer to data */
  inline unsigned char * getData()
    { return data; };
  /**
  Get cantime */
  inline UTime getTime()
    { return scanTime; };
  /**
  Get measurement unit 0 = cm, 1 = mm, 2 = 10cm */
  inline int getUnit()
    { return (data[6] >> 6); };
  /**
  Get measurement count */
  inline int getValueCount()
    { return (data[6] & 0x3f) * 0x100 + data[5]; };
  /**
  Set data content to be valid or invalid */
  inline void setValid(bool value)
    { valid = value; };
  /**
  Get data valid flag */
  inline bool isValid()
    { return valid; };
  /**
  Get first data value in buffer */
  unsigned char * getFirstVal()
    { return &data[7]; };
  /**
  Set time to Now() for this message */
  inline void setTime(double secAgo)
    {
     scanTime.Now();
     scanTime -= secAgo;
    };
  /**
  Copy sick data to laser data common structure */
  bool getDataTo(ULaserData * dest);
  
private:
  /**
  Valid data (CRC checked and found OK  */
  bool valid;
  /**
  Detection time for scan */
  UTime scanTime;
  /**
  Scan data */
  unsigned char data[MAX_MSG_LNG];
  /** lock, when reading or writing */
  pthread_mutex_t mLock; // pthread_mutex_lock

};

#endif
