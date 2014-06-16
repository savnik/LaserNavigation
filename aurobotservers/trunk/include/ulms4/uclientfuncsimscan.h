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
#ifndef UCLIENTFUNCSIMSCAN_H
#define UCLIENTFUNCSIMSCAN_H

#include <urob4/uclientfuncbase.h>
#include "ulaserdata.h"
/**
Decode of laser scan from simulated laser scanner
from visualized.
@author Christian Andersen
*/
class UClientFuncSimScan : public UClientFuncBase
{
public:
  /**
  Constructor */
  UClientFuncSimScan();
  /**
  Destructor */
  ~UClientFuncSimScan();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
  virtual const char * name();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  Set data structure to use */
  inline void setLaserData(ULaserData * value)
  { scan = value; };

protected:
  /**
  Decode laser scan parameters */
  bool handleLaserScan(USmlTag * tag);

protected:
  /**
  Received laser data */
  ULaserData * scan;
};

#endif
