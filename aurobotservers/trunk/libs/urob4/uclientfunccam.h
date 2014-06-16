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
#ifndef UCLIENTFUNCCAM_H
#define UCLIENTFUNCCAM_H

#include "uresifbase.h"

#include "uclientcams.h"

/**
Camera server client function to store camera data (camGet) data handler

@author Christian Andersen
*/
class UClientFuncCam : public UResIfBase // UClientFuncBase
{
public:
  /**
  Constructor */
  UClientFuncCam();
  /**
  Destructor */
  virtual ~UClientFuncCam();
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
  Get data structure with camera data */
  UClientCams * getCams()
  { return cams; };

protected:
  /**
  Decode received data */
  bool handleCamGet(USmlTag * tag);
  /**
  Called when a data set is received */
  virtual void gotNewData(int device)
  { /* nothing to do here */ };

protected:
  /**
  Camera data */
  UClientCams * cams;
};

#endif
