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
#ifndef UCLIENTFUNCPATH_H
#define UCLIENTFUNCPATH_H

#include <urob4/uresifbase.h>
#include <umap4/uprobpoly.h>

#include "uvisdata.h"

/**
Class to handle path data

@author Christian Andersen
*/
class UClientFuncPath : public UResIfBase
{
public:
  /**
  Constructor */
  UClientFuncPath();
  /**
  Destructor */
  ~UClientFuncPath();
  /**
  Name of function
  The returned name is intended as informative to users
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
  Get pointer to - newest - vision data */
  inline UVisData * getVisData()
  { return visData[0]; };
  /**
  Get pointer to vision data with this age */
  inline UVisData * getVisData(int idx)
  { return visData[maxi(0, mini(idx, visDataCnt - 1))]; };
  /**
  Get number of available vision based polygons */
  inline int getVisDataCnt()
  { return visDataCnt; };

protected:
  /**
  Unpack pathGet message */
  void handlePathGetMsg(USmlTag * tag);
  /**
  Unpack path polygon */
  void handlePathPolygonData(USmlTag * tag, UVisData * destination);
  /**
  This function is called, when a new polygon is unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable(UProbPoly * poly);

protected:
  static const int MAX_VIS_DATA = 100;
  /**
  Pointer to vision data */
  UVisData * visData[MAX_VIS_DATA];
  /**
  Maximum allowed vision data structures */
  int visDataMax;
  /**
  Actual number of used vision data */
  int visDataCnt;
};

#endif
