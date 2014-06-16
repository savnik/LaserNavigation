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

#include <urob4/uclientfuncbase.h>
#include <umap4/uprobpoly.h>

/**
Class to handle path data

@author Christian Andersen
*/
class UClientFuncPath : public UClientFuncBase
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

protected:
  /**
  Unpack pathGet message */
  void handlePathGetMsg(USmlTag * tag);
  /**
  Unpack path polygon */
  void handlePathPolygonData(USmlTag * iTag);
  /**
  This function is called, when a new polygon is unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable(UProbPoly * poly);

protected:
  /**
  Pointer to polygon data buffer */
  UProbPoly * poly;
  /**
  Name of received polygon */
  char polyName[MAX_SML_NAME_LENGTH];
private:
  /**
  Polygon buffer */
  UProbPoly polyData;
};

#endif
