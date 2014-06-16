/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
 *   jca@elektro.dtu.dk                                                    *
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

#ifndef U_RESBIN_H
#define U_RESBIN_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>
#include <ugen4/upolygon.h>
#include <urob4/uresifbase.h>



/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResBin : public UResIfBase, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResBin) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResBin()
  { // set name and version
    setResID("bintest", 1370);
    UResBinInit();
  };
  /**
  Destructor */
  virtual ~UResBin();
  /**
   * Initialize resource */
  void UResBinInit();
  /**
  Function, that shall return a string with all handled tag names */
  virtual const char * commandList()
  { return "bintest"; };
  /**
  Got fresh data from a stream destined to this function.
  that is, the received XML tag is in the commandList() for this resource. */
  virtual void handleNewData(USmlTag * tag);


protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();  

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /// number of established poly items
  UVariable * varBinCnt;
  /// last update time for polygons
  UVariable * varUpdTime;
};

#endif

