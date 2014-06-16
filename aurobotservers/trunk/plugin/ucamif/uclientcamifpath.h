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

#ifndef UCLIENTCAMIFPATH_H
#define UCLIENTCAMIFPATH_H

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>

#include "uclientfuncpath.h"

/**
* This class is a decoder resource to the camera interface plugin.
* The resource holds the vision detected road outline polygon.
*/
class UResCamIfPath : public UClientFuncPath
{
public:
  /**
  Constructor */
  UResCamIfPath()
  { // set name and version of plugin
    setResID(getResClassID(), 200);
    verboseMessages = true;
    createVarSpace(10, 0, 0, "Camera server road path detection variables", false);
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UResCamIfPath();
  /**
  Create basic polygon values */
  void createBaseVar();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "camPath"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 159; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources
  - this instance needs no resources. */
  // virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource)
  - needs no resources */
  // virtual bool setResource(UResBase * resource, bool remove);
  /**
  This function is called, when a new polygon is unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable(UProbPoly * poly);

protected:
  /// Time when road otline were detected
  UVariable * varUpdTime;
  /// Pose of robot when robot when road outline were detected
  UVariable * varUpdPose;
  /// Set to '1' when new data is available
  UVariable * varUpdNew;

};



#endif

