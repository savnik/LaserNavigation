/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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
#ifndef URESLASERIFSF_H
#define URESLASERIFSF_H

#include <urob4/uresifbase.h>
#include "ufeaturepool.h"

class UResPoseHist;

/**
Extract scanfeature lines

	@author Christian Andersen <jca@elektro.dtu.dk>
*/
class UResLaserIfSf : public UResIfBase
{
public:
  /**
  Constructor */
  UResLaserIfSf()
  { // set name and version
    setResID(getResClassID(), 200);
    //
    poseHist = NULL;
    createVarSpace(20, 0, 2, "Laser scanner server obstacle detection results", false);
    createBaseVar();
    sfPool = new UFeaturePool();
    callDispOnNewData = true;
  };
  /**
  Destructor */
  virtual ~UResLaserIfSf();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
//  virtual const char * name();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "laserSf"; };
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList();
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  Got fresh data destined to this function. */
  virtual void handleNewData(USmlTag * tag);
  /**
  Get a pointer to the scan feature pool */
  UFeaturePool * getSfPool()
  { return sfPool; };
  /**
  Create base variables and methods for var-pool */
  void createBaseVar();
  /**
  Print status for this resource */
  virtual void snprint(const char * preString, char * buff, int buffCnt);
  /**
  Clear all data in simple data pool */
  inline void clear()
  {
    sfPool->clear();
    newDataAvailable();
  };

protected:
  /**
  Decode the received ScanFeature message */
  bool handleSF(USmlTag * tag);
  /**
  Decode the received Passable interval message */
  bool handlePass(USmlTag * tag);
  /**
  Decode road edge lines */
  bool handleRoad(USmlTag * sfTag);
  /**
  Called after decode of a set of obstacle updates */
  virtual void newDataAvailable();

protected:
  /**
  Group of line segments from scanfeatures */
  UFeaturePool * sfPool;
  /**
  Group of line segments from scanfeatures */
  UResPoseHist * poseHist;
  /**
  Index to lastest related scan serial number */
  UVariable * varSerial;
  /**
  Index to latest update time */
  UVariable * varTime;
  /**
  Index to count of fature groups */
  UVariable * varSfGrps;
  /**
  Call display dunction when new data is available */
  bool callDispOnNewData;
};

#endif
