/***************************************************************************
 *   Copyright (C) 2007 by DTU Christian Andersen   *
 *   jca@elektro.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef URESLASERIFSCAN_H
#define URESLASERIFSCAN_H

#include <urob4/uresifbase.h>
//#include <urob4/uclientfuncbase.h>

/** client-side laser data */
#include "ulaserdataset.h"

class UResPoseHist;

/**
Client interface for basic scan data

	@author Christian Andersen <jca@elektro.dtu.dk>
*/
class UResLaserIfScan : public UResIfBase // UClientFuncBase , public UResVarPool
{
public:
  /**
  Constructor */
  UResLaserIfScan()
  { // set name and verison
    setResID(getResClassID(), 200);
    resVersion = getResVersion();
    //
    poseHist = NULL;
    //
    //createVarSpace(20, 0, 2, "Laser scanner basic data interface");
    createBaseVar();
    callDispOnNewData = true;
    strncpy(dataSourceName, "(none)", MAX_DATA_SOURCE_NAME);
  };
  /**
  Destructor */
  virtual ~UResLaserIfScan();
  /**
  Name of function
  The returned name is intended as informative to clients
  and should include a version number */
//  virtual const char * name();
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
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "laserScan"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 173; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);

public:
  /**
  Get pointer to the scan history data */
  ULaserDataHistory * getScanHist()
  { return &scanHist; };
  /**
   * The varPool has methods, and a call to one of these are needed.
   * Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
   * If the returnStruct and returnStructCnt is not NULL, then
  a number (no more than initial value of returnStructCnt) of
  structures based on UDataBase may be returned into returnStruct array
  of pointers. The returnStructCnt should be modified to the actual
  number of used pointers (if needed). */
/*  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);*/

protected:
  /**
  Called when a new scan is received */
  virtual void gotNewData(ULaserDataSet * scan);
  /**
  Create variables for this resource */
  void createBaseVar();
  /**
  Decode laser scan parameters */
  bool handleLaserScan(USmlTag * tag);

protected:
  /**
  Pointer to pose history resource */
  UResPoseHist * poseHist;
  /**
  Data buffer for received data */
  ULaserDataHistory scanHist;
  /**
  Index to variable with laser scanner position (latest sensor) */
  UVariable * varSensorX;
  /**
  Index to variable with laser scanner rotation (orientation) (latest sensor) */
  UVariable * varSensorOmega;
  /**
  Index to scannumber */
  UVariable * varScan;
    /**
  Call display dunction when new data is available */
  bool callDispOnNewData;
  /**
  Length of data source name */
  static const int MAX_DATA_SOURCE_NAME = 10;
  /**
  Data source name */
  char dataSourceName[MAX_DATA_SOURCE_NAME];
};


#endif
