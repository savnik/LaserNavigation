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

#ifndef URES_CAM_IF_CAM_H
#define URES_CAM_IF_CAM_H

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uclientfunccam.h>

/**
* A guidemark data structure holder for (especially) the camera client plugin.
* The data is extracted to a var-pool structure.
*/
class UResCamIfCam : public UClientFuncCam
{
public:
  /**
  Constructor */
  UResCamIfCam()
  { // set name and version
    setResID(getResClassID(), 200);
    resVersion = getResVersion();
    // create space for 10 variables, 5 structures and 10 functions
    createVarSpace(10, 0, 0, "Camera server interface camera lens data handling", false);
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UResCamIfCam();
  /**
  Create base variables in varPool for this interface type */
  void createBaseVar();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "camCam"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 174; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  // virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  //virtual bool setResource(UResBase * resource, bool remove);
  /**
  debug */
  //void doTimeTick();

protected:
  /**
  Called when a data set is received */
  virtual void gotNewData(int device);

protected:
  /**
  Index to variable with last update time */
  UVariable * varUpdTime;
  /**
  Index to variable with destination for last image */
  UVariable * varCamDevNum;
  /**
  Index to variable with number of images received */
  UVariable * varUpdCnt;
  /**
  Call display on data update */
  UVariable * varCallDisp;

  /// index to camera details
  UVariable * varPosX;
  /// index to camera details
  UVariable * varPosY;
  /// index to camera details
  UVariable * varPosZ;
  /// index to camera details
  UVariable * varPosO;
  /// index to camera details
  UVariable * varPosP;
  /// index to camera details
  UVariable * varPosK;
  /// index to camera details
  UVariable * varFocal;
  /// index to camera details
  UVariable * varWidth;
  /// index to camera details
  UVariable * varHeight;

private:
  //int tick;

};



#endif

