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

#ifndef UCLIENTCAMIFGMK_H
#define UCLIENTCAMIFGMK_H

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>

#include "uclientfuncgmk.h"

/**
* A guidemark data structure holder for (especially) the camera client plugin.
* The data is extracted to a var-pool structure.
*/
class UResCamIfGmk : public UClientFuncGmk
{
public:
  /**
  Constructor */
  UResCamIfGmk()
  { // set name and version
    setResID(getResClassID(), 200);
    verboseMessages = true;
    // create space for 10 variables, 5 structures and 10 functions
    createVarSpace(10, 0, 0, "Camera server interface guidemark variables", false);
    createBaseVar();
  };
  /**
  Destructor */
  virtual ~UResCamIfGmk();
  /**
  Create base variables in varPool for this interface type */
  void createBaseVar();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "camGmk"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 182; };*/
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
//  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
//  virtual bool setResource(UResBase * resource, bool remove);

protected:
  /**
  This function is called, when new guidemark(s) is/are unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable(int updCnt, UTime updTime);

protected:
  /// index to update flag of closest guidemark
  UVariable * varUpd;
  /// index to guidemark update time
  UVariable * varTime;
  /// index to position of guidemark X
  UVariable * varPosX;
  /// index to position of guidemark X
/*  UVariable * varPosY;
  /// index to position of guidemark X
  UVariable * varPosZ;
  /// index to orientation of guidemark Omega
  UVariable * varPosO;
  /// index to orientation of guidemark Phi
  UVariable * varPosP;
  /// index to orientation of guidemark Kappa
  UVariable * varPosK;*/
  /// index to code in this (closest) guidemark
  UVariable * varCode;
  /// index to number if guidemarks detected in last image
  UVariable * varGmkCnt;
  /// Index to call display variable
  UVariable * varCallDisp;
  //
  /// index to guidemark ID to watch for and update into 'sel' struct
  UVariable * varSelID;
  /// index to update flag of selected guidemark
  UVariable * varSelUpd;
  /// index to selected guidemark update time for selected GMK
  UVariable * varSelTime;
  /// index to position of selected GMK
  UVariable * varSelPosX;
  /// index to position of selected GMK
/*  UVariable * varSelPosY;
  /// index to position of selected GMK
  UVariable * varSelPosZ;
  /// index to orientation of selected GMK
  UVariable * varSelPosO;
  /// index to orientation of selected GMK
  UVariable * varSelPosP;
  /// index to orientation of selected GMK
  UVariable * varSelPosK;*/
  /// index to code (ID) of selected GMK
  UVariable * varSelCode;
};



#endif

