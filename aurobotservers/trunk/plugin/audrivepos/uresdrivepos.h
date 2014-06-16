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

#ifndef URESDRIVEPOS_H
#define URESDRIVEPOS_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <umap4/uposev.h>
#include <urob4/uresposehist.h>

//#include "ureslaserifroad.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResDrivePos : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResAvoid) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResDrivePos()
  {// set name and version
    setResID(getResClassID(), 200);
    resVersion = getResVersion();
    // other local initializations
    createVarSpace(20, 0, 0, "Odometry drive settings and functions", false);
    createBaseVar();
    // create path pool
    //roads = NULL;
    poseHist = NULL;
    varGlobal = NULL;
    man = NULL;
    odoDriveCnt = 0;
  };
  /**
  Destructor */
  virtual ~UResDrivePos();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "drivepos"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 177; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  The server will offer a resource pointer by this call.
  If the resource is used, please return true. */
  bool setResource(UResBase * resource, bool remove);
  /**
  * Is this resource missing any other resources to function optimally?
  * Returns false if any is missing, and a list of the missong resources in
    the string. */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:

  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
  Get latest used manouvre sequence in a locked state.
  When the pointer no longer needed a call must be made to 'setManUnlock()'
  to allow the roadDrive to continue fretching obstacle avoidance manoeuvres. */
  UManSeq * getManLocked();
  /**
  Unlock the use of the manouvre pointer - must be unlocked as soon as possible to
  allow roadDrive to function */
  inline void setManUnlocked()
  {
    manLock.unlock();
  };
  /**
  Drive to this pose position on odometry coordinates.
  Desired speed and other options are taken from the usual places. */
  double driveOdo(UPoseV exitPose, bool ignoreVel, int repeat);
  /**
  Get current pose from poseHist. Pose is (0,0,0) if no poseHist is found */
  UPoseV getCurrentPose();

protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
  Set target pose in varPool from these parameters */
  double setTarget(const char * sideStr, double dist);

protected:
  /**
  local variables provided by this resource. */
  /**
  Pose history */
  UResPoseHist * poseHist;
  /**
  Pose history */
  UResVarPool * varGlobal;
  /**
  The calculated (and used) target pose and velocity */
  UPoseV targetPose;
  /** index to varPool variable loopCount */
  UVariable *  varLoopCnt;
  /** index to latest update time (road drive calculation) */
  UVariable * varUpdateTime;
  /** index to varPool variable target pose X */
  UVariable * varOdoX;
  /** index to varPool variable targetPose Y */
//  UVariable * varOdoY;
  /** index to varPool variable target pose H */
//  UVariable * varOdoH;
  /** index to varPool variable target velocity */
  UVariable * varOdoV;
  /** index to used count of attempts to get a path, but failed, since lase success */
  UVariable * varFailCnt;
  /** index to area where no additional path calculations take place - close to
      desired exit position */
  UVariable * varFinalDistance;
  /**
  Pointer to Last found manouvre sequence (actually in avoid module) */
  UManSeq * man;
  /**
  Lock to be used during read of the found manoeuvre */
  ULock manLock;
  /**
  The target pose at the last calculation */
  UPoseV lastExitPose;
  /**
  The current pose used in previous calculation */
  UPoseV lastStartPose;
  /**
  The current pose on first call to function (relative call only) */
  UPoseV repeat0startPose;
  /**
  The update time used for the latest calculation */
  UTime lastObstUpdate;
  /**
  When wery close to target position it is no use to recalculate the planned path,
  so just continue on current plan until traveled distance to final point is reached */
  bool finalPart;
  /**
  The robot pose when entered into the final path */
  UPose finalPose;
private:
  /** debug count of odo-drives */
  int odoDriveCnt;
};

#endif

