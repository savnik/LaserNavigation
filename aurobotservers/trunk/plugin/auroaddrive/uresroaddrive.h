/***************************************************************************
 *   Copyright (C) 2006-2008 by DTU (Christian Andersen)                   *
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

#ifndef URESROADDRIVE_H
#define URESROADDRIVE_H

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
class UResRoadDrive : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResAvoid) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResRoadDrive()
  { // set name and version number
    setResID(getResClassID(), 200);
  // other local initializations
    createVarSpace(20, 0, 0, "Road line driver settings", false);
    createBaseVar();
  // create path pool
  //roads = NULL;
    poseHist = NULL;
    varGlobal = NULL;
    man = NULL;
  };
  /**
  Destructor */
  virtual ~UResRoadDrive();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "roaddrive"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 172; };*/
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
  * Find xit pose relative to road lines in odometry coordinates.
    if side is >= 0 then try the distance relative to this side.
  * Side is 0=left, 1=top, 2=right
  * Exit pose should be 'sideDist' away from side line.
  * Exit pose should be  'forwardDist' ahead of robot.
  * Returns true if successful. may fail if no road is avaiable
  * or call to road plugin failed. */
  bool findExitPose(int side, double sideDist, double forwardDist,
                     UPose * exitPose);
  /**
  Get latest used manouvre sequence in a locked state.
  When the pointer no longer needed a call must be made to 'setManUnlock()'
  to allow the roadDrive to continue fretching obstacle avoidance manoeuvres. */
  UManSeq * getManLocked();
  /**
  Unlock the use of the manouvre pointer - must be unlocked as soon as possible to
  allow roadDrive to function */
  inline void setManUnlocked()
  { manLock.unlock(); };
  /**
   * Drive along the road as possible, and try to maintain
  the specified distance from one of the road lines.
   * The refSide is 0=left, 1=top, 2=right. */
  double driveSide(const int refSide, const double refDist, int repeat);


protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
   * Drive along the road as possible, and try to maintain
  the specified distance from one of the road lines.
   * The refSide is 'left', 'top' or 'right'. */
  double driveRoad(const char * refSide, const double refDist);
  /**
  Set target pose in varPool from these parameters */
  double setTarget(const char * sideStr, double dist);

protected:
  /**
  local variables provided by this resource. */
  /**
  Link to road lines */
//  UResLaserIfRoad * roads;
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
  UVariable * varLoopCnt;
  /** index to latest update time (road drive calculation) */
  UVariable * varUpdateTime;
  /** index to varPool variable target pose X */
  UVariable * varTgtX;
  /** index to varPool variable targetPose Y */
  //int varTgtY;
  /** index to varPool variable tarhet pose H */
  //int varTgtH;
  /** index to varPool variable target velocity */
  UVariable * varTgtVel;
  /** index to used reference line edge distance (positive is left) */
  UVariable * varEdgeDist;
  /** index to used reference line (0=left, 1=center, 2=right) */
  UVariable * varEdge;
  /** index to used projection distance in front of robot */
  UVariable * varFwdDist;
  /** index to used count of attempts to get a path, but failed, since lase success */
  UVariable * varFailCnt;
  /**
  Last obtainge manouvre sequence obtained */
  UManSeq * man;
  /**
  Lock to be used during read of the found manoeuvre */
  ULock manLock;
  /**
  Time of last update of obstacle interface */
  UTime lastObstUpdate;
  /**
  Time of last update of road interface */
  UTime lastRoadUpdate;
  /**
  Last reference side for road drive */
  int lastRefSide;
  /**
  Last reference distance */
  double lastRefDist;
};

#endif

