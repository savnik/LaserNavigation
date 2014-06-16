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
#ifndef UCLIENTFUNCLASER_H
#define UCLIENTFUNCLASER_H

#include <urob4/uclientfuncbase.h>
#include <umap4/upose.h>
#include <umap4/uprobpoly.h>

#include "ulaserdataset.h"

/**
Namespace values */
#define NAMESPACE_MMRSR2 1
#define NAMESPACE_LASER  2
#define NAMESPACE_MMRD   3

/**
Max allowed control intervals received */
#define MAX_CI_INTERVALS 120
/**
Max allowed stored waypoint positions */
#define MAX_WP_POSITIONS 110
/**
Max pose history values */
#define MAX_POSE_HIST 640
/**
Number of allowed paths found in the area in front of the robot
using the most recent laser scans */
#define MAX_NEW_PATH_CANDIDATES 513
/**
Number of polygons reserved for free area polygons from vision */
#define MAX_FREE_POLY_HIST 99
class ULaserWpc
{
  public:
  /**
  Constructor */
  ULaserWpc();

  public:
  /**
  Current motot control command */
  double motorLeft;
  double motorRight;
  /**
  Current speed */
  double speedCurrent;
  /**
  Current base speed */
  double speedPlanned;
  /**
  Speed advice from path-finder */
  double speedAdvice;
  /**
  Position of waypoints in queue to be executed */
  UPosition wpPos[MAX_WP_POSITIONS];
  /**
  Number of used WP positions */
  int wpPosCnt;
};


/**
Support for laser scanner results
@author Christian Andersen
*/
class UClientFuncLaser : public UClientFuncBase
{
public:
  /**
  Constructor */
  UClientFuncLaser();
  /**
  Destructor */
  virtual ~UClientFuncLaser();
  /**
  Name of function
  The returned name is intended as informative to clients
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
  The server has set (or changed) the namespace */
  virtual void changedNamespace(const char * newNamespace);

protected:
  /**
  Called when a new scan is received */
  virtual bool gotNewData(ULaserDataSet * scan);
  /**
  Called when new ekf data is available */
  virtual bool gotNewEkfData();
  /**
  Called when new ekf data is available */
  virtual bool gotNewPlanData();
  /**
  Called when new ekf data is available */
  virtual bool gotNewPathData();
  /**
  get time of laser-data */
  UTime getLaserTime(int histNum);
  /**
  Set pose and pose history */
  void setPose(UPoseTime poseTime);
  /**
  Add this to pose history if the time is newer than the newest */
  void setPoseIfNewer(UPose pose, UTime t);
  /**
  Got new polygon data with passable area.
  Newest data is in freePoly[freePolyNewest] */
  virtual bool gotNewFreePolyData();

private:
  /**
  Decode laser scan parameters */
  bool handleLaserScan(USmlTag * tag);
  /**
  Handle Waypoint Finder data */
  bool handleWpf(USmlTag * ciTag);
  /**
  Handle Odometry controller data */
  bool handleOdo(USmlTag * tag);
  /**
  Handle control interval data */
  bool handleControlIntervals(USmlTag * ciTag);
  /**
  Handle EKF message */
  bool handleEkf(USmlTag * tag);
  /**
  Handle Planner message */
  bool handlePlan(USmlTag * tag);
  /**
  Handle a group of detected path options */
  bool handlePath(USmlTag * tag);
  /**
  Handle Waypoint Controller data */
  bool handleWpc(USmlTag * tag);
  /**
  Handle complex data from get command. */
  bool handleGetComplexData(USmlTag * tag);
  /**
  Handle data as variables */
  bool handleVarData(USmlTag * tag);

protected:
  /**
  Data buffer for received data */
  ULaserDataHistory scanHist;
  /**
  Detected path data wihin the laser-scans in front of the robot */
  ULaserPathResult * paths[MAX_NEW_PATH_CANDIDATES];
  /**
  Waypoint controller data */
  ULaserWpc wpcData;
  /**
  Count of valid detected paths */
  int pathsCnt;
  /**
  Corrent robot pose */
  UPoseTime odoPose;
  /**
  Estimated position by EKF */
  UPose ekfPose;
  /**
  Update time by ekf */
  UTime ekfTime;
  /**
  EKF updates skipped */
  int ekfuSkipd;
  /**
  EKF updates skipped */
  int ekfuUsed;

  /**
  Goal position for robot */
  UPose toPose;
  /**
  Goal position at current position in mission file */
  UPosition ciGoal;
  /**
  Position of laser scanner on robot */
  UPosRot laserPose;
  /**
  Odometry pose history */
  UPoseTime poseHist[MAX_POSE_HIST];
  /**
  Time of pose history */
  //UTime poseHistTime[MAX_POSE_HIST];
  /**
  Number of pose history entries */
  int poseHistCnt;
  /**
  Newest pose history index */
  int poseHistNewest;
  /**
  Planner data */
  UPlannerData planner;
  /**
  Free area from vision */
  UProbPoly freePoly[MAX_FREE_POLY_HIST];
  /**
  Used free poly hist entries */
  int freePolyCnt;
  /**
  Newest free polygon */
  int freePolyNewest;
};

#endif
