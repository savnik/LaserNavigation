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
#ifndef URESPASSABLE_H
#define URESPASSABLE_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uobstacle.h>

#include <ulms4/ulaserdata.h>

#include "ulaserpi.h"
#include "ulaserpoint.h"
#include "ulaserscan.h"
#include "ulaserobst.h"
#include "uresroadline.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResPassable : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResPassable()
  {
    setResID(getResClassID(), 200);
    UResPassableInit();
  };
  /**
  Destructor */
  virtual ~UResPassable();
  /**
   * Initialize class */
  void UResPassableInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "pass"; };
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  Add used parameter variables to the var pool */
  void addPassableIntervalParameters();
  /**
  Get values and settings from var pool */
  void getSettingsFromVarPool();
  /**
  Set new data into ULaserScan.  */
  bool setScan(ULaserData * source, UPose odoPose, UPosRot * laserPose);
  /**
  Get new data into ULaserScan.  */
  ULaserScan * getScan()
  { return scan; };
  /**
  Get number of passable intervals */
  inline int getPisCnt()
  { return pisCnt;};
  /**
  Get this passable interval */
  inline ULaserPi * getPi(int idx)
  { return &pis[idx]; };

  /**
  Add a new passable interval to scan.
  The position left and right are both included in the interval.
  'varMin' is the minimum half-robot-width variance for the interval. */
  bool addPassableInterval(const int right, const int left,
                           double varMin, double varMin2);
  /**
  Generate passable intervals using a line-fit method
  and accept if tilt is less than tiltLimit.
  and do edge detect (obstacles etc). */
  int makePassableIntervalsFit(//UObstacleHist * obsts,
      double * maxPassLeft, double * maxPassRight,
      FILE * logo, FILE * fdI1, FILE * fdI2);
  /**
  Create passable intervals using line-fit variance data.
  This replaces the x-variance method in makePassableIntervals */
  int makePassableIntervals2(const int leftLim, const int rightLim);
  /**
  Combine near intervals if they are close, and the split is
  not blocked by a major obstacle.
  'obstSize is in meter hight ABOVE passable areas.
  'lowVarLimit' is lower limit of variance, when
  compareing two intervals (if actual variance limit is
  lower than this, then this variance is used.) */
  bool combineNearIntervals(double obstSize, double lowVarLimit);
  /**
  Find top of passable interval.
  Returns true if found */
  bool findTopOfRoad(ULaserPi * pp);
  /**
  Get the passable interval (if any), that includes
  this measurement index point */
  ULaserPi * findPi(int withThisIdx);
  /**
  Set parameters used for calculationg variance and
  detection of passable intervals */
/*  void setLimitParams(double sdLimit,
                      double varConvexFactor,
                      double sdConvexOffset,
                      double tiltZLimit,
                      double tiltXLimit,
                      double endpointDev,
                      bool smooth,
                      bool combine );*/
  /**
  Analyze scandata and make passable intervals and obstacles.
  Obstacles are searched up to 'obstSearchExt' further away than
  most distant passable interval.*/
  bool doFullAnalysis(ULaserData * laserData, UPose pose,
                      UPosRot laserPose,
                      UObstaclePool * obsts,
                      bool outdoorContext,  double obstSearchExt,
                      UResRoadLine * roads);
  /**
   * analize for obstacles only, assuming the laserscanner is horisintal alligned
   * \param laserData laserscan to be analized
   * \param pose robot pose at scantime
   * \param laserPose the position and orientation (6D) of laser scanner
   * \param obsts is the obstacle pool, where the obstacles are to be delivered.
   * \param outdoorContext if true obstacles are combined on distance (~0.6 m), else
   * much tighter correlation (0.2m), to avoid to combine wincave walls.
   * \returns true */
  bool doObstAnalysis(ULaserData * laserData, UPose pose,
                      UPosRot laserPose,
                      UObstaclePool * obsts, bool outdoorContext);


                      
protected:
  /**
  Sensor data for robot */
  UPosRot * sensorPose;
  /**
  Passable intervals in this scan */
  ULaserPi pis[MAX_PASSABLE_INTERVALS_PR_SCAN];
  /**
  Number of passable intervals */
  int pisCnt;
  /**
  local variables provided by this resource. */
  ULaserScan * scan;
    /**
   * Parameter for detection of passable interval
   * If line fit variance is above this limit
  a passable interval can not be found here */
  double lineFitVarLimit;
  /**
   * Favor convex shaped roads, by adding this value
  to the signed distance from one of the end-points
  to the fittet line */
  double lineFitConvexOffset;
  /**
   * Factor used when combining variance and
  line curvature, in order to favor convex road shapes. */
  double lineFitConvexFac;
  /**
   * Tilt limit for final passable intervals
  when using y-z coordinate reference (value <= laser tilt) */
  double lineFitZTiltLimit;
  /**
   * Tilt limit for final passable intervals using
  y-x coordinate reference */
  double lineFitXTiltLimit;
  /**
   * Maximum deviation of endpoints of a passable interval
  relative to the minimum line-fit variance in
  the passable interval */
  double lineFitEndpointDev;
  /**
   * Use tresholds that favor flat smooth surfaces
  - i.e. more sensitive to faults. */
  bool lineSmoothSettings;
  /**
   * Debug flag to possibly disable combiner */
  bool lineUseCombine;
  /**
  width of line segment used for calculation of roughness */
  double lineFitWidth;
  /**
  Minimum number of measurements in a segment for roughness calculation */
  int lineFitMinCnt;
  /**
  Minimum combined segment width */
  double minCombinedSegmentWidth;
  /**
  Is context indoor or outdoor */
  //bool outdoorObsts;
  /**
  Statistics */
  int iHeight, iMaxVar, iEndDist, iInside, iMeasureDist;
  /**
   * Group of fixed obstacles from a-priori map (mapbase) */
//  UObstacleGroup * fixeds;
  /**
   * parameter for obstacle detection */
  UVariable * varMinObstDist;
  //UVariable * varUseOutdoorObst;
  UVariable * varMinCombinedWidth;
  UVariable * varUseIntervalCombiner;
  UVariable * varUseSmoothSettings;
  UVariable * varFitEndDev;
  UVariable * varFitXTiltLimit;
  UVariable * varFitZTiltLimit;
  UVariable * varConvexOffset;
  UVariable * varConvexFac;
  UVariable * varRoughMinCnt;
  UVariable * varRoughWidth;
  UVariable * varRoughSDLimit;
  UVariable * varIgnoreIfFixed;
};

#endif

