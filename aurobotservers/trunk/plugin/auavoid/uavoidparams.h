/** *************************************************************************
*   Copyright (C) 2010 by DTU (Christian Andersen)
*   jca@oersted.dtu.dk
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
*   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
*   Boston, MA 02110-1301, USA.
***************************************************************************/

#ifndef UAVOIDPARAMS_H
#define UAVOIDPARAMS_H

#include <ugen4/u3d.h>
#include <ugen4/uline.h>
#include <urob4/uobstacle.h>
#include <umap4/uposev.h>
#include <urob4/ulogfile.h>

class UReacObstGrps;
class UReacRoadLines;

/**
Class with parameters to be used during the path generation, and is linked to all path candidates. */
class UAvoidParams
{
public:
  /**
  Constructor */
  UAvoidParams();
  /**
  Destructor */
  ~UAvoidParams()
  {
    if (logAvoid != NULL)
      delete logAvoid;
    logAvoid = NULL;
  }
  /**
  Get safe distance from an obstacle point to other obstacles.
  The calculation uses robot width, minimum turn radius, position of
  front-left and front right extreeme.
  \param avoidLeft that is we are turning right
  \param inner is it the inner save distance (else outher). Outher includes allowance for turn at maximum turn radius (from avoid.minTurnRadius)
  \param tight use tight clearence, rather than the wider normal clearence
  \param linear is the pathe linear (as opposed to an arc)
  If 'tight' is true, the absolute minimum obst clearence distance is used.
  if 'linear' is true, then turn radius is assumed to be 10 times robot width (almost infinity).
    \returns the safe distance from path, where within no obstacles should be found. */
  double getSafeDistance(bool avoidLeft, bool inner, bool tight, bool linear);
  /**
  get diagonal of vehicle, where the with is extended with the desired cleaence
  \param avoidLeft if true, then the front-left corner of vehicle is used. (else ther front right).
  \param tight if true, then the minimum clearence is udes, else the desired clearence.
  \param diagAngle is (if not NULL) the diagonal angle away from forward direction - always positive.
  \returns the diagonal to the desired corner. */
  double getDiagonal(bool avoidLeft, bool tight, double * diagAngle);
  /**
  Get minimum turn radius */
  double getMinTurnRad();
  /**
  Get radius of smallest wall to wall turn circle.
  \param avoidLeft if true then avoiding an obstacle to the left (i.e. turning right).
  \param turnRadius is the actual turnradius.
  \returns circle radius for wall to wall circle (no safety space) */
  double getWallWallRadius(bool avoidLeft, double turnRadius);
  /**
  Get distance from robot centre (0,0) to one of the front corners.
  \param left get distance to the front-left corner - else the right corner (if not justMax)
  \param justMax return the bigger distance of either corner.
  \returns a positive distance - no clearence is added. */
  double getDistToCorner(bool left, bool justMax);
  /**
  Get the minimum opening, where this robot cam pass - inclusive of minimum clearence.
  \returns the minimum opening width. */
  double getMinOpening();
  /**
  Is this position within robot envelope plus a margin.
  \param UPose robot position.
  \param pos is position to test.
  \param margin to add to envelope - positive makes area bigger.
  \returns true if within. */
  bool withinRobotOutline(UPose robot, UPosition pos, double margin);


public:
  /**
  Minimum distance to obstacles (old method) */
  double obstMinDist;
  /**
  Absolute minimum distance to obstacles (old method) */
  double obstMinMinDist;
  /**
  Desired exit pose and speed */
  UPoseV exitPose;
  /**
  Start pose for the planning process */
  UPoseV startPose;
  /**
  Maximum lateral acceleration allowed */
  double maxAcceleration;
  /**
  Maximum turn (centrifugal) acceleration allowed */
  double maxTurnAcceleration;
  /**
  Minimum turn radius. This is assumed to be a hard limit. */
  double minTurnRadius;
  /**
  When true, then failed paths will survive marked as crashed */
  bool doCrashTest;
  /**
  Maximum number of nested levels, that is may a spawned path spawn further */
  int maxNestedLevels;
  /**
  Maximum number of loops testing for potential mid-poses */
  int maxAvoidLoops;
  /**
  Maximum number of spawned paths - in total */
  int maxSpawnCnt;
  /**
  Most limiting front left position (x is forwared y is left) */
  UPosition frontLeft;
  /**
  Most limiting front right position (x is forwared y is left) */
  UPosition frontRight;
  /**
  Pointer to obstacles that potentially can obstruct the path */
  UReacObstGrps * obsts;
  /**
  Potentially available roadlines that should be respected */
  UReacRoadLines * roads;
  /**
  Logfile for obstacle avoidance */
  ULogFile * logAvoid;
  /**
  Serial number for this obstacle avoidance route */
  int avoidSerial;
  /**
  Desired minimum obstacle clearence distance */
  double obstClearanceDesired;
  /**
  Absolute minimum obstacle clearence distance */
  double obstClearanceMinimum;
  /**
   * use driveon manoeuver estimation, rather the trying to go exactly to the the end pose. */
  bool useDriveon;
  /** driveon gain for angle */
  double driveonGA;
  /** driveon gain for distance from line (max equal angle value for 90 deg).
   * The approximated turn angle is (pi/2 * GA / GD) */
  double driveonGD;
  /**
  Allow obstacle avoidance routes that starting within this angle relative to current heading */
  double forwardOnly;
  /**
  Max number of tangent depth in path search, i.e. how many obstacles
  could be touched in the visibility graph (to limit search time) */
  int maxTangentDepth;
  /**
  Maximum number of tangent sequences that are tested if no valid
  manoeuvre can be found for the first tangent sequence */
  int maxTangentToMan;
  /**
  Debug parameter to ignore close obstacles after this number of
  avoided obstacles - apart from those in the visibility graph.  */
  int ignoreCloseObstAfter;
  /**
  Stop calculation after this solution number (debug value), default = 999 */
  int acceptAfterSolution;
  /**
  Try to follow line on last pose - usable in row following */
  bool followLineLastPose;
  /**
  Use any result - OK or not (debug flag) */
  bool useAnyResult;
  /**
  Use new cell-decomposition method */
  bool cellRev2;
  /**
  (debug flag) Make cell polygons 0=not, 1=all, 2=used only */
  int makeCellPolygon;
  /**
  (debug flag) limit for polygon size for display */
  double makeCellPolygonMaxY;
  /**
  (debug flag) Make manoeuvre footprint polygons */
  bool makeFootprintPolygon;
  /**
  scannumber of last processed laserscan (for obstacle detection) - debug info */
  int scanSerial;
};

#endif // UAVOIDPARAMS_H
