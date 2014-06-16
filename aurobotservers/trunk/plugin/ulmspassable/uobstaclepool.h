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
#ifndef UOBSTACLEPOOL_H
#define UOBSTACLEPOOL_H

#include <urob4/uobstacle.h>

#include "ulaserscan.h"

class UObstaclePass : public UObstacle
{
public:
  /**
   * Constructor */
  UObstaclePass()
  {
  }
  /**
   * destructor */
  virtual ~UObstaclePass()
  {
  }
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "obstPass";  };
};

/**
Obstacle group, that --- within reason --- can be assumed to
have same reference coordinate system, based on odometry.
That is, systematic errors in pose within group, can not be
corrected for. */
class UObstacleGroupLaser : public UObstacleGroup
{
  public:
  /**
  Constructor */
  UObstacleGroupLaser();
  /**
  Destructor */
  ~UObstacleGroupLaser();
  /**
  Add this obstacle to group
   * \param scan is the laserscan points
   * \param odoPose is odoPose at scantime
   * \param idx1 is first measurement in obstacle
   * \param idx2 is last measuremnt in obstacle
   * \param outdoorObst changes correlations criteria
   * \param horizontalScan if false, assumed tilted and then obstacles are stretched to correlate with rough grass in last scan.
   * \returns true if added */
  bool addObst(ULaserScan * scan, UPoseTime odoPose,
               int idx1, int idx2,
               bool outdoorObsts,
               bool horizontalScan);
  /**
  * Add this obstacle polygon.
  * \param newpoly The polygon coordinates is assumed to be in odometry coordinates.
  * \param outdoorObst If 'outdoorObsts', then an extra position is assumed added for
    correlation (if not horizontal scan).
  * This position is removed before any merge or create is implemented.
  * \param horizontalScan if false, then the extra correlation position is removed after use (for outdoor obstacles only)
  * \param polyHits is number of hits (scans) this obstacle has received
  * \Returns true if obstacle is merged or added successfuly */
  bool addObstPoly(UPolygon * newpoly, UPoseTime pt,
                   bool outdoorObsts, bool horizontalScan,
                   int polyHits = 1);
  /**
  Set previous postion.
  Previous position is the pose for the set of updates
  before the latest set of updates. */
  inline void setPosePrev(UPoseTime pt)
  { posePrev = pt; };
  /**
  Get previous postion.
  Previous position is the pose for the set of updates
  before the latest set of updates. */
  inline UPoseTime getPosePrev()
  { return posePrev; };
  /**
  Set merge parameters for this obstacle group.
  \param mergeDist - merge if this near other obstacles (and mergeNearObstacles is true)
  \param mergeNearObstacles - allow merge based on distance.
  \param cogLimit - if mergeCogEmbeddedObstacles is true, then the COG embedded obstacles
  may extend this far outside other obstacle and still be merged.
  \param mergeCogEmbeddedObstacles is this COG (Center Of Gravity) embedded merge method enabled. Relevant only if mergeNearObstacles is false.
  \param singleLimit single point obstacles may be merged if they are this close to other obstacles.
  \param mergeSingleObstacles should special single point obstacle merge rules be used. */
  void setMergeDistance(double mergeDist, bool mergeNearObstacles,
                        double cogLimit, bool mergeCogEmbeddedObstacles,
                        double singleLimit, bool mergeSingleObstacles);
  /**
   * This is where new obstacles are created, used by obstacle group,
   * when new obstacles are needed (old obstacles are reused if possible)
   * \returns pointer to the newly created obstacle */
  virtual UObstacle * makeNewObst()
  { return new UObstaclePass(); };

protected:
  /**
  Previous pose, to be used for
  merging obstacles in rough grass */
  UPoseTime posePrev;
  /**
  Merge distance - when obstacles are within this distance from other obstacles */
  double obstacleMergeMargin;
  /**
  Merge distance when COG of one obstacle is inside another, then merge if the embedded is no more than
  this limit outside the other. */
  double obstacleCogMergeLimit;
  /**
  Margin to merge single points with other obstacles. usefull if merge in general is not used, to limit
  number of obstacles. */
  double obstacleSingleMargin;
  /**
  Merge near obstacles */
  bool mergeObstacles;
  /**
  Allow merge of COG embedded obstacles */
  bool mergeCogEmbedded;
  /**
  Should singles be merged on its own criteria */
  bool mergeSingles;
};


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Historic updated obstacles, divided into groups

@author Christian Andersen */
class UObstaclePool
{
  public:
  /**
  Constructor */
  UObstaclePool();
  /**
  Destructor */
  virtual ~UObstaclePool();
  /**
  Clear all obstacles from obstacle pool, that is
  remove all obstacles from all groups, but maintain
  the groups with pose and other informations.*/
  void clear();
  /**
  Clear all obstacles from obstacle group. The 'idx' is the group to
  clear, idx = 0 clears the newest group and idx 1..N clears
  corresponding older groups.*/
  void clearGrp(int idx);
  /**
  Advance to next obstacle group, and
  return pointer to new cleared group. */
  UObstacleGroupLaser * advanceNewGroup();
  /**
  Get the obstacle group relevant for this time and robot position.
  Return pointer to the group. */
  UObstacleGroupLaser * getObstGrp(UPoseTime pt);
  /**
  Add an obstacle, using this scan and this index interval
   * \param scan laserscan
   * \param odoPose is the odoPose at scantime
   * \param idx1 is first measurement of obstacle
   * \param idx2 is last measurment in obstacle
   * \param outdoorObstacle allows more separation if true
   * \param ignoreIfFixed when true, then all obstacles that overlaps fixed obstacles gets ignored.
   * \param horizontalScan if false, then tilted and allows correlation of rough grass areas
   * \returns true if added */
  bool addObst(ULaserScan * scan, UPoseTime odoPose, int idx1, int idx2,
               bool outdoorObsts, bool horizontalScan, bool ignoreIfFixed);
  /**
  Add an obstacle to obstacle group, and advance to new group as intended
   * \param poly obstacle polygon to add
   * \param odoPose is the odoPose at scantime
   * \param outdoorObstacle allows more separation if true
   * \param horizontalScan if false, then tilted and allows correlation of rough grass areas
   * \param ignoreIfFixed when true, then all obstacles that overlaps fixed obstacles gets ignored.
   * \returns true if added */
  bool addObst(UPolygon * poly, UPoseTime odoPose,
                bool outdoorObsts, bool horizontalScan, bool ignoreIfFixed);
  /**
  Add a series of UPosition positions as an obstacle.
  The position need not be orderd.
  A new obstacle group will be added as needed.
  At least one point is needed and no more than 400 can be handled.
  If 'firmObst' then obstacle hit count is set to 2 to avoid
  beeing removed if not correlated.
  Returns true if successful. */
  //bool addObstPoints(UPoseTime poset, UPosition * points, int pointsCnt, bool firmObst);
  /**
  Print status to console */
  void print(const char * prestr);
  /**
  Get number of groups in history */
  inline int getGroupsCnt()
  { return groupsCnt; };
  /**
  Get number of groups in history */
  UObstacleGroupLaser * getGroup(int fromNewest);
  /**
   * Get pointer to obstacle group with near fixed obstacles */
  UObstacleGroup * getFixeds()
  { return fixeds; };
  /**
  Set distance traveled by robot before new obstacle group are to be formed (meter). */
  inline void setNewGrpDist(double value)
  { newGrpDist = value; };
  /**
  Set time passed before new obstacle group are to be formed (in seconds). */
  inline void setNewGrpTime(double value)
  { newGroupTime = value; };
  /**
  Set obstacle logfile */
  void setLogFile(FILE * logFile);
  /**
   * Load parameter values from current settings pool to this obstacle group */
  virtual void getObstacleGroupSettings(UObstacleGroupLaser * og);

  /**
  Obstacle data is updated - tell resource */
  virtual void obstDataUpdated(UTime poseTime);
  /**
   * Set group of fixed obstacles */
  void setFixedObstacles(UObstacleGroup * fixedObstGrp)
  {
    fixeds = fixedObstGrp;
  };
  /**
   * Convert parts of a laserscan to a convex polygon.
   * The measurements are converted to odometry coordinates on return.
   * \param scan is the laserscan
   * \param odoPose is the odometry pose at the time of the scan
   * \param idx1,idx2 is the first and last measurement to be converted into an obstacle polygon.
   * \param p40 is the destination polygon for the result.
   * \param outdoorObsts if frue, then horizontal obstacles and tilted sensor is assumed, and an extra point is added to the polygon to allow better correlation.
   * \param horizontalScan if true, then scan is assumed to be horizontal.
   * \returns true if a valid polygon is created. */
  bool pointsToPolygon(ULaserScan * scan, UPoseTime odoPose,
                       int idx1, int idx2,
                       UPolygon * p40,
                       bool outdoorObsts, bool horizontalScan);

public:
  /**
  Max number of groups */
  static const int MAX_OBSTACLE_GROUPS = 100;

protected:
  /**
  Obstacle groups, each holding a coordinate-wise
  coherent set of obstacles - distance less than e.g 25 m from
  start to end pose. */
  UObstacleGroupLaser * groups[MAX_OBSTACLE_GROUPS];
  /**
  Actual used number of obstacle groups */
  int groupsCnt;
  /**
  The most recent obstacle group.
  When out of groups, then reuse. */
  int newest;
  /**
  Distance traveled by robot before new obstacle group are to be formed (meter). */
  double newGrpDist;
  /**
  Time passed before new obstacle group are to be formed (in seconds). */
  double newGroupTime;
  /**
  Obstacle logfile */
  FILE * logo;
  /**
  Next serial number to use for obstacle group */
  unsigned long nextSerial;
  /**
   * Group of fixed obstacles from a-priori map (mapbase) */
  UObstacleGroup * fixeds;
};


#endif
