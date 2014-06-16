/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)                        *
 *   rse@elektro.dtu.dk                                                    *
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
#ifndef UOBJ3DPOOL_H
#define UOBJ3DPOOL_H

#include <urob4/uobstacle.h>


/**
 * Class to hold specific vision based obstacles, that ha additional information on
 * height and colour. */
class UObstacleVision : public UObstacle
{
public:
  /**
   * Constructor */
  UObstacleVision()
  {
    human = false;
  };
  /**
   * Destructor */
  virtual ~UObstacleVision()
  {
  };
  /**
  Clear obstaccle */
  virtual void clear();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "obstVision";  };
  /**
   * code any special attributes to go into this obstacle,
   * apart from type and name.
   * \param buff is the character buffer where the attributes are stored
   * \param buffCnt is the amount of buffer space allocated
   * \returns a pointer to the buffer */
  virtual const char * codeXmlAttributes(char * buff, const int buffCnt);

public:
  /**
   * Is this obstacle likely to be a human */
  bool human;
  /**
   * Is this obstacle source the ground contour */
  bool gndBased;
  /**
   * Height of the obstacle */
  double height;
};

/**
Obstacle group, that --- within reason --- can be assumed to
have same reference coordinate system, based on odometry.
That is, systematic errors in pose within group, can not be
corrected for. */
class UObj3dGroup : public UObstacleGroup
{
  public:
  /**
  Constructor */
  UObj3dGroup();
  /**
  Destructor */
  ~UObj3dGroup();
  /**
  * Add this obstacle polygon.
  * The polygon coordinates is assumed to be in odometry coordinates.
  * If 'outdoorObsts', then an extra position is assumed added for
    correlation. This position is removed before any merge or create
    is implemented.
  * Returns true if obstacle is merged or added successfuly */
  bool addObstPoly(UPolygon * newpoly, UPoseTime pt,
                   bool isHuman, bool isGndBased);
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
  Set merge arameters for this obstacle group */
  void setMergeDistance(double mergeDist, bool doMergeObstacles);
  /**
   * This is where new obstacles are created, used by obstacle group,
   * when new obstacles are needed (old obstacles are reused if possible)
   * \returns pointer to the newly created obstacle */
  virtual UObstacle * makeNewObst()
  { return new UObstacleVision(); };

protected:
  /**
  Previous pose, to be used for
  merging obstacles in rough grass */
  UPoseTime posePrev;
  /**
  Merge distance */
  double obstacleMergeDistance;
  /**
  Merge obstacles */
  bool mergeObstacles;
};


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Historic updated obstacles, divided into groups

@author Christian Andersen */
class UObj3dPool
{
  public:
  /**
  Constructor */
  UObj3dPool();
  /**
  Destructor */
  virtual ~UObj3dPool();
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
  UObj3dGroup * advanceNewGroup();
  /**
  Get the obstacle group relevant for this time and robot position.
  Return pointer to the group. */
  UObj3dGroup * getObstGrp(UPoseTime pt);
  /**
  Print status to console */
  void print(const char * prestr);
  /**
  Get number of groups in history */
  inline int getGroupsCnt()
  { return groupsCnt; };
  /**
  Get number of groups in history */
  UObj3dGroup * getGroup(int fromNewest);
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
  virtual void getObstacleGroupSettings(UObj3dGroup * og);

  /**
  Obstacle data is updated - tell resource */
  virtual void obstDataUpdated(UTime poseTime);
  /**
   * Add this polygon as an obstacle. The obstacle can be either a human
   * or a ground based or both. If not ground based, then the obstacle is detected
   * from higher than ground stereo detecttions.
   * \param newPoly is the new polygon, with positions in robot coordinates.
   * \param odoPose is the robot pose at sensor time
   * \param isHuman is detected from above ground and is "human" sized
   * \param isGndBased obstacle is detected as edge of ground plane. */
  void addObstacle(UPolygon * newPoly, UPoseTime odoPose,
                   bool isHuman, bool isGndBased);
  
public:
  /**
  Max number of groups */
  static const int MAX_OBSTACLE_GROUPS = 100;

protected:
  /**
  Obstacle groups, each holding a coordinate-wise
  coherent set of obstacles - distance less than e.g 25 m from
  start to end pose. */
  UObj3dGroup * groups[MAX_OBSTACLE_GROUPS];
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
};


#endif
