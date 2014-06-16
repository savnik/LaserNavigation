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
#ifndef UOBSTACLE_H
#define UOBSTACLE_H

#include <ugen4/ulock.h>
#include <ugen4/u3d.h>
#include <umap4/upose.h>
#include <ugen4/upolygon.h>
#include <ugen4/udatabase.h>


/**
A polygon description of an obstacle

@author Christian Andersen
*/
class UObstacle : public UPolygon40
{
public:
  /**
  Constructor */
  UObstacle();
  /**
  Destructor */
  ~UObstacle();
  /**
  Trim obstacle by new scan, i.e
  Either the obstacle is moved or
  pose error has changed, or invis
  due to observation.
  Takes a laserscan and trims obstacle if seen through */
  //bool trimObst(ULaserScan * scan);
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "obstacle";  };
  /**
  Clear obstaccle */
  virtual void clear();
  /**
  Set pose1 - obstacle first seen from this pose.
  also sets pose2, as this is also latest pose. */
  inline void initPoseFirst(UPoseTime pose)
  {
    pose1 = pose;
    pose2 = pose;
    hits = 1;
  };
  /**
  Set pose1 only (update). */
  inline void setPoseFirst(UPoseTime pose)
  { pose1 = pose; };
  /**
  Set pose2 - obstacle latest seen from this pose.
  and increase pose hit cnt */
  void setPoseLast(UPoseTime pose);
  /**
  Get pose1 - obstacle latest seen from this pose. */
  inline UPoseTime getPoseFirst()
  { return pose1; };
  /**
  Get pose2 - obstacle latest seen from this pose. */
  inline UPoseTime getPoseLast()
  { return pose2; };
  /**
  Get ref to pose1 - obstacle latest seen from this pose. */
  inline UPoseTime * getPPoseFirst()
  { return &pose1; };
  /**
  Get ref to pose2 - obstacle latest seen from this pose. */
  inline UPoseTime * getPPoseLast()
  { return &pose2; };
  /**
  Set valid flag */
  inline void setValid(bool value)
  { valid = value; };
  /**
  is valid flag set */
  inline bool isValid()
  { return valid; };
  /**
  Print status of this obstacle */
  void print(const char * prestr);
  /**
  Print obstacle group to string.
  Returns result in buff, a char buffer of length buffCnt. */
  void print(const char * prestr, char * buff, const int buffCnt);
  /**
  Set hit count */
  inline void setHits(int value)
  { hits = value; };
  /**
  Get hits count */
  inline int getHits()
  { return hits; };
  /**
  Save all obstacle verteces to logfile */
  void logObst(FILE * logo);
  /**
  Get serial number for obstacle in group */
  inline unsigned long getSerial()
  { return serial; };
  /**
  Set serial number for obstacle in group */
  inline void setSerial(unsigned long value)
  { serial = value; };
  /**
   * code any special attributes to go into this obstacle,
   * apart from type and name.
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \returns a pointer to the buffer */
  virtual const char * codeXmlAttributes(char * buf, const int bufCnt);
  /**
   * Code this structure in XML format. The open tag includes
   * any additional XML attributes from the codeXmlAttributes function call
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \returns a pointer to the buffer */
  virtual const char * codeXml(char * buf, const int bufCnt, char * extraAttr);
  /**
   * set margin for this obstacle. Margin is a distance
   * from the polygon line, where detections of the obstacle are likely. Used by locater - and obstacle correlation. */
  inline void setMargin(double value)
  { margin = value; };
  /**
   * get margin for this obstacle. Margin is a distance
   * from the polygon line, where detections of the obstacle are likely. Used by locater - and obstacle correlation. */
  inline double getMargin()
  { return margin; };
    /**
  Is other obstacle with meargeable with this. Requires that COG is inside this obstacle, and
  the most extreme vertex of other obstacle is within margin of this obstacle.
  The reverse situation is tested too - with same result.
  \param other is the other obstacle.
  \param margin is the maximum allowed distance of any vertex from other to be outside this obstacle (or the other way around).
  \returns true if mergeable. */
  bool mergeableOnCogLimits(UPolygon * other, double margin);

protected:
  int hits;
  /**
  First seen by */
  UPoseTime pose1;
  /**
  Last seen by */
  UPoseTime pose2;
  /**
  * Is obstacle valid, invalid if as long as it is small
    and not revalidated by another observation */
  bool valid;
  /**
  Obstacle serial number inside obstacle group */
  unsigned long serial;
  /**
   * margin for this obstacle, when correlation
   * measurements to the obstacle. - value from
   * mapbase and used by locater. */
  double margin;
};

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

/**
Obstacle group, that --- within reason --- can be assumed to
have same reference coordinate system, based on odometry.
That is, systematic errors in pose within group, can not be
corrected for. */
class UObstacleGroup : public ULock, public UDataBase
{
public:
  /**
  Constructor */
  UObstacleGroup();
  /**
  Destructor */
  ~UObstacleGroup();
  /**
  Clear obstacle group, but maintain
  the memory allocation of existing obstacle polygons. */
  void clear();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "obstGroup";  };
  /**
  Get first pose in this group */
  inline UPoseTime getPoseFirst()
  { return poseFirst; };
  /**
  Get last pose in this group */
  inline UPoseTime getPoseLast()
  { return poseLast; };
  /**
  Get ref to first pose in this group */
  inline UPoseTime * getPPoseFirst()
  { return &poseFirst; };
  /**
  Get ref to last pose in this group */
  inline UPoseTime * getPPoseLast()
  { return &poseLast; };
  /**
  Get serial number for group */
  inline unsigned long getSerial()
  { return serial; };
  /**
  Set serial number for group */
  inline void setSerial(unsigned long value)
  { serial = value; };
  /**
  Print status */
  void print(const char * prestr);
  /**
  Print obstacle group to string.
  If detail is true, then print details for each obstacle too.
  Returns result in buff, a char buffer of length buffCnt. */
  void print(const char * prestr, char * buff, const int buffCnt, bool detail);
  /**
  Get first pose in this group */
  inline void setPoseFirst(UPoseTime pose)
  {
    poseFirst = pose;
    poseLast = pose;
  };
  /**
  Get first pose in this group */
  inline void setPoseLast(UPoseTime pose)
  { poseLast = pose; };
  /**
  Get a pointer to a new obstacle added to the list.
  The obstacle is created if none is available.
  If no more space, then NULL is returned. */
  UObstacle * getNewObst();
  /**
   * This is where new obstacles are created, the function is virtual, so
   * it may be changed to a more special obstacle type
   * \returns pointer to the newly created obstacle */
  virtual UObstacle * makeNewObst()
  { return new UObstacle(); };

  /**
  Get maximum obstacle count */
  inline int getMaxObstsCnt()
  { return MAX_OBSTACLES; };
  /**
  Get actual count of obstacles */
  inline int getObstsCnt()
  { return obstsCnt; };
  /**
  Get obstacle pointer with this index.
  NB! no range chack in index. */
  inline UObstacle * getObstacle(int idx)
  { return obsts[idx]; };
  /**
  * Get obstacle with this serial number.
  * If the serial number is not found and 'mayCreate' then a new obstacle is added.
  * Returns a pointer to the found obstacle, and the obstacle index number in
  'obstacleIndex' (if 'obstacleIndex' is not NULL).
  * Returns NULL if no more space. */
  UObstacle * getObstacle(unsigned long oSerial, bool mayCreate, int * obstacleIndex);
  /**
  Remove this obstacle from list of active obstacles.
  Swaps with last obstacle in list and then
  reducec count of obstacles. */
  void removeObst(int idx);
  /**
  Remove all obstacles in one go, without clearing the
  start and end poses. */
  void removeAllObsts();
  /**
  Remove invalid obstacles.
  Do a timeout of obstacles valid only once. , i.e earlier than this
  time. */
  void removeInvalid(UTime before, int hitLimit);
  /**
  Log all obstacle in group to - matlab oriented - file */
  void logAll(unsigned int serial, FILE * logo);

public:
  /**
  Number of possible obstacles in a group */
  static const int MAX_OBSTACLES = 500;
protected:
  /**
  List of obstacles (on heap)*/
  UObstacle * obsts[MAX_OBSTACLES];
  /**
  Number of used obstacles */
  int obstsCnt;
  /**
  First poseTime that contributed to this obstacle */
  UPoseTime poseFirst;
  /**
  Latest poseTime that contributed to this obstacle */
  UPoseTime poseLast;
  /**
  Group serial number - the serial number is used when updating to ensure
  the correct group is updated. */
  unsigned long serial;
  unsigned long nextObstSerial;
};



#endif
