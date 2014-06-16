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
#ifndef URESLASERIFROAD_H
#define URESLASERIFROAD_H

#include <urob4/uresifbase.h>
#include <ugen4/upolygon.h>
#include <ugen4/uline.h>

/**
Relevant data for a road edge line */
class URoadLineData : public UDataBase
{

public:
  /**
  Constructor */
  URoadLineData();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "roadLine";  };

  /**
  Clear line as new */
  void clear();
  /**
  Update line with these data */
  void update(const int type, const unsigned long serial,
              const double lineQ, ULineSegment * seg,
              unsigned long scan);

public:
  /**
  Estimated road line */
  ULineSegment line;
  /**
  Edge 0=left 1=center 2=right */
  int edge;
  /**
  Is line valie */
  //bool valid;
  /**
  Updated by scan number */
  unsigned long scanSerial;
  /**
  Unique serial number for road - for correlation at client end */
  unsigned long lineSerial;
  /**
  Updated from this robot pose (and time) */
  //UPoseTime odoPose;
  /**
  Number of updates to this line */
  int updateCnt;
  /**
  History positions for this line */
  UPolygon400 edgeLine;
  /**
  newest Line quality  */
  double lineQuality;
};



/**
Class to handle detected road lines for reactive behaviour

@author Christian Andersen
*/
class UResLaserIfRoad : public UResIfBase
{
public:
  /**
  Constructor */
  UResLaserIfRoad()
  { // set name and version
    setResID(getResClassID(), 200);
    UResLaserIfRoadInit();
  };
  /**
  Destructor */
  ~UResLaserIfRoad();
  /**
   * Initalize class */
  void UResLaserIfRoadInit();
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
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "laserRoad"; };
  /**
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Is this resource missing any base ressources */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Set ressource as needed (probably not used by this resource) */
  virtual bool setResource(UResBase * resource, bool remove);
  /**
  * The varPool has methods, and a call to one of these are needed.
  * Do the call now and return (a double sized) result in 'value' and
    return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);

public:
  /**
  Get number of available road lines */
  inline int getRoadLinesCnt()
  { return roadLinesCnt; };
  /**
  Get specific road line */
  inline URoadLineData * getRoadLine(int idx)
  { return roadLines[idx]; };
  /**
  Get specific road line */
  URoadLineData * getRoadCurrent(int side);
  /**
  Clear all data (data structures are not released */
  inline void clear()
  {
    roadLinesCnt = 0;
    newDataAvailable();
  };

protected:
  /**
  Unpack the road message and extraction the best estimate of the three road lines. */
  void handleRoad(USmlTag * tag);
  /**
  This function is called, when a new polygon is unpacked.
  It can be used to trigger other functions. */
  virtual void newDataAvailable();
  /**
  Update line with this new road segment update */
  void updateLine(const int lineType, const unsigned long lineSerial,
                  const double lineQ, ULineSegment * lineSeg,
                  const unsigned long scan);
  /**
  Create the variables and methods valid (initially) for this var-pool */
  void createBaseVar();

protected:
  /**
  Latest update time - used to timeout old lines */
  UTime updTime;
  /**
  Latest laser scan number */
  unsigned long scanSerial;
  /**
  Maximum number of polylines used to for road lines */
  static const int MAX_ROAD_LINE_CNT = 20;
  /**
  Road lines */
  URoadLineData * roadLines[MAX_ROAD_LINE_CNT];
  /**
  Number of used road lines */
  int roadLinesCnt;
  /**
  Pointer to the newest line describing the left road side */
  URoadLineData * newestLeft;
  /**
  Pointer to the newest line describing the center road side */
  URoadLineData * newestTop;
  /**
  Pointer to the newest line describing the right road side */
  URoadLineData * newestRight;
  /** index to varpool variable for left roadside - update cnt */
  UVariable * varLN;
  /** index to varpool variable for left roadside - quality */
  UVariable * varLQ;
  /** index to varpool variable for center roadside - update cnt */
  UVariable * varCN;
  /** index to varpool variable for center roadside - quality */
  UVariable * varCQ;
  /** index to varpool variable for right roadside - update cnt */
  UVariable * varRN;
  /** index to varpool variable for right roadside - quality */
  UVariable * varRQ;
  /** index to variable with latest update time */
  UVariable * varUpdateTime;
  /**
  Call display update on new data */
  bool callDispOnNewData;
};

#endif
