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
#ifndef URESROADLINE_H
#define URESROADLINE_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>

#include <ulms4/ulaserdata.h>

#include "ulaserpi.h"
#include "uroadline.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
Average minimum roughness for different road types.
The values are used in UReactivePath for determination
of most likely surface type. */
const double ROAD_ROUGHNESS_ASPHALT = 0.0033;
const double ROAD_ROUGHNESS_GRAVEL  = 0.0045;
const double ROAD_ROUGHNESS_GRASS = 0.007;

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResRoadLine : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResRoadLine()
  {
    setResID(getResClassID(), 200);
    UResRoadLineInit();
  };
  /**
  Destructor */
  virtual ~UResRoadLine();
  /**
   * Initialize class */
  void UResRoadLineInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "road"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 167; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  Add used parameter variables to the var pool */
  void addRoadlineParameters();
  /**
  Get values and settings from var pool */
//  void getSettingsFromVarPool();
  /**
  Update road lines using these passable intervals */
  void update(unsigned long scan, ULaserPi * pis, int pisCnt,
              UPoseTime pose, UPosRot * sensorPose, double maxLineRange);
  /**
  Get pointer to road line element */
  inline URoadLine * getRoadLine(int idx)
  { return &roads[idx]; };
  /**
  Get index to best road line */
  inline int getBestLine()
  { return bestLine; };
  /**
  Get number of established road lines */
  inline int getRoadsCnt()
  { return roadsCnt; };
  /**
  Update all lines that has not received an update, that is
  drop unmaintained lines */
  void postUpdate(unsigned int scan, UPoseTime * pose, bool moving);
  /**
  Get left road-edge index.
  * Returns -1 if no valid line is estimated */
  inline int getLeftIdx()
  { return roadLeftIdx; };
  /**
  Get center road-line index.
  * Returns -1 if no valid line is estimated */
  inline int getCentIdx()
  { return roadCenterIdx; };
  /**
  Get right road-edge index.
  * Returns -1 if no valid line is estimated */
  inline int getRightIdx()
  { return roadRightIdx; };
  /**
  Update road variables */
  void updateRoadVariables(UPoseTime * pose);

protected:
  /**
  * Search among detected road lines to find the most likely
    curent road, that is the best left edge to the left that
    has a significant update count.
  * The best line is allowed to slightly on the wrong side of the robot.
  * Returns index to the best of the three road lines */
  int findCurrentRoad(UPoseTime * pose);
  /**
  Second extended version of curent road finder */
  void findCurrentRoad2(UPoseTime * pose);

protected:
  /**
  robot pose (last) */
  UPoseTime odoPose;
  /**
  Initial gain for road line update */
  double gain;
  /**
  Currently best line */
  int bestLine;
  /**
  MAX number of available road lines */
  static const int MAX_ROAD_LINES = 60;
  /**
  Lines */
  URoadLine roads[MAX_ROAD_LINES];
  /**
  Last used road line */
  int roadsCnt;
  /**
  Index to (best estimated) left road line */
  int roadLeftIdx;
  /**
  Index to (best estimated) right road line */
  int roadRightIdx;
  /**
  Index to (best estimated) center road line */
  int roadCenterIdx;
  /**
  Index to road distance var pool variables */
  UVariable * varPoolIdx[13];
  /**
  File handle for road log */
  FILE * roadLog;
  /**
  Next serial number for road lines */
  unsigned long roadSerial;

};

#endif

