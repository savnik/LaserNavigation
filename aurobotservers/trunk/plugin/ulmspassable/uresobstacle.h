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
#ifndef URESOBSTACLE_H
#define URESOBSTACLE_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/uvariable.h>

#include "uobstaclepool.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResObstacle : public UObstaclePool, public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResObstacle()
  { // set name and version number
    setResID(getResClassID(), 200);
    // other local initializations
    addObstacleParameters();
    fixeds = new UObstacleGroup();
  }
  ;
  /**
  Destructor */
  virtual ~UResObstacle();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "obst"; };
  /**
  print status to a string buffer */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  print status to a string buffer */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint( preString, buff, buffCnt); };

  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param params is an array of variable pointers with the actual parameters, in the order specified by order
   * \param returnStruct is an array of class object pointers that can return values or objects (may be NULL) if no result value is needed (a procedure call)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  bool methodCallV(const char * name, const char * paramOrder,
                  UVariable * params[],
                  UDataBase ** returnStruct,
                  int * returnStructCnt);

// the above methods are used by the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
  * Add special variable parameters for obstacle management */
  void addObstacleParameters();
  /**
  * Get the outdoor context variable from the variable pool.
  * Returns true if the outdoor context is true - or if no variable pool is available. */
  inline bool isOutdoorContext()
  { return var.outdoorContext->getBool(); };
  /**
   * Get the outdoor context variable from the variable pool.
   * Returns true if the outdoor context is true - or if no variable pool is available. */
  inline bool getOutdoorExtraDist()
  { return var.extDist->getBool(); };
  /**
  * Get number of fixed obstacles. */
  inline int getFixedObstCnt()
  { return var.fixedObsts->getInt(); };
  /**
  * Load parameter values from current settings pool to this obstacle group */
  virtual void getObstacleGroupSettings(UObstacleGroupLaser * og);
  /**
  Obstacle data is updated - tell resource */
  virtual void obstDataUpdated(UTime poseTime);

  /**
  Extract nearby obstacle lines from mapbase and add as fixed obstacles in odometry coordinates.
  Requires that mapbase and mapPose resources are available beforre the call.
  \returns true if obstacles are available. false if resources are missing. */
  bool updateMappedObstacles();

  
protected:
  // class variables
  struct UVar
  { /**  Index to variable in variable pool for fast access - max pose distance in one group  */
    UVariable * grpMaxDist;
    /**  Index to variable in variable pool for fast access - max time spend in one group */
    UVariable * grpMaxTime;
    /**  Index to variable in variable pool for fast access - obstacle merge distance (outdoor) */
    UVariable * outdoorCombineDist;
    /**  Index to variable in variable pool for fast access - obstacle merge distance (indoor) */
    UVariable * indoorCombineDist;
    /**  Index to variable in variable pool for fast access - is context outdoor */
    UVariable * outdoorContext;
    /**  Index to variable in variable pool for fast access - search distance for obstacles outside passable areas */
    UVariable * extDist;
    /**  Index to variable in variable pool for fast access - should obstacles actually be merged (mostly for debug if false, or when using v360 for memory) */
    UVariable * obstMerge;
    /** Time (tod) of last obstacle update */
    UVariable * updateTime; //
    /** number of obstacle groups in pool */
    UVariable * grps; //
    /** newest obstacle group */
    UVariable * group; //
    /** should obstacle merge be allowed based on COG embedded rules */
    UVariable * obstCogMerge;
    /** distance an COG embedded obstacle may extend outside the other obstacle before a merge is performed */
    UVariable * cogMergeLimit;
    /** single point obstacles merge margin */
    UVariable * obstSingleMargin;
    /** should single point obstacles be merged using special margin */
    UVariable * obstSingleMerge;
    /// number of fixed obstacles (from mapped obstacles)
    UVariable * fixedObsts;
  };
  UVar var;
};

#endif

