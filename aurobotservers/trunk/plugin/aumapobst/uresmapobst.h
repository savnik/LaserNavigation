/***************************************************************************
 *   Copyright (C) 2009 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URESMAPOBST_H
#define URESMAPOBST_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

 * The class has the resource to extract obstacle lines from the map in the vicinity of the current robot position.
 * 
@author Christian Andersen
*/
class UResMapObst : public UResVarPool, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResMapObst) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResMapObst()
  { // set name and version
    setResID("mapobst", 603);
    UResMapObstInit();
  };
  /**
  Destructor */
  virtual ~UResMapObst();
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
   * \param strings is an array of string pointers for the string type parameters (may be NULL if not used)
   * \param doubles is an array with double typed parameters (may be NULL if not used)
   * \param value is the (direct) result of the class, either a double value, or 0.0 for false 1.0 for true
   *              (2.0 for implicit stop if a controll call from mission sequencer).
   * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
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

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  // Public functions that this resource provides for all users.
  /**
   * extract map lines from obstacle bas that is near the current robot pose
   * \param odoPoseOrigin is current robot pose in odometry coordinates, and the related pose time
   * \param obstGrp is the obstacle group, where resulting obstacles are to be placed
   * \returns number of obstacles generated */
  int getNearObstacles(UPose mapPose, UPose odoPoseOrigin, UDataBase * obstGrp);
  /**
  Take map lines and feed them to the localizer using the core command queue.
  \param client is the 'fake' client number putting the commands on the queue.
  \param justObstacles use only those map lines that are marked as obstacles.
  \returns number of map lines loaded into localizer. */
  int sendMapLinesToLocalizer(int client, bool justObstacles);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
   * Initialize resource */
  void UResMapObstInit();

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /// obstacle count in last extraction
  UVariable * varObstCnt;
  /// position of robot at last extraction
  UVariable * varMapPose;
  /// time for last obstacle extraction
//  UVariable * varLastTime;
  /// map search volumen relative to robot
  UVariable * varFront;
  /// map search volumen relative to robot
  UVariable * varBack;
  /// map search volumen relative to robot
  UVariable * varWidth;
  /// part of 'perimeter' that is solid obstacle
  UVariable * varMarginSolidFactor;
  /// part of 'perimeter' that is more fluffy - expect false obstacles
  UVariable * varMarginFluffyFactor;
  /// number of map lines loaded to localizer
  UVariable * varMapLineCnt;
};

#endif

