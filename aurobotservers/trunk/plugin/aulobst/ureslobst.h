/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
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

#ifndef URES_LOBST_H
#define URES_LOBST_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>
#include <ulms4/ulaserpool.h>
#include "ecommons.h"


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResLobst : public UResVarPool, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLobst) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResLobst()
  { // set name and version
    setResID("lobst", 859);
    UResLobstInit();
  };
  /**
  Destructor */
  virtual ~UResLobst();
  /**
   * Initialize resource */
  void UResLobstInit();

public:
  /**
  Get list of found obstalcles */
  const char * getList(const char * preStr, char * buff, const int buffCnt);
  /**
  make obstalcles
  \param scan is the laser scan to handle
  \param las is the source laser device
  \returns true if obstacles were found  */
  bool makeObst(ULaserData * scan, ULaserDevice * las);
  /**
  Code the result of the line estimates into a sequence of xml tags, like
  <line x="" y="" th="" l="" name=""/>\n
  <line x="" ..-> />\n
  <line x="" ..-> />\n
  \param buff is the buffer to write the tags into.
  \param buffCnt is the size of the buffer.
  \returns a pointer to the buffer. */
  char * codeLines(char * buff, int buffCnt);
  /**
  Group remaining detections into obstacle groups - from points in calss array pntX, pntYm pntUse.
  \param splitDist is distance to next point for split.
  \param firstCnt is the first number to use when grouping the measurements.
  \returns number of found groups. */
  int makeObstGroups(double splitDist, int firstCnt);
  /**
  Generate obstacle polygons and send to obstacle resource.
  \returns true if send at least one obstacle */
  bool sendAsObstacles();
  /**
  Get number of generated obstacle groups */
  int getObstGrpCnt()
  { return obstGrpCnt; };
  /**
  Get number of generated obstacle groups */
  int getLineCnt()
  { return lineListCnt; };
  /**
  * Function to implement an inter plug-in method call using UVariables as parameter (and return value).
  * Structures of the base type UDatabase (but not UVariables) may be returned in the returnStruct pointer array.
  * The returnStructCnt should be set to returned count, if values are returned.
  * \param name is the name of the called function
  * \param paramOrder is a string with one char for each parameter in the call - d is double, s is string, c is class object.
  * \param params is an array of UVariable pointers, one pointer for each character in paramOrder. NB! params are not declared as constant, and
  *         may be used to return values.
  * \param returnStruct is an array of class object pointers that can be used as parameters or return objects (may be NULL)
  * \param returnStructCnt is the number of objects in the returnStruct buffer
  * \return true if the method is recognised (exists), when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCallV(const char * name, const char * paramOrder, UVariable * params[],
                           UDataBase ** returnStruct, int * returnStructCnt);

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  static const int MAX_RANSAC_LINES = 50;
  GFLine lineList[MAX_RANSAC_LINES];
  // number of valid lines
  int lineListCnt;
  /// number of found - and numbered - obstacle groups.
  int obstGrpCnt;
  /// measurements in angle order
  static const int MPC = 1200;
  /// measurements in angle order - X-coordinate (in robot coordiantes)
  double pntX[MPC];
  /// measurements in angle order - Y-coordinate (in robot coordiantes)
  double pntY[MPC];
  /// measurements point to line - or obstacle group - index.
  int pntUse[MPC];
  /// number of measurements in array
  int pntCnt;
  /// scantime for latest scan
  UTime scantime;
  /// scan serial number
  unsigned int serial;
  //
  /// ransac dist from line to support line
  UVariable * varDistThreshold;
  /// eat measurements this close to lines (after line creation)
  UVariable * varLineEatDist;
  /// minimum number of supporting measurements to create line
  UVariable * varMinLineSupport;
  /// maximum number of line finding iterations
  UVariable * varLineIter;
  /// number of line samples to get best line
  UVariable * varSamples;
  /// max distance between measurements to maintain line
  UVariable * varSplit;
  /// minimum measurements in split part of line
  UVariable * varMinSplitCnt;
  /// enable spilit of long lines.
  UVariable * varEnableSplit;
  /// ignore obstacles that correlated with fixed mapped obstacles
  UVariable * varIgnoreIfFixed;
  /// set global debugDump flag for avoid plugin
  UVariable * varDebugDumpScanOn;
  /// set global debugDump flag for avoid plugin
  UVariable * varDebugDumpScanOff;
  /// most recent processed scan number
  UVariable * varScan;
  /// area where obstacles should be ognored [x,y,x,y] front-left, back right
  UVariable * varNoObst;
  /// make polygons in poly-plugin for detected lines
  //UVariable * varMakePoly;
  /// number of generated laser lines
  UVariable * varLineCnt;
  /// number of maintained obstacles
  UVariable * varObstCnt;
};

#endif

