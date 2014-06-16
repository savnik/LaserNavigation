/** *************************************************************************
 *                                                                         *
 *   \file              uresScansaver.h                                    *
 *   \author            Lars Pontoppidan Larsen, Aske Olsson               *
 *   \date              Jan 2007                                           *
 *   \brief             linefinder using auextractfeature lib              *
 *                                                                         *
 *                      Copyright (C) 2006-2008 by DTU                     *
 *                      rse@oersted.dtu.dk                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef URES_AU_EF_H
#define URES_AU_EF_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <ulms4/ulaserpool.h>
#include <auef/auextractfeatures.h>

#include "au2dlineseg.h"

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class UResAuEf : public UResVarPool, public AUExtractFeatures
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResLine) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResAuEf()
  { // set name and version number
    setResID(getResClassID(), 200);
    UResAuEfInit();
  };
  /**
  Destructor */
  virtual ~UResAuEf();
  /**
   * Initialize class */
  void UResAuEfInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "line"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
/*  static int getResVersion()
  { return 195; };*/
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Called by the server core, when a new resource is
  available (or is removed), local pointers to the resource should be
  updated as appropriate. */
  //virtual bool setResource(UResBase * resource, bool remove);

// the above methods are used be the server core and should be present
// the print function is not strictly needed, but is nice.

  /** extract using current extraction criteria */
  int findFeatures(ULaserData * ldata);
  /**
  \brief open or close fixed logfile
  \param close closes the logfile if true, else open */
  void openLog(bool close);
  /**
  \brief ask if logfile is open
  \returns true if open */
  inline bool isLogOpen()
  { return llog != NULL; };
  /**
  \brief Initialize the wall estimate
  Use the longest seen line as a starting point and define other lines from this, setting unseed walls
  as close as possible. */
  bool initializeWalls();
  /**
  \brief update the estimate of walls, assuming the room is rectangular.
  The result is in the wall[0..3] array with 0=north, 1= east, 2=south, and 3=west,
  assuming initial position is heading east, i.e. heading at an approximately right angle towards the east wall.
  Updates further a wallValid[0..4] array.
  The method is to find
  \Returns number of sides that is updated. */
  int updateWalls(UPose newPose);
  /**
  \brief Reset wall estimate to no walls */
  void wallReset();
  /**
   * \brief get the last time the wall lines were updated
   * \return the timestamp, as set in the global variable */
  UTime getLastUpdateTime();

protected:
  /**
  Create global variables for this resiource */
  void createBaseVar();
  /// convert the polar rangedata to cartesian.
  bool toCartesian(ULaserData *ldata, RangeData *rdata);
  /// copy extractions parameters from global to extract function
  void getParams();
  /// convert the angle th into a value in the range +/- pi/2
  double limitToHalfPi(double th);

public:
  /// lock to disallow two threads to be in this function at the same time
  ULock extractLock;
  /// maximum number of lines allowed to be extracted (defined in auextractfeatures.h)
  static const int MAX_LINES = SEGMENTS_MAX;
  /// lines extracted in latest go
  AU2DLineSeg lines[MAX_LINES];
  /// lines extracted in Ax+By+C=0 format
  U2Dlined abcs[MAX_LINES];
  /// number of valid lines
  int linesCnt;
  /// name of logfile
  const char * llogName;
  /// wall lines north, east, west, south - in Ax+By+C=0 format
  U2Dlined walls[4];
  /// wall lines in pos-angle-length format
  AU2DLineSeg wallSeg[4];
  /// pose of box coordinate system
  UPose boxPose;
  /// box size "height" North-south
  double boxNS;
  /// box size "width" east-west
  double boxEW;
  /** \Brief to what level are the walls valid
      -1 = not initialized,
       0 = initialized, but not updated with reliable data
       1 = updated in latest scan
       N = Updated N scans ago, where N=2.. */
  int wallsAge[4];

protected:
  /// logfile handle
  FILE * llog;
  /// resource lock to protect log-file (avoid writing after log is just closed)
  ULock llogLock;
  /// wall correlation angle limit, when correlation with new lines
  double wallAngLimit;
  /// wall correlation distance limit for maintained wall
  double wallDistLimit;
  /// wall correlation distance is ignored if wall is not updated in this number of scans
  int wallAgeResetLimit;
  /// latest pose of robot
  UPose lastPose;
  /// minimum line length for association to a wall line
  double minLineLength;
  /// walls lines longer than this overwrites the distance criteria
  double safeWallLength;
  /// force walls to be at right angle to each other
  bool makeWalls90deg;

  /// index to extract parameter value doSplot
  UVariable * varSplit;
  /// index to extract parameter value doMerge
  UVariable * varMerge;
    /// index to extract parameter value doDiscard
  UVariable * varDiscard;
    /// index to extract parameter value minimum cluster size
  UVariable * varClustMinCnt;
    /// index to extract parameter value cluster separateion distance
  UVariable * varClustDif;
    /// index to extract parameter value split cluster deviation
  UVariable * varSplitDev;
    /// index to extract parameter value Merge deviation
  UVariable * varMergeDev;
    /// index to extract parameter value discard measurement count
  UVariable * varDiscardCnt;
    /// index to extract parameter value minimum line size
  UVariable * varDiscardSize;
    /// index to extract parameter value last found number of lines
  UVariable * varCnt;
    /// index to extract parameter value last analysis time
  UVariable * varTime;
  /// pointer to angle limid
  UVariable * varWallAngLimit;
  /// pointer to wall distance limit
  UVariable * varWallDistLimit;
  /// pointer to wall reset age
  UVariable * varWallAgeLimit;
  /// index to minimum line-to-wall association
  UVariable * varMinLineLength;
  /// index to safeWallLength
  UVariable * varSafeWallLength;
  /// index to makeWalls90deg boolean value
  UVariable * varMakeWalls90deg;
};

#endif

