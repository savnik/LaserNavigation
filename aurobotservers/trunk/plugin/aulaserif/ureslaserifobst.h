/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
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
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef URESLASERIFOBST_H
#define URESLASERIFOBST_H

#include <urob4/uresbase.h>
#include <urob4/usmltag.h>
#include <urob4/uresifbase.h>
#include <urob4/ulogfile.h>

#include "uobstaclehist.h"

/**
Handling of messages that should be converted to an obstacle history pool

	@author Christian <chrand@mail.dk>
*/
class UResLaserIfObst : public UResIfBase, public UObstacleHist
{
public:
  UResLaserIfObst()
  { // set name and version
    setResID(getResClassID(), 901);
    createVarSpace(20, 0, 2, "Laser server interface data handler", false);
    createBaseVar();
    callDispOnNewData = true;
    latestSerial = 0;
    olog.setLogName("obstc");
  };

  ~UResLaserIfObst();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "laserObst"; };
  /**
  Create base variables and methods for var-pool */
  void createBaseVar();
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
  Print status for this resource */
  virtual const char * snprint(const char * preString, char * buff, int buffCnt);
  /**
  Print status for this resource */
  inline virtual const char * print(const char * preString, char * buff, int buffCnt)
  { return snprint(preString, buff, buffCnt); };
  /**
  Called after decode of a set of obstacle updates */
  virtual void newDataAvailable();
  /**
  The varPool has methods, and a call to one of these is needed.
  Do what is needed and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  The 'paramOrder' indicates the valid parameters d, s or c for double, string or class that is available as input values for the call.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
  of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /** clear all obstacles */
  inline void clear()
  {
    UObstacleHist::clear();
    latestSerial = 0;
    newDataAvailable();
  }
  /**
  Make a short list of obstacles in this group
  \param group is the number of the group - 0 is the newest 1 the next older ....
  \param buff is the string buffer to write to
  \param buffCnt is the length of the string buffer
  \returns a char pointer to the buff */
  const char * listGroup(int group, char * buff, int buffCnt);
  /**
  Make a short list of obstacles in this group
  \param buff is the string buffer to write to
  \param buffCnt is the length of the string buffer
  \returns a char pointer to the buff */
  const char * listGroups(char * buff, int buffCnt);
  /**
  Set logfile open or closed */
  void setLog(bool open)
  {
    if (varOLog != NULL)
      varOLog->setBool(open);
  };

protected:
  /**
    Unpack the obst message. */
  void handleObst(USmlTag * tag);
  /**
  Handle the decode of an obstacle group */
  void handleObstGrp(USmlTag * tag);

public:
  /**
  Logfile */
  ULogFile olog;

protected:
  /**
  Tell display that new data is available */
  bool callDispOnNewData;
  /**
  Latest group serial number */
  unsigned long latestSerial;

private:
  /** index to variable with latest update time */
  UVariable * varTime;
  /** index to variable with latest serial number */
  UVariable * varSerial;
  /** Obstacle update counter */
  UVariable * varUpdate;
  /** index to variable with number of obstacle groups */
  UVariable * varGroups;
  /** Should logfile be created */
  UVariable * varOLog;
};

#endif
