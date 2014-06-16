/** \file uresmanager.h
 *  \ingroup Mission manager
 *  \brief UResource class for Robot Mission manager
 *
 *  Robot mission manager plugin for AU Robot Servers
 *
 *  \author Anders Billesø Beck
 *  $Rev: 51 $
 *  $Date: 2009-03-20 13:48:05 +0100 (Fri, 20 Mar 2009) $
 */
/***************************************************************************
 *                  Copyright 2009 Anders Billesø Beck, DTU                *
 *                       anders.beck@get2net.dk                            *
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

#ifndef URESMANAGER_H
    #define URESMANAGER_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>

#include <uresmapbase.h>

#include "robotmanager.h"


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Anders Billesø Beck
*/
class UResManager : public UResVarPool, public ULogFile, public UServerPushImplement
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResCron) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResManager()
  { // set name and version
    setResID("manager", 100);
    setDescription("Robot Mission Manager");
    UResManagerInit();
  };
  /**
  Destructor */
  virtual ~UResManager();
  /**
   * Initialize resource */
  void UResManagerInit();
  /**
  print status to a string buffer */
  virtual const char* print(const char * preString, char * buff, int buffCnt);
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
  The server will offer a resource pointer by this call.
  If the resource is used, please return true. */
  bool setResource(UResBase * resource, bool remove);

  virtual void createResources();

// the above methods are used be the server core and should be present
// the print function is not strictly needed, but is nice.



public:
  // Public functions that this resource provides for all users.
  /**
  Run the time loop to test for jobs to be activated.
  This call do not return until the threadStop flag is set true. */
  void run();
  /**
   * Service function for push queue - to store owner of function */
  int findFunctionOwner(const char * tagName);
  /**
   * Plan a route between two map locations
   **/
  int   planRoute(const char*, const char*);

  /**
   *Execute a previously planned route
   */
  int   executePlan(void);


protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  /**
  Start read thread
  \return Returns true if the read thread started. */
  bool start();
  /**
   * Stop read thread - and wait for thread join 
   * \param andWait wait for thread to finish (ignored, waits always)
  */
  void stop(bool andWait);

  /**
   * Send a string to the SMR
   */
  bool sendSmr(string);


public:
  /** Robot state object
   */
  robotManager rManager;
  
  /** Pointer to global mapbase resource
   */
  UResMapbase *mapbase;
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /**
  Thread handle for frame read thread. */
  pthread_t threadHandle;
  /**
  Is thread actually running */
  bool threadRunning;
  /**
  Should thread stop - terminate */
  bool threadStop;

  /**
   * Global variable pool
   */
  UResVarPool * varGlobal;

  bool  executing;

 //***Variables template...***
 /* /// index to idle count 
  UVariable * varIdleCnt;
  /// index to number of commands executed
  UVariable * varJobCnt;
  /// number of commands that returned error
  UVariable * varErrCnt;
  /// time it took to execute last command
  UVariable * varLastTime;
  /// index to minimum time interval between cron jobs
  UVariable * varCronTime;*/
};

#endif

