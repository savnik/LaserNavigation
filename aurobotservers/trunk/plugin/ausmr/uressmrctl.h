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

#ifndef URESSMR_CTL_H
#define URESSMR_CTL_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/ucmdexe.h>
#include <umap4/umanseq.h>

#include "uressmr.h"


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResSmrCtl : public UResVarPool
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResSmr) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResSmrCtl()
  { // set svn version number for most recent change
    setResID(getResClassID(), 208);
    UResSmrCtlInit();
  };
  /**
  Destructor */
  virtual ~UResSmrCtl();
  /**
   * Initialize class */
  void UResSmrCtlInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "smrctl"; };
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
   * Called by server core when new resources are available.
   * return true is resouurce is used
   * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);

// the above methods are udes be the server core and should be present
// the print function is not strictly needed, but is nice.
public:
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed. */
  bool methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct = NULL,
                                 int * returnStructCnt = NULL);
  /**
  Start running in thread */
  bool start();
  /**
  Stop drive thread and close logfile */
  virtual void stop(bool andWait);
  /**
  Main loop - supposed to run in a thread.
  Checkes the waypoint list and
  issued commands to the smr if a new list is available.*/
  void run();
  /**
  Start pose streaming -
  this will not be started if the control
  loop is not running.
  Returns true if stream start is send. */
  bool startPoseStreaming(int interval);


protected:
  // Locally used methods
  /**
  Create the smrif related variables */
  void createBaseVar();
  /**
  Implement this new short term drive plan from this driver.
  The plan may be discarded, if it is very close to to the last received
  plan from the same driver. */
  bool implementDrivePath(const char * driverName, UManSeq * man);
  /**
  Send this manoeuvre sequence to the robot real time controller */
  bool sendNewManoeuvreToSMR(UManSeq * manseq);
  /**
   * \brief Get the command string to pass to MRC for a drive type command
   * to this end pose (starting at the start pose).
   * at the break distance a user event should be plased in form of a \t (tab) character
   * on an otherwise empty line.
   * \param cmd is the string buffer, where the command should be placed.
   * \param cmdCnt is the size of the buffer
   * \param startPose is the assumed current pose
   * \param endPose is the desired end pose
   * \param breakAt is the distance from start pose to placement of the tab character
   * \param order distance is the end distance
   * \param final indicates that the order distance is to the end of the manoeuver, and
   * thus cam be used directly as the order distance.
   * \param t is current odometry time - for debug only, may be NULL */
  void getSMRCLDrive2cmd(char * cmd, const int cmdCnt,
                         UPoseV startPose, UPoseV endPose,
                         double breakAt, double orderDist,
                        bool final, UTime * t);

  
protected:
  /**
  local variables provided by this resource. */
  /**
  Pointer to resource with interface to MRC */
  UResSmrIf * smrif;
  /**
  Pointer to resource with current pose information */
  UResPoseHist * poseHist;
  /**
  Name of driver (is a string owned by the driver, and
  the pointer adress is used to determine a change
  in driver responcibility. */
  const char * driver;
  /**
  Newest plan of the type UManSeq to be implemented. */
  UManSeq * currentPlan;
  /**
  Newest plan of the type UManSeq to be implemented. */
  UManSeq * implementedPlan;
  /**
  Set this flag to true, when the current plan is changed */
  bool currentPlanNew;
  /**
  Stop non-realtime robot controller thread */
  bool stopFlag;
  /**
  Is non-realtime robot controller thread running */
  UVariable * varRunning;
  /**
  Index to variable with latest user event ID posted for the latest manoeuvre */
  UVariable * varManEndID;
  /**
  Index to time when the user event ID were posted for the latest manoeuvre */
  UVariable * varManEndTime;
  /**
  Index to time it takes to queue a manoeuvre command (the latest) */
  UVariable * varManQueueingTime;
  /**
  distance traveled where new man should be implemented even if it looks like the old plan,
  (to avoid stand still). */
  UVariable * varRenewDriveDist;
  /**
  minimum distance change in destination to trigger command renewal */
  UVariable * varRenewDestDist;
  /**
  minimum heading change in destination to trigger command renewal. */
  UVariable * varRenewDestHeading;
  /**
  minimum velocity change in destination to trigger command renewal. */
  UVariable * varRenewDestVel;
  /**
  Index to planning distance when implementing a manoeuvre */
  UVariable * varManPlanDist;
  /**
   * variable with max deviation from end-pose line for using drive command only */
  UVariable * varUseDriveonDist;
  /**
   * variable with max deviation from end-pose heading for using drive command only */
  UVariable * varUseDriveonHeading;
  /**
   * flag that indicates, that the received manoeuvre is direct to destination */
  UVariable * varManIsDirect;
  /**
   * Use driveon command if manoeuvre indicates direct is possible */
  UVariable * varUseDriveonIfDirect;
  /**
   * should direct manoeuvers be accepted only? */
  UVariable * varDirectOrWait;
  /**
   * are we waiting for a direct manoeuvre? */
  UVariable * varDirectWait;
  /**
  local variable is true if thread is running */
  bool running;
  /**
  Last ID of the main part of a manoeuvre sequence */
  int driveCmdFinishedID;
  /**
  Time the last manoeuvre were implemented */
  UTime driveCmdFinishedIDTime;
  /**
  Latest implemented start pose */
  UPose driveCmdStartPose;

public:
  /**
   * Logfile for drive controller */
  ULogFile logctl;
  /**
   * Logfile for drive primitives, lines and arcs, actually commanded */
  ULogFile logprim;

private:
  /**
   * Thread handle for main loop. */
  pthread_t  thDrive;
  /**
   * Simulated filename (for status print) */
  //char logFileName[MAX_FILENAME_SIZE];
};

#endif

