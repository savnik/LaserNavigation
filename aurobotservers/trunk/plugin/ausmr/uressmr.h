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

#ifndef URESSMR_H
#define URESSMR_H

#include <cstdlib>

#include <urob4/uresbase.h>
#include <urob4/uresvarpool.h>
#include <urob4/usmrcl.h>
#include <urob4/ucmdexe.h>
#include <urob4/ulogfile.h>


////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

class UMRCUserEvent
{
public:
  /**
   * Constructor */
  UMRCUserEvent()
  {
    id = -1;
    next = NULL;
  };
  /**
   * Destructor */
  ~UMRCUserEvent()
  {};
  /**
   * Event time */
  UTime eventTime;
  /**
   * Event identifier */
  int id;
  /**
   * Link to next event */
  UMRCUserEvent * next;
};

class UResPoseHist;

/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendent) as shown.

@author Christian Andersen
*/
class UResSmrIf : public UResVarPool, public USmrCl, public UServerPush
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResSmr) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResSmrIf()
  {
    setResID(getResClassID(), 208);
    UResSmrIfInit();
  };
  /**
  Destructor */
  virtual ~UResSmrIf();
  /**
   * Initialize class */
  void UResSmrIfInit();
  /**
  Fixed name of this resource type */
  static const char * getResClassID()
  { return "smr"; };
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
   * Called by server core when new resources are available.
   * return true is resouurce is used
   * Save a pointer to the resource as needed. */
  bool setResource(UResBase * resource, bool remove);
  /**
  A new pose state is available in the odoPose structure - update pose history */
  virtual void eventPoseUpdated(bool streamSource);
  /**
   * \brief A watch event occured, update as needed
   * \param name is the name of the watch fired
   * \param atTime is the MRC time reported. */
  virtual void eventWatchFired(const char * name, double atTime);
  /**
  A new GPS position is received from MRC with these values.
  is called from smr interface when streaming is active only.
   * \param heading may be from odo-gps kalman filter
  This requires that the MRC is connected to a gps, as for the HAKO.*/
  virtual void eventGpsUpdate(UPoseTime odoState,
                              double easting, double northing, double heading,
                              double quality, double satellites,
                              double dop);
  /**
  A new INS data is received from MRC with these values.
  It is called if ins-time is updated. */
  virtual void eventInsUpdate(UPoseTime odoState,
                              double accx, double accy, double accz,
                              double roll, double tilt,
                              double pan, double insTime);
  /**
   * hako status variables are received */
  virtual void eventHakoVarUpdate(int hakoManual, int liftPos, int ptoSpeed);
  /**
   * send command to MRC to create a new watch trap
   * \param name name of the watch - a local structure will be added with this name
   * \param condition the condition on wich the watch should be fired (using MRC variables)
   * \returns true if send. */
  bool sendAddWatch(const char * name, const char * condition);
  /**
   * Stop robot by sending flushcmds, stop and idle commands.
   * The connection needs to be open.
   * The socket connection is unchanged after the call. */
  void stopRobot();
  /**
  Stop the robot and shut down the socket connection */
  virtual void closeConnection();
  /**
  Stop any running action, we are going down */
  //virtual void stop(bool andWait);

public:
  /**
  \returns desired speed */
  double getSpeed();
  /**
  \returns desired acceleration */
  double getAcc();
  /**
  \returns desired maximum turn acceleration */
  double getTurnAcc();
  /**
  \returns desired minimum turn radius */
  double getMinTurnRad();
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  \return true if the method call is allowed. */
  bool methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct = NULL,
                                 int * returnStructCnt = NULL);
  /**
  Is pose updates received from MRC and transferred to poseHist? */
  bool isStreaming()
  { return poseStreaming; };
  /**
  \brief is the INS logfile (ins.log) open
  \returns true if open */
  bool isInsLogOpen()
  { return logINS != NULL; };
  /**
  Send a user event (with a new unique ID (increasing number from 101 and up).
   * \param evPreString This string is put in fromt of the ID number -
   * if empty (""), then no string is added.
   * \param evNumber  This is the number (positive) put after the pre-string.
   * if evNumber is negative, the autogenerated number is used (old type)
   * \Returns -1 if not send (not connected or communication error).
   * \returns the user event ID if queued. */
  int sendUserEvent(const char * evPreString, int evNumber);
  /**
  \brief opens or reopens the INS logfile.
  The logfile is intended to be used with data streamed from MRC, and especially when
  connected to HAKO tractor with cross-bow INS module. */
  void openINSlog(bool open);
  /**
  Issue a drive command (and wait until finished */
  double doUserEvent(int repeat);
    /**
   * Test if a specific event is received from MRC */
  bool testEvent(int evID, UTime * eventTime, bool removeIfFound);
  /**
  Issue a drive command (and wait until finished */
  double doSmrDrive(const char * cmd, int repeat);

protected:
  // Locally used methods
  /**
  \brief Create the smrif related variables */
  void createBaseVar();
  /**
  This function is called when an ID-queued -started or finished is received */
  virtual void lineStateUpdated();
  /**
  Called when connection to MRC is created or lost. */
  virtual void connectionChange(bool connected);

  /**
   * Is called by interface when a non-numeric userevent is received from MRC
   * \param eventstring is the string returned by MRC after the
   * keyword 'userevent' and stripped for whitespace. */
  virtual void gotUserEvent(const char * eventString);
  /**
   * Flush all received but not yet tested user events */
  void flushMRCUserEvents();

protected:
  /**
  local variables provided by this resource. */
  /** index to local variable speed */
  UVariable * varSpeed;
  /** index to local variable acc */
  UVariable * varAcc;
  /** index to local variable turnAcc */
  UVariable * varTurnAcc;
  /** index to local variable minimum turn radius */
  UVariable * varMinTurnRad;
  /**  Index to line ID queued */
  UVariable * varIdQueued;
  /**  Index to line ID started */
  UVariable * varIdFinished;
  /**  Index to line ID finished */
  UVariable * varIdStarted;
  /**  Index to line ID with syntax error */
  UVariable * varIdSyntaxErr;
  /**  Index to value of user event */
  UVariable * varIdUserEvent;
  /**
   * hako manual-automatic flag */
  UVariable * varHakoManual;
  /** position of lift */
  UVariable *  varLiftPos;
  /** speed of power take off */
  UVariable * varPtoSpeed;

  /**
  Pointer to pose history resource. */
  UResPoseHist * poseHist;
  /**
  Pointer to pose history resource. */
  UResPoseHist * utmPose;
  /**
  Is poses received using a stream command?
  The value is set when data is received. */
  bool poseStreaming;
  /**
  File handle for GPS log received from MRC */
  ULogFile logGPS;
  /**
  File handle for GPS log received from MRC */
  FILE * logINS;
  /**
  Should a log be made for GPS data */
  bool logGPSuse;

protected:
  /**
  Drive command ID and other options */
  int driveCommandID;
  /**
  Drive command ID and other options */
  int driveUserEventID;
  /**
  Maximum length of a drive command in the SMRCL language */
  static const int MAX_DRIVE_COMMAND_LENGTH = 500;
  /**
  Command last executed */
  //char driveCommandLast[MAX_DRIVE_COMMAND_LENGTH];
  /**
   * List of received user events with keyword "ev" */
  UMRCUserEvent * events;
  /**
   * Lock to ensyre that elements are inserted and deleted in
   * an orderly manner. */
  ULock eventsLock;
};

#endif

