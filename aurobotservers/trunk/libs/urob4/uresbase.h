/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)                        *
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
#ifndef URESBASE_H
#define URESBASE_H

#include <string.h>
#include <stdio.h>
#include <ugen4/ucommon.h>

#include <ugen4/ulock.h>
#include <ugen4/utime.h>

class UCmdExe;
class UVariable;
class UVarPool;

#include "ureplay.h"

/**
The maximum length of the ID describing the resource */
#define MAX_RESOURCE_ID_LENGTH 20
/**
Define the maximum length of the 'isAlso' string */
#define MAX_RESOURCE_ISALSO_LENGTH 100
/**
Base class for a shared resource.
Includes commen handling of replay.

@author Christian Andersen
*/
class UResBase : public UReplay, public ULock
{
public:
  /**
  Constructor */
  UResBase();
  /**
  Destructor */
  virtual ~UResBase();
  /**
  Get resource ID */
  const char * getResID()
  { return resID; };
  /**
  Get version number for this resource */
  int getResVersion()
  { return resVersion; };
  /**
  Tests if this resource matches the provided name */
  bool isA(const char  * idStr);
  /**
  Print the (short) status of resource */
  virtual void print(const char * preString);
  /**
  Print the (short) status of resource - to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Set reference index to function owning the resource.
  This is called by server core and should not be changed. */
  void setResFuncIdx(int idx)
  { resFuncIdx = idx; };
  /**
  Get the function index of the owner of this resource */
  int getResFuncIdx()
  { return resFuncIdx; };
  /**
  Called when resource situation has changed.
  No resources are needed here. */
  virtual bool setResource(UResBase * res, bool remove)
  { return false; };
  /**
  Test if all needed resources are loaded.
  'missingThese is a buffer where missing resources should be written.
  'missingTheseCnt' isthe buffer size not to be resexceeded.
  Should return true if all resources are available. */
  virtual bool gotAllResources(char * missingThese, int missingTheseCnt)
  { return true; };
  /**
  Test if this object has inherited from a ressource with this ID */
  bool isAlsoA(const char * id);
  /**
  Save settings for this resource as needed
  This is called after the stop call, so all action should be stopped at this call.
  but all modules are still loaded.
  */
  virtual void saveSettings()
  { /* nothing at this level */ };

  /**
  Function to set a resource ID, this should
  NOT be called after the resource is known to the server.
  \param[in] *id is the new name (possibly an alias) for this version of the resource */
  virtual void setResID(const char * id, const int version);
  /**
   * Set pointer to core (UCmdExe) */
  void setCorePointer(UCmdExe * serverCore)
  { core = serverCore; };
  /**
   * Get current pointer to core (UCmdExe) */
  UCmdExe * getCorePointer()
  { return core; };
  /**
   * Stop any running subthread - we are going down */
  virtual void stop(bool andWait)
  {
    //printf("stoping %s:%d resource\n", resID, resVersion);
  };
  /**
  Advance replay for all resources in replay mode until just before this time
  The function sets a flag and the steps are performed in main command thread
  as soon as possible (before next user queue, and before push events) */
//  virtual void replayAdvanceTime(UTime untilTime);
  /**
   * Replay one step in the logfile.
   * \returns false if no more steps area available in logfile */
//  virtual bool replayStep();
  /**
  Advance replay for this resources - if in replay mode - until just before this time
   * \param untilTime advance with steps in logfile until this time is reached
   * \returns true if advanced */
//  virtual bool replayToTime(UTime untilTime);
  /**
   * Is the resource in replay mode */
/*  bool isReplay()
  { return replay; };*/
  /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
/*  virtual bool decodeReplayLine(char * line)
  { // default is step is OK
    return true;
  }*/
  /**
   * Get the full replay logfile name.
   * This is the replay path followed by the resource name and the '.log' extension
   * \param fn buffer for destination filename
   * \param fnCnt is the length of the provided buffer
   * Returns pointer to fn[] string. */
//  char * getReplayFileName(char * fn, const int fnCnt);
  /**
   * Get the full logfile name.
   * This is the data path followed by the resource name and the '.log' extension
   * \param fn buffer for destination filename
   * \param fnCnt is the length of the provided buffer
   * Returns pointer to fn[] string. */
  char * getLogFileName(char * fn, const int fnCnt);
  /**
  Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources()
  {};
  /**
   * Replay N singles from the logfile.
   * \param steps number of steps to advance
   * \Returns true if N steps were available in replay file. */
//  int replayStep(int steps);
  /**
  * Set replay flag.
  * If replay flag is changed to true, the logfile (odo.log) is
    attempted opened.
  * If successfull, the current pose history is cleared and
    the first record read and implemented.
  * If not successfull, the replay flag is set to true,
    and the pose history is cleared only.
    * Returns true if replay file is opened */
//    bool setReplay(bool value);
  /**
  Set new replay file name.
  If replay is in progress, then the old file is closed, before assigning the new name.
  \param name is the new mane. */
//  void replaySetFileName(const char * name);
  /**
  Create local replay variables for status updates */
  void createReplayVar(UVarPool * pool);
  /**
  If replay status is maintained in global vars, then it is time to update */
  virtual void updateReplayStatus();

protected:
  /**
   * \brief get a pointer to a static resource.
   * \param resName The name of the resource to be acquired.
   * \param mayCreate if set to true, then the resource will be created
   * if it do not exist already.
   * \param staticOnly if true only a static resource is returned. Should realy
   * only be used for non-static resources if inside the same plugin.
   * \returns NULL if not found, else a bointer to an existing type */
  UResBase * getStaticResource(const char * resName, bool mayCreate,
                              bool staticOnly = true);

protected:
  /**
  Name of resource (identification) */
  char resID[MAX_RESOURCE_ID_LENGTH];
  /**
  Name of resource (identification) */
  char resIsAlso[MAX_RESOURCE_ISALSO_LENGTH];
  /**
  Resource version number.
  This number may be used for a compliance test. */
  int resVersion;
  /**
   * Is the resource in replay mode - this may trigger an update to time call */
//  bool replay;
  /**
   * Next Replay time available in logfile */
//  UTime replayTimeNext;
  /**
  Replay file handle */
//  FILE * replayFile;
  /// max length of replay filename
//  static const int REPLAY_FILE_NAME_LENGTH = 64;
  /// replay filename
//  char replayFileName[REPLAY_FILE_NAME_LENGTH];
  /**
  Maximum length of a line in the logfile */
//  static const int MAX_LOG_LINE_LENGTH = 10000;
  /**
  Buffer for latest line (but not used) line from logfile */
//  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Current line number in logfile */
//  int replayLogLine;

public:
  /**
   * replay to time needs an update - set by the replay advance time call
   * and reset by main command thread when serviced. */
//  bool replayTimeAdvancePending;
  /**
   * Current replay time, and the time that should be advanced to by all other replaying devices */
//  UTime replayTimeNow;

  
private:
  /**
   * Pointer to server core (UCmdExe), to access static resources */
  UCmdExe * core;
  /**
  Local variables for replay status */
  struct {
    UVariable * replay;
    UVariable * line;
    UVariable * time;
  } var;

protected:
  /**
  Index to function owner of resource */
  int resFuncIdx;
};

#endif
