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
#ifndef UREPLAY_H
#define UREPLAY_H

#include <string.h>
#include <stdio.h>
#include <ugen4/ucommon.h>

#include <ugen4/utime.h>


/**
Base class for a shared resource.
Includes commen handling of replay.

@author Christian Andersen
*/
class UReplay
{
public:
  /**
  Constructor */
  UReplay();
  /**
  Destructor */
  virtual ~UReplay();
  /**
  Advance replay for all resources in replay mode until just before this time
  The function sets a flag and the steps are performed in main command thread
  as soon as possible (before next user queue, and before push events) */
  virtual void replayAdvanceTime(UTime untilTime);
  /**
   * Replay one step in the logfile.
   * \returns false if no more steps area available in logfile */
  virtual bool replayStep();
  /**
  Advance replay for this resources - if in replay mode - until just before this time
   * \param untilTime advance with steps in logfile until this time is reached
   * \returns true if advanced */
  virtual bool replayToTime(UTime untilTime);
  /**
   * Is the resource in replay mode */
  bool isReplay()
  { return replay; };
  /**
   * Is the resource in replay mode */
  bool isReplayFileOpen()
  { return replayFile != NULL; };
  /**
  Decode this replay line, that is assumed to be valid.
  \param line is a pointer to the line.
  \returns true if the line is valid and used (step is successfull).
  \returns false if line is not a valid step. */
  virtual bool decodeReplayLine(char * line)
  { // default is step is OK
    return true;
  }
  /**
   * Get the full replay logfile name.
   * This is the replay path followed by the resource name and the '.log' extension
   * \param fn buffer for destination filename
   * \param fnCnt is the length of the provided buffer
   * Returns pointer to fn[] string. */
  char * getReplayFileName(char * fn, const int fnCnt);
  /**
   * Get the full logfile name.
   * This is the data path followed by the resource name and the '.log' extension
   * \param fn buffer for destination filename
   * \param fnCnt is the length of the provided buffer
   * Returns pointer to fn[] string. */
  char * getLogFileName(char * fn, const int fnCnt);
  /**
   * Replay N singles from the logfile.
   * \param steps number of steps to advance
   * \Returns the line number reached in the replay file and -1 if no usable file. */
  int replayStep(int steps);
  /**
  Set replay flag.
  \param value if true, then the logfile (name.log) is  attempted opened.
               If not successfull, the replay flag is still set to true, and attemped opened at next step
  \Returns true if replay file is opened or not */
  bool setReplay(bool value);
  /**
  Set new replay file name (fully qualified filename).
  If replay is in progress, then the old file is closed, before assigning the new name.
  \param name is the new mane. */
  void replaySetFileName(const char * name);
  /**
  Set basename for replay file, this is preceded by the replay path and a '.log' is added.
  \param name logfile name (excluding replay path and .log extension)
  \param preName optional additional string inserted just in front of name (no '/' or '.' added) */
  void replaySetBaseFileName(const char * name, const char * preName = "");
  /**
  If replay status is maintained in global vars, then it is time to update */
  virtual void updateReplayStatus()
  {};
  /**
  Set replay parent
  \param pointer to parent, parent has to be informed on steps, and if this child is deleted,
  then parent will inform if other childs has advanced. */
  void setParent(UReplay * parent);
  /**
  Add a child to this replay structure. */
  void addChild(UReplay * child);
  /**
  Remove this child from replay list
  \param child is a pointer to the replay structure of the child to be removed.
  The child pointer is removed and the cound of children is decreased.
  If child is not found, then nothing happends. */
  void removeChild(UReplay * child);
  /**
  Get current replay log linenumber */
  inline int getReplayLogLine()
  { return replayLogLine; };
  
protected:
  /**
   * Is the resource in replay mode - this may trigger an update to time call */
  bool replay;
  /**
   * Next Replay time available in logfile */
  UTime replayTimeNext;
  /**
  Replay file handle */
  FILE * replayFile;
  /// max length of replay filename
  static const int REPLAY_FILE_NAME_LENGTH = 264;
  /// replay filename
  char replayFileName[REPLAY_FILE_NAME_LENGTH];
  /**
  Maximum length of a line in the logfile */
  static const int MAX_LOG_LINE_LENGTH = 10000;
  /**
  Buffer for latest line (but not used) line from logfile */
  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Current line number in logfile */
  int replayLogLine;
  /**
  if not a resource itself, then set pointer to parent resource, to handle the replay of other foles */
  UReplay * replayParent;

public:
  /**
   * replay to time needs an update - set by the replay advance time call
   * and reset by main command thread when serviced. */
  bool replayTimeAdvancePending;
  /**
   * Current replay time, and the time that should be advanced to by all other replaying devices */
  UTime replayTimeNow;

  
private:
  /// list of childs that needs to be informed of replay steps.
  UReplay ** childs;
  /// number of childs in the child list.
  int childsCnt;
};

#endif
