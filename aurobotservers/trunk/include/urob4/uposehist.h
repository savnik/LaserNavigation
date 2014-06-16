/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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
#ifndef UPOSEHIST_H
#define UPOSEHIST_H

#include <ugen4/ulock.h>
#include <umap4/upose.h>

#include "uresbase.h"

#define MAX_HIST_POSES 2000

/**
Class to hold some 50 meter of pose history for drive calculations

@author Christian Andersen
*/
class UPoseHistNotUsed : public UResBase
{
public:
  /**
    Constructor */
  UPoseHistNotUsed();
  /**
  Constructor */
  ~UPoseHistNotUsed();
  /**
  Fixed name of this resource type */
  static const char * getResID()
  { return "poseHist"; };
  /**
  Fixed varsion number for this resource type.
  Should follow release version, i.e. version 1.28 gives number 128.
  Should be incremented only when there is change to this class
  definition, i.e new or changed functions or variables. */
  static int getResVersion()
  { return 167; };

public:
  /**
  Clear current pose history */
  void clear();
  /**
    Add a history pose (in cronological order) */
    void addPoseHist(UPoseTime poseAtTime);
  /**
    Add a history pose (in cronological order) */
    void addPoseHist(UPose pose, UTime atTime);
  /**
    Add this pose to the pose history - if needed, i.e.
    distance or heading change is above a limit.
    Returns true if position is added to history poses. */
    bool addIfNeeded(UPoseTime pose, double velocity);
  /**
    Get the nearest pose at this time */
    UPose getPoseAtTime(UTime);
  /**
    Get time of oldest data */
    UPoseTime getOldest();
  /**
    Get newest stored position.
    If storage is empty time is zero. */
    inline UPoseTime getNewest(double * velocity)
    {
      if (velocity != NULL)
        *velocity = speed;
      return poseNew;
    };
  /**
  Get the two nearst poses to time  */
  bool getPoseNearTime(UTime atTime, UPoseTime * justBefore,
                         UPoseTime * justAfter);
  /**
  Get Oldest supportet time */
  UTime getOldestTime();
  /**
  Get number of poses in pose history */
  inline int getPosesCnt()
  { return posesCnt; };
  /**
    Get a history pose by index from the newest
    and back - NB! unsafe (if not locked), as element at index
    may have changed - no lock using this function.
    (Relative safe for all the newer poses, but especially the
    older ones may be overwritten.) */
    UPoseTime getPose(int index);
  /**
  Get the two nearest poses this distance away from
  the reference position 'ref'. */
  bool getPoseNearDistance(const double away,
                            const UPose * ref,
                            UPoseTime * closer,
                            UPoseTime * moreDistant);
  /**
  Get time, when robot were (at least) this distance
  from newest position.
  If a pose at this distance is not in buffer, then
  the returned tme is zero. */
  UTime getTimeAtDistance(double away);
  /**
  Get Bet fit line from this distance interval.
  Return the fit variance, but negative (-1.0) if no result.
  Returns the close position, adjusted to the fittet line. */
  double getPoseFitAtDistance(double closeBy, double farAway,
                              const UPose * ref, UPoseTime * closeFitPose);
  /**
  Get heading over a distance in pose history from 'closeBy' to 'farAway'
  (in meters) relative to the 'ref' pose.
  Returns the general heading over this distance (in radians).
  If 'histAge' != NULL, then the age of the last*/
  double getHistHeading(double closeBy, double farAway,
                       const UPoseTime * ref,
                       double * histAge);
  /**
  Save pose history to logfile */
  void saveToLog(FILE * logf);
  /**
  Print the (short) status of resource - to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  Get minimum distance required to store a pose history pose */
  inline double getMinDist()
  { return minDistChange; };
  /**
  Get minimum distance required to store a pose history pose */
  inline double getMinTheta()
  { return minHeadingChange; };
  /**
  Get minimum distance required to store a pose history pose */
  inline double getMinTime()
  { return minTimeChange; };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinDist(double value)
  { minDistChange = value; };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinTheta(double value)
  { minHeadingChange = value; };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinTime(double value)
  { minTimeChange = value; };
  /**
  * Set replay flag.
  * If replay flag is changed to true, the logfile (odo.log) is
    attempted opened.
  * If successfull, the current pose history is cleared and
    the first record read and implemented.
  * If not successfull, the replay flag is set to true,
    and the pose history is cleared only.
  * Returns true if replay file is opened */
  bool setReplay(bool value);
  /**
  * Set replay subdir. Sets the value to the subdir buffer,
    but nothing else is changed. */
  void setReplaySubdir(const char * subdir);
  /**
  Get pointer to replay subdir */
  inline const char * getReplaySubdir()
  { return replaySubPath; };
  /**
  Get replat time now */
  inline UTime getReplayTimeNow()
  { return replayTimeNow; };
  /**
  Get replat time next */
  inline UTime getReplayTimeNext()
  { return replayTimeNext; };
  /**
  Get replay flag */
  inline bool isReplay()
  { return replay; };
  /**
  is replay file open */
  inline bool isReplayFileOpen()
  { return (replayFile != NULL); };
  /**
  Get current line number in replay logfile */
  inline int getReplayLogLine()
  { return replayLogLine; };
  /**
  * Get the full replay logfile name.
  * Returns pointer fn[] string. */
  char * getReplayFileName(char * fn, const int fnCnt);
  /**
   * Replay N singles from the logfile.
   * Returns true if N steps available. */
  bool replayStep(int steps);
  /**
  Replay until just before this time */
  bool replayTime(UTime untilTime);

protected:
  /**
  * Replay a single step from the logfile.
  * Returns true if EOF not found. */
  bool replayStep();

private:
  /**
  Newest pose */
  UPoseTime poseNew;
  /**
  Newest speed */
  double speed;
  /**
  Cyclic buffer of history poses */
  UPoseTime poses[MAX_HIST_POSES];
  /**
  Used poses entries */
  int posesCnt;
  /**
  number of newest pose */
  int newest;
  /**
  Distance change before qualified to be saved */
  double minDistChange; // 0.03
  /**
  Heading change before qualified to be saved */
  double minHeadingChange; // = 1.0 * M_PI / 180.0;
  /**
  Time change before qualified to be saved */
  double minTimeChange; // = 10.0; // seconds
  /**
  Replay flag - source of info for pose history
  a change in this flag clears the current pose history.*/
  bool replay;
  /**
  Replay file handle */
  FILE * replayFile;
  /**
  length of subpath */
  static const int MAX_SUBPATH_LENGTH = 100;
  /**
  Replay file subdir - relative to imagePath */
  char replaySubPath[MAX_SUBPATH_LENGTH];
  /**
  Maximum length of a line in the logfile */
  static const int MAX_LOG_LINE_LENGTH = 200;
  /**
  Buffer for latest line (but not used) line from logfile */
  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Replay time of last used data from logfile */
  UTime replayTimeNow;
  /**
  Replay time of next data from in logfile (line buffer) */
  UTime replayTimeNext;
  /**
  Current line number in logfile */
  int replayLogLine;
};



#endif
