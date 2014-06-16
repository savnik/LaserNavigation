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
 * $Id: uresposehist.h 1883 2012-04-08 05:37:46Z jca $
 ***************************************************************************/
#ifndef URESPOSEHIST_H
#define URESPOSEHIST_H

#include <ugen4/ulock.h>
#include <umap4/upose.h>

//#include "uresbase.h"
#include "uresvarpool.h"
#include "ucmdexe.h"
#include "uvariable.h"
#include "ulogfile.h"

/// max pose history count normal is 2000
#define MAX_HIST_POSES 5000

/**
Class to hold some 50 meter of pose history for drive calculations

@author Christian Andersen
*/
class UResPoseHist : public UResVarPool, public UServerPush
{
public:
  /**
    Constructor */
  UResPoseHist()
  { // set default name and version number
    setResID("odoPose", 1882);
    posesCnt = 0;
    newest = -1;
    replay = false;
    replayFile = NULL;
    replayLine[0] = '\0';
    replayLogLine = 0;
    newestSource = -1;
    replayTime2 = false;
    createBaseVar();
    odoPoseHist = NULL;
    odoPoseOrigin.clear();
    odoPoseOrigin.t.setTime(0.0);
    poseNew.clear();
  };
  /**
  Constructor */
  virtual ~UResPoseHist();
  /**
  Set resource ID - may be an alias name */
  virtual void setResID(const char * aliasName);
  /**
  Set resource ID - may be an alias name */
  virtual void setResID(const char * id, const int version)
  { UResBase::setResID(id, version); };
  /**
  Set (or remove) ressource (core pointer needed by event handling) */
  bool setResource(UResBase * resource, bool remove);
  /**
  Test if pool has all needed resources. */
  bool gotAllResources(char * missingThese, int missingTheseCnt);
  /**
  Constant name for defined alias for pose hist - odometry based on whell, INS or the like */
  static const char * getOdoPoseID() { return "odoPose"; };
  /**
  Constant name for defined alias for pose hist - UTM based on GPS source - may jump */
  static const char * getUtmPoseID() { return "utmPose"; };
  /**
  Constant name for defined alias for pose hist - from localization based on map coordinate system - may differ from utm */
  static const char * getMapPoseID() { return "mapPose"; };

public:
  /**
  Clear current pose history */
  void clear();
  /**
  Create initial var-pool variables */
  void createBaseVar();
  /**
    Add a history pose (in cronological order) */
    void addPoseHist(UPoseTVQ poseAtTime);
  /**
    Add a history pose (in cronological order) */
    void addPoseHist(UPose pose, UTime atTime, double v = 0.0, double qual = -1.0);
  /**
    Add this pose to the pose history - if needed, i.e.
    distance or heading change is above a limit.
    Returns true if position is added to history poses. */
  bool addIfNeeded(UPoseTVQ pose, int source);
  /**
    Get the nearest pose at this time.
    \param time is the time where pose is requested.
    \returns pos if it is available - interpolated from stored poses.
    if not available (time too new) and is the odometry pose, then the newest available pose is returned.
    if not available (too new) and not odometry, then the newest pose is extended using the odometry relation, to
      resulting pose when requesting the pose from the odometry pose store, and converted to the requested coordinate system.
      The returned time is the used pose-time.
      The returned 'q' and 'v' is the newest available quality from this coordinate system, even if extended by odometry.
      (the 'getNewest()' will get the available newest update to this coordinate system. */
  UPoseTVQ getPoseAtTime(UTime time);
  /**
    Get time of oldest data */
  UPoseTime getOldest();
  /**
    Get newest stored position.
    If storage is empty time is zero. */
  inline UPoseTime getNewest(double * velocity)
  {
    if (velocity != NULL)
      *velocity = poseNew.vel;
    return poseNew.getPoseTime();
  };
  /**
   * Get newest available pose */
  inline UPoseTVQ getNewest()
  {
    return poseNew;
  };
  /**
    Get source of newest stored position (a client number). */
  inline int getNewestSource()
  { return newestSource; };
  /**
  Get the two nearst poses to time  */
  bool getPoseNearTime(UTime atTime, UPoseTVQ * justBefore,
                         UPoseTVQ * justAfter);
  /**
  Get Oldest supportet time */
  UTime getOldestTime();
  /**
  Get number of poses in pose history */
  inline int getPosesCnt()
  { return posesCnt; };
  /**
   * Get index to newest saved pose */
  int getNewestIndex() { return newest; };
  /**
   * get pose at this index to the circulat queue */
  inline const UPoseTVQ * getPoseAtIndex(int index) const
  { if (index >= 0 and index < MAX_HIST_POSES)
      return & poses[index]; 
    else 
      return NULL;
  };
  /**
  Get a history pose by index from the newest
  and back - NB! unsafe (if not locked), as element at index
  may have changed - no lock using this function.
  (Relative safe for all the newer poses, but especially the
  older ones may be overwritten.) */
  UPoseTVQ getPose(int index);
  /**
  Get the two nearest poses this distance away from
  the reference position 'ref'. */
  bool getPoseNearDistance(const double away,
                            const UPose * ref,
                            UPoseTVQ * closer,
                            UPoseTVQ * moreDistant);
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
                              const UPose * ref, UPose * closeFitPose);
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
  { return varMinDist->getValued(); };
  /**
  Get minimum distance required to store a pose history pose */
  inline double getMinTheta()
  { return varMinTurn->getValued(); };
  /**
  Get minimum distance required to store a pose history pose */
  inline double getMinTime()
  { return varMinTime->getValued(); };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinDist(double value)
  { varMinDist->setValued(value, 0, false); };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinTheta(double value)
  { varMinTurn->setValued(value, 0, false); };
  /**
  Set minimum distance required to store a pose history pose */
  inline void setMinTime(double value)
  { varMinTime->setValued(value, 0, false); };
  /**
  * Set replay flag.
  * If replay flag is changed to true, the logfile is attempted opened.
  * If successfull, the current pose history is cleared and
    the first record read and implemented.
  * If not successfull, the replay flag is set to true,
    and the pose history is cleared only.
  * Returns true if replay file is opened */
  bool setReplay(bool value);
  /**
  Get replat time now */
  inline UTime getReplayTimeNow()
  { return replayTimeNow; };
  /**
  Get replat time next */
  inline UTime getReplayTimeNext()
  { return replayTimeNext; };
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
  * Returns pointer to fn[] string. */
  char * getReplayFileName(char * fn, const int fnCnt);
  /**
   * Replay N singles from the logfile.
   * Returns true if N steps available. */
  bool replayStep(int steps);
  /**
  Replay until just before this time */
//  bool replayTime(UTime untilTime);
  /**
  The varPool has methods, and a call to one of these are needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
   * May return struct values in the 'returnStruct' pointer list (when the
  pointer list is not NULL), and in this case the 'returnStructCnt' is set
  to the maximum number of pointers available and should be modified
  to the number of valid pointers returned.
  The method should return true if the method exist (with this parameter list) */
  virtual bool methodCall(const char * name, const char * paramOrder,
                          char ** strings, const double * doubles,
                          double * value,
                          UDataBase ** returnStruct = NULL,
                          int * returnStructCnt = NULL);
  /**
  Open pose history logfile, logs whenever a pose is stored
  in the pose history resource.
  the file is 'aliasName'.log and is stored in default dataPath.
  The format is time x y h, like: <br>
  123456789.111222 130.122 64.122 3.14076 <br>
  Returns true if logfile is open.
  If logfile is already open, then it is closed and reopened. */
  bool openLogfile();
  /**
  Close the logfile */
  void closeLogfile();
  /**
  Is logfile open */
  inline bool isLogfileOpen()
  { return logPose.isOpen(); };
  /**
  Get logfile name */
  inline const char * getLogfileName()
  { return logPose.getLogFileName(); };
  /**
  Set replay timestamp source to time stamp 2 (in logfile). */
  inline void setReplayTimestamp2(bool value)
  { replayTime2 = value; };
  /**
  Get value of replay timestamp source (timestamp 1 or 2 - in logfile).
  \return true if timestamp source is 2. */
  inline bool getReplayTimestamp2()
  { return replayTime2; };
  /**
   * Convert a utmPose source file to a kml-line for google earth
   * \param name is a string buffer with source name on entry
   * and conversion result on return.
   * \returns true if file found. */
  bool toKml(char * aKmlName, const int MRL);
  /**
   * Convert source utmPose file to kml for google earth.
   * \param fs is a source file handle with UTM coordinates
   * source format: time, east, north, heading (rad), speed, quality (gps-mode), (extra timestamp)
   * 1241603463.054044 707875.529 6174294.230 -2.09900 0.071 4 1241603463.056364
   * \param fd destination file
   * \param doLine make google draw a line (else icons)
   * \returns true if all lines had 6 floating point values (false if last line was less than full) */
  bool processUtmFile(FILE * fs, FILE * fd, bool doLine = true);
  /**
   * get position of odometry origin in this coordinate system */
  UPoseTime getOdoPoseOrigin()
  { return odoPoseOrigin; };

private:
  /**
  * Replay a single step from the logfile.
  * Returns true if EOF not found. */
  virtual bool replayStep();

protected:
  /**
  Update variables in var pool with newest pose */
  void updateVarPool(double tripDist);
  /**
  Set this poseTime value as result into the varPool method call. */
  void setPoseResult(bool found, UPoseTVQ pose1,
                     double * value,
                     UDataBase ** returnStruct, int * returnStructCnt);

public:
  /**
   * Source number for replay */
  static const int SOURCE_REPLAY = -3;
  /**
   * Source number for relay from other server (client interface) */
  static const int SOURCE_OTHER_SERVER = -2;
  
protected:
  /**
  Newest pose */
  UPoseTVQ poseNew;
  /**
  Cyclic buffer of history poses */
  UPoseTVQ poses[MAX_HIST_POSES];
  /**
  Used poses entries */
  int posesCnt;
  /**
  number of newest pose */
  int newest;
  /**
  Client number of newest update */
  int newestSource;
  /**
  Logfile */
  ULogFile logPose;
  /**
  Replay file handle */
  FILE * replayFile;
  /**
  Maximum length of a line in the logfile */
  static const int MAX_LOG_LINE_LENGTH = 200;
  /**
  Buffer for latest line (but not used) line from logfile */
  char replayLine[MAX_LOG_LINE_LENGTH];
  /**
  Is the second timestamp be used, replacing the first in each logline? */
  bool replayTime2;
  /**
  Current line number in logfile */
  int replayLogLine;
  /**
   * Pointer to related odometry pose history */
  UResPoseHist * odoPoseHist;
  /**
   * Position of the odometry pose origin in this coordinate system */
  UPoseTime odoPoseOrigin;
  /**
  Variable for static variables */
  /** index to parameter used by addIfNeeded(..) */
  UVariable * varMinDist;
  /** index to parameter used by addIfNeeded(..) */
  UVariable * varMinTurn;
  /** index to parameter used by addIfNeeded(..) */
  UVariable * varMinTime;
  /** index to trip distance (main or mission) */
  UVariable * varTrip; // = vp->addVar("trip", 0.0);
  /** index to trip distance (e.g some sub-mission trip) */
  UVariable * varTripA; // = vp->addVar("tripA", 0.0);
  /** index to trip distance (e.g. last drive command) */
  UVariable * varTripB; // = vp->addVar("tripB", 0.0);
  /** index to most current pose (all 3 values) */
  UVariable * varPose;
  /** index to most current velocity (m/s) */
  UVariable * varVel; // = vp->addVar("vel", 0.0);
  /** index to most current pose quality */
  UVariable * varQ; // = vp->addVar("vel", 0.0);
  /** index to most current update time (sec since 1 jan 1970) */
  UVariable * varTime; // = vp->addVar("time", 0.0);
  /** index to time since start of mission */
  UVariable * varTripTime;
  /** index to time since start of mission */
  UVariable * varTripTimeA;
  /** index to time since start of mission */
  UVariable * varTripTimeB;
  /** Update average heading the last 5 but 1 meter */
  UVariable * varPoseh5mUse; // = vp->addVar("poseh5m", 0.0);
  /** index to average heading the last 5 but 1 meter */
  UVariable * varPoseh5m; // = vp->addVar("poseh5m", 0.0);
  /** index to a calculated pose X */
  UVariable * varCalcPose; // = vp->addVar("calcX", 0.0, "pose");
  /// index to estimate heading variable */
  UVariable * varEstHead;
  /// index to estimate velocity */
  UVariable * varEstVel;
  /// origin of odometry pose in this coordinate system
  UVariable * varOdoPoseOrigin;
};


#endif
