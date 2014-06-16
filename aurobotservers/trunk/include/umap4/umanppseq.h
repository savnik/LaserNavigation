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
#ifndef UMANPPSEQ_H
#define UMANPPSEQ_H

#include "umanoeuvre.h"


/**
Holds the manoeuvres needed to get from one pose to another.
The start and end pose and velocity is stored too, to enable
revalculation of manoeuvre if start-speed is
changed.

@author Christian Andersen
*/
class UManPPSeq
{
public:
  /**
  Constructor */
  UManPPSeq();
  /**
  Destructor */
  ~UManPPSeq();
  /**
  Add a manoeuvre at end of sequence.
  Returns true if added. */
  bool add(UManoeuvre * man);
  /**
  Insert manoeuvre at this point in sequence
  Returns true if inserted. if point is larger
  than current size, it is added at end.
  if (point is 0 or less the manoeuvre is added at beginning.
  Returns true if inserted (enough space). */
  bool insert(UManoeuvre * man, int point);
  /**
  Extract a manoeuvre from sequence, moving the rest down as needed.
  Returns NULL if index is not inside valid range.
  Returns a pointer to the removed manoeuvre. */
  UManoeuvre * extract(int point);
  /**
  Get a pointer to a manoeuvre at this point */
  UManoeuvre * getMan(int point);
  /**
  Get maximum number of manoeuvre pointers allowed */
  inline int getMaxManCnt()
  { return MAX_MAN_CNT; };
  /**
  Get number of manoeuvres in sequence */
  inline int getSeqCnt()
  { return seqCnt; };
  /**
  Get start pose (poseV) */
  inline UPoseV getStartPoseV()
  { return startPose; };
  /**
  Set start pose (poseV) */
  inline void setStartPoseV(UPoseV poseV)
  { startPose = poseV; };
  /**
  Get end pose (poseV) */
  inline UPoseV getEndPoseV()
  { return endPose; };
  /**
  Get end position (z is set to 0.0) */
  inline UPosition getEndPos()
  { return endPose.getPos(0.0); };
  /**
  Get start position (z is set to 0.0) */
  inline UPosition getStartPos()
  { return startPose.getPos(0.0); };
  /**
  Set end pose (poseV) */
  inline void setEndPoseV(UPoseV poseV)
  { endPose = poseV; };
  /**
  Get acheived end velocity. Positive is forward. [m/s]. */
  inline double getEndVel()
  { return endVel; };
  /**
  Set acheived end velocity. Positive is forward. [m/s]. */
  inline void setEndVel(double vel)
  { endVel = vel; };
  /**
  Get poseV at time, using specified start pose */
  UPoseV getPoseV(double atManTime, UPoseV startPose);
  /**
  Get pose at end of this sequence.
  \param mn is the index of the manoeuvre of which the end pose is returned.
  \returns end pose of the indicated manoeuvre. */
  UPoseV getPoseV(int mn);
  /**
  Get poseV at time, using storet start pose */
  UPoseV getPoseV(double atManTime)
  { return getPoseV(atManTime, startPose); };
  /**
  Calculate end velocity, and manoeuvre time as well if
  andManTime != NULL. */
  double calcEndVel(double * andManTime, double * manDist);
  /**
  Calculate end pose using by predicting from manoeuvre sequence. */
  UPoseV calcEndPose(UPoseV startPose);
  /**
  Print status for manoeuvre section */
  void fprint(FILE * fd, const char * prestring);
  /**
  Print status to buffer string */
  void print(const char * prestring, char * buff, const int buffCnt);
  /**
  Get time spend in manoeuvre. */
  inline double getManTime()
  { return manTime; };
  /**
  Set time to complete manoeuvre. */
  inline void setManTime(double value)
  { manTime = value; };
  /**
  Get distance to this position from the manoeuvre lines
  \param pos is the posittion to be tested for distance.
  \param where is the closest position: 0=point on line,
  1= first point, 2= other end (end point).
  \param posIsRight if true, then positive is to the right of segment - seen from first end in direction towards other end.
  \param pHit is the pose on the manoeuvre sequence closest to the position
  If where is 0, then 'pHit' is set to the closest position to the path.
  If where != 0, then pHit will be unchanged.
  \param atT is set to the distance into full manoeuvre sequence - in m along the curve.
  \param atMan is the index to the hit manoeuvre (set if not NULL only)
  \returns distance to either end or to a point on line
  whatever is closest.   */
  double getDistanceXYSigned(UPosition pos, int * where,
                            bool posIsRight,
                            UPose * pHit, double * atT,
                            int * atMan = NULL);
  /**
  Get shortest distance to this manoeuvre route to 'seg'.
  If 'posIsRight', then the distance is positive to the right
  of the route.
  The closest point on the segnment is returned in 'posOnSeg'.
  If the closest point on the segment streach, then whereOnSeg = 0, if
  the ref end of segment, then whereOnSeg = 1, other end is 2.
  the closest point in the route is returned in 'poseOnSeg'.
  If the closest point on the manoeuvre is between endpoints, then whereOnMan = 0, if
  the first end of manoeuvre, then whereOnMan = 1, last end is 2. */
  double getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                        UPosition * posOnSeg,
                                        bool posIsRight,
                                        int * whereOnMan,
                                        UPose * poseOnMan);
  /**
  Set total manoeuvre distance */
  inline void setManDist(double value)
  { manDistance = value; };
  /**
  Set total manoeuvre distance */
  inline double getManDist()
  { return manDistance; };
  /**
  Get distance to end pose from this position */
  inline double getEndDistTo(UPosition pos)
  { return hypot(endPose.x - pos.x, endPose.y - pos.y);};
  /**
  Get distance to end pose from this position */
  inline double getStartDistTo(UPosition pos)
  { return hypot(startPose.x - pos.x, startPose.y - pos.y);};
  /**
  Get the maximum turn angle in this sequence of manoeuvres.
  Returns 0.0 if sequence has no arcs. */
  double getMaxTurnArc();
  /**
  Is this pose-to-pose sequence valid, i.e. are there ant defined manoeuvres. */
  inline bool isValid()
  { return (seqCnt > 0);};
  /**
  Is first pose fixed - e.g. passing a narrow passage */
  inline bool isFirstFixed()
  { return fixedFirstPose; };
  /**
  Is last pose fixed - e.g. passing a narrow passage */
  inline bool isLastFixed()
  { return fixedLastPose; };
  /**
  Set first pose as fixed, i.e. should not be changed by
  manoeuvre planning, even if a new waypoint is close by. */
  inline void setFirstFixed(bool value)
  { fixedFirstPose = value; };
  /**
  Set last pose as fixed, i.e. should not be changed by
  manoeuvre planning, even if a new waypoint is close by. */
  inline void setLastFixed(bool value)
  { fixedLastPose = value; };
  /**
   * Truncate this manoeuvre and the rest - the structures themself
   * must be removed by the caller.
   * \param i index to the first structure to remove */
  void truncate(int i);
  /**
   * workaround to limit max velocity
   * returns true if all velocities are less than 'maxVel' */
  bool setMaxVel(double maxVel);
  /**
  Truncate manoeucre at a distance before a given position.
  The closest position on the path is found, and a preDistance before that the path is truncated.
  It is used to stop a planned route, if an obstacle can not be avoided by other means.
  \param pos is the position to stop before.
  \param preDist is an additional position to stop before the path meets the possition 'pos'.*/
  void truncate(UPosition pos, double preDist);
  
protected:
  /**
  Start pose and (actual) start velocity */
  UPoseV startPose;
  /**
  End pose as desired (velocity may be different) */
  UPoseV endPose;
  /**
  Acheived end velocity (m/sec) */
  double endVel;
  /**
  Manoeuvre distance total. */
  double manDistance;
  /**
  Time to complete manoeuvre (sec) */
  double manTime;
  /**
  Maximum number of elemental sequences for manoeuvre */
  static const int MAX_MAN_CNT = 20;
  /**
  Sequence of simple manoeuvres to get from start pose
  to end pose. */
  UManoeuvre * seq[MAX_MAN_CNT];
  /**
  Number of manoeuvres in sequence */
  int seqCnt;
  /**
  Is the start pose fixed (e.g. passing a narrow segment) */
  bool fixedFirstPose;
  /**
  Is the end pose fixed (e.g. in a narrow segment) */
  bool fixedLastPose;
};

#endif
