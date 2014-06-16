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
#ifndef UMANSEQ_H
#define UMANSEQ_H

#include <ugen4/ulock.h>
#include <ugen4/udatabase.h>

#include "umanoeuvre.h"
#include "umanppseq.h"

/**
A sequence of manoeuvres and related functions

@author Christian Andersen
*/
class UManSeq : public ULock, public UDataBase
{
public:
  /**
  Constructor */
  UManSeq();
  /**
  Destructor */
  ~UManSeq();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "manseq";  };
  /**
  Free all manoeuvre elements (delete) */
  void freeAll();
  /**
  Release all manoeuvres from all pose-to-pose sequences.
  Returns true if is moved to reuse stack.
  If not space in reuse stack, then deleted altogether. */
  void releaseAllMan();
  /**
  Release last of the pose to pose manoeuvres,
  reducing the count of pp manoeuvres by 1. */
  void releaseLast();
  /**
  Release all manoeuvres to the reuse stack
  from thei pose-to-pose sequence */
  void releaseAll(UManPPSeq * ppseq);
  /**
  Truncate this pose-to-pose sequence to this number of manoeuvre primitives
  \param ppseq is the pose to pose sequence belonging to this sequence.
  \param newCnt is the new valid count of manoeuvres - must be >= 0. */
  void truncate(UManPPSeq * ppseq, int newCnt);
  
  /**
  Get manoeuvre count */
  inline int getP2PCnt()
  { return p2pCnt; };
  /**
  Get manoeuvre with this index */
  UManPPSeq * getP2P(int idx);
  /**
  Replace the manoeuvre set including fromIdx and toIdx
  (and any inbetween) with
  two manoeuvre sets using this intermediate pose
  If 'fixate' is true, then the new mid-pose is to be fixated, i.e.
  not to be replaced if route is near an obstacle close by, typically when
  passing a narrow gate.
  If 'newEndpoint' == true, then only one new sequence is generated.
  Returns true if all is OK.
  (okVEl not supported: Returns a suggested initian velocity in 'okVel'
  (>MIN_SPEED=0.2) if manoeuvres start with a emergency break),
  Returns 0.0 if no solution is found. */
  bool replaceMan(UPoseV midPose, bool fixate,
                  double maxAcc, double maxTurnAcc, double minTurnRad,
                  int fromIdx, int toIdx,
                  bool newEndpoint, double * okVel);
  /**
   * Replace some manoeuvres with a new new manoeuvres, that passes a new mid-point.
   * \param midPose is the new midPose.
   * \param fixate if true, the poses will not be moved??
   * \param turnRad is the turn radius to use
   * \param fromIdx is the first manoeuvre index to be replaced.
   * \param toIdx is the last index to be replaced.
   * \param newEndPoint if true, then the new manoeuvre should end at the midPose,
   * replacing the old manoeuvres.
   * \returns true if the last of the inserted manoeuvres end at the desired end pose.
   * returns false if the end pose line is reached later than desired. */
  bool replaceManDriveon(UPoseV midPose, bool fixate,
                          double turnRad,
                          int fromIdx, int toIdx,
                          bool newEndpoint);
  /**
  Adds a manoeuvre at the end of the current sequence.
  The from pose should be the same as the toPose of the last
  manPPSeq. The needed commands to get there are calculated and added.
  Returns false if a manoeuvre solution is not found.
  (okVel is no longer supported) */
  bool addMan(UPoseV fromPose, UPoseV toPose,
              double maxAcc, double maxTurnAcc, double minTurnRad,
              double * okVel = NULL);
  /**
   * add manoeuvre using an estimate of the drive or driveon command, using
   * the angle and cross distance gains as giudeline for turn radius.
   * \param fromPose is current pose.
   * \param toPose is desired destination pose (line)
   * \param turnRad is the turn radius to use.
   * \returns true if the desired pose is reached and false if the line
   * is reached after the desired end pose (the manoeuvre is then the best efford to reach the end pose line as fast as possible. */
  bool addManDriveon(UPoseV fromPose, UPoseV toPose, double turnRad);
  /**
  Make a new manoeuvre from these data.
  A straight line with this acceleration.
  Distance is traveled distance in current direction.
  Acceleration is signed, positive is increased speed forward.
  Returns a pointer to the created object. */
  UManoeuvre * getNewLine(double dist, double acceleration, double targetVel);
  /**
  Make a new manoeuvre from these data.
  An arc with this acceleration.
  Radius is turn radius in meter, sign is always positive.
  Angle is turn angle in radians, positive is left (counter-clockwise).
  Angle may be more than PI.
  Acceleration is signed, positive is increased speed forward.
  Returns a pointer to the created object. */
  UManoeuvre * getNewArc(double radius, double angle,
                         bool left, double acceleration,
                         double targetVel);
  /**
  Get a new arc amnoeuvre, where left or right is determined
  by angle sign. Else same as other function by same name. */
  inline UManoeuvre * getNewArc(double radius, double angle,
                         double acceleration,
                         double targetVel)
  { return getNewArc(radius, fabs(angle), angle >= 0.0, acceleration, targetVel); };
  /**
  Make a new stop manoeuvre. should set wheel speed to 0,0 at this
  point - may be an emergency stop, or just target reached.
  Returns a pointer to the created object. */
  UManoeuvre * getNewStop();
  /**
  Calculate needed acceleration to get from speed to an end speed
  using only this distance.
  uses the formular V� - V_0� = 2 a s */
  inline double getNeededAcc(double startVel, double endVel, double distance)
  { return (sqr(endVel) - sqr(startVel))/(2.0 * distance); };
  /**
  Calculate needed distance to accelerate from speed to an end speed
  using this accelertion.
  uses the formular V� - V_0� = 2 a s */
  inline double getNeededDist(double startVel, double endVel, double acc)
  { return (sqr(endVel) - sqr(startVel))/(2.0 * acc); };
  /**
  Print status for manoeuvre sequence */
  void fprint(FILE * fd, const char * prestr);
  /**
  Print manoeuvre status to string */
  const char * print(const char * prestr, char * buff, const int buffCnt);
  /**
  Get available objects in reuse stack.
  That is count of non-null pointers up to reuseCnt. */
  int reusableCnt();
  /**
  Get the end pose as notet in manoeuver sequence. */
  UPoseV getEndPoseV();
  /**
  Get the start pose as stored in this manoeuvre sequence. */
  UPoseV getStartPoseV();
  /**
  Get distance to this position from the manoeuvre lines.
  Positive is to the right of segment - seen from first end
  in direction towards other end (if 'posIsRight' is true).
  Returnes distance to either end or to a point on line
  whatever is closest. If 'where' is not NULL, then
  the closest part is returned here as: 0=point on line,
  1= first point, 2= other end, 3 = too far away ( > maxDist).
  If 'where' is here (== 0), then 'pHit' is set to the
  closest position to the path.
  'idx' is set to the manouvre sequence with the hit, else -1
  if closest point is beyond last point.
  't' is the distance into that sequence.
  If where != 0, then pHit will be unchanged */
  double getDistanceXYSigned(UPosition pos, int * where,
                             bool posIsRight,
                             double maxDist,
                             UPose * pHit,
                             int * idx, double * t);
  /**
  Get minimum distance to this line segment from the manoeuvre lines.
  Positive is to the right of segment - seen from first end
  in direction towards other end (if 'posIsRight' is true).
  Returnes distance to either end or to a point on line
  whatever is closest. If 'whereOnMan' is not NULL, then
  the closest part is returned here as: 0=point on line,
  1= first point, 2= other end, 3 = too far away ( > maxDist).
  If 'whereOnSeg' is not NULL, then
  the closest part of segment is returned here as: 0=point on line,
  1= first point, 2= other end.
  if 'poseOnMan' is not NULL then the closest pose is returned.
  if 'posOnSeg' is not NULL then closest position on segment 'seg' is returned.
  'idx' is set to the manouvre sequence with the hit, else -1
  if closest point is beyond last point. */
  double getMinDistanceXYSigned(ULineSegment * seg,
                                         int * whereOnSeg,
                                         UPosition * posOnSeg,
                                         bool posIsRight,
                                         double maxDist,
                                         int * whereOnMan,
                                         UPose * poseOnMan,
                                         int * idx);
  /**
  Get total route distance */
  double getDistance();
  /**
  Get deviation from the direct line to target */
  double getDeviationFromDirect();
  /**
  Get time to complete manoeuvre to end */
  double getTime();
  /**
  Copy the source manouevre sequence to this.
  Returns true. */
  bool copy(UManSeq * source);
  /**
  Expand a pose to pose manoeuvre to a sequence of simple manoeuvres.
  Returns false if not successful (no path found, i.e. 2cm backwards.
  REturns initial velocity in 'okVel' if all is OK.
  Returns a suggestec speed in 'okVel' (> MIN_SPEED = 0.2) if
  initial velocity is too fast.
  The 'maxAcc' is the normal lateral acceleration.
  The end speed is assumed to be a maximum, that will
  be reduced if the turn radius and maxTurnAcc requires
  lower speed.
  The 'maxTurnAcc' is the centrepetal acceleration allowed.
  This function sets the from-to pose, calls the expandAddMan(...) and
  set final velocity, distance and time used. */
  bool expandMan(UPoseV fromPose, UPoseV toPose, bool last,
                 double maxAcc, double maxTurnAcc, double minTurnRad,
                 UManPPSeq * ppSeq,
                 double * okVel);
  /**
   * Expand to an estimated begaviour for a driveon command, separating the behaviour
   * into a turn an optional straight part andother turn and potentially
   * a straight path to the end pose.
   * \param fromPose is the current pose and velocity.
   * \param toPose is the desired target pose (line) and velocity, used in the driveon command
   * \param turnRad is the turn radius to use
   * this givess with gA of 2.0 and gD 0.75 a turnradius og ~4m.
   * \param ppSeq is the manoeuvre sequence, where the manoeuvres are to be added.
   * \returns true, if the end pose can be reached with the manoeuvre, else the toPose
   * should be reevaluated from the manoeuvres.*/
  bool expandManDriveon(UPoseV fromPose, UPoseV toPose,
                                 double turnRad,
                                 UManPPSeq * ppSeq);
  /**
  Add a new (empty) UManPPSeq as a next pose-to-pose sequence.
  Intended if already calculated manoeuvres are transfreerd to this
  structure. */
  UManPPSeq * addP2P();
  /**
  Get the maximum turn angle in this sequence of manoeuvres.
  Returns 0.0 if no arcs is found.*/
  double getMaxTurnArc();
  /**
  Is manoeuvre sequence valid, i.e. atr there any movement defined */
  bool isValid();
  /**
  Get pose at this time. Time is in seconds into the manoeuvre from its start pose. */
  UPoseV getPoseV(double atManTime);
  /**
   * REmove last arc (probably unusable due to too small turn radius) */
  void removeLastArc();
  /**
   * \brief Calculate the distance from the end-pose line to the point on the manoeuvre furthest away from the line.
   * Only the start/end points of each manoeuvere is used, so if one of
   * the pose to pose manoeuvres has an almost full circle, then the circle is not counted.
   * \param maxHeading is - if not NULL - set to the most different heading,
   * relative to the end pose.
   * a positive angle is to the left relative to the end pose heading.
   * \returns the distance signed, where positive is to the left seen from end-pose pose. */
  double getDistanceFromEndPoseLine(double * maxHeading);

protected:
  /**
  Expand a pose to pose manoeuvre to a sequence of simple manoeuvres
  in 'ppSeq', that can be translated into drive commands.
  Returns true if successful.
  The 'maxAcc' is the normal lateral acceleration.
  The end speed is assumed to be a maximum, that will
  be reduced if the turn radius and maxTurnAcc requires
  lower speed.
  The 'maxTurnAcc' is the centrepetal acceleration allowed.
  This function calls itself recursively if needed. */
/*  bool expandAddMan(UPoseV fromPose, UPoseV toPose,
                 double maxAcc, double maxTurnAcc,
                 UManPPSeq * ppSeq);*/
  /**
  Expand a pose to pose manoeuvre to a sequence of simple manoeuvres
  in 'ppSeq', that can be translated into drive commands.
  Returns true if successful.
  The 'maxAcc' is the normal lateral acceleration.
  The end speed is assumed to be a maximum, that will
  be reduced if the turn radius and maxTurnAcc requires
  lower speed.
  The 'maxTurnAcc' is the centrepetal acceleration allowed.
  This function calls itself recursively if needed. */
  bool expandAddManALA(UManPPSeq * ppSeq, bool last,
                 double maxAcc, double maxTurnAcc,
                 double * okVel);
  /**
  Expand a pose to pose manoeuvre to a sequence of simple manoeuvres
  in 'ppSeq', that can be translated into drive commands.
  Returns true if successful.
  The 'maxAcc' is the normal lateral acceleration.
  The end speed is assumed to be a maximum, that will
  be reduced if the turn radius and maxTurnAcc requires
  lower speed.
  The 'maxTurnAcc' is the centrepetal acceleration allowed.
  This function calls itself recursively if needed. */
  bool expandAddManLALA(UManPPSeq * ppSeq, bool last,
                        double maxAcc, double maxTurnAcc, double minTurnRad);
                                 //double * okVel);
  /**
  Release a manoeuvre object for reuse */
  void release(UManoeuvre * toRelease);
  /**
  Request object, either from reuse stack, or freshly made
  from data heap. */
  UManoeuvre * requestObj(int manType);

protected:
  static const int MAX_PP_MAN_CNT = 100;
  /**
  Manoeuver sequence based on UManoeuvre */
  UManPPSeq * p2p[MAX_PP_MAN_CNT];
  /**
  Number of manoeuvres in sequence */
  int p2pCnt;

private:
  static const int MAX_REUSABLE_LINE_OBJ = 100;
  /**
  When an object is released, it is added to this list.
  When new objects are needed this list is searced first.
  Array of manoeuvres for reuse */
  UManoeuvre * reuse[MAX_REUSABLE_LINE_OBJ];
  /**
  Number of released pointers */
  int reuseCnt;

};

#endif
