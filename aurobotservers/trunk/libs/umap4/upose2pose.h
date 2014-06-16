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
#ifndef UPOSE2POSE_H
#define UPOSE2POSE_H

#include "uposev.h"

/**
Functions to get from one pose to another

@author Christian Andersen
*/
class UPose2pose : public UPoseV
{
public:
  /**
  Constructor */
  UPose2pose();
  /**
  Constructor with initial value */
  UPose2pose(double ix, double iy, double ih, double iv);
  /**
  Destructor */
  ~UPose2pose();
  /**
  Get a navigation solution with 2 arcs, where
  first arc is to the right (and the second to the left).
  The two arcs has the same radius, and is 'h1' to the right
  and 'h2' to the left after the right turn.
  Returns true if solution has found a solution in less
  than 10 iterations. */
  //bool get2arcRight(UPose toPose, double * r1, double * h1, double * h2);
  /**
  Get to this position from pose (0,0,0) in just 2 arcs (right, then left)
  with same turn radius.
  Returns true if valid result (otherwise most likely impossible).
  Returns result radius in 'r1', arc right in 'h1' (radians) and
  arc left in 'h2' (radians).*/
  bool get2arcQ4(double * r1, double * h1, double * h2, FILE * logFp = NULL);
  /**
  Get to this position from pose (0,0,0) in just 2 arcs (left, then right)
  with the same turn radius.
  Returns true if valid result (otherwise most likely impossible).
  Returns result radius in 'r1', first arc in 'h1' (radians) and
  second arc in 'h2' (radians).*/
  bool get22arcLeft(double * r1, double * h1, double * h2, FILE * logFp = NULL);
  /**
  Get to this position from pose (0,0,0) in just 2 arcs (right, then left)
  with the same turn radius.
  Returns true if valid result (otherwise most likely impossible).
  Returns result radius in 'r1', first arc in 'h1' (radians) and
  second arc in 'h2' (radians). */
  bool get22arcRight(double * r1, double * h1, double * h2, FILE * logFp = NULL);
  /**
  Try to find a manoeuver to this pose */
/*  bool get2here(int * manType,
                double * dist,
                double * radius,
                double * arc1,
                double * arc2,
                bool * left1st,
                FILE * logFp = NULL);*/
  /**
  Assign from base pose */
  inline UPose2pose operator= (UPose source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Assign from base pose */
  inline UPose2pose operator= (UPoseV source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a left turn.
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 5 result values are set (if not NULL or
  a solution is not found).
  Returns true if successful. */
  bool get2RightLineLeft(double initVel, double maxAcc, double maxTurnAcc,
                    double * radius1, double * arc1,
                    double * dist,
                    double * radius2, double * arc2,
                    FILE * logFp);
  /**
  Get the needed manoeuvre to get to this position, given this initial
  velocity. The solution consists of 2 arcs and a straight line in-between.
  There may be an initial straight line to break to an allowed speed for the turn. */
  bool get2ViaBreakRightLineLeft(double initVel,
                                double maxAcc, double maxTurnAcc, double minTurnRad,
                                double * initialBreakDist, double * toVel,
                                double * radius1, double * arc1,
                                double * dist,
                                double * radius2, double * arc2,
                                double * finalBreak);
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a right turn.
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The desired end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 5 result values are set (if not NULL or
  a solution is not found).
  Returns true if successful. */
  bool get2RightLineRight(double initVel, double maxAcc, double maxTurnAcc,
                    double * radius1, double * arc1,
                    double * dist,
                    double * radius2, double * arc2,
                    FILE * logFp);
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a right turn.
  An initial break or acceleration for distance 'initialBreakDist',
  may be needed (to turn velocity 'toVel'.
  A final break may be needed too for 'finalBreak' distance.
  The end velocity may be lower than specified! (but not faster)
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The desired end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 8 result values are set (if not NULL or
  no solution found).
  Returns true if successful. */
  bool get2ViaBreakRightLineRight(double initVel,
                    double maxAcc, double maxTurnAcc, double minTurnRad,
                    double * initialBreakDist, double * toVel,
                    double * radius1, double * arc1,
                    double * dist,
                    double * radius2, double * arc2,
                    double * finalBreak);
  /**
  Same as get2RightLineRight(...), just with mirrored values */
  bool get2LeftLineLeft(double initVel, double maxAcc, double maxTurnAcc,
                    double * radius1, double * arc1,
                    double * dist,
                    double * radius2, double * arc2,
                    FILE * logFp);
  /**
  Same as get2ViaBreakRightLineRight(...), just with mirrored values */
  bool get2ViaBreakLeftLineLeft(double initVel,
                    double maxAcc, double maxTurnAcc, double minTurnRad,
                    double * breakDist, double * toVel,
                    double * radius1, double * arc1,
                    double * dist,
                    double * radius2, double * arc2, double * finalBreak);
  /**
  Same as get2RightLineLeft(...), just with mirrored values */
  bool get2LeftLineRight(double initVel, double maxAcc, double maxTurnAcc,
                        double * radius1, double * arc1,
                        double * dist,
                        double * radius2, double * arc2,
                        FILE * logFp);
  bool get2ViaBreakLeftLineRight(double initVel, double maxAcc, double maxTurnAcc, double minTurnRad,
                        double * breakDist, double * toVel,
                        double * radius1, double * arc1,
                        double * dist,
                        double * radius2, double * arc2,
                        double * finalBreak);
  /**
  Get the best solution to destination pose.
  Uses at maximum two arcs and one line inbetween.
  Starts at 0,0 with heading 0 and speed 'initVel', and gets to
  this.x .y .h, aiming for the end velocity this.vel.
  If not possible, then ends in a smaller turnradius then
  needed for a this.vel speed.
  If 'logFp' != NULL, then some calculations may be written in logfile.
  Returns true if a solution is found.
  Fails if initial velocity is too high. */
  bool get2hereALA(int * manType,
                        double initVel, double maxAcc, double maxTurnAcc,
                        double * radius1,
                        double * arc1,
                        double * dist,
                        double * radius2,
                        double * arc2, FILE * logFp);
  /**
  Get the best solution to destination pose.
  Uses at maximum one initial break or accelerate line, two arcs and one line inbetween.
  Starts at 0,0 with heading 0 and speed 'initVel', and gets to
  this.x .y .h, aiming for the end velocity this.vel.
  If not possible, then ends in a smaller turnradius then
  needed for a this.vel speed.
  If 'logFp' != NULL, then some calculations may be written in logfile.
  Returns true if a solution is found.
  Fails if initial velocity is too high. */
  bool get2hereLALA(int * manType,
                                double initVel, double maxAcc, double maxTurnAcc, double minTurnRad,
                                double * breakDist, double * breakToVel,
                                double * radius1,
                                double * arc1,
                                double * dist,
                                double * radius2,
                                double * arc2, double * finalBreak);
  /**
   * Get a solution to get to the line defined by this pose, estimates driveon behavoiur
   * This function covers situations that will start with a left turn.
   * \param turnRad is the assumed turn radius
   * \param turn1 is the initial turn - positive is to the left
   * \param direct is the (potential) straight part
   * \param turn2 is the end turn - positive is left
   * \returns the pose (on the line), where the manoeuvre is (assumed to be) finished.  */
  UPose get2lineStartLeft(double turnRad, double * turn1, double * direct, double * turn2);
  /**
   * Get a solution to get to the line defined by this pose, estimates drive behavoiur
   * of the 'drive' and 'driveon' smrcl commands.
   * The solution is intended for obstacle collision detect, and implemented as one 'driveon' command.
   * The difference to real drive is untested, and will probably works best for
   * an ackermann steered robot, that sets a turning limit not much narrower than the calculated turn radius.
   * Turn radiuswill be calculated as 'pi/2 * gA / gD' (for gA=2 and gD=0.75 turn radius is about 4m)
   * \param turnRad used turn radius
   * \param turn1 is the initial turn - positive is to the left
   * \param direct is the (potential) straight part
   * \param turn2 is the end turn - positive is left
   * \returns the pose (on the line), where the manoeuvre is (assumed to be) finished.  */
  UPose get2line(double turnRad, double * turn1, double * direct, double * turn2);

public:
  /**
  Manoeuvre solution types returned by get2here(...) */
  //typedef enum MAN_SOL_TYP  {MST_2ARC, MST_LINE_ARC, MST_ARC_LINE, MST_LINE, MST_ARC_LINE_ARC, MST_NONE};

};

#endif
