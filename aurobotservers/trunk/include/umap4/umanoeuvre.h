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
#ifndef UMANOEUVRE_H
#define UMANOEUVRE_H

#include <ugen4/upolygon.h>
#include <ugen4/uline.h>

#include "uposev.h"

/**
Base classe for a series of basic manoeuvres

@author Christian Andersen
*/
class UManoeuvre
{
public:
  /**
  Constructor */
  UManoeuvre();
  /**
  destructor */
  virtual ~UManoeuvre();
  /**
  Time to complete manoeuver.
  Should be overwritten by a real manoeuvere. */
  virtual double getManTime(double startVel);
  /**
  Get relative pose at end of manoeuvre */
  virtual UPose getEndPose();
  /**
  Get end pose from this start pose */
  virtual UPoseV getEndPoseV(UPoseV startPoseV);
  /**
  Get velocity at end of manoeuvre */
  virtual double getEndV(UPoseV startPoseV);
  /**
  Get end velocity assuming this start velocity */
  virtual double getEndV(double startV);
  /**
  Get end pose from this start pose.
  Assumes that the 'atManTime' is within this manoeuvre,
  if not the result is unpredictable (most likely just extended) */
  virtual UPoseV getPoseV(double atManTime, UPoseV startPoseV, double endV);
  /**
  Get distance traveled */
  virtual double getDistance()
  { return 0.0; };
  /**
  Get acceleration during manoeuvre */
  inline double getAcc()
  { return acc; };
  /**
  Get acceleration during manoeuvre */
  inline void setAcc(double value)
  { acc = value; };
  /**
  Set drive vel at end of manoeuvre */
  inline void setVel(double value)
  { vel = value; };
  /**
  Get drive vel at end of manoeuvre */
  inline double getVel()
  { return vel; };
  /**
  Get manoeuvre type:
  0 = base - i.e. not valid manoeuvre.
  1 = line - straight in current direction
  2 = arc - a turn manoeuvre with constant radius and arc length
  3 = stop - stop all movement */
  inline int getManType()
  { return manType; };
  /**
  Print status info with this lead string */
  virtual void fprint(FILE * fd, const char * prestr, double * v = NULL);
  /**
  Print status info to string with this lead string */
  virtual inline const char * print(const char * prestr, double * v, char * buff, const int buffCnt)
  { return buff; };
  /**
  Get distance to this position from the manoeuvre line
  Positive is to the right of segment - seen from first end
  in direction towards other end if 'posIsRight' is true.
  Returnes distance to either end or to a point on line
  whatever is closest. If 'where' is not NULL, then
  the closest part is returned here as: 0=point on line,
  1= first point, 2= other end.
  Distance to ends are not calculated if 'centerOnly' is true.
  The closest position on route is returned in 'pHit' if
  closest on line. */
  virtual inline double getDistanceXYSigned(UPosition pos,
                                            int * where,
                                            bool posIsRight,
                                           bool centerOnly,
                                           UPose * pHit,
                                           double * atT)
  { return 0.0; }
  /**
  Get shortest distance to this line segment 'seg'.
  If 'posIsRight', then the distance is positive to the right
  of the route.
  The closest point on the segnment is returned in 'posOnSeg'.
  If the closest point on the segment streach, then whereOnSeg = 0, if
  the ref end of segment, then whereOnSeg = 1, other end is 2.
  the closest point in the route is returned in 'poseOnSeg'.
  If the closest point on the manoeuvre is between endpoints, then whereOnMan = 0, if
  the first end of manoeuvre, then whereOnMan = 1, last end is 2. */
  virtual inline double getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                UPosition * posOnSeg,
                                bool posIsRight,
                                int * whereOnMan,
                                UPose * poseOnMan)
  {
    return 0.0;
  }
  /**
  Code this manoeuvre into a SMRCL command string.
  Returns false if buffer is too short or command is not codeable. */
  virtual inline bool getSMRCLcmd(char * buf, int bufCnt)
  { return false; };
  /**
  Code this manoeuvre into a SMRCL command string.
  Return in short form, so that no ore than 'lineCntMax' lines are
  coded, and no more than 'distSumMax' are covered.
  if *first is true, then (distSum + distabce of this manoeuvre) must
  be longet than 'firstDistance'. This is to avoid a target position
  behind the robot. If manoeuvre is used *first is set to false, if not
  then there is no command in 'buf'.
  Returns new start position for next manoeuvre in 'startPose', and
  as well as new 'lineCnt' and distance travelled in 'distSum'.
   * \param t is optional current pose time for logging of manoeuvre primitives
   * \param logprim is the file handle to log to - format time, type (2 or 3) posev, posev
   * \Returns false if buffer is too short or command is not codeable. */
  virtual inline bool getSMRCLcmd2(char * buf, int bufCnt,
                             UPoseV * startPose,
                             bool * first, double firstDistance,
                             int * lineCnt, int lineCntMax,
                             double * distSum, double distSumMax,
                             UTime * t = NULL, FILE * logprim = NULL)
  { return false; };
  /**
  Minimum acceleration transferred to SMR, to avoid
  unwanted situations with no speed and too little acceleration */
  static double inline getMinAcc()
  {  return 0.04; };
  /**
  Get this manoeuvre as a polygin covering the robot foodprint
  during the manoeuvre.
  \param polyIncl is a polygon (convex) with sufficient corners for the manoeuvre.
  \param polyExcl is a polygon (convex) covering the concavity of the
  polyIncl that is not part of the manoeuvre footprint.
  \param leftX is the forward distance of the front-left-most extreme point
  of the robot (positive).
  \param leftY is the x-coordinate of the position of the front-left-most
  extreme point of the robot (should be positive).
  \param rightX is the forward distance of the front-right-most extreme point
  of the robot (positive).
  \param rightY is the y-coordinate of the position of the front-right-most
  extreme point of the robot (should be negative).
  \param clearence is the minimum clearence around extreme points.
  \param startPose is the start pose of the manoeuvre, indicating the coordinate system used.
  \param allowedErr is the allowed deviation from true polygon in meters (must be > 0.0).
  \returns true if polygon is valid. */
  virtual bool getFoodprintPolygon(UPolygon * polyIncl,
                                   UPolygon * polyExcl,
                                   double leftX, double leftY,
                                   double rightX, double rightY,
                                   double clearence,
                                   UPose * startPose, double allowedErr = 0.03)
  {
    return false;
  };

public:
  /**
  Manoeuvre types  */
  enum MAN_TYPE  {MAN_NONE, MAN_LINE, MAN_ARC, MAN_STOP};

protected:
  /**
  Acceleration enforced during manoeuver */
  double acc;
  /**
  End velocity after the manoeuvre.
  negative is reverse. */
  double vel;
  /**
  Manoeuvre type:
  0 = MAN_NONE = base - i.e. not valid manoeuvre.
  1 = MAN_LINE = line - straight in current direction
  2 = MAN_ARC  = arc - a turn manoeuvre with constant radius and arc length
  3 = MAN_STOP = stop all movement */
  MAN_TYPE manType;
};


/**
A manuvre to indicate a full stop.
This may be an emergenct or just a target reached. */
class UManStop : public UManoeuvre
{
public:
  /**
  Constructor */
  UManStop();
  /**
  destructor */
  virtual ~UManStop();
  /**
  Get velocity at end of manoeuvre */
  virtual inline double getEndV(UPoseV startPoseV)
  { return 0.0; };
  /**
  Get end velocity assuming this start velocity */
  virtual inline double getEndV(double startV)
  { return 0.0; };
  /**
  Time to complete manoeuver. This function assumes that the speed is low, and returns an
  arbitrary 5 seconds to complete. */
  virtual double getManTime(double startVel)
  { return 5.0; };
  /**
  Print status info with this lead string */
  virtual const char * print(char * prestr, double * v, char * buff, const int buffCnt);
  /**
  Code this manoeuvre into a SMRCL command string.
  Returns false if buffer is too short or command is not codeable. */
  virtual bool getSMRCLcmd(char * buf, int bufCnt);
  /**
   * get the string for the smr */
  virtual bool  getSMRCLcmd2(char * buf, int bufCnt,
                             UPoseV * startPose,
                             bool * first, double firstDistance,
                             int * lineCnt, int lineCntMax,
                             double * distSum, double distSumMax);

};


#endif
