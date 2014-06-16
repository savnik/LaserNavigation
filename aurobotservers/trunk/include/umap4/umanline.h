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
#ifndef UMANLINE_H
#define UMANLINE_H

#include <ugen4/upolygon.h>
#include "umanoeuvre.h"

/**
Straingt line manoeuvre

@author Christian Andersen
*/
class UManLine : public UManoeuvre
{
public:
  /**
  Constructor */
  UManLine();
  /**
  Destructor */
  virtual ~UManLine();
  /**
  Time to complete manoeuver with this start velocity.
  Should be overwritten by a real manoeuvere.
  Return 1e10 if start velocity 'startVel' = 0.0. */
//  virtual double getManTime(double startVel);
  /**
  Get relative pose at end of manoeuvre */
  virtual UPose getEndPose();
  /**
  Get end pose from this start pose */
  virtual UPoseV getEndPoseV(UPoseV startPoseV);
  /**
  Get end pose from this start pose.
  Assumes that the 'atManTime' is within this manoeuvre,
  if not the result is unpredictable (most likely just extended) */
  virtual UPoseV getPoseV(double atManTime, UPoseV startPoseV, double endV);
  /**
  Get distance traveled */
  inline double getDistance()
  { return dist; };
  /**
  Set distance traveled */
  inline void setDistance(double value)
  { dist = value; };
  /**
  Print status for this manoeuvre */
  virtual void fprint(FILE * fd, const char * prestr, double * v = NULL);
  /**
  Print status to string */
  virtual const char * print(const char * prestr, double * v, char * buff, const int buffCnt);
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
  virtual double getDistanceXYSigned(UPosition pos, int * where,
                                     bool posIsRight,
                                    bool centerOnly,
                                    UPose * pHit,
                                    double * atT);
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
  virtual double getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                          UPosition * posOnSeg,
                                          bool posIsRight,
                                          int * whereOnMan,
                                          UPose * poseOnMan);
  /**
  Code this manoeuvre into a SMRCL command string.
  Returns false if command is not codeable. */
  virtual bool getSMRCLcmd(char * buf, int bufCnt);
  /**
  Code this manoeuvre into a SMRCL command string.
  Return in short form, so that no ore than 'lineCntMax' lines are
  coded, and no more than 'distSumMax' are covered.
  Returns new start position for next manoeuvre in 'startPose', and
  as well as new 'lineCnt' and distance travelled in 'distSum'.
  if *first is true, then (distSum + distabce of this manoeuvre) must
  be longet than 'firstDistance'. This is to avoid a target position
  behind the robot. If manoeuvre is used *first is set to false, if not
  then there is no command in 'buf'.
  Returns false if buffer is too short or command is not codeable. */
  virtual bool getSMRCLcmd2(char * buf, int bufCnt,
                                   UPoseV * startPose,
                                   bool * first, double firstDistance,
                                   int * lineCnt, int lineCntMax,
                                   double * distSum, double distSumMax,
                                   UTime * t = NULL, FILE * logprim = NULL);
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
  \param clearence is the minimum clearence around extreme point.
  \param startPose is the start pose of the manoeuvre, indicating the coordinate system used.
  \param allowedErr is the allowed deviation from true polygon in meters (must be > 0.0).
  \returns true if polygon is valid. */
  virtual bool getFoodprintPolygon(UPolygon * polyIncl,
                                   UPolygon * polyExcl,
                                   double leftX, double leftY,
                                   double rightX, double rightY,
                                   double clearence, 
                                   UPose * startPose, double allowedErr = 0.03);

protected:
  /**
  Distance of line */
  double dist;
};

#endif
