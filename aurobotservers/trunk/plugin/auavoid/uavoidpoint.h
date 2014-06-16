/** *************************************************************************
*   Copyright (C) 2010 by DTU (Christian Andersen)
*   jca@oersted.dtu.dk
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Library General Public License as       *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU Library General Public     *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
*   Boston, MA 02110-1301, USA.
***************************************************************************/

#ifndef UAVOIDPOINT_H
#define UAVOIDPOINT_H

#include <ugen4/u3d.h>
#include <umap4/upose.h>

class UAvoidObst;

/**
Class to hold one offending points on a route, with safety adius and candidate mid-points */
class UAvoidPoint
{
public:
  /**
  Constructor */
  UAvoidPoint();
  /**
  Destructor */
  ~UAvoidPoint();
  /**
  Clear to empty point including next-prev flag */
  void clear();
  /**
  Insert new point after this point.
  \param newNext is the new point to add
  \returns pointer to just added point.
  */
  UAvoidPoint * insertAfter(UAvoidPoint * newNext);
  /**
     Set obsacle to avoid
     \param ob is pointer to obstacle
     \param idx is the point (vertex) in this obstacle
     \param cvLeft is to be avoided to the left (clockwise). */
  void setAvoidObst(UAvoidObst * ob, int idx , bool cvLeft);
  /**
  Is the passage of this point tight, i.e. a neighboring obstacle puts limits on manoeuvre space.
  */
  inline bool isTight()
  {
    return oob != NULL or useTight;
  };
  /**
  Is the passage of this point away from opposing obstacles that will limit manoeuvres.
  I.e. no close obstacles.
  */
  inline bool isFreeSpace()
  {
    return oob == NULL;
  };
  /**
  Set the angle to the next point in the point sequence */
  void setAngNext();
  /**
  Unlink the next point.
  \returns a pointer to the released point. */
  UAvoidPoint *  unlinkNext();
  /**
    Is the t-point visible from previous position.
    \param knifeOpening is set to true if narrow passage is a vertex at both sides.
        tPos is the same vertex as the aPosition.
    \param opposingFirst is set trur if the opposing obstacle closer to previous point than the current obstacle point.
    \returns true if visible - i.e. facing the surface of the obstacle point, else
             false, i.e. the tPoint is hidden behind the visibility line from previous point. */
  bool tPointVisible(bool * knifeOpening = NULL, bool * opposingFirst = NULL);
  /**
  Set turn centre from already set mid-pose, but use specified direction.
  Sets heading of turn-centre to point towards mid-pose.
  \param turnCentreRadius is distance from mid-point to turn centre.
  \param cvLeft if the direction to turn (true is CV (to avoid an obstacle to the left) and false is CCV) */
  void setTurnCentreFromMidPose(double turnCentreRadius, bool cvLeft);
  /**
  Set turn centre from already set mid-pose.
  Sets heading of turn-centre to point towards mid-pose.
  \param turnCentreRadius is distance from mid-point to turn centre. */
  void setTurnCentreFromMidPose(double turnCentreRadius);

  /**
  Debug print positions */
  void printPoses(const char * preStr, const char * postStr)
  {
    printf("%s aPos(%.2fx,%.2fy), oPos(%.2fx,%.2fy), mid(%.2fx,%.2fy,%.2fh)%s",
           preStr, aPos.x, aPos.y,
           oPos.x, oPos.y,
           mid.x, mid.y, mid.h, postStr);
  };

public:
  /**
  Position to be avoided */
  UPosition aPos;
  /**
  is this a prepoint, ahead of a tight passage. this may lead to
  speciat treatment, e.g. turn centre has a special calculateion
  and is not to be recalculated. */
  bool prePoint;
  /**
  Related obstacle */
  UAvoidObst * aob;
  /**
  Radius of safe distance circle */
//  double radius;
  /**
  Avoid point is to be avoided to the left */
  bool avoidLeft;
  /**
  Closest obstacle (if any) within 2xradius (Other obstacle) */
  UAvoidObst * oob;
  /**
  Distance to closest (other) obstacle */
  double oobDist;
  /**
  Closest position on other obstacle */
  UPosition oPos;
  /**
  Is the point at the other obstacle a vertex */
  bool oVertex;
  /**
  Is the t-point at a vertex (other than the a-point) */
  bool tVertex;
  /**
  The position on this obstacle closest to oPos */
  UPosition tPos;
  /**
  The distance from oPos to tPos */
  double otDist;
  ///  mid poses are
  //static const int MAX_MID_POSES = 1;
  /**
  Midpose(s) evaluated for this avoid point */
  UPose  mid; //[MAX_MID_POSES];
  /**
  Center for manoeuvre circle for this obstacle point */
  UPose mCent;
  /**
  Number of evaluated mid-poses */
  int midCnt;
  /**
  Link to next in sequence */
  UAvoidPoint * next;
  /**
  previous in sequence */
  UAvoidPoint * prev;
  /**
  Angle to next point */
  double angNext;
  /**
  Attempt to use follow line on last pose */
  bool followLineLastPose;
  /**
  In which generation of the creation process was this point created. */
  int generation;
  /**
  Should tight rules be applied (if this is otherwise open space) */
  bool useTight;
  /// serial number
  int serial;
};

#endif // UAVOIDPOINT_H
