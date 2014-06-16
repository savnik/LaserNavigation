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

#ifndef UAVOIDLINK_H
#define UAVOIDLINK_H

#include <ugen4/u3d.h>
#include <ugen4/uline.h>
// #include <urob4/uobstacle.h>

class UAvoidObst;
class UAvoidLinkSeq;


/**
An AvoidLine is describes tangent lines between two obstacles.
in total 4 tangent lines can exist between any two obstacles.
An array with 4 entries describe the valid tangent lines, and the
connection details. The lines atr always orderd as follows:
index : this   other <br/>
0   : CV(L)  CV(L)     -- cross tangent <br/>
1   : CCV(R) CV(L)     -- outher tangent <br/>
2   : CV(L)  CCV(R)    -- outher tangent <br/>
3   : CCV(R) CCV(R).   -- cross tangent.
<br/>
this  +--\--------2--------/--+   other<br/>
+++  \               /  +++       <br/>
+++++   --3---------0     ++++     <br/>
++++       /     \       ++++++   <br/>
+-------/----1---\ ------++     <br/>
I.e.: index 1 is CCV when tangent is fixed on 'this' obstacle and hits 'other' when turned CCV.
Or when fixed on 'other' obstacle hits 'this' obstacle when turned CV.
I.e.2: index 0 and 1 follows 'this' obstacle in a CCV direction,
and index 0 and 2 follows the 'other' obstacle in a CV direction.

Each tangent line is associated with an obstacle. The same line is thus defined
for both obstacles. One of the lines has a link to the other (mirror) line.
  */
class UAvoidLink
{
  public:
    /**
    Constructor */
    UAvoidLink()
    {
      clear();
      empty = NULL;
    };
    
  public:
    /**
    Clear to an empty state - except for the empty chain pointer */
    void clear()
    {
      int i;
      aob = NULL;
      tangentCnt = 0;
      serial = 0;
      for (i = 0; i < 4; i++)
      {
        valid[i] = false;
        idx[i] = -1;
        aobIdx[i] = -1;
        lnkSeq[i] = NULL;
      }
      mirror = NULL;
    }
    /**
    Test visibility of the lines in this link against the obstacle in the other link.
    This is done by testing the cross point between each line against a line
    across the other obstacle (between the tangent points). */
    void visibilityTest(UAvoidLink * other);
    /**
    Get position at the this end of the tangent line with this index number (k).
    If the tangent line is not valid, then en empty line segment is returned. */
    UPosition getThisEnd(int k);
    /**
    Get position at the other end of the tangent line with this index number (k).
    If the tangent line is not valid, then en empty line segment is returned. */
    UPosition getOtherEnd(int k);
    /**
    Get the tangent line defined by this index 'idx' number.
    Returns an empty line (length = -1) if tangent line or index is not valid */
    ULineSegment getTangentLine(int idx);
    /**
    A tangent index number for this obstacle can also be seen from the other obstacle, but with a different index number. This function returns that index number.
    The index number is the same for index 0 and 3 and 1 and 2 are reversed.
    Returns the converted index number. */
    int getOtherEndIdx(int k);
    /**
    when outher tangens are valid, the inner part of vetices (and edges) are tested
    for proximity to other obstacle (within 'margin'), if within margin
      those vertices and adjecent edge atr marked as nogo (for path tangents). */
    void setBadEdgeAndVertex(double margin);
    
  private:
    /**
    \returns true if x is between these two limits within a margin of 1e-4.
    \param x is the test value.
    \param lim1 is the first limit (may be smaller or larget than lim2.
    \param lim1 is the second limit (may be smaller or larget than lim1.
    \returns true if at least an epsilon of 0.0001 within these limits. */
    bool isWithin(double x, double lim1, double lim2);
    
  public:
    /**
    This obstacle */
    UAvoidObst * tob;
    /**
    the other obstacle */
    UAvoidObst * aob;
    /**
    Index to position in other obstacle polygon.
    The 4 elements are tangents in the direction from this and the other:
    index : this   other <br/>
    0   : CV(L)  CV(L)     -- cross tangent <br/>
    1   : CCV(R) CV(L)     -- outher tangent <br/>
    2   : CV(L)  CCV(R)    -- outher tangent <br/>
    3   : CCV(R) CCV(R).   -- cross tangent. */
    int aobIdx[4];
    /**
    Index to a vertex in this obstacle */
    int idx[4];
    /**
    Is connection valid (otherwise it is assumed to be obstructed. */
    bool valid[4];
    /**
    Link to link sequence that that this link is part of */
    UAvoidLinkSeq * lnkSeq[4];
    /**
    Number of tangent lines connecting this object with the other.
    It sould be 4 for normal objects, but will be 2 for point obstacles
    and for start and end points. */
    int tangentCnt;
    /**
    Link to next in free chain */
    UAvoidLink * empty;
    /**
    Link to next obstacl link description */
    UAvoidLink * next;
    /**
    Mirror partner seen from other obstacle.
    This is NULL for the "original" and a pointer for the "mirrir",
    The tangent lines are the same except index 1 and 2 that is mirrored. */
    UAvoidLink * mirror;
    /**
    Debug serial number for created links */
    int serial;
};


//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


/**
Class to hold the references to the tangent lines that in sequence
forms a possible path from current position to destination position.*/
class UAvoidLnkSeq
{ // UAvoidLnkSeq
public:
  /**
  Constructor */
  UAvoidLnkSeq()
  {
    clear();
  };
  /**
  Clear any data in the object (except recycle link).  */
  inline void clear()
  {
    costDist = 0.0;
    costAngle = 0.0;
    costF = 0.0;
    costG = 0.0;
    costH = 0.0;
    tangLine = NULL;
    next = NULL;
    prev = NULL;
    tangIdx = -1;
    serial = -1;
  }
  /**
  Is this line segment crossing any of the tangent lines in this sequence.
  Returns true if a crossing is detected */
  bool pathCrossing(ULineSegment seg);
  /**
  Get the distance and angle sum for this tangent sequence */
  double getDistance(double entryAngle,
                     double exitAngle,
                     double * turnAngleSum);
  /**
    Get the position of this element in the tangent sequence */
  UPosition getThisEnd()
  {
    return tangLine->getThisEnd(tangIdx);
  };
  /**
  Get the position of this element in the tangent sequence */
  UPosition getOtherEnd()
  {
    return tangLine->getOtherEnd(tangIdx);
  };

public:
  /**
  Link to the next tangent line in a candidate path */
  UAvoidLnkSeq * next;
  /**
  Link to the previous tangent line in a candidate path */
  UAvoidLnkSeq * prev;
  /**
  Link to the tangent line */
  UAvoidLink * tangLine;
  /**
  Index to the tangent line used (out of maximum 4 tangent lines). */
  int tangIdx;
  /**
  Cost of this link in terms of distance */
  double costDist;
  /**
  Cost of this link in terms of angle */
  double costAngle;
  /**
  Minimum cost from this point to the end pose. */
  double costF;
  /// cost from this point to destination
  double costH;
  /// tentative score so far (from start pose to this point)
  double costG;
  /// debug serial number
  int serial;
};




#endif // UAVOIDLINK_H
