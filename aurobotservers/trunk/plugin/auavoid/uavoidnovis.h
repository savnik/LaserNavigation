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

#ifndef UAVOIDNOVIS_H
#define UAVOIDNOVIS_H

#include <ugen4/u3d.h>
#include <ugen4/uline.h>
#include <urob4/uobstacle.h>

class UAvoidObst;

/**
Class with line segments that connects obstacles in a group, to avoid visibility between obstacles
that are too close to pass between. These lines atre used in the visibility test for tangent lines. */
class UAvoidNoVis
{
  public:
    /**
    Constructor */
    UAvoidNoVis()
    {
      clear();
    }
    /**
    Clear the no-visibility data set */
    void clear()
    {
      aobThis = NULL;
      aobOther = NULL;
      //firstCnt = 0;
    }
    /**
    Get no-visibility line segment.
    \param idx if 0, then from 1cm from vertex on other obst to COG of this
    \param idx if 1, then from 1cm from vertex /towards this obst) to COG of other obstacle. 
    \returns the specified line segment
    */
    ULineSegment getNoVisSegment(int idx);
    /**
    Get no-visibility line segment.
    From near vertex of other obstaclel (inside or on edge) to COG of this obsatcle.
    \returns the found line segment.
    */
    ULineSegment getNoVisSegment();
    /**
    Get number of non-visibility segment count. */
    /*  int getNoVisSegmentCnt()
    { return firstCnt * 2; };*/
    /**
    Add a set of vertices that is too close to another obstacle. */
    void setNoVisSegment(int firstNear, int lastNear, UAvoidObst * thisObst, UAvoidObst * otherObst);
    /**
    */
    void setNoVisSegment(UPosition posX, UAvoidObst * thisObst, UAvoidObst * otherObst);
    /**
    A tangent entering the obstacle from position
    'p1' following the obstacle edges CV (or CCV) as indicated by 'isCV'. Is the path thorugh the (first) vertex crossing
    the no-visibility lines?
    Returns true if the line is crossed. */
    bool crossingNoVisLine(bool isCV, UPosition p1);
    
  public:
    /**
    The no-visibility is due to nearness to this other obstacle */
    UAvoidObst * aobOther;
    /**
    This obstacle is the one where the index numbers relate to. */
    UAvoidObst * aobThis;
    /**
    Number of points allowed in the near array. */
    // static const int MAX_NEAR_SETS = 3;
    /**
    Vertex span too close to another obstacle to allow passage.
    First is the lowest index number and last the highest. If first is larger than
    last, then the index rolls over. if first and last is the same, then there is only one- */
    //int first; //[MAX_NEAR_SETS];
    //int last; //[MAX_NEAR_SETS];
    UPosition aobPos;
    /**
    Number of valid data sets. */
    //int firstCnt;
};

#endif // UAVOIDNOVIS_H
