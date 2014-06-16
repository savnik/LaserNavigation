/** *************************************************************************
*   Copyright (C) 2010 by DTU (Christian Andersen)
*   jca@elektro.dtu.dk
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

#ifndef UAVOIDOBST_H
#define UAVOIDOBST_H

#include <ugen4/u3d.h>
#include <ugen4/uline.h>
#include <urob4/uobstacle.h>

#include "uavoidnovis.h"

class UAvoidLink;
/**
Encapsulation of an obstacle with links that is used to obstacle groups where passage between the
obstacles in the group is not possible. */
class UAvoidObst
{
public:
  /**
  Constructor */
  UAvoidObst()
  {
    empty = NULL;
    clear();
  }
  /**
  Add an obstacle 'other' to the group if within a margin distance or closer to any of the obstacles in the group already.
  If an existing obstacle is embedded in this new obstacle, then the embedded obstacle is added
  to the 'embList', as the group (no-visibility-lines) need to be recalculated.
  If 'embList' is NULL, then nothing is added, and the new obstacle is just added if there is an overlap.
  \param other is the obstacle under test.
  \param   noAdd can be set if obstacle is already added to a group,
  but may be in another group too.
  If the new obstacle is fully embedded in another obstacle,
  then it is not added (but function returnes true).
  \returns true if added. */
  bool addToGroupIfWithinMargin2(UAvoidObst * other, double margin,
                                 bool noAdd);
  /**
  Clear this group */
  void clear()
  {
    noVisCnt = 0;
    obst = NULL;
    grp = NULL;
    links = NULL;
    embedStart = false;
    embedEnd = false;
    isStart = false;
    isExit = false;
  };
  /**
  Get the (up to) 4 tangent lines between these two obstacles */
  // void addtangentLines(UAvoidObst * other, UAvoidLink * ala, UAvoidLink * alb);
  /**
  Assign the outher and cross tangens between these two obstacles.
  index : this   other <br/>
  0   : CV(L)  CV(L)     -- cross tangent <br/>
  1   : CCV(R) CV(L)     -- outher tangent <br/>
  2   : CV(L)  CCV(R)    -- outher tangent <br/>
  3   : CCV(R) CCV(R).   -- cross tangent. <br>
  If obstacles har a slight overlap, then only two (outher - index 1 and 2) tangents are created.
  if obstacles cross each other then 4 outher tangents exist, in this case the additional set of links alc and ald are used to describe one of the sets.
    if second set of tangents (alc and ald) may be NULL.
    \param other is the other obstacle.
    \param ala is describing the up to 4 link from this obstacle to the other
    \param alb is describing the up to 4 link from the other obstacle to this (is a mirror to ala, but has itself a mirror set to NULL).
    \param alc is describing up to 2 outher tangents if the obstacles cross each other (else 0)
    \param ald is describing up to 2 outher tangents if the obstacles cross each other (else 0) - is a mirror to alc.
    \returns true if second set of tangent descriptors are not needed. returns false if second set are needed. Returns also false if second set is needed but is set to NULL.
    */
  bool addtangentLines(UAvoidObst * other, UAvoidLink * ala, UAvoidLink * alb,
                                   UAvoidLink * alc, UAvoidLink * ald);
  /**
  Test established tangent lines for visibility.
  \param logdbg handle to a debug file (NULL if not debugging)
  All link objects to other obstacles are tested against all other obstacles.
  If a tangent line is found to be crossing another obstacle, the line is declared invisible. */
  void testVisibility(FILE * logdbg);
  /**
  Test if a tangent entry at index 'vertexIn' and exit at 'vertexOut' uses
  edges that is marked hidden by other obstacles in the 'noVis' list.
  When the polygon is traversed in CV or CCV direction as indicated by the 'cv' parameter.
  Truerns true if at least one hidden edge is discovered. */
  //bool isTraversedEdgesHidden(int vertexIn, int vertexOut, bool cv,
  //                           bool fromOther, bool toOther);
  /**
  Get tangent points to this obstacle seen from the position 'pos'.
  The function returns the number of tangent points found, i.e. 0 if 'pos' is inside the obstacle
  or 2 if 'pos' is outside (the x,y part of the position and polygon is used only).
  The index to the tangent vertices are returned in 'idxCv' and 'idxCcv'.
  The idxCv is the left tangent when seen from 'pos'. */
  int getTangentVertexFrom(UPosition pos, int * idxCv, int * idxCcv);
  /**
  Test if this tangent has a valid connection that do not
  pass a nogoEdge. A nogoEdge is marked in the nogoEdge array
  during obstacle grouping function.
  If entry and exit vertex is identical, a further test
  is performed to see if the two tangents cross a no-visibility line -
  'p1' is used in this test.
  Returns true if a connection is possible. */
  bool validTangent(bool entryCV, int atVertex, UPosition p1);
  /**
  Has this obstacle a nogo edge between these two vertex numbers?.
  If the vertex numbers are equal, then returnes true.
  If entryCV is used to determine the vertex sequence. */
  bool passingNoGoEdge(bool entryCV, int entryVertex, int exitVertex);
  /**
  Is the line from previous position to entry-vertex and continuing from exit-ertex
  to next position passing nogo lines, i.e. passing through this obstacle group.
  \param entryCV entry is touching this obstacle in a CV direction.
  \param entryVertex is the vertex number where the entry hits the obstacle.
  \param exit vertex is the vertex where the exit tangen touches the obstacle. in between there may be a number of obstacle edges touched on the go.
  \param pPrev is the start position.
  \param pNext is the destination after the obstacle.
  \param isStart is true if the from position is the current robot position.
  \param isExit is true if the next position is the target.
  \returns true if the route passes the internal of the obstacle group. */
  bool passingNoGoEdge2(bool entryCV, int entryVertex, int exitVertex,
                        UPosition pPrev, UPosition pNext,
                        bool isStart, bool isExit);
  /**
  Is this line segment crossing any of the no-visibility lines
  in this obstacle group.
  \param visLine the visibility line to test for crossing of non-visibility lines
  \param logdbg handle to a logfile, if further debug data is to be logged.
  \Returns true if segment is crossing a no-visibility line. */
  bool crossingNonVisibilityLine(ULineSegment * visLine, FILE * logdbg);
  /**
  Get closest point on on polygon to this point - either a vertex or a position on a side.
  \param to is the position to evaluate (x,y is used only)
  \param distLimit a distance closer than this is to be returned only.
  \param closest is where the position on the polygon is returned (if valid within dist limit)
  \param atVertex is set to true if the closest point (within relevant distance) is a vertex.
  \Returns the distance to the polygon if it is within the max relevant distance
  'maxRelevantDist' and the actual closest position in '*closest'.
  \returns a distance above maxRelevantDist when nothing within distLimit and then no change to '*closest'. */
  double getClosestPoint(UPosition to, double distLimit, UPosition * closest, bool * atVertex);
  /**
  Is the other obstacle in the same group as this group.
  Searches from this obstacle onwards in group, i.e. if full group is to be searched, then start with
  first obstacle in group.
  \param other is the other obstacle.
  \returns true if same group. */
  bool isInSameGroup(UAvoidObst * other);

public:
  /**
  Link to obstacle in the group */
  UObstacle * obst;
  /**
  Link to remaing obstacles in group (linked list) */
  UAvoidObst * grp;
  /**
  group number of this obstacle */
  int grpIdx;
  /**
  Maximum number of no-visibility segments */
  static const int NO_VIS_SEG_CNT = 30;
  /**
  List of no visibility lines, that should not be crossed.
  The line is from this COG to the vertex specified by the link. */
  UAvoidNoVis noVis[NO_VIS_SEG_CNT];
  /**
  Number of actually used no visibility line segments */
  int noVisCnt;
  /**
  Pointer to the next obstacle group in the empty (free) list */
  UAvoidObst * empty;
  /**
  Vertex connection list for this obstacle to all other obstacles */
  UAvoidLink * links;
  /**
  List of edges and vertices that are unusable in a path graph.
  Bad edge 3 is between vertex 3 and 4. If vetex count is 4, then the edge is
  between vertex 3 and 0.
  A bad edge is marked with bit 0 (i.e. value is 1 or 3).
  A bad vertex is arked with bit 1 (i.e. value 2 or 3 - value 2 is not used, if vertex is invalid, then edge is too.*/
  int badEdges[UPolygon40::MAX_POINTS];
  /**
  This obstacle embeds the start position */
  bool embedStart;
  /**
  This obstacle embeds the exit position */
  bool embedEnd;
  /**
  This obstacle is the start position */
  bool isStart;
  /**
  This obstacle is the exit position */
  bool isExit;
  /**
  Hidden edges and vertices. A hidden vertex is marked with bit 1 and markes that
  the adjacent edges are not usable in a path either. Bit 2 markes that the edge
  (between this and the next vertex) is hidden (not to pass) only. */
  bool nogoEdge[UPolygon40::MAX_POINTS];
  bool nogoVertex[UPolygon40::MAX_POINTS];
};



#endif // UAVOIDOBST_H
