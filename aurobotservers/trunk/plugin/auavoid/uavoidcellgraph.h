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

#ifndef UAVOIDCELLGRAPH_H
#define UAVOIDCELLGRAPH_H

#include <ugen4/u2dline.h>
#include <ugen4/upolygon.h>
#include <urob4/ulogfile.h>


class UAvoidObst;
class UAvoidParams;
//class UAvoidVertexCost;

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

/**
Structure holding a bad position (on the edge or vertex of an obstacle).
The additional cost to pass this point by the A* algorithm, and the
side the path will pass the obstacle (the other way is not affected) */
class UAvoidVertexCost
{
public:
  /**
  Print value to console */
  void print(const char * pre)
  {
    printf("%s at %.3fx,%.3fy left=%s cost=%.2f\n", pre, pos.x, pos.y, bool2str(avoidLeft), cost);
  };
  /** position of vetrex with extra cost - a punished vertex position */
  U2Dpos pos;
  /**
  is this point a point to avoid on the left side (in drive direction) */
  bool avoidLeft;
  /** extra cost value */
  double cost;
};

/**
Class to hold positions experienced to result in bad or impossible manoeuvres. */
class UAvoidVertexCosts
{
public:
  /// make default number of entries
  UAvoidVertexCosts()
  {
    costsCntMax = 0;
    costsCnt = 0;
    costs = NULL;
    resize(5);
    PosLimit = 0.01;
  };
  /// destructor
  ~UAvoidVertexCosts()
  {
    free(costs);
  };
  /// resize array to this number of entries
  void resize(int newCnt)
  {
    costs = (UAvoidVertexCost *) realloc(costs, newCnt * sizeof(UAvoidVertexCost));
    costsCntMax = newCnt;
  }
  /** get cost of this position
  \param position to test
  \param isLeft is the side to pass point.
  \returns additional cost value */
  double getExtraCost(U2Dpos position, bool isLeft)
  {
    double extra = 0.0;
    UAvoidVertexCost * ec = costs;
    //
    for (int i = 0; i < costsCnt; i++)
    {
      if (ec->avoidLeft == isLeft)
      {
        if (ec->pos.dist(position) < PosLimit)
        {
          extra = ec->cost;
          break;
        }
      }
      ec++;
    }
    return extra;
  };
  /** set cost of this position
  \param position to test
  \param isLeft is the side to pass point.
  \param extraCost is the extra cost to put on this point. */
  void setExtraCost(UPosition position, bool isLeft, double extraCost)
  {
    UAvoidVertexCost * ec = costs;
    int i;
    for (i = 0; i < costsCnt; i++)
    {
      if (ec->avoidLeft == isLeft)
      {
        if (ec->pos.dist(position) < PosLimit)
        {
          ec->cost += extraCost;
          ec->print("setExtraCost:add");
          break;
        }
      }
      ec++;
    }
    if (i == costsCnt)
    { // extra point is needed
      if (costsCnt >= costsCntMax)
        resize(costsCntMax + 10);
      ec->avoidLeft = isLeft;
      ec->pos = position;
      ec->cost = extraCost;
      ec->print("setExtraCost:new");
      costsCnt++;
    }
  };
  void list()
  {
    printf(" - Extra cost at %d vertex positions\n", costsCnt);
    for (int c = 0; c < costsCnt; c++)
      printf("  -- cost %8.2f at %7.3fx,%7.3fy (left %s)\n",
             costs[c].cost, costs[c].pos.x, costs[c].pos.y, bool2str(costs[c].avoidLeft));
  };
public:
 /// additional costs entries
 UAvoidVertexCost * costs;
 /// number of used values
 int costsCnt;
 /// max number of entries in array
 int costsCntMax;
 /// position tolerance limit when comparing cost positions
 double PosLimit; // = 0.01;
};

/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

/**
Path result  entry, with all vertices in the
path from start to end pose
It holds the vertex points and the reference to obstcles as
well as position and vertex number on the obstace, and
if it is to be avoided left (to the left of the obstacle) or right. */
class UAvoidCellVertex
{
public:
  /**
  Constructor */
  UAvoidCellVertex()
  {
    obst = NULL;
    finished = false;
    avoidLeft = false;
    finished= false;
    nextIdx = -1;
  };
  /**
  Is the next direction forward to a more positive x-value */
  inline bool isFwd()
  { return nextIdx > 1; };
  /**
  Vertex position */
  UPosition pos;
  /**
  Hit obstacle - NULL if no obstacle is hit */
  UAvoidObst * obst;
  /**
  Should it be avoided left or right */
  bool avoidLeft;
  /**
  Opening segment when leaving the cell.
  This is not relevant for the last cell. */
  U2Dseg exitSeg;
  /**
  Is the cell calculation finished */
  bool finished;
  /**
  The index to the next cell, to indicate the direction.
  0 is back (negative x direction) most positive Y value
  1 is back (negative x direction) to most negative Y value
  2 is forward (positive x direction) to most positive Y value
  3 is forward (positive x direction) to most negative Y value
  -1 if no next cell.*/
  int nextIdx;
};

class UAvoidVertexIdx;
class UAvoidCell;

/**
Class for path planning using an exact cell decomposition method. */
class UAvoidCellGraph
{
public:
  /**
  Constructor */
  UAvoidCellGraph();
  /**
  Destructor */
  ~UAvoidCellGraph();
  /**
  Convert obstacles into a cell map using an absolute cell decomposition method
  \param aogs is the obstacles to avoid
  \param aogsCnt is the number of obstacle groups.
  \param par is the obstacle avoidance parameters to use.
  \param debugDump saves a lot of values to file if true. */
  void makeCellDecomposition(UAvoidObst* aogs[], int aogsCnt, UAvoidParams* par, bool debugDump = false);
  /**
  Find best path in the constructed cells (in pacs array).
  \param par is the obstacle avoidance parameters to use.
  \param costAdd additional cost to selected points
  \param debugDump saves a lot of values to file if true.
  \returns number of vertecies to e avoided. */
  int findBestCellPath(UAvoidParams* par, UAvoidVertexCosts * costAdd, bool debugDump);
  /**
  Get number of produced cells */
  inline int getCellsCnt()
  { return cellsCnt; };
  /**
  Get a specific cell converted to a 4-point polygon
  \param idx is the cell to convert
  \param poly is a pointer to the polygon structure, where to put the data.
  \param maxBoxEdgeValue limits the value of produced polygon vertex.
  \returns true if the polygon structure is set. */
  bool getCellPoly(int idx, UPolygon * poly, double maxBoxEdgeValue);
  /**
  Get number of cells in path */
  inline int getCellsPathCnt()
  { return pacsCnt; };
  /**
  Get a specific cell used in the path converted to a 4-point polygon
  \param idx is the cell to convert
  \param poly is a pointer to the polygon structure, where to put the data.
  \param maxBoxEdgeValue limits the value of produced polygon vertex.
  \returns ID of the cell (cell number). */
  int getCellPathPoly(int idx, UPolygon * poly, double maxBoxEdgeValue);
  /**
  Get list of obstacle points to avoid.
  This is the result of the cell-based obstacle avoidance path.
  The points to use is those where obstacle pointer is not NULL.
  Points are in the order from start pose to end pose, but
  start and exit point is not in the list.
  \returns a pointer to the vertex list array. */
  UAvoidCellVertex * getVertexList()
  { return pavs; };
  /**
  Get number of valid elements in the vertex list*/
  int getVertexListCnt()
  { return pavsCnt; };
  
  

private:
  /// get total number of vertices in graph
  int getMaxVertexCount(UAvoidObst ** aogs, const int aogsCnt);
  /**
  make a sortex list of valid vertices in the set of obstacles.
  \param aogs is a list of obstacle groups
  \param aogsCnt is the number of obstacle groups
  \param verts vertex list for cell generation
  \param vertsp vertex pointer list for cell generation
  \param vertMaxCnt is the size of the allocated vertex list array
  \returns number of valid vertices. */
  int fillVerticeList(UAvoidObst ** aogs, const int aogsCnt,
                      UAvoidVertexIdx * verts, UAvoidVertexIdx ** vertsp, const int vertMaxCnt);
  /**
  Make cells from these vertice points
  \param vertsp is a sorted list (in x order) of verteces
  \param vertsCnt is the number of vertices
  \param cells is an array of cells for the resulting cells (max size is twice then number of vertices.)
  \returns the number of produced cells.  */
  int makeCells(UAvoidVertexIdx** vertsp, const int vertsCnt, UAvoidCell* cells, bool debugDump);
  /**
  Get new initialized cell. */
  UAvoidCell * getNewCell();
  /**
  Find the cell with this position
  \param pos is the position for looking for a cell.
  \returns NULL if no cell is found - position is inside an obstacle.*/
  UAvoidCell * findCell(UPose pos);
  /**
  Find shortest path between these obstacles.
  \param pStart is the start pose and
  \param pEnd is the endPose
  \param costAdd list of positions with added cost.
  \returns number of path vertices returned in the 'pavs' array. */
  int findCellPath(UPose pStart, UPose pEnd, UAvoidVertexCosts * costAdd);
  /**
  Find avoidance points from list of cells - from this start end end position.
  The result is saved in the 'pavs' array and the pavsCnt is set to the number of cells->
  If a cell has an obstacle that is not NULL, then it is a point to avoid.
  \param pStart is the start pose and
  \param pEnd is the endPose
  \returns true if the 'pavs' array is set sucessfully */
  bool findCellPointPath(UPose pStart, UPose pEnd);
  /**
  Dump vertex list to console
  \param dest is a destination streem handle. */
  void printVetrexList(FILE * dest);
  /**
  Dump obstacles to console
  \param dest is a destination streem handle.
  \param aogs is array of obstacle groups
  \param aogsCnt is number of obstacle groups */
  void printObstacleList(FILE * dest, UAvoidObst ** aogs, const int aogsCnt);

private:
  /**
  Array of cells that constitute the path - reallocated if needed */
  UAvoidCell * cells;
  /**
  Number of allocated cells */
  int cellsCntMax;
  /**
  Number of actually used cells */
  int cellsCnt;
  /**
  Vertex list to pass to get from start to end, i.e. best path through the obstacles. */
  UAvoidCellVertex * pavs;
  /**
  Maksimum number of elements in verts */
  int pavsCntMax;
  /**
  Used number of vertices in verts */
  int pavsCnt;
  /**
  Array of cells from start to end. */
  UAvoidCell ** pacs;
  /**
  Number of cells in path */
  int pacsCnt;
  /**
  Pointer to array of vertices in current obstacle configuration */
  UAvoidVertexIdx * verts;
  /**
  Array of pointers in increasing x order to vertices in current obstacle configuration */
  UAvoidVertexIdx ** vertsp;
  /**
  Current number of vertices in array */
  int vertsCnt;
  /**
  Number of valid entries in vertex array */
  int vertsMaxCnt;
  /**
  Debug logfile */
  ULogFile debugLog;
  /**
  list of less wanted vertices, i.e additional cost when passing these vertex positions */
  //UAvoidVertexCost * costs;
  /**
  Number of valid positions in costs list */
  //int costsCnt;
  /**
  Number of available positions in costs list */
  //int costsCntMax;
  
public:
  /**
  Polygon with raw cell path */
  UPolygon40 polyCell;
  /**
  Polygon with cell path vertex list */
  UPolygon40 polyVertex;
};

#endif // UAVOIDCELLGRAPH_H
