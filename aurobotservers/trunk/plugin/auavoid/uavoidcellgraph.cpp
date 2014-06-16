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

#include <stdio.h>

#include "uavoidobst.h"
#include "uavoidparams.h"

#include "uavoidcellgraph.h"


// class UAvoidVertexCost
// {
// public:
//   /** position of vetrex with extra cost - a punished vertex position */
//   U2Dpos pos;
//   /**
//   is this point a point to avoid on the left side (in drive direction) */
//   bool avoidLeft;
//   /** extra cost value */
//   double cost;
// };

////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
#include "uavoidcellgraph.h"

class UAvoidVertexIdx
{
public:
  /// obstacle
  UAvoidObst * obst;
  /// other obstacle if this is an intersection
  UAvoidObst * obst2;
  /// index into obstacle (negative if an intersection)
  int oIdx;
  /// no-visibility line index (as registred in obst)
  /// when >= 0 then obst is the source object and obst2 is the obst with the cross-point
  // int novisIdx;
  /// point position for the cell generation
  U2Dpos pos;
  /** vertex type
  0 : left extreme (minimum x)
  1 : right extreme (max x)
  2 : upper point
  3 : lower point.
  4 : intersection close
  5 : intersection open
  6 : intersection upper
  7 : intersection lower
  8 : error
  */
  int type;

public:
  /**
  Set all values in vertex point */
  void set(UAvoidObst * ob1, UAvoidObst * ob2, int idx, UPosition pkt, int vType)
  {
    obst = ob1;
    obst2 = ob2;
    pos = pkt;
    oIdx = idx;
    type = vType;
  }
  /**
  Set all values in vertex point */
  void set(UAvoidObst * ob1, UAvoidObst * ob2, int idx, UPosition pkt)
  {
    obst = ob1;
    obst2 = ob2;
    pos = pkt;
    oIdx = idx;
    if (ob2 == NULL and idx >= 0)
      type = getPktType1();
    else
      type = -1;
  }
  /**
  Get point type
  -1 : undetermined, but one of 4..7
  0 : left extreme (minimum x)
  1 : right extreme (max x)
  2 : upper point
  3 : lower point.
  4 : intersection close
  5 : intersection open
  6 : intersection upper
  7 : intersection lower
  \returns point type */
  int getPktType()
  {
    int result;
    if (oIdx >= 0)
      // simpel vertex
      result = getPktType1();
    else
      // junction
      result = getPktType2();
    // debug
    if (result != type)
    {
      printf("******* type conflict estimated a %d, but previously found a type %d\n", result, type);
    }
    
    // debug end
    return result;
  }
  /**
  Get type if obstacle vertex. */
  int getPktType1()
  {
    int type;
    int p, n, m;
    double x1, x2, x3;
    m = obst->obst->getPointsCnt();
    if (oIdx == 0)
      p = m - 1;
    else
      p = oIdx - 1;
    if (m <= 0)
    { // debug - observed stray error, resulting in SIGFPE interrupt (during debug session)
      printf("UAvoidVertexIdx::getPktType1: error in obstacle size? odd! m=%d, but obst has %d edges\n", m,  obst->obst->getPointsCnt());
      n = 0;
    }
    else
      n = (oIdx + 1) % m;
    x1 = obst->obst->getPoint(p).x;
    x2 = pos.x;
    x3 = obst->obst->getPoint(n).x;
    if (x1 >= x2 and x3 > x2)
      type = 0;
    else if (x1 < x2 and x3 <= x2)
      type = 1;
    else if (x1 > x2 or x3 < x2)
      // previous is to the right and we are going CCV
      type = 2;
    else
      type = 3;
    return type;
  }
  /**
  get type, if point is an intersection */
  int getPktType2()
  {
    int type, is;
    U2Dpos py1, py2, p1;
    //bool isLine1, isLine2;
    ULineSegment seg1, seg2;
    // find position of a vertex to the left of the crossing point
    py1 = pos;
    py2 = pos;
/*    isLine1 = obst->obst->getPointsCnt() <= 2;
    // line 2 is treated as an obstacle if it is a no-visibility line
    isLine2 = obst2->obst->getPointsCnt() <= 2;
    if (not isLine1)
    {*/
      obst->obst->getDistance(pos.x, pos.y, &is);
      seg1 = obst->obst->getSegment(is);
/*    }
    if (not isLine2)
    {*/
      obst2->obst->getDistance(pos.x, pos.y, &is);
      seg2 = obst2->obst->getSegment(is);
//    }
    type = -1;
    if ((/*isLine1 or*/ seg1.vec.x <= 0.0) and
        (/*isLine2 or*/ seg2.vec.x <= 0.0))
    { // is an upper crossing - both segments has negative X-vector (CCV polygon)
      type = 6;
/*      if (oIdx == -2)
      { // is from obst->vertex to obst2.cog
        seg1 = stub->obst->getSegment(0);
        if (seg1.vec.x < 0.0)
          seg1 = stub->obst->getSegment(1);
        seg2 = getOther(stub)->obst->getSegment(0);
        if (seg2.vec.x < 0.0)
          seg2 = getOther(stub)->obst->getSegment(1);
        if (stubIsFwd and seg1.vec.y < seg2.vec.y)
          // lower crossing - and then open a new
          type = 7;
        else if (not stubIsFwd and seg1.vec.y > seg2.vec.y)
          // a closing
          type = 4;
      }*/
    }
/*    else if (isLine1 or isLine2)
    { // is either a close or a lower crossing
      // one line and one polygon
      if (isLine1)
        stub = obst;
      else
      {
        stub = obst2;
        seg2 = seg1;
      }
      seg1 = stub->obst->getSegment(0);
      if (seg1.vec.x < 0.0)
        seg1 = stub->obst->getSegment(1);
      // seg1 is segment in line or line-stub
      // seg 2 is segment on polygon at crossing pointing CCV
      if (seg1.vec.y > seg2.vec.y)
        type = 4;
      else
        type = 7;
    }*/
    else if (seg1.vec.x * seg2.vec.x < 0.0)
    { // is an opening or a close
      if (seg1.vec.x > 0)
      { // seg1 is an underside of a polygon (positive x and CCV polygon)
        // reverse vector for obstacle 2 and compare y-part
        if (-seg2.vec.y > seg1.vec.y)
          type = 4;
        else
          type = 5;
      }
      else
      { // seg2 is an underside of a polygon (positive x and CCV polygon)
        // reverse vector for obstacle 1 and compare y-part
        if (-seg1.vec.y > seg2.vec.y)
          type = 4;
        else
          type = 5;
      }
/*      if (obLow == NULL)
        type = 5;
      else if (obLow->obst == obst->obst or obLow->obst == obst2->obst)
        // one of the obstacles are below cell, then it is a close
        type = 4;
      else
      {  // none is below, then a new cell
        type = 5;
        printf("getPktType2: type=5, but this should not in this part of the if?\n");
      }*/
    }
    else
      // must be that both segments has positive X-vector, and thus
      // a crossing below obstacles.
      type = 7;
    return type;
  }
  /**
  Get upper obstacle to the right of an opening junction, with increasing x */
  UAvoidObst * getUpper()
  {
    int edge, vert;
    ULineSegment seg1;
    // find position of a vertex to the left of the crossing point
    obst->obst->getDistance(pos.x, pos.y, &edge, &vert);
    if (vert == -1)
      // close to an edge - should be, as this is a crossing
      seg1 = obst->obst->getSegment(edge);
    else
    {  // else just get segment after vertex - should not be the case
      printf("UAvoidVertexIdx::getUpper: Result is undefined as point is a vertex\n"); 
      seg1 = obst->obst->getSegment(vert);
    }
    if (seg1.vec.x > 0.0)
      return obst;
    else
      return obst2;
  }
  /**
  Get upper obstacle to the right of junction */
/*  UAvoidObst * getLower()
  {
    U2Dpos py1, py2, p1;
    // find position of a vertex to the left of the crossing point
    py1 = pos;
    py2 = pos;
    for (int i=0; i < obst->obst->getPointsCnt(); i++)
    {
      p1 = obst->obst->getPoint(i);
      if (p1.x > py1.x)
      {
        py1 = p1;
        break;
      }
    }
    for (int i=0; i < obst2->obst->getPointsCnt(); i++)
    {
      p1 = obst2->obst->getPoint(i);
      if (p1.x > py2.x)
      {
        py2 = p1;
        break;
      }
    }
    // normalize
    py1.y /= py1.dist(pos);
    py2.y /= py2.dist(pos);
    if (py1.y > py2.y)
      return obst2;
    else
      return obst;
  }*/
  /**
  Get upper obstacle to the right of junction */
  UAvoidObst * getOther(UAvoidObst * notThis)
  {
    if (obst == notThis)
      return obst2;
    else
      return obst;
  }
  /**
  Extend one of the obstacles with a third point with another x-value,
  to avoid x-clashes. */
  void extent4thPnt(U2Dpos pos)
  {
    UObstacle * om = obst->obst;
    int im = 0;
    double d, dm;
    //
    dm = pos.dist(om->getPoint(im));
    for (int i = 1; i < om->getPointsCnt(); i++)
    {
      d = pos.dist(om->getPoint(i));
      if (d < dm)
      {
        dm = d;
        im = i;
      }
    }
    if (obst2 != NULL)
    {
      for (int i = 0; i < obst2->obst->getPointsCnt(); i++)
      {
        d = pos.dist(obst2->obst->getPoint(i));
        if (d < dm)
        {
          dm = d;
          im = i;
          om = obst2->obst;
        }
      }
    }
    UPolygon40 p40;
    om->copyTo(&p40);
    // move conflict point to slightly earlier, and
    p40.getPoints()[im].x -= 0.0012;
    // ... add a new point later (in x) to solve x-clash problem
    p40.add(pos.x + 0.0012, pos.y);
    // make sure result is convex
    p40.extractConvexTo(om);
    // if (om->getPointsCnt() < p40.getPointsCnt())
    {
      printf("¤¤¤¤¤¤¤ extended visibility line (obst=%lu, index=%d) from %.5fx,%.5fy with %.5fx,%.5fy, from %d to %d pkts\n",
             om->getSerial(), im, pos.x, pos.y, pos.x+0.005, pos.y, p40.getPointsCnt() -1, om->getPointsCnt());
    }
  }

};

////////////////////////////////////////////////////////////

int compareUAvoidVertexIdx (const void * a, const void * b)
{
  UAvoidVertexIdx ** vta = (UAvoidVertexIdx**) a;
  UAvoidVertexIdx ** vtb = (UAvoidVertexIdx**) b;
  if ((*vta)->pos.x > (*vtb)->pos.x)
    return 1;
  else if ((*vta)->pos.x == (*vtb)->pos.x)
    return 0;
  else
    return -1;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

class UAvoidCell
{
public:
  /**
  Constructor */
  UAvoidCell()
  {
    init();
  }
  /**
  Clear cell */
  inline void init()
  {
    for (int i = 0; i < 4; i++)
    {
      cells[i] = NULL;
    }
    obsts[0] = NULL;
    obsts[1] = NULL;
  };
  /// set x value for start positions
  void setXLeft(double x)
  {
    oPoss[0].x = x;
    oPoss[1].x = x;
  };
  
  /// set x value for start positions
  void setXRight(double x)
  {
    oPoss[2].x = x;
    oPoss[3].x = x;
  };
  
  /// is this y-value inside right side of cell
  bool isInRightY(double y)
  {
    return y <= oPoss[2].y and y >= oPoss[3].y;
  }
  
  /**
  Set intersection points for this open cell.
  Sets also the next cells further down the line.
  \param x is the x-intersection value.
  \param vt is the related vertex*/
  bool setCuts(double x, ULogFile * debugLog)
  { // get obstacle intersection with line at this x
    U2Dlined ly(x, 0.0,   x, 1.0);
    int i;
    U2Dpos cuts[2];
    bool result = true;
    //
    if (obsts[0] == NULL)
      // top most cell
      oPoss[2].set(x, 1e6);
    else if (obsts[0] != cells[2]->obsts[1])
    { // Upper limit of this cell is different than the cell above, needs recalculation 
      i = obsts[0]->obst->cutPoints(&ly, cuts, 2);
      if (i == 2 and cuts[0].y > cuts[1].y)
        // we need lower value only
        cuts[0].y = cuts[1].y;
      oPoss[2] = cuts[0];
    }
    if (obsts[1] != NULL)
    {
      i = obsts[1]->obst->cutPoints(&ly, cuts, 2);
      if (i == 2 and cuts[0].y < cuts[1].y)
      { // swap y values
        double d = cuts[0].y;
        cuts[0].y = cuts[1].y;
        cuts[1].y = d;
      }
      else if (i == 1)
        cuts[1] = cuts[0];
      else if (i == 0)
      { // just passed the polygon ??
        if (debugLog != NULL and debugLog->getF() != NULL)
          fprintf(debugLog->getF(), "no intersections - cell %d, obst %lu at %.5fx - should not be!!!!!!! - find error!\n", id, obsts[1]->obst->getSerial(), x);
        printf("no intersections - cell %d, obst %lu at %.5fx - should not be!!!!!!! - find error!\n", id, obsts[1]->obst->getSerial(), x);
        result = false;
      }
      //
      if (result)
      { // set buttom value
        oPoss[3] = cuts[0];
        // set top value of next cell
        if (cells[3] == NULL)
        {
          if (debugLog != NULL and debugLog->getF() != NULL)
            fprintf(debugLog->getF(), "UAvoidCell::setCuts cell %d has a lower obstacle (%lu), but is not linked to a lower cell?? - needs fixing!!\n",
                id, obsts[1]->obst->getSerial());
          printf("UAvoidCell::setCuts cell %d has a lower obstacle (%lu), but is not linked to a lower cell?? - needs fixing!!\n",
                id, obsts[1]->obst->getSerial());
        }
        else
        { // if same obstacle, then set next top edge of cell
          cells[3]->oPoss[2] = cuts[1];
          // set also next cell further down
          result = cells[3]->setCuts(x, debugLog);
        }
      }
    }
    else
      // bottom most cell
      oPoss[3].set(x, -1e6);
    return result;
  }

  /**
  Remove this open cell from open cell list */
  void unlinkOpen()
  {
    if (cells[2] != NULL)
      // set above open cell to be just above the one below this cell.
      cells[2]->cells[3] = cells[3];
    if (cells[3] != NULL)
      // set the below cell to be just below the cell above this.
      cells[3]->cells[2] = cells[2];
    cells[2] = NULL;
    cells[3] = NULL;
  }
  /**
  Link this cell to these 4 other cells.
  \param arg1 is the previous (upper) left
  \param arg2 is the previous lower left
  \param arg3 is the next (upper) right cell,
  \param arg4 is the next lower right cell. */
  void link(UAvoidCell* arg1, UAvoidCell * arg2, UAvoidCell* arg3, UAvoidCell* arg4, bool openList)
  {
    cells[0] = arg1;
    cells[1] = arg2;
    cells[2] = arg3;
    cells[3] = arg4;
    if (openList)
    { // link into open list
      if (cells[2] != NULL)
        cells[2]->cells[3] = this;
      if (cells[3] != NULL)
        cells[3]->cells[2] = this;
    }
  }
  /**
  Print cell content */
  void print(FILE * dest)
  {
    int p[4];
    int s[2];
    for (int i = 0; i < 4; i++)
    {
      if (cells[i] != NULL)
        p[i] = cells[i]->id;
      else
        p[i] = -1;
    }
    for (int i = 0; i < 2; i++)
    {
      if (obsts[i] != NULL)
        s[i] = (int)obsts[i]->obst->getSerial();
      else
        s[i] = -1;
    }
    fprintf(dest, "Cell %3d prev: %3d %3d, next: %3d %3d, upper %2d (%.3fx,%.3fy - %.3fx,%.3fy)\n"
           "                                       lower %2d (%.3fx,%.3fy - %.3fx,%.3fy)\n",
           id, p[0], p[1], p[2], p[3],
           s[0], oPoss[0].x, oPoss[0].y, oPoss[2].x, oPoss[2].y,
           s[1], oPoss[1].x, oPoss[1].y, oPoss[3].x, oPoss[3].y);
  }
  /**
  Set cell corners into polygon */
  void getPoly(UPolygon * poly, double maxVal)
  { // get values in CCV order
    // top left
    poly->add(fmax(fmin(oPoss[0].x, maxVal), -maxVal), fmax(fmin(oPoss[0].y, maxVal), -maxVal));
    // bottom-left
    poly->add(fmax(fmin(oPoss[1].x, maxVal), -maxVal), fmax(fmin(oPoss[1].y, maxVal), -maxVal));
    // bottom-right
    poly->add(fmax(fmin(oPoss[3].x, maxVal), -maxVal), fmax(fmin(oPoss[3].y, maxVal), -maxVal));
    // top right
    poly->add(fmax(fmin(oPoss[2].x, maxVal), -maxVal), fmax(fmin(oPoss[2].y, maxVal), -maxVal));
    // set as closed polygon
    poly->setAsPolygon();
  }
  /**
  Is this next ID a cell in front of this cell - i.e. either cell[2] or cell[3] */
  bool isFwd(int nextID)
  {
    bool result = false;
    if (cells[2] != NULL)
    {
      result = cells[2]->id == nextID;
      if (not result and cells[3] != NULL)
        result = cells[3]->id == nextID;
    }
    return result;
  }
  /**
  Is this next ID a cell behind this cell - i.e. either cell[0] or cell[1] */
  bool isBack(int nextID)
  {
    bool result = false;
    if (cells[0] != NULL)
    {
      result = cells[0]->id == nextID;
      if (not result and cells[1] != NULL)
        result = cells[1]->id == nextID;
    }
    return result;
  }
  /**
  Is index to this ID
  \param nextID is the cell ID to look for.
  \returns -1 if cell ID is not found.
  \returns 0..3 if the cell ID is found. */
  int getCellIdx(int nextID)
  {
    int result = -1;
    for (int i = 0; i < 4; i++)
    {
      if (cells[i] != NULL)
        if (cells[i]->id == nextID)
        { // cell ID is found.
          result = i;
          break;
        }
    }
    return result;
  }
  /**
  add punishment to this cell
  \param value is added to the cost of passing the cell */
/*  void punish(double value)
  {
    punishment += value;
  }*/
  /**
  Set const of this cell
  \param value set as the cost of the cell */
/*  void cost(double value)
  {
    punishment = value;
  }*/
  
public:
  /// neighboring cells top-left, bot-left, top-right, bot-right
  /// top is higher y-value
  /// not valid are set to NULL
  UAvoidCell * cells[4];
  /// neighboring obstacles top (high y), bottom (low y)
  /// if free space then set to NULL
  UAvoidObst * obsts[2];
  /// position of corner - if free space set to +/- 1e6
  U2Dpos oPoss[4];
  /// ID of this cell
  int id;
  // / punishment
  // double punishment;
};


////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

UAvoidCellGraph::UAvoidCellGraph()
{
  cells = NULL;
  cellsCntMax = 0;
  cellsCnt = 0;
  pavs = NULL;
  pavsCnt = 0;
  pavsCntMax = 0;
  pacs = NULL;
  pacsCnt = 0;
  verts = NULL;
  vertsp = NULL;
  vertsCnt = 0;
  vertsMaxCnt = 0;
  // debug
  // debugLog.openLog("avoidCell");
  // debug end
}

////////////////////////////////////////////////////////////

UAvoidCellGraph::~UAvoidCellGraph()
{
  if (cells != NULL)
    free(cells);
  if (pavs != NULL)
    free(pavs);
  if (pacs != NULL)
    free(pacs);
  if (vertsp != NULL)
  { // vertex list and sorted pointer array is allocated
    free(verts);
    free(vertsp);
  }
  debugLog.closeLog();
}

////////////////////////////////////////////////////////////

void UAvoidCellGraph::makeCellDecomposition(UAvoidObst * aogs[], int aogsCnt, UAvoidParams* par, bool debugDump)
{
  UTime t;
  int n;
  //
  t.now();
  if (debugLog.isOpen())
    debugLog.toLog("scan", par->scanSerial, "from lobst.scan");
  // count vertices in all obstacles.
  n = getMaxVertexCount(aogs, aogsCnt) + 1;
  if (n > vertsMaxCnt)
  { // reallocate array of vertices
    vertsMaxCnt = n;
/*    if (vertsp != NULL)
    { // remove previous too short array
      free(verts);
      free(vertsp);
    }*/
    // allocate space for index
    verts = (UAvoidVertexIdx*)realloc(verts, sizeof(UAvoidVertexIdx) * vertsMaxCnt * 3); // new UAvoidVertexIdx[vertsMaxCnt * 3];
    vertsp = (UAvoidVertexIdx**)realloc(vertsp, sizeof(UAvoidVertexIdx*)* vertsMaxCnt * 3); // new UAvoidVertexIdx*[vertsMaxCnt * 3];
  }
  // fill with vertices
  for (int i = 0; i < 5; i++)
  {
    vertsCnt = fillVerticeList(aogs, aogsCnt, verts, vertsp, vertsMaxCnt * 3);
    if (vertsCnt >= -1)
      // there is no x-position clash - or an error
      break;
    // there is an x-position clash - try again
  }
  // debug
  printf("UAvoidCellGraph::findPath: finished fillVerticeList (cnt=%d) in %.5f sec.\n", vertsCnt, t.getTimePassed());
  // debug end
  if (vertsCnt == -1)
    // error situation - out of space - assume next scan is better
    return;
  //
  if (vertsCnt < 0)
  { // still clash, bu we will continue
    vertsCnt = -vertsCnt;
    if (debugLog.isOpen())
      fprintf(debugLog.getF(), "UAvoidCellGraph::findPath (fillVerticeList): vertex clash in x-values (5 times - continues)\n");
  }
  // at maximum twice as many cells as vertices
  if (cellsCntMax < vertsCnt * 2 + 2)
  {
    cellsCntMax = vertsCnt * 2 + 2;
/*    if (cells != NULL)
      delete cells;*/
    cells = (UAvoidCell*)realloc(cells, sizeof(UAvoidCell)*cellsCntMax); // new UAvoidCell[cellsCntMax];
    //index cells
    for (int i = 0; i < cellsCntMax; i++)
    {
      cells[i].init();
      cells[i].id = i;
    }
  }
  // make cells
  cellsCnt = 0;
  cellsCnt = makeCells(vertsp, vertsCnt, cells, debugDump);
  if (cellsCnt < 0 or debugDump)
  { // error in generation of cells - dump obstacles and vertices
    if (debugLog.isOpen())
    { // dump situation to file
      printObstacleList(debugLog.getF(), aogs, aogsCnt);
      printVetrexList(debugLog.getF());
      fflush(debugLog.getF());
    }
    // debug
    //printObstacleList(stdout, aogs, aogsCnt);
    //printVetrexList(stdout);
    // debug end
  }
  printf("UAvoidCellGraph::findPath: finished makeCells after %.5f sec.\n", t.getTimePassed());
  // debug print cells
/*  printf("--finished making graph %d cells out of %d vertices\n", cellsCnt, vertsCnt);
  for (int i = 0; i < cellsCnt; i++)
  {
    cells[i].print(stdout);
  }*/
  // debug end
  // set cell cost (in addition to distance) for all cells to zero
}

/////////////////////////////////////////////////////////////////////////////

int UAvoidCellGraph::findBestCellPath(UAvoidParams* par, UAvoidVertexCosts * costAdd, bool debugDump)
{ // find cell with start point
  // is result array big enough?
  if (pavsCntMax < cellsCntMax)
  { // make space for result
    int n = maxi(1024, cellsCntMax);
    // debug
    printf("Allocates space for %d, but needs just %d (was %d) avoid cells vertex, and avoid cells\n", n, cellsCntMax, pavsCntMax);
    // debug end
    pavs = (UAvoidCellVertex*) realloc(pavs, sizeof(UAvoidCellVertex) * n);
    memset(pavs, 0, sizeof(UAvoidCellVertex) * n);
    pacs = (UAvoidCell**) realloc(pacs, sizeof(UAvoidCell*) * n);
    pavsCntMax = n;
  }
  //
/*  if (punishCurrentPath)
  { // punish the full path - except start and end cell
    for (int i = 1; i < pavsCnt - 1; i++)
    {
      if (pavs[i].obst != NULL)
      {
        for (c = 0; c < costsCnt; c++)
        {
          if (costs[c].pos.dist(pavs[i].pos) < 0.01 and costs[c].avoidLeft == pavs[i].avoidLeft)
          { // existing position
            costs[c].cost += 0.1;
            break;
          }
        }
        if (c == costsCnt)
        { // new position
          costs[c].cost = 0.1;
          costs[c].pos = pavs[i].pos;
          costs[c].avoidLeft = pavs[i].avoidLeft;
          costsCnt = c;
        }
      }
    }
  }*/
  for (int i = 0; i < 1; i++)
  {
    int n = 0, i1 = -1, i2;
    double aPath, aRelStart, aRelEnd;
    // find sequence of celle using an A* algorithm
    findCellPath(par->startPose, par->exitPose, costAdd);
    //  printf("UAvoidCellGraph::findPath: finished    findCellPath after %.5f sec.\n", t.getTimePassed());
    //
    // debug
/*    for (int m = 0; m < pacsCnt; m++)
    {
      int ou = -1,ol = -1;
      if (pacs[m]->obsts[0] != NULL)
      {
        if (pacs[m]->obsts[0]->obst == NULL)
          printf("UAvoidCellGraph::findBestCellPath: odd - an obstacle exist, but the obstacle is missing?\n");
        else
          ou = pacs[m]->obsts[0]->obst->getSerial();
      }
      if (pacs[m]->obsts[1] != NULL)
        ol = pacs[m]->obsts[1]->obst->getSerial();
      if (pacs[m]->id > 2000)
        printf("*************** bad! cell %d has ID = %d\n", m, pacs[m]->id);
      printf("%d Path cell has ID=%d and obstup=%3d, obstDown=%3d\n", m, pacs[m]->id, ou, ol);
    }*/
    // debug end 
    // find vertices to avoid in this cell sequence
    findCellPointPath(par->startPose, par->exitPose);
    //
    for (int j = 0; j < pavsCnt; j++)
    { // find first and last vertex to avoid
      if (pavs[j].obst != NULL)
      {
        if (i1 < 0)
          i1 = j;
        i2 = j;
        n++;
        // debug
        // printf("Path test %d path index %d cell %d vertex position %.3fx,%.3fy\n", i, j,
        //       pacs[j]->id, pavs[j].pos.x, pavs[j].pos.y);
        // debug end
      }
      //else
        // debug
      //  printf("Path test %d path index %d cell %d \n", i, j, pacs[j]->id);
        // debug end
    }
    if (n == 0)
      // nothing to avoid - must be best path
      break;
    // test for starting and ending backwards
    aPath = atan2(pavs[i1].pos.y - par->startPose.y,
                  pavs[i1].pos.x - par->startPose.x);
    aRelStart = limitToPi(aPath - par->startPose.h);
    // end pose
    aPath = atan2(par->exitPose.y - pavs[i2].pos.y,
                  par->exitPose.x - pavs[i2].pos.x);
    aRelEnd = limitToPi(par->exitPose.h - aPath);
    //
    printf(" -- start turn %.4f to path index %d, finish turn %.4f from path index %d\n", aRelStart, i1, aRelEnd, i2);
    //
    if (fabs(aRelStart) < M_PI / 3.0 and fabs(aRelEnd) < M_PI / 3.0)
      break;
    // punish the vertex position next after the start position
    costAdd->setExtraCost(pavs[i1].pos, pavs[i1].avoidLeft, fabs(aRelStart) / 2.0);
    costAdd->setExtraCost(pavs[i2].pos, pavs[i2].avoidLeft, fabs(aRelEnd) / 2.0);
    // debug
    //costAdd->list();
    // debug end
  }
  //
  // debug polyline from avoid points
  polyVertex.clear();
  polyVertex.setAsPolyline();
  polyVertex.add(par->startPose.x, par->startPose.y);
  for (int i = 0; i < pavsCnt; i++)
  {
    if (pavs[i].obst != NULL)
      polyVertex.add(pavs[i].pos.x, pavs[i].pos.y);
  }
  polyVertex.add(par->exitPose.x, par->exitPose.y);
  //  printf("UAvoidCellGraph::findPath: finished findCellPointPath after %.5f sec.\n", t.getTimePassed());
  // debug end
  //
  return pavsCnt;
}

////////////////////////////////////////////////////////////

void UAvoidCellGraph::printObstacleList(FILE * dest, UAvoidObst ** aogs, const int aogsCnt)
{
  UAvoidObst * og;
  //
  fprintf(dest, "Obstacle list of %d spatially separated obstacle groups:\n", aogsCnt);
  for (int i = 0; i < aogsCnt; i++)
  {
    og = aogs[i];
    fprintf(dest, " - obstacle group %d\n", i);
    while (og != NULL)
    {
      fprintf(dest, " -- serial %3lu (%2d vis lines) :", og->obst->getSerial(), og->noVisCnt);
      for (int k = 0; k < og->obst->getPointsCnt(); k++)
      {
        UPosition pos;
        pos = og->obst->getPoint(k);
        fprintf(dest, " (%.4fx,%.4fy)", pos.x, pos.y);
      }
      fprintf(dest, "\n");
      og = og->grp;
    }
  }
}

////////////////////////////////////////////////////////////

int UAvoidCellGraph::getMaxVertexCount(UAvoidObst ** aogs, const int aogsCnt)
{
  UAvoidObst * og;
  int result = 0;
  bool single;
  //
  for (int i = 0; i < aogsCnt; i++)
  {
    og = aogs[i];
    // is it a single point and not in a group.
    single = og->obst->getPointsCnt() == 1 and og->grp == NULL;
    // single points do not count.
    while (og != NULL and not single)
    {
      result += og->obst->getPointsCnt();;
      og = og->grp;
    }
  }
  return result;
}

//////////////////////////////////////////////////////////////

int UAvoidCellGraph::fillVerticeList(UAvoidObst ** aogs, const int aogsCnt,
                                     UAvoidVertexIdx * verts, UAvoidVertexIdx ** vertsp, const int vertMaxCnt)
{
  int n, vertsCnt = 0;
  UAvoidVertexIdx * vi = verts, *vi2;
  UAvoidVertexIdx ** vip = vertsp;
  UAvoidObst * og, *ogg, *ogh;
  UPosition p1;
  bool failed = false;
  const int XPM = 8;
  UPosition xes[XPM];
  int xType[XPM];
  bool clash;
  struct
  { /**
    Is this position inside another obstacle */
    bool pktEmbedded(UPosition pkt, UAvoidObst * group, UAvoidObst * not1, UAvoidObst * not2, UAvoidObst ** closeTo)
    {
      UAvoidObst * ogt = group;
      double d = 1.0;
      while (ogt != NULL)
      { // test if point is embedded in another obstacle
        if (ogt != not1 and ogt != not2 and ogt->obst->getPointsCnt() > 2)
        { // not the same obstacle
          d = ogt->obst->getDistanceXYsigned2(pkt, NULL, NULL, NULL);
          if (d < 0.0)
            break;
        }
        ogt = ogt->grp;
      }
      return d < 0.0;
    }
    /**
    Is the point near a side of another obstacle - then move the point */
    int pktMoveIfClose(UPosition * pkt, UAvoidObst * group, UAvoidObst * not1, double borderDist)
    {
      UAvoidObst * ogt = group;
      double d;
      int s1idx, s2idx = -1;
      bool aVertex;
      int moves = 0;
      UPosition pos;
      //
      while (ogt != NULL)
      { // test if point is embedded in another obstacle
        if (ogt != not1 and ogt->obst->getPointsCnt() > 2)
        { // not the same obstacle
          d = ogt->obst->getDistanceXYsigned2(*pkt, &s1idx, &pos, &aVertex);
          if (d < borderDist and d > -borderDist)
          {
            ULineSegment s1, s2, s3;
            UPosition v1;
            double t;
            s1 = ogt->obst->getSegment(s1idx);
            if (aVertex)
            { // find normal vector to the left (inwards)
              // relative to sum of 2 adjesent sides
              s2idx = s1idx - 1;
            }
            else
            { // closest to a side, but mat be close to a vertex too.
              t = s1.getPositionOnLine(pos);
              if (t < borderDist)
              { // close to previous - move away from start of edge
                s2idx = s1idx - 1;
                aVertex = true;
                pos = s1.pos;
              }
              else if (s1.length - t < borderDist)
              { // close to end of side - move away from end of edge
                s2idx = (s1idx + 1) % ogt->obst->getPointsCnt();
                aVertex = true;
                pos = s1.getOtherEnd();
              }
            }
            // get normal (unit vector) to side inwards (to the left) and a bit
            v1.set(-s1.vec.y * 1.01, s1.vec.x * 1.01, 0.0);
            if (aVertex)
            { // close to a vertex, so add also a distance inwards
              // from other side (and a bit)
              if (s2idx < 0)
                s2idx = ogt->obst->getPointsCnt() - 1;
              s2 = ogt->obst->getSegment(s2idx);
              // add unit vector away from this side too.
              v1.x += -s2.vec.y * 1.01;
              v1.y += s2.vec.x * 1.01;
            }
            // get segment from cog to current point
            s3.setFromPoints(not1->obst->getCogXY(), pos);
            // find new position away from polygon
            if (d > 0.0)
              // move outwards - change sign and scale
              v1 *= -borderDist;
            else
              // move inwards - just scale
              v1 *= borderDist; 
            pos += v1;
            t = s3.getPositionOnLine(pos);
            if (t < s3.length)
            { // obstacle has mooved towards obstacle centre - this may lead to openings and is bad
              // move back to same distance - and a bit
              v1 = s3.vec * (s3.length - t + borderDist);
              pos += v1;
            }
            // implement new position
            // debug
/*            printf("obstacle moved (%d.%02lu) near %gm obst (%d.%02lu) vertex=%s (idx1=%d idx2=%d) at %.4fx,%.4fy to %.4fx,%.4fy\n",
                      not1->grpIdx, not1->obst->getSerial(), d, ogt->grpIdx, ogt->obst->getSerial(), bool2str(aVertex), s1idx, s2idx,
                      pkt->x, pkt->y, pos.x, pos.y);*/
            // debug end

            // move actual vertex on obstacle
            (*pkt) = pos;
            moves++;
          }
        }
        ogt = ogt->grp;
      }
      return moves;
    }
    /**
    Is this obstacle embedded in any other from the group */
    bool obsEmbedded(UAvoidObst * obs, UAvoidObst * group)
    {
      UAvoidObst * ogt = group;
      UPolygon * op = obs->obst;
      double d = 1.0;
      while (ogt != NULL)
      { // test if obs is embedded in the ogt obstacle
        if (ogt != obs and ogt->obst->isEmbedded(op, &d))
          break;
        ogt = ogt->grp;
      }
      return (ogt != NULL);
    }
  } test;
  ULineSegment seg;
  int moves;
  double borderDist = 0.005;
  //
  printf("----------------- moving vertices near other obstacles inside the other obstacle\n");
  for (int v = 0; v < 3; v++)
  {
    moves = 0;
    for (int i = 0; i < aogsCnt; i++)
    { // condition all vertices, so that all points are either safe within other obstacles or safe outside.
      // this is to avoid missing cell-edges due to rounding errors
      og = aogs[i];
      // single points do not count (will not change route)
      while (og != NULL and not failed)
      { // single points are not used - covered by visibility lines
        // normal valid obstacle >= 3 points
        int mCnt = 0;
        if (og->obst->getPointsCnt() >= 3)
        { // add crossing points with other obstacles

          // debug
          if (debugLog.isOpen())
          { // dunp obstacles to logfile too
            fprintf(debugLog.getF(), "obstacle (%d.%02lu) %d: ", og->grpIdx, og->obst->getSerial(), og->obst->getPointsCnt());
            UPosition pu;
            for (int u = 0; u < og->obst->getPointsCnt(); u++)
            {
              pu = og->obst->getPoint(u);
              fprintf(debugLog.getF(), " %.4fx,%.4fy;", pu.x, pu.y);
            }
            fprintf(debugLog.getF(), "\n");
          }
          // debug end

          // add all visible vertices on this polygon
          for (int j = 0; j < og->obst->getPointsCnt(); j++)
          { // test this point
            UPosition * pp1 = og->obst->getPoints();
            int m = test.pktMoveIfClose(pp1, aogs[i], og, borderDist);
            if (m > 0 and debugLog.isOpen())
            {
              fprintf(debugLog.getF(), " - moved point %d to %.4fx,%.4fy in %d steps\n", j, pp1->x, pp1->y, m);
            }
            mCnt += m;
          }
        }
        if (mCnt > 0)
          // ensure result is convex
          og->obst->toConvex();
        og = og->grp;
        moves += mCnt;
      }
    }
    //printf("----------------- moved %d vertices - border dist = %g\n", moves, borderDist);
    if (debugLog.isOpen())
      fprintf(debugLog.getF(), "----------------- moved %d vertices - border dist = %g\n", moves, borderDist);
      
    borderDist *= 0.5;
    if (moves == 0)
      break;
  }
  //
  for (int i = 0; i < aogsCnt; i++)
  {
    og = aogs[i];
    //single = og->obst->getPointsCnt() == 1 and og->grp == NULL;
    // single points do not count (will not change route)
    while (og != NULL and not failed)
    { // single points are not used - covered by visibility lines
      if (og->obst->getPointsCnt() == 1 or og->obst->getPointsCnt() == 2)
      { // points are to be ignored and lines should have been removed (to 3-point obsts) 
        if (og->obst->getPointsCnt() == 2
            and debugLog.isOpen())
          fprintf(debugLog.getF(), "UAvoidCellGraph::fillVerticeList: Found TWO-point obstacle (%d.%lu) "
             "- should have been changed to 3-point!\n", og->grpIdx, og->obst->getSerial());
        //if (debugLog.isOpen())
        //  fprintf(debugLog.getF(), "UAvoidCellGraph::fillVerticeList: Found ONE-point obstacle (%d.%lu) - is eliminated by visibility line!\n", og->grpIdx, og->obst->getSerial());
      }
      // normal valid obstacle >= 3 points
      else if (not test.obsEmbedded(og, aogs[i]))
      {// add all visible vertices on this polygon
        for (int j = 0; j < og->obst->getPointsCnt(); j++)
        { // test this point
          p1 = og->obst->getPoint(j);
          ogh = NULL;
          if (not test.pktEmbedded(p1, aogs[i], og, NULL, &ogh))
          { // vertex is not embedded, so save
            vi->set(og, NULL, j, p1);
            // advance to next
            *vip++ = vi++;
            failed = (vi - verts > vertMaxCnt);
            if (failed)
              break;
          }
        }
        // add crossing points with other obstacles
        ogg = og->grp;
        while (ogg != NULL)
        { // test if there is an overlap
          // debug
/*          if ((ogg->obst->getSerial() == 80 and og->obst->getSerial() == 82) or
              (ogg->obst->getSerial() == 82 and og->obst->getSerial() == 80))
            printf("we have reached crossing of obst 80 and 82\n");*/
          // debug end
          if (og->obst->getPointsCnt() > 2)
          {
            og->obst->isOverlappingXYtype(ogg->obst, &n, xes, XPM, xType);
            if (n >= 2)
            { // 2 (or 4) crossings, so add all
              for (int xp = 0; xp < n; xp++)
              { // add crossing point if not embedded
                if (not test.pktEmbedded(xes[xp], aogs[i], og, ogg, &ogh))
                { // point is not embedded, so save
                  vi->set(og, ogg, -1, xes[xp], xType[xp]);
                  // advance to next
                  *vip++ = vi++;
                  failed = (vi - verts >= vertMaxCnt);
                  if (failed)
                    break;
                }
              }
            }
            if ((n % 2) == 1 and debugLog.isOpen())
            {
              fprintf(debugLog.getF(), "UAvoidCellGraph::fillVerticeList: obst %d.%lu and %d.%lu."
              " is just touching (%d crossings) - should have been removed earlier\n",
              og->grpIdx, og->obst->getSerial(), ogg->grpIdx, ogg->obst->getSerial(), n);
            }
          }
          ogg = ogg->grp;
        }
        if (failed)
          break;
      }
      og = og->grp;
    }
  }
  // get number of valid vertex points
  vertsCnt = vi - verts;
  if (failed)
  {
    printf("UAvoidCellGraph::fillVerticeList - out of space - had space for %d\n", vertMaxCnt);
    return -1;
  }
  //
/*  for (int i = 0; i < vertsCnt; i++)
  {
    int o2 = -1;
    vi = vertsp[i];
    if (vi->obst2 != NULL)
      o2 = vi->obst2->obst->getSerial();
    // debug
    printf("- vertex %d (obst %lu idx %d) (and %d) at (%.4fx,%.4fy)\n", i, vi->obst->obst->getSerial(), vi->oIdx, o2, vi->pos.x, vi->pos.y);
    // debug end
  }*/
  // sort vertices pointer list
  qsort(vertsp, vertsCnt, sizeof(UAvoidVertexIdx*), compareUAvoidVertexIdx);

  //printf("\n");
  clash = false;
  for (int i = 0; i < vertsCnt; i++)
  {
    vi = vertsp[i];
    if (i > 0)
    {
      vi2 = vertsp[i - 1];
      if (vi->pos.x == vi2->pos.x)
      { // there is an x-position clash - may give wrong results
        if (debugLog.isOpen())
          fprintf(debugLog.getF(), "UAvoidCellGraph::fillVerticeList: there is an x-position clash at (%fx,%fy) obst %lu idx %d <==> (%fx,%fy) obst %lu idx %d\n", vi->pos.x, vi->pos.y,
               vi->obst->obst->getSerial(), vi->oIdx, vi2->pos.x, vi2->pos.y,
               vi2->obst->obst->getSerial(), vi2->oIdx);
        if (vi->oIdx >= 0)
        { // move this point a bit forward
          vi->obst->obst->getPoints()[vi->oIdx].x += 0.001;
          // and set the new vertex position too.
          vi->pos = vi->obst->obst->getPoints()[vi->oIdx];
          clash = true;
        }
        else if (vi2->oIdx >= 0)
        { // move previous point instead
          vi2->obst->obst->getPoints()[vi2->oIdx].x += 0.001;
          // and set the new vertex position too.
          vi2->pos = vi2->obst->obst->getPoints()[vi2->oIdx];
          clash = true;
        }
        else
        {
          int viObst2 = -1, vi2Obst2 = -1;
          if (vi->obst2 != NULL)
            viObst2 = vi->obst2->obst->getSerial();
          if (vi2->obst2 != NULL)
            vi2Obst2 = vi2->obst2->obst->getSerial();
          if (debugLog.isOpen())
            fprintf(debugLog.getF(), "UAvoidCellGraph::fillVerticeList: obstacle-crossings has same x-value (%fx,%fy) obst %lu,%d idx %d <==> (%fx,%fy) obst %lu,%d idx %d\n",
                    vi->pos.x, vi->pos.y,
               vi->obst->obst->getSerial(), viObst2, vi->oIdx, vi2->pos.x, vi2->pos.y,
               vi2->obst->obst->getSerial(), vi2Obst2, vi2->oIdx);
          // debug
/*          printf("UAvoidCellGraph::fillVerticeList: obstacle-crossings has same x-value (%fx,%fy) obst %lu,%d idx %d <==> (%fx,%fy) obst %lu,%d idx %d\n",
                    vi->pos.x, vi->pos.y,
               vi->obst->obst->getSerial(), viObst2, vi->oIdx, vi2->pos.x, vi2->pos.y,
               vi2->obst->obst->getSerial(), vi2Obst2, vi2->oIdx);*/
          // debug end
          vi->extent4thPnt(vi->pos);
          clash = true;
        }
      }
    }
    // debug
    if (debugLog.isOpen())
      fprintf(debugLog.getF(), "- sorted %d (obst %lu idx %d) at (%.4fx,%.4fy) type %d\n", i, vi->obst->obst->getSerial(), vi->oIdx, vi->pos.x, vi->pos.y, vi->type);
    // debug end
  }
  if (clash)
    return -vertsCnt;
  else
    return vertsCnt;
}

////////////////////////////////////////////////////////////////

void UAvoidCellGraph::printVetrexList(FILE * dest)
{
  UAvoidVertexIdx * vi;
  int pt;
  //
  fprintf(dest, "Vertex list sorted in x-order, cnt=%d\n", vertsCnt);
  for (int i = 0; i < vertsCnt; i++)
  {
    int o2 = -1;
    vi = vertsp[i];
    if (vi->obst2 != NULL)
      o2 = vi->obst2->obst->getSerial();
    pt = vi->getPktType();
    if (pt != vi->type)
      fprintf(dest, " * ");
    else
      fprintf(dest, " - ");
    fprintf(dest, "vertex %3d (obst %3lu idx %2d) (and %3d) at (%8.4fx,%8.4fy) type %d(%d) ",
            i, vi->obst->obst->getSerial(), vi->oIdx, o2, vi->pos.x, vi->pos.y, pt, vi->type);
    switch (pt)
    {
      case 0: fprintf(dest,"- split one into two\n"); break;
      case 1: fprintf(dest,"- merge two into one (this and the one below)\n"); break;
      case 2: fprintf(dest,"- top side vertex\n"); break;
      case 3: fprintf(dest,"- bottom side vertex\n"); break;
      case 4: fprintf(dest,"- close dead end\n"); break;
      case 5: fprintf(dest,"- create new in concavity\n"); break;
      case 6: fprintf(dest,"- upper side junction\n"); break;
      case 7: fprintf(dest,"- lower side junction\n"); break;
      default: fprintf(dest,"- unknown cell connection\n"); break;
    }
  }
}

/////////////////////////////////////////////////////////

int UAvoidCellGraph::makeCells(UAvoidVertexIdx ** vertsp, const int vertsCnt, UAvoidCell * cells, bool debugDump)
{
  UAvoidCell *oc, *oc1, *oc1u, *oc1l, *oc2, *oc3;
  double y;
  UAvoidVertexIdx * vt;
  int pt;
  double dy;
  ULogFile * dbl = NULL;
  //
//  if (debugDump)
    dbl = &debugLog;
  //
  vt = vertsp[0];
  oc = getNewCell();
  if (oc == NULL)
  {
    if (debugLog.isOpen())
      fprintf(debugLog.getF(), "UAvoidCellGraph::makeCells: no cells available!\n");
    return 0;
  }
  oc->oPoss[0].set(-1e6, 1e6);
  oc->oPoss[1].set(-1e6, -1e6);
  oc->oPoss[2].set(1e6, 1e6);
  oc->oPoss[3].set(1e6, -1e6);
  oc2 = NULL;
  for (int i= 0; i < vertsCnt; i++)
  {
    vt = vertsp[i];
    //
    if (debugDump and debugLog.isOpen())
    { // old point type calculation
      pt = vt->getPktType();
/*      if (pt < 4)
        fprintf(debugLog.getF(), "\nNew vertex %d pos %.4fx,%.4fy, type %d - obstacle %lu\n", i, vt->pos.x, vt->pos.y, pt, vt->obst->obst->getSerial());
      else
        fprintf(debugLog.getF(), "\nNew vertex %d pos %.4fx,%.4fy, type %d - obstacle %lu and %lu\n",
              i, vt->pos.x, vt->pos.y, pt, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial());*/
    }
    // get point type 0=left extreme, 1 = right extreme, 2 = upper, 3 = lower
    // 4..7 is intersection points:
    // 4 : close,  5 : open,   6 : upper junction,  7 : lower jounction
    pt = vt->type; // vt->getPktType();
    // set obstacle cuts for this x-value
    if (not oc->setCuts(vt->pos.x, dbl))
    { // error in setting of crossing points for open cells - this will probably crash the cell generation
      // bail out
      if (debugLog.isOpen())
      { // find top most open cell.
        oc1 = oc;
        debugLog.toLog("Setting of line-cuts failed");
        fprintf(debugLog.getF(), "Open cells at x-line for vertex %.5fx%.5fy:\n", vt->pos.x, vt->pos.y);
        while (oc1 != NULL)
        {
          oc1->print(debugLog.getF());
          oc1 = oc1->cells[3];
        }
        fprintf(debugLog.getF(), "All cells (total %d):\n", cellsCnt);
        for (int k = 0; k < cellsCnt; k++)
        {
          oc1 = &cells[k];
          oc1->print(debugLog.getF());
        }
      }
      cellsCnt = -1;
      break;
    }
    else if (debugDump and debugLog.isOpen())
    { // list open cells
      oc1 = oc;
      fprintf(debugLog.getF(), "Open cell dump at x-line for vertex %.5fx%.5fy (a %d):\n", vt->pos.x, vt->pos.y, pt);
      while (oc1 != NULL)
      {
        oc1->print(debugLog.getF());
        oc1 = oc1->cells[3];
      }
    }
    // find cell that holds new vertex
    // start from top of open cells
    oc1 = oc;
    oc2 = NULL;
    y = vt->pos.y;
    while (oc1 != NULL)
    { // go down until cell with new point is found
      dy = 1.0;
      switch(pt)
      { // 0 : left extreme (new), 1 : right extreme (join), 2 : upper, 3 : lower
        // 4 : close,  5 : open,   6 : upper junction,  7 : lower jounction
        case 0: // new
          dy = oc1->oPoss[3].y - y;
          //if (dy <= 0.0)
          //  printf("vertex=%d Found a 0 new        cell=%d, obst=%lu\n", i, oc1->id, vt->obst->obst->getSerial());
          break;
        case 1: // join
          if (oc1->obsts[1] == vt->obst)
          { // printf - this is it
            //printf("vertex=%d Found a 1 join cells cell=%d, obst=%lu\n", i, oc1->id, vt->obst->obst->getSerial());
            dy = -1.0;
          }
          break;
        case 2: // upper
          if (oc1->obsts[1] == vt->obst)
          { // printf - this is it
            //printf("vertex=%d Found a 2 upper      cell=%d, obst=%lu\n", i, oc1->id, vt->obst->obst->getSerial());
            dy = -1.0;
          }
          break;
        case 3: // lower
          if (oc1->obsts[0] == vt->obst)
          { // printf - this is it
            //printf("vertex=%d Found a 3 lower      cell=%d, obst=%lu\n", i, oc1->id, vt->obst->obst->getSerial());
            dy = -1.0;
          }
          break;
        case 4: // dead end
          if ((oc1->obsts[1] == vt->obst or  oc1->obsts[0] == vt->obst) and
             (oc1->obsts[1] == vt->obst2 or  oc1->obsts[0] == vt->obst2))
          { // printf - this is it
            //printf("vertex=%d Found a 4 dead end   cell=%d, obst=%lu and %lu\n", i, oc1->id, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial());
            dy = -1.0;
          }
          break;
        case 5: // new in concavity
          dy = oc1->oPoss[3].y - y;
          //if (dy <= 0.0)
          //  printf("vertex=%d Found a 5 new in con cell=%d, obst=%lu and %lu\n", i, oc1->id, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial());
          break;
        case 6: // upper junction
          if (oc1->obsts[1] == vt->obst or  oc1->obsts[1] == vt->obst2)
          { // printf - this is it
            //printf("vertex=%d Found a 6 upper jnk  cell=%d, obst=%lu amd %lu\n", i, oc1->id, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial());
            dy = -1.0;
          }
          break;
        case 7: // upper junction
          if (oc1->obsts[0] == vt->obst or  oc1->obsts[0] == vt->obst2)
          { // printf - this is it
            //printf("vertex=%d Found a 7 lower jnk  cell=%d, obst=%lu and %lu\n", i, oc1->id, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial());
            dy = -1.0;
          }
          break;
        default: break;
      }
      if (dy <= 0.0)
      { // when point is a stub-junction, that do not
        // change cell above line, then the active cell
        // is one further down.
        //if ((pt == 7 or pt == 4) and dy > -0.0001 and (oc1->oPoss[2].y - y > 0.0001))
        //  oc1 = oc1->cells[3];
        break;
      }
      oc1 = oc1->cells[3];
    }
    //
    if (oc1 == NULL)
    {
      if (debugLog.isOpen())
      {
        debugLog.toLog("Error");
        fprintf(debugLog.getF(), "UAvoidCellGraph::makeCells: target cell for vertex %d at %.3fx,%.3fy found no cell?? -- fix error!!\n",
             i, vt->pos.x, vt->pos.y);
        printf("UAvoidCellGraph::makeCells: target cell for vertex %d at %.3fx,%.3fy found no cell?? -- fix error!!\n",
             i, vt->pos.x, vt->pos.y);
      }
      break;
    }
    // adjust cell
    switch (pt)
    {  // pt (point type)
       // 0 : left extreme (new), 1 : right extreme (join), 2 : upper, 3 : lower
       // 4 : close,  5 : open,   6 : upper junction,  7 : lower jounction
    case 1: // merge
    case 2: // upper side vertex
      if (oc1->obsts[1] != vt->obst)
      { // switch to cell above
        //printf("(%d) using cell %d rather than %d\n", pt, oc1->cells[2]->id, oc1->id);
        oc1 = oc1->cells[2];
      }
      break;
    case 3: // lower side vertex
      if (oc1->obsts[0] != vt->obst)
      { // switch to cell below
        //printf("(%d) using cell %d rather than %d\n", pt, oc1->cells[3]->id, oc1->id);
        oc1 = oc1->cells[3];
      }
      break;
    case 4: // closing dead end
      // may be cell above or below
      if (not ((oc1->obsts[1] == vt->obst or oc1->obsts[1] == vt->obst2) and
               (oc1->obsts[0] == vt->obst or oc1->obsts[0] == vt->obst2)))
      { // not the right cell
        if (oc1->obsts[1] == vt->obst or oc1->obsts[1] == vt->obst2)
          // lower obstacle of cell is in commen with vertex
          // so it must be cell below
          oc1 = oc1->cells[3];
        else if (oc1->obsts[0] == vt->obst or oc1->obsts[0] == vt->obst2)
          // then it must be cell above
          oc1 = oc1->cells[2];
        else
        {
          if (debugLog.isLogOpen())
            fprintf(debugLog.getF(), "Type 4->8, a failed dead end, but obstacles do not match: vertex %d: obst %lu and %lu - cell %d has %lu and %lu\n",
                    i, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial(),
                    oc1->id, oc1->obsts[0]->obst->getSerial(), oc1->obsts[1]->obst->getSerial());
          fprintf(stdout, "Type 4->8, a failed dead end, but obstacles do not match: vertex %d: obst %lu and %lu - cell %d has %lu and %lu\n",
                    i, vt->obst->obst->getSerial(), vt->obst2->obst->getSerial(),
                    oc1->id, oc1->obsts[0]->obst->getSerial(), oc1->obsts[1]->obst->getSerial());
          pt = 8;
        } 
        if (oc1->obsts[0] == NULL or oc1->obsts[1] == NULL)
          printf("Error we can not have a dead end without 2 obstacles\n");
      }
      break;
    case 6: // upper side junction
      // may be cell above
      if (not(oc1->obsts[1] == vt->obst or oc1->obsts[1] == vt->obst2))
      { // switch to cell above
        //printf("(%d) using cell %d rather than %d\n", pt, oc1->cells[2]->id, oc1->id);
        oc1 = oc1->cells[2];
      }
      break;
    case 7: // obstacle lower side junction
      // may be the cell below.
      // test if upper side of cell is far from vertex
      if (not (oc1->obsts[0] == vt->obst or oc1->obsts[0] == vt->obst2))
      { // switch to cell below
        //printf("(%d) using cell %d rather than %d\n", pt, oc1->cells[3]->id, oc1->id);
        oc1 = oc1->cells[3];
      }
      break;
    default: // new cell (type 0 or 5) - oc1 can not fail
      break;
    }
    //
    if (oc1 == NULL)
    {
      if (debugLog.isOpen())
        debugLog.toLog("UAvoidCellGraph::makeCells: **** Failed to find relevant cell, this is bad?  ****\n");
      cellsCnt = 0;
      break;
    }
    // some points may have more than one action meaning
    // if the obstacle is a line (or a point or a noVisibility line)
    //printf("--------------------- New vertex %lu:%d point %.4fx,%.4fy\n", vt->obst->obst->getSerial(), vt->oIdx, vt->pos.x, vt->pos.y);
    //{ // get upeer and lower cell
      if (debugDump and debugLog.isOpen())
      {
        if (pt != vt->type)
          printf("--------type conflict %d != %d\n", pt, vt->type);
        fprintf(debugLog.getF(), "-- vertex %d at %.5fx,%.5fy in cell %d type %d(%d), obst %lu ", i, vt->pos.x, vt->pos.y, oc1->id, pt, vt->type, vt->obst->obst->getSerial());
        if (pt >=4)
          fprintf(debugLog.getF(), "and %lu ", vt->obst2->obst->getSerial());
        switch (pt)
        {
          case 0: fprintf(debugLog.getF(), "- split one into two\n"); break;
          case 1: fprintf(debugLog.getF(), "- merge two into one (this and the one below)\n"); break;
          case 2: fprintf(debugLog.getF(), "- top side vertex\n"); break;
          case 3: fprintf(debugLog.getF(), "- bottom side vertex\n"); break;
          case 4: fprintf(debugLog.getF(), "- close dead end\n"); break;
          case 5: fprintf(debugLog.getF(), "- create new in concavity\n"); break;
          case 6: fprintf(debugLog.getF(), "- upper side junction\n"); break;
          case 7: fprintf(debugLog.getF(), "- lower side junction\n"); break;
          case 8: fprintf(debugLog.getF(), "- error - trying to recover\n"); break;
          default: fprintf(debugLog.getF(), "- unknown cell connection\n"); break;
        }
      }
      if (pt < 8)
      {
        // oc1 is the cell to close (if y is within limits)
        oc1u = oc1->cells[2];
        oc1l = oc1->cells[3];
        if (pt != 5)
          // there is a cell to close
          oc1->unlinkOpen();
        switch (pt)
        {
          case 0: // new (first seen) obstacle - split in two
            oc2 = getNewCell();
            oc3 = getNewCell();
            // continue the closed cell into these two open cells
            oc1->cells[2] = oc2;
            oc1->cells[3] = oc3;
            // new open cells
            oc2->link(oc1, NULL, oc1u, oc3, true);
            // associate obstacles above and below cell
            oc2->obsts[0] = oc1->obsts[0];
            oc2->obsts[1] = vt->obst;
            oc3->link(oc1, NULL, oc2, oc1l, true);
            oc3->obsts[0] = vt->obst;
            oc3->obsts[1] = oc1->obsts[1];
            // and left side position
            oc2->oPoss[0] = oc1->oPoss[2];
            oc2->oPoss[1] = vt->pos;
            oc3->oPoss[0] = vt->pos;
            oc3->oPoss[1] = oc1->oPoss[3];
            break;
          case 1: // closes 2 cells and opens one
            oc2 = getNewCell();
            // next cell to close
            oc3 = oc1l;
            // debug
            if (oc3 == NULL)
              printf("Not good!\n");
            // debug end
            oc1l = oc3->cells[3];
            oc3->unlinkOpen();
            // link two closed cells
            oc3->cells[2] = oc2;
            oc3->cells[3] = NULL;
            oc1->cells[2] = oc2;
            oc1->cells[3] = NULL;
            // open new cell
            oc2->link(oc1, oc3, oc1u, oc1l, true);
            oc2->obsts[0] = oc1->obsts[0];
            oc2->obsts[1] = oc3->obsts[1];
            // and left side position
            oc2->oPoss[0] = oc1->oPoss[2];
            oc2->oPoss[1] = oc3->oPoss[3];
            break;
          case 2: // top side vertex
          case 3: // bottom side vertex
            oc2 = getNewCell();
            // continue the closed cell into these two open cells
            oc1->cells[2] = oc2;
            oc1->cells[3] = NULL;
            // open new cell
            oc2->link(oc1, NULL, oc1u, oc1l, true);
            oc2->obsts[0] = oc1->obsts[0];
            oc2->obsts[1] = oc1->obsts[1];
            // and left side position
            oc2->oPoss[0] = oc1->oPoss[2];
            oc2->oPoss[1] = oc1->oPoss[3];
            break;
          case 4: //dead end - just set close position
            oc1->oPoss[2] = vt->pos;
            oc1->oPoss[3] = vt->pos;
            break;
          case 5:
            // a new cell is to be created only (above ((or below)) this oc1)
            //if ((oc1->obsts[0] == vt->obst or oc1->obst[0] == vt.obst2)
              // creating a new graph start point above this cell
              oc1l = oc1;
            //else
              // an opening created by a concavity below this cell (line intersection
            //  oc1u = oc1;
            //oc1 = NULL; // no valid just closed cell
            oc2 = getNewCell();
            // link into open cells
            oc2->link(NULL, NULL, oc1u, oc1l, true);
            // set opstacles and positions
            oc2->obsts[0] = vt->getUpper();
            oc2->obsts[1] = vt->getOther(oc2->obsts[0]);
            oc2->oPoss[0] = vt->pos;
            oc2->oPoss[1] = vt->pos;
            break;
          case 6: // upper side of junction
            oc2 = getNewCell();
            // link in closed cell
            oc1->cells[2] = oc2;
            oc1->cells[3] = NULL;
            // open new cell
            oc2->link(oc1, NULL, oc1u, oc1l, true);
            oc2->obsts[0] = oc1->obsts[0];
            if (oc1->obsts[1] == vt->obst)
              oc2->obsts[1] = vt->obst2;
            else
              oc2->obsts[1] = vt->obst;
            // and left side position
            oc2->oPoss[0] = oc1->oPoss[2];
            oc2->oPoss[1] = oc1->oPoss[3];
            break;
          case 7: // lower side of junction
            oc2 = getNewCell();
            // link in closed cell
            oc1->cells[2] = oc2;
            oc1->cells[3] = NULL;
            // open new cell
            oc2->link(oc1, NULL, oc1u, oc1l, true);
            oc2->obsts[1] = oc1->obsts[1];
            if (oc1->obsts[0] == vt->obst)
              oc2->obsts[0] = vt->obst2;
            else
              oc2->obsts[0] = vt->obst;
            // and left side position
            oc2->oPoss[0] = oc1->oPoss[2];
            oc2->oPoss[1] = oc1->oPoss[3];
            // test for continued use of vertex point
            break;
          default:
            break;
        }
      }
      // debug
//      printf("------ Cell generation %d at vertex %lu:%d point %gx,%gy vertex type %d\n", i, vt->obst->obst->getSerial(), vt->oIdx, vt->pos.x, vt->pos.y, pt);
/*      for (int k=0; k < cellsCnt; k++)
        cells[k].print(); */
/*      if (oc2 != NULL)
      { // find top most open cell.
        oc = oc2;
        while (oc->cells[2] != NULL)
          oc = oc->cells[2];
      }*/
      // debug
/*      if (oc1 != NULL)
        printf("  - open cells (oc1 cell is %d):\n", oc1->id);
      else
        printf("  - open cells (oc1 == NULL):\n");
      oc3 = oc;
      while (oc3 != NULL)
      {
        oc3->print();
        oc3 = oc3->cells[3];
      }*/
      // debug end
//    }
    // this is needed, as top cell may have been closed
    // and oc2 is always a newly created open cell.
    if (oc2 != NULL)
    { // find top most open cell.
      oc = oc2;
      while (oc->cells[2] != NULL)
        oc = oc->cells[2];
    }
  }
  // last cell should be closed already
  if (oc2 != NULL and cellsCnt >= 0)
  { // set to free space corners
    oc2->oPoss[2].set(1e6, 1e6);
    oc2->oPoss[3].set(1e6, -1e6);
  }
  //
  return cellsCnt;
}

///////////////////////////////

bool UAvoidCellGraph::getCellPoly(int idx, UPolygon * poly, double maxBoxEdgeValue)
{
//  const int EDGE_BOX_OF_POLYGON = 100; // meters away from odo-origo, x and y
  bool isOK;
  UAvoidCell * cell;
  //
  isOK = idx >= 0 and idx < cellsCnt;
  if (isOK)
  {
    cell = &cells[idx];
    poly->clear();
    cell->getPoly(poly, maxBoxEdgeValue);
  }
  return isOK;
}

///////////////////////////////

int UAvoidCellGraph::getCellPathPoly(int idx, UPolygon * poly, double maxBoxEdgeValue)
{
  bool isOK;
  UAvoidCell * cell;
  int n = -1;
//  const int EDGE_BOX_OF_POLYGON = 100; // meters away from odo-origo, x and y
  //
  isOK = idx >= 0 and idx < pacsCnt;
  if (isOK)
  {
    cell = pacs[idx];
    n = cell->id;
    poly->clear();
    cell->getPoly(poly, maxBoxEdgeValue);
  }
  return n;
}

///////////////////////////////////

UAvoidCell * UAvoidCellGraph::getNewCell()
{
  UAvoidCell * result = NULL;
  if (cellsCnt >= cellsCntMax)
    printf("UAvoidCellGraph::getNewCell: cell overflow\n");
  else
  {
    result = &cells[cellsCnt++];
    result->init();
  }
  return result;
}

/////////////////////////////////////////////////

UAvoidCell * UAvoidCellGraph::findCell(UPose pos)
{
  UAvoidCell * result = NULL, *c;
  U2Dlined lt, lb;
  double dt, db;
  //
  for (int i= 0; i < cellsCnt; i++)
  {
    c = &cells[i];
    if (pos.x >= c->oPoss[0].x and pos.x < c->oPoss[2].x)
    { // is in the right x-segment
      // top line of cell
      lt.set2P(c->oPoss[0].x, c->oPoss[0].y, c->oPoss[2].x, c->oPoss[2].y);
      dt = lt.distanceSigned(pos.x, pos.y);
      // bottom line of cell
      lb.set2P(c->oPoss[1].x, c->oPoss[1].y, c->oPoss[3].x, c->oPoss[3].y);
      db = lb.distanceSigned(pos.x, pos.y);
      if (dt * db <= 0.0)
      {
        result = c;
        break;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////

int UAvoidCellGraph::findCellPath(UPose pStart, UPose pEnd, UAvoidVertexCosts * costAdd)
{ // make an A-star search into the cell graph
  UAvoidCell * cStart, *cEnd, *xoc, *yoc;
  int cOpenCnt = 0, cClosedCnt = 0;
  class UOpenCell
  { // structure for open and closed cells
    public:
      double calcGScoreTo(UAvoidCell * yoc, double * nx, double * ny, UAvoidVertexCosts * costAdd)
      {
        double d, dy, dx, thisx, nextx, prevx;
        bool fwd, sameDir;
        int avoidLeft = -1;
        static const double IS_NEAR = 0.01;
        // centre of cell (x-vise)
        nextx = (yoc->oPoss[0].x + yoc->oPoss[2].x) / 2.0;
        thisx = (cell->oPoss[0].x + cell->oPoss[2].x) / 2.0;
        // 'fwd' means now going in the positive x direction
        fwd = nextx > thisx;
        if (fwd)
          *nx = yoc->oPoss[0].x;
        else
          *nx = yoc->oPoss[2].x;
        if (came_from == NULL)
          sameDir = true;
        else
        { // may have changed direction around an obstacle
          prevx = (came_from->cell->oPoss[0].x + came_from->cell->oPoss[2].x) / 2.0;
          sameDir = (fwd == (thisx > prevx));
        }
        if (sameDir)
        { // continue in same direction
          *ny = y;
          if (fwd)
          { // is y-value changed
            if (y < fmax(cell->oPoss[3].y, yoc->oPoss[1].y) + IS_NEAR) // fmax(yoc->oPoss[3].y, yoc->oPoss[1].y))
            {
              *ny = fmax(cell->oPoss[3].y, yoc->oPoss[1].y);
              avoidLeft = true;
            }
            else if (y > fmin(cell->oPoss[2].y, yoc->oPoss[0].y) - IS_NEAR) // fmin(yoc->oPoss[2].y, yoc->oPoss[0].y))
            {
              *ny = fmin(cell->oPoss[2].y, yoc->oPoss[0].y);
              avoidLeft = false;
            }
          }
          else
          { // backwards x is decreasing
            // is y-value changed by new cell
            if (y < fmax(cell->oPoss[1].y, yoc->oPoss[3].y) + IS_NEAR) // fmax(yoc->oPoss[1].y, yoc->oPoss[3].y))
            {
              *ny = fmax(cell->oPoss[1].y, yoc->oPoss[3].y);
              avoidLeft = false;
            }
            else if (y > fmin(cell->oPoss[0].y, yoc->oPoss[2].y) - IS_NEAR) // fmin(yoc->oPoss[0].y, yoc->oPoss[2].y))
            {
              *ny = fmin(cell->oPoss[0].y, yoc->oPoss[2].y);
              avoidLeft = true;
            }
          }
          dy = fabs(*ny - y);
          dx = fabs(*nx - x);
        }
        else
        { // changing direction around an obstacle
          dy = came_from->y;
          if (fwd)
          { // to the closest corner at the leftmost side of new cell
            avoidLeft = (fabs(dy - yoc->oPoss[0].y) > fabs(dy - yoc->oPoss[1].y));
            if (avoidLeft)
              *ny = yoc->oPoss[1].y;
            else
              *ny = yoc->oPoss[0].y;
            dy = fabs(dy - *ny);
            // dx is back and fwd from old x to this x to new x
            dx = (came_from->x - cell->oPoss[2].x) + (*nx - cell->oPoss[2].x);
          }
          else
          { // to the closest cornet of the rightmost side of new cell
            avoidLeft = (fabs(dy - yoc->oPoss[2].y) < fabs(dy - yoc->oPoss[3].y));
            if (avoidLeft)
              *ny = yoc->oPoss[2].y;
            else
              *ny = yoc->oPoss[3].y;
            dy = fabs(dy - *ny);
            // dx is forward and baxk from old x to this x to new x
            dx = (cell->oPoss[0].x - came_from->x); // + (cell->oPoss[0].x - *nx);
          }
        }
        d = g_score + hypot(dx, dy);
        if (avoidLeft >= 0)
        { // we are turning a corner - so we may have been here
          // before with a bad result
          U2Dpos p;
          p.set(*nx, *ny);
          d += costAdd->getExtraCost(p, avoidLeft);
        }
        return d;
      }
    public:
      UAvoidCell * cell;
      double g_score, h_score, f_score;
      UOpenCell * came_from;
      double x, y;
      int posIdx;
  };
  UOpenCell *cOpen, *cClosed, *xo, *yo, sw, *tmpo;
  bool inSet, tentative_is_better;
  double nodeX, nodeY; // tentative node position
  //
  cStart = findCell(pStart);
  cEnd = findCell(pEnd);
  if (cStart != NULL and cEnd != NULL)
  { // make an A-star search into the cell graph
    // A(start,goal)
    cOpen = (UOpenCell*)malloc(cellsCnt * sizeof(UOpenCell)); // new UOpenCell[cellsCnt];
    cClosed = (UOpenCell*)malloc(cellsCnt * sizeof(UOpenCell)); //new UOpenCell[cellsCnt];
    //      closedset := the empty set                 // The set of nodes already evaluated.
    pavsCnt = 0;
    //      openset := set containing the initial node // The set of tentative nodes to be evaluated.
    xo = &cOpen[cOpenCnt++];
    xo->cell = cStart;
    //      g_score[start] := 0                        // Distance from start along optimal path.
    xo->g_score = 0.0;
    //      h_score[start] := heuristic_estimate_of_distance(start, goal)
    xo->h_score = pStart.getDistance(pEnd);
    //      f_score[start] := h_score[start]           // Estimated total distance from start to goal through y.
    xo->f_score = xo->h_score;
    xo->x = pStart.x;
    xo->y = pStart.y;
    xo->came_from = NULL;
    //      while openset is not empty
    while (cOpenCnt > 0)
    { //          x := the node in openset having the lowest f_score[] value
      // is always sorted in f_score order, so index 0 is est.
      xo = &cOpen[0];
      xoc = xo->cell;
      //          if x = goal
      //              return reconstruct_path(came_from[goal])
      if (xoc == cEnd)
      {
        // debug
        //printf("A* found end cell %d (remaining open cells=%d - ends at %gx,%gy\n", xo->cell->id, cOpenCnt, xo->x, xo->y);
        // debug end
        break;
      }
      // debug
      //printf("A* testing  cell %d (remaining open cells=%d - ends at %gx,%gy\n", xo->cell->id, cOpenCnt, xo->x, xo->y);
      // debug end
      //          remove x from openset
      //          add x to closedset
      cOpenCnt--;
      cClosed[cClosedCnt] = cOpen[0];
      xo = &cClosed[cClosedCnt++];
      memmove(cOpen, &cOpen[1], cOpenCnt * sizeof(UOpenCell));
      //          foreach y in neighbor_nodes(x)
      for (int i = 0; i < 4; i++)
      {
        yoc = xoc->cells[i];
        if (yoc == NULL)
          continue;
        //              if y in closedset
        //                  continue
        inSet = false;
        for (int j = cClosedCnt - 1; j >= 0; j--)
        {
          if (cClosed[j].cell == yoc)
          {
            inSet = true;
            break;
          }
        }
        if (inSet)
          continue;
        //              tentative_g_score := g_score[x] + dist_between(x,y)
        double tentative_g_score;
/*        if (yoc == cEnd)
        { // end cell, so we shall not pass
          //nodeX = pEnd.x;
          //nodeY = pEnd.y;
          tentative_g_score = xo->calcGScoreTo(yoc, &nodeX, &nodeY, costAdd);
          // add distance in last cell (yoc)
          //tentative_g_score = xo->g_score + hypot(nodeY - pEnd.y, nodeX - pEnd.x);
        }
        else*/
          // we need to pass this cell (too)
          tentative_g_score = xo->calcGScoreTo(yoc, &nodeX, &nodeY, costAdd);
        //
//        for (int c = 0; c < costsCnt; c++)
        if (false)
        {
          if (i < 2)
          { // mooving backwards (in x) so moving into segment from position 2 to 3 in new cell
/*            if (costs[c].avoidLeft and costs[c].pos.dist(yoc->oPoss[2]) < 0.01)
              tentative_g_score += costs[c].cost;
            if (not costs[c].avoidLeft and costs[c].pos.dist(yoc->oPoss[3]) < 0.01)
              tentative_g_score += costs[c].cost;*/
            tentative_g_score += costAdd->getExtraCost(yoc->oPoss[2], true);
            tentative_g_score += costAdd->getExtraCost(yoc->oPoss[3], false);
          }
          else
          { // mooving forward (in x) so moving into segment from position 0 to 1 in new cell
/*            if (not costs[c].avoidLeft and costs[c].pos.dist(yoc->oPoss[0]) < 0.01)
              tentative_g_score += costs[c].cost;
            if (costs[c].avoidLeft and costs[c].pos.dist(yoc->oPoss[1]) < 0.01)
              tentative_g_score += costs[c].cost;*/
            tentative_g_score += costAdd->getExtraCost(yoc->oPoss[0], false);
            tentative_g_score += costAdd->getExtraCost(yoc->oPoss[1], true);
          }
        }
        //
        inSet = false;
        for (int j = 0; j < cOpenCnt; j++)
        {
          if (cOpen[j].cell == yoc)
          {
            yo = &cOpen[j];
            inSet = true;
            break;
          }
        }
        if (not inSet)
        { //              if y not in openset
          //                  add y to openset
          //                  tentative_is_better := true
          yo = &cOpen[cOpenCnt++];
          yo->cell = yoc;
          yo->posIdx = -1;
          tentative_is_better = true; // the only one
//          printf("opened  ");
        }
        else if (tentative_g_score < yo->g_score)
        { //              elseif tentative_g_score < g_score[y]
          //                  tentative_is_better := true
          tentative_is_better = true;
//          printf("Updated ");
        }
        else
          //              else
          //                  tentative_is_better := false
          tentative_is_better = false;
        if (tentative_is_better)
        { //              if tentative_is_better = true
          //                  came_from[y] := x
          yo->x = nodeX;
          yo->y = nodeY;
          yo->came_from = xo;
          //                  g_score[y] := tentative_g_score
/*          printf("cell %d to new %7.3fx, %7.3fy - gScore from %.3f to %.3f\n",
                 yo->cell->id, nodeX, nodeY, yo->g_score, tentative_g_score);*/
          yo->g_score = tentative_g_score;
          //                  h_score[y] := heuristic_estimate_of_distance(y, goal)
          yo->h_score = hypot(nodeY - pEnd.y, nodeX - pEnd.x);
          //                  f_score[y] := g_score[y] + h_score[y]
          yo->f_score = yo->g_score + yo->h_score;
          //
          // sort openSet after f_value
          tmpo = yo;
          tmpo--;
          while (tmpo >= cOpen)
          { // is this (yo) worse than this entry in priority list 
            if (yo->f_score > tmpo->f_score)
              // yes it is worse - do not go any further
              break;
            tmpo--;
          }
          // up one to the new position og yo
          tmpo++;
          if (tmpo != yo)
          { // priorities has changed - move part of list 1 entry down
            // save the yo entry
            sw = *yo;
            // move
            memmove(&tmpo[1], tmpo, (yo - tmpo) * sizeof(UOpenCell));
            // insert yo in its rightfull place 
            *tmpo = sw;
          }
        }
      }
/*      for (int n = 0; n < cOpenCnt; n++)
      {
        yo = &cOpen[n];
        printf("++ open   cell %3d from %3d at %7.3fx,%7.3fy - score %7.3ff, %7.3fg, %7.3fh\n",
               yo->cell->id, yo->came_from->cell->id, yo->x, yo->y, yo->f_score, yo->g_score, yo->h_score);
      }
      for (int n = 0; n < cClosedCnt; n++)
      {
        int fid = -1;
        yo = &cClosed[n];
        if (yo->came_from != NULL)
          fid = yo->came_from->cell->id;
        printf("## closed cell %3d from %3d at %7.3fx,%7.3fy - score %7.3ff, %7.3fg, %7.3fh\n",
               yo->cell->id, fid, yo->x, yo->y, yo->f_score, yo->g_score, yo->h_score);
      }
      printf("\n");*/
    }
    // 
    if (xoc == cEnd)
    { // reconstruct_path(current_node)
      // debug
      //printf("UAvoidCellGraph::findCellPath: Found cell order (back to start):\n");
      //printf(" - cell %d at %.3fx,%.3fy\n", cEnd->id, pEnd.x, pEnd.y);
      // debug end
      tmpo = xo;
      pacsCnt = 0;
      // debug
      polyCell.clear();
      polyCell.setAsPolyline();
      polyCell.add(pEnd.x, pEnd.y);
      // debug end
      // count cells in found path
      while (tmpo != NULL)
      { //      if came_from[current_node] is set
        //          p = reconstruct_path(came_from[current_node])
        //          return (p + current_node)
        //      else
        //          return current_node
        // debug
        //printf(" - cell %d at %.3fx,%.3fy score %3f\n", tmpo->cell->id, tmpo->x, tmpo->y, tmpo->g_score);
        polyCell.add(tmpo->x, tmpo->y);
        // debug end
        tmpo = tmpo->came_from;
        pacsCnt++;
      }
      // save path in pacs array
      tmpo = xo;
      for (int i = pacsCnt - 1; i >= 0; i--)
      { // save path of cells
        pacs[i] = tmpo->cell;
        tmpo = tmpo->came_from;
      }
    }
    else
    { //      return failure
      printf("UAvoidCellGraph::findCellPath: No solution found\n");
      pavsCnt = -1;
    }
    free(cOpen);
    free(cClosed);
/*    delete cOpen;
    delete cClosed;*/
  }
  else
  {
    pavsCnt = 0;
    pacsCnt = 0;
  }
  return pavsCnt;
}

///////////////////////////////////////////////////

bool UAvoidCellGraph::findCellPointPath(UPose pStart, UPose pEnd)
{
  bool finished = false;
  UAvoidCell *c1, *c2;
  UAvoidCellVertex *cv, *cv1, *cv2;
  U2Dpos p1, p2, x1, x2;
  U2Dseg l1, l2;
  U2Dpos yTop, yBot;
  int v1, v2;
  double d, dMaxP, dMaxN;
  int dMaxPi, dMaxNi;
  bool fwdX;
  //
  // fill cell list
  // add start point
  for (int i = 0; i < pacsCnt - 1; i++)
  {
    cv = &pavs[i];
    c1 = pacs[i];
    c2 = pacs[i+1];
    if (c2 != NULL)
      cv->nextIdx = c1->getCellIdx(c2->id);
    else
      cv->nextIdx = -1;
    // set exit segments (pointing down)
    if (cv->isFwd())
    { // going positive x direction, so exit at position 2-3
      if (c1->cells[3] == NULL)
        // this cell has the most narrow exit
        cv->exitSeg.setFromPoints(c1->oPoss[2], c1->oPoss[3]);
      else
        // next cell has the most narrow entry
        cv->exitSeg.setFromPoints(c2->oPoss[0], c2->oPoss[1]);
    }
    else
    { // going in negative x direction. so exit is position 0-1
      if (c1->cells[1] == NULL)
        // this cell has most narrow exit
        cv->exitSeg.setFromPoints(c1->oPoss[0], c1->oPoss[1]);
      else
        // next cell has most narrow entry
        cv->exitSeg.setFromPoints(c2->oPoss[2], c2->oPoss[3]);
    }
    cv->finished = false;
    cv->obst = NULL;
    // debug
/*    x1 = cv->exitSeg.getFirstEnd();
    x2 = cv->exitSeg.getOtherEnd();
    printf("exit segment seq %d cellID %d from %gx,%gy to %gx,%gy\n",
           i, c1->id, x1.x, x1.y, x2.x, x2.y);*/
    // debug end
  }
  pavsCnt = pacsCnt;
  // last cell ends in exit position in same direction as previous cell
  if (pavsCnt > 0)
  {
    cv = &pavs[pavsCnt - 1];
    cv->pos.set(pEnd.x, pEnd.y, 0.0);
    // set exit segment to zero length at end position
    cv->exitSeg.setFromPose(pEnd.x, pEnd.y, -M_PI/2.0, 0.0);
    cv->finished = false;
    cv->nextIdx = -1;
    cv->obst = NULL;
  }
  //
  // set finished cells - either reversing direction or clear direction change
  p1.set(pStart.x, pStart.y);
  finished = true;
  for (int i = 0; i < pavsCnt - 1; i++)
  {
    c1 = pacs[i];
    c2 = pacs[i + 1];
    cv1 = &pavs[i];
    cv2 = &pavs[i + 1];
    if (cv2->finished)
    { // if noone earlier passes open segments,
      // then we are finished.
      if (finished)
        cv1->finished = true;
    }
    else if (cv1->isFwd() == cv2->isFwd() or cv2 == cv)
    { // continue in same direction (and is not the last cell)
      // may be a finished cell if a line to ends of next exit segment
      // both cross exit segment of this cell above or below.
      l2 = cv1->exitSeg;
      l1.setFromPoints(p1, cv2->exitSeg.getFirstEnd());
      l1.getCrossing(l2, &x1.x, &x1.y);
      l1.setFromPoints(p1, cv2->exitSeg.getOtherEnd());
      l1.getCrossing(l2, &x2.x, &x2.y);
      yTop = l2.getFirstEnd();
      yBot = l2.getOtherEnd();
      if (x1.y >= yTop.y and x2.y >= yTop.y)
      { // both above, so fix this passage to the upper point
        p1 = yTop;
        cv1->pos = yTop;
        cv1->obst = c1->obsts[0];
        cv1->finished = true;
      }
      else if (x1.y <= yBot.y and x2.y <= yBot.y)
      { // both above, so fix this passage to the upper point
        p1 = yBot;
        cv1->pos = yBot;
        cv1->obst = c1->obsts[1];
        cv1->finished = true;
      }
      else
        finished = false;
      if (cv2 == cv)
        // the last, so mark end cell as finished too
        cv2->finished = true;
    }
    else
    { // changing direction, so avoid point exist
      if (cv1->isFwd())
      { // going from forward x to backwards x
        if (cv2->nextIdx == 0)
        { // from lower to upper y-value
          cv1->obst = c1->obsts[0];
          cv1->pos = c1->oPoss[2];
          cv1->avoidLeft = false;
        }
        else
        { // from upper y to lower y
          cv1->obst = c1->obsts[1];
          cv1->pos = c1->oPoss[3];
          cv1->avoidLeft = true;
        }
      }
      else
      { // going from backwards x to forward x
        if (cv2->nextIdx == 2)
        { // from lower to upper y-value
          cv1->obst = c1->obsts[0];
          cv1->pos = c1->oPoss[0];
          cv1->avoidLeft = true;
        }
        else
        { // from upper y to lower y
          cv1->obst = c1->obsts[1];
          cv1->pos = c1->oPoss[1];
          cv1->avoidLeft = false;
        }
      }
      cv1->finished = true;
    }
    // debug
    //printf("vertex %d position %7.2fx,%7.2fy avoidLeft %s is finished %s\n",
    //       i, cv1->pos.x, cv1->pos.y, bool2str(cv1->avoidLeft), bool2str(cv1->finished));
    // debug end
  }
  // debug
  //cv1 = &pavs[pavsCnt - 1];
  //printf("vertex %d position %7.2fx,%7.2fy avoidLeft %s is finished %s - last\n",
  //          pavsCnt - 1, cv1->pos.x, cv1->pos.y, bool2str(cv1->avoidLeft), bool2str(cv1->finished));
  
  // debug end
  // the remaining unfinished cells need a multicell check
  v1 = -1;
  p1.set(pStart.x, pStart.y);
  while (not finished)
  {
    for (v2 = v1 + 1; v2 < pacsCnt - 1; v2++)
    { //
      if (pavs[v2].finished)
        break;
    }
    if (v2 == pacsCnt - 1)
      // last position
      p2.set(pEnd.x, pEnd.y);
    else
      p2 = pavs[v2].pos;
    // find vertex points to avoid
    // in cell v1+1 to v2-1
    // connecting point p1 to p2
    if (v2 > v1 + 1)
    { // there is passages to settle
      // set line from known start to known end in positive x-direction
      fwdX = (p2.x > p1.x);
      if (fwdX)
        l1.setFromPoints(p1, p2);
      else
        l1.setFromPoints(p2, p1);
      // find most positive and most negative
      // vertex in the range
      dMaxP =  1e-5; // positive - less rounding errors
      dMaxN = -1e-5; // negative - less rounding errors
      dMaxPi = -1; 
      dMaxNi = -1;
      for (int i = v1 + 1; i < v2; i++)
      {
        cv1 = &pavs[i];
        x1 = cv1->exitSeg.getFirstEnd();
        d = l1.distanceSigned(x1.x, x1.y);
        if (d < dMaxN)
        {
          dMaxN = d;
          dMaxNi = i;
        }
        x2 = cv1->exitSeg.getOtherEnd();
        d = l1.distanceSigned(x2.x, x2.y);
        if (d > dMaxP)
        {
          dMaxP = d;
          dMaxPi = i;
        }
      }
      if (dMaxP > -dMaxN and dMaxPi >= 0)
      { // Positive (positive Y compared to direct line) is most problematic,
        // so add as new vertex point
        cv2 = &pavs[dMaxPi];
        c1 = pacs[dMaxPi];
        if (cv2->nextIdx == 0)
        { // going in negative x direction - firnd most problematic obstacle, either in this or in next cell
          if (c1->oPoss[1].y > c1->cells[0]->oPoss[3].y)
            // this cell lower obstacle is most problematic
            cv2->obst = c1->obsts[1];
          else
            // next cell (less x) is worse
            cv2->obst = c1->cells[0]->obsts[1];
        }
        else if (cv2->nextIdx == 2)
        { // going in positive x direction 
          if (c1->oPoss[3].y > c1->cells[2]->oPoss[1].y)
            // this cell lower obstacle is most problematic
            cv2->obst = c1->obsts[1];
          else
            // next cell is worse
            cv2->obst = c1->cells[2]->obsts[1];
        }
        else
          // next cell share lower obstacle, so use lower obstacle in this cell
          cv2->obst = c1->obsts[1];
        cv2->pos = cv2->exitSeg.getOtherEnd();
        cv2->finished = true;
        cv2->avoidLeft = fwdX;
      }
      else if (dMaxNi >= 0)
      { // Negative (negative Y compared to direct line) is most problematic, add as new vertex point
        cv2 = &pavs[dMaxNi];
        c1 = pacs[dMaxNi];
        if (cv2->nextIdx == 1 or cv2->nextIdx == 3)
          // this(c1) is a merger or is split and we go the cell with lower y, so use next cells upper obstacle
          cv2->obst = c1->cells[cv2->nextIdx]->obsts[0];
        else
          // next cell share upper obstacle, so use upper obstacle in this cell
          cv2->obst = c1->obsts[0];
        cv2->pos = cv2->exitSeg.getFirstEnd();
        cv2->finished = true;
        cv2->avoidLeft = not fwdX;
      }
      else
      { // no needed vertex points between v1 and v2
        for (int i = v1 + 1; i < v2; i++)
          pavs[i].finished = true;
      }
    }
    // advance to next
    while (pavs[v1 + 1].finished and v1 < pavsCnt)
      v1++;
    finished = (v1 >= pavsCnt - 1);
    if (not finished and v1 >= 0)
    {
      p1 = pavs[v1].pos;
    }
  }
  // debug
/*  printf("Final vertex list\n");
  for (int i = 0; i < pavsCnt; i++)
  {
    cv1 = &pavs[i];
    if (cv1->obst == NULL)
      v1 = -1;
    else
      v1 = cv1->obst->obst->getSerial();
    printf("vertex %d position %7.2fx,%7.2fy (obst %d) avoidLeft %s is finished %s\n",
           i, cv1->pos.x, cv1->pos.y, v1,
           bool2str(cv1->avoidLeft), bool2str(cv1->finished));
  }*/
  // debug end
  return finished;
}


