/** *************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
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
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <ugen4/uline.h>
#include <umap4/umanarc.h>

#include "uavoidpath2.h"
#include "uavoidnovis.h"

#include "ureactivepath.h"
#include "uavoidcellgraph.h"


/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////

UAvoidPath2::UAvoidPath2()
{
  int i;
  //
  pathUsed = false;
  valid = false;
  crash = false;
  manFull = NULL;
  par = NULL;
  startExitObst[0] = new UObstacle();
  startExitObst[0]->setSerial(31001);
  startExitObst[1] = new UObstacle();
  startExitObst[1]->setSerial(31002);
  aPointRoot = NULL;
  pointListFinal = NULL;
  avoidSerial = -1;
  aLnkLast = NULL;
  aLnkRoot = NULL;
  aogLast = NULL;
  aogRoot = NULL;
  aogsCnt = 0;
  lnkSeqRoot = NULL;
  lnkSeqLast = NULL;
  lnkSeqsCnt = 0;
  logdbg = NULL;
  // debug
  // logdbg = fopen("avoid-dbg.log", "w");
  // debug end
  for (i = 0; i < MAX_FOODPRINT_POLYS; i++)
    polys[i] = NULL;
  polysCnt = 0;
  oldFootCnt = 0;
  lnkSerial = 0;
  for (i = 0; i < MAX_LOCAL_OBSTS; i++)
    aogsObst[i] = NULL;
  aogsObstCnt = 0;
  avCellGraph = NULL;
  solutionCnt = 0;
  poolIdx = 0;
}

////////////////////////////////////////

UAvoidPath2::~UAvoidPath2()
{
  int i;
  //
  if (manFull != NULL)
    delete manFull;
  delete startExitObst[0];
  delete startExitObst[1];
  while (aogRoot != NULL)
  {
    aogLast = aogRoot;
    aogRoot = aogRoot->empty;
    delete aogLast;
  }
  aogLast = NULL;
  while (aLnkRoot != NULL)
  {
    aLnkLast = aLnkRoot;
    aLnkRoot = aLnkRoot->empty;
    delete aLnkLast;
  }
  aLnkLast = NULL;
  while (lnkSeqRoot != NULL)
  {
    lnkSeqLast = lnkSeqRoot;
    lnkSeqRoot = lnkSeqRoot->next;
    delete lnkSeqLast;
  }
  lnkSeqLast = NULL;
  for (i = 0; i < MAX_FOODPRINT_POLYS; i++)
  {
    if (polys[i] != NULL)
      delete polys[i];
    else
      break;
  }
  for (i = 0; i < MAX_LOCAL_OBSTS; i++)
  {
    if (aogsObst[i] != NULL)
      delete aogsObst[i];
    else
      break;
  }
  if (logdbg != NULL)
    fclose(logdbg);
  if (avCellGraph != NULL)
    delete avCellGraph;
}

////////////////////////////////////////

void UAvoidPath2::clear()
{
  // debug
/*  const int MSL = 100;
  char s[MSL];
  if (logdbg == NULL)
  {
    snprintf(s, MSL, "avoid-dbg%d.log", poolIdx);
    logdbg = fopen(s, "w");
    if (logdbg != NULL)
      printf("**** NB! UAvoidPath2::UAvoidPath2() DEBUG TO FILE %s is open (much data)!\n", s);
  }*/
  if (logdbg != NULL)
    fprintf(logdbg, "**** UAvoidPath2::clear()\n");
  // debug end
  pathUsed = false;
  valid = false;
  crash = false;
  if (manFull == NULL)
    manFull = new UManSeq();
  else
    manFull->releaseAllMan();
  openSet = NULL;
  closedSet = NULL;
}

////////////////////////////////////////

double UAvoidPath2::getManTime()
{
  double result = 0.0;
  if (manFull != NULL)
    result = manFull->getTime();
  return result;
}

////////////////////////////////////////

void UAvoidPath2::findObstAvoidRoute(bool debugDump)
{
  ULineSegment dLine;
  UPosition pos1, pos2;
  double manLength;
  //double speed;
  UTime t, t2;
  int cnt1, cnt2, cnt3, cnt4;
  double minPassableDistance;
  double dist, minTurnRadius;
  int i;
  // debug
  //if (logdbg == NULL)
  //  logdbg = fopen("avoid-dbg.log", "w");
  if (logdbg != NULL)
    printf("**** NB! UAvoidPath2::UAvoidPath2() DEBUG TO FILE ./avoid-dbg.log is open (much data)!\n");
  // debug end
  t.now();
  // Clear old results;
  aogLast = aogRoot;
  aogsCnt = 0;
  aLnkLast = aLnkRoot;
  avoidSerial = par->avoidSerial;
  // get start and end poistion
  pos1.set(par->startPose.x, par->startPose.y, 0.0);
  pos2.set(par->exitPose.x, par->exitPose.y, 0.0);
  // and a line from start to exit
  dLine.setFromPoints(pos1, pos2);
  // get manoeuvre margin - based on turn radius and robot size
  //speed = maxd(par->startPose.getVel(), par->exitPose.getVel());
//  manWidth = sqr(speed) / par->maxTurnAcceleration;
//  freeLeft = par->frontLeft.y + manWidth;
//  freeRight = -par->frontRight.y + manWidth;
  // group all obstacles based on obstacle separation and robot width
  minPassableDistance = par->getMinOpening();
  groupObstacles(minPassableDistance);
  if (logdbg != NULL)
  {
    fprintf(logdbg, "\n******* Created groups\n");
    logObstacleGroups();
  }
  // test for obstacles that embed start and exit pose
  invalidateObstaclesAtStartAndExit();



  if (par->cellRev2)
  { // use absolute cell decomposition
    //int n;
    //
    printf("UAvoidPath2::findObstAvoidRoute: trying the trapez cell solution.\n");
    t2.now();
    addNoVisLinesAsObstacles(aogs, aogsCnt);

    // debug
/*    for (int g = 0; g < aogsCnt; g++)
    {
      UAvoidObst * og = aogs[g];
      while (og != NULL)
      {
        printf("Obst %d.%lu (%d):", g, og->obst->getSerial(), og->obst->getPointsCnt());
        for (int v = 0; v < og->obst->getPointsCnt(); v++)
          printf(" %.4fx,%.4fy;", og->obst->getPoint(v).x, og->obst->getPoint(v).y);
        printf("\n");
        og = og->grp;
      }
    }*/
    // debug end
    
    printf("UAvoidPath2::findObstAvoidRoute: finished addNoVisLinesAsObstacles after %.5f sec.\n", t2.getTimePassed());
    if (avCellGraph == NULL)
      avCellGraph = new UAvoidCellGraph();
    // make cells
    avCellGraph->makeCellDecomposition(aogs, aogsCnt, par, debugDump);
    printf("UAvoidPath2::findObstAvoidRoute: finished the trapez cell decomposition (%d cells) after %.5f sec\n", avCellGraph->getCellsCnt(), t2.getTimePassed());
    // add start and exit (as obstacles) - to associate waypoints and obstacles
    addStartAndExit();
    // clear extra cost list
    costAdd.costsCnt = 0;
    // try up to three path generations
    for (int a = 0; a < 3; a++)
    {
      // find best path in cells
      avCellGraph->findBestCellPath(par, &costAdd, debugDump);
      // create avoid point list
      if (pointListFinal != NULL)
        recyclePoints(pointListFinal);
      // make point list - including start and exit
      generation = 0;
      pointListFinal = createCellBasedPointList(avCellGraph->getVertexList(), avCellGraph->getVertexListCnt());
      printf("UAvoidPath2::findObstAvoidRoute: finished createCellBasedPointList after %.5f sec\n", t2.getTimePassed());
      // convert point list to man seq.
      serialNext = 0;
      closeCnt = 0;
      manFull->releaseAllMan();
      if (pointListFinal != NULL)
      { // now convert to manoeuvre sequence
        valid = convertToManSeq(pointListFinal, manFull, par->minTurnRadius, &costAdd);
        printf("UAvoidPath2::findObstAvoidRoute: finished convertToManSeq          after %.5f sec\n", t2.getTimePassed());
        // debug cost results
        for (int h = 0; h < costAdd.costsCnt; h++)
        {
          printf("cost add at %.3fx,%.3fy cost %.1f, left=%s\n", costAdd.costs[h].pos.x, costAdd.costs[h].pos.y, costAdd.costs[h].cost, bool2str(costAdd.costs[h].avoidLeft));
        }
        // debug end
      }
      if (valid)
        break;
    }
  }
  else
  {
    // test obstacles groups for embedded vertices, and mark these as
    // nogoVertices - no tangents from these
    invalidateEmbeddedVertices();
    //
    //printf("timing - grouping took %f sec\n", t.getTimePassed());
    // add start and exit points as single point obstacles
    if (aogsCnt <= MAX_AOG_OBST_GRPS - 2)
    { // add start and exit position as "obstacles" in two extra obstacle groups
      addStartAndExit();
      // create links from obstacle to obstacle - 4 lines for each obstacle pair
      createTangentLinks(minPassableDistance);
      // debug
      //printf("timing + create tangents %f sec\n", t.getTimePassed());
      if (logdbg != NULL)
      {
        fprintf(logdbg, "\n******* Created links\n");
        logObstacleGroups();
      }
      // debug end
      // count tangent lines before visibility test
      cnt1 = countValidVisibilityLines();
      //printf("avoid:: after createTangentLinks()     visibility lines %d\n", cnt1);
      // test the tangent lines for visibility - invalidate the lines with no visibility
      testVisibility();
      //printf("timing + test visibilty %f sec\n", t.getTimePassed());
      // count tangent lines before concavity test test
      cnt2 = countValidVisibilityLines();
      // debug
      // printf("avoid:: after testVisibility()         visibility lines %d\n", cnt2);
      if (logdbg != NULL)
      {
        fprintf(logdbg, "\n******* removed lines with no visibility by ********************"
            "other obstacles (cnt %d to %d)\n", cnt1, cnt2);
        logObstacleGroups();
      }
      // debug end
      // delete bad tangent lines - concave part of obstacle groups
  /*    removeTangentInConcavities(); */
      // count lines after concavity test
      cnt3 = countValidVisibilityLines();
      // debug
      if (logdbg != NULL)
      {
        fprintf(logdbg, "\n******* removed tangents against noVis lines "
            "(cnt %d to %d)\n", cnt2, cnt3);
        logObstacleGroups();
      }
      // debug end
      // delete tangents that has no exit tangent
      removeTangentIfNoExitTangent();
      //printf("timing + remove if no exit %f sec\n", t.getTimePassed());
      // count tangents
      cnt4 = countValidVisibilityLines();
      // debug
      // printf("avoid:: after removeTangentIfNoExitTangent() visibility lines %d\n", cnt4);
      if (logdbg != NULL)
      {
        fprintf(logdbg, "\n******* removed tangentsWithNoExit ****************************"
            "(cnt %d to %d)\n", cnt2, cnt4);
        logObstacleGroups();
      }
      // debug end
      // Find a sequence of tangents from start to finish
      t2.now();
      //findRoutes();
      findRoutesA();
      // debug
      // printf("visual graph timing - %f sec (find routes=%.4f sec)\n",
      //       t.getTimePassed(), t2.getTimePassed());
      t2.now();
      // debug end
      // debug
      if (logdbg != NULL)
      {
        fprintf(logdbg, "\n******* after find routes\n");
        logObstacleGroups();
        logPathSequences();
      }
      // debug end
      //
      // find best route as a manoeuvre sequence
      // find point list - convert to man-seq
      if (lnkSeqsCnt > 0)
      { // should always be valid - at least one from start to finish
        //
        // get minimum turn radius
        dist = par->startPose.getDistance(par->exitPose);
        //lnkSeq = NULL;
        minTurnRadius = par->minTurnRadius;
        if (dist < 2.0 * minTurnRadius)
          // getting close allow tighter turns to avoid no-solution lockups
          minTurnRadius *= 0.6;
        if (par->useDriveon)
          // use the driveon minimum turn radius
          minTurnRadius = par->driveonGA * M_PI / (par->driveonGD * 2.0);
        //
        // convert best route to avoid points
        //  - avoid points are points on obstacles to avoid
        for (i = 0; i < lnkSeqsCnt; i++)
        { // test only a few of the best visibility graphs
          if (i >= par->maxTangentToMan)
          { // do not test any further - better to say no path found.
            valid = true;
            break;
          }
          closeCnt = 0;
          valid = expandVisLinesToManSeq(lnkSeqs[i]);
          // debug
          printf("visual graph timing - %.4f sec (expand to man-sequence=%.4f sec)\n",
          t.getTimePassed(), t2.getTimePassed());
          t2.now();
          // debug end
          if (not valid and par->useAnyResult)
          {
            printf("Using an invalid result - as avoid.useAnyResult is true\n");
            valid = true;
          }
          // use first valid path
          if (valid)
            break;
          manLength = manFull->getDistance();
          if (manLength > par->getMinOpening())
          { // there is som OK distance - try it out
            valid = true;
            // debug
            printf("Using solution #%d as it is OK for %.2fm\n", i, manLength);
            // debug end
            break;
          }
          printf("UAvoidPath2:: "
                "discarded solution #%d: dist %.2fm (link-seq: dist=%.2fm ang=%.1fdeg)\n",
                i, manLength,
                lnkSeqs[i]->costDist,
                lnkSeqs[i]->costAngle * 180.0 / M_PI);
        }
      }
    }
  }
  if (not valid)
    printf("UAvoidPath2::findObstAvoidRoute: **** no usable path found\n");
  // debug
  if (logdbg != NULL)
  {
    fflush(logdbg);
    fclose(logdbg);
    logdbg = NULL;
    printf("UAvoidPath2::findObstAvoidRoute: !!!!!!! logdbf written! file closed\n");
  }
  // debug end
}

/////////////////////////////////////////////////////

int compareXposition (const void * a, const void * b)
{
  UPosition ** pa = (UPosition**) a;
  UPosition ** pb = (UPosition**) b;
  if ((*pa)->x > (*pb)->x)
    return 1;
  else if ((*pa)->x == (*pb)->x)
    return 0;
  else
    return -1;
}

///////////////////////////////////////////

void UAvoidPath2::removeEqualXvertices(UObstacle * obst)
{
  UPosition **pp, *pn;
  UPolygon40 p40;
  bool moved = false;
  // When two vertices has the same x-value, the ordering
  // may be wrong, especially if the vertices
  // belong to the same obstacle.
  pp = obst->getPPoints();
  pn = obst->getPoints();
  for (int n = 0; n < obst->getPointsCnt(); n++)
    pp[n] = &pn[n];
  qsort(pp, obst->getPointsCnt(), sizeof(UPosition*), compareXposition);
  for (int n = 1; n < obst->getPointsCnt(); n++)
  {
    if (pp[n]->x <= pp[n-1]->x)
    {
      pp[n]->x = pp[n-1]->x + 0.001;
      moved = true;
    }
  }
  if (moved)
  { // make shure the result is convex
    obst->extractConvexTo(&p40);
    // copy positions back
    p40.copyTo(obst);
    obst->setValid(true);
  }
}

//////////////////////////////////////////

void UAvoidPath2::groupObstacles(double margin)
{
  int i, j, k, m;
  UObstacleGroup * obstGrp;
  UObstacle * obst;
  const int MML = 10;
  int merge[MML];
  int mergeCnt;
  UAvoidObst ** aog, *og;
  UAvoidObst * aogNew;
  UPosition pNew, pGrp;
  //
  if (logdbg != NULL)
  {
    fprintf(logdbg, "Source obstacles: %d source groups\n", par->obsts->getGroupsCnt());
  }
  maxSerial = 0;
  for (i = 0; i < par->obsts->getGroupsCnt(); i++)
  { // find nearest obstacle to the left and right
    obstGrp = par->obsts->getGroup(i);
    if (logdbg != NULL)
    {
      fprintf(logdbg, "Source obstacles grp %d serial %lu has %d obsts\n", i, obstGrp->getSerial(), obstGrp->getObstsCnt());
    }
    for (j = 0; j < obstGrp->getObstsCnt(); j++)
    { // get next obstacle
      obst = obstGrp->getObstacle(j);
      if (obst->getSerial() > maxSerial)
        maxSerial = obst->getSerial();
      if (par->cellRev2)
      { // remov equal-x vertices
        removeEqualXvertices(obst);
      }
        // test if validated obstacle
      if (not obst->isValid())
        // small obstacles not seen in two scans are invalid and should be ignored
        continue;
      aogNew = getEmptyAog();
      if (aogNew != NULL)
      {
        aogNew->obst = obst;
        aog = aogs;
        // debug
        pNew = obst->getCogXY();
        // debug end
        // set all vertices and edges as valid tangent points
        for (k = 0; k < obst->getPointsCnt(); k++)
        {
          aogNew->badEdges[k] = 0;
          aogNew->nogoEdge[k] = false;
          aogNew->nogoVertex[k] = false;
        }
        // may be part of a obstacle group
        mergeCnt = 0;
        for (k = 0; k < aogsCnt; k++)
        {
          // debug
          pGrp = (*aog)->obst->getCogXY();
          // debug end
          if ((*aog)->addToGroupIfWithinMargin2(aogNew, margin, mergeCnt > 0)) // embList, &embListCnt, ELC))
          { // merged with this group
            if (mergeCnt >= MML)
            {
              printf("UAvoidPath2::groupObstacles - Too many merges\n");
              break;
            }
            merge[mergeCnt++] = k;
          }
          // move to next
          aog++;
        }
        if (logdbg != NULL)
        {
          pGrp = obst->getCogXY();
          fprintf(logdbg, "   %2d obst %lu:%lu has %d vertices and COG (%.2fx,%.2fy) valid=%s (newGrp %d.%lu)\n", j, obstGrp->getSerial(),
                  obst->getSerial(), obst->getPointsCnt(), pGrp.x, pGrp.y, bool2str(obst->isValid()),
                  aogNew->grpIdx, aogNew->obst->getSerial());
        }
        if ((mergeCnt == 0) and (aogsCnt < MAX_AOG_OBST_GRPS))
        { // not merged, so set as first in a new isolated group
          aog = &aogs[aogsCnt++];
          (*aog) = aogNew;
          if (logdbg != NULL)
          {
            fprintf(logdbg, "   %2d obst %lu:%lu started new group %d\n", j, obstGrp->getSerial(),
                    obst->getSerial(), aogsCnt - 1);
          }
        }
        else if (mergeCnt == 1)
        {
          if (logdbg != NULL)
          {
            fprintf(logdbg, "   %2d obst %lu:%lu put into with group %d\n", j, obstGrp->getSerial(),
                    obst->getSerial(), merge[0]);
          }
        }
        else if (mergeCnt > 1)
        { // this obstacle merges two or more current groups
          // get new merged group as the first in the list
          aogNew = aogs[merge[0]];
          // then add the others
          if (logdbg != NULL)
          {
            fprintf(logdbg, "   %2d obst %lu:%lu merge into with group %d += ", j, obstGrp->getSerial(),
                    obst->getSerial(), merge[0]);
          }
          for (k = mergeCnt - 1; k > 0; k--)
          { // advance to end of this merged group
            // debug
            if (logdbg != NULL)
              fprintf(logdbg, "%d ", merge[k]);
            // debug end
            while (aogNew->grp != NULL)
              aogNew = aogNew->grp;
            // get index of group to be merged
            m = merge[k];
            // add it to the end
            aogNew->grp = aogs[m];
            //move remaining pointers one step back
            if (m < aogsCnt - 1)
              memmove(&aogs[m], &aogs[m+1], sizeof(void*) * (aogsCnt - m - 1));
            aogsCnt--;
          }
          // debug
          if (logdbg != NULL)
            fprintf(logdbg, "\n");
          // debug end
        }
        if (logdbg != NULL)
          for (k = 0; k < aogsCnt; k++)
          {
            og = aogs[k];
            fprintf(logdbg, "   newGrp %d has %lu", k, og->obst->getSerial());
            while (og->grp != NULL)
            {
              og = og->grp;
              fprintf(logdbg, " %lu", og->obst->getSerial());
            }
            fprintf(logdbg, "\n");
          }
      }
      else
        printf("UAvoidPath2::groupObstacles: No more stack space?\n");
    }
  }
  // set new obstacle group index on all abstacles in new groups
  for (int g = 0; g < aogsCnt; g++)
  {
    og = aogs[g];
    while (og != NULL)
    {
      og->grpIdx = g;
      og = og->grp;
    }
  }
}

/////////////////////////////////////////

UAvoidObst * UAvoidPath2::getEmptyAog()
{
  UAvoidObst * result;
  //
  if (aogRoot == NULL)
  {
    aogRoot = new UAvoidObst();
    result = aogRoot;
    result->empty = NULL;
  }
  else
  {
    result = aogLast->empty;
    if (result == NULL)
    {
      result = new UAvoidObst();
      result->empty = NULL;
      aogLast->empty = result;
    }
  }
  aogLast = result;
  result->clear();
  return result;
}

/////////////////////////////////////////

UAvoidLink * UAvoidPath2::getEmptyALnk(int serial)
{
  UAvoidLink * result;
  //
  if (aLnkRoot == NULL)
  {
    aLnkRoot = new UAvoidLink();
    result = aLnkRoot;
    result->empty = NULL;
  }
  else
  {
    result = aLnkLast->empty;
    if (result == NULL)
    {
      result = new UAvoidLink();
      result->empty = NULL;
      aLnkLast->empty = result;
    }
  }
  aLnkLast = result;
  result->clear();
  result->serial = serial;
  return result;
}

/////////////////////////////////////////

UAvoidLnkSeq * UAvoidPath2::getEmptyLnkSeq()
{
  UAvoidLnkSeq * result;
  //
  if (lnkSeqRoot == NULL)
    // nothing to reuse
    result = new UAvoidLnkSeq();
  else
  { // reuse first from released stack
    result = lnkSeqRoot;
    lnkSeqRoot = result->next;
  }
  // make nice initial values
  result->clear();
  result->serial = lnkSerial++;
  return result;
}

/////////////////////////////////////////

UAvoidPoint * UAvoidPath2::getEmptyPoint()
{
  UAvoidPoint * result;
  //
  if (aPointRoot == NULL)
    // nothing to reuse
    result = new UAvoidPoint();
  else
  { // reuse first from released stack
    result = aPointRoot;
    aPointRoot = result->next;
  }
  // make nice initial values
  result->clear();
  result->serial = serialNext++;
  return result;
}

/////////////////////////////////////////

void UAvoidPath2::recyclePoint(UAvoidPoint * ap)
{
  // recycle this point only
  ap->next = aPointRoot;
  aPointRoot = ap;
}

/////////////////////////////////////////

void UAvoidPath2::recyclePoints(UAvoidPoint * first)
{ // recycle all points in next chain
  UAvoidPoint * ap = first;
  //
  if (ap != NULL)
  { // find last
    while (ap->next != NULL)
      ap = ap->next;
    // add existent recycled points to the end
    ap->next = aPointRoot;
    // first is now the start of the recycled list
    aPointRoot = first;
  }
}

/////////////////////////////////////////

void UAvoidPath2::recycleLnkSeq(UAvoidLnkSeq * ls)
{
  ls->prev = NULL;
  // add at end of list
  if (lnkSeqRoot == NULL)
  { // recycle list is empty, so just set
    lnkSeqRoot = ls;
    lnkSeqLast = ls;
  }
  else
    // recycle list is not empty, add at the end
    lnkSeqLast->next = ls;
  // advance the last pointer to the last element.
  while (lnkSeqLast->next != NULL)
  {
    lnkSeqLast = lnkSeqLast->next;
    // the prev seqence is not valid in recycle list.
    lnkSeqLast->prev = NULL;
  }
}

////////////////////////////////////////////


void UAvoidPath2::createTangentLinks(double margin)
{
  UAvoidObst * aoga, *aogb;
  int i, j, n = 0;
  UAvoidLink *la, *lb, *lc = NULL, *ld = NULL;
  bool isOK;
  //
  for (i = 0; i <  aogsCnt; i++)
  {
    aoga = aogs[i];
    while (aoga != NULL)
    { // assign group number to all members of group
      aoga->grpIdx = i;
      j = i;
      aogb = aoga->grp;
      while (true)
      {
        while (aogb != NULL)
        { // we have 2 obstacles aoga ad aogb
          // we need a link description between these
          la = getEmptyALnk(n++);
          lb = getEmptyALnk(n++);
          isOK = aoga->addtangentLines(aogb, la, lb, NULL, NULL);
          if (j == i)
          { // obstacles are in the same group, so additional possibilities exist.
            //
            // obstacles may cross each other, and therefore need another set of
            // links, as up to 4 outher links are valid in this situation only.
            if (not isOK)
            { // a second set of outher tangents are needed
              lc = getEmptyALnk(n++);
              ld = getEmptyALnk(n++);
              // retry with double set of link structures
              aoga->addtangentLines(aogb, la, lb, lc, ld);
              invalidateConcavities(lc, ld);
              lc->setBadEdgeAndVertex(margin);
              ld->setBadEdgeAndVertex(margin);
              if (lc->tangentCnt > 0)
              { // link in the extra set of outher tangents
                lc->next = aoga->links;
                aoga->links = lc;
                ld->next = aogb->links;
                aogb->links = ld;
              }
            }
            // set edges and vertices that is not usable for
            // link lines at all - not only in concavities.
            // setting the nogoEdge and nogoVertex arrays
            la->setBadEdgeAndVertex(margin);
            lb->setBadEdgeAndVertex(margin);
            // same group, so invalidate vertices
            // in the concavity between the objects
            invalidateConcavities(la, lb);
          }
          if (la->tangentCnt > 0)
          { // link in the primary set of tangens
            la->next = aoga->links;
            aoga->links = la;
            lb->next = aogb->links;
            aogb->links = lb;
          }
          //
          aogb = aogb->grp;
        }
        j++;
        if (j < aogsCnt)
          aogb = aogs[j];
        else
          break;
      }
      //
      aoga = aoga->grp;
    }
  }
}

///////////////////////////////////////////

void UAvoidPath2::invalidateConcavities(UAvoidLink * la, UAvoidLink  * lb)
{
  int i, a, b, n, m;
  //
  //
  // is this then surplus?
  // is setting badEdges array
  if (la->valid[1] and la->valid[2])
  { // this is the outher tangents, and
    // the vertices in between is not vald as
    // tangent points.
    // get number of vertices in polygon
    m = la->tob->obst->getPointsCnt();
    if (m > 1)
    { // if one point obstacle, then no incalid tangent lines
      a = la->idx[1];
      b = la->idx[2];
      n = b - a;
      if (n <= 0)
        // spanning index 0
        n += m;
      // set no-visibility interval
      for (i = 0; i < n; i++)
      {
        if ((i == 0) and (la->tob->badEdges[a] < 3))
          // invalid edge
          la->tob->badEdges[a] = 1;
        else
          // point and edge invalid
          la->tob->badEdges[a] = 3;
        a++;
        if (a >= m)
          a = 0;
      }
    }
  }
  // and the same for the other obstacle
  if (lb->valid[1] and lb->valid[2])
  { // this is the outher tangents.
    // get number of vertices in polygon
    m = lb->tob->obst->getPointsCnt();
    if (m > 1)
    {
      a = lb->idx[1];
      b = lb->idx[2];
      n = b - a;
      if (n <= 0)
        // spanning index 0
        n += m;
      for (i = 0; i < n; i++)
      {
        if ((i == 0) and (lb->tob->badEdges[a] == 0))
          // point is valid, but not the edge
          lb->tob->badEdges[a] = 1;
        else
          // point and edge invalid
          lb->tob->badEdges[a] = 3;
        a++;
        if (a >= m)
          a = 0;
      }
    }
  }
}

///////////////////////////////////////////

void UAvoidPath2::removeTangentInConcavities()
{
  UAvoidObst * aoga;
  int i, j, n, k;
  UAvoidLink *la, *la2;
  int z = 0;
  UPosition p1, p2;
  //
  for (j = 0; j <  aogsCnt; j++)
  {
    aoga = aogs[j];
    if (aoga->grp != NULL)
    { // more than one obstacle in group, so
      // there may be concavities
      while (aoga != NULL)
      { // test all tangents starting from this
        // obstacle if it origins in a concavity
        // and ends in another obstacle in the same group
        //m = aoga->obst->getPointsCnt();
        la = aoga->links;
        la2 = la;
        while (la != NULL)
        { // test all valid links
          if (la->aob->grpIdx == j)
          { // same group
            k = 0;
            for (i = 0; i < la->tangentCnt; i++)
            { // 0=cv-cv; 1 = ccv-cc; 2 = cv-ccv; 3 = ccv-ccv
              if (la->valid[i])
              {
                n = la->idx[i];
                if (aoga->badEdges[n] == 3)
                {
                  la->valid[i] = false;
                  if (la->mirror != NULL)
                  { // remove the mirror set too
                    if (i == 0 or i == 3)
                      la->mirror->valid[i] = false;
                    else if (i == 1)
                      la->mirror->valid[2] = false;
                    else
                      la->mirror->valid[1] = false;
                  }
                }
                else
                  k = i + 1;
                // debug
/*                p1 = la->tob->obst->getPoint(la->idx[i]);
                p2 = la->aob->obst->getPoint(la->aobIdx[i]);
                printf("Valid %s tangent grp %d tang %d type %d %.2fx %.2fy to %.2fx %.2fy\n",
                      bool2str(la->valid[i]), j, z, i, p1.x, p1.y, p2.x, p2.y);*/
                // debug
              }
            }
            if (k == 0)
            { // remove the link altogeather
              // the link stays in the empty chain
              // for reuse at next run
              if (la == aoga->links)
              {
                aoga->links = la->next;
                la2 = aoga->links;
              }
              else
                la2->next = la->next;
            }
            else if (k < la->tangentCnt)
            { // just reduce count
              la->tangentCnt = k;
              la2 = la;
            }
            else
              la2 = la;
          }
          else
            la2 = la;
          z++;
          la = la->next;
        }
        aoga = aoga->grp;
      }
    }
  }
}

///////////////////////////////////////////

void UAvoidPath2::removeTangentIfNoExitTangent()
{
  UAvoidObst * aog;
  int i, k, m;
  UAvoidLink * lnk;
  bool validFwd, validBack;
  UPosition p1;

  for (i = 0; i < aogsCnt; i++)
  {
    aog = aogs[i];
    while (aog != NULL)
    { // test all links connecting this obstacle to others
      lnk = aog->links;
      while (lnk != NULL)
      {
        if ((lnk->mirror != NULL))
        {
          m = 0;
          for (k = 0; k < lnk->tangentCnt; k++)
          {
            if (lnk->valid[k])
            {
              validFwd = lnk->aob->isExit;
              if (not validFwd)
              { // find source point at this obstacle - used in evaluation
                p1 = aog->obst->getPoint(lnk->idx[k]);
                validFwd = lnk->aob->validTangent(k % 2 == 0, lnk->aobIdx[k], p1);
              }
              if (validFwd)
              { // need to test backwards too, same line but back towards this (aog)
                // obstacle, if start pose, then always valid
                validBack = lnk->tob->isStart;
                if (not validBack)
                { // not start, so test. Get other end of tangent line
                  p1 = lnk->aob->obst->getPoint(lnk->aobIdx[k]);
                  validBack = lnk->tob->validTangent(k < 2, lnk->idx[k], p1);
                }
              }
              else
                validBack = false;
              if (validFwd and validBack)
                m = k + 1;
              else
              { // not a valid tangent - remove
                lnk->valid[k] = false;
                // and remove mirror tangent too
                if (k == 0 or k == 3)
                  lnk->mirror->valid[k] = false;
                else if (k == 1)
                  lnk->mirror->valid[2] = false;
                else
                  lnk->mirror->valid[1] = false;
              }
            }
          }
          // reduce link count
          lnk->tangentCnt = m;
        }
        //
        lnk = lnk->next;
      }
      // continue with next obstacle in group
      aog = aog->grp;
    }
  }
}

///////////////////////////////////////////

void UAvoidPath2::testVisibility()
{
  UAvoidObst * aoga;
  int i, n = 0, m = 0, j = 0;
  //int cnt1, cnt2, cnt3;
  //
  for (i = 0; i <  aogsCnt; i++)
  {
    aoga = aogs[i];
    while (aoga != NULL)
    {
      if (logdbg != NULL)
      {
        fprintf(logdbg, "visibility test for obstacle group %d, obstacle %d\n", i, aoga->grpIdx);
      }
      m++;
      //cnt1 = countValidVisibilityLines(aoga->links);
      aoga->testVisibility(logdbg);
      //cnt2 = countValidVisibilityLines(aoga->links);
      testNoVisSegVisibility(aoga, &n, &j);
      //cnt3 = countValidVisibilityLines(aoga->links);
      // debug
      //printf("UAvoidPath2::testVisibility() obst %lu reduced from %d to %d to %d\n",
      //       aoga->obst->getSerial(), cnt1, cnt2, cnt3);
      // debug end
      aoga = aoga->grp;
    }
    //printf("tested grp %d obst %d link %d deleted %d\n", i, m, n, j);
  }
}

///////////////////////////////////////////

int UAvoidPath2::countValidVisibilityLines()
{
  UAvoidObst * aoga;
  int i;
  int cnt = 0;
  //
  for (i = 0; i <  aogsCnt; i++)
  {
    aoga = aogs[i];
    while (aoga != NULL)
    {
      cnt += countValidVisibilityLines(aoga->links);
      aoga = aoga->grp;
    }
  }
  return cnt;
}

///////////////////////////////////////////////////////

int UAvoidPath2::countValidVisibilityLines(UAvoidLink * startLnk)
{
  UAvoidLink * aLnk = startLnk;
  int k, cnt = 0;
  //
  while (aLnk != NULL)
  {
    if (aLnk->mirror != NULL)
    {
      for (k = 0; k < aLnk->tangentCnt; k++)
      {
        if (aLnk->valid[k])
        {
          cnt++;
        }
      }
      // debug
      //if (n == 0)
      //  printf("UAvoidPath2::countValidVisibilityLines found a link set with no valid links\n"
      //      "-- group %d obst=%x link %d\n", i, (unsigned int)aoga, m);
      // debug end
    }
    aLnk = aLnk->next;
  }
  return cnt;
}

///////////////////////////////////////////

void UAvoidPath2::testNoVisSegVisibility(UAvoidObst * aogb, int * n, int * m)
{
  int k = 0, j = 0;
  ULineSegment seg;
  UAvoidLink * aLnk, *aLnk2;
  UPosition p1, p2;
  bool isOnePoint;
  bool isolated;
  bool alnkIsInvalid = false;
  //
  aLnk = aogb->links;
  if (aLnk != NULL)
  { // no previous tangent
    aLnk2 = NULL;
    isOnePoint = (aLnk->tob->obst->getPointsCnt() == 1 and
                  not aLnk->tob->isStart and
                  not aLnk->tob->isExit);
    // debug
    if (isOnePoint)
    {
      p1 = aLnk->tob->obst->getPoint(0);
      j = 1;
    }
    // debug end
    isolated = aLnk->tob->noVisCnt == 0;
    // debug
    if (isolated)
      j = j + 1;
    if (isolated and isOnePoint)
    {
      printf("Isolated one-point at %.2fx,%.2fy\n", p1.x, p1.y);
      j = j + 2;
    }
    // debug end
    while (aLnk != NULL)
    { // try all links away from this obstacle
      j = 0;
      // isolated points are too small to be part of route
      if (isolated and isOnePoint)
      { // debug
        // printf("found an isolated point\n");
      } // debug end
      else if (aLnk->mirror != NULL)
      { // not isolated or not a point
        // test if other end is an isolated single point obstacle
        if (aLnk->mirror->tangentCnt == 0 or
            (aLnk->mirror->tob->obst->getPointsCnt() == 1 and
             aLnk->mirror->tob->noVisCnt == 0 and
             not aLnk->mirror->tob->isStart and
             not aLnk->mirror->tob->isExit))
        { // mirror is isolated - drop this too line too
          // printf("Mirror har %d points at %.2fx,%.2fy and %d no-visibility lines and %d tangent lines\n",
          //        aLnk->mirror->tob->obst->getPointsCnt(), aLnk->mirror->tob->obst->getPoint(0).x, aLnk->mirror->tob->obst->getPoint(0).y,
          //        aLnk->mirror->tob->noVisCnt, aLnk->mirror->tangentCnt);
          // j = 0;
        }
        else
        {
          for (k = 0; k < aLnk->tangentCnt; k++)
          { // try all valid lines from this obstacle
            if (aLnk->valid[k])
            { // get line segment
              // debug
              (*n)++; //
              // debug end
              p1 = aogb->obst->getPoint(aLnk->idx[k]);
              p1.z = 0.0;
              p2 = aLnk->aob->obst->getPoint(aLnk->aobIdx[k]);
              p2.z = 0.0;
              seg.setFromPoints(p1, p2);
              if (testNonVisibilityLineCross(&seg))
              {
                aLnk->valid[k] = false;
                if (k == 0 or k == 3)
                  aLnk->mirror->valid[k] = false;
                else if (k == 1)
                  aLnk->mirror->valid[2] = false;
                else
                  aLnk->mirror->valid[1] = false;
                (*m)++;
              }
              else
                j = k+1;
            }
          }
        }
      }
      else // just a mirror entry - and is tested from other end
      { // just test for valid exit lines
        for (k = 0; k < aLnk->tangentCnt; k++)
          if (aLnk->valid[k])
            j = k + 1;
        // debug
        if (j == 0)
          k = 0;
        // debug end
      }
      alnkIsInvalid = (j == 0);
      if (alnkIsInvalid)
      { // not valid anymore - remove from list
        if (aLnk == aogb->links)
        { // first in list
          aogb->links = aLnk->next;
          // there is no previous tangent
          aLnk2 = NULL;
        }
        else
        { // lnk2 is OK, just bypass aLnk
          aLnk2->next = aLnk->next;
        }
        // mirror set should be removed too, but count is
        // set to zero only (pointers are not readily available)
        if (aLnk->mirror == NULL)
          // this mirror may still be accessed from other end
          // so set valid line count to 0
          aLnk->tangentCnt = 0;
        else
          // incalidate mirror too
          aLnk->mirror->tangentCnt = 0;
      }
      else
      { // reduce count as appropriate
        aLnk->tangentCnt = j;
      }
      // move to next
      if (not alnkIsInvalid)
        // advance previous link pointer
        aLnk2 = aLnk;
      if (aLnk2 == NULL)
        // no previous
        aLnk = aogb->links;
      else
        // take next link
        aLnk = aLnk2->next;
    }
  }
}

///////////////////////////////////////////

bool UAvoidPath2::testNonVisibilityLineCross(ULineSegment * visLine)
{
  UAvoidObst * aoga;
  int i;
  bool isX = false;
  //
  for (i = 0; i <  aogsCnt; i++)
  { // test no-visiblility lines for all obstacles groups
    aoga = aogs[i];
    isX = aoga->crossingNonVisibilityLine(visLine, logdbg);
    if (isX)
      break;
  }
  return isX;
}

///////////////////////////////////////////

void UAvoidPath2::invalidateObstaclesAtStartAndExit()
{
  UAvoidObst * aoga;
  int i;
  UPosition p1, p2;
  //
  p1 = par->startPose.getPos(0.0);
  p2 = par->exitPose.getPos(0.0);

  for (i = 0; i <  aogsCnt; i++)
  {
    aoga = aogs[i];
    while (aoga != NULL)
    {
      if (aoga->obst->isInsideConvex(p1.x, p1.y, 0.01))
      {
        aoga->embedStart = true;
        // debug
        printf("UAvoidPath2::invalidateObstaclesAtStartAndExit start embedded inside obstacle! - obstacle ignored\n");
        // debug end 
      }
      if (aoga->obst->isInsideConvex(p2.x, p2.y, 0.01))
      {
        aoga->embedEnd = true;
        // debug
        printf("UAvoidPath2::invalidateObstaclesAtStartAndExit destination embedded inside obstacle! - obstacle ignored\n");
        // debug end
      }
      aoga = aoga->grp;
    }
  }
}

///////////////////////////////////////////

void UAvoidPath2::addStartAndExit()
{
  UAvoidObst * aog;
  UPoseTime pose;
  UTime t;
  // start point
  t.Now();
  aog = getEmptyAog();
  aog->obst = startExitObst[0];
  aog->isStart = true;
  pose.setPt(par->startPose.getPose(), t);
  aog->obst->clear();
  aog->obst->initPoseFirst(pose);
  aog->obst->add(pose.x, pose.y, 0.0);
  aog->obst->setValid(true);
  aog->nogoVertex[0] = false;
  aog->nogoEdge[0] = false;
  aog->badEdges[0] = 0;
  aog->grpIdx = aogsCnt;
  aogs[aogsCnt++] = aog;
  // exit point
  aog = getEmptyAog();
  aog->obst = startExitObst[1];
  aog->isExit = true;
  pose.setPt(par->exitPose.getPose(), t);
  aog->obst->clear();
  aog->obst->initPoseFirst(pose);
  aog->obst->add(pose.x, pose.y, 0.0);
  aog->obst->setValid(true);
  aog->nogoVertex[0] = false;
  aog->nogoEdge[0] = false;
  aog->badEdges[0] = 0;
  aog->grpIdx = aogsCnt;
  aogs[aogsCnt++] = aog;
}

////////////////////////////////////////////

void UAvoidPath2::findRoutes()
{
  UAvoidLnkSeq * ls;
  UAvoidObst * aog, *aogDest;
  UAvoidLink * aLnk;
  int k, i, n, no;
  ULineSegment seg;
  UPosition pd, pr;
  UTime t;
  bool isOK;
  double a;
  //
  // recycle link sequence objects
  for (i = 0; i < lnkSeqsCnt; i++)
    recycleLnkSeq(lnkSeqs[i]);
  lnkSeqsCnt = 0;
  // start at current position
  n = 0;
  no = 0;
  aog = aogs[aogsCnt-2];
  aLnk = aog->links;
  // get pointer to final position (a one point obstacle)
  aogDest = aogs[aogsCnt-1];
  maxDistCost = 1e4;
  maxAngleCost = 3.0 * M_PI;
  tanSeqCnt = 0;
  // now try all the possible link lines from the start position
  while (aLnk != NULL)
  { // each link may have up to 2 (valid) tangent lines
    no++;
    for (k = 0; k < aLnk->tangentCnt; k++)
    {
      n++;
      ls = NULL;
      if (aLnk->valid[k])
      { // the tangent line is valid
        // start a sequence
        ls = getEmptyLnkSeq();
        tanSeqCnt++;
        // add the selected tangent line
        ls->tangLine = aLnk;
        ls->tangIdx = k;
        seg = ls->tangLine->getTangentLine(k);
        // approximate cost based on visibility lines only
        // for search reduction.
        ls->costDist = seg.length;
        ls->costAngle = fabs(limitToPi(seg.getXYHeading() - par->startPose.h));
        if (aLnk->aob == aogDest)
        { // this is a direct line - no need to go any further
          addToLnkSeqs(ls);
          ls = NULL;
          break;
        }
        isOK = true;
        t.now();
        if (par->forwardOnly < M_PI)
        { // test if obstacle point is behind robot
          pd = aLnk->getOtherEnd(k);
          pr = par->startPose.getMapToPose(pd);
          a = atan2(pr.y, pr.x);
          // is forward
          isOK = (fabs(a) <= par->forwardOnly);
        }
        if (isOK)
        { // start with this line and find destination
          findRouteToDest(ls, ls, 0);
          //printf("Obst %d Link seq %d took %.5f sec\n", no, n, t.getTimePassed());
        }
      }
      if (ls != NULL)
        // recycle failed paths (e.g. a dead end)
        recycleLnkSeq(ls);
    }
    aLnk = aLnk->next;
  }
  // debug
  printf("UAvoidPath2::findRoutes: found %d tangent sequences in %d obsts with %d tangents from start pose\n", lnkSeqsCnt, no, countValidVisibilityLines());
  // debug end
}

///////////////////////////////////////////////

void UAvoidPath2::findRoutesA()
{
  UAvoidLnkSeq * ls;
  UAvoidObst * aog, *aogDest;
  UAvoidLink * aLnk;
  int k, i, n, no;
  ULineSegment seg;
  UPosition ps, pe;
  UTime t;
  bool isOK;
  UPose poDest;
  double minTurnRadius;
  //
  // recycle link sequence objects
  for (i = 0; i < lnkSeqsCnt; i++)
    recycleLnkSeq(lnkSeqs[i]);
  lnkSeqsCnt = 0;
  // start at current position
  n = 0;
  no = 0;
  aog = aogs[aogsCnt-2];
  aLnk = aog->links;
  // get pointer to final position (a one point obstacle)
  aogDest = aogs[aogsCnt-1];
  maxDistCost = 1e4;
  maxAngleCost = 3.0 * M_PI;
  tanSeqCnt = 0;
  minTurnRadius = par->getMinTurnRad();
  poDest = par->exitPose;
  openSet = NULL;
  lnkSerial = 0;
  // now set all links from start position from the start position
  // using the weight from initial turn and first link distance
  ls = NULL;
  i = 0;
  while (aLnk != NULL)
  { // each link may have up to 2 (valid) tangent lines
    no++;
    for (k = 0; k < aLnk->tangentCnt; k++)
    {
      n++;
      if (aLnk->valid[k])
      { // the tangent line is valid
        // start a sequence
        if (ls == NULL)
          ls = getEmptyLnkSeq();
        // add the selected tangent line
        ls->tangLine = aLnk;
        ls->tangIdx = k;
        ls->prev = NULL;
        seg = ls->tangLine->getTangentLine(k);
        // approximate cost based on visibility lines only
        // for search reduction.
        ls->costDist = seg.length;
        ls->costAngle = fabs(limitToPi(seg.getXYHeading() - par->startPose.h));
        // start cost to this edge (angle change only)
        ls->costG = /*ls->costAngle * minTurnRadius + */ seg.length;
        // cost to tne pose
        pe = seg.getOtherEnd();
        ls->costH = poDest.getDistance(pe) + fabs(limitToPi(seg.getXYHeading() - poDest.h)) * minTurnRadius;
        // total (minimum cost of this path from start to end.
        ls->costF = ls->costG + ls->costH;
        isOK = true;
        if (par->forwardOnly < M_PI)
        { // test if obstacle point is behind robot
          // is forward
          isOK = (fabs(ls->costAngle) <= par->forwardOnly);
        }
        // debug
/*        ps = ls->getThisEnd();
        pe = ls->getOtherEnd();
        printf("findRoutesA init %2d from (%.2fx,%.2fy) to %.2fx,%.2fy) (%.2fg,%.2ff) %s\n",
                ls->serial, ps.x, ps.y, pe.x, pe.y, ls->costG, ls->costF, bool2str(isOK));*/
        // debug end
        if (isOK)
        { // add to candidate list
          addToOpenSet(ls);
          ls = NULL;
        }
      }
    }
    aLnk = aLnk->next;
  }
  if (ls != NULL)
    recycleLnkSeq(ls);
  while (openSet != NULL)
  { // expand on the best candidate
    // get best candidate
    ls = openSet;
    // remove from open set
    openSet = openSet->next;
    // test if finished
    if (ls->tangLine->aob == aogDest)
      break;
    // add all exit links to open set
    addLnksToOpenSet(ls->tangLine->aob, ls);
    // add to closed set
    addToClosedSet(ls);
    ls = NULL;
  }
  if (ls != NULL)
  { // destination is found - now roll back to start
    ls->next = NULL;
    while (ls->prev != NULL)
    { // connect 'next' pointer to final link sequence
      ls->prev->next = ls;
      ls = ls->prev;
    }
    // save result.
    lnkSeqs[lnkSeqsCnt++] = ls;
  }
  // debug
//  printf("UAvoidPath2::findRoutesA: found %d tangent sequences in %d obsts with %d tangents from start pose\n", lnkSeqsCnt, no, countValidVisibilityLines());
  // debug end
}


//////////////////////////////////////////////////////////////

void UAvoidPath2::addLnksToOpenSet(UAvoidObst * aog, UAvoidLnkSeq * prev)
{
  bool valid = false;
  UAvoidLnkSeq * ls;
  //UAvoidObst *aogStart, *aogDest;
  UAvoidLink * aLnk;
  int k, i, idx;
  ULineSegment seg2, seg1;
  bool isStart, isExit, isPoint;
  UPosition p1, p2, pe;
  double a1, a2, ad, edgeDist;
  double minTurnRadius;
  //
  // aog is current obstacle we reached from link in prev
  // get first set of links from current obstacle
  aLnk = aog->links;
  // get index to tangent connecting to last obstacle (aob)
  // index 0 and 2 is a CV tangent and index 1 and 3 is CCV
  idx = prev->tangIdx;
  seg1 = prev->tangLine->getTangentLine(idx);
  a1 = seg1.getXYHeading();
  // start at current position
  //aogStart = aogs[aogsCnt-2];
  // get pointer to final position (a one point obstacle)
  //aogDest = aogs[aogsCnt-1];
  minTurnRadius = par->getMinTurnRad();
  // now add all the possible link lines possible from this arrive line
  while (aLnk != NULL)
  { // try all links from this obstacle
    // except if it goes back the source (lsEnd)
    if (prev->tangLine->mirror != aLnk and aLnk->mirror != prev->tangLine)
    { // is it a point obstacle
      isPoint = aLnk->tob->obst->getPointsCnt() == 1;
      //is previous obsacle the start pose
      isStart = prev->tangLine->tob->isStart;
      // is the next obstacle the exit pose
      isExit = aLnk->aob->isExit;
      // each link may have up to 2 (usable) tangent lines to next obstacle
      for (i = 0; i < 2; i++)
      {
        if (idx == 0 or idx == 2)
          // touches this obstacle in a CV tangent, so the exit
          // must touch this obstacle in a CCV tangent.
          k = 2 + i;
        else
          // entry in a CCV tangent, exit must touch CV
          k = i;
        // get tangent (if such exist)
        ls = NULL;
        if (isPoint and not aLnk->valid[k])
        { // points has less tangents, but are equally valid - try the other
          k = (k + 2) % 4;
        }
        valid = aLnk->valid[k];
        if (valid)
        { // the tangent line is valid
          // start a sequence
          ls = getEmptyLnkSeq();
          // add the selected tangent line
          ls->tangLine = aLnk;
          ls->tangIdx = k;
          ls->prev = prev;
          ls->next = NULL;
          //
          valid = not isInClosedSet(ls, prev, &edgeDist);
        }
        if (valid)
        { // get source point from last obstacle
          p1 = prev->tangLine->tob->obst->getPoint(prev->tangLine->idx[prev->tangIdx]);
          // get destination point on next obstacle
          p2 = aLnk->aob->obst->getPoint(aLnk->aobIdx[k]);
          //
          // is entry-exit connection in violation of group connections
          valid = not aog->passingNoGoEdge2((prev->tangIdx % 2) == 0,
                      prev->tangLine->aobIdx[prev->tangIdx],
                      aLnk->idx[k], p1, p2, isStart, isExit);
        }
        if (valid)
        { // is it just bouncing back with no need to use this obstacle
          seg2 = aLnk->getTangentLine(k);
          a2 = seg2.getXYHeading();
          ad = limitToPi(a2 - a1);
          // Test if new tangent is crosses the
          // test for need to touch this obstacle
          if (idx == 1 or idx == 3)
            valid = (ad >= 0.0);
          else
            valid = (ad <= 0.0);
        }
        if (valid)
        { // is it too expensive - angle wise
          ls->costDist = prev->costDist + seg2.length + edgeDist;
          ls->costAngle = prev->costAngle + fabs(ad);
          if (isExit)
            // punished by last turn up to exit pose
            ad = limitToPi(par->exitPose.h - a2);
          else
            ad = 0;
          ls->costG = prev->costG + seg2.length + edgeDist + fabs(ad) * minTurnRadius;
          valid = not terminateWorseCandidatesInOpenSet(ls, prev);
        }
        if (valid)
        { // this candidate is the best to this point so far
          pe = seg2.getOtherEnd();
          ls->costH = par->exitPose.getDistance(pe) + fabs(limitToPi(a2 - par->exitPose.h)) * minTurnRadius;
          ls->costF = ls->costG + ls->costH;
          addToOpenSet(ls);
          ls = NULL;
        }
        else if (ls != NULL)
        { // this candidate is not valid
          ls->next = NULL;
          recycleLnkSeq(ls);
        }
      }
    }
    aLnk = aLnk->next;
  }
}

//////////////////////////////////////////////////////////////

void UAvoidPath2::addToOpenSet(UAvoidLnkSeq * ls)
{ // add to open set in cost order
  UAvoidLnkSeq * os = openSet;
  UAvoidLnkSeq * os2 = NULL;
  //
  // add new using bouble sort.
  if (openSet == NULL)
    openSet = ls;
  else
  {
    while (os != NULL)
    {
      if (ls->costF < os->costF)
      { // insert here
        ls->next = os;
        if (os2 == NULL)
          // new first
          openSet = ls;
        else
          // just instert
          os2->next = ls;
        break;
      }
      // advance to next
      os2 = os;
      os = os->next;
    }
    if (os == NULL)
      // worst so far - so add at the end
      os2->next = ls;
  }
}

/////////////////////////////////////////////////


void UAvoidPath2::addToClosedSet(UAvoidLnkSeq * ls)
{
  ls->next = closedSet;
  closedSet = ls;
}

/////////////////////////////////////////////////

bool UAvoidPath2::isInClosedSet(UAvoidLnkSeq * candidate, UAvoidLnkSeq * prev, double * edgeDist)
{
  UAvoidLnkSeq * ci = closedSet;
  bool found = false;
  int idx1, idx2, idxC;
  bool ccv;
  UPosition p1, p2;
  //    0 ----2-----o
  //   oo   \    0  oo
  //   oo    3  /   oo
  //    o  ---1---- o
  // if entry tangent is 0 or 2 then exit tangent must be either 2 or 3 (CV on this obstacle)
  // if entry tangent is 1 or 3 then exit tangent must be either 0 or 1 (CCV on this obstacle)
  idx1 = prev->tangLine->aobIdx[prev->tangIdx];
  ccv = prev->tangIdx % 2 == 1;
  if (ccv)
    // counter clockwise (as is polygon),
    // so candidate (exit) vertex has higher index than entry (or folded)
    idx2 = candidate->tangLine->idx[candidate->tangIdx];
  else
  { // we are going clockwise, so candidate (exit) has a lower vertex number.
    idx2 = idx1;
    idx1 = candidate->tangLine->idx[candidate->tangIdx];
  }
  while (ci != NULL)
  { // test if the edges traversed from previous link to this is crossing
    // a vertex in the closed set.
    if (ci->tangLine->aob == candidate->tangLine->tob)
    { // same obstacle, but must be same point on the obstacle too
      // get polygin vertex number on this obstacle
      idxC = ci->tangLine->aobIdx[ci->tangIdx];
      // is this in the needed vertex list from previous to candidate
      if (idx1 <= idx2)
        found = idxC >= idx1 and idxC <= idx2;
      else
        found = idxC <= idx1 and idxC >= idx2;
      if (found)
        break;
    }
    // test if the destination vertex of the candidate already is in the closed set.
    if (ci->tangLine->aob == candidate->tangLine->aob)
    {
      found = ci->tangLine->aobIdx[ci->tangIdx] == candidate->tangLine->aobIdx[candidate->tangIdx];
      if (found)
        break;
    }
    ci = ci->next;
  }
  if (not found and edgeDist != NULL)
  {
    *edgeDist = 0.0;
    p1 = candidate->tangLine->tob->obst->getPoint(idx1);
    while (idx1 != idx2)
    { // get next point along edge of obstacle
      p2 = p1;
      idx1 = (idx1 + 1) % candidate->tangLine->tob->obst->getPointsCnt();
      p1 = candidate->tangLine->tob->obst->getPoint(idx1);
      *edgeDist += hypot(p1.y - p2.y, p1.x - p2.x);
    }
  }
  return found;
}

////////////////////////////////////////////////

bool UAvoidPath2::terminateWorseCandidatesInOpenSet(UAvoidLnkSeq * candidate, UAvoidLnkSeq * prev)
{
  UAvoidLnkSeq * oi = openSet, *oi2 = NULL;
  bool found = false;
  int idx1, idx2, idxC; //, i1, i2, i3, i4;
  bool ccv, dropOpen = false;
  UPosition p1, p2;
  double tentG;
  //double x1, y1, x2, y2, x3, y3, x4, y4;
  //
  // if entry tangent is 0 or 1 then exit tangent must be either 1 or 3 (CV on this obstacle)
  // if entry tangent is 2 or 3 then exit tangent must be either 0 or 2 (CCV on this obstacle)
  idx1 = prev->tangLine->aobIdx[prev->tangIdx];
  ccv = (prev->tangIdx % 2) == 1;
  if (ccv)
    // so idx 2 is higher than idx1 (or folded)
    idx2 = candidate->tangLine->idx[candidate->tangIdx];
  else
  { // we are going clockwise, so candidate (exit) has a lower vertex number.
    idx2 = idx1;
    idx1 = candidate->tangLine->idx[candidate->tangIdx];
  }
  while (oi != NULL)
  { // test if the edges traversed from previous link to this is crossing
    // a vertex in the closed set.
    dropOpen = false;
    if (oi->tangLine->aob == candidate->tangLine->tob)
    { // same obstacle, but must be same point on the obstacle too
      // get polygin vertex number on this obstacle
      idxC = oi->tangLine->aobIdx[oi->tangIdx];
      // is this in the needed vertex list from previous to candidate
      if (idx1 <= idx2)
        found = idxC >= idx1 and idxC <= idx2;
      else
        found = idxC <= idx1 and idxC >= idx2;
      if (found)
      { // extend tentative G from previous point to the candidate in open set
        tentG = prev->costG;
        // get end point on obstacle at start of candidate obstacle
        // get entry point into this obstacle
        // debug
//         // index this end
//         i1 = candidate->tangLine->idx[candidate->tangIdx];
//         // position this end
//         p1 = candidate->getThisEnd();
//         x1 = p1.x; y1=p1.y;
//         //
//         i2 = candidate->tangLine->aobIdx[candidate->tangIdx];
//         p1 = candidate->getOtherEnd();
//         x2 = p1.x; y2=p1.y;
//         //
//         i3 = oi->tangLine->aobIdx[oi->tangIdx];
//         p1 = oi->getOtherEnd();
//         x3 = p1.x; y3=p1.y;
//         //
//         i4 = prev->tangLine->aobIdx[prev->tangIdx];
//         p1 = prev->getOtherEnd();
//         x4 = p1.x; y4=p1.y;
//         // debug end
        //
        // start at entry point (endpoint of prev tangent)
        idx1 = prev->tangLine->aobIdx[prev->tangIdx];
        p1 = candidate->tangLine->tob->obst->getPoint(idx1);
        // get distance to vertex in open set
        while (idx1 != idxC)
        { // get next point along edge of obstacle following
          // direction of end-of-prev towards start-of-candidate line
          p2 = p1;
          if (ccv)
            // same order as obstacle index - so increase
            idx1 = (idx1 + 1) % candidate->tangLine->tob->obst->getPointsCnt();
          else if (idx1 == 0)
            // clockwise, so decrease index - i.e. fold to last point
            idx1 = candidate->tangLine->tob->obst->getPointsCnt() - 1;
          else
            // clockwise, so decrease index
            idx1--;
          p1 = candidate->tangLine->tob->obst->getPoint(idx1);
          tentG += hypot(p1.y - p2.y, p1.x - p2.x);
        }
        // add length of new
        dropOpen = (tentG < oi->costG);
        found = not dropOpen;
      }
    }
    // test if the destination vertex of the candidate already is in the closed set.
    if (oi->tangLine->aob == candidate->tangLine->aob)
    {
      found = oi->tangLine->aobIdx[oi->tangIdx] == candidate->tangLine->aobIdx[candidate->tangIdx];
      if (found)
      {
        dropOpen = (candidate->costG < oi->costG);
        found = not dropOpen;
      }
    }
    if (dropOpen)
    {
      if (oi2 == NULL)
        openSet = oi->next;
      else
        oi2->next = oi->next;
      oi->next = NULL;
      recycleLnkSeq(oi);
      oi = oi2;
      if (oi == NULL)
        break;
    }
    if (found)
      break;
    oi2 = oi;
    oi = oi->next;
  }
  return found;
}


////////////////////////////////////////////////

void UAvoidPath2::addToLnkSeqs(UAvoidLnkSeq * ls)
{
  double dist, ang;
  int i;
  const double DIST_COST_LIMIT = 50.0; // more than current best path
//  UAvoidLnkSeq * ls2;
  UPosition p1, p2;
  // evaluate route cost
  dist = ls->getDistance(par->startPose.h,
                                    par->exitPose.h,
                                    &ang);
  ls->costDist = dist;
  ls->costAngle = ang;
  //
  for (i = lnkSeqsCnt; i > 0; i--)
  {
    if (lnkSeqs[i - 1]->costDist > ls->costDist)
    {
      if (i < MAX_LINK_SEQS)
        lnkSeqs[i] = lnkSeqs[i - 1];
      else
      { // top sequence is too bad - recycle
        recycleLnkSeq(lnkSeqs[i - 1]);
        lnkSeqs[i - 1] = NULL;
      }
    }
    else
      break;
  }
  if (i < MAX_LINK_SEQS)
    lnkSeqs[i] = ls;
  else
    recycleLnkSeq(ls);
  if (lnkSeqsCnt < MAX_LINK_SEQS)
    lnkSeqsCnt++;
  // reduce the maximum allowed sequence length
  // (including obstacle edge distances)
  if ((i <= 2 and lnkSeqsCnt > 2) or lnkSeqsCnt == 3)
  {
    maxDistCost = fmin(lnkSeqs[2]->costDist, lnkSeqs[0]->costDist + DIST_COST_LIMIT);
    if (lnkSeqs[2]->costAngle < maxAngleCost)
      maxAngleCost = lnkSeqs[2]->costAngle;
  }
  //
  // debug
//   if (i < MAX_LINK_SEQS)
//   { // added/inserted new sequence
//     printf("new seq %d/%d - cost=%.1fm %.2frad - searched %d tangents\n", i, lnkSeqsCnt, ls->costDist, ls->costAngle, tanSeqCnt);
// /*    ls2=ls;
//     while (ls2 != NULL)
//     {
//       p1 = ls2->getThisEnd();
//       k1 = ls2->tangIdx;
//       p2 = ls2->getOtherEnd();
//       printf(" - %d from %.2fx,%.2fy to %.2fx,%.2fy (dist=%.1fm)\n", k1, p1.x, p1.y, p2.x, p2.y, ls2->costDist);
//       ls2 = ls2->next;
//     }*/
//   }
  // debug end
}

///////////////////////////////////////////////

void UAvoidPath2::findRouteToDest(UAvoidLnkSeq * lsFirst, UAvoidLnkSeq * lsEnd, int n)
{
  bool valid = false;
  UAvoidLnkSeq * ls, *lsCopy;
  UAvoidObst *aog, *aogStart, *aogDest;
  UAvoidLink * aLnk;
  int k, i, idx, i1, i2, ii;
  ULineSegment seg2, seg1;
  bool isStart, isExit, isPoint;
  UPosition p1, p2;
  double a1, a2, ad;
  //
  aog = lsEnd->tangLine->aob;
  aLnk = aog->links;
  // get index to tangent connecting to last obstacle (aob)
  // index 0 and 2 is a CV tangent and index 1 and 3 is CCV
  idx = lsEnd->tangIdx;
  seg1 = lsEnd->tangLine->getTangentLine(idx);
  a1 = seg1.getXYHeading();
  // start at current position
  aogStart = aogs[aogsCnt-2];
  // get pointer to final position (a one point obstacle)
  aogDest = aogs[aogsCnt-1];
  // now try all the possible link lines from the start position
  while (aLnk != NULL)
  { // try all links from this obstacle
    // except if it goes back the source (lsEnd)
    if (lsEnd->tangLine->mirror != aLnk and aLnk->mirror != lsEnd->tangLine)
    { // is it a point obstacle
      isPoint = aLnk->tob->obst->getPointsCnt() == 1;
      //is previous obsacle the start pose
      isStart = lsEnd->tangLine->tob->isStart;
      // is the next obstacle the exit pose
      isExit = aLnk->aob->isExit;
      // debug
      if (isPoint and isExit and isStart)
      { // get this point position
        p1 = aLnk->tob->obst->getPoint(0);
        // get a (first) position of other obstacle
        p2 = aLnk->aob->obst->getPoint(0);
        printf("UAvoidPath2::findRouteToDest: a Point at %.2fx,%.2fy -> %.2fx,%.2fy\n",
               p1.x, p1.y, p2.x, p2.y);
      }
      // debug end
      // each link may have up to 2 (usable) tangent lines to next obstacle
      for (i = 0; i < 2; i++)
      {
        if (idx == 0 or idx == 2)
          // touches this obstacle in a CV tangent, so the exit
          // must touch this obstacle in a CCV tangent.
          k = 2 + i;
        else
          // entry in a CCV tangent, exit must touch CV
          k = i;
        // get tangent (if such exist)
        ls = NULL;
        valid = false;
        if (isPoint and not aLnk->valid[k])
        { // points has less tangents, but are equally valid - try the other
          k = (k + 2) % 4;
        }
        if (aLnk->valid[k])
        { // the tangent line is valid
          // start a sequence
          ls = getEmptyLnkSeq();
          tanSeqCnt++;
          // add the selected tangent line
          ls->tangLine = aLnk;
          ls->tangIdx = k;
          // get source point from last obstacle
          p1 = lsEnd->tangLine->tob->obst->getPoint(lsEnd->tangLine->idx[lsEnd->tangIdx]);
          // get destination point on next obstacle
          p2 = aLnk->aob->obst->getPoint(aLnk->aobIdx[k]);
          //
          // is entry-exit connection in violation of group connections
          valid = not aog->passingNoGoEdge2((lsEnd->tangIdx % 2) == 0,
                       lsEnd->tangLine->aobIdx[lsEnd->tangIdx],
                       aLnk->idx[k], p1, p2, isStart, isExit);
          if (valid)
          { // is it too expensive distance wise
            seg2 = aLnk->getTangentLine(k);
            ls->costDist = lsEnd->costDist + seg2.length;
            valid = ls->costDist <= maxDistCost;
          }
          if (valid)
          { // Test if new tangent is crosses the
            // already established path
            valid = not lsFirst->pathCrossing(seg2);
          }
          a2 = seg2.getXYHeading();
          ad = limitToPi(a2 - a1);
          if (valid)
          { // test for need to touch this obstacle
            if (idx == 1 or idx == 3)
              valid = (ad >= 0.0);
            else
              valid = (ad <= 0.0);
          }
          if (valid)
          { // is it too expensive - angle wise
            ls->costAngle = lsEnd->costAngle + fabs(ad);
            valid = ls->costAngle < maxAngleCost;
          }
          if (valid)
          { // the tangent may not end in current position (redundant but fast test)
            valid = ls->tangLine->aob != aogStart;
          }
          if (valid)
          { // test for going in a loop back to same tangent
            // that is: is the tangent point already in the
            // established sequence.
            i1 = ls->tangLine->aobIdx[ls->tangIdx];
            valid = not
                findRouteInALoop(
                    lsFirst, // start of tangent sequence
                    ls->tangLine->aob, // purposed end obstacle
                    i1, -1); // vertex number
          }
          if (valid)
          { // test also the traveresd range of vertices
            // in the previous obstacle
            // first the entry vertex
            i1 = lsEnd->tangLine->aobIdx[lsEnd->tangIdx];
            // then the exit vertes
            i2 = ls->tangLine->idx[ls->tangIdx];
            if (lsEnd->tangIdx %2 == 0)
            { // the vertices are traversed in decreaing order
              // so swap
              ii = i1;
              i1 = i2;
              i2 = ii;
            }
            valid = not
                findRouteInALoop(
                    lsFirst, // start of tangent sequence
                    ls->tangLine->tob, // last obstacle
                    i1, i2); // vertex numbers (low->high)
          }
        }
        if (valid)
        { // add this tangent and continue
          lsEnd->next = ls;
          if (aLnk->aob == aogDest)
          { // finished -- add as candidate tangent sequence
            //make a copy to save
            lsCopy = copyAvoidLnkSeq(lsFirst);
            addToLnkSeqs(lsCopy);
          }
          else if ((n + 1) < par->maxTangentDepth)
          { // not finished - continue to next
            findRouteToDest(lsFirst, ls, n + 1);
          }
          //
          // everything after lsEnd must be removed to continue
          recycleLnkSeq(lsEnd->next);
          lsEnd->next = NULL;
        }
        else if (ls != NULL)
          // recycle the last tangent only (e.g. found a dead end)
          recycleLnkSeq(ls);
      }
    }
    aLnk = aLnk->next;
  }
}

//////////////////////////////////////////////////////////

bool UAvoidPath2::findRouteInALoop(UAvoidLnkSeq * lsFirst,
                                   UAvoidObst * obstTest,
                                   int edgeIndex1,
                                  int edgeIndex2)
{
  UAvoidLnkSeq * ls;
  int a, b, high, low, n, m;
  bool inLoop = false;
  //
  ls = lsFirst;
  // test all obstacles on the route (except the last, as it lags an exit point)
  while (ls->next != NULL)
  { // test if tangent ends in an obstacle already in the sequence
    if (ls->tangLine->aob == obstTest)
    { // tangents the same object, so the obstacle corner numbers
      // must be tested too.
      // get the entry corner number
      a = ls->tangLine->aobIdx[ls->tangIdx];
      // and the exit corner number
      b = ls->next->tangLine->idx[ls->next->tangIdx];
      // The used corners depend on
      // the tanget entry angle (CV or CCV touch)
      // if CV touch , then the exit point has a lower (or equal) index
      // CV = 0 or 2, i.e. index modulus 2 == 0
      if (ls->tangIdx % 2 == 0)
      {
        high = a;
        low = b;
      }
      else
      {
        high = b;
        low = a;
      }
      n = edgeIndex1;
      if (edgeIndex2 < 0)
        m = n;
      else
        m = edgeIndex2;
      while (true)
      {
        if (high >= low)
          inLoop = (n <= high) and (n >= low);
        else
          inLoop = (n <= high) or (n >= low);
        if (inLoop)
          break;
        if (n == m)
          break;
        n++;
        if (n >= obstTest->obst->getPointsCnt())
          n = 0;
      }
      if (inLoop)
        // no need to continue
        break;
    }
    ls = ls->next;
  }
  // return result
  return inLoop;
}

//////////////////////////////////////////////////////////

UAvoidLnkSeq * UAvoidPath2::copyAvoidLnkSeq(UAvoidLnkSeq * source)
{
  UAvoidLnkSeq *result = NULL, *ls, *dest;;
  //
  ls = source;
  if (ls != NULL)
  {
    result = getEmptyLnkSeq();
    dest = result;
    *dest = *source;
    ls = ls->next;
  }
  while (ls != NULL)
  {
    dest->next = getEmptyLnkSeq();
    dest = dest->next;
    *dest = *ls;
    ls = ls->next;
  }
  return result;
}

////////////////////////////////////

void UAvoidPath2::logObstacleGroups()
{
  int i, j, l, n;
  UAvoidObst * aog;
  UPosition p1, p2;
  ULineSegment seg;
  UAvoidLink * lnk;
  //
  fprintf(logdbg, "Obstacle log\n");
  for (i = 0; i < aogsCnt; i++)
  {
    aog = aogs[i];
    while (aog != NULL)
    {
      p1 = aog->obst->getCogXY();
      n = aog->obst->getPointsCnt();
      fprintf(logdbg, "\n");
      fprintf(logdbg, "---------- obst %d.%lu has %d vertices COG=(%.2fx,%.2fy)\n",
              i, aog->obst->getSerial(), n, p1.x, p1.y);
      for (j = 0; j < n; j++)
      {
        p1 = aog->obst->getPoint(j);
        fprintf(logdbg, "vertice %d    %.3fx,%.3fy\n", j, p1.x, p1.y);
      }
      //
      fprintf(logdbg, "concavity: ");
      for (j = 0; j < n; j++)
        fprintf(logdbg, "%d ", aog->badEdges[j]);
      fprintf(logdbg, "\n");
      fprintf(logdbg, "nogo edge: ");
      for (j = 0; j < n; j++)
        fprintf(logdbg, "%d ", aog->nogoEdge[j]);
      fprintf(logdbg, "\n");
      fprintf(logdbg, "nogo vertx:");
      for (j = 0; j < n; j++)
        fprintf(logdbg, "%d ", aog->nogoVertex[j]);
      fprintf(logdbg, "\n");
      //
      fprintf(logdbg, "No visibility from %d point sets\n", aog->noVisCnt);
      for (j = 0; j < aog->noVisCnt; j++)
      {
        for (l = 0; l < 2; l++)
        {
          seg = aog->noVis[j].getNoVisSegment(l);
          p1 = seg.getOtherEnd();
          fprintf(logdbg, "%d to obst %lu (%.2fx,%.2fy) to (%.2fx,%.2fy)\n",
                  l, aog->noVis[j].aobOther->obst->getSerial(),
                      seg.pos.x, seg.pos.y, p1.x, p1.y);
        }
      }
      //
      // Tangent lines
      lnk = aog->links;
      while (lnk != NULL)
      {
        fprintf(logdbg, "tangent set %d to obst %lu cnt=%d\n", lnk->serial,
                lnk->aob->obst->getSerial(), lnk->tangentCnt);
        for (j = 0; j < lnk->tangentCnt; j++)
        {
          if (lnk->valid[j])
          {
            p1 = aog->obst->getPoint(lnk->idx[j]);
            p2 = lnk->aob->obst->getPoint(lnk->aobIdx[j]);
            fprintf(logdbg, "    k=%d idx=%d (%3.2fx,%3.2fy) to other's idx=%d (%3.2fx,%3.2fy)\n",
                    j, lnk->idx[j], p1.x, p1.y, lnk->aobIdx[j], p2.x, p2.y);
          }
        }
        //
        lnk = lnk->next;
      }
      aog = aog->grp;
    }
  }
  fprintf(logdbg, "---end---\n");
}


////////////////////////////////////

void UAvoidPath2::logPathSequences()
{
  UAvoidLnkSeq * lnkSeq;
  int i, a1, a2, j;
  UPosition p1, p2;
  //
  fprintf(logdbg, "\n");
  fprintf(logdbg, "Tangent Sequence list (max %d and max depth = %d)\n",
            MAX_LINK_SEQS, par->maxTangentDepth);
  for (i = 0; i < lnkSeqsCnt; i++)
  {
    lnkSeq = lnkSeqs[i];
    fprintf(logdbg, "Tangent Sequence %d (distance %.2f angle %.2f)\n", i,
            lnkSeq->costDist, lnkSeq->costAngle);
    j = 0;
    while (lnkSeq != NULL)
    {
      a1 = lnkSeq->tangLine->idx[lnkSeq->tangIdx];
      a2 = lnkSeq->tangLine->aobIdx[lnkSeq->tangIdx];
      p1 = lnkSeq->tangLine->tob->obst->getPoint(a1);
      p2 = lnkSeq->tangLine->aob->obst->getPoint(a2);
      fprintf(logdbg, "   %d tangent %d.%d from obst %lu idx %d (%.2fx, %.2fy) to %lu idx %d (%.2fx, %.2fy)\n", j,
              lnkSeq->tangLine->serial, lnkSeq->tangIdx,
              lnkSeq->tangLine->tob->obst->getSerial(),
                  a1, p1.x, p1.y,
                  lnkSeq->tangLine->aob->obst->getSerial(),
                      a2, p2.x, p2.y);
      //
      lnkSeq = lnkSeq->next;
      j++;
    }
  }
}

///////////////////////////////////

UAvoidPoint * UAvoidPath2::createAvoidPoinsPath(UAvoidLnkSeq * source)
{
  UAvoidPoint * ap;
  int m;
  double dMin;
  UPosition pos, oPos;
  UAvoidPoint * aPointList;
  //
  // create the point list to avoid - from sequence of tangents
  aPointList = createPointList(source);
  // find closest obstacle
  ap = aPointList;
  m = 0;
  dMin = par->getSafeDistance(ap->avoidLeft, false, false, false);
  while (ap != NULL)
  { // test all groups except start and end (not tight and allow turns)
    // except start and exit as these are not relevant
    if (ap->aob->isStart or ap->aob->isExit)
      // reduced search radius
      setClosestPoint(ap, dMin);
    else
      setClosestPoint(ap, dMin * 2.0);
    ap->generation = generation;
    ap = ap->next;
    m++;
  }
  //
  return aPointList;
}

///////////////////////////////////

UAvoidPoint * UAvoidPath2::createPointList(UAvoidLnkSeq * source)
{
  // create point list from link list
  UAvoidPoint * ap, *last = NULL;
  UAvoidPoint * result = NULL;
  UAvoidLnkSeq * aSeq = source;
  int idx1, idx2, k1, k2, i1;
  UAvoidObst * aob;
  bool cvLeft, srcIsPoint;
  double a1, a2 = 0.0;
  // safe radius for initial estimate
//  double radSafe = 1.0;
//  const int MSL = 200;
//  char s[MSL];
  // start pose with speed estimated turn radius
  ap = getEmptyPoint();
  // insert data
  // used tangent line (out of 4)
  k1 = aSeq->tangIdx;
  ap->aPos = aSeq->tangLine->getThisEnd(k1);
  ap->avoidLeft = true; // not relevant for first point (yet)
  ap->aob = aSeq->tangLine->tob;
  // debug
  if (ap->aob == NULL)
  {
    printf("UAvoidPath2::createPointList (1) This is an error, any tangent should have a related object\n");
    ap->aPos.print("-- at this position");
  }
  // debug end
  //ap->radius = 0.0;
  // set as start and end of link
  last = ap;
  result = ap;
  // continue until no more tangents
  while (true)
  { // is point to be passed left or right - tested later if point
    cvLeft = (k1 % 2 == 0);
    // add other end of link line
    ap = getEmptyPoint();
    ap->followLineLastPose = false;
    ap->setAvoidObst(aSeq->tangLine->aob, aSeq->tangLine->aobIdx[k1], cvLeft);
/*    ap->aPos = aSeq->tangLine->getOtherEnd(k1);
    ap->aob = aSeq->tangLine->aob;
    ap->avoidLeft = cvLeft;
    ap->radius = radSafe;*/
    // debug
    if (ap->aob == NULL)
    {
      printf("UAvoidPath2::createPointList (2) This is an error, any tangent should have a related object\n");
      ap->aPos.print("-- at this position");
    }
    // debug end
    srcIsPoint = last->aob->obst->getPointsCnt() == 1;
    if (last->prev != NULL)
      a2 = last->prev->angNext;
    // add to end of list
    last = last->insertAfter(ap);
    if (srcIsPoint and last->prev->prev != NULL)
    { // avoid left or right depends on angles in and out.
      a1 = last->prev->angNext;
      a1 = limitToPi(a1 - a2);
      last->prev->avoidLeft = a1 < 0.0;
    }
    //
    // now add all points along next obstacl vertices as points too
    idx1 = aSeq->tangLine->aobIdx[k1]; // vertex index on destination obstacle
    aob = aSeq->tangLine->aob; // destination obstacle (same as source obstacle on next link)
    // get next link in sequence
    aSeq = aSeq->next;
    if (aSeq == NULL)
      // finished
      break;
    // there is an exit tangent - continue to that
    k2 = aSeq->tangIdx; // exit tangent line index (out of the 4 possible)
    idx2 = aSeq->tangLine->idx[k2]; // vertex index on source (this) obstacle
    // if entry and exit vertex is different, then follow obstacle edges
    i1 = idx1;
    while (i1 != idx2)
    { // need to follow obstacle edge to exit vertex
      // move to next point (entry point is covered already)
      if (cvLeft)
      { // clockwise
        i1--;
        if (i1 < 0)
          i1 = aob->obst->getPointsCnt() - 1;
      }
      else
      { // counter clockwise
        i1++;
        if (i1 >= aob->obst->getPointsCnt())
          i1 = 0;
      }
      ap = getEmptyPoint();
      ap->followLineLastPose = false;
      ap->setAvoidObst(aob, i1, cvLeft);
/*      ap->aPos = aob->obst->getPoint(i1);
      ap->avoidLeft = cvLeft;
      ap->radius = radSafe;
      ap->aob = aob;*/
      // angle to this node
//      last->angNext = atan2(ap->aPos.y - last->aPos.y, ap->aPos.x - last->aPos.x);
      // link to list
      last = last->insertAfter(ap);
    }
    // add start point of next tangent
    k1 = k2;
  }
  // last heading is destination heading
  ap->angNext = par->exitPose.h;
  ap->followLineLastPose = par->followLineLastPose;
  // debug
//   ap = result;
//   i1 = 0;
//   while (ap != NULL)
//   {
//     i1++;
//     ap->aPos.snprint("aPos", s, MSL);
//     printf("#%d: %s - avoidLeft=%s angle=%.2fdeg\n", i1, s, bool2str(ap->avoidLeft), ap->angNext);
//     //
//     ap = ap->next;
//   }
  // debug end
  return result;
}

///////////////////////////////////////////////////////////////

bool UAvoidPath2::createMidPathPoints(UAvoidPoint * ptLstBase,
                                      bool * ommitBase, bool * movedBase)
{
  bool result = true;
  UAvoidPoint * wp1 = NULL; // ap1 is more forward than ap2
  UAvoidPoint * wp2 = NULL; // earlier point on path
  double avTurn; // avoid distance during a turn - robot centre to obst point
  double avNoTurn; // avoid distance robot centre to obstacle
  double mtr; // minimum physical turn radius - should not be 0
  double b, c, d, turnDist, dx, dy, da = 0.0, dda, hp, hpd, rp; //, diag;
  UAvoidPoint * apNew = NULL;
  UPosition po1, po2;
  bool doAddPoint;
  U2Dseg seg;
  U2Dpos pos;
  bool /*facingTPos,*/ knifeOpening, opposingFirst;
  UPose mid, apose;
  enum SolutionType {none, removeAp1, removeAp2, freeSpace, freeSpace1,
                     firstPointFwd,
                     centrePoint, centrePointRev, centreMovedBack,
                     prePointKnife, knifeReverseOpening, knifeNoPrePoint,
                     prePointMother, prePointFlat, prePointDropped, lastPrePoint,
                     finalPointDirect};
  SolutionType solution = none;
  bool omegaTurn = false;
  bool midPointFailed;
  double turnCentreRadius;
  //
  if (ptLstBase == NULL)
    return false;
  //
  // start with first point after the start pose
  if (ptLstBase->prev == NULL)
    // first point is start pose - set fake mid point
    ptLstBase->mid = par->startPose;
  wp2 = ptLstBase;
  int i = 0;
  while (wp2 != NULL)
  {
    wp2->serial = i++;
    wp2 = wp2->next;
  }
  wp2 = ptLstBase;
  wp1 = wp2->next;
  //
  // debug
  // printf("midPoint from %d (mid=%6.2fx,%6.2fy,%4.2fh)\n", ap2->serial, ap2->mid.x, ap2->mid.y, ap2->mid.h);
  // debug end
  // geometric safety distance to turn centre
  avNoTurn = par->getSafeDistance(wp1->avoidLeft, true, true, true);
  // get minimum turn radius - frem control settings
  mtr = par->getMinTurnRad();
  // turnCentreRadius
  turnCentreRadius = fmax(mtr, avNoTurn);
  *ommitBase = false;
  *movedBase = false;
  //
  // find manoeuvre points for all remaing obstacle points
  while (wp1 != NULL)
  {
    // debug
    if (wp1->serial > 0)
      printf("debug %d\n", wp1->serial);
    else
      printf("debug serial = %d\n", wp1->serial);
    // debug end
    // investigating point ap1
    // use loose rules if possible (open corners)
    //tight = false;
    doAddPoint = false;
    omegaTurn = false;
    solution = none;
    turnDist = 1e3;
    midPointFailed = false;
    //
    // use first rule - this will succede if in open space, or if sufficient room for manoeuvres
    // 1. Space for manoeuvres
    // 2. Not space for free manoeuvres
    //
    // 1. Space for manoeuvres
    // - 1.a fully open - keep desired distance to obstacle
    //   1.a.a half angle between approaching and destination angles, when
    //         fine space from last and next obstacle point
    //   1.a.b allign with approach, when previous point is very close
    //   1.a.c allign with next, when next point is wery close
    // - 1.b Tight - the turn centre is alligned for best clearence in narrow opening
    //             The available clearance space is divided, to maximum clearence, until
    //             not enough space for minimum clearence (then it fails)
    //   1.c tight, but leaving tight area - tightest point is close to robot, but
    //              avoid-point is further froward
    // pre-points are not recalculated.
    result = false;
    while (not result)
    { // this point may be may calculated (or recalculated)
      if (wp1->next != NULL and not wp2->prePoint)
      {// set turn centre with best margin and an appropriate angle
        // relative to obstacle point (returns false if obstacle)
        result = setTurnCentre(wp1, turnCentreRadius, &turnDist);
        //
        if (not result and wp2->prev == NULL and wp1->isTight())
        { // leaving a tight area - test
          // get tightest mid-point
          po1.x = (wp1->oPos.x + wp1->tPos.x)/2.0;
          po1.y = (wp1->oPos.y + wp1->tPos.y)/2.0;
          // get mid-point in current robot coordinates
          po2 = wp2->mid.getMapToPose(po1);
          if (fabs(po2.x) < turnDist and fabs(po2.y) < turnDist)
          { // this is a bad plave to put a mid-pose, so use
            // turn centre at aPoint if better
            po1 = wp2->mid.getMapToPose(wp1->aPos);
            if (po1.x > po2.x)
            { // this should be better - use
              // calculated turn centre as is.
              result = true;
            }
          }
        }
        // Turn Center crossing test
        // if too close to last point, then turn centres may be crossing, and
        // thus make manoeuvre impossibel - test
        if (result and
            wp1->avoidLeft == wp2->avoidLeft and
            wp1->aPos.dist(wp2->aPos) < (turnCentreRadius - avNoTurn) * 2.0)
        { // there is a potential, that the line from
          // center to obstacle point for ap1 crosses that for ap2.
          // if so, then the tangent line of the circles will be on the back.
          // And if so, one point is ignorable.
          // if ap2 is start pose, then this might not be calculated yet.
          if (wp2->prev == NULL)
            // just to set turn centre for ap2
            setMidPoint(wp1, turnCentreRadius);
          else
          { // else test for impossible plan - too tight manoeuvres
            result = turnCentreCrossing(wp1, wp2, par->obstClearanceMinimum);
            if (result)
            { // remove the last point (ap1)
              solution = removeAp1;
              // debug
              //printf("Turn-centre crossing ap1(%.2fx,%.2fy) ap2(%.2fx,%.2fy)\n",
              //       ap1->mCent.x, ap1->mCent.y, ap2->mCent.x, ap2->mCent.y);
              // debug end
              // adjust ap2 mid-point to a compromice between the two
              wp2->mCent.x = (wp2->mCent.x + wp1->mCent.x) / 2.0;
              wp2->mCent.y = (wp2->mCent.y + wp1->mCent.y) / 2.0;
              wp2->mCent.h = wp1->mCent.h + limitToPi(wp2->mCent.h - wp1->mCent.h) / 2.0;
              // the mid-point needs to be recalculated for ap2.
              result = setMidPoint(wp2, turnCentreRadius);
              if (result and wp2 == ptLstBase)
                // need to inform caller
                *movedBase = true;
              //result = true;
            }
            else
              // not crossing - that is the easy solution
              result = true;
          }
        }
      }
      if ((result or wp1->next == NULL) and (solution != removeAp1))
      { // space for turning - and trurn centre set OK, (re)calculate
        // targetpoint from last pose.
        if (wp1->prev == NULL)
          printf("A bit unexpected that ap1->prev == NULL\n");
        else
        {
          result = setMidPoint(wp1, turnCentreRadius);
          if (not result and wp1->next == NULL)
          { // failed to get to
            solution = finalPointDirect;
            result = true;
          }
          else if (result)
            solution = freeSpace;
          else
            midPointFailed = true;
        }
      }
      if (wp1->isTight() or result)
        break;
      wp1->useTight = true;
    }
    //
    if (wp2->prePoint)
    { // pre-point is in place, so we just need to adjust mid-point to already calculated pre-point.
      solution = prePointMother;
      result = setMidPoint(wp1, turnCentreRadius);
    }
    if (midPointFailed and wp2->prev == NULL)
    { // we could not palace a mid-point just after current position
      // try a solution where current heading is maintained (without hitting the obstacle)
      wp2->setTurnCentreFromMidPose(turnCentreRadius, wp1->avoidLeft);
      // set line to point in the direction of the solution.
      seg.setFromPose(wp2->mCent.x, wp2->mCent.y, wp2->mid.h);
      // must keep a distance to the obstacle point that ensures arc is avNoTurn from the obstacle.
      result = seg.getCircleCrossings(wp1->aPos.x, wp1->aPos.y, turnCentreRadius - avNoTurn, &b, &c);
      d = fmin(b, c);
      if (not result)
      { // circle is too small - move center to align with aPos (obstacle vertex)
        pos = wp2->mid.getMapToPose(wp1->aPos);
        result = (pos.x > 0);
        d = pos.x;
      }
      if (result and d <= 0.0)
      { // this obstacle will never be hit due to turn radius, so
        // ignore point.
        solution = removeAp1;
        result = true;
      }
      else if (result)
      {
        pos = seg.getPositionOnLine(d + 0.005);
        wp1->mCent.x = pos.x;
        wp1->mCent.y = pos.y;
        result = setMidPoint(wp1, turnCentreRadius);
        if (result)
          solution = firstPointFwd;
      }
      else
      { // trying to move backwards in a tight opening - this is not good
        // or turning 180 deg - that is handled later.
/*        printf("Trying to move back in a tight opening - not implemented yet!\n");
        break;*/
      }
    }
    if (not result and turnDist > avNoTurn and not wp1->isFreeSpace())
    { // Some manoeuvre space is available, so
      // there is two possibilities
      // 2.1 (one point)   hold the center of a corridor - not much heading change, so may opening may be narrow
      // 2.2 (prePoint)  add a pre-point before narrow opening to limit turn inside opening.
      //
      //
      // 2.1 one point
      // 2.1.a centrePoint. Passing centre point - mid-point before opening.
      // 2.1.b centrePointRev. Passing centre point - mid-point after opening.
      //
      // get opening type
      /*facingTPos =*/ wp1->tPointVisible(&knifeOpening, &opposingFirst);
      // calculate how much turn angle the opening allows
      // dy is the extra (available) space ex clearence
      // dx is the distance to front corner
      dy = turnDist - avNoTurn;
      if (wp1->avoidLeft)
        dx = par->frontLeft.x;
      else
        dx = par->frontRight.x;
      // 'da' is allowd turn angle to fill the available space
      da = atan2(dy, dx);
      // let angle be allowed angle (relative to perpendicular)
      // include sign (negative if opposingFirst and avoidLeft and
      //               negative if not opposingFirst and not avoidLeft
      if (wp1->avoidLeft)
        da = -da;
      if (not opposingFirst)
        da = -da;
      // set a mid point at the narrow passage with the
      // allowed extra heading.
      // hpd is the angle from obstacle to other side of opening
      hpd = atan2(wp1->oPos.y - wp1->tPos.y, wp1->oPos.x - wp1->tPos.x);
      // rp is the distance across
      rp = hypot(wp1->oPos.y - wp1->tPos.y,
                          wp1->oPos.x - wp1->tPos.x);
      // Get direct direction out of opening
      if (wp1->avoidLeft)
        // get heading of most direct path between these obstacles
        hp = limitToPi(hpd - M_PI / 2.0);
      else
        hp = limitToPi(hpd + M_PI / 2.0);
      // there is cases where t-point is first, but where
      // robot do not reach an angle to face the t-point. In
      // these cases the optimal angle delta (da) should turn towards obstacle, not away.
      if (opposingFirst and 
          ((limitToPi(hp - wp1->prev->mid.h) < 0.0 and wp1->avoidLeft) or
           (limitToPi(hp - wp1->prev->mid.h) > 0.0 and not wp1->avoidLeft)))
      { // just continue to follow the obstacle
        da = -da;
      }
      // get passing point - ideally for optimal turn space
      if ((da > 0.0) == wp1->avoidLeft)
      { // keep as far from opposing obstacle as possible, as we are
        // turning towards this obstacle
        mid.x = wp1->oPos.x + (wp1->tPos.x - wp1->oPos.x)*turnDist/rp;
        mid.y = wp1->oPos.y + (wp1->tPos.y - wp1->oPos.y)*turnDist/rp;
      }
      else
      { // keep as far from this obstacle as possible, as we are approaching this obstacle
        mid.x = wp1->tPos.x - (wp1->tPos.x - wp1->oPos.x)*turnDist/rp;
        mid.y = wp1->tPos.y - (wp1->tPos.y - wp1->oPos.y)*turnDist/rp;
      }
      mid.h = limitToPi(hp + da);
      // test if behind last pose, could happend during
      // close obstacle recalculation
      apose = mid.getMapToPosePose(&wp2->mid);
      if (apose.x > -par->obstClearanceMinimum and
          fabs(apose.y) < avNoTurn)
      { // remove the surplus point
        if (wp1->prev != ptLstBase and wp2->prev != NULL)
        {  // a better solution is to remove the previous point, as
          // ap1 were acceptable in the old solution
          // debug
          // printf("too close - remove ap2\n");
          // debug end
          solution = removeAp2;
        }
        else
        { // this is first point, or previous point is start pose,
          // so remove ap1
          // debug
          // printf("too close - remove ap1\n");
          // debug end
          solution = removeAp1;
        }
        result = true;
      }
      if (not result)
      { // not finished - continue preparation for placement of ap1.mid point.
        //
        // maybe this waypoint can be positioned with no extra point
        // when passing the mid-point of opening
        if (not wp1->prePoint)
        { // set new turn centre from mid-point (but never for a pre-point)
          // mid-point in corridor and and heading at right angle to opening
          wp1->mCent.x = (wp1->aPos.x + wp1->oPos.x)/2.0;
          wp1->mCent.y = (wp1->aPos.y + wp1->oPos.y)/2.0;
          wp1->mCent.h = hpd; // heading towards opposing point
          wp1->mCent.add(-turnCentreRadius, 0.0);
          c = wp1->mCent.getDistance(wp1->aPos);
          if (opposingFirst and c > (turnCentreRadius - avNoTurn))
          { //This may hit the aPos when turning - move mCent forward towards aPos
            if (wp1->avoidLeft)
              seg.setFromPose(wp1->mCent.x, wp1->mCent.y, wp1->mCent.h - M_PI / 2.0);
            else
              seg.setFromPose(wp1->mCent.x, wp1->mCent.y, wp1->mCent.h + M_PI / 2.0);
            // must keep a distance to the obstacle point that ensures arc is avNoTurn from the obstacle.
            if (not seg.getCircleCrossings(wp1->aPos.x, wp1->aPos.y,
                                           turnCentreRadius - avNoTurn, &b, &c))
            { // circle do not touch line - most likely
              //   as (turnCentreRadius - avNoTurn) is too small
              //   so move the full distance - should always be OK, as obstacle is conveks
              b = c;
            }
            pos = seg.getPositionOnLine(fmin(b, c) + 0.005);
            wp1->mCent.x = pos.x;
            wp1->mCent.y = pos.y;
          }
        }
        //else
        //  // debug
        //  printf("passable test at used angle ... ");
        // find resulting mid-point
        if (not setMidPoint(wp1, turnCentreRadius))
        { // failed to set mid - point - what to do now
          printf("failed to set midpoint for ap1! - what now\n");
        }
        else
        { // possible to set mid-point to center, but
          // is it usefull
          if (wp1->prePoint)
            d = wp1->oPos.dist(wp1->next->oPos);
          else
            d = 1e5;
          if (d < par->obstClearanceMinimum)
          { // same obstacle for pre-point mother and pre-point, so pre-point solution is OK
            solution = lastPrePoint;
            result = true;
          }
          else
          { // need passability check
            // calculate relative passage angle and ability to pass (up to almost a right angle)
            dda = fmin(fabs(limitToPi(hp - wp1->mid.h)), M_PI / 2.1);
            // get (half) space used in opening at this approach angle
            avTurn = par->getSafeDistance(wp1->avoidLeft, false, true, false);
            d = fmin(avNoTurn / cos(dda), avTurn);
            // get outher half space for manoeuvre
            if (wp1->prePoint)
            { // prepoints go as close to aPos as possible
              c = wp1->otDist - avNoTurn - par->obstClearanceMinimum * 0.5;
              // if turn cente is not on the obst-aPos line,
              // then allow obstacle to be flat, and reduce available space
              c -= (1.0 - cos(wp1->mCent.h - hpd)) * (turnCentreRadius - avNoTurn);
            }
            else
              // other cases uses mid-point (centre of opening)
              c = wp1->otDist / 2.0;
            if (d < c)
            { // space for this manoeuvre (except for offencive turns
              // that are to be handled using footprint envolope)
              // debug
              // printf(" ... [OK] had %.2fm used %.2f\n", c, d);
              // debuge end
              if (wp1->prePoint)
              {
                solution = lastPrePoint;
                result = true;
              }
              else if ((wp1->avoidLeft and dda <= 0.0 and -dda < fabs(da)) or
                (not wp1->avoidLeft and dda >= 0.0 and dda < fabs(da)))
              { // approach angle is small - will not offend opening, and
                // touch point is before opening, this is OK -(if continued turn along this
                // turn centre
                solution = centrePoint;
                result = true;
              }
              else
              { // touching at the exit side, move turn
                // centre a bit back to allow passage at centre of opening
                d = (1.0 - cos(dda)) * turnCentreRadius;
                if (d > (wp1->otDist / 2.0 - avNoTurn))
                  // limit the movement, so that the tPos is not hit
                  d = wp1->otDist / 2.0 - avNoTurn;
                wp1->mCent.add(-d, 0.0);
                setMidPoint(wp1, turnCentreRadius);
//                d = avNoTurn * 2.0 / cos(hp - ap1->mid.h);
//                if (d < ap1->otDist)
                { // solution is OK for the opening
                  solution = centrePointRev;
                  result = true;
                }
              }
              d = par->obstClearanceMinimum * 4.0;
              if (wp1->prePoint)
                // reduce to minimum
                d = par->obstClearanceMinimum;
              if (result and turnCentreCrossing(wp1, wp2, d))
              { // this brings the turn-centres too close, and this result in too
                // exaturated manoeuvres (circle crossing too far from optimal point.
                // debug
                // printf("Turn-centre crossing ap1(%.2fx,%.2fy) ap2(%.2fx,%.2fy)\n",
                //       ap1->mCent.x, ap1->mCent.y, ap2->mCent.x, ap2->mCent.y);
                // debug end 
                solution = removeAp1;
                result = true;
              }
            }
            //else
            //  // debug
            //  printf(" ... [failed] had %.2fm need %.2f\n", c, d);
          }
        }
      }
    }
    else if (turnDist <= avNoTurn)
    {
      printf("No space for a turn!, so stop here!\n!");
      wp2->next = NULL;
      recyclePoint(wp1);
      result = true;
      break;
    }
    //
    if (not result)
    { // There is now two pre-point possibilityes
      // for the extra point
      // 2.1 tight manoeuvre in need for an pre-point (an omega turn)
      // 2.2 Knife opening in need for a pre-point;
      //
      // an extra point is needed to approach at an OK angle to avoid
      // opposing obstacle
      // get the extra waypoint
      apNew = getEmptyPoint();
      // debug
      // printf("got new point serial %d - in createMidPathPoints\n", apNew->serial);
      // debug end
      if (wp1->isFreeSpace())
      { // free space - so must need a cross tangent
        apNew->avoidLeft = not wp1->avoidLeft;
        apNew->aob = wp1->aob;
        apNew->aPos = wp1->aPos;
      }
      else
      { // is tight space (knife opening)
        apNew->avoidLeft = da > 0.0;
        if (apNew->avoidLeft == wp1->avoidLeft)
        {
          apNew->aob = wp1->aob;
          apNew->aPos = wp1->aPos;
        }
        else
        {
          apNew->aob = wp1->oob;
          apNew->aPos = wp1->oPos;
        }
      }
      apNew->prePoint = true;
      apNew->generation = generation;
      apNew->followLineLastPose = false;
      doAddPoint = true;
      // manoeuvre distance on outside of path
      avTurn = par->getSafeDistance(wp1->avoidLeft, false, true, false);
      // I.  - hit an isolated obstacle - try an omage expansion
      // II. - a knife opening
      if (wp1->isFreeSpace())
      { // we are in need of a pre-point - probably an omega turn
        solution = freeSpace1;
      }
      else // if (not ap1->isFreeSpace())// a knifeOpening
      { 
        if (not wp1->prePoint)
        { // set turn centre from desired mid-point (pre-points has OK turn centre
          wp1->mid = mid;
          // set also associated turn centre
          wp1->setTurnCentreFromMidPose(turnCentreRadius);
        }
        // avTurn is needed for a full turn - from turn-centre to obstacle
        // po1 is new fake obstacle position and is relative to ap1->mid pose
        b = wp1->mCent.getDistance(wp1->oPos);
        hpd = atan2(wp1->oPos.y - wp1->mCent.y, wp1->oPos.x - wp1->mCent.x);
        c = avTurn + turnCentreRadius + par->obstClearanceMinimum;
        if (apNew->avoidLeft == wp1->avoidLeft)
          po1.x = triangleFinda(b, c, hpd - mid.h + M_PI);
        else
          po1.x = triangleFinda(b, c, hpd - mid.h);
        //
        doAddPoint = po1.x > 0.0;
        if (doAddPoint)
        {
          if (wp1->prePoint)
          {
            apose = wp1->mCent;
            apose.h = mid.h;
            po1.y = 0.0;
          }
          else
          {
            apose = wp1->mid;
            if (apNew->avoidLeft)
              po1.y = -turnCentreRadius;
            else
              po1.y =  turnCentreRadius;
          }
          apNew->mCent = apose.getPoseToMapPose(-po1.x, po1.y, 0.0);
          //
          if (wp1->avoidLeft != apNew->avoidLeft)
            // different arc directions, so find cross tangent point
            result = crossTangent(apNew->mCent, wp1->mCent, not apNew->avoidLeft, turnCentreRadius, &apose);
          else
            // same arc direction, so find outher tangent
            result = outherTangent(apNew->mCent, wp1->mCent, not apNew->avoidLeft, turnCentreRadius, &apose);
          //
          if (not result)
            printf("pre-point - set direction to a-point failed - should never fail\n");
          //
          apNew->mCent.h = atan2(apose.y - apNew->mCent.y, apose.x - apNew->mCent.x);
          apNew->aPos = apNew->mCent.getPoseToMap(turnCentreRadius - avNoTurn, 0.0);
          // set direction for potential omega expansion
          apNew->mCent.h = wp1->mid.h;
          //
          solution = prePointKnife;
        }
        else
        { // extra point is not needed
          if (wp1->avoidLeft == apNew->avoidLeft)
          { //the pre-point is just not needed
            //setTurnCentre(ap1, turnCentreRadius, NULL);
            result = setMidPoint(wp1, turnCentreRadius);
            if (not result)
            {
              printf("*** knifeNoPrePoint failed to set mid-point - bail out\n");
              break;
            }
            solution = knifeNoPrePoint;
          }
          else
          { // the other side of opening is to be negosiated rather than the one in ap1.
            // replace - no new point
            wp1->aob = apNew->aob;
            wp1->avoidLeft = apNew->avoidLeft;
            wp1->aPos = apNew->aPos;
            setClosestPoint(wp1, avTurn * 2.0);
            // update angle and mid-point from last
            wp2->setAngNext();
            wp1->setAngNext();
            solution = knifeReverseOpening;
          }
        }
        result = true;
        //
      }
    }
    // add rest of management
    if (doAddPoint and solution != knifeReverseOpening)
    {
      result = insertNewPointAfter(wp1, apNew);
      if (result)
        apNew = NULL;
      else
      { // may be better off without new mid-point - let actual manoeuvre decide
        // debug
        //printf("new prepoint with obst-pos at %.2fx,%.2fy failed - try without\n",
        //        apNew->aPos.x, apNew->aPos.y);
        // debug end
        wp2->unlinkNext();
        solution = prePointDropped;
      }
    }
    if (apNew != NULL)
    {
      recyclePoint(apNew);
      if (solution == none)
        solution = prePointDropped;
      result = true;
      apNew = NULL;
    }
    //
    if (not result and wp1->prePoint)
    { // it may be possible to move pre-point further away from tight entrance
      // to avoid turn circles to overlap
      result = expandOmegaTurn(wp1->mCent, wp2->mCent, turnCentreRadius, wp1->next->mid.h, &wp1->mCent);
      if (result)
        // try again
        result = setMidPoint(wp1, turnCentreRadius);
      if (result)
      {
        omegaTurn = true;
      }
    }
    //
    // new point may be too close to previous point
    if (result and
        wp1->next != NULL and
        not wp2->aob->isStart and
        solution != removeAp1 and
        solution != removeAp2 and
        wp2->next == wp1)
    { // new points not added, so test 
      po2 = wp2->mid.getMapToPose(wp1->mid);
      d = hypot(po2.x, po2.y);
      if (d < par->obstClearanceMinimum * 2.0 or
            (po2.x < 0.0 and
            not wp2->aob->isStart and
            d < turnCentreRadius))
      { // remove most forward avoid point
        solution = removeAp1;
        if ((wp1->avoidLeft == wp2->avoidLeft) and
            ((po2.y > 0 and wp1->avoidLeft) or
              (po2.y < 0 and not wp1->avoidLeft)))
          // except if previous point is closer to obstacle
          solution = removeAp2;
        // debug
/*        if (solution == removeAp1)
          printf("too close - remove ap1 ap2(ap1)=%.2fx,%.2fy\n", po2.x, po2.y);
        else
          printf("too close - remove ap2 ap2(ap1)=%.2fx,%.2fy\n", po2.x, po2.y);*/
        // debug end
        
      }
    }
    if (solution == removeAp1)
    { // this point is too close to make any sence
      // remove the last point (ap1)
      // ap1->printPoses("ap1:", " removed\n");
      if (wp2 != wp1->prev)
        printf("ap1 is not ap2->next - we are removing the wrong guy!\n");
      wp2->unlinkNext();
      recyclePoint(wp1);
      // result OK so far
      result = true;
    }
    if (solution == removeAp2)
    { // this point is too close to make any sence
      // remove the previous point (ap2)
      if (wp2 == ptLstBase)
      { // the root of this search is to be ommitted,
        // but needs to be done by the caller, so inform and exit.
        // ap2->printPoses("ap2:", " to be removed by caller\n");
        *ommitBase = true;
        result = false;
      }
      else
      { // just remove
        // ap2->printPoses("ap2:", " removed\n");
        wp1 = wp2->prev;
        wp1->unlinkNext();
        recyclePoint(wp2);
        wp2 = wp1;
        result = true;
      }
    }
    // failed to place a manoeuvre point -
    // we have to bail out, the route is useless
    if (not result)
      break;
    //
    solutionCnt++;
    // debug
    if (true)
    {
      if (wp1 != NULL)
        if (wp1->serial > 0)
          printf("(ap1:%d) sol %d ", wp1->serial, solutionCnt);
      switch(solution)
      { // 
        case none:          wp1->printPoses(":", " none - (error)\n"); break;
        case removeAp1:     printf(": removed ap1 (turn centre crossing or point before last point)\n"); break;
        case removeAp2:     printf(": removed ap2 (added ap2 too close to next)\n"); break;
        case freeSpace:     wp1->printPoses(": ", " free space\n"); break;
        case firstPointFwd: wp1->printPoses(": ", " first point moved forward\n"); break;
        case freeSpace1:    wp1->printPoses(": ", " free space 1\n"); break;
        case centrePoint:   wp1->printPoses(": ", " centre point\n"); break;
        case prePointMother:                wp1->printPoses(": ", " pre-point mother (kept as is)\n"); break;
        case centrePointRev:                wp1->printPoses(": ", " centre point - reversed centre\n"); break;
        case centreMovedBack:                wp1->printPoses(": ", " centre moved back\n"); break;
        case knifeReverseOpening:                wp1->printPoses(": ", " knifeReverseOpening - no pre point - reversed o<->a\n"); break;
        case lastPrePoint:   wp1->printPoses(": ", " last or OK prepoint\n"); break;
        case knifeNoPrePoint:                wp1->printPoses(": ", " knife - no point\n"); break;
        case prePointKnife:  wp1->printPoses(": ", " knife opening - pre point\n"); break;
        case prePointFlat:   wp1->printPoses(": ", " Along flat obstacle - pre point\n"); break;
        case prePointDropped: wp1->printPoses(": ", " prePointDropped\n"); break;
        case finalPointDirect: wp1->printPoses(": ", " final point usees line drive\n"); break;
        default: printf(": Unknown solution\n"); break;
      }
      if (omegaTurn)
        printf("  -- omega expanded\n");
    } 
    // debug end
    if (solutionCnt >= par->acceptAfterSolution)
    {
      result = true;
      break;
    }
    if (wp1->prePoint and wp1->isFreeSpace())
       // if this is a prepoint during recalculation, so the remaining forward points are unchanged
       break;
    if (*movedBase)
      // need recalculation
      break;
    // debug
    if (wp1 != wp2->next)
      // this should be simple - just retry with same ap2
//      printf("Replace next if with a simple ap1 = ap2->next! OK?\n");
    // debug end
//    if (solution == prePointKnife and not ap2->next->isFreeSpace())
    { // inserted a new pre-point that has an obstacle that needs to be negosiated too
      // redo with this new obstacle
      wp1 = wp2->next;
    }
    else // if (solution != knifeReverseOpening)
    { // not a redo of avoid point ap1 (with a<->o point swap), so continue
      wp2 = wp1;
      wp1 = wp1->next;
    }
  }
  if (false and not result)
  {
    if (*ommitBase)
      printf("delete ap2 and call again\n");
    else
      printf("UAvoidPath2::createMidPathPoints: failed - no possible mid-point - failed towards %d point from start\n", solutionCnt);
  }
  //
  // this is the end of this generation of mid-poses
  generation++;
  return result;
}

///////////////////////////////////////////////////////////////

bool UAvoidPath2::insertNewPointAfter(UAvoidPoint * ap1, UAvoidPoint * apNew)
{
  UAvoidPoint * ap2;
  bool result;
  double avTurn, avNoTurn, turnCentreRadius;
  UPose rel, c2;
  double dir, a1, a2;
  U2Dseg seg;
  U2Dpos pos;
  //
  turnCentreRadius = par->getMinTurnRad();
  avTurn = par->getSafeDistance(ap1->avoidLeft, false, true, false);
  avNoTurn = par->getSafeDistance(ap1->avoidLeft, true, true, true);
  // add position of other obstacles within safe turn distance
  // both ways - to be safe double turn distance
  setClosestPoint(apNew, avTurn * 2.0);
  // inser into point sequence
  ap2 = ap1->prev;
  ap2->insertAfter(apNew);
  // debug
  // printf("Inserted %d in series %d->%d->%d\n", apNew->serial, ap2->serial, ap2->next->serial, ap1->serial);
  // debug end
  //
  if (ap1->isFreeSpace())
  { // try to place a new point as an omega extension
    a2 = ap2->mid.h - M_PI;
    if (ap1->angNext < a2 and ap1->avoidLeft)
      a1 = ap1->angNext + 2.0 * M_PI;
    if (ap1->angNext > a2 and not ap1->avoidLeft)
      a1 = ap1->angNext - 2.0 * M_PI;
    else
      a1 = ap1->angNext;
      // average is towards obstacle
    dir = limitToPi((a1 + a2) / 2.0);
    // try omega turn expansion of turn centre - away from direction d
    // place apNew.mCent at the mid-point between previous mid-point and comming mid-point (assuming
    // comming mid-point is plased on line from last to coming a-point (turnCentre)
    seg.setFromPoints(ap2->mid.x, ap2->mid.y, ap1->aPos.x, ap1->aPos.y);
    pos = seg.getPositionOnLine((seg.length - avNoTurn) / 2.0);
    apNew->mCent.x = pos.x;
    apNew->mCent.y = pos.y;
    //apNew->mCent.h = d;
    c2 = ap1->mCent;
    result = false;
  }
  else
  { // try set mid-point
    setMidPoint(ap1, turnCentreRadius);
    result = setMidPoint(apNew, turnCentreRadius);
    // new point must be behind destination point - else the turn-centres are crossed
    if (result)
    { // discard if pre-point is in front (crossed turn centres)
      rel = ap1->mid.getMapToPosePose(&apNew->mid);
      result = rel.x < 0.0;
    }
    c2 = ap2->mCent;
    dir = apNew->mCent.h;
  }
  // try an omega turn if not finished yet
  if (not result)
  { // it may be possible to move the new pre-point further away from tight entrance
    // to avoid turn circles to overlap
      result = expandOmegaTurn(apNew->mCent, c2, turnCentreRadius, dir, &apNew->mCent);
    if (result)
    { // try again to set mid point (should succeed)
      result  = setMidPoint(ap1, turnCentreRadius);
      result &= setMidPoint(apNew, turnCentreRadius);
      // debug
      if (result)
        printf("---omega expanded point %d to new turn centre at %.2fx,%.2fy\n", apNew->serial, apNew->mCent.x, apNew->mCent.y);
      // debug end
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UAvoidPath2::expandOmegaTurn(UPose c1, UPose c2, double turnRad, double angle, UPose * mCent)
{
  double c1c2; // turn centre distance
  double acc;  // angle from previous turn centre (c2) to this turn centre (c1)
  bool result;
//  UPose c1, c2;
  double at; // triangle angle, where unknown side is the distance to move c1
  double a; //, dx = 0.0; // distance to move
  double m; // minor solution
//  UPose mC;
  //
  // solve the triagle (c1, c2 mC), where mC is the new turn centre replacing c1
  // the distance c1c2 is known (calculated)
  // the distance c2mC must be 2 * turnRad to allow a placement of mid-point (cross tangent)
  // the distance c1mC is the distance to move
  // the angle at c1 is at and can be calculated as ('angle' - 'acc').
  //
//  c1 = ap1->mCent;
//  c2 = ap1->prev->mCent;
  c1c2 = c1.getDistance(c2);
  acc = atan2(c1.y - c2.y, c1.x - c2.x);
  at = limitToPi(angle - acc);
  // solve using cosine relation
  a = triangleFinda(c1c2, 2*turnRad, at, &m);
  result = (a >= 0);
  if (result)
  { // add a bit to avoid rounding errors prohibiting a solution
    a += 0.005;
    // get new turn centre
    c1.h = angle;
    *mCent = c1.getPoseToMapPose(-a, 0.0, 0.0);
    //ap1->mCent = mC;
    // debug
    c1c2 = hypot(mCent->y - c2.y, mCent->x - c2.x);
    if (c1c2 < 2.0 * turnRad)
      printf("failed placement of omega turn centre\n");
    // debug end
  }
  return result;
}

///////////////////////////////////////////////////////////////////

double UAvoidPath2::triangleFinda(double b, double c, double C, double * minor)
{
  double a; // triangle angle, where unknown side is the distance to move c1
  double de, cC;
  //
  // solve cosine relation:
  // c^2 = a^2 + b^2 - 2 a b cos(C) - or
  // a^2 + a 2 b cos(C) - c^2 + b^2 = 0
  // a = b * cos(C) +/- sqrt((b * cos(C))^2 + c^2 - b^2)
  //
  cC = cos(C);
  de = sqr(b * cC) + sqr(c) - sqr(b);
  if (de >= 0.0)
  { // can be calculated
    a = b * cC + sqrt(de);
    if (minor != NULL)
      *minor = b * cC - sqrt(de);
  }
  else
    a = -1.0;
  return a;
}

///////////////////////////////////////////////////////////////

bool UAvoidPath2::convertToManSeq(UAvoidPoint * ptLst, UManSeq * manSeq, double minTurnRadius, UAvoidVertexCosts * addCost)
{
  bool result = false;
  UAvoidPoint *ap2 = ptLst, *ap1 = NULL, *apNew; // ap1 is more forward than ap2
  UAvoidPoint *  excludedList = NULL;
  UPoseV poEndMan, po4;
  bool close;
  UAvoidObst * oObs;
  UPosition oPos, cog, oRel, oRel2;
  double dist;
  UManPPSeq * newPP;
  bool cvLeft, postObst;
  double d, dMin;
  int apCnt = 0, mHit;
  const int MAX_AP_CNT = 20;
  U2Dseg ls;
  int n, f;
  double turnRad;
  const int MFC = 3;
  int validFootprintPolyCnt[MFC] = {0, 0, 0};
  bool retry = false;
  bool ommitAp2 = false;
  bool movedAp2 = false;
  const int MRC = 3;
  UAvoidPoint * remList[MRC];
  int remListCnt;
  //
  result = ap2 != NULL;
  solutionCnt = 0;
  if (result)
    result = createMidPathPoints(ptLst, &ommitAp2, &movedAp2);
  if (result)
  {
    oldFootCnt = polysCnt;
    for (n = 0; n < oldFootCnt; n++)
      polys[n]->clear();
    polysCnt = 0;
    ap1 = ap2->next;
    poEndMan = ap2->mid;
  }
  // get turn radius to use from parameter settings
  turnRad = par->getMinTurnRad();
  //
  while (ap1 != NULL and result and apCnt < MAX_AP_CNT)
  { // investigating point ap1 (ap2 is previous)
    apCnt++;
    remListCnt = 0;
    //
    // debug
    if (manSeq->getP2PCnt() > 0)
    {
      po4 = manSeq->getEndPoseV();
      d = po4.getDistance(ap2->mid);
      if (d > 0.01)
        printf("Previous manoeuver does not end where ap2 starts! distrance is %.2fm\n", d);
    }
    printf("-- generation %d \n", generation);
    UAvoidPoint *apn;
    apn = ptLst;
    while (apn != NULL)
    {
      printf("  + path %d gen %d, pre %d, obst (%.2fx,%.2fy) ", apn->serial,
             apn->generation, apn->prePoint, apn->aPos.x, apn->aPos.y);
      printf("mid (%.2fx,%.2fy,%.2fh) ap1=%d ap2=%d\n",
             apn->mid.x, apn->mid.y, apn->mid.h, apn== ap1, apn==ap2);
      apn = apn->next;
    }
    apn = excludedList;
    while (apn != NULL)
    {
      printf("  - excl gen %d, obst (%.2fx,%.2fy)\n",
             apn->generation, apn->aPos.x, apn->aPos.y);
             apn = apn->next;
    }
    // debug end
    //
    // save current planned end pose
    poEndMan = ap1->mid;
    // extend the manoeuvre from ap2 mid point to ap1 mid-point
    newPP = extendManoeuvre(manSeq, ap2, ap1);
    //
    // this new pose-to-pose manoeuvre may get too close to other obstacles - test
    dist = par->obstClearanceDesired;
    ls.setFromPoints(ap2->aPos.x, ap2->aPos.y, ap1->aPos.x, ap1->aPos.y);
    // ensure backtrace points (mostly debug)
    for (f = MFC - 1; f > 0; f--)
      validFootprintPolyCnt[f] = validFootprintPolyCnt[f - 1];
    validFootprintPolyCnt[0] = polysCnt;
    //
    // get closest obstacle within manoeuvre envalope plus clearence margin
    close = getClosestObst2(newPP, &ls, ap1->isTight() or ap2->isTight(),
                            &oPos, &dist, &oObs, &mHit,
                            &cvLeft, ap2->prev == NULL, ap1->next == NULL);
    //
    if (solutionCnt >= par->acceptAfterSolution)
    { // debug stop here
      result = true;
      break;
    }
    if (close)
      // is it a valid new close obstacle point
      close = validNewClosePoint(oPos, dist, ap2, ap1);
    // is it to be ignored by count (debug feature)
    if (close and closeCnt >= par->ignoreCloseObstAfter)
    { // debug ignore next (and remaing) close obstacles
      close = false;
      // debug
      printf("  # ignored point %d (ignore after %d) at (%.2fx,%.2fy) at %.2fm from envelope - stops path here.\n", closeCnt, par->ignoreCloseObstAfter, oPos.x, oPos.y, dist);
      // debug end
      manSeq->releaseLast();
      result = false;
      break;
    }
    // we will not try to avoid obstacles while
    // trying to stay on a line
    if (close and not ap1->followLineLastPose)
    { // obstacle is to be avoided
      closeCnt++;
      // where to insert
      postObst = isAfterDestination(oPos, ap1);
      //
      if (postObst and ap1->next == NULL)
      { // after final destination, so just stop
        result = false;
        if (mHit > 0)
        {
          manSeq->truncate(newPP, mHit);
          printf("Hit an obstacle after destination point (reduced last manoeuvre to %d mans\n", mHit);
        }
        else
        {
          n = manSeq->getP2PCnt();
/*          if (n > 1)
          {
            /// @todo if just one manoeuvre, then it should probably be shortened, and
            ///       not just left as is.
            manSeq->releaseLast();
          }
          else*/
            newPP->truncate(oPos, fmax(par->frontLeft.x, par->frontRight.x) + par->obstClearanceDesired);
          printf("Hit an obstacle after destination point (reduced manoeuvres from %d to %d)\n",
          n, manSeq->getP2PCnt());
        }
        break;
      }
      // is the new point to be avoided left or right
      cvLeft = avoidToTheLeft(oPos, oObs, postObst, ap1);
      // first manoeuvre has less flexibility -
      // fail if impossible
      if (ap2->avoidLeft != cvLeft and ap2->prev == NULL)
      { // test if this is possible, must be relative far from
        // start turn centre to succeed
        d = ap2->mCent.getDistance(oPos);
        if (d < turnRad + par->getSafeDistance(cvLeft, false, true, false))
        { // this is not possible
          result = false;
          manSeq->releaseLast();
          printf("Impossible near (%.2fm) obstacle at %.2fx,%.2fy - fail this solution\n",
                 d, oPos.x, oPos.y);
          break;
        }
      }
      // debug
      //printf("  # found close point %d at (%.2fx,%.2fy) at distance %.2fm (post=%d avLeft=%d)\n",
      //       closeCnt, oPos.x, oPos.y, dist, postObst, cvLeft);
      // debug end
      // add new waypoint - except if ap1 is a prepoint close to the new obstacle
      apNew = NULL;
      if (ap1->prePoint)
      {
        oRel = ap1->mid.getMapToPose(oPos);
        if (oRel.x > -turnRad)
          // reuse and replace ap1 rather than adding a new point
          apNew = ap1;
      }
      if (apNew == NULL)
      {
        apNew = getEmptyPoint();
        // debug
        // printf("got new point serial %d - in convertToManSeq\n", apNew->serial);
        // debug end
      }
      apNew->avoidLeft = cvLeft;
      apNew->aPos = oPos;
      apNew->aob = oObs;
      apNew->prePoint = false;
      apNew->generation = generation;
      retry = true;
      if (ap1->next == NULL)
      { // we have inserted a new point, so retry to follow line
        // in last segment of manoeuvre (if so desired)
        ap1->followLineLastPose = par->followLineLastPose;
      }
      // insert into point sequence
      if (apNew != ap1)
      { // this is a new point, and must be insterted
        if (postObst and ap1->next != NULL)
        {  // insert after this. NB! not if it would be new endpose
          ap1->insertAfter(apNew);
        }
        else
        { // to be insterted before the ap1 point
          ap2->insertAfter(apNew);
          if (postObst)
            // a pre stop point may be bossible is needed to
            // avoid a destination frontal obstacle
            setPreStopPoint(cvLeft, oPos, apNew, ap1);
        }
        //
        // should this new obstacle point replace any of the
        // two neighboroing points - just before and just after
        if (toReplaceOldPoint(apNew, apNew->next, excludedList))
        {
          // debug
          // printf("  ¤ New point at %.2fx,%.2fy replaces %.2fx,%.2fy (apNew->next)\n",
          //       apNew->aPos.x, apNew->aPos.y, apNew->next->aPos.x, apNew->next->aPos.y);
          // debug end
          remList[remListCnt++] = apNew->next;
          apNew->useTight = true;
        }
        if (apNew->prev->prePoint)
          // inserted just after a pre-point - then remove the prepoint
          remList[remListCnt++] = apNew->prev;
        else if (toReplaceOldPoint(apNew, apNew->prev, excludedList))
        { // apNew->prev is a normal point (but may have a pre-point)
          // debug
          // printf("  ¤ New point at %.2fx,%.2fy replaces %.2fx,%.2fy (apNew->prev) (prev==ap2 %s)\n",
          //       apNew->aPos.x, apNew->aPos.y, apNew->prev->aPos.x, apNew->prev->aPos.y, bool2str(apNew->prev == ap2));
          // debug end
          remList[remListCnt++] = apNew->prev;
          if (apNew->prev->prev->prePoint)
            // remove also prepoint
            remList[remListCnt++] = apNew->prev->prev;
          // assume this is a slightly problematic area
          apNew->useTight = true;
        }
      }
      // add position of other obstacles within safe turn distance
      // both ways - to be safe (one turn and one straight should be OK)
      dMin = par->getSafeDistance(apNew->avoidLeft, false, false, false);
      // test if this is a tight passage
      setClosestPoint(apNew, dMin * 2.0);
    }
    else if (close)
    { // retry without follow line as soon as possible
      ap1->followLineLastPose = false;
      //result = createMidPathPoints(ap1, &ommitAp2);
      retry = true;
    }
    //
    if (close)
    { // now we are ready to reevaluate route points (unless we have to make new manoeuvre)
      ommitAp2 = false;
      for (n = 0; n <= remListCnt; n++)
      { //   mid-points are to be recalculated
        if (n == remListCnt)
          result = createMidPathPoints(ap2, &ommitAp2, &movedAp2);
        if (ommitAp2)
        { // remove ap2->next, and retry to find solution
          ap1 = ap2;
          result = true;
        }
        else if (n == remListCnt)
          break;
        else if (remList[n] != NULL)
          ap1 = remList[n]->prev;
        else
          ap1 = NULL;
        // if ap2 is to be removed, then we need to move ap2 back first
        while (ap1 != NULL)
        { // ensure to remove all extra prepoints
          if (ap1->next == ap2)
          { // ap2 is to be deleted
            // go one step back
            ap2 = ap2->prev;
            // garbage removal (manoeuvres and envelope polygons)
            manSeq->releaseLast();
            for (f = 0; f < MFC - 1; f++)
              validFootprintPolyCnt[f] = validFootprintPolyCnt[f + 1];
          }
          for (f = n + 1; f < remListCnt; f++)
          { // test if we are removing one that is on the remove-list
            if (remList[f] == ap1->next)
              // do not remove it twice
              remList[f] = NULL;
          }
          if (ap1->next->prePoint)
            // do not put pre-points into exclude list
            recyclePoint(ap1->unlinkNext());
          else if (excludedList == NULL)
            excludedList = ap1->unlinkNext();
          else
            excludedList->insertAfter(ap1->unlinkNext());
          if (not ap1->prePoint or ap1->prev == NULL)
            break;
          ap1 = ap1->prev;
        } 
        retry = true;
      }
      if (not retry)
        retry = movedAp2;
      if (not retry)
      { // test also for movement of ap1 midpoint
        ap1 = ap2->next;
        d = ap1->mid.getDistance(&poEndMan);
        if (d > par->obstClearanceMinimum or limitToPi(poEndMan.h - ap1->mid.h) > 0.01)
        {
          printf("Retry triggered by movement of ap1 midpoint only (surplus? +++++++++)\n");
          retry = true;
        }
      }
      while (retry)
      { // last manoeuvre and envelope polygons are to be recalculated
        // so remove as garbage
        manSeq->releaseLast();
        polysCnt = validFootprintPolyCnt[0];
        for (f = 0; f < MFC - 1; f++)
          validFootprintPolyCnt[f] = validFootprintPolyCnt[f + 1];
        if (not movedAp2 or ap2->prev == NULL)
          break;
        // we need to go back a step
        ap2 = ap2->prev;
        movedAp2 = false;
      }
      // ensure order of ap2 to ap1
      ap1 = ap2->next;
    }
    // ready to continue
    if (result and ap1 != NULL)
    { // finished with this part, move on
      if (retry)
      { // debug
        // printf("recalculate manoeuvre from (%.2fx,%.2fy,%.2frad) now to (%.2fx,%.2fy,%.2frad)\n",
        //        ap2->mid.x, ap2->mid.y, ap2->mid.h, ap1->mid.x, ap1->mid.y, ap1->mid.h);
        // debug end
        retry = false;
      }
      else
      { // advance to next segment
        ap2 = ap1;
        ap1 = ap1->next;
      }
    }
  }
  if (apCnt >= MAX_AP_CNT)
  {
    printf("Reached maximum waypoint count (%d), with no solution\n", apCnt);
    result = false;
  }
  if (not result)
  { // punish obstacle positions near failed point
    UAvoidPoint *ap3;
    bool found = false;
    if (true) //apCnt < MAX_AP_CNT)
    { // punish the last vertices hard - the one where passing failed
      ap3 = ap1;
      while (ap3 != NULL)
      {
        if (ap3->next == NULL)
          break;
        if (not ap3->prePoint and ap3->generation == 0)
        { // add only original path points
          costAdd.setExtraCost(ap3->aPos, ap3->avoidLeft, 1e3);
          found = true;
          break;
        }
        ap3 = ap3->next;
        if (ap3->aPos.dist(ap1->aPos) > minTurnRadius * 10.0)
          break;
      }
      ap3 = ap2;
      while (ap3 != NULL and not found)
      { // and previous point too
        if (ap3->prev == NULL)
          break;
        if (not ap3->prePoint and ap3->generation == 0)
        { // add only original path points
          costAdd.setExtraCost(ap3->aPos, ap3->avoidLeft, 1e3);
          found = true;
          break;
        }
        ap3 = ap3->prev;
        if (ap3->aPos.dist(ap1->aPos) > minTurnRadius * 10.0)
          break;
     }
    }
    // debug
    if (found)
      printf("Path failed, found point to punish at %.3fx,%.3fy\n", ap3->aPos.x, ap3->aPos.y);
    else
      printf("Path failed, but found noone to blame (punish)!\n");
    // debug end
  }
  // ensure these points are recycled
  if (excludedList != NULL)
    recyclePoints(excludedList);
  //
  return result;
}

////////////////////////////////////////////////

bool UAvoidPath2::setTurnCentre(UAvoidPoint * ap1, double turnCentreRadius, double * availableOuther)
{
  UAvoidPoint * ap2 = ap1->prev; // ap1 is more forward than ap2
  double saa, caa; // average angle of entry and exit - i.e. closest point on avoid route
  UPose c1, c2, c1r; // center for avoid circle
  double toMp, h;
  bool result = true;
  double safeInner, safeOuther;
  //
  safeInner = par->getSafeDistance(ap1->avoidLeft, true, ap1->isTight(), false);
  if (not ap1->isFreeSpace())
  { // get margins - near and far
    safeOuther = par->getSafeDistance(ap1->avoidLeft, false, true, false);
    if (ap1->otDist > safeInner + safeOuther and not ap1->prePoint)
      // possible to get through in a normal (full) turn
      toMp = (ap1->otDist - (safeInner + safeOuther))/2.0 + safeInner;
    else
    { // too tight for full turn, so get as close as possible on the inner side
      toMp = safeInner;
    }
    // result is not direct usable.
    result = false;
    if (availableOuther != NULL)
      *availableOuther = ap1->otDist - toMp;
    // get vector to opposing point - from most tight point
    saa = (ap1->oPos.y - ap1->tPos.y)/ap1->otDist;
    caa = (ap1->oPos.x - ap1->tPos.x)/ap1->otDist;
    // center for avoidance path for the ap1 obstacle
    c1.set(ap1->aPos.x - caa * (turnCentreRadius - toMp),
           ap1->aPos.y - saa * (turnCentreRadius - toMp), atan2(saa, caa));
  }
  else if (not ap1->prePoint)
  { // get vector (saa,caa) for path passing this point
    // if distance to next point is short, then
    if (ap1->avoidLeft == ap2->avoidLeft and
      ap1->aPos.dist(ap2->aPos) < turnCentreRadius)
    { // short distance to prevoius, so align center with this
      h = ap2->angNext;
      saa = sin(h);
      caa = cos(h);
    }
    else if (ap1->next != NULL and
             ap1->avoidLeft == ap1->next->avoidLeft and
             ap1->aPos.dist(ap1->next->aPos) < turnCentreRadius)
    { // short distance to next 
      h = ap1->angNext;
      saa = sin(h);
      caa = cos(h);
    }
    else
    { // else use average heading
      saa = (sin(ap2->angNext) + sin(ap1->angNext))/ 2.0;
      caa = (cos(ap2->angNext) + cos(ap1->angNext))/ 2.0;
      h = atan2(saa, caa);
    }
    // this distance from obstacle point
    if (ap1->avoidLeft)
    { // center of avoidance turn for current obstacle
      // at right angle to heading direction
      c1.set(ap1->aPos.x + saa * (turnCentreRadius - safeInner),
             ap1->aPos.y - caa * (turnCentreRadius - safeInner), h + M_PI / 2.0);
    }
    else
    { // center of avoidance turn for current obstacle
      // at right angle the other way.
      c1.set(ap1->aPos.x - saa * (turnCentreRadius - safeInner),
             ap1->aPos.y + caa * (turnCentreRadius - safeInner), h - M_PI / 2.0);
    }
  }
  if (not ap1->prePoint)
    ap1->mCent = c1;
  return result;
}

//////////////////////////////////////////////

bool UAvoidPath2::setMidPoint(UAvoidPoint * ap1, double manRad, bool * reversedAp2mCent)
{
  UAvoidPoint * ap2 = ap1->prev; // ap1 is more forward than ap2
  UPose c1, c2, c1r, dest; // center for avoid circle
  bool result = true;
  bool midSet = false;
  bool ap2avoidLeft = ap2->avoidLeft;
  //
  // default when both center positions are known
  c1 = ap1->mCent;
  c2 = ap2->mCent;
  // start and end position are different
  if (ap2->prev == NULL and ap1->next == NULL and ap1->followLineLastPose)
  { // this is a direct path, no further processing needed
    ap1->mid = par->exitPose;
    midSet = true;
    result = true;
  }
  else if (ap2->prev == NULL)
  { // ap2 is the the start position, and may start left or right
    // so recalculate manoeuvre centre
    if (ap1->next == NULL)
      ap1->mCent = par->exitPose;
    c1r = ap2->mid.getMapToPosePose(&ap1->mCent);
    if (ap1->avoidLeft and c1r.y < -manRad)
      ap2->avoidLeft = true;
    else if (not ap1->avoidLeft and c1r.y < manRad) // and fabs(c1r.h) < M_PI / 2.0)
      ap2->avoidLeft = true;
    // no - this would make the turn the other way around
    //else if (ap1->avoidLeft and c1r.y < manRad and fabs(c1r.h) > M_PI / 2.0)
    //  ap2->avoidLeft = true;
    else
      ap2->avoidLeft = false;
    // center of manoeuvre circle for previous point
    if (ap2->avoidLeft)
      // center of man circle from previous point
      c2.set(ap2->mid.x + sin(ap2->mid.h) * manRad,
             ap2->mid.y - cos(ap2->mid.h) * manRad, ap2->mid.h + M_PI / 2.0);
    else
      c2.set(ap2->mid.x - sin(ap2->mid.h) * manRad,
             ap2->mid.y + cos(ap2->mid.h) * manRad, ap2->mid.h - M_PI / 2.0);
    // save for potential omega turns
    ap2->mCent = c2;
  }
  if (ap1->next == NULL and not midSet)
  { // ap1 is the end position and not from start,
    // centre may therefore be left or right,
    // so recalculate manoeuvre centre
    // except when following a line
    dest = par->exitPose;
    c1r = dest.getMapToPosePose(&ap2->mid);
    if ((fabs(c1r.h) < M_PI / 4.0 and
         fabs(c1r.y) < manRad * 0.5 and
         ap1->followLineLastPose) or
        hypot(c1r.y, c1r.x) <= manRad)
    { // either target pose is too close to hit or
      // we are almost on target line (within 0.5 turn radius)
      // and should try to follow the line.
      ap1->mid = dest;
      midSet = true;
    }
    else
    { // not close to line and posibly at a usable distance from destination
      // get last turn-centre relative to exit pose
      // or not allowed to follow line.
      c1r = dest.getMapToPosePose(&ap2->mCent);
      if (ap2->avoidLeft and c1r.y < -manRad)
        ap1->avoidLeft = true;
      else if (not ap2->avoidLeft and c1r.y < manRad)
        ap1->avoidLeft = true;
      else
        ap1->avoidLeft = false;
      // center of manoeuvre circle for previous point
      if (ap1->avoidLeft)
        // center of man circle from previous point
        c1.set(dest.x + sin(dest.h) * manRad,
               dest.y - cos(dest.h) * manRad, dest.h + M_PI / 2.0);
      else
        c1.set(dest.x - sin(dest.h) * manRad,
               dest.y + cos(dest.h) * manRad, dest.h + M_PI / 2.0);
      // save for potential omega turns
      ap1->mCent = c1;
    }
  }
  if (not ap2->prePoint)
  { // maybe the avoid-left should be changed
    // try a redo of center 2
    c1r = ap2->mid.getMapToPosePose(&ap1->mCent);
    if (hypot(c1r.x, c1r.y) >= manRad)
    { // can make sence to reevaluate 
      if (ap1->avoidLeft and c1r.y < -manRad)
        ap2->avoidLeft = true;
      else if (not ap1->avoidLeft and c1r.y < manRad)
        ap2->avoidLeft = true;
      else
        ap2->avoidLeft = false;
      // center of manoeuvre circle for previous point
      if (ap2->avoidLeft)
          // center of man circle from previous point
        c2.set(ap2->mid.x + sin(ap2->mid.h) * manRad,
                ap2->mid.y - cos(ap2->mid.h) * manRad, ap2->mid.h - M_PI / 2.0);
      else
        c2.set(ap2->mid.x - sin(ap2->mid.h) * manRad,
                ap2->mid.y + cos(ap2->mid.h) * manRad, ap2->mid.h + M_PI / 2.0);
                // save for potential omega turns
      ap2->mCent = c2;
    }
    // debug
    else
    {
      printf("Too close to reevaluate turncentre for ap2\n");
    } // debug end
  }
  //
  if (not midSet)
  { // find tangent point as new mid-point
    if (ap1->avoidLeft != ap2->avoidLeft)
    { // different arc directions, so find cross tangent point
      result = crossTangent(c1, c2, ap1->avoidLeft, manRad, &ap1->mid);
      if (not result and ap1->next == NULL)
        // too close to plan a cross-tangent, so head for the line instead
        ap1->mid = par->exitPose;
    }
    else
      // same arc direction, so find outher tangent
      result = outherTangent(c1, c2, ap1->avoidLeft, manRad, &ap1->mid);
    // set ap2 avoid side back to original (to match aPoint)
    if (reversedAp2mCent != NULL)
      *reversedAp2mCent = (ap2->avoidLeft != ap2avoidLeft);
    // keep true avoid direction
    ap2->avoidLeft = ap2avoidLeft;
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UAvoidPath2::crossTangent(UPose c1, UPose c2,
                               bool cv, double manRad,
                               UPose * tanpt)
{
  double c1c2; // distance from c1 to c2
  double ac1c2; // angle to c1-c2 line
  double accm1; // angle from new mid-point to center-center line
  bool result = false;
  //
  c1c2 = hypot(c2.y - c1.y, c2.x - c1.x);
  result = c1c2 > 2.0 * manRad;
  if (result)
  {
    ac1c2 = atan2(c2.y - c1.y, c2.x - c1.x);
    // The triangle from c1 through tangent point (mid1) circle 1 ectended with radius
    // of other circle (also manRad), at right angle to c2
    // find the angle at c1
    accm1 = acos(2.0 * manRad / c1c2);
    // angle to tangent point is always more positive than angle to other centre.
    if (cv)
      c1.h = ac1c2 - accm1;
    else
      c1.h = ac1c2 + accm1;
    // find new mid-pose angle
    *tanpt = c1;
    tanpt->add(manRad, 0.0);
    if (cv)
      tanpt->h = limitToPi(c1.h - M_PI / 2.0);
    else
      tanpt->h = limitToPi(c1.h + M_PI / 2.0);
  }
  //
  return result;
}

//////////////////////////////////////////

bool UAvoidPath2::outherTangent(UPose c1, UPose c2,
                               bool cv, double manRad,
                               UPose * tanpt)
{
  double c1c2; // distance from c1 to c2
  double ac1c2; // angle to c1-c2 line
  bool result;
  //
  // center-center distance
  c1c2 = hypot(c2.y - c1.y, c2.x - c1.x);
  result = c1c2 > 1e-4;
  if (result)
  { // angle x-axis to line from c1 to c2
    ac1c2 = atan2(c2.y - c1.y, c2.x - c1.x);
    // The triangle right angle at c1 to tangent point (mid1) circle 1
    // and to other circle centre c2
    if (cv)
      c1.h = ac1c2 - M_PI/2.0;
    else
      c1.h = ac1c2 + M_PI/2.0;
    // find new mid-pose angle
    *tanpt = c1;
    tanpt->add(manRad, 0.0);
    tanpt->h = limitToPi(ac1c2 + M_PI);
  }
  return result;
}

////////////////////////////////////////////

bool UAvoidPath2::pointTangent(UPose dest, UPose c2,
                                bool cv, double c2rad,
                                UPose * tanpt)
{
  double p1c2; // distance from c1 to c2
  double ap1c2; // angle to c1-c2 line
  double a; // angle from ap1c2 to tangent point
  bool result;
  double sa, ca;
  //
  // center-center distance
  p1c2 = hypot(c2.y - dest.y, c2.x - dest.x);
  result = p1c2 > c2rad;
  if (result)
  { // angle x-axis to line from c1 to c2
    ap1c2 = atan2(c2.y - dest.y, c2.x - dest.x);
    // The triangle right angle at tangent point
    // and to other circle centre c2
    a = asin(c2rad/p1c2);
    if (not cv)
      a = -a;
    a += ap1c2;
    sa = sin(a);
    ca = cos(a);
    // find new mid-pose angle
    tanpt->x = c2.x - sa * c2rad;
    tanpt->y = c2.y + ca * c2rad;
    tanpt->h = limitToPi(a + M_PI);
  }
  return result;
}

////////////////////////////////////

void UAvoidPath2::getManAsPolygon(UManPPSeq * manSeq, UPolygon * p40, double endsMargin)
{
  UPolygon400 * poly;
  p40->clear();
  int i;
  UManoeuvre * man;
  UManArc * arc;
  UPoseV pv1, pv2, pv3;
  U2Dpos pos, pos2;
  double d, di, ds = 0.0, de;
  const double INCREMENT = 0.2; // meter
  //
  poly = new UPolygon400();
  pv1 = manSeq->getStartPoseV();
  // last distance to be used
  de = manSeq->getManDist() - endsMargin;
  //poly->add(manSeq->getStartPos());
  for (i = 0; i < manSeq->getSeqCnt(); i++)
  {
    man = manSeq->getMan(i);
    pv2 = man->getEndPoseV(pv1);
    if (ds + man->getDistance() > endsMargin and ds <= de)
    {
      if (ds >= endsMargin)
        poly->add(pv1.x, pv1.y, 0.0);
      if (man->getManType() == UManoeuvre::MAN_LINE)
      {
        if (ds < endsMargin)
        { // start after entry point
          pos.y = 0.0;
          pos.x = endsMargin - ds;
          pos2 = pv1.getPoseToMap(pos);
          poly->add(pos2.x, pos2.y, 0.0);
        }
        if (ds + man->getDistance() > de)
        {
          pos.y = 0.0;
          pos.x = de - ds;
          pos2 = pv1.getPoseToMap(pos);
          poly->add(pos2.x, pos2.y, 0.0);
        }
      }
      else if (man->getManType() == UManoeuvre::MAN_ARC)
      {
        arc = (UManArc *) man;
        di = INCREMENT/arc->getTurnRadius();
        if (ds > endsMargin)
          d = di;
        else
        { // not started yet
          d = (endsMargin - ds - di * arc->getTurnRadius()) / arc->getTurnRadius();
        }
        while (d < fabs(arc->getTurnAngle()))
        {
          if (arc->getTurnAngle() > 0)
          {
            pos.x = sin(d) * arc->getTurnRadius();
            pos.y = (1 - cos(d)) * arc->getTurnRadius();
          }
          else
          {
            pos.x = sin(d) * arc->getTurnRadius();
            pos.y = -(1 - cos(d)) * arc->getTurnRadius();
          }
          pos2 = pv1.getPoseToMap(pos);
          poly->add(pos2.x, pos2.y, 0.0);
          d += di;
          if (d * arc->getTurnRadius() + ds > de)
            break;
        }
      }
    }
    // add end pose too
    ds += man->getDistance();
    pv1 = man->getEndPoseV(pv1);
  }
  poly->extractConvexTo(p40);
  delete poly;
}

/////////////////////////////////////////////////////////////////

bool UAvoidPath2::getClosestObst2(UManPPSeq * manSeq,
                                  U2Dseg * visLine,
                                  bool tight,
                                  UPosition * oPos,
                                  double * dist,
                                  UAvoidObst ** oObst,
                                  int * mHit,
                                  bool * avoidLeft,
                                  bool firstIsCurrentPose,
                                 bool nextIsDestination)
{
  int i, k1, m;
  UAvoidObst * og;
  double sd, d, d1, d2, dm;
  UPosition pos, pos2, mPos;
  UPoseV p0, p1, p2, p3;
  U2Dseg seg;
  U2Dpos p2d;
  bool found;
  double dMarg, dMargArc, diag;
  UPolygon * polyIncl, *polyExcl;
  UManoeuvre * man;
  bool avLeft = true;
  ULineSegment ls;
  double t1, t2;
  int whereOnMan, manHit;
  UPose pHit;
  UPolygon400 * p400;
  UPolygon  * polyInclAll; //, *polyDummy;
  int polyMan0;
  double samePoint;
  bool endPoint;
  double allowInside; // allow obstacles this far inside envolope
  //
  // safe distance when no turns (assume symmetric robot)
  dMargArc = par->getSafeDistance(true, false, tight, false);
  if (tight)
    dMarg = par->obstClearanceMinimum;
  else
    dMarg = par->obstClearanceDesired;
  samePoint = dMarg/3.0;
  allowInside = dMarg/4.0;
  *dist = dMarg;
  *mHit = -1;
  diag = par->getDiagonal(true, true, NULL);
  // start position of manoeuvre sequence
  p1 = manSeq->getStartPoseV();
  p0 = p1;
  p2 = manSeq->getEndPoseV();
  p400 = new UPolygon400();
  polyInclAll = getFoodprintPoly();
  // just to keep odd polygins an exclusion polygon
  /*polyDummy =*/ getFoodprintPoly();
  // get index to first manoeuvre sub-polygon
  polyMan0 = polysCnt;
  // line segment of from-to line segment
  seg.setFromPoints(p1.x, p1.y, p2.x, p2.y);
  if (seg.length > dMargArc / 3.0 or nextIsDestination)
  { // not too short
    sd = 0;
    pos = visLine->getFirstEnd();
    p400->add(pos);
    pos = visLine->getOtherEnd();
    p400->add(pos);
    for (m = 0; m < manSeq->getSeqCnt(); m++)
    {
      man = manSeq->getMan(m);
      // get end pose of this part, i.e. p1 is current start pose and p3 is end pose
      p3 = man->getEndPoseV(p1);
//      if (sd + man->getDistance() > dMargArc or not firstIsCurrentPose)
      { // relevant manoeuvre to consider
        polyIncl = getFoodprintPoly();
        polyExcl = getFoodprintPoly();
        // get footprint polygon
        man->getFoodprintPolygon(polyIncl, polyExcl,
                                 par->frontLeft.x, par->frontLeft.y,
                                 par->frontRight.x, par->frontRight.y,
                                 dMarg,
                                 &p1, dMarg / 20.0);
        // summarize to a full manoeuvre polygon for first analysis.
        p400->add(polyIncl);
      }
      sd += man->getDistance();
      // advance start position to next manoeuvre in sequence
      p1 = p3;
    }
    p400->extractConvexTo(polyInclAll);
    // find potential obstacles
    for (i = 0; i < (aogsCnt - 2); i++)
    { // test all obstacles (or groups of near obstacles)
      // get closest point within a desired manoeuvre space
      og = aogs[i];
      while (og != NULL)
      { // try all obstacles in combined obstacle group.
        // get center of gravity (cog) for polygon
        found = polyInclAll->isOverlappingXYconvex2(og->obst, 0.0);
        if (found)
        { // first test if most offending point is within endpoints of current manoeuvre segment
          // - this requires knowledge of avoid left or right
          // 1. is a vertex inside all-polygon
          //    - then
          // 1.1  Is it obstacle vertex between the planned path and the visibility line (best)
          // 1.2  (if not between) is it inside a man polygon (use distance to visibility line relative to  
          // 2. test if vertex is on the outher side 
          // test if any point is close to polyIncl and not
          // inside the exclude polygon.
          //bool vertexInside = false;
          for (k1 = 0; k1 < og->obst->getPointsCnt(); k1++)
          { // test vertex points against manoeuvre footprint too
            pos = og->obst->getPoint(k1);
            if (firstIsCurrentPose)
            { // obstacles close to current pose is ignored
              // most likely a detection error or a self detection
              d2 = hypot(pos.y - p0.y, pos.x - p0.x);
              if (d2 < diag)
                continue;
            }
            d = polyInclAll->getDistance(pos.x, pos.y, NULL, NULL);
            found = d < -allowInside;
            manHit = -1;
            d1 = 0.0;
            endPoint = false;
            if (found)
            { // a vertex is found inside foodprint polygon
              //vertexInside = true;
              // is it between vis line and path.
              t1 = visLine->getPositionOnLine(pos.x, pos.y);
              // position must be along the visibility line, but not the
              // end points of the visibility line
              found = (t1 > -dMargArc and t1 < visLine->length + dMargArc);
              if (found) // and fabs(d) < allowInside * 2.0)
              { // alow endpoints of visibility line to be a bit closer to path
                endPoint = (fabs(t1) < samePoint or fabs(t1 - visLine->length) < samePoint);
                found = not endPoint;
              }
              if (found)
              { // is within visibility line
                d1 = visLine->getDistanceSigned(pos.x, pos.y, NULL);
                d2 = manSeq->getDistanceXYSigned(pos, &whereOnMan, false, &pHit, &t2, &manHit);
                found = (d1 * d2 < 0.0);
                /// there could be exceptions before or after the visibility line
                /// that is not handled.
              }
              if (found)
              { // is between visibility line and path, but may be
                // a previous obstacle vertex that is ignored due to
                // crossing turn-centres
                t2 = par->getMinTurnRad();
                found =  t1 < 0.0 or
                        (t1 > t2 and t1 < (visLine->length - t2)) or
                         t1 > visLine->length;
              }
              avLeft = d1 < 0.0;
              if (found)
                // make d2 distance very negative as it is on the other side of path 
                d2 = -fabs(d2) - dMargArc;
            }
            if (not found and manHit >= 0 and not endPoint)
            { // may be inside manoeuvre polygon (but outside part of path)
              d2 = isInsidePoly(pos, polyMan0, &manHit, -allowInside);
              found = d2 < -allowInside;
            }
            if (found and d2 < *dist)
            {
              *dist = -fabs(d2);
              *oPos = pos;
              *oObst = og;
              *avoidLeft = avLeft;
              *mHit = manHit;
            }
          }
          if (*mHit < 0) // and not vertexInside)
          { // no obstacle is found so we should try if manoeuvre enters a very long obstacle
            // we try with the convex polygon for the full manoeuvre
            //
            found = isCrossingManPoly(og->obst, &pos, &dm, polyMan0, &manHit, -allowInside);
            if (found)
            { // just to be sure that the point is on the edge of the obstacle polygon
              og->obst->getClosestDistance(pos.x, pos.y, 0.1, &pos);
              // now find value of this point
              d1 = visLine->getDistanceSigned(pos.x, pos.y, &whereOnMan);
              // sanity check to see if obstacle is crossing the visibility line
              // this may be caused by numeric problems if visibility line is
              // along a polygon
              d = visLine->getDistanceSigned(og->obst->getCogXY().x, og->obst->getCogXY().y, NULL);
              // they must have same sign (and not zero)
              found = (d * d1 > 0.001 or whereOnMan != 0);
              // debug
              if (not found)
                printf("  * found an obstacle at %.3fx,%.3fy that crosses the visibility line (ignored (numeric error))\n",
                      pos.x, pos.y);
              // debug end
              if (found)
              { // - so far a valid point
                d2 = manSeq->getDistanceXYSigned(pos, &whereOnMan, false, &pHit, &t2, &manHit);
                found = (d1 * d2 < 0.0);
                avLeft = d1 < 0.0;
                if (found)
                  // make d2 distance very negative as it is on the other side of path, i.e.
                  // in the same side as the visibility line
                  dm = -fabs(d2) - dMargArc;
                if (dm < *dist)
                {
                  *dist = dm;
                  *oPos = pos;
                  *oObst = og;
                  *avoidLeft = avLeft;
                  *mHit = manHit;
                }
              }
            }
          }
        }
        og = og->grp;
      }
    }
  }
  delete p400;
  return *mHit >= 0;
}

////////////////////////////////////////////////////////////////////

bool UAvoidPath2::withinAngleLimits(double a, bool cvLeft, double h1, double h2, double angMarg)
{
  bool result = false;
  double aMin, aMax;
  //
  if (cvLeft)
  {
    aMin = h2;
    aMax = h1;
  }
  else
  {
    aMin = h1;
    aMax = h2;
  }
  if (aMax < aMin)
    aMax += M_PI * 2.0;
  // expand with extra margin)
  aMin -= angMarg;
  aMax += angMarg;
  // lower limit - change to upeer limit test
  if (a < aMin)
    a += M_PI * 2.0;
  result = a < aMax;
  //
  return result;
}

////////////////////////////////////////////////////////////////////////

bool UAvoidPath2::expandVisLinesToManSeq(UAvoidLnkSeq * ls)
{
  bool result = false;
  UAvoidPoint * aPointList;
  // create point-list to avoid - result i aPointList
  generation = 0;
  serialNext = 0;
  aPointList = createAvoidPoinsPath(ls);
  if (aPointList != NULL)
  { // now convert to manoeuvre sequence
    result = convertToManSeq(aPointList, manFull, par->minTurnRadius, &costAdd);
  }
  if (pointListFinal != NULL)
    recyclePoints(pointListFinal);
  pointListFinal = aPointList;
  return result;
}

//////////////////////////////////////////////////////////

void UAvoidPath2::setClosestPoint(UAvoidPoint * ap, double dMin)
{
  int i, w;
  double d = 0.0, d2, t;
  UAvoidObst * og; // group of closely connected obstacles
  UPosition pos;
  ULineSegment l2;
  bool obOK;
  bool aVertex;
  UPosition rel;
  //
  ap->oobDist = dMin;
  ap->oob = NULL;
  if (ap->aob == NULL)
  { // debug
    printf("UAvoidPath2::createAvoidPoinsPath: This must be an error, a link with no object?\n");
    // debug end
  }
  else
  { // ignore the points obstacle group at the end (start and end positions)
    for (i = 0; i < (aogsCnt - 2); i++)
    { // there should be an obstacle associated
      if (i != ap->aob->grpIdx)
      { // get closest point within a desired manoeuvre space
        og = aogs[i];
        d = og->getClosestPoint(ap->aPos, dMin, &pos, &aVertex);
        if (d < ap->oobDist)
        { // check that the point is on the right side of the path
          // at start and end there is no wrong side
          obOK = ap->prev == NULL or ap->next == NULL;
          if (not obOK)
          { // from behind
            l2.setFromPoints(ap->prev->aPos, ap->aPos);
            d2 = l2.getDistanceXYSigned(pos, &w);
            // allow previous obstacle to be new opposing obstacle
            // this is needed when passing openings where the visibility line
            // do not touch either side, but both sides needs negosiation.
            obOK = (ap->avoidLeft and d2 > -0.001) or
                   (not ap->avoidLeft and d2 < 0.001);
            if (obOK)
            { // test also if this part of route is relevant
              t = l2.getPositionOnLine(pos);
              obOK = t < l2.length + dMin / 2.0;
            }
          }
          if (not obOK)
          { // line towards next waypoint
            l2.setFromPoints(ap->aPos, ap->next->aPos);
            d2 = l2.getDistanceXYSigned(pos, &w);
            obOK = (ap->avoidLeft and d2 > 0.0) or
                   (not ap->avoidLeft and d2 < 0.0);
            if (obOK)
            { // test also if this part of route is relevant
              t = l2.getPositionOnLine(pos);
              obOK = t > -dMin / 2.0;
            }
          }
          if (obOK)
          { // should not be too close to (part of) robot
            rel = par->startPose.getPoseToMap(pos);
            obOK = (rel.y > par->frontLeft.y + par->obstClearanceMinimum) or
                   (rel.y < par->frontRight.y - par->obstClearanceMinimum) or
                   (rel.y < -par->obstClearanceMinimum) or
                   (rel.y > fmax(par->frontLeft.x, par->frontRight.x) + par->obstClearanceMinimum);
            // but force to close rules anyhow
            ap->useTight = true;
          }
          if (obOK)
          { // this is best so far
            ap->oobDist = d;
            ap->oob = og;
            ap->oPos = pos;
            ap->oVertex = aVertex;
          }
        }
      }
    }
    // set the most tigt point too
    ap->tPos = ap->aPos;
    if (ap->oob != NULL)
    { // get the position on this obstacle that
      // is closest to other point
      if (ap->prePoint)
        // prepoints has a false obstacle reference
        ap->otDist = ap->oPos.dist(ap->tPos);
      else
        ap->otDist = ap->aob->getClosestPoint(ap->oPos, ap->oobDist + 0.001, &ap->tPos, &ap->tVertex);
    }
    else
    {
      ap->tVertex = false;
      ap->otDist = 0.0;
    }
  }
}

//////////////////////////////////////////////////////////

bool UAvoidPath2::turnCentreCrossing(UAvoidPoint * ap1, UAvoidPoint * ap2, double limit)
{
  U2Dseg se1;
  double d1, d;
  bool result;
  //
  d = ap1->mCent.getDistance(ap2->mCent);
  if (d < par->getMinTurnRad())
  { // centers are relative close, but are they crossing?
    se1.setFromPoints(ap1->mCent.x, ap1->mCent.y, ap1->aPos.x, ap1->aPos.y);
    //se2.setFromPoints(ap2->mCent.x, ap2->mCent.y, ap2->aPos.x, ap2->aPos.y);
    d1 = se1.distanceSigned(ap2->mCent.x, ap2->mCent.y);
    if (ap1->avoidLeft)
      result = d1 < limit;
    else
      result = d1 > -limit;
  }
  else
    result = false;
  //
  return result;
}

////////////////////////////////////////////////////////////

UPolygon * UAvoidPath2::getFoodprintPoly()
{
  if (polysCnt == MAX_FOODPRINT_POLYS)
  {
    polysCnt = 0;
    printf("UAvoidPath2::getFoodprintPoly: no more space - recycled all %d polygons.", MAX_FOODPRINT_POLYS);
  }
  if (polys[polysCnt] == NULL)
    polys[polysCnt] = new UPolygon40();
  return polys[polysCnt++];
}

///////////////////////////////////////////////////////////

void UAvoidPath2::expandPolyToSegment(U2Dseg * seg,
                                      UPolygon * polyIncl,
                                      UPolygon * polyExcl,
                                      bool manIsLeft,
                                      bool turningLeft)
{
  UPolygon40 p40, p41;
  int i, n;
  double t, t1, t2, tmin = 0.0, tmax = 0.0, d, dmin = 0.0, dmax = 0.0;
  UPosition p1, pdMin, pdMax;
  double tdMin = 0.0, tdMax = 0.0;
  U2Dpos pt;
  bool oneSide/*, isLeft*/, useMin, newExcl = false;
  U2Dpos xPnts[2];
  //
  // p1 = polyIncl->getCogXY();
  // get sign and distance from direct line to center of gravity
  //di = seg->distanceSigned(p1.x, p1.y);
  // take
  for (i = 0; i < polyIncl->getPointsCnt(); i++)
  { // find end projection points on the direct (seg) line 
    p1 = polyIncl->getPoint(i);
    t = seg->getPositionOnLine(p1.x, p1.y);
    // and the signed distance
    d = seg->distanceSigned(p1.x, p1.y);
    //isLeft = (d >= 0);
    if (i == 0)
    {
      tmin = fmax(0.0, t);
      tmax = fmin(seg->length, t);
      dmin = d;
      dmax = d;
      tdMin = t;
      tdMax = t;
      pdMin = p1;
      pdMax = p1;
    }
    else if (t < tmin)
      tmin = fmax(0.0, t);
    else if (t > tmax)
      tmax = fmin(seg->length, t);
    if (d < dmin)
    {
      dmin = d;
      pdMin = p1;
      tdMin = t;
    }
    else if (d > dmax)
    {
      dmax = d;
      pdMax = p1;
      tdMax = t;
    }
  }
  oneSide = dmin * dmax >= 0.0;
  if (tmax > tmin)
  { // there is something to add
    if (not oneSide and polyExcl->getPointsCnt() == 0)
    { // Find most extreme point away from direct line
      // get cut-points between the direct line and the footprint envelope
      p40.clear();
      n = polyIncl->cutPoints(seg, xPnts, 2);
      if (n == 2)
      { // there is cut-points (as there should)
        // get away from direct line point
        if (manIsLeft)
          p1 = pdMin;
        else
          p1 = pdMax;
        p40.add(p1);
        // now use one of the tPoints and one of the crossings
        useMin = (tdMin > tdMax and manIsLeft) or (tdMin <= tdMax and not manIsLeft);
        t1 = seg->getPositionOnLine(xPnts[0]);
        t2 = seg->getPositionOnLine(xPnts[1]);
        if (useMin)
        {
          if (t1 < t2)
            p40.add(xPnts[0].x, xPnts[0].y);
          else
            p40.add(xPnts[1].x, xPnts[1].y);
          pt = seg->getPositionOnLine(tmin);
          p40.add(pt.x, pt.y);
        }
        else
        {
          if (t1 > t2)
            p40.add(xPnts[0].x, xPnts[0].y);
          else
            p40.add(xPnts[1].x, xPnts[1].y);
          pt = seg->getPositionOnLine(tmax);
          p40.add(pt.x, pt.y);
        }
        p40.extractConvexTo(polyExcl);
        newExcl = true;
      }
    }
    //
    // extend the incl polygon
    // polygon is away from line (on one side) and inside segment line
    // so it is relevant to expand the polyIncl zone
    polyIncl->copyTo(&p40);
    pt = seg->getPositionOnLine(tmin);
    p40.add(pt.x, pt.y);
    pt = seg->getPositionOnLine(tmax);
    p40.add(pt.x, pt.y);
    p40.extractConvexTo(polyIncl);
  }
  // now the exclusive polygon
  if (polyExcl->getPointsCnt() > 0 and tmax > tmin and turningLeft != manIsLeft)
  { // there is an exclude polygon but may be no need or too big
    if (oneSide)
    { // exclude part of polygon is between direct line and include polygon
      // so no obstacle should be here, i.e. no exclude area
      polyExcl->clear();
    }
    else if (not newExcl)
    { // limit (arc) excl to side with no driving
      if (manIsLeft)
        // the manoeuvre is to the left (of segment), so right side is valid only
        polyExcl->cut(seg, polyExcl, NULL, polyExcl);
      else
        // the manoeuvre is to the right (of segment), so left side is valid only
        polyExcl->cut(seg, polyExcl, polyExcl, NULL);
    }
  }
}

//////////////////////////////////////////////////

void UAvoidPath2::invalidateEmbeddedVertices()
{
  UAvoidObst * aoga, *aogb;
  int i, j;
  UPosition * p1, p2;
  double d;
  bool aVertex;
  //
  for (i = 0; i <  aogsCnt; i++)
  { // test all obstacle groups
    aoga = aogs[i];
    if (aoga->grp != NULL)
    { // this is a group of more than one obstacle
      while (aoga != NULL)
      { // try all obstacles
        aogb = aogs[i];
        while (aogb != NULL)
        { // compare with all other obstacles in group
          if (aogb != aoga)
          { // two different obstacles in the same group
            p1 = aoga->obst->getPoints();
            for (j = 0; j < aoga->obst->getPointsCnt(); j++)
            { // allow vertex to be 3mm instade the other to avoid
              // numeric problems
              d = aogb->obst->getClosestDistance(p1->x, p1->y, 1e5, &p2, &aVertex);
              if (d < 0.002 and not aVertex)
              { // close to or inside the polygon and not near vertex
                // no tangents from this vertex
                aoga->nogoVertex[j] = true;
              }
              // goto next point in polygon
              p1++;
            }
          }
          // continue
          aogb = aogb->grp;
        }
        // continue
        aoga = aoga->grp;
      }
    }
  }
}

////////////////////////////////////////////////////////////

UManPPSeq * UAvoidPath2::extendManoeuvre(UManSeq * manSeq, UAvoidPoint * ap2, UAvoidPoint * ap1)
{
  double da;
  double turnRad;
  UManArc * arc;
  UPoseV poFrom, poTo;
  UManPPSeq * newPP;
  const double allowedEndHeadingError = 10.0 * M_PI / 180.0;
//  const int MRL = 2000;
//  char reply[MRL];
  //
  poFrom = ap2->mid;
  poTo = ap1->mid;
  poFrom.vel = par->exitPose.vel;
  poTo.vel = par->exitPose.vel;
  turnRad = par->getMinTurnRad();
  //
  if (par->useDriveon)
    manSeq->addManDriveon(poFrom, poTo, turnRad);
  else
    manSeq->addMan(poFrom, poTo,  par->maxAcceleration, par->maxTurnAcceleration,  turnRad);
  //
  // get the just generated pose-to-pose manoeuvre
  newPP = manSeq->getP2P(manSeq->getP2PCnt() - 1);
  // exit point may miss last arc
  if (ap1->next == NULL)
  {
    // at destination point
    da = limitToPi(par->exitPose.h - poTo.h);
    if (fabs(da) >  allowedEndHeadingError)
    { // add an extra arc to get to destination pose
      arc = new UManArc();
      arc->setTurnRadius(turnRad);
      if (ap1->avoidLeft)
      { // turning CV (negative angle)
        if (da > 0.0)
          da -= 2.0 * M_PI;
        arc->setTurnAngle(da);
      }
      else
      { // turning CCV
        if (da < 0.0)
          da += 2.0 * M_PI;
        arc->setTurnAngle(da);
      }
      newPP->add(arc);
      newPP->setEndPoseV(par->exitPose);
    }
  }
  newPP->getEndPoseV();
  // debug
  //newPP->print("== manExtend:", reply, MRL);
  //printf("%s\n", reply);
  // debug end
  return newPP;
}

////////////////////////////////////////////////////////////////////

bool UAvoidPath2::validNewClosePoint(UPosition oPos, double dist,
                                     UAvoidPoint * ap2, UAvoidPoint *  ap1)
{
  bool close;
  double a1, a2;
  double d, dMin;
  double odap2;
  double samePoint = par->obstClearanceMinimum / 4.0 + 0.01;
  //
  // test for (known) near obstacles
  if (ap2->prev == NULL and
      par->withinRobotOutline(par->startPose, oPos, par->obstClearanceMinimum))
  { // too close to side of robot - must be a detection error, or
    // a self detection - e.g. wheels on an smr.
    close = false;
  }
  else if (ap1->next != NULL  and ap1->oob != NULL and
            oPos.dist(ap1->oPos) < samePoint and
            dist > par->obstClearanceMinimum * 0.7)
    // is the opposing obstacle of a handled point - ignore, except in some cases, where aPos and tPos are far apart
    // resulting in a situation that must be handled here.
    close = false;
  else
  { // it could be same as previous mid-point obstacle too
    odap2 = oPos.dist(ap2->aPos);
    if (oPos.dist(ap1->aPos) < samePoint or
        odap2 < samePoint)
      close = false;
      // already known obstacle point should not be ignored, as it
      // catches an early mid-point, when obstacle is late on the path and
      // the turncentre is to be reversed before and after ap2.
      // else if (ap2->oob != NULL and ap2->prev != NULL)
      //  close = oPos.dist(ap2->oPos) > samePoint;
    else
      close = true;
    //
    if (not close and odap2 >= samePoint)
    { // this should be handled safely already, excepy if previous midpoint is
      // far away due to large turn angle at this point.
      a1 = atan2(oPos.y - ap2->aPos.y, oPos.x - ap2->aPos.x);
      a2 = atan2(ap1->mid.y - ap2->aPos.y, ap1->mid.x - ap2->aPos.x);
      // this limit of pi/4 is not absolute, ome may argue, that
      // less is safer, but may lead to impossible manoeuvres
      if (fabs(limitToPi(a2 - a1)) > M_PI / 4.0)
      {
        printf("maybe ignoring this obstacle point is an error, and next line should be in\n");
        //close = true;
      }
    }
  }
/*  if (close and not (ap1->isTight() or ap2->isTight())
  { // we are using open rules, maybe just change to tight rules and ignore
    d = oPos.dist(ap2->mid);
    if (d < (par.getMinTurnRad() * 2.0))
    { // probably only relevant if ap2 is start point.
      ap2->useTight = true;
      close = dist < (par->obstClearanceMinimum - par->obstClearanceDesired);
    }
  }*/
  if (close and dist >= par->obstClearanceMinimum * 0.7)
  { // test if tight rules should apply - as they should close to existing
    // waypoints. 70% of minimum clearence must remain.
    // clearence radius to allow a full turn
    dMin = par->getSafeDistance(ap1->avoidLeft, false, false, false) * 2.0;
    d = ap2->mid.getDistance(oPos);
    if (d < dMin)
      // near previous point
      close = false;
    else
    { // try next waypoint too
      d = ap1->mid.getDistance(oPos);
      if (d < dMin)
        // near next point
        close = false;
    }
  }
  return close;
}

//////////////////////////////////////////////////////////////////////////

bool UAvoidPath2::isAfterDestination(UPosition oPos, UAvoidPoint * ap1)
{
  bool postObst;
  U2Dseg ls;
  UPose po3;
  UPosition oRel, oRel2;
  UAvoidPoint * ap2 = ap1->prev;
  double t2;
  // is it to be avoided left or right
  ls.setFromPoints(ap2->aPos.x, ap2->aPos.y, ap1->aPos.x, ap1->aPos.y);
  if (false)
  {
    t2 = ls.getPositionOnLine(oPos.x, oPos.y);
    postObst = (t2 > ls.length);
  }
  else
  {
    po3 = ap1->mid; // newPP->getEndPoseV();
    // is it in front of target position for this manoeuvre
    oRel = po3.getMapToPose(oPos);
    // and more in front than the obstacle related to the ap1 mid-pose
    oRel2 = po3.getMapToPose(ap1->aPos);
    // it should further be rather close to the ap1.mid pose (as this is the end pose for the test)
    // if the ap1.mid pose is in a sharp turn, then this last condition is needed.
    postObst = oRel.x > oRel2.x and oRel.dist() < par->getDiagonal(false, false, NULL);
  }
  return postObst;
}

///////////////////////////////////////////////////////////////////////////////

bool UAvoidPath2::avoidToTheLeft(UPosition oPos, UAvoidObst * oObs, bool postObst, UAvoidPoint * ap1)
{
  bool cvLeft;
  UPosition oRel, p1;
  UAvoidPoint * ap2 = ap1->prev;
  double d;
  U2Dseg ls;
  UPose sp;
  //
  if (postObst)
  { // obstacle is past end point, and avoid side may depend on end heading
    oRel = ap1->mid.getMapToPose(oPos);
    cvLeft = oRel.y < 0.0;
  }
  else
  { // use along vis-line - use vis-line to determine avoid side
    // except if obstacle is member of same group as ap1 or ap2 obstacles
    if (ap2->aob == oObs)
      cvLeft = ap2->avoidLeft;
    else if (ap1->aob == oObs)
      cvLeft = ap1->avoidLeft;
    else
    { // not part of end groups
      ls.setFromPoints(ap2->aPos.x, ap2->aPos.y, ap1->aPos.x, ap1->aPos.y);
      d = ls.distanceSigned(oPos.x, oPos.y);
      cvLeft = d < 0.0;
    }
  }
  // if ap1 and ap2 is on an obstacle on the  same obstacle group, and
  // avoid direction is to different sides, then we are entering or leaving a
  // concavity in a group of obstacles. If this is the case: split the group
  if (ap2->aob->grpIdx == oObs->grpIdx and ap2->avoidLeft != cvLeft)
  {
    sp.x = (oPos.x + ap2->aPos.x)/2.0;
    sp.y = (oPos.y + ap2->aPos.y)/2.0;
    // at a normal angle to the opening
    sp.h = atan2(oPos.x - ap2->aPos.x, ap2->aPos.y - oPos.y);
    splitObstGroup(oObs->grpIdx, sp.x, sp.y, sp.h);
    // also reevaluate closest point for ap2, as it probably is wrong
    d = par->getSafeDistance(ap2->avoidLeft, false, false, false);
    // test if this is a tight passage
    setClosestPoint(ap2, d * 2.0);
  }
  else if (ap1->aob->grpIdx == oObs->grpIdx and ap1->avoidLeft != cvLeft)
  {
    sp.x = (oPos.x + ap1->aPos.x)/2.0;
    sp.y = (oPos.y + ap1->aPos.y)/2.0;
    sp.h = atan2(oPos.x - ap1->aPos.x, ap1->aPos.y - oPos.y);
    splitObstGroup(oObs->grpIdx, sp.x, sp.y, sp.h);
    // also reevaluate closest point for ap1, as it probably is wrong
    d = par->getSafeDistance(ap1->avoidLeft, false, false, false);
    // test if this is a tight passage
    setClosestPoint(ap1, d * 2.0);
  }
  return cvLeft;
}

///////////////////////////////////////////////////////

bool UAvoidPath2::setPreStopPoint(bool cvLeft, UPosition oPos, UAvoidPoint * apNew, UAvoidPoint * ap1)
{
  UPosition oRel, oRel2;
  UAvoidPoint * ap2 = ap1->prev;
  UPose po4;
  bool result;
  double turnRad;
  //
  oRel = ap1->mid.getMapToPose(oPos);
  if ((cvLeft and oRel.y < par->frontRight.y) or
      (not cvLeft and oRel.y > par->frontLeft.y))
  { // try solve by approaching from further back
    oRel2 = ap1->mid.getMapToPose(ap2->mid);
    apNew->avoidLeft = oRel2.y < 0.0;
    if (cvLeft)
      po4.x = oRel.x - par->frontRight.x;
    else
      po4.x = oRel.x - par->frontLeft.x;
    po4.x -=  par->obstClearanceDesired;
    po4.h = 0.0;
    po4.y = par->getSafeDistance(apNew->avoidLeft, true, true, true);
    if (apNew->avoidLeft)
      po4.y = -po4.y;
    apNew->aPos = ap1->mid.getPoseToMap(po4);
    turnRad = par->getMinTurnRad();
    po4.y = turnRad;
    po4.h = M_PI / 2.0;
    if (apNew->avoidLeft)
    {
      po4.y = -po4.y;
      po4.h = -po4.h;
    }
    apNew->mCent = ap1->mid.getPoseToMapPose(po4);
    apNew->prePoint = true;
    apNew->generation = generation;
    setMidPoint(apNew, turnRad);
    result = true;
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////////////

bool UAvoidPath2::toReplaceOldPoint(UAvoidPoint * apNew, UAvoidPoint * apOld, UAvoidPoint * excludedList)
{
  bool result;
  UAvoidPoint * apt;
  const double samePointDist = 0.03;
  double replaceDist;
  double d;
  //
  // do not replace start and end points.
  result = (apOld->prev != NULL and apOld->next != NULL);
  if (result)
    // do not replace prepoints
    result = not apOld->prePoint;
  if (result)
  { // removable, if in same group, and not too far away.
    if (apOld->aob->grpIdx == apNew->aob->grpIdx)
    { // same group, but is it near by
      d = apNew->aPos.dist(apOld->aPos);
      replaceDist = fmax(par->getMinOpening(), par->getMinTurnRad());
      result = d < replaceDist;
    }
    else
      // not in the same group
      result = false;
  }
  if (result)
  { // is the old point excluded earlier, then do not do it again
    apt = excludedList;
    while (apt != NULL)
    { // is it removed before
      d = apOld->aPos.dist(apt->aPos);
      if (d < samePointDist)
      { // we have tryed to remove this point before, so keep it this time.
        result = false;
        break;
      }
      apt = apt->next;
    }
  }
  return result;
}
  
///////////////////////////////////////////////

double UAvoidPath2::isInsidePoly(UPosition pos, int polyMan0, int * manHit, double limit)
{ // NB! limit should be negative (or zero) to work
  UPolygon * polyIncl;
  UPolygon * polyExcl;
  double d, d1, d2;
  int m, mh = *manHit;
  //
  d2 = limit;
  for (m = maxi(*manHit - 1, 0); m <= *manHit; m++)
  {
    polyIncl = polys[polyMan0 + m * 2];
    d = polyIncl->getDistance(pos.x, pos.y, NULL, NULL);
    if (d < d2)
    {
      polyExcl = polys[polyMan0 + m * 2 + 1];
      if (polyExcl->getPointsCnt() > 0)
        d1 = polyExcl->getDistance(pos.x, pos.y, NULL, NULL);
      else
        d1 = 1e4;
      if (d1 > -limit)
      { // not in exclude polygon)
        d2 = d;
        mh = m;
      }
    }
  }
  if (d2 < limit)
    *manHit = mh;
  return d2;
}

////////////////////////////////////////////////////////

bool UAvoidPath2::isCrossingManPoly(UPolygon * obst, UPosition * pos, double * dist, int polyMan0, int * manHit, double limit)
{
  UPolygon * polyIncl;
  UPolygon * polyExcl;
  double d, d1; //, d2;
  int m, n;
  bool found = false;
  const int MXC = 6;
  UPosition xes[MXC];
  int k1, k2, k1m, k2m;
  double dMax;
  //
  //d2 = limit;
  for (m = 0; m < (polysCnt - polyMan0) / 2; m++)
  {
    polyIncl = polys[polyMan0 + m * 2];
    // find all crossings between these polygons
    obst->isOverlappingXY(polyIncl, &n, xes, MXC);
    if (n > 1)
    {
      dMax = 0.01; // to avoid same point crossings
      k1m = -1;
      k2m = -1;
      for (k1 = 0; k1 < n - 1; k1++)
      { // all points compared with all the rest
        for (k2 = k1 + 1; k2 < n; k2++)
        { // get xy distance between these points
          d = xes[k1].distXY(xes[k2]);
          if (d > dMax)
          {
            dMax = d;
            k1m = k1;
            k2m = k2;
          }
        }
      }
      found = (k1m >= 0);
      if (found)
      { // get midpoint between crossings - should signal the
        // largest overlap.
        pos->x = (xes[k1m].x + xes[k2m].x) / 2.0;
        pos->y = (xes[k1m].y + xes[k2m].y) / 2.0;
        polyExcl = polys[polyMan0 + m * 2 + 1];
        if (polyExcl->getPointsCnt() > 0)
          d1 = polyExcl->getDistance(pos->x, pos->y, NULL, NULL);
        else
          d1 = 1e4;
        found = (d1 > -limit);
        if (found)
          break;
      }
    }
  }
  if (found)
  { // there is crossings
    // just to be sure that the point is on the edge of the obstacle polygon
    obst->getClosestDistance(pos->x, pos->y, 0.1, pos);
    if (dist != NULL)
      *dist = polyIncl->getDistance(pos->x, pos->y);
    if (manHit != NULL)
      *manHit = m;
  }
  return found;
}

///////////////////////////////////////////////////

void UAvoidPath2::splitObstGroup(int grpIdx, double x, double y, double h)
{ // split this obstacle group the furthest away from the x,y position
  UPosition pcog;
  double d;
  UAvoidObst * og, *og0, *ogNew = NULL;
  U2Dlined split;
  int i, idxNew;
  //
  if (aogsCnt == MAX_AOG_OBST_GRPS)
  {
    printf("UAvoidPath2::splitObstGroup: ***** failed, as there is no more group pointers available\n");
    return;
  }
  // move last 2 obstacle (start and end) to make space for new group.
  for (i = aogsCnt; i > aogsCnt - 2; i--)
  {
    aogs[i] = aogs[i-1];
    aogs[i]->grpIdx = i;
  }
  idxNew = aogsCnt - 2;
  aogs[idxNew] = NULL;
  aogsCnt++;
  //
  // find split support point
  // debug
/*  printf("Splitting group %d - (%.2fx,%.2fy,%.3frad) obstacles:", grpIdx, x, y, h);
  og = aogs[grpIdx];
  while (og != NULL)
  {
    printf(" %lu", og->obst->getSerial());
    og = og->grp;
  }
  printf("\n");*/
  // debug end
  // make split line
  split.setPH(x, y, h);
  // make the split
  og0 = NULL;
  og = aogs[grpIdx];
  while (og != NULL)
  { // divide all obstacles in group
    pcog = og->obst->getCogXY();
    d = split.distanceSigned(pcog.x, pcog.y);
    if (d > 0.0)
    { // move to new group
      if (aogs[idxNew] == NULL)
        aogs[idxNew] = og;
      else
        ogNew->grp = og;
      ogNew = og;
      ogNew->grpIdx = idxNew;
      // remove from old list
      if (og0 == NULL)
        aogs[grpIdx] = og->grp;
      else
        og0->grp = og->grp;
    }
    else
      og0 = og;
    og = og->grp;
  }
  // terminate new group
  if (ogNew != NULL)
    ogNew->grp = NULL;
  // if one of the groups are NULL, then
  // the rest must be moved
  bool moveBack = false;
  if (aogs[grpIdx] == NULL)
  {
    aogs[grpIdx] = aogs[idxNew];
    og = aogs[grpIdx];
    while (og != NULL)
    { // reassign group index
      og->grpIdx = grpIdx;
      og = og->grp;
    }
    moveBack = true;
    printf("UAvoidPath2::splitObstGroup: Noone left in source group! - move back\n");
  }
  else if (aogs[idxNew] == NULL)
  {
    moveBack = true;
    printf("UAvoidPath2::splitObstGroup: Noone in new group! - move back\n");
  }
  if (moveBack)
  {
    for (i = aogsCnt - 2; i < aogsCnt; i++)
    {
      aogs[i - 1] = aogs[i];
      aogs[i - 1]->grpIdx = i - 1;
    }
    aogsCnt--;
  }
  // debug
/*  printf("  new group %d - (%.2fx,%.2fy,%.3frad) obstacles:", grpIdx, x, y, h);
  og = aogs[grpIdx];
  while (og != NULL)
  {
    printf(" %lu", og->obst->getSerial());
    og = og->grp;
  }
  printf("\n");
  printf("  new group %d - (%.2fx,%.2fy,%.3frad) obstacles:", idxNew, x, y, h);
  og = aogs[idxNew];
  while (og != NULL)
  {
    printf(" %lu", og->obst->getSerial());
    og = og->grp;
  }
  printf("\n");*/
  // debug end
}

////////////////////////////////////////////////////////

void UAvoidPath2::addNoVisLinesAsObstacles(UAvoidObst * aogs[], const int aogsCnt)
{
  UAvoidObst * og, *aogNew, *ogt;
  int i;
  ULineSegment seg;
  UObstacle * ob;
  bool isOK = true;
  bool isIn;
  UPosition p1, p2, p3;
  UPolygon40 poly;
  //
  aogsObstCnt = 0;
  for (i = 0; i < aogsCnt; i++)
  {
    og = aogs[i];
    while (og != NULL and isOK)
    { // change one-point obstacles to short triangles - in x-direction
      // 0.026 m long and y 0.013.
      // NB! this changes the obstacle in in the obstacle history store too.
      if (og->obst->getPointsCnt() == 1 and og->noVisCnt == 0)
      { // change isolated point to a short triangle
        p1 = og->obst->getPoint(0);
        p1.x -= 0.013;
        og->obst->setPoint(0, p1);
        p1.x += 0.026;
        og->obst->add(p1);
        p1.x -= 0.013;
        p1.y += 0.013;
        og->obst->add(p1);
      }
      else if (og->obst->getPointsCnt() == 2)
      { // extend to square 0.4cm wide and make CCV oriented polygon
        og->obst->copyTo(&poly);
        seg = og->obst->getSegment(0);
        p1 = seg.pos;
        p1.y += seg.vec.x * 0.004;
        p1.x -= seg.vec.y * 0.004;
        poly.add(p1);
        p1 = seg.getOtherEnd();
        p1.y += seg.vec.x * 0.004;
        p1.x -= seg.vec.y * 0.004;
        poly.add(p1);
        poly.extractConvexTo(og->obst);
        og->obst->setValid(true);
      }
      og = og->grp;
    }
    og = aogs[i];
    while (og != NULL and isOK)
    { // convert no-visibility lines to obstacles
      for (int j = 0; j < og->noVisCnt; j++)
      {
        ogt = og->grp;
        isIn = false;
        while (ogt != NULL and not isIn)
        {
          isIn = ogt == og->noVis[j].aobOther;
          ogt = ogt->grp;
        }
        if (not isIn)
          // this no-visibility connection is in already
          continue;
        // this no-visibility line shoule be in, as it
        // connects to a group member further down the list.
        // -- unless the two obstacles overlap
        if (not og->noVis[j].aobThis->obst->isOverlappingXYconvex2(og->noVis[j].aobOther->obst, -0.001))
        { // is not overlapping so line is needed.
          seg = og->noVis[j].getNoVisSegment();
          // get new obstacle
          if (aogsObst[aogsObstCnt] == NULL)
            aogsObst[aogsObstCnt] = new UObstacle;
          ob = aogsObst[aogsObstCnt];
          aogsObstCnt++;
          // fill obstacle
          ob->setPoseFirst(og->obst->getPoseFirst());
          ob->setSerial(++maxSerial);
          ob->setValid(true);
          poly.clear();
          // extend the two endpoints to be sure to overlap connected obstacles
          p1 = seg.getPositionOnLine(-0.005);
          p2 = seg.getPositionOnLine(seg.length + 0.005);
          // make a small side (5mm) at both ends
          p3 = p1; 
          p1.x -= seg.vec.y * 0.002;
          p1.y += seg.vec.x * 0.002;
          p3.x += seg.vec.y * 0.003;
          p3.y -= seg.vec.x * 0.003;
          poly.add(p3);
          poly.add(p1);
          // and the other end
          p3 = p2;
          p2.x -= seg.vec.y * 0.002;
          p2.y += seg.vec.x * 0.002;
          p3.x += seg.vec.y * 0.003;
          p3.y -= seg.vec.x * 0.003;
          // add all 3 points and make it a convex CCV polygon
          poly.add(p3);
          poly.add(p2);
          poly.extractConvexTo(ob);
          ob->setValid(true);
          // get new avoid obstacle entry
          aogNew = getEmptyAog();
          isOK = aogNew != NULL;
          if (not isOK)
            break;
          aogNew->obst = ob;
          for (int k = 0; k < ob->getPointsCnt(); k++)
          {
            aogNew->badEdges[k] = 0;
            aogNew->nogoEdge[k] = false;
            aogNew->nogoVertex[k] = true;
          }
          // add to group
          aogNew->grp = og->grp;
          aogNew->grpIdx = og->grpIdx;
          og->grp = aogNew;
          // stop if no more space
          isOK = aogsObstCnt < MAX_LOCAL_OBSTS;
          if (not isOK)
            break;
        }
      }
      og = og->grp;
    }
    if (not isOK)
      break;
  }
  if (not isOK)
    printf("Too many noVisibility obstacles (%d/%d) or too many groups (%d/%d) - ignoreg the rest\n", aogsObstCnt, MAX_LOCAL_OBSTS, aogsCnt, MAX_AOG_OBST_GRPS);
}

////////////////////////////////////////////////////

UAvoidPoint * UAvoidPath2::createCellBasedPointList(UAvoidCellVertex * cvs, const int cvsCnt)
{
  // create point list from link list
  UAvoidPoint * ap, * apn;
  UAvoidPoint * result = NULL;
  UPose p1, p2;
  UAvoidCellVertex * cv1;
  int i;
  double dMin;
  // start pose 
  ap = getEmptyPoint();
  ap->generation = -1;
  p1 = par->startPose;
  ap->aPos.set(p1.x, p1.y, 0.0);
  ap->avoidLeft = false; // not relevant for first point
  ap->aob = aogs[aogsCnt - 2];
  result = ap;
  // search cell vertex list for poinrts to avoid.
  for (i = 0; i < cvsCnt; i++)
  {
    cv1 = &cvs[i];
    if (cv1->obst != NULL)
    { // there is an obstacle to avoid
      apn = getEmptyPoint();
      apn->generation = -1;
      apn->aPos = cv1->pos;
      apn->avoidLeft = cv1->avoidLeft;
      apn->aob = cv1->obst;
      // link it in
      ap->insertAfter(apn);
      ap = ap->next;
    }
  }
  // then add also the exit point
  apn = getEmptyPoint();
  apn->generation = -1;
  p1 = par->exitPose;
  apn->aPos.set(p1.x, p1.y, 0.0);
  apn->avoidLeft = false; // not relevant for exit point
  apn->aob = aogs[aogsCnt - 1];
  apn->next = NULL;
  ap->insertAfter(apn);
  //
  ap = result;
  i = 0;
  dMin = par->getSafeDistance(ap->avoidLeft, false, false, false);
  while (ap != NULL)
  { // test all groups except start and end (not tight and allow turns)
    // except start and exit as these are not relevant
    if (ap->aob->isStart or ap->aob->isExit)
      // reduced search radius
      setClosestPoint(ap, dMin);
    else
      setClosestPoint(ap, dMin * 2.0);
    ap->generation = generation;
    // debug
    printf("Avoid point %d at %.2fx,%.2fy avoidleft=%s\n", i++, ap->aPos.x, ap->aPos.y, bool2str(ap->avoidLeft));
    // debug end
    ap = ap->next;
  }
  return result;
}


