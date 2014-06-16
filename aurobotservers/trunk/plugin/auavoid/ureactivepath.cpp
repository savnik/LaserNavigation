/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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

#include <umap4/uposev.h>

#include "ureactivepath.h"

/////////////////////////////////////////////////////////

#define WPF_ROUTE_GO_LEFT     0
#define WPF_ROUTE_GO_CENTER   1
#define WPF_ROUTE_GO_RIGHT    2
#define WPF_ROUTE_GO_DIRECT   3
#define ROBOTWIDTH            0.5
//#define WPF_SECURITY_DISTANCE 0.2
#define ROBOTLENGTH           1.0

///////////////////////////////////////////

UReacRoadLines::UReacRoadLines()
{
  clear();
}

///////////////////////////////////////////

void UReacRoadLines::clear()
{
  leftCnt = 0;
  rightCnt = 0;
}

///////////////////////////////////////////

void UReacRoadLines::addLeftLine(ULineSegment * firstSeg, UPolygon * lineHist)
{
  if (leftCnt < MAX_ROAD_SEGS)
  {
    left[leftCnt].setLine(firstSeg, lineHist);
    leftCnt++;
  }
}

///////////////////////////////////////////

void UReacRoadLines::addRightLine(ULineSegment * firstSeg, UPolygon * lineHist)
{
  if (rightCnt < MAX_ROAD_SEGS)
  {
    right[rightCnt].setLine(firstSeg, lineHist);
    rightCnt++;
  }
}

///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

UReacObstGrps::UReacObstGrps()
{
  clear();
}

///////////////////////////////////////////

void UReacObstGrps::clear()
{
  obstsCnt = 0;
}

///////////////////////////////////////////

void UReacObstGrps::addObstGrp(UObstacleGroup * obstGrp)
{
  if (obstsCnt < MAX_OBST_GRPS  and obstGrp != NULL)
  {
    obsts[obstsCnt] = obstGrp;
    obstsCnt++;
  }
}

///////////////////////////////////////////

UObstacleGroup * UReacObstGrps::getGroup(int idx)
{
  UObstacleGroup * result;
  //
  if (idx >= 0 and idx < obstsCnt)
    result = obsts[idx];
  else
    result = NULL;
  //
  return result;
}

///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////



UObstInfo::UObstInfo()
{
  listMax = LIST_SIZE;
  clear();
}

/////////////////////////////////////////////////////////

UObstInfo::~UObstInfo()
{
}

/////////////////////////////////////////////////////////

int UObstInfo::getObstInfo(UObstacle * ob)
{
  int result = -1;
  int i;
  //
  for (i = 0; i < obstCnt; i++)
  {
    if (obst[i] == ob)
    {
      result = info[i];
      break;
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

bool UObstInfo::setObstInfo(UObstacle * ob, int value)
{
  bool result = true;
  int i;
  //
  // debug
  if (value > 1)
    printf("UObstInfo::setObstInfo set a value if %d - should be 0 or 1\n", value);
  // debug edn
  for (i = 0; i <= obstCnt; i++)
  {
    if (i == obstCnt)
    { // store value and obstacle reference
      if (i < getMaxSize())
      {
        obstCnt = i + 1;
        obst[i] = ob;
        info[i] = value;
        break;
      }
      else
        result = false;
    }
    else if (obst[i] == ob)
    {
      info[i] = value;
      break;
    }
  }
  return result;
}

/////////////////////////////////////////////////////////

void UObstInfo::print(const char * prestring)
{
  int i;
  UPosition p1;
  //
  printf("%s holds %d obst(s) (of %d)\n", prestring,  obstCnt, getMaxSize());
  for (i = 0; i < obstCnt; i++)
  {
    p1 = obst[i]->getPoint(0);
    printf(" ... %d at (%.2fx,%.2fy)\n", info[i], p1.x, p1.y);
  }
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


UReactivePath::UReactivePath()
{
  clear();
  man = new UManSeq();
}

////////////////////////////////////////////////////////

UReactivePath::~UReactivePath()
{
  delete man;
}

////////////////////////////////////////////////////////

void UReactivePath::clear()
{ // clear path
  //int i;
  //
  intervalsCnt = 0;
  //routeCnt = 0;
  //for (i = 0; i < MAX_LASER_SCANS; i++)
  //  intervals[i] = NULL;
  pathUsed = false;
  endsInObst = false;
  crash = false;
  midPosesCnt = 0;
  exitPose.vel = 0.345;
}

////////////////////////////////////////////////////////

void UReactivePath::copy(UReactivePath * source)
{ // clear path
  int i;
  //
  clear();
  //
  routeAngleExit = source->getRouteAngleExit();
  routeAngleFirst = source->getRouteAngleFirst();
  routeDist = source->getRouteDist();
  routeSD = source->getRouteSD();
  //

  // passable intervals
/*  for (i = 0; i < source->getIntervalCnt(); i++)
    intervals[i] = source->getInterval(i);
  intervalsCnt = source->getIntervalCnt();*/
  // path used
  pathUsed = false;
  endsInObst = false;
  // manoeuvre
  man->copy(source->getManSeq());
  //
  crash = source->isACrash();
  valid = source->isValid();
  midPosesCnt = source->getMidPosesCnt();
  for (i = 0; i < midPosesCnt; i++)
    midPoses[i] = source->getMidPose(i);
}

////////////////////////////////////////////////////////

int UReactivePath::testForObstacleNearExit(UReacObstGrps * obsts,
                               const int usedEdge,
                               UPosition * posExit,
                               ULineSegment * seg,
                               double obstDist,
                                       int n)
{
  int result = 0; // 0=no change (isOK), 1= change OK, 2=no valid pos found
  UObstacleGroup * obstGrp;
  UObstacle * obs;
  UPosition cog, newPos;
  double dc, d, d2, t;
  int where;
  int idx, res2, i, j;
  UPosition posPol;
  //
  for (i = 0; i < obsts->getGroupsCnt(); i++)
  { // find nearest obstacle to the left and right
    obstGrp = obsts->getGroup(i);
    for (j = 0; j < obstGrp->getObstsCnt(); j++)
    { // get next obstacle
      obs = obstGrp->getObstacle(j);
      //
      cog = obs->getCogXY();
      dc = obs->getCogXYmaxDist();
      d = hypot(posExit->x - cog.x, posExit->y - cog.y);
      if (d < dc)
      {
        d = obs->getDistanceXYsigned2(*posExit, &idx, &posPol, NULL);
        if (d < obstDist)
        { // something needs to be done
          t = seg->getPositionOnLine(posPol);
          // get perpendicular distance
          d2 = seg->getDistanceXYSigned(posPol, &where);
          // get distance to move exit point
          if (where == 0)
            d = sqrt(sqr(obstDist) - sqr(d2));
          else
            d = obstDist;
          switch (usedEdge)
          {
            case WPF_ROUTE_GO_LEFT:
              t -= (d + 0.01); // plus a bit to avoid rounding errors
              break;
            case WPF_ROUTE_GO_RIGHT:
              t += (d + 0.01); // plus a bit to avoid rounding errors
              break;
            default:
              // should not be (leave as is)
              break;
          }
          if ((t > obstDist) and (t < seg->length - obstDist) and n < 3)
          { // increase iteration count
            n++;
            newPos = seg->getPositionOnLine(t);
            // test this position
            res2 = testForObstacleNearExit(obsts, usedEdge, &newPos, seg, obstDist, n);
            if (res2 == 2)
              result = 2;
            else
            {
              result = 1;
              // debug
              // printf("UReactivePath::testForObstacleNearExit: moved from %.3fx,%.3fy to %.3fx,%.3fy\n",
              //       posExit->x, posExit->y, newPos.x, newPos.y);
              // debug end
              *posExit = newPos;
            }
          }
          else
            result = 2;
        }
      }
      if (result != 0)
        break;
    }
    if (result != 0)
      break;
  }
  return result;
}

////////////////////////////////////////////////////////

bool UReactivePath::findObstAvoidRoute(UReacObstGrps * obsts,
                                       UReacRoadLines * roads,
                                       UAvoidPathPool * paths,
                                       bool ignoreObstacles,
                                       bool directTestOnly,
                                      bool driveon, double turnRad)


/*    int edge,
                            UPose destination, // if go-direct to pos only
                            UPoseV pose, // current robot pose
                            UObstacleHist * obsts,
                            const int obstsCnt,
                            UAvoidPathPool * pathPool,
                            const double laserTilt,
                            const double distFromEdge,
                            double maxVel,
                            double maxAcc, double maxTurnAcc,
                            bool ignoreObstacles)*/
{
  // --- first find start and end
  // --- build a route avoiding obstacles on basis of first cut
  // --- if crossing an obstacle - try avoid left OR right
  // ---                           may later try both left and right
  bool result;
  int i;
  UPose dest; // destination position for obstacle avoidance manoeuvre
  UPoseV toPose, midPose;
  double okInitVel; // estimated OK initial velocity - if a break is needed
//  double d, a, v;
//  const double MIN_SPEED = 0.2;
//  double MAX_BREAK_ACC = paths->getMaxAcc() * 2.5;
  double maxTurnAcc, maxAcc;
  //bool insideSegment = true;
  UObstInfo obstSideList;
  UPoseV startPose;
  double maxVel, minTurnRadius;
  double dist;
  bool ignoreLastArc = false;
  //
  toPose = exitPose;
  // set start position and velocity
  man->releaseAllMan();
  startPose = paths->getStartPoseVel();
  maxTurnAcc = paths->getMaxTurnAcc();
  minTurnRadius = paths->getMinTurnRad();
  dist = startPose.getDistance(toPose);
  if (dist < 2.0 * minTurnRadius)
  { // getting close allow tighter turns to avoid no-solution lockups
    // make first turn sharp and remove last turn
    minTurnRadius *= 0.3;
    ignoreLastArc = true;
  }
  maxAcc = paths->getMaxAcc();
  maxVel = maxd(exitPose.getVel(), startPose.getVel());
  if (driveon)
    result = man->addManDriveon(startPose, toPose, minTurnRadius);
  else
    result = man->addMan(startPose, toPose,
                         maxAcc, maxTurnAcc,  minTurnRadius, &okInitVel);

  if (ignoreLastArc)
  {
    man->removeLastArc();
  }
  //
  // set valid for now
  valid = result;
  if (not valid and paths->getCrashTest())
  {
    valid = true;
    crash = true;
  }
  if (result and not ignoreObstacles)
  { // change route to avoid obstacles
    // debug
    if (man->getP2PCnt() < 1)
      printf("UReactivePath::findObstAvoidRoute: NO initial pose-to-pose!!!?\n");
    // debug end
    i = 0;
    result = avoidObst(obsts,
                      paths,
                      obstSideList, true,
                      maxVel, maxAcc, maxTurnAcc, minTurnRadius, &i, 0, directTestOnly, driveon, turnRad);
  }
  return result;
}

/////////////////////////////////////////////////////

bool UReactivePath::avoidObst(
                        UReacObstGrps * obsts,
                        UAvoidPathPool * pathPool,
                        UObstInfo obstSideList,
                        bool obstSideSpawn,
                        //const double distFromEdge,
                        double maxVel,
                        double maxAcc, double maxTurnAcc, double minTurnRad,
                        int * spawnCnt, int spawnNestLevel, bool directTestOnly,
                             bool driveon, double turnRad)
{
  bool result;
  int i, j, k, n;
  int loop = -1, side;
  bool newPoints;
  double dist1;
  UPosition p1, p2, cog;
  int lsEnd;
  UPoseV toPose, midPose, midPose2;
  bool midPose2Set = false;
  UPose pHit; // position on route closest to obstacle
  ULineSegment seg;
  UManPPSeq * mpp;
  double hitT; // distance into route sequence
  int manIdx; // index to route sequence
  double okInitVel; // estimated OK initial velocity - if a break is needed
  double a, d1, d2, passWidth = 0.0;
  bool routePossible = true;
  bool newEndpoint = false;
  // closest obstacle (left (0) and right (1))
  bool bothSides;
  UObstacleGroup * obstGrp;
  UObstacle * obs;
  double obstDistLimit;
  bool replaceNeeded = false;
  UObstInfo osl1;
  UObstInfo osl2;
  UObstInfo osl = obstSideList; // local copy of obstacle side list
  UObstInfo * obstSideListUsed = &osl1;
  UObstInfo * ignoreObst = &osl2;
  UReactivePath * altPath;
  UPoseV startPose;
  bool fixateMidPose;
  bool isOK;
  //double startV;
  int maxNestedLevels = pathPool->getMaxNestedLevels();
  int maxAvoidLoops = pathPool->getMaxAvoidLoops();
  int maxSpawnCnt = pathPool->getMaxSpawnCnt();
/*  const int MAX_NESTED_LEVELS = 2;
  const int MAX_LOOPS = 8;
  const int MAX_SPAWN_CNT = 4;*/
  bool toAvoid;
  int cogSide;
  double minDist[2];
  int minDistIntervalG[2];
  //int minDistIntervalO[2];
  UPosition minDistPos[2]; // obstacle position closest to path
  //int minCogSide[2]; // obstacle center og gravity position to route
  UPose minHitPose[2]; // pose hit on the route
  int minIdx1[2] = {0,0};
  int minIdx2[2] = {0,0};
  UObstacle * minObs[2] = {NULL, NULL};
  int minObsVertex[2]; // min distance obstacle vertex (segment) index
  //int minObsEnd[2];
  int wS = 0; // min distance obstacle end of segment
  const int MINIMUM_VERTEX_COUNT = 2;
  //
  result = false;
  // get destination point (desired exit point)
  toPose = man->getP2P(man->getP2PCnt() - 1)->getEndPoseV();
  // get startvel (current pose)
  startPose = man->getP2P(0)->getStartPoseV();
  *spawnCnt += 1;
  obstDistLimit = pathPool->getObstMinDist();
  //
  for (loop = 0; loop < maxAvoidLoops; loop++)
  {
    // debug
/*    if (man->getP2PCnt() < 1)
      printf("UReactivePath::avoidObst: NO initial pose-to-pose!!!?\n");*/
    // debug end
    replaceNeeded = true;
    newPoints = false;
    minDist[0] = obstDistLimit + 0.1;
    minDist[1] = obstDistLimit + 0.1;
    minDistIntervalG[0] = -1; // left
    minDistIntervalG[1] = -1; // right
//     minDistIntervalO[0] = -1; // left
//     minDistIntervalO[1] = -1; // right
    minObs[0] = NULL;
    minObs[1] = NULL;
    for (i = 0; i < obsts->getGroupsCnt(); i++)
    { // find nearest obstacle to the left and right
      obstGrp = obsts->getGroup(i);
      for (j = 0; j < obstGrp->getObstsCnt(); j++)
      { // get next obstacle
        obs = obstGrp->getObstacle(j);
        // test if validated obstacle
        if (not obs->isValid())
        { // small obstacles not seen in two scans should be ignored
          continue;
        }
        // test if obstacles are to be ignored
        if (ignoreObst->getObstInfo(obs) == 1)
        { // continue with next obstacle
          continue;
        }
        // find distance to planned route from obstacle centre
        cog = obs->getCogXY();
        dist1 = man->getDistanceXYSigned(cog, &lsEnd,
                                         true, 10.0 + obs->getCogXYmaxDist(),
                                         &pHit, &manIdx, &hitT);
        if (dist1 < 0.0)
          cogSide = 0; // to the left of current route
        else
          cogSide = 1; // to thr right of cureent route
        // get signed distance, where positive is to the right
        side = obstSideList.getObstInfo(obs);
        if (side < 0)
          // use center og gravity side as passage side
          side = cogSide;
        // debug
        if (side > 1)
          printf("UReactivePath::avoidObst: Value of side if out of range 1!! side=%d\n", side);
        // debug end
        // try avoid if a vertex may get closer than security distance to route
        toAvoid = (fabs(dist1) - obstDistLimit) < obs->getCogXYmaxDist();
        // debug (do not use precalculated distance)
/*        toAvoid = true;
        obs->setCogXYvalid(false);*/
        // debug end
        // now try all vertices of obstacle, and find the closest
        if (lsEnd != 3 and toAvoid)
        { // not too far away
          for (k = 0; k < obs->getPointsCnt(); k++)
          { // test all vertieces
            seg = obs->getSegment(k);
            dist1 = man->getMinDistanceXYSigned(&seg, &wS, &p1,
                side==1, obstDistLimit + 0.5,  &lsEnd, &pHit, &manIdx);
            // save obstacle closest to planned route line
            toAvoid = (lsEnd != 3) and (dist1 < minDist[side]);
            // debug
            if (side > 1)
              printf("UReactivePath::avoidObst: Value of side if out of range 2!! side=%d\n", side);
            // debug end
            if (toAvoid)
            { // this is a candidate
              // test if endpoint is inside an obstacle, in which case the obstacle is to be ignored
              if (ignoreObst->getObstInfo(obs) < 0)
              { // not tested before
                // and route flagged as ending in an obstacle
                if (obs->isInsideConvex(toPose.x, toPose.y, 0.0))
                { // no point in attempt to avoid obstacle, mark as ending in obst ...
                  endsInObst = true;
                  endsInObstHere = minDistPos[side];
                  // set as should be ignored
                  ignoreObst->setObstInfo(obs, 1);
                  toAvoid = false;
                  // no need to test further vertices
                  break;
                }
                else
                  // set as tested with result: should not be ignored.
                  ignoreObst->setObstInfo(obs, 0);
              }
            }
            // debug
            if (side > 1)
              printf("Value of side if out of range!! side=%d\n", side);
            // debug end
            if (toAvoid)
            { // ignore obstacles too close to robot
              // to prohibit attempt to side-step
              d1 = hypot(startPose.y - pHit.y, startPose.x - pHit.x);
              if (d1 < pathPool->getObstMinDist())
                toAvoid = false;
            }
            if (toAvoid)
            { // ignore obstacles too close to exit pose
              // (assuming it will be solved later)
              d1 = hypot(toPose.y - pHit.y, toPose.x - pHit.x);
              if (d1 < obstDistLimit)
                toAvoid = false;
            }
/*            if (toAvoid)
            { // ignore if too close to endpoint
              d2 = toPose.getDistance(p1);
              if (d2 < WPF_SECURITY_DISTANCE)
                toAvoid = false;
            }*/
            // debug
            if (side > 1)
              printf("Value of side if out of range!! side=%d\n", side);
            // debug end
            if (toAvoid)
            { // this should be considered as a candidate obstacle
              minDist[side] = dist1; // distance from path-line to obstacle (neg is inside or other side of obstacle)
              minDistIntervalG[side] = i; // passable interval index - group
              //minDistIntervalO[side] = j; // passable interval index - obstacle
              minDistPos[side] = p1; // obstacle position (closest to route)
              minHitPose[side] = pHit; // position hit on route (closest to obstacle)
              minIdx1[side] = manIdx; // index to sequence that need replacement (first)
              minIdx2[side] = manIdx; // index to sequence that need replacement (last)
              minObs[side] = obs; // offending obstacle
              //minCogSide[side] = cogSide;
              minObsVertex[side] = k;
              //minObsEnd[side] = wS;
              // debug
              if ((minObs[0] == NULL) and (minDistIntervalG[0] > 0))
              {
                printf("minObs[0] = NULL! (side=%d)  --- start\n", side);
              }
              // debug end
            }
          } // end for (k = vertex
        }
      } // end for (j = obst
    } // end for (i= obst group
    //
    if (directTestOnly)
    {
      result = not ((minDistIntervalG[0] >= 0) or (minDistIntervalG[1] >= 0));
      break;
    }
    // set flag if near obstacle is found at both sides
    bothSides = (minDistIntervalG[0] >= 0) and (minDistIntervalG[1] >= 0);
    //
    if ((minObs[0] == NULL) and (minDistIntervalG[0] > 0))
    {
      printf("minObs[0] = NULL! (2)\n");
    }
    // test for trying to go between overlapping obstacles
    if (bothSides)
    { // reduce safety distance, if already close to object
      p1 = startPose.getPos(0.0);
      d1 = minObs[0]->getDistanceXYsigned2(p1, NULL, NULL, NULL);
      if ((d1 > pathPool->getObstMinMinDist()) and (d1 < obstDistLimit))
        obstDistLimit = d1;
      d1 = minObs[1]->getDistanceXYsigned2(p1, NULL, NULL, NULL);
      if ((d1 > pathPool->getObstMinMinDist()) and (d1 < obstDistLimit))
        obstDistLimit = d1;
      //
      // debug
      if ((minObs[0] == NULL) or (minObs[1] == NULL))
        printf("****** hard error \n");
      // debug end
      if (minObs[0]->isOverlappingXYconvex(minObs[1], ROBOTWIDTH))
      { // try the other way arouund not forced obstacle
        if (obstSideList.getObstInfo(minObs[0]) < 0)
          side = 0;
        else if (obstSideList.getObstInfo(minObs[1]) < 0)
          side = 1;
        else
          side = -1;
        if (side >= 0)
          // try same side around other obstacle too
          result = obstSideList.setObstInfo(minObs[side], ((side + 1) % 2));
        if (result and (side >= 0))
        { // now continue in a new loop
          replaceNeeded = false;
          newPoints = true;
          // but do not count loop
          loop--;
          //continue;
        }
        else
        { // This will not work, terminate path
          routePossible = false;
          replaceNeeded = false;
          // debug
/*          printf("*** Failed to - can not pass between 2 obst (%.2fx,%.2fy) - (%.2fx,%.2fy)\n",
                 minObs[0]->getPoint(0).x, minObs[0]->getPoint(0).y,
                 minObs[1]->getPoint(0).x, minObs[1]->getPoint(0).y);*/
          // debug end
          //break;
        }
      }
    }
    // debug
    if ((minObs[0] == NULL) and (minDistIntervalG[0] > 0))
    {
      printf("minObs[0] = NULL! (3)\n");
    }
    // debug end
    if (replaceNeeded)
    { // optimize minimum distance
      for (side = 0; side < 2; side++)
      { // The minimum distance may be to a side and not just a vertex as tested above
        // so now find the real closest position, for both sides.
        if (minDistIntervalG[side] >= 0)
        {
          if ((minDist[side] < 0.001) and (minObs[side]->getPointsCnt() > 2))
          { // route hits an obstacle (or on the wrong side), find most compromizing vertex
            obs = minObs[side];
            k = minObsVertex[side];
            mpp = man->getP2P(minIdx1[side]);
            getWorstVertex2(obs, side, &minDist[side], &minDistPos[side], mpp);
            while (obs != NULL)
            { // look for other obstacles that need to be passed at the same side
              obs = getNearObstacle(obsts, MINIMUM_VERTEX_COUNT,
                                    ROBOTWIDTH,
                                    minDistPos[side], &k, &obstSideList, obs);
              //
              if (obs != NULL)
              { // use new info instead as old offending position
                // get most offending vertex for new obstacle
                getWorstVertex2(obs, side, &d2, &p1, mpp);
                // add to side list
                // debug
                if (side > 1)
                  printf("Side out of range (0,1) is %d\n", side);
                // debug end
                if (not obstSideList.setObstInfo(obs, side))
                { // no space for more - so stop here
                  // debug
                  printf("Out of UObstacleInfo space\n");
                  // debug end
                  break;
                }
                if (d2 < minDist[side])
                { // save this as the new most offending position/obstacle
                  minDist[side] = d2;
                  minObs[side] = obs;
                  minDistPos[side] = p1;
                }
              }
            }
          }
        }
      }
    }
    //
    if (replaceNeeded)
    { // there is a (potentially solvable) situation
      if ((minDistIntervalG[0] >= 0) or (minDistIntervalG[1] >= 0))
      { // a detour is needed to avoid obstacle
        if (not bothSides)
        { // get offended side
          if (minDistIntervalG[0] >= 0)
            side = 0; // obstacle is to the left, so pass right
          else
            side = 1; // obstacle is to the right, so pass left
        }
        else
        { // get most offended side ...
          if (minDist[0] < minDist[1])
            side = 0;
          else
            side = 1;
          // ... and passage width
          passWidth = minDistPos[0].dist(minDistPos[1]);
          if (passWidth > (2.1 * pathPool->getObstMinMinDist()))
          { // May be near to an established narrow passage
            if (man->getP2P(minIdx1[side])->isFirstFixed() or
                man->getP2P(minIdx2[side])->isLastFixed())
              // try as one-side adjust only
              bothSides = false;
          }
        }
      }
    }
    //
    if (replaceNeeded)
    { // get all involved parts of the total manoeuvre
      // i.e. index to list of pose-to-pose manoeuvre list
      for (i = 0; i < 2; i++)
      { // test if more sequences are to be replaced
        // are inside or on security distance
        if (minDistIntervalG[i] >= 0)
        { // there is an offending obstacle, replace
          // test if this affects the previous or next manoeuvres too
          p1.set(minHitPose[i].x, minHitPose[i].y, 0.0);
          while (true)
          { // include next pose-to-pose waypoint too - if not fixated
            if (man->getP2P(minIdx1[i])->isFirstFixed() and not bothSides)
              break;
            // calculate distance start of manoeuvre to obstacle
            d1 = man->getP2P(minIdx1[i])->getStartDistTo(p1);
            if ((d1 > (ROBOTLENGTH * 0.3)) or (minIdx1[i] == 0))
              break;
            // include the previous manoeuvre too
            minIdx1[i]--;
          }
          while (true)
          { // include next pose-to-pose waypoint too - if not fixated
            if (man->getP2P(minIdx2[i])->isLastFixed() and not bothSides)
              break;
            // calculate distance from path point closest to obst.
            // to other waypoints.
            d2 = man->getP2P(minIdx2[i])->getEndDistTo(p1);
            if ((d2 > (ROBOTLENGTH * 0.3)) or (minIdx2[i] == man->getP2PCnt() - 1))
              break;
            // include the next manoeuvre too
            minIdx2[i]++;
          }
          // now find a new pose at the security distance from
          // the obstacle 'minDistPos' where to go instead
        }
      }
    }
    //
    if (replaceNeeded)
    {  // there is a (potentially solvable) situation
      replaceNeeded = (minDistIntervalG[0] >= 0) or (minDistIntervalG[1] >= 0);
    }
    if (replaceNeeded)
    { //
      // try another waypoint to avoid closest obstacle(s) - if needed
      fixateMidPose = false;
      if (minDist[side] >= obstDistLimit)
      { // this is OK, no further action needed
        // this is needed to be able to catch narrow doors
        // and avoid pushing midPoint backand forth
        //printf("Movement is OK\n");
        replaceNeeded = false;
      }
    }
    if (replaceNeeded)
    { // a detour is needed to avoid obstacle
      // debug
/*      if (bothSides)
        printf(" To y=%.2f, Obsts %2d.%02d and %2d.%02d, depth %d loop %d (%g m)\n",
              man->getP2P(man->getP2PCnt() - 1)->getEndPoseV().y,
              minDistIntervalG[0], minDistIntervalO[0],
              minDistIntervalG[1], minDistIntervalO[1],
              obstSideList.getCnt(), loop, man->getDistance());
      else
        printf(" To y=%.2f, Side %d only     %2d.%02d, depth %d loop %d (%g m)\n",
              man->getP2P(man->getP2PCnt() - 1)->getEndPoseV().y,
              side,
              minDistIntervalG[side],
              minDistIntervalO[side],
              obstSideList.getCnt(), loop, man->getDistance());*/
      // debug end
      //
      // there may be another way around the closest obstacle
      if (obstSideSpawn and
          (obstSideListUsed->getObstInfo(minObs[side]) < 0) and
    //      (minDist[side] < obstDistLimit) and   // not needed - tested in previous if
          (spawnNestLevel < maxNestedLevels) and
          (*spawnCnt < maxSpawnCnt))
      { // the other way around this ibstacle is not tested - try it
        // if not tried before
        if (osl.getObstInfo(minObs[side]) == -1)
        { // not tried yet
          /** @todo create alternative path */
          altPath = pathPool->getNewPath();
          // allow depth 1 only
          if (altPath != NULL)
          { // there is space for another path alternative
            // take list for the path so far ..
            osl = obstSideList;
            // ... and add the new passing exception
            osl.setObstInfo(minObs[side], (side + 1) % 2);
            obstSideListUsed->setObstInfo(minObs[side], (side + 1) % 2);
            // copy this path so far, but now go for the other side
            altPath->copy(this);
            // plan route -- may end invalid, but that is OK
            altPath->avoidObst(obsts, pathPool, osl,
                                obstSideSpawn,
                                maxVel, maxAcc, maxTurnAcc, minTurnRad, spawnCnt,
                                spawnNestLevel + 1, false, driveon, turnRad);

          }
        }
      }
    }
    if (replaceNeeded)
    {
      if ((not bothSides) or (passWidth > (2.0 * pathPool->getObstMinDist())))
      { // one side is offended (mainly)
        // so try a better solution
        // newEndpoint = d2 < WPF_SECURITY_DISTANCE;
        // find new waypoint as tangetnt to distance form obstacle point
        // andallow for small errors
        for (n = 0; n < 10; n++)
        { // try maximum N times iteration to the optimat mid-position.
          // The 'minDistPos[side]' may be wrong especially if the old route
          // crosses the obstacle.
          midPose = getTangentMidPose(minIdx1[side],
                                      minIdx2[side], side,
                                      minDistPos[side], newEndpoint,
                                      obstDistLimit + 0.02);
          if (pathPool->getCrashTest() and midPosesCnt < MAX_MID_POSES)
            midPoses[midPosesCnt++] = midPose;
          // test if mid-distance is to close to other part
          // of obstacle, if so avoid the now closer point
          p1.set(midPose.x, midPose.y, 0.0);
          minDist[side] = minObs[side]->getDistanceXYsigned2(p1, NULL, &minDistPos[side], NULL);
          if (minDist[side] > obstDistLimit)
            // it wont get any better that this
            break;
        }
      }
      else if (passWidth > ROBOTWIDTH)
      { // passage is narrow, go through midpoint
        // get involved manoeuvres
        minIdx1[side] = mini(minIdx1[0], minIdx1[1]);
        minIdx2[side] = maxi(minIdx2[0], minIdx2[1]);
        // keep endpoint for now
        newEndpoint = false;
        routePossible =  getMidPoseHere(minObs[0], minObs[1],
                                minDistPos[0], minDistPos[1], -1,
                                &midPose, &d2,
                                man->getP2P(minIdx1[side])->getStartPoseV());
        if (routePossible)
        { // set new obstacle limit -- d2 is passage width
          obstDistLimit = mind(obstDistLimit, maxd(pathPool->getObstMinMinDist(), (d2 * 0.95)/2.0));
          replaceNeeded = true;
          fixateMidPose = true;
        }
        else
        {
          // debug
          printf("*** Failed to find narrow midPose between (%.2fx,%.2fy) - (%.2fx,%.2fy)\n",
                minObs[0]->getPoint(0).x, minObs[0]->getPoint(0).y,
                minObs[1]->getPoint(0).x, minObs[1]->getPoint(0).y);
          // debug end
        }
      }
      else
      { // no space to pass - maybe another way
        if (obstSideList.getObstInfo(minObs[0]) < 0)
          side = 0;
        else if (obstSideList.getObstInfo(minObs[1]) < 0)
          side = 1;
        else
          side = -1;
        if (side > 0)
          // try other side of obstacle - if not tested before
          result = obstSideList.setObstInfo(minObs[side], ((side + 1) % 2));
        if (result and (side > 0))
        { // now continue in a new loop
          replaceNeeded = false;
          newPoints = true;
          // but do not count loop
          loop--;
          //continue;
        }
        else
        { // This will not work, terminate path
          routePossible = false;
          replaceNeeded = false;
          //break;
          // debug
/*          printf("*** Failed, no space (2) to pass between (%.2fx,%.2fy) - (%.2fx,%.2fy)\n",
                 minObs[0]->getPoint(0).x, minObs[0]->getPoint(0).y,
                 minObs[1]->getPoint(0).x, minObs[1]->getPoint(0).y); */
/*          printf("  -- midDist=%.2f (%.2fx,%.2fy) - (%.2fx,%.2fy)\n",
                 passWidth,
                 minDistPos[0].x, minDistPos[0].y,
                 minDistPos[1].x, minDistPos[1].y);*/
          // debug end
        }
      }
    }
    if (replaceNeeded)
    { // a new mid-pose is calculated, so replace
      if (routePossible and replaceNeeded)
      { // test if replacing the same point
        if (midPose2Set and (midPose.getDistance(midPose2) < 0.01))
        {  // same position,
          if (obstDistLimit > (ROBOTWIDTH/2.0))
          { // try with resuced security margin first
            // debug
//            printf("loop-fail (limit %.3f reduced to %.3f)\n", obstDistLimit, ROBOTWIDTH);
            // debug end
            obstDistLimit = ROBOTWIDTH/2.0;
            replaceNeeded = false;
            newPoints = true;
          }
          else
          { //stop here
            replaceNeeded = false;
            // maybe dump solution??
            routePossible = false;
            // debug
            // debug end
          }
        }
        midPose2Set = true;
        midPose2 = midPose;
      }
    }
    if (replaceNeeded)
    { // set mid-point velocity
      midPose.setVel(maxVel);
      // test distance to endpoint
      d2 = midPose.getDistance(toPose);
      if (d2 < obstDistLimit / 2.0)
      { // close to endpoint, do not overshoot
        a = atan2(midPose.y - toPose.y, midPose.x - toPose.x);
        a = limitToPi(a - toPose.h);
        if (a < (M_PI / 2.0))
        { // midPose is in front of toPose - ignore offending obstacle
          result = true;
          break;
        }
        else
          // too close to endpoint to do safe manoeuvres - stop here
          newEndpoint = true;
      }
      //startV = man->getP2P(minIdx1[side])->getStartPoseV().getVel();
      if (driveon)
        isOK = man->replaceManDriveon(midPose, // new pose to pass
                          fixateMidPose,
                          turnRad, // turn behaviour
                          minIdx1[side], minIdx2[side],    // sequences to replace
                          newEndpoint // new endpoint
                          );
      else
        isOK = man->replaceMan(midPose, // new pose to pass
                               fixateMidPose,
                               maxAcc,  maxTurnAcc, minTurnRad,// acceleration limits
                               minIdx1[side], minIdx2[side],    // sequences to replace
                               newEndpoint, // new endpoint
                               &okInitVel);
/*      while ((okInitVel > 0.02) and
              (okInitVel < startV) and
              isOK)
      { // requires lower initial speed, and is not first manoeuvre
        // replace the previous midPose, replacing the just created
        // pair of poses
        if (newEndpoint)
          minIdx2[side] = minIdx1[side]; // replaced by just 1 sequence
        else
        {
          if (fixateMidPose)
            // keep the just plased mid-pose
            minIdx2[side] = minIdx1[side];
          else
            // OK to move this too
            minIdx2[side] = minIdx1[side] + 1;
        }
        if (minIdx1[side] > 0)
        { // just exit of previous man
          minIdx1[side]--;
          newEndpoint = false; // do not reduce further
          midPose = man->getP2P(minIdx1[side])->getEndPoseV();
          midPose.setVel(okInitVel);
        }
        else
        { // try breaking from current position to get down to okInitVel
          // find distance to break to okVel
          d = (sqr(startPose.getVel()) - sqr(okInitVel))/(2.0 * maxAcc);
          d *= 1.1; // and a bit to avoid floating point compare
          // offset this distance to this pose
          //midPose.x += cos(startPose.h) * d;
          //midPose.y += sin(startPose.h) * d;
          midPose.x = startPose.x + cos(startPose.h) * d;
          midPose.y = startPose.y + sin(startPose.h) * d;
          midPose.h = startPose.h;
          midPose.setVel(okInitVel);
        }
        isOK = man->replaceMan(midPose, // new pose to pass
                                    true, // this mid-pose is OK to move later if needed
                                    maxAcc,  maxTurnAcc, minTurnRad, // acceleration limits
                                    minIdx1[side], minIdx2[side],    // sequences to replace
                                    newEndpoint, // new endpoint
                                    &okInitVel);
      }*/
      routePossible = isOK; //okInitVel > 0.0;
      newPoints = true;
      if (not routePossible)
      {
        // debug
        //printf("*** Failed, to implement new mid-pose at = %.2fx, %.2fy, %.3fh\n",
        //          midPose.x, midPose.y, midPose.h);
        // debug end
      }
    }
    if ((not newPoints) or (not routePossible))
    { // no change, or no result - so exit
      result = routePossible;
      break;
    }
  } // for (loop = 0; loop < MAX_LOOPS ...
  //
  if (loop >= maxAvoidLoops)
  {
    result = false;
    // debug
    //printf("*** Failed to find solution in %d loops\n", loop);
    // debug end
  }
  // debug
/*  printf("loops %d - ", loop);
  obstSideList.print("obstSideList");
  obstSideListUsed->print("obstSideListUsed");
  ignoreObst->print("ignoreObst (1=ignore)");
  if (result)
    man->fprint(stdout, "isOK  ");
  else
  {
    //result = true;
    man->fprint(stdout, "Failed");
  }*/
  // debug end

  if (not result)
  { // should remain valid if crashed paths are to be verified
    valid = pathPool->getCrashTest();
    crash = true;
  }
  // debug
/*  printf("avoidObst result = (%s, %g m), now %d valid paths\n",
         bool2str(result), man->getDistance(), pathPool->getPathValidCnt());*/
  // debug end
  // show anyhow
  // debug tramp test
/*  if ((DEBUGa != 777.0) or (DEBUGb != 777.0) or (DEBUGc != 777.0))
  {
    printf("UReactivePath::avoidObst: TRAMP *** ERROR: %g==%g==%g (!=777.0) !\n",
          DEBUGa, DEBUGb, DEBUGc);
  }*/
  //
  return result;
}

////////////////////////////////////////////////////////

bool UReactivePath::getMidPoseHere(UObstacle * obst1, UObstacle * obst2,
                     UPosition minPos1, UPosition minPos2, int side,
                     UPose * midPose, double * width,
                     UPose fromPose)
{
  int i;
  UPosition pm, p1, p2;
  double d1 = 0.0, d2 = 0.0;
  bool w1, w2;
  int idx1, idx2;
  bool result = false;
  ULineSegment seg;
  double a;
  // iterate to most limiting position
  p1 = minPos1;
  p2 = minPos2;
  if (side == 0)
  { // use p1 as reference - nearest to the left
    d1 = obst1->getDistanceXYsigned2(p1, &idx1, NULL, &w1);
    d2 = obst2->getDistanceXYsigned2(p1, &idx2, &p2, &w2);
  }
  else if (side == 1)
  { // use p2 as reference - nearest to the right
    d1 = obst1->getDistanceXYsigned2(p2, &idx1, &p1, &w1);
    d2 = obst2->getDistanceXYsigned2(p2, &idx2, &p2, &w2);
  }
  // get first midpoint
  pm.x = (p1.x + p2.x) / 2.0;
  pm.y = (p1.y + p2.y) / 2.0;
  if (side < 0)
  { // use midpoint and iterate a solution
    for (i = 0; i < 5; i++)
    { // get distance to both polygons
      d1 = obst1->getDistanceXYsigned2(pm, &idx1, &p1, &w1);
      d2 = obst2->getDistanceXYsigned2(pm, &idx2, &p2, &w2);
      if (fabs(d1 - d2) < 0.01)
        break;
/*      if (w2 and not w1)
      { // p2 is a vertwx and p1 is not
        seg = obst1->getSegment(idx1);
        // get closest point on side in obstacle 1
        // for vertex from obstacle 2
        a = seg.getPositionOnLine(p2);
        p1 = seg.getPositionOnLine(a);
      }
      else if (w1 and not w2)
      { // p1 is a vertex and p2 is not
        seg = obst2->getSegment(idx2);
        // get closest point on side in obstacle 2
        // for vertex from obstacle 1
        a = seg.getPositionOnLine(p1);
        p2 = seg.getPositionOnLine(a);
      }*/
      // try with new midpoint
      pm.x = (p1.x + p2.x) / 2.0;
      pm.y = (p1.y + p2.y) / 2.0;
    }
  }
  result = d1 + d2 > ROBOTWIDTH;
  if (midPose != NULL)
  { // calculated mid-pose
    midPose->x = pm.x;
    midPose->y = pm.y;
    // if closest to side of obstacle, then keep heading from obstacle side
    if (not w1)
    { // obstacle 1 is hit on a side
      seg = obst1->getSegment(idx1);
      midPose->h = seg.getXYHeading();
    }
    else if (not w2)
    { // obstacle 2 is hit on a side
      seg = obst2->getSegment(idx2);
      midPose->h = seg.getXYHeading();
    }
    else
    { // heading must be at right angle to the line between points
      midPose->h = atan2(p1.y - p2.y, p1.x - p2.x);
      midPose->h += (M_PI / 2.0);
    }
    // get heading from start position to new waypoint
    a = atan2(midPose->y - fromPose.y, midPose->x - fromPose.x);
    // find difference from base heading to suggested heading
    a = limitToPi(midPose->h - a);
    if (fabs(a) > (M_PI / 2.0))
      // wrong direction - reverse
      midPose->h = limitToPi(midPose->h + M_PI);
  }
  //
  // return also passage width
  if (width != NULL)
    *width = d1 + d2;
  return result;
}

////////////////////////////////////////////////////////

// double UReactivePath::getFlatTiltValue()
// {
//   double ang;
//   ULaserPi * pi;
//   ULaserScan * scan;
//   double result = 0.0; // tilt angle (0.0 is invalid)
//   double range;
//   //
//   pi = getFirstLaserPi();
//   if (pi != NULL)
//   {
//     ang = getRouteAngleExit();
//     scan = pi->getScan();
//     range = scan->getRange(ang);
//     if (range > SCANNERHEIGHT)
//       result = asin(SCANNERHEIGHT / range);
//   }
//   //
//   return result;
// }

////////////////////////////////////////////////////////

// ULaserPi * UReactivePath::getFirstLaserPi()
// {
//   ULaserPi * result = NULL;
//   int i;
//   //
//   for (i = 0; i < intervalsCnt; i++)
//   {
//     result = (ULaserPi *)intervals[i];
//     if (result->isLaser())
//       break;
//   }
//   if (not result->isLaser())
//     result = NULL;
//   return result;
// }

////////////////////////////////////////////////////////

// bool UReactivePath::isFirstVision()
// {
//   bool result;
//   //
//   result = (intervalsCnt > 0);
//   if (result)
//     result = intervals[0]->isVision();
//   return result;
// }

////////////////////////////////////////////////////////

// ULineSegment UReactivePath::getFirstLaserSeg()
// {
//   ULineSegment result;
//   ULaserPi * pis;
//   //
//   pis = getFirstLaserPi();
//   if (pis != NULL)
//     result = *pis->getSegment();
//   return result;
// }

////////////////////////////////////////////////////////

/*bool UReactivePath::getLastVisionSeg(ULineSegment * segv)
{
  bool result = false;
  UPassInterval * pis = NULL;
  UPassInterval * pisv = NULL;
  int i;
  //
  for (i = 0; i < intervalsCnt; i++)
  {
    pis = intervals[i];
    if (pis->isLaser())
      break;
    pisv = pis;
  }
  result = (pisv != NULL);
  if (result)
    *segv = *pisv->getSegment();
  return result;
}*/

////////////////////////////////////////////////////////

// double UReactivePath::getRoadWidth()
// {
//   double result = 0.0;
//   ULaserPi * pp;
//   //
//   result = (intervalsCnt > 0);
//   if (result)
//   {
//     // get effective road width
//     if (isRoadValid())
//       result = roadWidth;
//     else
//     {
//       pp = getFirstLaserPi();
//       result = pp->getYWidth();
//     }
//   }
//   return result;
// }

////////////////////////////////////////////////////////

void UReactivePath::setRouteStats(UPose odoPose)
{
  double ang;
  ULineSegment seg;
  UPosition pr, p1, p2;
  //
  // get current position
  pr.set(odoPose.x, odoPose.y, 0.0);
  // get exit point
  p2 = man->getP2P(man->getP2PCnt() - 1)->getEndPos();
  // calculate angle offset to end point - from robot
  ang = atan2(p2.y - pr.y, p2.x - pr.x);
  // ... relative to current heading
  ang -= odoPose.h;
  // save angle offset to exit point on route
  routeAngleExit = limitToPi(ang);
  // get direct distance to exit pose
  distExitDirect = hypot(p2.y - pr.y, p2.x - pr.x);
  // calculate deviation from line
  routeSD = man->getDeviationFromDirect();
  // add last distance back to robot
  routeDist = man->getDistance();
  // calculate angle offset for first waypoint from robot
  p2 = man->getP2P(0)->getEndPos();
  ang = atan2(p2.y - pr.y, p2.x - pr.x);
  // ... relative to current heading
  ang -= odoPose.h;
  // save angle offset to first waypoint
  routeAngleFirst = limitToPi(ang);
}

///////////////////////////////////////////////////////////

// double UReactivePath::getEdgeDistanceLeft(UPose fromPose)
// {
//   double result = 100.0;
//   UPosition pos;
//
//   if (isEdgeLeftValid())
//   {
//     pos.set(fromPose.x, fromPose.y, 0.0);
//     result = edgeLeft.getDistanceFromSeg(pos, NULL);
//   }
//   return result;
// }
//
// ///////////////////////////////////////////////////////////
//
// double UReactivePath::getEdgeDistanceRight(UPose fromPose)
// {
//   double result = 100.0;
//   UPosition pos;
//
//   if (isEdgeRightValid())
//   {
//     pos.set(fromPose.x, fromPose.y, 0.0);
//     result = edgeRight.getDistanceFromSeg(pos, NULL);
//   }
//   return result;
// }
//
// ////////////////////////////////////////////////////////
//
// double UReactivePath::getEdgeDistanceTop(UPose fromPose)
// {
//   double result = 100.0;
//   UPosition pos;
//
//   if (isEdgeTopValid())
//   { // current position (as 3D)
//     pos.set(fromPose.x, fromPose.y, 0.0);
//     result = -edgeTop.getXYsignedDistance(pos);
//   }
//   return result;
// }

////////////////////////////////////////////////////////

// bool UReactivePath::isAWall()
// {
//   bool result = false;
//   UPassInterval * pi;
//   int i;
//   const int WPF_WALL_MAX_AGE = 4; // a valid wall is within the last few scans
//                                   // else assumed an erronious wall count
//   int n = mini(WPF_WALL_MAX_AGE, intervalsCnt);
//   //
//   for (i = 0; i < n; i++)
//   {
//     pi = intervals[i];
//     result = (pi->getWallCnt() > 1);
//     if (result)
//       break;
//   }
//   return result;
// }

////////////////////////////////////////////////////////

UPoseV UReactivePath::getTangentMidPose(int minIdx1, int minIdx2,
                                     int side, UPosition minDistPos,
                                     bool newEndPoint, double atDistance)
{
  UPosition p1; // tangent position from start
  UPosition p2; // tangent position from destination
  UPosition p3;
  double a1, a2, a3;
  double d1, d2;
  UPoseV midPose;
  bool isOK2, isOK3;
  //
  // get start position
  p3 = man->getP2P(minIdx1)->getStartPos();
  // find tanget point from robot side
  p1 = p3.getTangentPointXY(minDistPos,
                            atDistance,
                            side == 0, &isOK3, &a1); // side==0 obstacle is to the left
  if (not isOK3)
  { // use start position
    p1 = p3;
    a1 = atan2(p1.y - minDistPos.y, p1.x - minDistPos.x);
  }
  d1 = minDistPos.dist(p3);
  // find tangent point from end point, but only if endpoint is not too close
  // to obstacle.
  if (not newEndPoint)
  { // not too close - get destination position
    p3 = man->getP2P(minIdx2)->getEndPos();
    // around obstacle from destination point
    p2 = p3.getTangentPointXY(minDistPos,
                              atDistance,
                              side == 1, &isOK2, &a2); // side==0 obstacle is to the left
    if (not isOK2)
    { // use end position
      p2 = p3;
      a2 = atan2(p2.y - minDistPos.y, p2.x - minDistPos.x);
    }
    d2 = minDistPos.dist(p3);
    // now, d1 is distance from obstacle to start point
    //      d2 is distance to endpoint
    //      a1 is tangent angle to start point
    //      a2 is tanget angle to end point
    // both tangents should be OK (isOK2 and isOK3), no need to test further
    // ensure a1-a2 interval is spanning new touchpoint a3
    if ((side == 0) and (a2 < a1))
      // side==0 obstacle is to the left (bypassing obstacle counter clockwise)
      a2 += 2.0 * M_PI;
    else if ((side == 1) and (a1 < a2))
      // side==1 obstacle is to the right (bypassing obstacle clockwise)
      a1 += 2.0 * M_PI;
    // find the right place to touch
    // based on distance from obstacle
    if (fabs(a1 - a2) > M_PI)
    { // too big an arc, the tangent lines are crossing (before touching circle)
      // use direction to offending point instead
/*      a3 = atan2(offendedPoint.y - minDistPos.y,
                 offendedPoint.x - minDistPos.x);*/
      // NO, bad if new passage it at the other side, use in stead
      // the closest touchpoint
      if (d1 < d2)
        a3 = a1;
      else
        a3 = a2;
    }
    else if (a2 > a1)
      // use weighted midpoint
      a3 = a1 + (a2 - a1) * d1/(d1 + d2);
    else
      a3 = a2 + (a1 - a2) * d1/(d1 + d2);
  }
  else
    // if target is too close to obstacle, then stop at this angle
    a3 = a1;
    // now finds touch position
  midPose.x = minDistPos.x + atDistance * cos(a3);
  midPose.y = minDistPos.y + atDistance * sin(a3);
  if (side == 0) // obstacle to the left
    midPose.h = a3 + 0.5 * M_PI;
  else // obstacle to the right
    midPose.h = a3 - 0.5 * M_PI;
  //
  return midPose;
}

////////////////////////////////////////////////////////

bool UReactivePath::getWorstVertex(UObstacle * obs, int minVertex, int end,
                           int side, double * minDist,
                           UPosition * minDistPos, UManPPSeq * mpp)
{
  int k, i;
  double d1;
  bool ccv;
  UPosition p1;
  //
  k = minVertex;
  if (end == 1)
    k -= 1;
  if (k < 0)
    k = obs->getPointsCnt();
  p1 = obs->getPoint(k);
  d1 = mpp->getDistanceXYSigned(p1, NULL, side == 1, NULL, NULL);
  ccv = (d1 > mind(0.0, minDist[side])); // increase k to get worse value
  if (ccv)
  {
    k = minVertex;
    if (end == 2)
      k++;
    k = (k + 1) % obs->getPointsCnt();
    p1 = obs->getPoint(k);
    d1 = mpp->getDistanceXYSigned(p1, NULL, side == 1, NULL, NULL);
  }
            // continue until most distant point is found
  for (i = 0; i < obs->getPointsCnt() - 1; i++)
  {
    if (d1 < *minDist)
    { // save new candidate
      *minDist = d1;
      *minDistPos = p1;
    }
    else
      // not better, so stop
      break;
    // advance to next
    if (ccv)
      k = (k +1) % obs->getPointsCnt();
    else
      if (k == 0)
        k = obs->getPointsCnt() - 1;
    else
      k--;
    p1 = obs->getPoint(k);
    d1 = mpp->getDistanceXYSigned(p1, NULL, side == 1, NULL, NULL);
  }
  return true;
}

////////////////////////////////////////////////////////

bool UReactivePath::getWorstVertex2(UObstacle * obs,
                                int side, double * minDist,
                                UPosition * minDistPos, UManPPSeq * mpp)
{
  int i;
  double d1;
  UPosition p1;
  //
  // continue until most distant point is found
  for (i = 0; i < obs->getPointsCnt(); i++)
  {
    p1 = obs->getPoint(i);
    d1 = mpp->getDistanceXYSigned(p1, NULL, side == 1, NULL, NULL);
    if (d1 < *minDist)
    { // save new candidate
      *minDist = d1;
      *minDistPos = p1;
    }
  }
  return true;
}

////////////////////////////////////////////////////////

UObstacle * UReactivePath::getNearObstacle(UReacObstGrps * obsts,
                                        int minVertexCnt,
                            double dist, UPosition pos, int * vertex,
                           UObstInfo * exclude, UObstacle * notThis)
{
  int i, j, k, mK = -1;
  UObstacle * obs, * mObs = NULL;
  UObstacleGroup * grp;
  UPosition p1;
  double d, mD = sqr(dist);

  for (i = 0; i < obsts->getGroupsCnt(); i++)
  { // find nearest obstacle to the left and right
    grp = obsts->getGroup(i);
    for (j = 0; j < grp->getObstsCnt(); j++)
    { // get next obstacle
      obs = grp->getObstacle(j);
      if ((obs != notThis) and
           (exclude->getObstInfo(obs) < 0) and
           (obs->getPointsCnt() > minVertexCnt))
      { // obstacle is not mentioned in exclude list
        // the 'obs' should be in the list too.
        for (k = 0; k < obs->getPointsCnt(); k++)
        {
          p1 = obs->getPoint(k);
          d = sqr(p1.x - pos.x) + sqr(p1.y - pos.y);
          if (d < mD)
          {
            mD = d;
            mK = k;
            mObs = obs;
          }
        }
      }
    }
  }
  if ((mObs != NULL) and (vertex != NULL))
  {
    *vertex = mK;
  }
  return mObs;
}

////////////////////////////////////////////////////////

// double UReactivePath::getAverageVarMin(double * vsd)
// {
//   double result = 0.0;
//   int i, n;
//   UPassInterval * pp;
//   double v;
//   double sds, esdv = 0.0;
//   bool first = true;
//   double ref = 0.0;
//   //
//   n = 0;
//   sds = 0.0;
//   for (i = 0; i< intervalsCnt; i++)
//   {
//     pp = intervals[i];
//     v = pp->getVarMin();
//     if (v > 1e-7)
//     { // valid minimum variance
//       n++;
//       result += v;
//       if (vsd != NULL)
//       {
//         if (first)
//         { // use first value as reference
//           ref = sqrt(v);
//           first = false;
//         }
//         else
//           sds += sqr(sqrt(v) - ref);
//       }
//     }
//   }
//   if (n > 0)
//   { // get average variance
//     result /= n;
//     if (vsd != NULL)
//     {
//       if (n > 1)
//         // get variance on deviation
//         esdv = sds / (n - 1);
//       // and return standard deviation
//       *vsd = sqrt(esdv);
//     }
//   }
//   return result;
// }

///////////////////////////////////////////////////////

// double UReactivePath::getRoadQual()
// {
//   double result;
//   const double ww = 1.0; // weight of road width quality
//   const double wv = 1.0; // weight of min variance deviation
//   double vsd;
//   double vmin;
//   double vsdq;
//   //
//   vmin = getAverageVarMin(&vsd);
//   // get deviasion from average surface variance as quality
//   vsdq = 1.0/(1.0 + vsd * 20.0);
//   // get average quality
//   result = (roadWidthQual * ww + vsdq * wv) / (ww + wv);
//   return result;
// }

///////////////////////////////////////////////////////

// double UReactivePath::getRoadQual2()
// {
//   double result;
//   const double ww = 1.0; // weight of road width quality
//   const double wrm = 1.0; // weight of roughness value
//   const double wrv = 1.0; // weight of roughness deviation
//   double vsdq; // roughness SD quality
//   double rmq; // roughness mean quality
//   //
//   // get deviasion from average surface variance as quality
//   vsdq = 1.0/(1.0 + sqrt(roughMeanVariance) * 20.0);
//   // get roughness quality - rough is about 0.06 -> q is less than 0.5
//   rmq = 1.0 / (1.0 + roughMean * 20.0);
//   // get average quality
//   result = (roadWidthQual * ww + vsdq * wrv + rmq * wrm) / (ww + wrv + wrm);
//   return result;
// }

///////////////////////////////////////////////////////

// UPassInterval * UReactivePath::getNewestLaserInterval()
// {
//   UPassInterval * result = NULL;
//   int i;
//   bool found = false;
//   //
//   for (i = 0; i < intervalsCnt; i++)
//   {
//     result = intervals[i];
//     if (result->isLaser())
//     {
//       found = true;
//       break;
//     }
//   }
//   if (not found)
//     result = NULL;
//   return result;
// }

///////////////////////////////////////////////////////

// int UReactivePath::getIntervalCntLaser()
// {
//   UPassInterval * pi;
//   int i;
//   int result = 0;
//   //
//   for (i = 0; i < intervalsCnt; i++)
//   {
//     pi = intervals[i];
//     if (pi->isLaser())
//     {
//       result = intervalsCnt - i;
//       break;
//     }
//   }
//   return result;
// }


////////////////////////////////////////////////////////

// bool UReactivePath::getVisionRoadWidth(double * width,
//                                       double * widthVar,
//                                       double * tiltFac,
//                                    double * vwNear)
// {
//   UVisPi * vp;
//   int i, n = 0;
//   double ws = 0.0; // width sum/average
//   double ws2 = 0.0; // width sum squared
//   double wv; // width variance
//   double w1 = 0.0, w2 = 0.0;
//   bool first = true;
//   //
//   for (i = 0; i < intervalsCnt; i++)
//   {
//     vp = (UVisPi *)intervals[i];
//     if (not vp->isVision())
//       break;
//     if (vp->isObstLeft() and vp->isObstRight())
//     { // both sides should be valid road edges
//       ws += vp->getSegment()->length;
//       ws2 += sqr(vp->getSegment()->length);
//       n++;
//       if (first)
//       { // save first (far away)
//         w1 = vp->getSegment()->length;
//         first = false;
//       }
//       else
//         // save closest full width
//         w2 = vp->getSegment()->length;
//     }
//   }
//   if (n > 1)
//   {
//     ws /= double(n);
//     ws2 /= double(n);
//     wv = ws2 - sqr(ws);
//     if (width != NULL)
//       *width = ws;
//     if (widthVar != NULL)
//       *widthVar = wv;
//     if ((tiltFac != NULL) and (w2 > 0.0))
//     { // calculate a value that can be used
//       // to adjust the camera tilt angle
//       // > 0.0 tilt angle too small
//       // < 0.0 tilt angle too large
//       *tiltFac = (w1 - w2)/ws;
//     }
//     if ((vwNear != NULL) and (w2 > 0.0))
//       *vwNear = w2;
//   }
//   return n > 1;
// }

////////////////////////////////////////////

// int UReactivePath::getRoadTypeEstimate()
// { // estimate road type based on roughness
//   int result = 0;
//   //
//   if (roughMean < (ROAD_ROUGHNESS_ASPHALT + ROAD_ROUGHNESS_GRAVEL)/2.0)
//     result = 0;
//   else if (roughMean < (ROAD_ROUGHNESS_GRASS + ROAD_ROUGHNESS_GRAVEL)/2.0)
//     result = 1;
//   else
//     result = 2;
//   //
//   return result;
// }

///////////////////////////////////////////////////////

void UReactivePath::print(const char * prestring, char * buff, const int buffCnt)
{
  man->print(prestring, buff, buffCnt);
}


///////////////////////////////////////////////
///////////////////////////////////////////////
///////////////////////////////////////////////

UAvoidPathPool::UAvoidPathPool()
{
  int i;
  pathsCnt = 0;
  avoidPathsCnt = 0;
  pathBest = 0;
  for (i = 0; i < MAX_PATHS_IN_POOL; i++)
    avoidPaths[i].poolIdx = i;
}

///////////////////////////////////////////////

UAvoidPathPool::~UAvoidPathPool()
{
}

///////////////////////////////////////////////

UReactivePath * UAvoidPathPool::findPathToHere(UPoseV exitPose,
                                               UReacObstGrps * obsts,
                                               UReacRoadLines * roads,
                                               bool ignoreObstacles, bool directTestOnly,
                                              bool driveon, double turnRad)
{
  UReactivePath * result = NULL;
  UReactivePath * rp;
  int i;
//  const int MSL = 3000;
//  char s[MSL];
  double tMin = 1e15;
  //
  //
  lock();
  pathsCnt = 0;
  rp = getNewPath();
  if (rp != NULL)
  {
    rp->clear();
    // debug
    //printf("UAvoidPathPool::findPathToHere: avoiding %d obst-grps\n", obsts->getGroupsCnt());
    // debug end
    rp->setExitPose(exitPose);
    // recursively exploid possible paths
    rp->findObstAvoidRoute(obsts, roads, this, ignoreObstacles, directTestOnly, driveon, turnRad);
    //if (pathsCnt > 0)
    {
/*      for (i = 0; i < pathsCnt; i++)
      { // print found paths
        rp = &paths[i];
        rp->print( "path", s, MSL);
        printf("%s", s);
      }*/
      //printf("UAvoidPathPool::findPathToHere: Found %d possible paths\n", pathsCnt);
    }
    // find best path (based on time)
    for (i = 0; i < pathsCnt; i++)
    { // print found paths
      rp = &paths[i];
      if (rp->isValid() and not rp->isACrash())
        if (rp->getManTime() < tMin)
        {
          tMin = rp->getManTime();
          result = rp;
        }
    }
  }
  else
    fprintf(stderr, "UAvoidPathPool::findPathToHere: no free space for manoevres\n");
  // lock the result to allow use and monitoring at a later time
  if (result != NULL)
  {
    result->setPathUsed(true);
    result->getManSeq()->lock();
  }
  // unlock the manoeuvre pool
  unlock();
  //
  return result;
}

////////////////////////////////////////////////

UAvoidPath2 * UAvoidPathPool::findPathToHere2(UPoseV exitPose,
                                               UReacObstGrps * obsts,
                                               UReacRoadLines * roads,
                                               bool ignoreObstacles, bool debugDump)
{
  UAvoidPath2 * result = NULL;
  UAvoidPath2 * ap;
  UPoseV posev;
  //
  lock();
  avoidPathsCnt = 0;
  ap = getNewAvoidPath();
  if (ap != NULL)
  {
    ap->clear();
    par.exitPose = exitPose;
    par.obsts = obsts;
    // debug
    printf("UAvoidPathPool::findPathToHere2: got %d obst groups:", obsts->getGroupsCnt());
    UObstacleGroup * og;
    for (int i = 0; i < obsts->getGroupsCnt(); i++)
    {
      og = obsts->getGroup(i);
      printf(" %lu", og->getSerial());
    }
    printf("\n");
    // debug end
    par.roads = roads;
    ap->setPar(&par);
    // find route to destination given
    // - destination (exitPose)
    // - obstacles (obsts)
    // - robot parameters (par)
    ap->findObstAvoidRoute(debugDump);
    // any luck?
    if (ap->isValid() and ap->isACrash())
      // use the result - else report no result
      ap->setValid(false);
    result = ap;
  }
  else
  { // all manoeuvres are locked - not a good sign
    fprintf(stderr, "UAvoidPathPool::findPathToHere: no free space for manoevres - waiting for release (rather bad)\n");
  }
  // lock the result to allow use and monitoring at a later time
  if (result->isValid())
  {
    result->setPathUsed(true);
    result->getManSeq()->lock();
    //result->getManSeq()->fprint(stdout, "path result");
  }
  // unlock the manoeuvre pool
  unlock();
  //
  return result;
}

////////////////////////////////////////////////

UPoseV UAvoidPathPool::getStartPoseVel()
{
  UPoseV result;
  //
  result.set(par.startPose.getPose(), par.startPose.getVel());
  //
  return result;
}

///////////////////////////////////////////////////

// void UAvoidPathPool::setParameters(
//     double maxAcc,
//     double maxTurnAcc,
//     double minTurnRad,
//     bool crashTest,
//     int nestedLevels,
//     int avoidLoops,
//     int spawnCnt)
// {
//   par.maxAcceleration = maxAcc;
//   par.maxTurnAcceleration = maxTurnAcc;
//   par.minTurnRadius = minTurnRad;
//   par.doCrashTest = crashTest;
//   par.maxNestedLevels = nestedLevels;
//   par.maxAvoidLoops = avoidLoops;
//   par.maxSpawnCnt = spawnCnt;
// }

///////////////////////////////////////////////////////

UReactivePath * UAvoidPathPool::getNewPath()
{
  int i;
  UReactivePath * rp;
  //
  rp = paths;
  for (i = 0; i < MAX_PATHS_IN_POOL; i++)
  {
    if (not rp->getManSeq()->tryLock())
    { // this is used by some other function - leave as is, but set invalud
      rp->setValid(false);
    }
    else
    {
      rp->getManSeq()->unlock();
      if ((not rp->isValid() and (i < pathsCnt)) or (i >= pathsCnt))
        // manoeuvre sequence is free - use
        break;
    }
    rp++;
  }
  if (i >= MAX_PATHS_IN_POOL)
    rp = NULL;
  else if (i >= pathsCnt)
  {
    // debug
    // printf("UAvoidPathPool::getNewPath: making new path in slot %d\n", i);
    // debug end
    pathsCnt = i + 1;
    rp->setPathUsed(false);
  }
  return rp;
}

/////////////////////////////////////////////////////

UAvoidPath2 * UAvoidPathPool::getNewAvoidPath()
{
  int i;
  UAvoidPath2 * ap;
  //
  ap = avoidPaths;
  for (i = 0; i < MAX_PATHS_IN_POOL; i++)
  {
    if (not ap->getManSeq()->tryLock())
    { // this is used by some other function - leave as is, but set invalid
      ap->setValid(false);
    }
    else
    { // release again
      ap->getManSeq()->unlock();
      if ((not ap->isValid() and (i < avoidPathsCnt)) or (i >= avoidPathsCnt))
        // manoeuvre sequence is free - use
        break;
    }
    ap++;
  }
  if (i >= MAX_PATHS_IN_POOL)
    ap = NULL;
  else if (i >= avoidPathsCnt)
  {
    avoidPathsCnt = i + 1;
    ap->setPathUsed(false);
  }
  return ap;
}

/////////////////////////////////////////////////////

int UAvoidPathPool::getValidPathsCnt()
{
  UReactivePath * rp;
  int i, n;
  //
  rp = paths;
  n = 0;
  for (i = 0; i < pathsCnt; i++)
  {
    if (rp->isValid())
        n++;
    rp++;
  }
  return n;
}

//////////////////////////////////////////////////////

int UAvoidPathPool::getValidAvoidPathsCnt()
{
  UAvoidPath2 * rp;
  int i, n;
  //
  rp = avoidPaths;
  n = 0;
  for (i = 0; i < avoidPathsCnt; i++)
  {
    if (rp->isValid())
      n++;
    rp++;
  }
  return n;
}
