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
#include "uobstaclepool.h"

///////////////////////////////////////////////////////////

UObstacleGroupLaser::UObstacleGroupLaser()
  : UObstacleGroup()
{
  obstacleMergeMargin = 0.25; // default is outdoor merge distance
  obstacleCogMergeLimit = 1.0;
  mergeObstacles = true;
}

///////////////////////////////////////////////////////////

UObstacleGroupLaser::~UObstacleGroupLaser()
{
}


//////////////////////////////////////////////////////////

// bool UObstacleGroupLaser::addObst(ULaserScan * scan,
//                                   int idx1, int idx2, bool outdoorObsts)
// {
//   UObstacle * ob, * ob2;
//   int i;
//   UPosition p1;
//   UPolygon400 p400;
//   UPolygon40 p40;
//   bool isOK;
//   UPosition * pp;
//   const double MARGIN_OUTDOOR = ROBOTWIDTH * 0.4; // margin for overlap
//   const double MARGIN_INDOOR = 0.03; // margin for overlap indoor
//   double usedMargin = MARGIN_INDOOR;
//   int merge[MAX_OBSTACLES];
//   int mergeCnt = 0;
//   int h;
//   UPoseTime pt;
//   UPoseV pv = scan->getPoseV();
//   UPosition minPos;
//   const double NEAR_VERTEX_DIST = 0.03;
//   //
//   isOK = (idx2 >= idx1);
//   if (isOK)
//   { // creat a polygon from these points
//     for (i = idx1; i <= idx2; i++)
//     {
//       p1 = scan->getPos(i);
//       p400.add(p1);
//       if ((p1.x < minPos.x) or (i == idx1))
//         minPos = p1;
//     }
//     // make conves hull polygon
//     isOK = p400.extractConvexTo(&p40);
//     // reduce vertex count if near
//     p40.removeNearVertex(NEAR_VERTEX_DIST);
//     // convert to map (odo) coordinates
//     pp = p40.getPoints();
//     for (i = 0; i < p40.getPointsCnt(); i++)
//     { // move position to odometry coordinates
//       *pp = scan->odoPose.getPoseToMap(*pp);
//       pp++;
//     }
//   }
//   // test if seen before
//   if (isOK)
//   { // add the point with the smallest local x value
//     // converted to map position using
//     // the previous pose
//     if (outdoorObsts)
//     { // if outdoor obstacles, then assume most are low grass
//       usedMargin = MARGIN_OUTDOOR;
//       p1 = getPosePrev().getPoseToMap(minPos);
//       p40.add(p1);
//     }
//     for (i = 0; i < obstsCnt; i++)
//     { // test if there are overlap with existing obstacles
//       ob = obsts[i];
//       if (ob->isOverlappingXYconvex(&p40, usedMargin))
//         merge[mergeCnt++] = i;
//     }
//     // debug
//     if (mergeCnt >= 1)
//     {
//       if (hypot(p1.x - 8.0, p1.y + 1.0) < 0.3)
//         printf("+++ merge %d with pose at (8.0, -1.0)\n", mergeCnt);
//       ob = obsts[merge[0]];
//       p1 = ob->getPoint(0);
//       if (hypot(p1.x - 8.0, p1.y + 1.0) < 0.3)
//         printf("+++ merge %d from pose at (8.0, -1.0)\n", mergeCnt);
//     }
//     // debug end
//     // remove the extra point
//     if (outdoorObsts)
//       p40.pop();
//     // no overlap - add as new
//     if (mergeCnt == 0)
//     { // test for space
//       isOK = obstsCnt < MAX_OBSTACLES;
//       if (isOK)
//       {
//         ob = obsts[obstsCnt];
//         if (ob == NULL)
//         {
//           ob = new UObstacle();
//           obsts[obstsCnt] = ob;
//         }
//         obstsCnt++;
//       }
//     }
//     else
//       // get destination obstacle
//       ob = obsts[merge[0]];
//   }
//   if (isOK)
//   {
//     ob->lock();
//     if (mergeCnt == 0)
//     { // use this obstacle as new obst
//       ob->clear();
//       ob->initPoseFirst(scan->getPoseTime());
//       ob->add(&p40);
//     }
//     else
//     { // merge all with this obstacle
//       p400.clear();
//       // add new obstacle
//       p400.add(&p40);
//       pt = ob->getPoseFirst();
//       h = 1;
//       for (i = 0; i < mergeCnt; i++)
//       { // merge all the the others too
//         ob2 = obsts[merge[i]];
//         isOK = p400.add(ob2);
//         if (not isOK)
//           printf("Polygon overflow - unlikely, but not fatal?\n");
//         h = maxi(h, ob2->getHits());
//         if ((pt.t - ob2->getPoseFirst().t) > 0.0)
//           pt = ob2->getPoseFirst();
//       }
//       // extract to new destination obstacle
//       p400.extractConvexTo(ob);
//       // reduce vertex count
//       ob->removeNearVertex(NEAR_VERTEX_DIST);
//       // merge attributes
//       ob->setPoseFirst(pt); // set oldest start pose-time
//       ob->setHits(h); // and highest hit-count
//       ob->setPoseLast(scan->getPoseTime());
//     }
//     ob->unlock();
//     if (poseLast.t != scan->time)
//     {
//       posePrev = poseLast;
//       poseLast = scan->getPoseTime();
//     }
//     if (mergeCnt > 1)
//     { // remove all other merged obstacles
//       for (i = 1; i < mergeCnt; i++)
//         removeObst(merge[i]);
//     }
//   }
//   return isOK;
// }

//////////////////////////////////////////////////

bool UObstacleGroupLaser::addObst(ULaserScan * scan, UPoseTime odoPose,
                                  int idx1, int idx2,
                                  bool outdoorObsts, bool horizontalScan)
{
  int i;
  UPosition p1;
  UPolygon400 p400;
  UPolygon40 p40;
  bool isOK;
  UPosition * pp;
  UPosition minPos;
  const double NEAR_VERTEX_DIST = 0.03;
  UMatrix4 mStoR;
  UPosRot pr;
  //
  isOK = (idx2 >= idx1);
  if (isOK)
  { // create a polygon from these points
    for (i = idx1; i <= idx2; i++)
    {
      p1 = scan->getPos(i);
      p400.add(p1);
      if ((p1.x < minPos.x) or (i == idx1))
        minPos = p1;
    }
    // make conves hull polygon
    isOK = p400.extractConvexTo(&p40);
    // reduce vertex count if near
    p40.removeNearVertex(NEAR_VERTEX_DIST);
    //
    // convert to odometry coordinates
    pr = *scan->getSensorPose();
    // tilt and height is used already, so zero these
    pr.setPhi(0.0);
    pr.setZ(0.0);
    mStoR = pr.getRtoMMatrix();
    pp = p40.getPoints();
    for (i = 0; i < p40.getPointsCnt(); i++)
    { // move position to odometry coordinates
      // first move to robot coordinates
      p1 = mStoR * *pp;
      // then to odometry map coordinates
      *pp = odoPose.getPoseToMap(p1);
      // advance to next position in polygon
      pp++;
    }
  }
  // add correlation point if outdoor obstacle mode
  if (isOK)
  { // add the point with the smallest local x value
    // converted to map position using
    // the previous pose
    if (outdoorObsts and not horizontalScan)
    { // if outdoor obstacles, then assume most are low grass
      // seen at same relative position from time to time
      p1 = getPosePrev().getPoseToMap(mStoR * minPos);
      p40.add(p1);
    }
  }
  if (isOK)
  {
    //addObstPoly(UPolygon * newpoly, UPoseTime poset, bool outdoorObsts)
    isOK = addObstPoly(&p40, odoPose,
                        outdoorObsts, horizontalScan);
    //isOK = addObstPoly(&p40, scan->getPoseTime(), outdoorObsts, false);
  }
  return isOK;
}

//////////////////////////////////////////////////

// bool UObstacleGroupLaser::addObstPoints(UPoseTime poset, UPosition * points, int pointsCnt, bool firmObst)
// {
//   int i;
//   UPosition * p1;
//   UPolygon400 p400;
//   UPolygon40 p40;
//   bool isOK;
//   UPosition minPos;
//   const double NEAR_VERTEX_DIST = 0.03;
//   //
//   isOK = (pointsCnt > 0) and (pointsCnt < p400.getPointsMax());
//   if (isOK)
//   { // creat a polygon from these points
//     p1 = points;
//     for (i = 0; i < pointsCnt; i++)
//       p400.add(p1++);
//     // make conves hull polygon
//     isOK = p400.extractConvexTo(&p40);
//     // reduce vertex count if near
//     p40.removeNearVertex(NEAR_VERTEX_DIST);
//   }
//   if (isOK)
//     // NB! merge option should be TRUE (last parameter)!!! (false is for debug only)
//     isOK = addObstPoly(&p40, poset, false, true, firmObst);
//     //isOK = addObstPoly(&p40, poset, false, false, firmObst);
//   //
//   return isOK;
// }

//////////////////////////////////////////////////

bool UObstacleGroupLaser::addObstPoly(UPolygon * newpoly, UPoseTime poset,
                                      bool outdoorObsts, bool horizontalScan,
                                      int polyHits /* = 1 (seen in one scan) */)
{
  UObstacle * ob, * ob2;
  int i;
  UPosition p1;
  UPolygon400 p400;
  bool isOK;
  int merge[MAX_OBSTACLES];
  int mergeCnt = 0;
  int h;
  UPoseTime pt;
  const double NEAR_VERTEX_DIST = 0.03;
  double d, dMin;
  //
  isOK = (newpoly != NULL);
  if (isOK)
  {
    if (outdoorObsts and not horizontalScan)
    { // a connection point is added
      isOK = newpoly->getPointsCnt() > 1;
    }
    else
      isOK = newpoly->getPointsCnt() > 0;
  }
  // test if seen before
  if (isOK)
  { // add the point with the smallest local x value
    // converted to map position using
    dMin = 1e8;
    if (mergeObstacles or mergeCogEmbedded or mergeSingles)
    {
      for (i = 0; i < obstsCnt; i++)
      { // test if there are overlap with existing obstacles
        ob = obsts[i];
        if (mergeSingles and (ob->getPointsCnt() == 1 or newpoly->getPointsCnt() == 1))
        { // at least one of the obstacles is a single point obstacle
          if (ob->getPointsCnt() == 1)
          {
            p1 = ob->getPoint(0);
            d = newpoly->getDistance(p1.x, p1.y);
          }
          else
          {
            p1 = newpoly->getPoint(0);
            d = ob->getDistance(p1.x, p1.y);
          }
          if (d < dMin and (d < obstacleSingleMargin or (mergeObstacles and d < obstacleMergeMargin)))
          { 
            if (mergeObstacles)
              merge[mergeCnt++] = i;
            else
            { // singles are not allowed to merge with more than one obstacle, when
              // obstacles in general are not merged
              mergeCnt = 1;
              merge[0] = i;
              dMin = d;
            }
          }
        }
        else if (mergeObstacles)
        {
          if (ob->isOverlappingXYconvex(newpoly, obstacleMergeMargin))
            merge[mergeCnt++] = i;
        }
        else if (mergeCogEmbedded)
        {
          if (ob->mergeableOnCogLimits(newpoly, obstacleCogMergeLimit))
          {
            merge[mergeCnt++] = i;
/*            printf("Merge current obst %d (cnt=%d) at (%.2fx,%.2fy) with new (cnt=%d) at (%.2fx,%.2fy)\n", i,
                   ob->getPointsCnt(), ob->getCogXY().x, ob->getCogXY().y,
                   newpoly->getPointsCnt(), newpoly->getCogXY().x, newpoly->getCogXY().y);*/
          }
        }
        // debug
/*        if (mergeCnt > 0 and merge[mergeCnt-1] == i)
          printf("  Obstacle pool merge %d of (new cog %.2fx,%.2fy cnt=%d %.2fm2) and (%d cog %.2fx,%.2fy cnt=%d %.2fm2)\n",
                 mergeCnt, newpoly->getCogXY().x, newpoly->getCogXY().y, newpoly->getPointsCnt(), newpoly->getXYarea(),
                 i, ob->getCogXY().x, ob->getCogXY().y, ob->getPointsCnt(), ob->getXYarea());*/
        // debug end
      }
    }
    //
    if (outdoorObsts and not horizontalScan)
      // remove the extra point
      newpoly->pop();
    //
    if (mergeCnt == 0)
    { // no overlap - add as new
      ob = getNewObst();
      isOK = (ob != NULL);
      if (not isOK)
        printf("UObstacleGroupLaser::addObstPoly: no more space in obstacle group %lu - used %d of %d slots\n", serial, obstsCnt, MAX_OBSTACLES);
    }
    else
      // get destination obstacle
      ob = obsts[merge[0]];
  }
  if (isOK)
  {
    ob->lock();
    if (mergeCnt == 0)
    { // use this obstacle as new obst
      ob->clear();
      ob->initPoseFirst(poset);
      if (not mergeObstacles)
      { // mark as seen twice to let it live
        ob->setHits(2);
        ob->setValid(true);
      }
      if (polyHits > 1)
      { // override the default one-scan detections are invalid
        ob->setHits(polyHits);
        ob->setValid(true);
      }
      ob->add(newpoly);
    }
    else
    { // merge all with this obstacle
      p400.clear();
      // add new obstacle
      p400.add(newpoly);
      pt = ob->getPoseFirst();
      h = 1;
      for (i = 0; i < mergeCnt; i++)
      { // merge all the the others too
        ob2 = obsts[merge[i]];
        //
/*        printf("Merge now     obst %d (cnt=%d) at (%.2fx,%.2fy) with new (cnt=%d) at (%.2fx,%.2fy)\n", merge[i],
                   ob2->getPointsCnt(), ob2->getCogXY().x, ob2->getCogXY().y,
                   p400.getPointsCnt(), p400.getCogXY().x, p400.getCogXY().y);*/
        //
        isOK = p400.add(ob2);
        if (not isOK)
          printf("Polygon overflow - unlikely, but not fatal?\n");
        h = maxi(h, ob2->getHits());
        if (polyHits > h)
          h = polyHits;
        if ((pt.t - ob2->getPoseFirst().t) > 0.0)
          pt = ob2->getPoseFirst();
      }
      // extract to new destination obstacle
      p400.extractConvexTo(ob);
      // reduce vertex count
      ob->removeNearVertex(NEAR_VERTEX_DIST);
      // merge attributes
      ob->setPoseFirst(pt); // set oldest start pose-time
      ob->setHits(h); // and highest hit-count
      if (h > 1 and not ob->isValid())
        ob->setValid(true);
      ob->setPoseLast(poset);
      // debug
/*      printf("  Obstacle pool merge (of %d obsts) - added (cog %.2fx,%.2fy %d pnts %.2fm2) result (cog %.2fx,%.2fy), area %.2fm2\n",
                  mergeCnt, newpoly->getCogXY().x, newpoly->getCogXY().y, newpoly->getPointsCnt(), newpoly->getXYarea(),
                  ob->getCogXY().x, ob->getCogXY().y, ob->getXYarea());
      if (ob->getXYarea() > 3.0)
        printf("  Obstacle pool merge of large obst!\n");*/
      // DEBUG END
    }
    ob->unlock();
    if (poseLast.t != poset.t)
    {
      posePrev = poseLast;
      poseLast = poset;
    }
    if (mergeCnt > 1)
    { // remove all other merged obstacles
      for (i = 1; i < mergeCnt; i++)
        removeObst(merge[i]);
    }
  }
  return isOK;
}

///////////////////////////////////////////////////////////

void UObstacleGroupLaser::setMergeDistance(double mergeDist, bool mergeNearObstacles,
                                           double cogLimit, bool mergeCogEmbeddedObstacles,
                                           double singleLimit, bool mergeSingleObstacles)
{
  obstacleMergeMargin = mergeDist;
  mergeObstacles = mergeNearObstacles;
  obstacleCogMergeLimit = cogLimit;
  mergeCogEmbedded = mergeCogEmbeddedObstacles;
  mergeSingles = mergeSingleObstacles;
  obstacleSingleMargin = singleLimit;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

UObstaclePool::UObstaclePool()
{
  int i;
  for (i = 0; i < MAX_OBSTACLE_GROUPS; i++)
    groups[i] = NULL;
  groupsCnt = 0;
  logo = NULL;
  nextSerial = 1;
  fixeds = NULL;
  newest = 0;
}

///////////////////////////////////////////////////

UObstaclePool::~UObstaclePool()
{
  int i;
  for (i = 0; i < groupsCnt; i++)
  { // delete from heap
    if (groups[i] != NULL);
    {
      delete groups[i];
      groups[i] = NULL;
    }
  }
  groupsCnt = 0;
  if (logo != NULL)
  {
    if (groupsCnt > 0)
      groups[newest]->logAll(1, logo);
    fclose(logo);
  }
  logo = NULL;
}

///////////////////////////////////////////////////

void UObstaclePool::clear()
{
  int i;
  //
  for (i = 0; i < groupsCnt; i++)
    groups[i]->removeAllObsts();
  if (groupsCnt > 0)
    obstDataUpdated(getGroup(0)->getPoseLast().t);
}

///////////////////////////////////////////////////

void UObstaclePool::clearGrp(int idx)
{ // idx is index from newest
  int i;
  //
  i = newest - idx;
  if (i < 0)
    i += groupsCnt;
  if (i >= 0 and i < groupsCnt)
  {
    groups[i]->removeAllObsts();
    obstDataUpdated(getGroup(0)->getPoseLast().t);
  }
}

///////////////////////////////////////////////////

UObstacleGroupLaser * UObstaclePool::advanceNewGroup()
{ // advance and clear
  UObstacleGroupLaser * og;
  //
  // debug
  // og = groups[newest];
  // if (og != NULL)
  //   printf("UObstaclePool::advanceNewGroup finished group %d with %d obsts\n", newest, og->getObstsCnt());*/
  // debug end
  if (groupsCnt < MAX_OBSTACLE_GROUPS)
  { // space for new groups
    if (groups[newest] != NULL)
      newest++;
    og = groups[newest];
    if (og == NULL)
    {
      og = new UObstacleGroupLaser();
      groups[newest] = og;
    }
    groupsCnt = newest + 1;
  }
  else
  {
    newest++;
    if (newest >= MAX_OBSTACLE_GROUPS)
      newest = 0;
    og = groups[newest];
    og->clear();
  }
  og->setSerial(nextSerial++);
  return og;
}

///////////////////////////////////////////////////////////

/*bool UObstaclePool::addObstPoints(UPoseTime poset, UPosition * points, int pointsCnt, bool firmObst)
{
  UObstacleGroupLaser * og;
  //
  og = getObstGrp(poset);
  // add new obstacle
  return og->addObstPoints(poset, points, pointsCnt, firmObst);
}*/

///////////////////////////////////////////////////////////

bool UObstaclePool::addObst(ULaserScan * scan, UPoseTime odoPose,
                            int idx1, int idx2,
                            bool outdoorObsts, bool horizontalScan, bool ignoreIfFixed)
{
  UPolygon40 poly40;
  bool isOK;
  // convert points to convex polygon in odometry coordinates
  isOK = pointsToPolygon(scan, odoPose, idx1, idx2, &poly40, outdoorObsts, horizontalScan);
  if (isOK)
    isOK = addObst(&poly40, odoPose, outdoorObsts, horizontalScan, ignoreIfFixed);
  return isOK;
}

/////////////////////////////////////////////////////

bool UObstaclePool::addObst(UPolygon * poly, UPoseTime odoPose,
                            bool outdoorObsts, bool horizontalScan, bool ignoreIfFixed)
{
  UObstacleGroupLaser * og;
  int i;
  bool isOK = true;
  UObstacle * fob; // fixed obstacle
  //
  // correlation with known fixed obstacles
  if (ignoreIfFixed and fixeds != NULL)
  { // test for correlation with fixed obstacles
    for (i = 0; i < fixeds->getObstsCnt(); i++)
    { // test if there are overlap with existing obstacles
      fob = fixeds->getObstacle(i);
      if (fob->isOverlappingXYconvex(poly, fob->getMargin()))
      {
        isOK = false;
        break;
      }
    }
  }
  // get and - if needed - advance to next obstacle group
  og = getObstGrp(odoPose);
  // update group settings - in case parameters has changed since creation og group
  getObstacleGroupSettings(og);
  //
  if (isOK)
  { // not fixed, so merge with known obstacles in recent group
    isOK = og->addObstPoly(poly, odoPose, outdoorObsts, horizontalScan);
    // debug
/*    if (not isOK)
      printf("UObstaclePool::addObst failed (obstacle group %d)\n", newest);*/
    // debug end
  }
  // add new obstacle
  //return og->addObst(scan, odoPose, idx1, idx2, outdoorObsts, horizontalScan);
  return isOK;
}
///////////////////////////////////////////////////////////

UObstacleGroupLaser * UObstaclePool::getObstGrp(UPoseTime pt)
{
  UObstacleGroupLaser * og;
  double d, dt;
  UPoseTime pt1;
  //
  og = groups[newest];
  if (og == NULL)
  {
    og = advanceNewGroup();
    og->setPoseFirst(pt);
    og->setPosePrev(pt);
    getObstacleGroupSettings(og);
  }
  else
  { // test if new group is to be added
    getObstacleGroupSettings(NULL);
    pt1 = og->getPoseFirst();
    d = hypot(pt.y - pt1.y, pt.x - pt1.x);
    // dt may be negative during simulation and replay, so remove sign
    dt = fabs(pt.t - pt1.t);
    if ((d > newGrpDist) or (dt > newGroupTime))
    { // remove invalid obstacles from the now closed group
      og->removeInvalid(pt.t, 1);
      if (logo != NULL)
        og->logAll(1, logo);
      og = advanceNewGroup();
      getObstacleGroupSettings(og);
      og->setPoseFirst(pt);
      og->setPosePrev(pt);
    }
  }
  // return obstacle group relevant for this position and time
  return og;
}

///////////////////////////////////////////////////////////////

void UObstaclePool::print(const char * prestr)
{
  int i;
  //
  printf("%s groupsCnt=%d\n", prestr, groupsCnt);
  for (i = 0; i < groupsCnt; i++)
  {
    groups[i]->print("--");
  }
}

//////////////////////////////////////////////////////////////

UObstacleGroupLaser * UObstaclePool::getGroup(int fromNewest)
{
  int n;
  UObstacleGroupLaser * ogl = NULL;
  //
  n = newest - fromNewest;
  if (n < 0)
    n += MAX_OBSTACLE_GROUPS;
  if ((n >= 0) and (n < MAX_OBSTACLE_GROUPS))
    ogl = groups[n];
  return ogl;
}

/////////////////////////////////////////

void UObstaclePool::setLogFile(FILE * logFile)
{
  if ((logo != NULL) and (logo != logFile))
    fclose(logo);
  logo = logFile;
}

/////////////////////////////////////////

void UObstaclePool::getObstacleGroupSettings(UObstacleGroupLaser * og)
{
  // Keep default setting
  // this method should be overwritten at a level that knows the
  // current parameter values
}

/////////////////////////////////////////

void UObstaclePool::obstDataUpdated(UTime poseTime)
{
  // debug 
  printf("nej! - UObstaclePool::dataUpdated - this is just an interface\n");
  // debug end
}

//////////////////////////////////////////////////

bool UObstaclePool::pointsToPolygon(ULaserScan * scan, UPoseTime odoPose,
                              int idx1, int idx2,
                              UPolygon * p40,
                              bool outdoorObsts, bool horizontalScan)
{
  int i;
  UPosition p1;
  UPolygon400 p400;
  //UPolygon40 p40;
  bool isOK;
  UPosition * pp;
  UPosition minPos;
  const double NEAR_VERTEX_DIST = 0.03;
  UMatrix4 mStoR;
  UPosRot pr;
  UObstacleGroupLaser * og;
  //
  isOK = (idx2 >= idx1);
  if (isOK)
  { // create a polygon from these points
    for (i = idx1; i <= idx2; i++)
    {
      p1 = scan->getPos(i);
      p400.add(p1);
      if ((p1.x < minPos.x) or (i == idx1))
        minPos = p1;
    }
    // make conves hull polygon
    isOK = p400.extractConvexTo(p40);
    // reduce vertex count if near
    p40->removeNearVertex(NEAR_VERTEX_DIST);
    //
    // convert to odometry coordinates
    pr = *scan->getSensorPose();
    // tilt and height is used already, so zero these
    pr.setPhi(0.0);
    pr.setZ(0.0);
    mStoR = pr.getRtoMMatrix();
    pp = p40->getPoints();
    for (i = 0; i < p40->getPointsCnt(); i++)
    { // move position to odometry coordinates
      // first move to robot coordinates
      p1 = mStoR * *pp;
      // then to odometry map coordinates
      *pp = odoPose.getPoseToMap(p1);
      // advance to next position in polygon
      pp++;
    }
  }
  // add correlation point if outdoor obstacle mode
  if (isOK)
  { // add the point with the smallest local x value
    // converted to map position using
    // the previous pose
    if (outdoorObsts and not horizontalScan)
    { // if outdoor obstacles, then assume most are low grass
      // seen at same relative position from time to time
      og = getObstGrp(odoPose);
      p1 = og->getPosePrev().getPoseToMap(mStoR * minPos);
      p40->add(p1);
    }
  }
  return isOK;
}
