/***************************************************************************
 *   Copyright (C) 2008 by DTU (Christian Andersen)                        *
 *   rse@elektro.dtu.dk                                                    *
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
#include "uobj3dpool.h"

///////////////////////////////////////////////////////////

void UObstacleVision::clear()
{
  human = false;
  height = 0.0;
  UObstacle::clear();
}

///////////////////////////////////////////////////////////

const char * UObstacleVision::codeXmlAttributes(char * buff, const int buffCnt)
{
    snprintf(buff, buffCnt, " human=\"%s\" gndBased=\"%s\" height=\"%.2f\"",
             bool2str(human), bool2str(gndBased), height);
  //
  return buff;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

UObj3dGroup::UObj3dGroup()
  : UObstacleGroup()
{
  obstacleMergeDistance = 0.25; // default is outdoor merge distance
  mergeObstacles = true;
}

///////////////////////////////////////////////////////////

UObj3dGroup::~UObj3dGroup()
{
}

//////////////////////////////////////////////////

bool UObj3dGroup::addObstPoly(UPolygon * newpoly, UPoseTime poset,
                              bool isHuman, bool isGndBased)
{
  UObstacleVision * ob, * ob2;
  int i;
  UPosition p1;
  UPolygon400 p400;
  bool isOK;
//  const double MARGIN_OUTDOOR = 0.25; // ROBOTWIDTH * 0.4; // margin for overlap // 0.1;
//  const double MARGIN_INDOOR = 0.03; // margin for overlap indoor
//  double usedMargin = mergeMargin;
  int merge[MAX_OBSTACLES];
  int mergeCnt = 0;
  int h;
  UPoseTime pt;
  const double NEAR_VERTEX_DIST = 0.03;
  //
  isOK = (newpoly != NULL);
  if (isOK)
  {
    isOK = newpoly->getPointsCnt() > 0;
  }
  // test if seen before
  if (isOK)
  { // add the point with the smallest local x value
    // converted to map position using
    if (mergeObstacles)
    {
      for (i = 0; i < obstsCnt; i++)
      { // test if there are overlap with existing obstacles
        ob = (UObstacleVision*) obsts[i];
        if (ob->isOverlappingXYconvex(newpoly, obstacleMergeDistance))
          merge[mergeCnt++] = i;
      }
    }
    //
    if (mergeCnt == 0)
    { // no overlap - add as new
      ob = (UObstacleVision*) getNewObst();
      isOK = (ob != NULL);
      if (not isOK)
        printf("UObj3dGroup::addObstPoly: no more space in obstacle group\n");
    }
    else
      // get destination obstacle
      ob = (UObstacleVision*) obsts[merge[0]];
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
        ob2 = (UObstacleVision*) obsts[merge[i]];
        isOK = p400.add(ob2);
        if (not isOK)
          printf("Polygon overflow - unlikely, but not fatal?\n");
        h = maxi(h, ob2->getHits());
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
      ob->setPoseLast(poset);
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

void UObj3dGroup::setMergeDistance(double mergeDist, bool doMergeObstacles)
{
  obstacleMergeDistance = mergeDist;
  mergeObstacles = doMergeObstacles;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

UObj3dPool::UObj3dPool()
{
  int i;
  for (i = 0; i < MAX_OBSTACLE_GROUPS; i++)
    groups[i] = NULL;
  groupsCnt = 0;
  logo = NULL;
  nextSerial = 0;
}

///////////////////////////////////////////////////



///////////////////////////////////////////////////

UObj3dPool::~UObj3dPool()
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
  newest = 0;
  if (logo != NULL)
  {
    if (groupsCnt > 0)
      groups[newest]->logAll(1, logo);
    fclose(logo);
  }
  logo = NULL;
}

///////////////////////////////////////////////////

void UObj3dPool::clear()
{
  int i;
  //
  for (i = 0; i < groupsCnt; i++)
    groups[i]->removeAllObsts();
  obstDataUpdated(getGroup(0)->getPoseLast().t);
}

///////////////////////////////////////////////////

void UObj3dPool::clearGrp(int idx)
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

UObj3dGroup * UObj3dPool::advanceNewGroup()
{ // advance and clear
  UObj3dGroup * og;
  //
  if (groupsCnt < MAX_OBSTACLE_GROUPS)
  { // space for new groups
    if (groups[newest] != NULL)
      newest++;
    og = groups[newest];
    if (og == NULL)
    {
      og = new UObj3dGroup();
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

UObj3dGroup * UObj3dPool::getObstGrp(UPoseTime pt)
{
  UObj3dGroup * og;
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
    dt = pt.t - pt1.t;
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

void UObj3dPool::print(const char * prestr)
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

UObj3dGroup * UObj3dPool::getGroup(int fromNewest)
{
  int n;
  UObj3dGroup * ogl = NULL;
  //
  n = newest - fromNewest;
  if (n < 0)
    n += MAX_OBSTACLE_GROUPS;
  if ((n >= 0) and (n < MAX_OBSTACLE_GROUPS))
    ogl = groups[n];
  return ogl;
}

/////////////////////////////////////////

void UObj3dPool::setLogFile(FILE * logFile)
{
  if ((logo != NULL) and (logo != logFile))
    fclose(logo);
  logo = logFile;
}

/////////////////////////////////////////

void UObj3dPool::getObstacleGroupSettings(UObj3dGroup * og)
{
  // Keep default setting
  // this method should be overwritten at a level that knows the
  // current parameter values
}

/////////////////////////////////////////

void UObj3dPool::obstDataUpdated(UTime poseTime)
{
  // debug 
  printf("now! - UObj3dPool::dataUpdated should be overwritten\n");
  // debug end
}

///////////////////////////////////////////////

void UObj3dPool::addObstacle(UPolygon * newPoly, UPoseTime odoPose,
                   bool isHuman, bool isGndBased)
{
  UObj3dGroup * og;
  UPosition * pp;
  int i;
  //
  pp = newPoly->getPoints();
  for (i = 0; i < newPoly->getPointsCnt(); i++)
  {
    *pp = odoPose.getPoseToMap(*pp);
    pp++;
  }
  og = getObstGrp(odoPose);
  //
  og->addObstPoly(newPoly, odoPose, isHuman, isGndBased);
  obstDataUpdated(odoPose.t);
}

//////////////////////////////////////////////
