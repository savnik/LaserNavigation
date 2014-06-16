/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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
#include "uobstacle.h"

///////////////////////////////////////////////////

UObstacle::UObstacle()
        : UPolygon40()
{
  valid = false;
  hits = 0;
  margin = 0.01;
}

///////////////////////////////////////////////////

UObstacle::~UObstacle()
{
}

///////////////////////////////////////////////////

// bool UObstacle::trimObst(ULaserScan * scan)
// {
//   printf("Not implemented\n");
//   return false;
// }

///////////////////////////////////////////////////

void UObstacle::clear()
{
  valid = false;
  hits = 0;
  UPolygon::clear();
}

///////////////////////////////////////////////////

void UObstacle::print(const char * prestr)
{
  const int MSL = 500;
  char s[MSL];
  print(prestr, s, MSL);
  printf("%s", s);
}

    ///////////////////////////////////////////////////

void UObstacle::print(const char * prestr, char * buff, const int buffCnt)
{
  UPosition pos;
  //
  pos = getCogXY();
  snprintf(buff, buffCnt, "%s (%s) hits=%d cnt=%d cog=%.2f,%.2f cogDist=%.2f\n",
         prestr, bool2str(isValid()), hits, pointsCnt, pos.x, pos.y, getCogXYmaxDist());
}

///////////////////////////////////////////////////

void UObstacle::setPoseLast(UPoseTime pose)
{
  if (pose2.t != pose.t)
  {
    pose2 = pose;
    hits++;
    valid = true;
  }
}

///////////////////////////////////////////////////

void UObstacle::logObst(FILE * logo)
{
  int i;
  UPosition * pos;
  //
  if (logo != NULL)
  {
    pos = points;
    for (i = 0; i < pointsCnt; i++)
    {
      fprintf(logo, "%d %.2f %.2f\n", i, pos->x, pos->y);
      pos++;
    }
  }
}

///////////////////////////////////////////////////

const char * UObstacle::codeXmlAttributes(char * buf, const int bufCnt)
{
  snprintf(buf, bufCnt, " hits=\"%d\" valid=\"%s\" serial=\"%lu\"",
           getHits(), bool2str(isValid()), getSerial());
  // this is the root obstacle type, so no more is needed
  return buf;
}

//////////////////////////////////////////////////

const char * UObstacle::codeXml(char * buf, const int bufCnt, char * extraAttr)
{
  char * p1;
  int n;
  const char * TAG_NAME = "obst";
  //
  snprintf(buf, bufCnt, "<%s", TAG_NAME);
  n = strlen(buf);
  p1 = &buf[n];
  if (extraAttr != NULL)
  { // there is an attribute pointer
    if (strlen(extraAttr) > 0)
    { // include any specific extra attributes - e.g. a name fiels
      snprintf(p1, bufCnt - n, " %s", extraAttr);
      n += strlen(p1);
      p1 = &buf[n];
    }
  }
  // code obstacle type
  snprintf(p1, bufCnt - n, " type=\"%s\"", getDataType());
  n += strlen(p1);
  p1 = &buf[n];
  // code specific attributes for this obstacle type
  codeXmlAttributes(p1, bufCnt - n);
  n += strlen(p1);
  p1 = &buf[n];
  // close header tag
  strncpy(p1, ">\n", mini(bufCnt - n, 4));
  n += strlen(p1);
  p1 = &buf[n];
  //
  pose1.codeXml(p1, bufCnt - n, "name=\"first\"");
  n += strlen(p1);
  p1 = &buf[n];
  //
  pose2.codeXml(p1, bufCnt - n, "name=\"last\"");
  n += strlen(p1);
  p1 = &buf[n];
  //
  UPolygon::codeXml(p1, bufCnt - n, NULL);
  n += strlen(p1);
  p1 = &buf[n];
  // end tag
  snprintf(p1, bufCnt - n, "</%s>\n", TAG_NAME);
  //
  return buf;
}

///////////////////////////////////////////////////

bool UObstacle::mergeableOnCogLimits(UPolygon * other, double margin)
{
  double d;
  bool mergeable1 = false;
  bool mergeable2 = false;
  UPolygon *o1, *o2;
  int i;
  UPosition p1, cog, pAt;
  bool aVertex;
  //
  if (getPointsCnt() > 2) 
  { // this has a positive area (is not a point or a line)
    cog = other->getCogXY();
    d = getClosestDistance(cog.x, cog.y, 2.0, &pAt, &aVertex);
    mergeable1 = (d <= 0.0);
  }
  if (not mergeable1)
  { // not meragbele by other inside this, so other way
    if (other->getPointsCnt() > 2)
    { // other has a positive area, so test 
      // d = other->getDistanceXYsigned2(getCogXY(), NULL, NULL, NULL);
      cog = getCogXY();
      d = other->getClosestDistance(cog.x, cog.y, 2.0, &pAt, &aVertex);
      mergeable2 = (d < 0.0);
    }
  }
  if (mergeable1 or mergeable2)
  { // test if
    if (mergeable1)
    {
      o1 = this;
      o2 = other;
    }
    else
    {
      o1 = other;
      o2 = this;
    }
    for (i = 0; i < o2->getPointsCnt(); i++)
    {
      p1 = o2->getPoint(i);
      //d = o1->getDistanceXYsigned2(p1, NULL, NULL, NULL);
      d = o1->getClosestDistance(p1.x, p1.y, margin + 0.1, &pAt, &aVertex);
      if (d > margin)
      { // too far away to merge
        mergeable1 = false;
        mergeable2 = false;
        break;
      }
    }
/*    if (merged1 or merged2)
    {
      other->add(this);
      other->extractConvexTo(obst);
    }*/
  }
  //
  return mergeable1 or mergeable2;
}



///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UObstacleGroup::UObstacleGroup()
{
  int i;
  clear();
  serial = 0;
  for (i = 0; i < MAX_OBSTACLES; i++)
    obsts[i] = NULL;
}

///////////////////////////////////////////////////

UObstacleGroup::~UObstacleGroup()
{
  int i;
  // delete associated obstacles.
  for (i = 0; i < MAX_OBSTACLES; i++)
  {
    if (obsts[i] != NULL)
      delete obsts[i];
    else
      break;
  }
}

///////////////////////////////////////////////////

void UObstacleGroup::clear()
{
  obstsCnt = 0;
  poseFirst.clear();
  poseFirst.clear();
  nextObstSerial = 1;
  // serial = 0;
}

///////////////////////////////////////////////////

void UObstacleGroup::removeAllObsts()
{
  lock();
  // remove all
  obstsCnt = 0;
  unlock();
}

///////////////////////////////////////////////////

void UObstacleGroup::print(const char * prestr)
{
  const int MSL = 500;
  char s[MSL];
  print(prestr, s, MSL, true);
  printf("%s", s);
}

///////////////////////////////////////////////////

void UObstacleGroup::print(const char * prestr, char * buff, const int buffCnt, bool detail)
{
  int i, val = 0;
  int n;
  char * p1;
  //
  for (i = 0; i < obstsCnt; i++)
  {
    if (obsts[i]->isValid())
      val++;
  }
  snprintf(buff, buffCnt, "%s obstCnt=%d (%.2fx, %.2fy) to (%.2fx, %.2fy), valid=%d\n",
         prestr, obstsCnt,
        poseFirst.x, poseFirst.y,
        poseLast.x, poseLast.y,
        val);
  if (detail)
  {
    n = strlen(buff);
    p1 = &buff[n];
    for (i = 0; i < obstsCnt; i++)
    {
      obsts[i]->print("---", p1, buffCnt - n);
      n += strlen(p1);
      p1 = &buff[n];
    }
  }
}

////////////////////////////////////////////////

UObstacle * UObstacleGroup::getNewObst()
{
  UObstacle * result = NULL;
  //
  if (obstsCnt < MAX_OBSTACLES)
  {
    result = obsts[obstsCnt];
    if (result == NULL)
    {
      // result = new UObstacle();
      result = makeNewObst();
      obsts[obstsCnt] = result;
    }
    obstsCnt++;
    if (result != NULL)
    {
      result->clear();
      result->setSerial(nextObstSerial++);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

void UObstacleGroup::removeObst(int idx)
{
  UObstacle * ob;
  //
  if ((idx >= 0) and (idx < obstsCnt))
  {
    obstsCnt--;
    // if last, then no action needed
    if (idx < obstsCnt)
    { // swap with last before reduce count
      ob = obsts[obstsCnt];
      obsts[obstsCnt] = obsts[idx];
      obsts[idx] = ob;
    }
  }
}

///////////////////////////////////////////////////

void UObstacleGroup::removeInvalid(UTime before, int hitLimit)
{
  int i;
  UObstacle * ob;
  UTime t;
  //
  for (i = 0; i < obstsCnt; i++)
  {
    ob = obsts[i];
    if (ob->getHits() <= hitLimit)
    {
      t = ob->getPPoseLast()->t;
      if ((before - t) > 0.01)
        removeObst(i);
    }
  }
}

///////////////////////////////////////////////////////////

void UObstacleGroup::logAll(unsigned int serial, FILE * logo)
{
  int i;
  UObstacle * ob;
  //
  if (logo != NULL)
  { // print         time first pose (x,y)
    fprintf(logo, "%lu.%06lu %.2f %.2f\n", poseFirst.t.getSec(),
            poseFirst.t.getMicrosec(), poseFirst.x, poseFirst.y);
    // print       scan lastPose (x,y)
    fprintf(logo, "-%u %.2f %.2f\n", serial, poseLast.x, poseLast.y);
    for (i = 0; i < obstsCnt; i++)
    { // in form:    index vertex (x,y)
      ob = obsts[i];
      ob->logObst(logo);
    }
  }
}

///////////////////////////////////////////////////

UObstacle * UObstacleGroup::getObstacle(unsigned long oSerial, bool mayCreate, int * obstacleIndex)
{
  int i;
  UObstacle ** obpp, * result;
  bool found = false;
  //
  obpp = obsts;
  for (i = 0; i < obstsCnt; i++)
  {
    found = ((*obpp)->getSerial() == oSerial);
    if (found)
    {
      break;
    }
    obpp++;
  }
  if (found)
    result = *obpp;
  else
  { // else take a new (new obstacles are pre-cleared)
    i = obstsCnt;
    result = getNewObst();
    if (result != NULL)
      result->setSerial(oSerial);
  }
  if ((result != NULL) and (obstacleIndex != NULL))
    *obstacleIndex = i;
  return result;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////



