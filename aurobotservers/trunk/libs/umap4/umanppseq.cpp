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

#include "umanppseq.h"
#include "umanarc.h"
#include "umanline.h"

UManPPSeq::UManPPSeq()
{
  int i;
  //
  seqCnt = 0;
  endVel = 0.0;
  manTime = 0.0;
  manDistance = 0.0;
  for (i = 0; i < MAX_MAN_CNT; i++)
    seq[i] = NULL;
  fixedFirstPose = false;
  fixedLastPose = false;
}

//////////////////////////////////////////

UManPPSeq::~UManPPSeq()
{ // delete any remaining manoeuvres
  int i;
  if (seqCnt > 0)
  {
    for (i = 0; i < seqCnt; i++)
      delete seq[i];
  }
}

//////////////////////////////////////////////

bool UManPPSeq::add(UManoeuvre * man)
{
  bool result = (seqCnt < getMaxManCnt());
  //
  if (result and man != NULL)
    seq[seqCnt++] = man;
  //
  return result;
}

//////////////////////////////////////////////

bool UManPPSeq::insert(UManoeuvre * man, int point)
{
  bool result = (seqCnt < getMaxManCnt());
  int at;
  //
  if (result and man != NULL)
  { // test index validity
    at = maxi(0, mini(point, seqCnt));
    if (at < seqCnt)
      memmove(&seq[at+1], &seq[at], sizeof(UManoeuvre *) * (seqCnt - at));
    seq[at] = man;
    seqCnt++;
  }
  return result;
}

//////////////////////////////////////////////

UManoeuvre * UManPPSeq::extract(int point)
{
  UManoeuvre * result = NULL;
  //
  if ((point >= 0) and (point < seqCnt))
  {
    result = seq[point];
    seqCnt--;
    if (point < (seqCnt))
      memmove(&seq[point], &seq[point+1],
              sizeof(UManoeuvre *) * (seqCnt - point));
  }
  //
  return result;
}

//////////////////////////////////////////////

UManoeuvre * UManPPSeq::getMan(int point)
{
  UManoeuvre * result = NULL;
  //
  if ((point >= 0) and (point < seqCnt))
    result = seq[point];
  //
  return result;
}

/////////////////////////////////////////////////

UPoseV UManPPSeq::getPoseV(double atManTime, UPoseV startPose)
{
  UPoseV pv = startPose;
  int i;
  UManoeuvre ** man;
  double sec, ssec = 0.0;
  //
  man = seq;
  for (i = 0; i < seqCnt; i++)
  {
    sec = (*man)->getManTime(startPose.getVel());
    if (sec + ssec > atManTime)
    { // end time is within this manoeuvre
      pv = (*man)->getPoseV(atManTime - ssec, pv, endPose.getVel());
      break;
    }
    else
      // continue to next
      pv = (*man)->getEndPoseV(pv);
    // summ used time
    ssec += sec;
    man++;
  }
  return pv;
}

//////////////////////////////////////////////

UPoseV UManPPSeq::getPoseV(int mn)
{
  UPoseV pv = startPose;
  int i;
  UManoeuvre ** man;
  //
  man = seq;
  for (i = 0; i <= mn; i++)
  { // get end pose
    if (i >= seqCnt)
      break;
    pv = (*man)->getEndPoseV(pv);
    man++;
  }
  return pv;
}

//////////////////////////////////////////////

void UManPPSeq::fprint(FILE * fd, const char * prestring)
{
  int i;
  double v = startPose.getVel();
  //
  fprintf(fd, "%s seqCnt=%d, endVel=%.2fm/s, dt=%.2fs, dist=%.3fm\n",
         prestring, seqCnt, endVel, manTime, manDistance);
  startPose.fprint(fd, " --- from");
  endPose.fprint(fd,   " --- to  ");
  for (i = 0; i < seqCnt; i++)
    seq[i]->fprint(fd, " --- ", &v);
}

//////////////////////////////////////////////

void UManPPSeq::print(const char * prestring, char * buff, const int buffCnt)
{
  int i;
  double v = startPose.getVel();
  char * p1 = buff;
  int n = 0;
  //
  snprintf(buff, buffCnt, "%s seqCnt=%d, endVel=%.2fm/s, dt=%.2fs, dist=%.3fm\n",
          prestring, seqCnt, endVel, manTime, manDistance);
  n += strlen(p1);
  p1 = &buff[n];
  startPose.print(" --- from", p1, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  endPose.print(" --- to  ", p1, buffCnt - n);
  for (i = 0; i < seqCnt; i++)
  {
    n += strlen(p1);
    p1 = &buff[n];
    seq[i]->print(" --- ", &v, p1, buffCnt - n);
  }
}

///////////////////////////////////////////////

double UManPPSeq::calcEndVel(double * andManTime, double * manDist)
{
  double v = startPose.getVel();
  double t = 0.0;
  double d = 0.0;
  int i;
  UManoeuvre ** ma = seq;
  //
  for (i = 0; i < seqCnt; i++)
  {
    if (andManTime != NULL)
      t += (*ma)->getManTime(v);
    if (manDist != NULL)
      d += (*ma)->getDistance();
    v = (*ma)->getEndV(v);
    ma++;
  }
  if (andManTime != NULL)
    *andManTime = t;
  if (manDist != NULL)
    *manDist = d;
  return v;
}

///////////////////////////////////////////////

bool UManPPSeq::setMaxVel(double maxVel)
{ // workaround to make sure this is the max end velocity
  bool isOK = true;
  int i;
  UManoeuvre ** ma = seq;
  //
  if (fabs(startPose.getVel()) > maxVel)
  {
    isOK = false;
    if (startPose.getVel() > 0.0)
      startPose.setVel(maxVel);
    else
      startPose.setVel(-maxVel);
  }
  if (fabs(endVel) > maxVel)
  {
    isOK = false;
    if (endVel > 0.0)
      endVel = maxVel;
    else
      endVel = -maxVel;
  }
  //
  for (i = 0; i < seqCnt; i++)
  {
    if (fabs((*ma)->getVel()) > maxVel)
    {
      isOK = false;
      if ((*ma)->getVel() > 0.0)
        (*ma)->setVel(maxVel);
      else
        (*ma)->setVel(-maxVel);
    }
    ma++;
  }
  return isOK;
}

///////////////////////////////////////////////

UPoseV UManPPSeq::calcEndPose(UPoseV startPose)
{
  UPoseV mid;
  int i;
  UManoeuvre * man;
  //
  mid = startPose;
  for (i = 0; i < seqCnt; i++)
  {
    man = seq[i];
    mid = man->getEndPoseV(mid);
  }
  return mid;
}

///////////////////////////////////////////////

double UManPPSeq::getDistanceXYSigned(UPosition pos, int * where,
                                      bool posIsRight,
                                      UPose * pHit, double * atT,
                                      int * atMan)
{
  UPosition p1, p2;
  int i;
  UManoeuvre * man;
  int minW = 1, w; // 0=on line, 1=near first end, 2=near far end
  double d, minDs = 0.0, minD = 0.0;
  UPoseV pose1, pose2;
  double t, dt;
  UPose pH;
  //int minMan = -1;
  //
  p1 = startPose.getMapToPose(pos);
  pose1 = startPose;
  t = 0.0;
  for (i = 0; i < seqCnt; i++)
  {
    man = seq[i];
    d = man->getDistanceXYSigned(p1, &w, false, false, &pH, &dt);
    if ((fabs(d) < minD) or (i == 0))
    {
      minDs = d;
      minD = fabs(d);
      minW = w;
      //minMan = i;
    }
    if (w == 0)
      break;
    if ((i + 1) >= seqCnt)
      break;
    pose2 = man->getEndPose();
    // get point seen locally from next manoeuvre
    p1 = pose2.getMapToPose(p1);
    // get global base for next manoeuvre
    pose1 = pose1.getPoseToMapPose(pose2);
    // summ the distance to the hitpoint
    t += man->getDistance();
  }
  // convert closest point to map coordinates
  if  (pHit != NULL)
    *pHit = pose1.getPoseToMapPose(pH);
  if (atT != NULL)
    // return also distance into manoeuvre
    *atT = t + dt;
  // return distance and where
  if (posIsRight)
    minDs = -minDs;
  if (where != NULL)
    *where = minW;
  if (atMan != NULL)
    *atMan = i;
  return minDs;
}

///////////////////////////////////////////////////

double UManPPSeq::getMinDistanceXYSigned(ULineSegment * seg, int * whereOnSeg,
                                UPosition * posOnSeg,
                                bool posIsRight,
                                int * whereOnMan,
                                UPose * poseOnMan)
{
  UPosition p1, p2, pS, minPs;
  int i;
  UManoeuvre * man;
  int wS, minWs = 1, minWm = 1, wM; // 0=on line, 1=near first end, 2=near far end
  double d, minDs = 0.0, minD = 0.0;
  UPoseV pose1, pose2, minPose;
  UPose pM, minPm;
  ULineSegment segL; // segment in local coordinates
  //
  segL = *seg;
  segL.pos.z = 0.0;
  segL.vec.z = 0.0;
  pose2 = startPose;
  pose1 = startPose;
  minPose = pose1;
  for (i = 0; i < seqCnt; i++)
  { // get segment in local coordinates for first manoeuvre
    p1 = pose2.getMapToPose(segL.pos);
    p2 = pose2.getMapToPose(segL.getOtherEnd());
    segL.setFromPoints(p1, p2);
    // get distace
    man = seq[i];
    d = man->getMinDistanceXYSigned(&segL, &wS, &pS, posIsRight, &wM, &pM);
    if ((fabs(d) < minD) or (i == 0))
    { // get closest distance in absolute values
      minDs = d; // but save the signed value
      minD = fabs(d);
      minWs = wS;
      minWm = wM;
      minPose = pose1;
      minPm = pM;
      minPs = pS;
    }
    if ((i + 1) >= seqCnt)
      break;
    // prepare for next manoeuvre
    pose2 = man->getEndPose();
      // get global base for next manoeuvre
    pose1 = pose1.getPoseToMapPose(pose2);
  }
  // closest manoeuvre point to map coordinates
  if  (poseOnMan != NULL)
    *poseOnMan = minPose.getPoseToMapPose(minPm);
  // point on segment to map coordinates
  if (posOnSeg != NULL)
    *posOnSeg = minPose.getPoseToMap(minPs);
  if (whereOnMan != NULL)
    *whereOnMan = minWm;
  if (whereOnSeg != NULL)
    *whereOnSeg = minWs;
  return minDs;
}
///////////////////////////////////////////////////

double UManPPSeq::getMaxTurnArc()
{
  double maxArc = 0.0;
  int i;
  UManoeuvre * mn;
  UManArc * ma;
  //
  for (i = 0; i < seqCnt; i++)
  {
    mn = seq[i];
    if (mn->getManType() == UManoeuvre::MAN_ARC)
    {
      ma = (UManArc*)mn;
      if (fabs(ma->getTurnAngle()) > maxArc)
        maxArc = fabs(ma->getTurnAngle());
    }
  }
  return maxArc;
}

/////////////////////////////////////////

void UManPPSeq::truncate(int i)
{
  int n;
  for (n = i; n < seqCnt; n++)
    seq[i] = NULL;
  seqCnt = i;
}

/////////////////////////////////////////

void UManPPSeq::truncate(UPosition pos, double preDist)
{
  UPose pHit;
  double atT, d;
  int m, i, w;
  UManoeuvre * man;
  UManArc * mArc;
  UManLine * mLine;
  //
  getDistanceXYSigned(pos, &w, false, &pHit, &atT, &m);
  if (preDist >= atT)
    truncate(0);
  else
  {
    d = 0.0;
    for (i = 0; i < seqCnt; i++)
    {
      man = seq[i];
      if (d + man->getDistance() >= atT - preDist)
      {
        switch (man->getManType())
        {
          case UManoeuvre::MAN_LINE:
            mLine = (UManLine*) man;
            mLine->setDistance(atT - preDist - d);
            break;
          case UManoeuvre::MAN_ARC:
            mArc = (UManArc*) man;
            if (mArc->getTurnAngle() > 0.0)
              mArc->setTurnAngle((atT - preDist - d) / mArc->getTurnRadius());
            else
              mArc->setTurnAngle(-(atT - preDist - d) / mArc->getTurnRadius());
          default:
            break;
        }
        truncate(i + 1);
        endPose = calcEndPose(startPose);
        break;
      }
    }
  }
}

