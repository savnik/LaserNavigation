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
#include "ufeaturepool.h"

/////////////////////////////////////////
  
UFeatureData::UFeatureData()
{
  clear();
}
/////////////////////////////////////////

UFeatureData::~UFeatureData()
{
}

/////////////////////////////////////////

// bool UFeatureData::addSegment(ULineSegmet * seg)
// {
// }

/////////////////////////////////////////

void UFeatureData::clear()
{
  segsCnt = 0;
  valid = false;
  coordinateRef = -1;
  scanTime.clear();
}

/////////////////////////////////////////

void UFeatureData::moveLocalToOdo(UPose scanPose, UPosRot sensorPose)
{
  int i;
  ULineSegment * seg;
  UPosition p, p1, p2;
  UMatrix4 mLtoR;
  //
  if (coordinateRef == -1)
  { // is in sensor coordinates
    seg = segs;
    pose = scanPose;
    mLtoR = sensorPose.getRtoMMatrix();
    for (i = 0; i < segsCnt; i++)
    {
      p = mLtoR * seg->pos;
      p1 = pose.getPoseToMap(p);
      p = mLtoR * seg->getOtherEnd();
      p2 = pose.getPoseToMap(p);
      seg->setFromPoints(&p1, &p2);
      seg++;
    }
    coordinateRef = 1;
  }
}

/////////////////////////////////////////

bool UFeatureData::addSegment( ULineSegment * seg, int segInt, double segVal, const char * idStr)
{
  bool result = segsCnt < MAX_SEGMENTS;
  //
  if (result)
  {
    segs[segsCnt] = *seg;
    segsInt[segsCnt] = segInt;
    strncpy(segsStr[segsCnt], idStr, MAX_SEG_STR_LENGTH);
    segsValue[segsCnt] = segVal;
    segsCnt++;
  }
  //
  return result;
}

/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////

UFeaturePool::UFeaturePool()
{
  scansCnt = 0;
  newest = 0;
}

/////////////////////////////////////////

UFeaturePool::~UFeaturePool()
{
}

////////////////////////////////////////

void UFeaturePool::clear()
{
  int i;
  UFeatureData * sf = scans;
  //
  for (i = 0; i < scansCnt; i++)
    sf++->clear();
  scansCnt = 0;
  newest = 0;
}

/////////////////////////////////////////


bool UFeaturePool::addData(UFeatureData * data)
{
  int i, n;
  bool result = false;
  UFeatureData * dp;
  ULineSegment * sp;
  int * si;
  double * sv;
  //
  if (scansCnt < getScansMax())
    n = scansCnt;
  else
    n = (newest + 1) % getScansMax();
  dp = &scans[n];
  dp->lock();
  dp->clear();
  // remove newest marking on previous entries
  markAsNotNew(data->type);
  // copy data
  dp->scanTime = data->scanTime;
  dp->pose = data->pose;
  strncpy(dp->type, data->type, data->getTypeMax());
  // add line segments of scan
  sp = data->segs;
  si = data->segsInt;
  sv = data->segsValue;
  for (i = 0; i < data->segsCnt; i++)
  {
    result = dp->addSegment(sp++, *si++, *sv++, data->segsStr[i]);
  }
  dp->valid = true;
  // mark as the newest set of data
  dp->isNewest = true;
  dp->unlock();
  // increase scans count as needed
  scansCnt = maxi(scansCnt,  n + 1);
  // move the newest pointer
  newest = n;
  return result;
}

/////////////////////////////////////////////

void UFeaturePool::markAsNotNew(const char * dataType)
{
  UFeatureData * dp;
  int i, n;
  //
  n = newest;
  for (i = 0; i < scansCnt; i++)
  {
    dp = &scans[n];
    if (dp->isA(dataType))
    {
      if (dp->isNewest)
        dp->isNewest = false;
      else
        // no need to test any more, the rest is not newest 
        break;
    }
    n--;
    if (n < 0)
      n = getScansMax() - 1;
  }
}

/////////////////////////////////////////////

UFeatureData * UFeaturePool::getScan(int age)
{
  UFeatureData * result = NULL;
  int n;
  //
  if (scansCnt > 0)
  {
    n = newest - age;
    if (n < 0)
      n += getScansMax();
    result = &scans[n];
  }
  return result;
}

///////////////////////////////////////////
