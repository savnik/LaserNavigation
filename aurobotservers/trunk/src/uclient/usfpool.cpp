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
#include "usfpool.h"

/////////////////////////////////////////
  
USFData::USFData()
{
  clear();
}
/////////////////////////////////////////

USFData::~USFData()
{
}

/////////////////////////////////////////

// bool USFData::addSegment(ULineSegmet * seg)
// {
// }

/////////////////////////////////////////

void USFData::clear()
{
  segsCnt = 0;
  valid = false;
  scanTime.clear();
}

/////////////////////////////////////////

void USFData::moveLocalToMap(UPose scanPose, UPosRot sensorPose)
{
  int i;
  ULineSegment * seg;
  UPosition p, p1, p2;
  UMatrix4 mLtoR;
  //
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
}

/////////////////////////////////////////

bool USFData::addSegment( ULineSegment * seg, int segInt, const char * idStr)
{
  bool result = segsCnt < MAX_SEGMENTS;
  //
  if (result)
  {
    segs[segsCnt] = *seg;
    segInteger[segsCnt] = segInt;
    strncpy(segStr[segsCnt], idStr, MAX_SEG_STR_LENGTH);
    segsCnt++;
  }
  //
  return result;
}

/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////
/////////////////////////////////////////

USFPool::USFPool()
{
  scansCnt = 0;
  newest = 0;
}

/////////////////////////////////////////

USFPool::~USFPool()
{
}

////////////////////////////////////////

void USFPool::clear()
{
  int i;
  USFData * sf = scans;
  //
  for (i = 0; i < scansCnt; i++)
    sf++->clear();
  scansCnt = 0;
  newest = 0;
}

/////////////////////////////////////////


bool USFPool::addData(USFData * data)
{
  int i, n;
  bool result = false;
  USFData * dp;
  ULineSegment * sp;
  int * si;
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
  si = data->segInteger;
  for (i = 0; i < data->segsCnt; i++)
  {
    result = dp->addSegment(sp++, *si++, data->segStr[i]);
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

void USFPool::markAsNotNew(const char * dataType)
{
  USFData * dp;
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

USFData * USFPool::getScan(int age)
{
  USFData * result = NULL;
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
