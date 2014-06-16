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
#include <stdio.h>

#include <ugen4/ucommon.h>

#include "ulaserdataset.h"

UClientLaserData::UClientLaserData()
{ // falsiy optional flags
  flag = 0;
  //zoneB = false;
  //dazzle = false;
}
/////////////////////////////////////////////////////////

void UClientLaserData::setValue(double ang,
                          unsigned char lsb, unsigned char msb,
                          double measurementUnits)
{
  range = double((msb & 0x7f) * 256 + lsb) * measurementUnits;
  angle = ang * M_PI / 180.0;
  flag = (msb & 0x80) >> 6;
/*  zoneA = ((msb & 0x80) != 0) - only used flag now;
  zoneB = ((msb & 0x40) != 0) - not used;
  dazzle = ((msb & 0x20) != 0) - not used;
  changed with lms100 to allow 15 bit range 
  */
}

/////////////////////////////////////////////////////////

// void UClientLaserData::setValue(double ang, double distance, bool inA, bool inB, bool inZ)
// {
//   angle = ang;
//   range = distance;
//   zoneA = inA;
//   zoneB = inB;
//   dazzle = inZ;
// }

/////////////////////////////////////////////////////////

void UClientLaserData::print(char * preString)
{
  printf("%s az %6.2f deg, rng%6.3f m, valid=%s\n",
     preString, angle * 180.0 / M_PI, range, bool2str(isValid()));
}

/////////////////////////////////////////////////////////

bool UClientLaserData::setFromTag(USmlTag * tag)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  double v = 200.0;
//  int zz = -1;
  // <lval a=0 b=0 z=0 ang=-90.00 dist=1.647/>
  // <lval a=0 b=0 z=0 ang=-88.00 dist=1.600/>
  //
  range = -0.1;
/*  zoneA = false;
  zoneB = false;
  dazzle = false;*/
  flag = 0;
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "a") == 0)
      flag += strtol(val, NULL, 10);
    else if (strcasecmp(att, "b") == 0)
      flag += strtol(val, NULL, 10) * 2;
    else if (strcasecmp(att, "z") == 0)
      flag += strtol(val, NULL, 10) * 3;
    else if (strcasecmp(att, "zz") == 0)
      flag = strtol(val, NULL, 10);
    else if (strcasecmp(att, "ang") == 0)
    {
      sscanf(val, "%lf", &v);
      angle = v * M_PI/180.0;
    }
    else if (strcasecmp(att, "dist") == 0)
      sscanf(val, "%lf", &range);
/*    else if (strcasecmp(att, "var") == 0)
      sscanf(val, "%lf", &var);*/
    else if (strcasecmp(att, "varL") == 0)
      sscanf(val, "%lf", &varL);
/*    else if (strcasecmp(att, "varI") == 0)
      sscanf(val, "%lf", &varI);*/
/*    else if (strcasecmp(att, "varLC") == 0)
      sscanf(val, "%lf", &varLC);*/
    else if (strcasecmp(att, "tilt") == 0)
      sscanf(val, "%lf", &tilt);
/*    else if (strcasecmp(att, "edge") == 0)
      sscanf(val, "%lf", &edge);
    else if (strcasecmp(att, "curv") == 0)
      sscanf(val, "%lf", &curv);*/
  }
  //printf("Range should be 0.5 < range < 8.1, but is %f\n", range);
/*  switch (zz)
  { // zones coded as int (enum={easy, rough, not})
    case -1: break;
    case 0: break;
    case 1: zoneA = true; break;
    case 2: zoneB = true; break;
    case 3: zoneA = true; zoneB = true; break;
    default: break;
  }*/
  return ((v < 181.0) and (range > 0.0));
}

/////////////////////////////////////////////////////////

UPosition UClientLaserData::getPosition(UPosition laserPos, double cosLaserTilt)
{
  UPosition result = laserPos;
  //
  result.x += getDistance() * cos(getAngle()) * cosLaserTilt; // forward
  result.y += getDistance() * sin(getAngle()); // right
  //
  return result;
}

/////////////////////////////////////////////////////////

UPosition UClientLaserData::getPosition(UMatrix4 * movLtoR)
{
  UPosition result;
  UPosition p;
  //
  p.x = getDistance() * cos(getAngle());
  p.y = getDistance() * sin(getAngle());
  p.z = 0;

  result = (*movLtoR) * p;
  //
  return result;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

UClientLaserPi::UClientLaserPi()
{
}

/////////////////////////////////////////////////////////

UClientLaserPi::~UClientLaserPi()
{
}

/////////////////////////////////////////////////////////

void UClientLaserPi::clear()
{
  leftAngle = 0.0;
  rightAngle = 0.0;
  linksCnt = 0;
  leftSide.clear();
  rightSide.clear();
  topPos.clear();
  leftPos.clear();
  rightPos.clear();
}

/////////////////////////////////////////////////////////

bool UClientLaserPi::setFromTag(USmlTag * tag)
{
  bool result = true;
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  int n = 0, k = 0;
//<pi left=-6.00 right=-67.00 lx=3.884 ly=-0.346, lz=-0.078 rx=3.751 ry=-7.456, rz=-0.058 linkCnt=0/>
//<pi left=54.00 right=4.00 lx=3.882 ly=4.543, lz=-0.078 rx=3.170 ry=0.181, rz=0.029 linkCnt=2/>
  //
  clear();
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "left") == 0)
      n = sscanf(val, "%lf", &leftAngle);
    else if (strcasecmp(att, "right") == 0)
      n = sscanf(val, "%lf", &rightAngle);
    else if (strcasecmp(att, "lx") == 0)
      n = sscanf(val, "%lf", &leftPos.x);
    else if (strcasecmp(att, "ly") == 0)
      n = sscanf(val, "%lf", &leftPos.y);
    else if (strcasecmp(att, "lz") == 0)
      n = sscanf(val, "%lf", &leftPos.z);
    else if (strcasecmp(att, "rx") == 0)
      n = sscanf(val, "%lf", &rightPos.x);
    else if (strcasecmp(att, "ry") == 0)
      n = sscanf(val, "%lf", &rightPos.y);
    else if (strcasecmp(att, "rz") == 0)
      n = sscanf(val, "%lf", &rightPos.z);
    else if (strcasecmp(att, "lsx") == 0)
      leftSide.x = strtod(val, NULL);
    else if (strcasecmp(att, "lsy") == 0)
      leftSide.y = strtod(val, NULL);
    else if (strcasecmp(att, "lsz") == 0)
      leftSide.z = strtod(val, NULL);
    else if (strcasecmp(att, "rsx") == 0)
      rightSide.x = strtod(val, NULL);
    else if (strcasecmp(att, "rsy") == 0)
      rightSide.y = strtod(val, NULL);
    else if (strcasecmp(att, "rsz") == 0)
      rightSide.z = strtod(val, NULL);
    else if (strcasecmp(att, "ctx") == 0)
      topPos.x = strtod(val, NULL);
    else if (strcasecmp(att, "cty") == 0)
      topPos.y = strtod(val, NULL);
    else if (strcasecmp(att, "ctz") == 0)
      topPos.z = strtod(val, NULL);
    else if (strcasecmp(att, "linkCnt") == 0)
      n = sscanf(val, "%d", &linksCnt);
    k++;
  }
  result = (leftPos.dist(rightPos) > 0.1);
  if (n < 0)
    printf("Range should be 0.02 < range < maxLaserRange, but is %f\n", rightPos.dist());
  return result;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

ULaserDataSet::ULaserDataSet()
{
  clear();
}

/////////////////////////////////////////////////////////


ULaserDataSet::~ULaserDataSet()
{
}

/////////////////////////////////////////////////////////

void ULaserDataSet::clear()
{
  pisCnt = 0;
  count = 0;
  statValid = false;
  odoValid = false;
  scanSerial = 0;
  laserTilt = 0.0;
}

/////////////////////////////////////////////////////////

void ULaserDataSet::setData(
           unsigned int serial,
           int firstVal, int dataInterval,
           int dataCount, UTime time,
           double measurementUnit,
           bool statDataValid,
           double statWindowWidth,
           double laserTilt,
           double maxRange
                           )
{
  scanSerial = serial;
  first = firstVal;
  interval = dataInterval;
  count = dataCount;
  dataTime = time;
  unit = measurementUnit;
  statValid = statDataValid;
  statWidth = statWindowWidth;
  maxValidRange = maxRange;
}

/////////////////////////////////////////////////////////

void ULaserDataSet::setPose(UPose pose)
  {
    odoPose = pose;
    odoValid = true;
  }

/////////////////////////////////////////////////////////

void ULaserDataSet::addPassableInterval(UClientLaserPi * passableInterval)
{
  if (pisCnt < (MAX_PIS_COUNT - 1))
    pisCnt++;
  pis[pisCnt - 1] = *passableInterval;
}


/////////////////////////////////////////////////////////

void ULaserDataSet::print(char * preString, bool verbose)
{
  int i;
  UClientLaserData * pd;
  const int MSL = 30;
  char sd[MSL];
  char st[MSL];
  //
  dataTime.getDateString(sd, true);
  dataTime.getTimeAsString(st, true);
  printf("%s at %s %s, unit %4.2f m, %d values\n",
      preString, sd, st, unit, count);
  if (count > 0)
    printf(" - values from %4.2f to %4.2f deg, interval %4.2f deg\n",
       data[0].getAngle() * 180.0 / M_PI,
       data[count-1].getAngle() * 180.0 / M_PI,
       data[1].getAngle() - data[0].getAngle());
  if (verbose)
  {
    pd = data;
    for (i = 0; i < count; i++)
    {
      snprintf(st, MSL, " - #%03d: ", i);
      pd->print(st);
      pd++;
    }
  }
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


ULaserDataHistory::ULaserDataHistory()
{
  int i;
  ULaserDataSet ** ls;
  //
  newest = -1;
  scansCnt = 0;
  ls = scans;
  for (i = 0; i < MAX_STORED_LASER_SCANS; i++)
  {
    *ls = NULL;
    ls++;
  }
}

/////////////////////////////////////////////////////////////////

ULaserDataHistory::~ULaserDataHistory()
{ // release all scans
  int i;
  ULaserDataSet ** ls;
  //
  ls = scans;
  for (i = 0; i < MAX_STORED_LASER_SCANS; i++)
    if (ls != NULL)
      delete *ls++;
    else
      break;
}

/////////////////////////////////////////////////////////////////

void ULaserDataHistory::clear()
{
  scansCnt = 0;
  newest = -1;
}

/////////////////////////////////////////////////////////////////

ULaserDataSet * ULaserDataHistory::getNewScan()
{
  ULaserDataSet * ls;
  // advance to next scan slot
  newest = (newest + 1) % MAX_STORED_LASER_SCANS;
  // advance count if needed
  if (newest >= scansCnt)
    scansCnt++;
  // create new if needed
  if (scans[newest] == NULL)
    scans[newest] = new ULaserDataSet();
  // get pointer to scan
  ls = scans[newest];
  return ls;
}

/////////////////////////////////////////////////////////////////

ULaserDataSet * ULaserDataHistory::getNewest()
{
  ULaserDataSet * ls = NULL;
  // advance to next scan slot
  // get pointer to scan
  if (newest >= 0)
    ls = scans[newest];
  return ls;
}

/////////////////////////////////////////////////////////////////

ULaserDataSet * ULaserDataHistory::getScan(int histNum)
{
  ULaserDataSet * ls;
  int n;
  //
  n = newest - histNum;
  if (n < 0)
    n += MAX_STORED_LASER_SCANS;
  if ((n > scansCnt) or (n < 0))
    ls = NULL;
  else
    ls = scans[n];
  return ls;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

