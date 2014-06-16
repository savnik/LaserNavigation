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
#include <math.h>

#include <ugen4/ucommon.h>

#include "ulaserdataset.h"

ULaserData::ULaserData()
{ // falsiy optional flags
  flag = 0;
  //zoneB = false;
  //dazzle = false;
}
/////////////////////////////////////////////////////////

void ULaserData::setValue(double ang,
                          unsigned char lsb, unsigned char msb,
                          double measurementUnits)
{
  range = double((msb & 0x1f) * 256 + lsb) * measurementUnits;
  angle = ang * M_PI / 180.0;
  flag = (msb & 0xC0) >> 6;
/*  zoneA = ((msb & 0x80) != 0);
  zoneB = ((msb & 0x40) != 0);
  dazzle = ((msb & 0x20) != 0);*/
}

/////////////////////////////////////////////////////////

// void ULaserData::setValue(double ang, double distance, bool inA, bool inB, bool inZ)
// {
//   angle = ang;
//   range = distance;
//   zoneA = inA;
//   zoneB = inB;
//   dazzle = inZ;
// }

/////////////////////////////////////////////////////////

void ULaserData::print(char * preString)
{
  printf("%s az %6.2f deg, rng%6.3f m, A=%d B=%d Z=%d\n",
     preString, angle * 180.0 / M_PI, range,
     inZoneA(), inZoneB(), isDazzled());
}

/////////////////////////////////////////////////////////

bool ULaserData::setFromTag(USmlTag * tag)
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

UPosition ULaserData::getPosition(UPosition laserPos, double cosLaserTilt)
{
  UPosition result = laserPos;
  //
  result.x += getDistance() * cos(getAngle()) * cosLaserTilt; // forward
  result.y += getDistance() * sin(getAngle()); // right
  //
  return result;
}

/////////////////////////////////////////////////////////

UPosition ULaserData::getPosition(UMatrix4 * movLtoR)
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

ULaserPi::ULaserPi()
{
}

/////////////////////////////////////////////////////////

ULaserPi::~ULaserPi()
{
}

/////////////////////////////////////////////////////////

void ULaserPi::clear()
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

bool ULaserPi::setFromTag(USmlTag * tag)
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
  if (n < 0)
    result = false;
  else
    result = (leftPos.dist(rightPos) > 0.1);
  //printf("Range should be 0.5 < range < 8.1, but is %f\n", range);
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
           double laserTilt)
{
  scanSerial = serial;
  first = firstVal;
  interval = dataInterval;
  count = dataCount;
  dataTime = time;
  unit = measurementUnit;
  statValid = statDataValid;
  statWidth = statWindowWidth;
}

/////////////////////////////////////////////////////////

void ULaserDataSet::setPose(UPose pose)
  {
    odoPose = pose;
    odoValid = true;
  }

/////////////////////////////////////////////////////////

void ULaserDataSet::addPassableInterval(ULaserPi * passableInterval)
{
  if (pisCnt < (MAX_PIS_COUNT - 1))
    pisCnt++;
  pis[pisCnt - 1] = *passableInterval;
}

/////////////////////////////////////////////////////////

void ULaserDataSet::print(const char * preString, bool verbose)
{
  int i;
  ULaserData * pd;
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

ULaserPathResult::ULaserPathResult()
{
  passLinesCnt = 0;
  edgeLeftValid = false;
  edgeRightValid = false;
  edgeTopValid = false;
  routeCnt = 0;
  man = 0;
}

/////////////////////////////////////////////////////////////////

ULaserPathResult::~ULaserPathResult()
{ // nothing yet
}

/////////////////////////////////////////////////////////////////

bool ULaserPathResult::setFromTag(USmlTag * tag)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  //int interv = 0;
  bool result = false;
  USmlTag nTag;
  UPose pose;
  //
  // init to not valid ..
  edgeLeftValid = false;
  edgeTopValid = false;
  edgeRightValid = false;
  // ... not through a wall
  isAWall = false;
  // ... and no passable lines
  passLinesCnt = 0;
  bool gotdata = false;
  /*
<path intervals=5 usedPath=true path=false isAWall=false edgeLeft=true edgeRight=true edgeTop=true>
<lineSeg name="leftEdge" length=1.95030689239502e+00>
<pos3d name="start" x=2.15716743469238e+00 y=-1.88146317005157e+00 z=1.80217644064948e-01/>
<pos3d name="vector" x=9.99599030828172e-01 y=5.72597424506551e-03 z=-2.77303761985254e-02/>
<lineSeg/>
<lineSeg name="rightEdge" length=4.35410022735596e+00>
<pos3d name="start" x=1.46796917915344e+00 y=-3.50925517082214e+00 z=2.42707570923525e-01/>
<pos3d name="vector" x=6.73178089212711e-01 y=-7.39479378965110e-01 z=-1.25262436578425e-03/>
<lineSeg/>
<lineSeg name="topEdge" length=2.70666003227234e+00>
<pos3d name="start" x=1.84651243686676e+00 y=-2.91633820533752e+00 z=2.14772807523954e-01/>
<pos3d name="vector" x=9.62534821456491e-01 y=-2.70804501717895e-01 z=-1.38415755166105e-02/>
<lineSeg/>
<lineSeg name="passableInterval" length=4.59317588806152e+00>
<pos3d name="start" x=4.57920722402584e+00 y=-6.56502241494822e+00 z=2.37253518887672e-01/>
<pos3d name="vector" x=-1.02714499275071e-01 y=9.94416597062924e-01 z=-2.41921105046797e-02/>
<lineSeg/>
...
<lineSeg name="passableInterval" length=1.43088471889496e+00>
<pos3d name="start" x=1.71957826614380e+00 y=-3.28020524978638e+00 z=2.42707570923525e-01/>
<pos3d name="vector" x=3.05969167544701e-01 y=9.51039274669251e-01 z=-4.36722302177049e-02/>
<lineSeg/>

  ------pos3d is depricated
<pos3d name="oaRoute" x=3.05969167544701e-01 y=9.51039274669251e-01 z=-4.36722302177049e-02/>
<pos3d name="oaRoute" x=3.05969167544701e-01 y=9.51039274669251e-01 z=-4.36722302177049e-02/>
<pos3d name="oaRoute" x=3.05969167544701e-01 y=9.51039274669251e-01 z=-4.36722302177049e-02/>

  ------manseq is new path
  <manseq cnt="2" endV="0.455" dt="24.215">
  <posev name="start" x="0.034" y="-0.001" h="-0.062" v="0.115"/>
  <posev name="end" x="3.873" y="-4.784" h="-0.062" v="0.800"/>
  <manoeuvre typ="arc" rad="2.073" arc="-1.6653" acc="0.0140562" endVel="  0.8"/>
  <manoeuvre typ="arc" rad="2.073" arc="1.6653" acc="0.0140562" endVel="  0.8"/>
  </manseq>

  </path>
*/  //
  pathUsed = false;
  routeCnt = 0;
  if (man != NULL)
    man->releaseAllMan();
  //
  tag->reset();
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "intervals") == 0)
    {
      //interv = strtol(val, NULL, 0);
      result = true;
    }
    else if (strcasecmp(att, "pathUsed") == 0)
      // ia path actually used for control
      pathUsed = str2bool(val);
    else if (strcasecmp(att, "isAWall") == 0)
      // ia path actually used for control
      isAWall = str2bool(val);
    else if (strcasecmp(att, "aCrash") == 0)
      isACrash = str2bool(val);
    // edge valid is set when edge is decoded
  }
  if (result and tag->isAStartTag())
  {
    while (result)
    { // continue until endTag is found
      result = tag->getNextTag(&nTag, 400);
      if (not result)
        break;
      if (nTag.isTagA("lineSeg"))
      { // all subgroups should be a line segment (up to now)
        while (nTag.getNextAttribute(att, val, MVL))
        { // get name of line segment to determine where to put it
          if (strcasecmp(att, "name") == 0)
          { // name is found
            if (strcasecmp(val, "leftEdge") == 0)
              edgeLeftValid = nTag.getLineSegment(&edgeLeft);
            else if (strcasecmp(val, "rightEdge") == 0)
              edgeRightValid = nTag.getLineSegment(&edgeRight);
            else if (strcasecmp(val, "topEdge") == 0)
              edgeTopValid = nTag.getLineSegment(&edgeTop);
            else if (strcasecmp(val, "passableInterval") == 0)
            { // one of a number of intervals
              // if no space, then drop the rest
              if (passLinesCnt < MAX_PASSABLE_LINE_SEGMENTS)
                result = nTag.getLineSegment(
                      &passLines[passLinesCnt]);
              if (result and (passLinesCnt < MAX_PASSABLE_LINE_SEGMENTS))
                passLinesCnt++;
            }
            gotdata = true;
            // tag should be used by now
            break;
          }
        } // while more attributes
      }
      else if (nTag.isTagA("manseq"))
      {
        if (man == NULL)
          man = new UManSeq();
        nTag.getManSeq(man);
        gotdata = true;
      }
      else if (nTag.isTagA("pos3d"))
      { // obstacle avoid route points
        // first point is far away
        if (routeCnt < MAX_PASSABLE_LINE_SEGMENTS)
          nTag.getPosition(&route[routeCnt++]);
        //
        // debug print
        //printf("ULaserPathResult::setFromTag: got path %d ", routeCnt - 1);
        //route[routeCnt - 1].show(" ");
        // debug print end
        //
        gotdata = true;
      }
      else if (nTag.isTagA("pose"))
      { // obstacle avoid route tested mid-poses
        // in order of generation
        if (routeCnt < MAX_PASSABLE_LINE_SEGMENTS)
        { // save the pose in the route array of 3d positions
          nTag.getPose(&pose);
          route[routeCnt].x = pose.x;
          route[routeCnt].y = pose.y;
          route[routeCnt].z = pose.h;
          routeCnt++;
        }
        //
        // debug print
        //printf("ULaserPathResult::setFromTag: got path %d ", routeCnt - 1);
        //route[routeCnt - 1].show(" ");
        // debug print end
        //
        gotdata = true;
      }
      else if (nTag.isTagAnEnd(tag->getTagName()))
        // end tag is found - no more data
        break;
    } // while more sub-tags
  }
  //if (passLinesCnt != interv)
  //  printf("ULaserPathResult::setFromTag: expected %d intervals, got %d\n",
  //      interv, passLinesCnt);
  //
  return gotdata;
}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

bool UPlannerValue::setFromTag(USmlTag * tag,
                               UPoseTime * odoPose)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  unsigned long sec, usec;
/*
<get name="roadLeft" valid=true dist=0.88 qual=0.677/>
*/  //
  tag->reset();
  clear();
  if (tag->isTagA("pose") or tag->isTagA("poseTime"))
    type = VALTYP_POSE;
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "name") == 0)
      strncpy(name, val, MAX_VARIABLE_NAME_SIZE);
    else if (strcasecmp(att, "valid") == 0)
      // ia path actually used for control
      valid = str2bool(val);
    else if (strcasecmp(att, "typ") == 0)
    { // ia path actually used for control
      if (strcasecmp(val, "d") == 0)
        type = VALTYP_D;
      else if (strcasecmp(val, "ds") == 0)
        type = VALTYP_DS;
      else if (strcasecmp(val, "3d") == 0)
        type = VALTYP_3D;
      else if (strcasecmp(val, "time") == 0)
        type = VALTYP_TIME;
      else if (strcasecmp(val, "pose") == 0)
          type = VALTYP_POSE;
      else if (strcasecmp(val, "dq") == 0)
        type = VALTYP_DQ;
    }
    else if ((strcasecmp(att, "dist") == 0) or
             (strcasecmp(att, "width") == 0) or
             (strcasecmp(att, "value") == 0) or
              (strcasecmp(att, "x") == 0))
    {
      if (strcmp(name, "odoTime") == 0)
      {
        if (sscanf(val, "%lu.%lu", &sec, &usec) == 2)
          odoPose->t.setTime(sec, usec);
      }
      value[0] = strtod(val, NULL);
    }
    else if ((strcasecmp(att, "qual") == 0) or
              (strcasecmp(att, "y") == 0))
    {
      value[1] = strtod(val, NULL);
      if (type == VALTYP_D)
        type = VALTYP_DQ;
    }
    else if ((strcasecmp(att, "z") == 0) or
             (strcasecmp(att, "h") == 0) or
              (strcasecmp(att, "th") == 0))
      value[2] = strtod(val, NULL);
    // else ignore
  }
  if (strcmp(name, "odoPose") == 0)
    odoPose->set(value[0], value[1], value[2]);
  return (strlen(name) > 0);
}

////////////////////////////////////////////////////////////////////////

void UPlannerValue::clear()
{
  valid = false;
  name[0] = '\0';
  type = VALTYP_D;
}

////////////////////////////////////////////////////////////////////////

bool UPlannerValue::getAsString(char * buffer, int bufferLng)
{
  const int STL = 30;
  char st[STL];
  UTime t;
  bool result = true;
  //
  buffer[0] = '\0';
  if (strcasecmp(name, "time") == 0)
  {
    t.setTime(value[0]);
    t.getTimeAsString(st, true);
    snprintf(buffer, bufferLng, "%12s (%5s) %s",
            name, bool2str(valid), st);

  }
  else
    // ordinary format, so do-it-yourself
    result = false;
  //
  return result;
}

////////////////////////////////////////////////////////////////////////

bool UPlannerData::setVarFromTag(USmlTag * tag,
                                UPoseTime * odoPose)
{
  UPlannerValue pv;
  UPlannerValue * ppv;
  bool result;
  int i;
  //
  result = pv.setFromTag(tag, odoPose);
  if (result)
  { // test if known in advance
    ppv = vars;
    for (i = 0; i < varsCnt; i++)
    {
      if (strcmp(pv.getName(), ppv->getName()) == 0)
        break;
      ppv++;
    }
    if (i == varsCnt)
    { // unknown
      if (varsCnt < MAX_VARS_STORED)
        // room for more
        varsCnt++;
      else
        // store as first value
        ppv = vars;
    }
    // store value
    *ppv = pv;
  }
  return result;
}

///////////////////////////////////////////////////////

UPlannerValue * UPlannerData::findValue(const char * name)
{
  int i;
  UPlannerValue * result = NULL;
  UPlannerValue * pv;
  //
  pv = vars;
  for (i = 0; i < varsCnt; i++)
  {
    if (strcasecmp(name, pv->getName()) == 0)
    {
      result = pv;
      break;
    }
    pv++;
  }
  return result;
}

////////////////////////////////////////////////////////////

bool UPlannerData::findPose(const char * nameX, const char * nameY, const char * nameH, UPose * poseDest)
{
  bool result = true;
  UPlannerValue * val;
  UPose po;
  //
  val = findValue(nameX);
  result = (val != NULL);
  if (result)
  {
    po.x = val->getValue();
    val = findValue(nameY);
    result = (val != NULL);
  }
  if (result)
  {
    po.y = val->getValue();
    val = findValue(nameH);
    result = (val != NULL);
  }
  if (result)
    po.h = val->getValue();
  if (result and (poseDest != NULL))
    *poseDest = po;
  return result;
}


///////////////////////////////////////////////////////
