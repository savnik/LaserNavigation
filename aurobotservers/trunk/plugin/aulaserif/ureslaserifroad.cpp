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

#include <urob4/usmltag.h>
#include <urob4/uvarcalc.h>

#include "ureslaserifroad.h"

///////////////////////////////////////////////////

URoadLineData::URoadLineData()
{
  clear();
}

///////////////////////////////////////////////////

void URoadLineData::clear()
{
  edge = -1;
  updateCnt = 0;
  edgeLine.clear();
  scanSerial = 0;
  lineSerial = 0;
}

///////////////////////////////////////////////////

void URoadLineData::update(const int type, const unsigned long serial,
                           const double lineQ, ULineSegment * seg,
                           unsigned long scan)
{
  double d;
  const double MIN_UPDATE_DIST = 0.01; // 1 centimeter
  //
  if (updateCnt == 0)
  { // new line, so set static data too
    edge = type;
    edgeLine.setAsPolyline();
    lineSerial = serial;
  }
  line = *seg;
  if (updateCnt > 0)
    d = edgeLine.getLastPoint().dist(line.pos);
  else
    d = 1.0;
  if (d > MIN_UPDATE_DIST)
  { // add the new position
    // test if space in polyline for an extra position
    // this part is critical for users of the line, so lock
    if (edgeLine.tryLock())
    {
      if (edgeLine.getPointsCnt() >= edgeLine.getPointsMax())
      { // remove the oldest position
        edgeLine.remove(0);
      }
      edgeLine.add(line.pos);
      edgeLine.unlock();
    }
  }
  updateCnt++;
  lineQuality = lineQ;
  scanSerial = scan;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

void UResLaserIfRoad::UResLaserIfRoadInit()
{
  int i;
  //
/*  setResID( getResID());
  resVersion = getResVersion();*/
  createVarSpace(20, 0, 2, "Laser scanner server road detection interface", false);
  createBaseVar();
  updTime.now();
  roadLinesCnt = 0;
  for (i = 0; i < MAX_ROAD_LINE_CNT; i++)
    roadLines[i] = NULL;
  scanSerial = 0;
  callDispOnNewData = true;
  newestLeft = NULL;
  newestRight = NULL;
  newestTop = NULL;
}

///////////////////////////////////////////////////

UResLaserIfRoad::~UResLaserIfRoad()
{
  int i;
  //
  for (i = 0; i < roadLinesCnt; i++)
  {
    if (roadLines[i] != NULL)
      delete roadLines[i];
  }
}

///////////////////////////////////////////////////

// const char * UResLaserIfRoad::name()
// {
//   return "roadLines (2007 jca@oersted.dtu.dk)";
// }

///////////////////////////////////////////////////

const char * UResLaserIfRoad::commandList()
{
  return "road";
}

///////////////////////////////////////////////////

bool UResLaserIfRoad::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  result &= UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResLaserIfRoad::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  //
  result = UResVarPool::setResource(resource, remove);
  return result;
}

///////////////////////////////////////////

void UResLaserIfRoad::createBaseVar()
{
  UVarPool * vp;
  const char * roadN = "Number of updates";
  const char * roadQ = "Quality of road line";
  const char * roadGetSeg = "Get line segment for road line";
  const char * roadGetLine = "Get polyline for road line";
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varLN = vp->addVar("leftN", 0.0, "d", roadN);
    varLQ = vp->addVar("leftQ", 0.0, "d", roadQ);
    varCN = vp->addVar("centerN", 0.0, "d", roadN);
    varCQ = vp->addVar("centerQ", 0.0, "d", roadQ);
    varRN = vp->addVar("rightN", 0.0, "d", roadN);
    varRQ = vp->addVar("rightQ", 0.0, "d", roadQ);
    varUpdateTime = vp->addVar("updateTime", 0.0, "d", roadQ);
    vp->addMethod(this, "leftSeg", "", roadGetSeg);
    vp->addMethod(this, "centerSeg", "", roadGetSeg);
    vp->addMethod(this, "rightSeg", "", roadGetSeg);
    vp->addMethod(this, "leftLine", "", roadGetLine);
    vp->addMethod(this, "centerLine", "", roadGetLine);
    vp->addMethod(this, "rightLine", "", roadGetLine);
  }
}

////////////////////////////////////////////////////

const char * UResLaserIfRoad::snprint(const char * preString, char * buff, int buffCnt)
{
  const int MSL = 30;
  char s[MSL];
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt - n, "%s Handles interface tags: %s\n",
           preString, commandList());
  n = strlen(p1);
  p1 = &buff[n];
  updTime.getTimeAsString(s, true);
  snprintf(p1, buffCnt - n, "%s holds %d roadLines, last update at %s\n",
          preString, roadLinesCnt, s);
  return buff;
}

///////////////////////////////////////////////////

void UResLaserIfRoad::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("road") )
    handleRoad(tag);
  else
    printReply(tag, "UResLaserIfRoad::handleNewData: not mine");
  msgHandled++;
}

///////////////////////////////////////////////////

void UResLaserIfRoad::handleRoad(USmlTag * tag)
{ // handle pathGet data
  const int MSL = 200;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  USmlTag nTag;
  bool gotTime = false;
  bool gotSerial = false;
  bool bestLine;
  unsigned long lineSerial = 0;
  unsigned long scan = 0;
  double lineQ = 0.0;
  int lineType = -1;
  ULineSegment lineSeg;
  bool gotData = false;
  /* format sample:
  <road scan="6" tod="1148031885.456355" updLim="3" qualLim="0" bestOnly="false">
  <lineSeg name="left" length="0.78317335538665" q="0.27995" n="4" valid="true"
      best="true" idx="0" scan="6" pis="1" serial="0">
  <pos3d name="start" x="1163.7788699898" y="1203.8938879495" z="0"/>
  <pos3d name="vector" x="0.67894213065579" y="0.73419178912637" z="0"/>
  </lineSeg>
  <lineSeg name="center" length="0.82714918245615" q="0.27709" n="4" valid="true"
      best="true" idx="1" scan="6" pis="1" serial="1">
  <pos3d name="start" x="1165.5092388966" y="1202.438781918" z="0"/>
  <pos3d name="vector" x="0.69707693787456" y="0.71699633380055" z="0"/>
  </lineSeg>
  <lineSeg name="right" length="0.88159842520556" q="0.25772" n="4" valid="true"
      best="true" idx="2" scan="6" pis="1" serial="2">
  <pos3d name="start" x="1167.5098390544" y="1201.4232155629" z="0"/>
  <pos3d name="vector" x="0.70184546564037" y="0.71232923733344" z="0"/>
  </lineSeg>
  </road>
  */
  while (tag->getNextAttribute(att, val, MSL))
  {
    if ((strcasecmp(att, "warning") == 0) or
         (strcasecmp(att, "info") == 0))
    {
      // no data printf("*** failed: %s\n", val);
      break;
    }
    else if (strcasecmp(att, "tod") == 0)
    {
      updTime.setTimeTod(val);
      gotTime = true;
    }
    else if (strcasecmp(att, "scan") == 0)
    {
      scanSerial = strtol(val, NULL, 10);
      gotSerial = true;
    }
  }
  if (gotSerial and gotTime and tag->isAStartTag())
  {
    while (tag->getNextTag(&nTag, 200))
    {
      if (nTag.isTagA("lineSeg"))
      { /* get specific header data
        <lineSeg name="right" length="0.88159842520556" q="0.25772" n="4" valid="true"
            best="true" idx="2" scan="6" pis="1" serial="2">
            <pos3d name="start" x="1167.5098390544" y="1201.4232155629" z="0"/>
            <pos3d name="vector" x="0.70184546564037" y="0.71232923733344" z="0"/>
            </lineSeg> */
        if (nTag.getAttValue("name", val, MSL))
        { // get line type
          if (strcmp(val, "left") == 0)
            lineType = 0;
          else if (strcmp(val, "center") == 0)
            lineType = 1;
          else if (strcmp(val, "right") == 0)
            lineType = 2;
          else
            lineType = -1;
        }
        if (nTag.getAttValue("q", val, MSL))
          lineQ = strtod(val, NULL);
        if (nTag.getAttValue("scan", val, MSL))
          scan = strtol(val, NULL, 10);
        if (nTag.getAttValue("serial", val, MSL))
          lineSerial = strtol(val, NULL, 10);
        if (nTag.getAttValue("best", val, MSL))
          bestLine = str2bool(val);
        else
          bestLine = false;
        if (bestLine)
        {
          nTag.getLineSegment(&lineSeg);
          // debug
          //lineSeg.print( "debug line from laser server");
          // debug end
          updateLine(lineType, lineSerial, lineQ, &lineSeg, scan);
        }
        else
          nTag.skipToEndTag(200);
        gotData |= (lineType >= 0 and bestLine);
      }
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        nTag.skipToEndTag(200);
      // remember to test for endflag too
      else if (nTag.isTagAnEnd(tag->getTagName()))
        break;
    }
  }
  if (gotData)
    newDataAvailable();
  if (gotData)
  {
    varUpdateTime->setTime(updTime);
  }
}

//////////////////////////////////////////////////////////

void UResLaserIfRoad::newDataAvailable()
{
  char * dataType = (char*)"laser";
  double v;
  bool isOK;
  //
  //setLocalVar(varScan, scan->getSerial());
  // tell display system of new image
  if (callDispOnNewData)
  {
    v = scanSerial;
    isOK = callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
    //printf("UResLaserIfRoad::newDataAvailable: Now %d road lines\n", roadLinesCnt);
    callDispOnNewData = isOK;
  }
  // update
}

//////////////////////////////////////////////////////////

void UResLaserIfRoad::updateLine(const int lineType,
                const unsigned long lineSerial,
                const double lineQ, ULineSegment * lineSeg,
                const unsigned long scan)
{
  int i;
  URoadLineData ** rl;
  int oldest = 0;
  unsigned long scanOldest = 0;
  //
  rl = roadLines;
  if (roadLinesCnt > 0)
    scanOldest = (*rl)->scanSerial;
  for (i = 0; i < roadLinesCnt; i++)
  {
    if ((*rl)->scanSerial < scanOldest)
    { // find oldest line if relevant to replace
      scanOldest = (*rl)->scanSerial;
      oldest = i;
    }
    if ((*rl)->lineSerial == lineSerial)
      // line to update is found;
      break;
    rl++;
  }
  if (i >= MAX_ROAD_LINE_CNT)
  { // line not found an all lines are used
    rl = &roadLines[oldest];
    (*rl)->clear();
  }
  else if ((*rl) == NULL)
  { // new road line
    *rl = new URoadLineData();
    roadLinesCnt++;
  }
  else
    // reuse after clear of data, so just clear
    (*rl)->clear();
  if ((*rl) != NULL)
  { // update the line with newest data
    if ((*rl)->scanSerial != scan)
    {
      (*rl)->update(lineType, lineSerial, lineQ, lineSeg, scan);
      //setLocalVar(varUpdateTime, updTime.getDecSec());
      switch (lineType)
      {
        case 0:
          newestLeft = *rl;
          varLN->setInt(newestLeft->updateCnt, 0);
          varLN->setDouble(lineQ, 1);
          varLQ->setDouble(lineQ, 0);
          break;
        case 1:
          newestTop = *rl;
          varCN->setInt(newestTop->updateCnt, 0);
          varCN->setDouble(lineQ, 1);
          varCQ->setDouble(lineQ, 0);
          break;
        case 2:
          newestRight = *rl;
          varRN->setInt(newestRight->updateCnt, 0);
          varRN->setDouble(lineQ, 1);
          varRQ->setDouble(lineQ, 0);
          break;
        default:
          break;
      }
    }
  }
}


//////////////////////////////////////////////////////

bool UResLaserIfRoad::methodCall(const char * name, const char * paramOrder,
                               char ** strings, const double * pars,
                               double * value,
                               UDataBase ** returnStruct,
                               int * returnStructCnt)
{
  bool isSeg = true, isLine = false;
  URoadLineData * rl = NULL;
  // evaluate standard functions
/*  vp->addMethod(this, "leftSeg", "");
  vp->addMethod(this, "centerSeg", "");
  vp->addMethod(this, "rightSeg", "");*/
  if ((strcasecmp(name, "leftSeg") == 0) and (strlen(paramOrder) == 0))
    rl = newestLeft;
  else if ((strcasecmp(name, "centerSeg") == 0) and (strlen(paramOrder) == 0))
    rl = newestTop;
  else if ((strcasecmp(name, "rightSeg") == 0) and (strlen(paramOrder) == 0))
    rl = newestRight;
  else
    isSeg = false;
  //
  if (rl != NULL)
  { // requestr for a line segment
    if (value != NULL)
      *value = rl->lineQuality;
    if (returnStruct != NULL and returnStructCnt != NULL)
    {
      returnStruct[0] = &rl->line;
      *returnStructCnt = 1;
    }
  }
  else if (isSeg and (value != NULL))
    *value = 0.0;

  if (not isSeg)
  {
    isLine = true;
    if ((strcasecmp(name, "leftLine") == 0) and (strlen(paramOrder) == 0))
      rl = newestLeft;
    else if ((strcasecmp(name, "centerLine") == 0) and (strlen(paramOrder) == 0))
      rl = newestTop;
    else if ((strcasecmp(name, "rightLine") == 0) and (strlen(paramOrder) == 0))
      rl = newestRight;
    else
      isLine = false;
    if (rl != NULL)
    { // request for a polyline
      if (value != NULL)
        *value = rl->lineQuality;
      if (returnStruct != NULL and returnStructCnt != NULL)
      {
        returnStruct[0] = &rl->edgeLine;
        *returnStructCnt = 1;
      }
    }
    else if (isLine and (value != NULL))
      *value = 0.0;
  }
  if (not isLine and not isSeg)
  {
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
    if (value != NULL)
      *value = 0.0;
  }
  return isSeg or isLine;
}

/////////////////////////////////////////////////////////

URoadLineData * UResLaserIfRoad::getRoadCurrent(int side)
{
  URoadLineData * result;
  switch (side)
  {
    case 0: result = newestLeft; break;
    case 1: result = newestTop; break;
    case 2: result = newestRight; break;
    default: result = NULL;
  }
  return result;
}
