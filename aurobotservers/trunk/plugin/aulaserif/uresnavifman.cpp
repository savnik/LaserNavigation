/***************************************************************************
 *   Copyright (C) 2007 by Christian Andersen   *
 *   jca@oersted.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>
#include <urob4/usmltag.h>

#include "uresnavifman.h"



bool UClientManSeq::setFromTag(USmlTag * tag)
{
  const int MVL = 30; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  //int interv = 0;
  bool result = false;
  USmlTag nTag;
  UPose pose;
  bool gotdata = false;
  //int manCnt;
  /*
  <path intervals="2" pathUsed="true" tod="1149865866.441293" aCrash="false">
  <manseq cnt="4" endV="1.200" dt="3.442">
  <posev name="start" x="369.543" y="57.896" h="-0.247" v="0.807"/>
  <posev name="end" x="372.228" y="57.050" h="-0.371" v="1.200"/>
  <manoeuvre typ="line" dist="0.258" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.06495" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="line" dist="1.96" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.05921" acc="0.3" endVel="1.2"/>
  </manseq>
  <manseq cnt="3" endV="1.200" dt="4.384">
  <posev name="start" x="372.228" y="57.050" h="-0.371" v="1.200"/>
  <posev name="end" x="377.056" y="54.971" h="-0.247" v="1.200"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.05056" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="line" dist="4.18" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="0.1747" acc="0.3" endVel="1.2"/>
  </manseq>
  </path>
  */
  pathUsed = false;
  isACrash = false;
  //manCnt = 0;
  posesCnt = 0;
  segsCnt = 0;
  releaseAllMan();
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
    else if (strcasecmp(att, "aCrash") == 0)
      // path could not be completed to destination pose
      isACrash = str2bool(val);
    else if (strcasecmp(att, "tod") == 0)
      // path could not be completed to destination pose
      updTime.setTimeTod(val);
    // edge valid is set when edge is decoded
  }
  if (result and tag->isAStartTag())
  {
    posesCnt = 0;
    while (result)
    { // continue until endTag is found
      result = tag->getNextTag(&nTag, 400);
      if (not result)
        break;
      if (nTag.isTagA("manseq"))
      {
        nTag.getManSeq(this);
        gotdata = true;
      }
      else if (nTag.isTagA("pose"))
      { // obstacle avoid route tested mid-poses
        // in order of generation
        if (posesCnt < MAX_TESTED_POSES)
        { // save the pose in the route array of 3d positions
          nTag.getPose(&pose);
          poses[posesCnt] = pose;
          posesCnt++;
        }
        //
        // debug print
        //printf("ULaserPathResult::setFromTag: got pose %d ", posesCnt);
        //route[routeCnt - 1].show(" ");
        // debug print end
        //
        gotdata = true;
      }
      else if (nTag.isTagA("lineSeg"))
      { // obstacle avoid route tested mid-poses
        // in order of generation
        nTag.getAttValue("name", val, MVL);
        if (segsCnt < MAX_TESTED_LINES)
        { // store first character of segment name in
          // segment character array
          // e.g. 't' for tangent line and 'n' for no visibility line
          segsChar[segsCnt] = val[0];
          // save the line segment associated to the path
          nTag.getLineSegment(&segs[segsCnt++]);
        }
        else
          nTag.skipToEndTag(400);
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


void UResNavIfMan::UResNavIfManInit()
{
/*  setResID( getResID());
  resVersion = getResVersion();*/
  //
  createVarSpace(20, 0, 2, "Navigation server manoeuvre interface", false);
  createBaseVar();
  //
  for (mansCnt = 0; mansCnt < MAX_STORED_MANS; mansCnt++)
    mans[mansCnt] = NULL;
  mansCnt = 0;
  callDispOnNewData = true;
}

/////////////////////////////////////////

UResNavIfMan::~UResNavIfMan()
{
  int i;
  //
  mansCnt = 0;
  for (i = 0; i < MAX_STORED_MANS; i++)
    if (mans[i] != NULL)
      delete mans[i];
    else
      // no more to  delete
      break;
}

//////////////////////////////////////////

const char * UResNavIfMan::name()
{
  return "Takes manoeuvre sequence to 'navMan' resource";
}

//////////////////////////////////////////

const char * UResNavIfMan::commandList()
{
  return "avoid pathGet roaddrive drivePos pp";
}


///////////////////////////////////////////

void UResNavIfMan::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "(ro) Resource version");
    varMansCnt =  vp->addVar("manCnt", 0.0, "d", "(ro) Number of available manoeuvres");
    varManTime = vp->addVar("Time", 0.0, "d", "(ro) Latest update time");
    //vp->addMethod(this, "rightLine", "", roadGetLine);
  }
}

//////////////////////////////////////////////////////////////////

bool UResNavIfMan::setResource(UResBase * resource, bool remove)
{
  bool result = true;
/*  if (resource->isA(UResPoseHist::getResID()))
  { // delete any local
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  else
    result = false;*/
  //
  result |= UResVarPool::setResource(resource, remove);
  //
  return result;
}

/////////////////////////////////////////////////////

bool UResNavIfMan::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
/*  if (poseHist == NULL)
  {
    strncpy(p1, UResPoseHist::getResID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }*/
  result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

/////////////////////////////////////////

const char * UResNavIfMan::snprint(const char * preString, char * buff, int buffCnt)
{
  int m = 0;
  char * p1;
  const int MSL = 30;
  char s[MSL];
  UTime t;
  //
  if (buff != NULL)
  {
    p1 = buff;
    t = varManTime->getTime();
    t.getTimeAsString(s, true);
    snprintf(p1, buffCnt - m, "%s navigation manoeuvres.,holds %d sequences, updated %s\n",
             preString, mansCnt, s);
  }
  return buff;
}

/////////////////////////////////////////

void UResNavIfMan::handleNewData(USmlTag * tag)
{ // print if desired
  bool isInfo;
  const int MVL = 32; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  // het first attribute
  tag->getNextAttribute(att, val, MVL);
  tag->reset();
  isInfo = (strcmp(att, "warning") == 0) or
      (strcmp(att, "info") == 0) or
      (strcmp(att, "debug") == 0) or
      (strcmp(att, "error") == 0);
  if (verboseMessages)
    tag->print("");
  else if (isInfo)
    tag->print("");
  // distribute to sub-functions
  if (tag->isTagA("avoid"))
    handleManData(tag);
  else if (tag->isTagA("pathGet"))
    handleManData(tag);
  else if (tag->isTagA("roaddrive"))
    handleManData(tag);
  else if (tag->isTagA("drivepos"))
    handleManData(tag);
  else if (tag->isTagA("pp"))
    handleManData(tag);
  else
    printReply(tag, "UClientFuncLaser::handleNewData: not mine");
}

//////////////////////////////////////////////

  /*  <avoid cnt="1">
      <path intervals="2" pathUsed="true" tod="1149865866.441293" aCrash="false">
      <manseq cnt="4" endV="1.200" dt="3.442">
      <posev name="start" x="369.543" y="57.896" h="-0.247" v="0.807"/>
      <posev name="end" x="372.228" y="57.050" h="-0.371" v="1.200"/>
      <manoeuvre typ="line" dist="0.258" acc="0.3" endVel="1.2"/>
      <manoeuvre typ="arc" rad="4.8" arc="-0.06495" acc="0.3" endVel="1.2"/>
      <manoeuvre typ="line" dist="1.96" acc="0.3" endVel="1.2"/>
      <manoeuvre typ="arc" rad="4.8" arc="-0.05921" acc="0.3" endVel="1.2"/>
      </manseq>
      <manseq cnt="3" endV="1.200" dt="4.384">
      <posev name="start" x="372.228" y="57.050" h="-0.371" v="1.200"/>
      <posev name="end" x="377.056" y="54.971" h="-0.247" v="1.200"/>
      <manoeuvre typ="arc" rad="4.8" arc="-0.05056" acc="0.3" endVel="1.2"/>
      <manoeuvre typ="line" dist="4.18" acc="0.3" endVel="1.2"/>
      <manoeuvre typ="arc" rad="4.8" arc="0.1747" acc="0.3" endVel="1.2"/>
      </manseq>
      </path>
      </avoid>*/

bool UResNavIfMan::handleManData(USmlTag * tag)
{
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag nTag;
  int cnt = 0;
  bool gotData = false;
  UTime t;
  UPoseV pv;
  bool setPose;
/*
  <pathget pathCnt=2>
  <path intervals="2" pathUsed="true" tod="1149865866.441293" aCrash="false">
  <manseq cnt="4" endV="1.200" dt="3.442">
  <posev name="start" x="369.543" y="57.896" h="-0.247" v="0.807"/>
  <posev name="end" x="372.228" y="57.050" h="-0.371" v="1.200"/>
  <manoeuvre typ="line" dist="0.258" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.06495" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="line" dist="1.96" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.05921" acc="0.3" endVel="1.2"/>
  </manseq>
  <manseq cnt="3" endV="1.200" dt="4.384">
  <posev name="start" x="372.228" y="57.050" h="-0.371" v="1.200"/>
  <posev name="end" x="377.056" y="54.971" h="-0.247" v="1.200"/>
  <manoeuvre typ="arc" rad="4.8" arc="-0.05056" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="line" dist="4.18" acc="0.3" endVel="1.2"/>
  <manoeuvre typ="arc" rad="4.8" arc="0.1747" acc="0.3" endVel="1.2"/>
  </manseq>
  </path>
  <path intervals ...
  ...
  </path>
  </pathget>
*/
  mansCnt = 0;
  setPose = false;
  manBest = -1;
  t.clear();
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "pathCnt") == 0)
      cnt = strtol(val, NULL, 0);
    if (strcasecmp(att, "cnt") == 0)
      cnt = strtol(val, NULL, 0);
  }
  if (tag->isAStartTag())
  {
    while (result)
    {
      result = tag->getNextTag(&nTag, 400);
      if (result and nTag.isTagA("path"))
      { // a path is a sequence of manoeuvres
        if (nTag.getAttValue("tod", val, MVL) and not setPose)
          t.setTimeTod(val); // timestamp
        // create entry if not available from last update
        if (mans[mansCnt] == NULL)
          mans[mansCnt] = new UClientManSeq();
        result = (mans[mansCnt] != NULL);
        // unpack path structure
        if (result and mans[mansCnt]->setFromTag(&nTag))
        { // unpacked OK
          gotData = true;
          if (mans[mansCnt]->isBest())
            manBest = mansCnt;
          if ((mansCnt < MAX_STORED_MANS))
            mansCnt++;
        }
      }
      else if (nTag.isTagAnEnd(tag->getTagName()))
        // end of all paths for this time.
        break;
    }
  }
  if ((cnt != mansCnt) and verboseMessages)
    // @todo some paths are counter, but not valid and not send
    printf("UClientFuncLaser::handlePath: expected %d paths got %d\n",
           cnt, mansCnt);
  // announce the new data
  if (gotData)
    gotNewData();
  return result;
}

///////////////////////////////////////////////////////

void UResNavIfMan::gotNewData()
{
  char * dataType = (char*)"nav";
  double v;
  bool isOK;
  UTime t;
  //
  if (callDispOnNewData)
  { //setLocalVar(varScan, scan->getSerial());
    // tell display system of new image
    t.now();
    v = t.getDecSec();;
    isOK  = callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    isOK |= callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
    // is no display to call, then stop dooing so
    callDispOnNewData = isOK;
  }
}
