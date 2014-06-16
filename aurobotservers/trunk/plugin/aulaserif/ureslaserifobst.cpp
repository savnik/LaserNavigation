/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
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
#include <urob4/usmltag.h>

#include "ureslaserifobst.h"


// UResLaserIfObst::UResLaserIfObst()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   createVarSpace(20, 0, 2, "Laser scanner server obstacle detection results");
//   createBaseVar();
//   callDispOnNewData = true;
//   latestSerial = 0;
// }


UResLaserIfObst::~UResLaserIfObst()
{
  olog.closeLog();
}

///////////////////////////////////////////////////

// const char * UResLaserIfObst::name()
// {
//   return "Obstacle handler (2007 jca@oersted.dtu.dk)";
// }

///////////////////////////////////////////////////

const char * UResLaserIfObst::commandList()
{
  return "obst";
}

////////////////////////////////////////////////////

void UResLaserIfObst::createBaseVar()
{
    addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varSerial = addVar("serial", -1.0, "d", "(r) latest obstacle group serial number (from laser server)");
    varUpdate = addVar("update", -1.0, "d", "(r) Update number - increased when new obstacles arrive");
    varTime = addVar("time", -1.0, "t", "(r) time for last update");
    varGroups = addVar("groups", -1.0, "d", "(r) Number of obstacle groups");
    varOLog = addVar("log", 0.0, "d", "(r/w) Should logfile be open");
    //
    addMethod("obstacles", "ddd", "Get obstacle groups, first 'd' is requested number of dynamic obstacle groups, second d is number of fixed obstacle groups (0 or 1) third 'd' is lock request (1=locked groups) - must be unlocked by caller");
}

////////////////////////////////////////////////////

const char * UResLaserIfObst::snprint(const char * preString, char * buff, int buffCnt)
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
  snprintf(p1, buffCnt - n, "%s Holds %d obstacle groups, last update at %s\n",
           preString, groupsCnt, s);
  return buff;
}

///////////////////////////////////////////////////

const char * UResLaserIfObst::listGroup(int group, char * buff, int buffCnt)
{
  char * p1 = buff;
  int i, j, n = 0;
  UObstacleGroup * og = NULL;
  UObstacle * ob;
  UPosition pos1;
  //
  i = groupsNewest - group;
  if (group < 0)
    og = &fixeds;
  else
  { // regular obstacle group
    if (i < 0 and groupsCnt == MAX_OBST_GRPS)
      i += MAX_OBST_GRPS;
    if (i >= 0 and i < MAX_OBST_GRPS)
      og = &groups[i];
  }
  if (og != NULL)
  {
    og->lock();
    if (group >= 0)
      snprintf(p1, buffCnt - n, "Group %d of %d serial %lu has %d obstacles\n", group, groupsCnt, og->getSerial(), og->getObstsCnt());
    else
      snprintf(p1, buffCnt - n, "Group fixed  serial %lu has %d obstacles\n", og->getSerial(), og->getObstsCnt());
    for (j = 0; j < og->getObstsCnt(); j++)
    {
      n += strlen(p1);
      p1 = &buff[n];
      ob = og->getObstacle(j);
      pos1 = ob->getCogXY();
      snprintf(p1, buffCnt - n, "  - #%d has %d points at odo-COG %.2fx,%.2fy\n", j, ob->getPointsCnt(), pos1.x, pos1.y);
    }
    og->unlock();
  }
  else
    snprintf(p1, buffCnt - n, "Group %d of %d is not available\n", group, groupsCnt);
  return buff;
}

///////////////////////////////////////////////////

const char * UResLaserIfObst::listGroups(char * buff, int buffCnt)
{
  char * p1 = buff;
  int i, j, n = 0;
  UObstacleGroup * og = NULL;
  const int MSL = 30;
  char s[MSL];
  //
  lock();
  i = groupsNewest;
  snprintf(p1, buffCnt - n, "Obstacle group list has %d groups\n", groupsCnt);
  if (fixeds.getObstsCnt() > 0)
  {
    og = &fixeds;
    n += strlen(p1);
    p1 = &buff[n];
    og->getPoseLast().t.getDateTimeAsString(s, true);
    snprintf(p1, buffCnt - n, "Group fixed  serial %lu has %d obstacles (upd at %s)\n", og->getSerial(), og->getObstsCnt(), s);
  }
  for (j = 0; j < groupsCnt; j++)
  {
    og = &groups[i];
    if (og != NULL)
    {
      n += strlen(p1);
      p1 = &buff[n];
      og->getPoseLast().t.getDateTimeAsString(s, true);
      snprintf(p1, buffCnt - n, "Group %d of %d serial %lu has %d obstacles (upd at %s)\n", i, groupsCnt, og->getSerial(), og->getObstsCnt(), s);
    }
    i--;
    if (i < 0)
      i += MAX_OBST_GRPS;
  }
  unlock();
  return buff;
}

///////////////////////////////////////////////////

void UResLaserIfObst::handleNewData(USmlTag * tag)
{
  if (tag->isTagA("obst") )
    handleObst(tag);
  else
    printReply(tag, "UResLaserIfObst::handleNewData: not mine");
  msgHandled++;
}

//////////////////////////////////////////////////////////

void UResLaserIfObst::handleObst(USmlTag * tag)
{ // handle block with all obstacles in one update
  const int MSL = 200;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  USmlTag nTag;
//  bool gotTime = false;
  ULineSegment lineSeg;
  bool gotData = false;
  /* format sample:
  <obst cnt="2" tod="1148031884.497858">
    <obstGrp cnt="3" serial="11" update="true">
      <poset name="first">
        <pose x="1162.565" y="1201.157" h="0.822"/>
        <timeofday tod="1148031884.497858"/>
      </poset>
        <poset name="last">
        <pose x="1163.969" y="1202.649" h="0.833"/>
        <timeofday tod="1148031886.772080"/>
      </poset>
      <obst serial="0" noChange="true"/>
      <obst hits="8" valid="true" serial="1">
         .........
      </obst>
      <obst hits="4" valid="true" serial="4">
         ............
      </obst>
    </obstGrp>
    <obstGrp cnt="12" serial="10" update="true">
      .........
    </obstGrp>
  </obst>
  */
  if (tag->cnnVerbose())
    printf("%s ...\n", tag->getTagStart());
  if (varOLog->getBool() != olog.isOpen())
    olog.openLog(varOLog->getBool());
  //
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
//      gotTime = true;
    }
  }
  if (tag->isAStartTag())
  {
    if (ogLock.tryLock())
    {
      while (tag->getNextTag(&nTag, 200))
      {
        if (nTag.isTagA("obstGrp"))
        { //
          handleObstGrp(&nTag);
          gotData = true;
        }
        else if (nTag.isAStartTag())
          // is an unwanted tag goup - skip
          nTag.skipToEndTag(200);
        // remember to test for endflag too
        else if (nTag.isTagAnEnd(tag->getTagName()))
          break;
      }
      ogLock.unlock();
    }
    else
    {
      //printf("The obstacle resource is locked -- postponding update\n");
      if (tag->isAStartTag())
      tag->skipToEndTag(200);
    }
  }
/*  if (not gotTime)
    // use current time
    updTime.now();*/
  if (gotData)
  {
    varUpdate->add(1);
    varGroups->setInt(groupsCnt);
    varTime->setTime(updTime);
    varSerial->setInt(latestSerial, 0);
    newDataAvailable();
  }
}

//////////////////////////////////////////////////////////

void UResLaserIfObst::handleObstGrp(USmlTag * tag)
{ // handle pathGet data
  const int MVL = 100;
  char val[MVL];
  unsigned long grpSerial = 0;
  UObstacleGroup * og;
//  bool anUpdate = false;
  //
  /* format sample:
  <obstGrp cnt="3" serial="1" update="true">
    <poset name="first">
      <pose x="1162.565" y="1201.157" h="0.822"/>
      <timeofday tod="1148031884.497858"/>
    </poset>
    <poset name="last">
      <pose x="1163.969" y="1202.649" h="0.833"/>
      <timeofday tod="1148031886.772080"/>
    </poset>
    <obst serial="0" noChange="true"/>
    <obst hits="8" valid="true" serial="1">
      <poset name="first">
        <pose x="1162.565" y="1201.157" h="0.822"/>
        <timeofday tod="1148031884.497858"/>
      </poset>
      <poset name="last">
        <pose x="1163.969" y="1202.649" h="0.833"/>
        <timeofday tod="1148031886.772080"/>
      </poset>
      <polygon cnt="14">
        <pos3d x="1162.5180707271" y="1203.6425570087" z="0"/>
        <pos3d x="1163.3963005251" y="1203.7279421302" z="0"/>
        <pos3d x="1163.6512586026" y="1203.85665595" z="0"/>
        ........
        <pos3d x="1162.4201902918" y="1203.6486076783" z="0"/>
      </polygon>
    </obst>
    <obst hits="4" valid="true" serial="4">
      ...............
    </obst>
  </obstGrp>
  */
  // lock resource while unpacking
    if (tag->getAttValue("serial", val, MVL))
    {
      grpSerial = strtol(val, NULL, 10);
      if (grpSerial > latestSerial)
        latestSerial = grpSerial;
    }
//     if (tag->getAttValue("update", val, MVL))
//       anUpdate = str2bool2(val, true);
    if (tag->isAStartTag())
    { // find group to update (or create)
      if (grpSerial == 0)
        // serial zero is fixed obstacles
        // sets also serial number
        og = &fixeds;
      else
        og = getGroup(grpSerial, true);
      if (og != NULL)
      { // unpack group - an urob4 library function
        // group may be locked by obstacle avoidance
        if (og->tryLock())
        { // not used in a critical sense, so replace with new data
          //if (anUpdate)
          og->clear(); // does not clear serialnumber
          tag->getObstacleGroup(og, NULL);
          og->unlock();
          // debug
          //printf("UResLaserIfObst:: obstacle group %lu updated with %d/%d obstacles\n", grpSerial, og->getObstsCnt(), og->getMaxObstsCnt());
          // debug end
          if (olog.isLogOpen())
            og->logAll(grpSerial, olog.getF());
        }
        else
        { // group is in use, so wait until next update
          og = NULL;
          // debug
          printf("UResLaserIfObst:: found obstacle group %lu locked -- skip update of this group\n", grpSerial);
          // debug end
          if (olog.isLogOpen())
            olog.toLog("Obstacle group", grpSerial, "is locked - and thus not updated");
        }
      }
      if (og == NULL)
        // no space for data
        tag->skipToEndTag(200);
    }
}

//////////////////////////////////////////////////////////

void UResLaserIfObst::newDataAvailable()
{
  char * dataType = (char*) "laser";
  double v;
  bool isOK;
  //
  //setLocalVar(varScan, scan->getSerial());
  if (callDispOnNewData)
  {
    // tell display system of new image
    v = varSerial->getValued();
    isOK = callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    isOK = callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
    //printf("UResLaserIfObst::newDataAvailable: now %d obstacle group lines\n",
    //     groupsCnt);
    callDispOnNewData = isOK;
  }
}


//////////////////////////////////////////////////////

bool UResLaserIfObst::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  int i, n, g, nf;
  bool getLocked = false;
  UObstacleGroup * og;
  // evaluate standard functions
  if ((strcasecmp(name, "obstacles") == 0) and (strcasecmp(paramOrder, "ddd") == 0))
  { // lock resource or wait until unlocked
    ogLock.lock();
    n = roundi(pars[0]);
    nf = roundi(pars[1]);
    getLocked = roundi(pars[2]) != 0;
    if (nf != 0)
      nf = 1;
    if ((returnStructCnt != NULL) and (returnStruct != NULL))
    {
      if (n >  *returnStructCnt - nf)
        n = *returnStructCnt - nf;
      if (n > getGroupsCnt())
        n = getGroupsCnt();
      g = getGroupNewest();
      // debug
      //printf("UResLaserIfObst::methodCall: regular obst (max%d)", n);
      // debug end
      for (i = 0; i < n; i++)
      {
        og = &groups[g];
        if (getLocked)
          og->lock();
        returnStruct[i] = og;
        g--;
        if (g < 0)
          g = MAX_OBST_GRPS -  1;
        // debug
        //printf(" serial %lu has %d.", og->getSerial(), og->getObstsCnt());
        // debug end
      }
      // debug
      //printf("\n");
      //printf("UResLaserIfObst::methodCall: fixed obstacle cnt is %d, (obst grps = %d)\n", fixeds.getObstsCnt(), n);
      // debug end
      if (nf > 0)
      { // fixed obstacles too
        if (fixeds.getObstsCnt() > 0 and n > 0)
        { // fixed obstacles are available
          returnStruct[n] = &fixeds;
          n++;
        }
      }
      *returnStructCnt = n;
    }
    if (value != NULL)
      *value = n;
    ogLock.unlock();
  }
  else
    result = false;
  return result;
}
