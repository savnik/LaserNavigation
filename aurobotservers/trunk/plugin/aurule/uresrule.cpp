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

#include <stdio.h>
#include <math.h>

#include <urob4/uvarcalc.h>
#include <urob4/usmlfile.h>
#include <urob4/usmlstring.h>
#include <urob4/usmltag.h>

#include "uresrule.h"

//////////////////////////////////////////////////////

UMisLoadedRule::UMisLoadedRule()
{
  nextRule = NULL;
  sourceFileName[0] = '\0';
  busyCnt = 0;
  edit = true;
  editLines = NULL;
}

//////////////////////////////////////////////////////

UMisLoadedRule::~UMisLoadedRule()
{
  if (nextRule != NULL)
    delete nextRule;
  nextRule = NULL;
  if (editLines != NULL)
    free(editLines);
}

//////////////////////////////////////////////////////

bool UMisLoadedRule::unpack(USmlSource * cnn, USmlTag * tag, UVarCalc * calc)
{ // unpack a plan
  //UMisRule * newplan;
  bool result;
  //
  // set pointer to global varables and the calculator
  setCalc(calc);
  // let this unpack the plan itself
  result = unpackRule(cnn, tag);
  if (not result and cnn->isVerbose())
     printf("Rule not added (load failed)\n");
  return result;
}

//////////////////////////////////////////////////////

bool UMisLoadedRule::completeEdit(UVarCalc * calc)
{ // unpack a plan
  USmlString cnn;
  bool result = true;
  //
  if (editLines != NULL)
  { // set pointer to global varables and the calculator
    setCalc(calc);
    // set edit append mode
    inEditAppend = true;
    // set the source for adding more lines
    cnn.setSourceString(editLines);
    while (result and cnn.isSourceAvailable())
    { // unpack the plan itself
      // plan may be in more segments,
      result = unpackRule(&cnn, NULL);
    }
    if (result)
    { // remove source lines, only when successful
      free(editLines);
      editLines = NULL;
      edit = false;
    }
  }
  inEditAppend = false;
  return result;
}

//////////////////////////////////////////////////////

int UMisLoadedRule::setBusyCnt(int delta)
{
  busyCnt += delta;
  //printf("UMisLoadedRule::setBusyCnt: plan %s now %d!\n", name, busyCnt);
  if (busyCnt < 0)
  {
    printf("UMisLoadedRule::setBusyCnt: negative busy count!\n");
    busyCnt = 0;
  }
  return busyCnt;
}

//////////////////////////////////////////////////////

bool UMisLoadedRule::addEditLines(const char * lines)
{
  int n, m;
  bool result = true;
  //
  n = strlen(lines);
  if (n > 0)
  {
    if (editLines == NULL)
    {
      editLines=(char*)malloc(n+1);
      m = 0;
    }
    else
    {
      m = strlen(editLines);
      editLines=(char*)realloc(editLines, m + n + 1);
    }
    result = editLines != NULL;
    if (result)
    { // copy de-escaped lines to edit buffer
      stripEscape(&editLines[m], lines);
    }
  }
  return result;
}

//////////////////////////////////////////////////////

void UMisLoadedRule::stripEscape(char * dest, const char * source)
{
  char * dp;
  const char * sp;
  //
  dp = dest;
  sp = source;
  while (*sp != '\0')
  {
    if (*sp == '\\')
    {
      sp++;
      switch (*sp)
      {
        case 'n': *dp++ = '\n'; break;
        case '\\': // allow multible backslash before escape, and keep just one
          sp--; break;
        default : // add both
          *dp++ = '\\';
          *dp++ = *sp;
          break;
      }
    }
    else
      *dp++ = *sp;
    sp++;
  }
  *dp++ = '\n';
  *dp = '\0';
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

UResRule::~UResRule()
{
  if (planRoot != NULL)
    delete planRoot;
}

///////////////////////////////////////////

void UResRule::createBaseVar()
{
  varMisCount = addVar("count", 1, "d", "Number of loaded rules");
}

///////////////////////////////////////////

const char * UResRule::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  snprintf(buff, buffCnt, "%s Rule database has %d loaded mission\n",
           preString, getMissionCnt());
  return buff;
}

////////////////////////////////////////

bool UResRule::loadFile(const char * name)
{
  bool result = false;
  USmlFile cnn;
  USmlTag tag;
  //
  cnn.setVerbose(true);
  cnn.allowTagAtStartOfLineOnly();
  result = cnn.openSmlFile(name);
  synErrBuf[0] = '\0';
  cnn.setErrorBuffer(synErrBuf, SYN_ERR_BUF_SIZE);
  if (result)
  {
    while (cnn.getNextTag(&tag))
    {
      if (tag.isAPhonyTag())
      { // just ignore phony tags
      }
      else if (tag.isTagA("rule"))
      { // tag is a plan, so create a new mission plan
        // as appropriate
        result = unpackMissionRule(&cnn, &tag, name);
        if (result)
          varMisCount->add(1.0, 0);
      }
      else if (tag.isAStartTag())
      { // ignore any other tags
        cnn.skipToEndTag(&tag);
      }
    }
    toLog("opened", name);
    cnn.closeSmlFile();
  }
  //
  return result;
}

/////////////////////////////////////////

int UResRule::getMissionCnt()
{
  int result = 0;
  UMisLoadedRule * plan = planRoot;
  //
  while (plan != NULL)
  {
    result++;
    plan = plan->nextRule;
  }
  return result;
}

////////////////////////////////////////

bool UResRule::unpackMissionRule(USmlSource * cnn, USmlTag * tag, const char * fileName)
{
  USmlTag mTag;
  UMisLoadedRule * plan;
  const int MNL = 100;
  char ruleName[MNL];
  bool result;
  //
  result = tag->getAttValue("name", ruleName, MNL);
  if (result)
  {
    plan = getRule(ruleName, false);
    if (plan != NULL)
    {
      if (cnn->isVerbose())
        printf("Rule %s is loaded already, unload first\n", ruleName);
      cnn->skipToEndTag(tag);
    }
    else
    { // make new plan
      plan = new UMisLoadedRule();
      plan->setLogFile(this);
      result = plan->unpack(cnn, tag, getVarPool());
      if (result)
      { // save the source name
        strncpy(plan->sourceFileName, fileName, MAX_FILENAME_LENGTH);
        plan->isTop = true;
        plan->setEdit(false);
        appendRule(plan);
/*        if (plan->isDefRule())
          strncpy(def, plan->getRuleName(), MAX_VARIABLE_NAME_SIZE);*/
      }
      else
        delete plan;
    }
  }
  else
  { // a plan with no name can not be used
    if (cnn->isVerbose())
      printf("UResRule::unpackMissionRule: failed, plan has no name\n");
    cnn->skipToEndTag(tag);
  }
  return result;
}

/////////////////////////////////////////////

UMisLoadedRule * UResRule::getRule(const char * name, bool runableRulesOnly)
{
  UMisLoadedRule * plan = planRoot;
  //
  lock();
  while (plan != NULL)
  {
    if (not (runableRulesOnly and plan->isEdit()))
    {
      if (strcasecmp(plan->getName(), name) == 0)
        break;
    }
    plan = plan->nextRule;
  }
  unlock();
  return plan;
}

/////////////////////////////////////////////

UMisLoadedRule * UResRule::getRule(const int idx)
{
  UMisLoadedRule * plan = planRoot;
  int i = 0;
  //
  lock();
  while (plan != NULL and i != idx)
  {
    plan = plan->nextRule;
    i++;
  }
  unlock();
  return plan;
}

///////////////////////////////////////////////

bool UResRule::tryUnloadRule(const char * name)
{
  bool result = false;
//  UMisLoadedRule * plan = planRoot;
  //
  lock();
  UMisLoadedRule ** p = &planRoot;
  UMisLoadedRule *dp = NULL;
  //
  while (*p != NULL)
  {
    if (strcasecmp((*p)->getName(), name) == 0)
    {
      result = not (*p)->isBusy();
      if (result)
      {
        dp = *p;
        // change link to next rather than this plan
        *p = dp->nextRule;
        // make plan to the dele a single plan (an orphant)
        dp->nextRule = NULL;
        // delete plan - the caller must secure that the plan is not busy
        delete dp;
        // decrease mission count
        varMisCount->add(-1.0, 0);
        //
        toLog("unloaded", name);
      }
      break;
    }
    else
      // advance to (adress of) next pointer
      p = &(*p)->nextRule;
  }
  unlock();
  return result;
}

///////////////////////////////////////////////

bool UResRule::getRuleList(const char * name, char * buf, const int bufCnt)
{
  UMisLoadedRule * plan = planRoot;
  char * p1 = buf;
  int n = 0;
  bool result = true;
  //
  plan = getRule(name, false);
  if (plan != NULL)
  {
/*    snprintf(p1, bufCnt - n, "; %s (run=%s rule=%s) file: %s\n",
             plan->getRuleName(), bool2str(plan->isRun), bool2str(plan->isRule), plan->sourceFileName);
    n += strlen(p1);
    p1 = &buf[n];*/
    plan->print("", p1, bufCnt - n);
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////

void UResRule::appendRule(UMisLoadedRule * plan)
{
  UMisLoadedRule ** p = &planRoot;
  //
  while (*p != NULL)
    p = &(*p)->nextRule;
  *p = plan;
}

////////////////////////////////////////////////

UMisLoadedRule * UResRule::addNewRule(const char * name)
{
  UMisLoadedRule * newRule;
  //
  newRule = new UMisLoadedRule();
  if (newRule != NULL)
  {
    newRule->setEdit(true);
    newRule->setName(name);
    newRule->isFull = true;
    newRule->isTop = true;
    appendRule(newRule);
  }
  return newRule;
}

////////////////////////////////////////////////

bool UResRule::completeEdit(UMisLoadedRule * plan)
{
  bool result = false;
  //
  if (plan != NULL)
  {
    plan->setLogFile(this);
    result = plan->completeEdit(getVarPool());
  }
  return result;
}
