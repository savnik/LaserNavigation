/***************************************************************************
 *   Copyright (C) 2007-2008 by DTU (Christian Andersen)                   *
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

#include "ufuncrule.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncRule();
}
//
#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncRule::~UFuncRule()
{ // possibly remove allocated variables here - if needed
  if (seq  != NULL)
    delete seq;
  if (rules != NULL)
    delete rules;
  //if (resDum != NULL)
  //  delete resDum;
}

///////////////////////////////////////////////////

void UFuncRule::createResources()
{
  seq = new UResRuleState();
  addResource(seq, this);
  rules = new UResRule();
  addResource(rules, this);
}

///////////////////////////////////////////////////

bool UFuncRule::handleCommand(UServerInMsg * msg, void * extra)
{  // message is for this plugin
  return handleRuleSet(msg);
}


/////////////////////////////////////////////////////////////

bool UFuncRule::handleRuleSet(UServerInMsg * msg)
{ // extract parameters
  bool result = true;
  // decode vars
  char att[MAX_SML_NAME_LENGTH];
  const int VBL = 2000;
  char val[VBL];
  // camera and source image variables
  bool ask4help = false;
  bool gotStatus = false;
  bool sendReply = false;
  const int MNL = MAX_FILENAME_LENGTH;
  const int MRL = 300;
  char reply[MRL];
  bool gotLoad = false;
  char gotLoadName[VBL];
  bool gotUnload = false;
  char gotUnloadName[VBL];
  //bool gotClearAll = false;
  bool gotAll = false;
  bool gotList = false;
  bool gotState = false;
  char gotListName[VBL];
//  bool gotDef = false;
//  char gotDefName[VBL];
  bool gotEnable = false;
  bool gotDisable = false;
  const int MPL = 33;
  char gotRuleName[MPL] = "";
  bool gotResume = false;
  bool gotStop = false;
  bool gotFile = false;
  char gotFileName[VBL];
//  bool gotPause = false;
//  bool gotPauseValue = true;
  bool gotStep = false;
  int gotStepValue = 1;
  bool gotSeqIterate = false;
  bool gotLog = false;
  bool gotLogValue = false;
  bool gotPrint = false;
  int  gotPrintValue = msg->client;
  bool gotEdit = false;
  char gotEditStr[VBL] = "";
  bool gotEditComplete = false;
  //
  // extract parameters
  while (msg->tag.getNextAttribute(att, val, VBL))
  { // camera device
    if (strcasecmp(att, "seqIterate") == 0)
    {
      gotSeqIterate = true;
    }
    else if (strcasecmp(att, "load") == 0)
    {
      strncpy(gotLoadName, val, MNL);
      gotLoad = true;
    }
    else if (strcasecmp(att, "unload") == 0)
    { // new mission line
      gotUnload = true;
      strncpy(gotUnloadName, val, VBL);
    }
//     else if (strcasecmp(att, "clearAll") == 0)
//     { // clear current pgm
//       gotClearAll = true;
//     }
    else if (strcasecmp(att, "list") == 0)
    { // test if no specific rule - if no rule name lists all
      if (strlen(val) == 0)
        gotAll = true;
      else
      { // list specific rule only, but in full length
        gotList = true;
        strncpy(gotListName, val, VBL);
      }
    }
    else if (strcasecmp(att, "state") == 0)
    { // get current state
      gotState = true;
    }
    else if (strcasecmp(att, "all") == 0)
    { // list all plans
      gotAll = true;
    }
    else if (strcasecmp(att, "file") == 0)
    {
      gotFile = true;
      strncpy(gotFileName, val, VBL);
    }
/*    else if (strcasecmp(att, "abort") == 0)
    {
      gotAbort = true;
    }*/
    else if (strcasecmp(att, "stop") == 0)
    {
      gotStop = true;
    }
    else if (strcasecmp(att, "resume") == 0)
    {
      gotResume = true;
    }
    else if (strcasecmp(att, "enable") == 0)
    {
      gotEnable = true;
      strncpy(gotRuleName, val, MPL);
    }
    else if (strcasecmp(att, "disable") == 0)
    {
      gotDisable = true;
      strncpy(gotRuleName, val, MPL);
    }
    else if (strcasecmp(att, "step") == 0)
    {
      gotStep = true;
      if (strlen(val) > 0)
        gotStepValue = strtol(val, NULL, 0);
    }
/*    else if (strcasecmp(att, "def") == 0)
    {
      gotDef = true;
      strncpy(gotDefName, val, VBL);
    }*/
    else if (strcasecmp(att, "status") == 0)
    {
      gotStatus = true;
    }
    else if (strcasecmp(att, "log") == 0)
    {
      gotLog = true;
      gotLogValue = str2bool2(val, true);
    }
    else if (strcasecmp(att, "getPrint") == 0)
    {
      gotPrint = true;
      if (strlen(val) > 0)
        gotPrintValue = strtol(val, NULL, 0);
    }
    else if (strcasecmp(att, "edit") == 0)
    {
      strncpy(gotRuleName, val, MPL);
    }
    else if (strcasecmp(att, "complete") == 0)
    {
      gotEditComplete = str2bool2(val, true);
      if (strlen(val) > 0 and str2bool2(val, false) != gotEditComplete and
          strlen(gotRuleName) == 0)
        // assume complete is followed by plan name
        strncpy(gotRuleName, val, MPL);
    }
    else if (strcasecmp(att, "add") == 0)
    {
      gotEdit = true;
      strncpy(gotEditStr, val, VBL);
    }
    else if (strcasecmp(att, "help") == 0)
      ask4help = true;
    else
    { // unknown attribute
      result = false;
      break;
    }
  }
  //
  if (seq != NULL and rules != NULL)
  {
    if (ask4help)
    {
      sendHelpStart(msg, "RULE");
      sendText(msg, "---- Available RULE (mission rules) options:\n");
      sendText(msg, "load=\"file\"        Load a rule file\n");
      sendText(msg, "unload=\"rule\"      Remove a rule\n");
      sendText(msg, "all                List loaded rules\n");
      sendText(msg, "list[=rule]        List a specific rule (source code as loaded)\n");
      sendText(msg, "state              List current state of a rule (debug info)\n");
      sendText(msg, "file=file          List a rule to this file (used with list or state)\n");
      sendText(msg, "step[=N]           perform a single (or N) sequencer steps - stopping after the step\n");
      sendText(msg, "                   Use resume to continue.\n");
      snprintf(reply, MRL, "stop               Stop (pause) execution - is stopped (%s)\n", bool2str(seq->isStopped()));
      sendText(msg, reply);
      sendText(msg, "resume             Continue execution of loaded rules\n");
      sendText(msg, "enable=rule        Activate or start a rule - if not running already\n");
      sendText(msg, "disable=rule       Inactivate or stop a rule (runs post code)\n");
      sendText(msg, "edit=rule add=\"rule lines\" [complete[=true]]\n");
      sendText(msg, "                   Create or append lines to a rule\n");
      sendText(msg, "                   NB! add no more than 900 characters in one go\n");
      sendText(msg, "                   eg. add='<rule if=\"true\">\\n print(\"OK\")\\n </rule>\n");
      sendText(msg, "getPrint[=N]       Get sequencer prints to client N (default = this)\n");
   //   sendText(msg, "clearAll           remove all rules\n");
      snprintf(reply, MRL, "log[=false]        Open or close logfiles rule.log and rulestate.log - is open (%s)\n",
               bool2str(seq->isLogOpen()));
      sendText(msg, reply);
      //sendText(msg, "sim=true | false   Set simulation flag (not implemented yet)\n");
      sendHelpDone(msg);
      sendReply = true;
    }
    else if (gotSeqIterate)
    { // sequencer loop iterate
      seq->seqIterateStep();
      // just call, do not send reply if client is < 0
      if (msg->client >= 0)
        sendInfo(msg, "done");
      // this command can not be combined with any other
    }
    else if (result)
    { // all remaining commands
      if (gotStep)
        sendReply = runStep(msg, gotStepValue);
      else if (gotStop)
        // stop sequencer or stop one plan only
        sendReply = runStep(msg, 0);
      else if (gotResume)
        // resume sequenser
        sendReply = runStep(msg, -1);
      if (gotDisable)
        sendReply = startStopRule(msg, gotRuleName, false);
      else if (gotEnable)
        // start one plan
        sendReply = startStopRule(msg, gotRuleName, true);
      if (gotStatus)
        sendReply = sendStateMessage(msg, msg->tag.getTagName());
      if (gotLoad)
        sendReply = loadRuleFile(msg, gotLoadName);
      if (gotAll and gotFile)
        sendReply = listAllMissions(msg, gotFileName);
      if (gotAll and not gotFile)
        sendReply = listAllMissions(msg, NULL);
      if (gotList and gotFile)
        sendReply = listMission(msg, gotListName, gotFileName);
      if (gotList and not gotFile)
        sendReply = listMission(msg, gotListName, NULL);
      if (gotState and gotFile)
        sendReply = listState(msg, gotFileName);
      if (gotState and not gotFile)
        sendReply = listState(msg, NULL);
      if (gotUnload)
        sendReply = unloadRule(msg, gotUnloadName);
      if (gotLog)
        sendReply = openLog(msg, gotLogValue);
      if (gotPrint)
      {
        seq->setPrintClientNumber(gotPrintValue);
        sendReply = sendInfo(msg, "printout redirected");
      }
      if ((gotEdit or gotEditComplete) and strlen(gotRuleName) > 0)
        sendReply = editRule(msg, gotRuleName, gotEditStr, gotEditComplete);
      //
      if (not sendReply)
        sendInfo(msg, "done");
    }
    else if (not result)
    {
      snprintf(reply, MRL, "The %s attribute %s=\"%s\" is not supported", msg->tag.getTagName(), att, val);
      sendWarning(msg, reply);
    }
  }
  else
  {
    snprintf(reply, MRL, "Error: Both resources (rules=%s state=%s) must be available",
            bool2str(seq != NULL), bool2str(rules != NULL));
    sendWarning(msg, reply);
  }
  return result;
}

///////////////////////////////////////////////////////////////////

bool UFuncRule::sendStateMessage(UServerInMsg * msg,
                                     const char * tagName)
{
  const int MRL = 200000;
  char reply[MRL];
  bool result = true;
  //
/*    if (lineActive != NULL)
      str2xml(cmd, MCL, lineActive->getCmdLine());*/
  seq->getStateStr("  ", reply, MRL);
  sendMsg(msg, "<help subject=\"rule execution state\">\n");
  sendText(msg, reply);
  sendMsg(msg, "/help>\n");
  return result;
}

///////////////////////////////////////////////////

const char * UFuncRule::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s Rule module (rulestate=%s rule=%s)\n",
             preString, bool2str(seq != NULL), bool2str(rules != NULL));
  return buff;
}

///////////////////////////////////////////////////////////////////

bool UFuncRule::loadRuleFile(UServerInMsg * msg, const char * name)
{
  bool result;
  const int MRL = 1000;
  char reply[MRL];
  char s[MRL];
  int n1, n2, i;
  const char * p1;
  UMisLoadedRule * mlp;
  //
  n1 = rules->getMissionCnt();
  result = rules->loadFile(name);
  n2 = rules->getMissionCnt();
  p1 = rules->getSyntaxErrorString();
  if (strlen(p1) > 0)
  {
    sendMsg(msg, "<help subject=\"Load rule file - report\"/>\n");
    sendText(msg, p1);
    sendMsg(msg, "</help>\n");
  }
  if (result)
  {
    snprintf(reply, MRL, "File (%s) is loaded, added %d now %d rules (see 'rule list')", name, n2 - n1, n2);
    sendInfo(msg, reply);
    for (i = n1; i < n2; i++)
    { // start newly loaded plans
      mlp = rules->getRule(i);
      seq->addRule(mlp);
    }
  }
  else
  { // get the full filename used in fopen() call
    getFullFilename(dataPath, name, s, MRL);
    snprintf(reply, MRL, "Failed to load (%s)", s);
    result = sendWarning(msg, reply);
  }
  return result;
}

//////////////////////////////////////////////////

bool UFuncRule::listAllMissions(UServerInMsg * msg, const char * filename)
{
  const int MRL = 1000;
  char reply[MRL];
  int n, i;
  UMisLoadedRule * plan;
  FILE * fp = NULL;
  const int MBL = 300000;
  char buf[MBL];
  char fn[MRL];
  char s[MRL];
  bool isActive;
  //
  n = rules->getMissionCnt();
  if (n > 0)
  {
    if (filename != NULL)
    { // save result to file too)
      if (strlen(filename) == 0)
        // use rule name as filename, but append an extension
        getFullFilename(dataPath, "all.rule", s, MRL);
      else
        // default is in data path (unless fully qualified filename)
        getFullFilename(dataPath, filename, fn, MRL);
      fp = fopen(fn, "w");
      if (fp != NULL)
        fprintf(fp, "<?xml version=\"1.0\" source=\"%s %g "
            __DATE__ " " __TIME__ "\"?>\n",
            rules->getResID(), rules->getResVersion()/ 100.0);
    }
    snprintf(reply, MRL, "Rule list (holds %d rules)", n);
    sendHelpStart(msg, reply);
    for (i = 0; i < n; i++)
    {
      plan = rules->getRule(i);
      isActive = seq->isActive(plan->getName());
      snprintf(reply, MRL, "rule (active=%5s run=%5s rule=%5s "
          "busy=%d) name=%s\n",
          bool2str(isActive), bool2str(plan->isRun),
          bool2str(plan->isRule), plan->setBusyCnt(0),
          plan->getName());
      sendText(msg, reply);
      if (fp != NULL)
      { // save the full plan
        plan->print("", buf, MBL);
        fprintf(fp, "%s\n", buf);
      }
    }
    sendHelpDone(msg);
//    sendInfo(msg, "done");
    if (fp == NULL and filename != NULL)
    {
      snprintf(reply, MRL, "Failed to save rules to %s", fn);
      sendWarning(msg, reply);
    }
    if (fp != NULL)
    {
      fclose(fp);
      snprintf(reply, MRL, "Rules saved to %s", fn);
      sendInfo(msg, reply);
    }
  }
  else
    sendWarning(msg, "No rules loaded");
  return true;
}

///////////////////////////////////////////////////////

bool UFuncRule::listMission(UServerInMsg * msg,
                           const char * name,
                          const char * filename)
{
  bool result;
  const int MRL = 1000;
  char reply[MRL];
  const int MBL = 300000;
  char buf[MBL];
  char s[MRL];
  char fn[MRL];
  FILE * fp;
  UMisRule * mr;
  //
  mr = rules->getRule(name, false);
  result = rules->getRuleList(name, buf, MBL);
  if (result)
  {
    if (filename == NULL)
    { // just send result to client
      snprintf(reply, MRL, "<%s subject=\"rule list\">\n",
              msg->tag.getTagName());
      sendMsg(msg, reply);
      snprintf(reply, MRL, "# source file %s\n", mr->getFileName());
      sendText(msg, reply);
      sendMsg(msg, buf);
      sendEndTag(msg);
    }
    else
    { // save list to file
      if (strlen(filename) == 0)
      { // use plan name as filename, but append an extension
        snprintf(s, MRL, "%s.rule", name);
        getFullFilename(dataPath, s, fn, MRL);
      }
      else
        getFullFilename(dataPath, filename, fn, MRL);
      // try to open file
      fp = fopen(fn, "w");
      if (fp != NULL)
      { // file open OK, so output mission
        fprintf(fp, "# source file %s", mr->getFileName());
        fprintf(fp, "%s", buf);
        fclose(fp);
        snprintf(reply, MRL, "Rule saved to %s", fn);
        sendInfo(msg, reply);
      }
      else
      { // file could nt be opened - send warning
        snprintf(reply, MRL, "Failed to save rule to %s", fn);
        sendWarning(msg, reply);
      }
    }
  }
  else
    sendWarning(msg, "Rule not found");
  return result;
}

////////////////////////////////////////////////////////////////

bool UFuncRule::listState(UServerInMsg * msg,
                           const char * filename)
{
  bool result = true;
  const int MRL = 1000;
  char reply[MRL];
  const int MBL = 300000;
  char buf[MBL];
  char s[MRL];
  char fn[MRL];
  FILE * fp;
  //
  seq->getStateStr("", buf, MBL);
  if (filename == NULL)
  {
    if (msg->client >= 0)
    {
      sendHelpStart("rule execution state");
      sendText(buf);
      sendHelpDone();
      result = sendInfo("done");
    }
    else
    {  // just send result to console
      snprintf(reply, MRL, "<%s subject=\"mission state\">\n",
                  msg->tag.getTagName());
      sendMsg(msg, reply);
      sendMsg(msg, buf);
      result = sendEndTag(msg);
    }
  }
  else
  { // save list to file
    if (strlen(filename) == 0)
    { // use plan name as filename, but append an extension
      snprintf(s, MRL, "mission-seq-%06d.state", seq->getIterationCnt());
      getFullFilename(dataPath, s, fn, MRL);
    }
    else
      getFullFilename(dataPath, filename, fn, MRL);
    // try to open file
    fp = fopen(fn, "w");
    if (fp != NULL)
    { // file open OK, so output mission
      fprintf(fp, "Mission state at iteration %d:\n", seq->getIterationCnt());
      fprintf(fp, "%s", buf);
      fclose(fp);
      snprintf(reply, MRL, "Mission state saved to %s", fn);
      result = sendInfo(msg, reply);
    }
    else
    { // file could nt be opened - send warning
      snprintf(reply, MRL, "Failed to save mission state to %s", fn);
      result = sendWarning(msg, reply);
    }
  }
  return result;
}

///////////////////////////////////////////////////////

bool UFuncRule::unloadRule(UServerInMsg * msg,
                           const char * name)
{
  bool result = true;
  const int MRL = 1000;
  char reply[MRL];
  UMisLoadedRule * plan, *ruleNext;
  const char * pname;
  UMisRuleState * caller;
  const char *p1, *p2;
  //
  // delete this mission only
  plan = rules->getRule(name, false);
  if (plan != NULL)
  { // remove ant state that involves this plan
    if (not plan->isBusy())
    {
      if (not plan->isEdit())
        // plan is not in edit mode, so remove plan state structure too
        seq->removeRule(name);
      //
      // now remove the plan specification
      if (rules->tryUnloadRule(name))
      {
        snprintf(reply, MRL, "deleted rule %s", name);
        sendInfo(msg, reply);
      }
      else
      {
        snprintf(reply, MRL, "failed in unloading of rule %s", name);
        sendError(msg, reply);
      }
    }
    else
    {
      caller = seq->getCaller(name);
      if (caller!=NULL)
        pname = caller->getCalled()->getRule()->getName();
      else
        pname = "none";
      snprintf(reply, MRL, "rule %s is buzy (by a call from %s)", name, pname);
      sendWarning(msg, reply);
    }
  }
  else
  { // plan not found, or name is a filename
    // assume name is a filename and unload all plans with this filename
    plan = rules->getRule(0);
    result = false;
    while (plan != NULL)
    {
      p1 = plan->sourceFileName;
      p2 = strstr(p1, name);
      ruleNext = plan->nextRule;
      if (p2 != NULL and (strlen(p2) + (p2 - p1) == strlen(p1)))
      {
        pname = plan->getName();
        snprintf(reply, MRL, "deleted rule %s", pname);
        if (not plan->isBusy())
        {
          if (not plan->isEdit())
            // plan is not in edit mode, so remove plan state structure too
            seq->removeRule(pname);
          if (rules->tryUnloadRule(pname))
          { // send the prepared message
            sendInfo(msg, reply);
            result = true;
          }
        }
        else
        { // change to a busy message
          snprintf(reply, MRL, "rule %s is buzy - try later", pname);
          sendWarning(msg, reply);
          result = true;
        }
      }
      plan = ruleNext;
    }
    if (not result)
    {
      snprintf(reply, MRL, "rule (or filename) %s not found", name);
      sendWarning(msg, reply);
      result = true;
    }
  }
  return result;
}

//////////////////////////////////////////////////

bool UFuncRule::startStopRule(UServerInMsg * msg, const char * name, bool start)
{
  bool result;
  const int MRL = 1000;
  char reply[MRL];
  //
  result = seq->startStopRule(name, start);
  if (result)
  { //
    if (start)
      snprintf(reply, MRL, "Rule %s started", name);
    else
      snprintf(reply, MRL, "Rule %s stopped (set inactive)", name);
    sendInfo(msg, reply);
  }
  else
  {
    snprintf(reply, MRL, "Rule %s not found", name);
    sendWarning(msg, reply);
  }
  // a reply is always send
  return true;
}

//////////////////////////////////////////////////


bool UFuncRule::runStep(UServerInMsg * msg, int value)
{
  bool result = true;
  const int MRL = 100;
  char reply[MRL];
  //
  if (seq->getRulesCnt() > 0)
  {
    if (value == 0)
      snprintf(reply, MRL, "Stopped (paused) execution - continue with resume or step");
    else if (value == -1)
      snprintf(reply, MRL, "Resumed execution");
    else
      snprintf(reply, MRL, "running %d step(s)", value);
  }
  else
  {
    if (value == 0)
      snprintf(reply, MRL, "Execution stopped - continue with resume or step (no rules loaded)");
    else if (value == -1)
      snprintf(reply, MRL, "Execution resumed (but no rules loaded)");
    else
      snprintf(reply, MRL, "Scheduled %d step(s), but no rules loaded", value);
  }
  seq->setSteps(value);
  result = sendInfo(msg, reply);
  //
  return result;
}

///////////////////////////////////////////////////

bool UFuncRule::openLog(UServerInMsg * msg, bool open)
{
  const int MRL = 600;
  char reply[MRL];
  //
  if (open)
  {
    seq->openLog();
    rules->openLog();
    snprintf(reply, MRL, "opened %s.log and %s.log in path %s", seq->getLogName(), rules->getLogName(), dataPath);
  }
  else
  {
    seq->closeLog();
    rules->closeLog();
    snprintf(reply, MRL, "closed %s.log and %s.log in path %s", seq->getLogName(), rules->getLogName(), dataPath);
  }
  sendInfo(msg, reply);
  return true;
}

///////////////////////////////////////////////////

bool UFuncRule::editRule(UServerInMsg * msg, const char * name,
                        const char * editStr,
                        const bool complete)
{
  UMisLoadedRule * plan;
//  UMisRuleState * caller;
  bool isOK;
  //
  // delete this mission only
  plan = rules->getRule(name, false);
  if (plan == NULL)
  { // plan do not exist, create it
    plan = rules->addNewRule(name);
    if (plan == NULL)
      sendWarning(msg, "failed to add empty rule");
  }
  if (plan != NULL)
  { // remove ant state that involves this plan
    if (not plan->isBusy())
    {
      if (not plan->isEdit())
      {
        seq->removeRule(name);
        plan->setEdit(true);
      }
      if (plan->isEdit())
      {
        if (strlen(editStr) > 0)
          plan->addEditLines(editStr);
        if (complete)
        {
          isOK = rules->completeEdit(plan);
          if (isOK)
          {
            isOK = seq->addRule(plan);
            if (isOK)
              sendInfo(msg, "rule succesfully completed");
          }
          else
            sendWarning(msg, "Failed to complete rule - there were errors");
        }
        else
          sendInfo(msg, "rule appended and is in edit mode");
      }
    }
    else
      sendWarning(msg, "Failed to edit rule - rule is busy (code used by some rule)");
  }
  return true;
}


