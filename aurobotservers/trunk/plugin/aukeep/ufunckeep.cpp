/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   xx 03 2007 Modified by Lars                                           *
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

#include "ufunckeep.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncKeep' with your classname, as used in the headerfile */
  return new UFuncKeep();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncKeep::~UFuncKeep()
{ // possibly remove allocated variables here - if needed
  if (keep != NULL)
    delete keep;
}

///////////////////////////////////////////////////

bool UFuncKeep::handleCommand(UServerInMsg * msg, void * extra)
{
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("keep"))
    result = handleKeep(msg);
  else if (msg->tag.isTagA("keepWhenOn"))
    result = handleKeepPush(msg, true);
  else if (msg->tag.isTagA("keepWhenOff"))
    result = handleKeepPush(msg, false);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;

}

///////////////////////////////////////////////////////

bool UFuncKeep::handleKeep(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 10000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool replySend = false;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotAdd = false;
  bool gotDel = false;
  bool gotOnExpr = false;
  bool gotOffExpr = false;
  bool gotUnknownExpr = false;
  const int MEL = 1250;
  char expr[MEL] = "";
//  bool gotName = false;
  const int MNL = 100;
  char name[MNL] = "";
  const int MSL = 100;
  bool gotStartCmd = false;
  bool gotStopCmd = false;
  char startCmd[MSL];
  char stopCmd[MSL];
  bool isOK;
  bool gotWait = false;
  bool gotStop = false;
  bool gotGo = false;
  bool gotList = false;
  bool gotAllow = false;
  double gotAllowValue;
  bool gotAllowOff = false;
  bool gotAllowOffValue = false;
  UKeepItem * ki;
  int i;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = str2bool2(val, true);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
    gotStop  = msg->tag.getAttValue("stop",  name,  MNL);
    gotWait = msg->tag.getAttValue("wait", name, MNL);
    gotGo = msg->tag.getAttValue("go", name, MNL);
    gotAdd = msg->tag.getAttValue("add", name, MNL) or msg->tag.getAttValue("set", name, MNL);
    gotDel = msg->tag.getAttValue("del", name, MNL);
    gotOnExpr = msg->tag.getAttValue("onExpr", expr, MEL);
    gotOffExpr = msg->tag.getAttValue("offExpr", expr, MEL);
    gotUnknownExpr = msg->tag.getAttValue("unknownExpr", expr, MEL);
    gotStartCmd = msg->tag.getAttValue("toStart", startCmd, MSL);
    gotStopCmd  = msg->tag.getAttValue("toStop",  stopCmd,  MSL);
    gotList  = msg->tag.getAttValue("list",  NULL, 0);
    gotAllow  = msg->tag.getAttDouble("allow",  &gotAllowValue, 1.0);
    gotAllowOff  = msg->tag.getAttBool("allowOff",  &gotAllowOffValue, false);
  }
  // ask4help = false, i.e. no 'help' option.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart( msg, "KEEP");
    sendText( msg, "--- available KEEP options (keep a server running!)\n");
    if (keep == NULL)
    {
      sendText( msg, "*** The needed KEEP resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "log[=false]        Open or close logfile 'keep.log'"
             " - is open (%s)\n", bool2str(keep->isLogOpen()));
      sendText(msg, reply);
      sendText(msg,        "set=name           Add or change a keep function with this name\n");
      sendText(msg,        "del=name           Delete a keep function\n");
      sendText(msg,        "onExpr=\"expr\"    expression that evaluates true if process is running OK\n");
      sendText(msg,        "offExpr=\"expr\"   expression that evaluates true if process is not running\n");
      sendText(msg,        "unknownExpr=\"expr\" expression that evaluates true if process state is unknown\n");
      sendText(msg,        "allow=time         Allow this time before monitoring (is %gsec)\n");
      sendText(msg,        "allowoff           Off is an allowed stable condition (no retrigger of off event)\n");
      sendText(msg,        "wait[=name]        Set monitoring on standby (default=wait) if no name, then global wait flag\n");
      sendText(msg,        "go[=name]          Start monitoring 'name' process (if no name, then global wait flag).\n");
      snprintf(reply, MRL, "verbose[=false]    Output debug messages if true (is %s)\n",
               bool2str(keep->verbose));
      sendText(msg, reply);
      snprintf(reply, MRL, "list               List current keep processes (has %d)\n", keep->getKeepsCnt());
      sendText(msg, reply);
      sendText(msg,        "help               This message\n");
      sendText(msg,        "--------\n");
      sendText(msg,        "see also     keepWhenOn keepWhenOf   for actions on event,\n"
                           "             var keep        for status and settings\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (keep == NULL)
  {
    sendWarning(msg, "no KEEP resource to do that - try unload and reload plug-in");
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotVerbose)
    {
      keep->verbose = gotVerboseValue;
      sendInfo(msg, "done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      keep->openLog(gotOpenLogValue);
      sendInfo(msg, "done");
      replySend = true;
    }
    if (strlen(name) > 0)
    {
      ki = keep->getKeepItem(name);
      if (gotDel)
      {
        if (ki != NULL)
        {
          isOK = keep->deleteKeep(name);
          if (isOK)
            sendInfo(msg, "deleted");
          else
            sendWarning(msg, "found, but failed to delete");
        }
        else
          sendWarning(msg, "noone with this name");
        replySend = true;
      }
      if (gotAdd)
      { // add expression or add feature to expression
        strncpy(reply, "done", MRL);
        isOK = true;
        i = -2;
        if (ki == NULL)
        {
          ki = keep->add(name);
          if (ki == NULL)
          {
            strncpy(reply, "Failed to create keep", MRL);
            isOK = false;
          }
        }
        if (ki != NULL and strlen(expr) > 0)
        {
          if (gotOnExpr)
            i = 1;
          else if (gotOffExpr)
            i = 0;
          else if (gotUnknownExpr)
            i = -1;
          ki->setExpr(i, expr);
        }
        if (ki != NULL and strlen(expr) > 0)
        { // test expression
          ki->monitorTest(i, &isOK);
          if (not isOK)
            snprintf(reply, MRL, "Syntax error: %s", ki->getErrorTxt());
        }
        if (isOK)
          replySend = sendInfo(msg, reply);
        else
          replySend = sendWarning(msg, reply);
      }
      if (gotWait and ki != NULL)
      {
        if (ki != NULL)
          ki->setWait(true);
        else
          keep->setWait(true);
        if (not replySend)
          sendInfo(msg, "done");
        replySend = true;
      }
      if (gotGo)
      {
        if (ki != NULL)
          ki->setWait(false);
        else
          keep->setWait(false);
        if (not replySend)
          sendInfo(msg, "done");
        replySend = true;
      }
      if (gotAllow and ki != NULL)
      {
        ki->setAllow(gotAllowValue);
        if (not replySend)
          sendInfo(msg, "done");
        replySend = true;
      }
      if (gotAllowOff and ki != NULL)
      {
        ki->setAllowOff(gotAllowOffValue);
        if (not replySend)
          sendInfo(msg, "done");
        replySend = true;
      }
    }
    if (gotList)
    {
      keep->getKeepList("keep", reply, MRL);
      sendHelpStart(msg, "KEEP list");
      sendText(msg, reply);
      replySend = sendHelpDone(msg);
    }
  }
  if (not replySend)
    sendInfo(msg, "no action performed (no name?)");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////

bool UFuncKeep::handleKeepPush(UServerInMsg * msg, bool onCommand)
{ // send a reply back to the client requested the 'bark'
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4list = false;
  const int MRL = 1000;
  char reply[MRL];
  const int MPL = 100;
  char process[MPL] = "";
  UKeepItem * ki;
  UServerPush * kip;
  const char * onOff = "ON";
  //
  if (not onCommand)
    onOff = "OFF";
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4list = msg->tag.getAttValue("list", val, VBL);
  msg->tag.getAttValue("process", process, MPL);
  // ignore all other attributes
  if (ask4help)
  {
    if (onCommand)
    {
      sendMsg(msg, "<help subject=\"keepWhenOn\">\n");
      sendText(msg, "----------------Available keepWhenOn settings:\n");
    }
    else
    {
      sendMsg(msg, "<help subject=\"keepoff\">\n");
      sendText(msg, "----------------Available keepWhenOff settings:\n");
    }
    sendText(msg, "process=name      Relevant for this process (see 'keep list')\n");
    sendText(msg, "total=k or n=k    Stop after k events (def: no stop)\n");
    sendText(msg, "interval=k or i=k At this event interval only (def = 1)\n");
    sendText(msg, "flush[=cmd]       Remove event command (default: all)\n");
    sendText(msg, "cmd=\"cmd\"         Do execute 'cmd' command at event\n");
    sendText(msg, "call=method(par)  Do call method on event\n");
    sendText(msg, "list              List all active event commands\n");
    sendText(msg, "See also KEEP, KEEPWHENON, KEEPWHENOFF\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (keep != NULL)
  { // push command
    ki = keep->getKeepItem(process);
    if (ki == NULL)
    {
      snprintf(reply, MRL, "No such process (%s)", process);
      sendWarning(msg, reply);
    }
    else
    {
      kip = ki->getPushHandle(onCommand);
      if (ask4list)
      {
        sendMsg(msg, "<help subject=\"keep event command list\">\n");
        if (strlen(process) == 0)
          snprintf(val, VBL, "keep %s event", onOff);
        else
          snprintf(val, VBL, "%s %s event", process, onOff);
        kip->print(val, reply, MRL);
        sendText(msg, reply);
        sendMsg(msg, "</help>\n");
        sendInfo(msg, "done");
      }
      else
      {
        result = kip->addPushCommand(msg);
        if (result)
        {
          snprintf(val, VBL, "event command succeded - now %d",
                  kip->getPushCmdCnt(NULL, NULL));
          sendInfo(msg, val);
        }
        else
          sendWarning(msg, "event command failed (unknown command?)");
      }
    }
  }
  else
    sendWarning(msg, "No keep resource");

  return result;
}

////////////////////////////////////////////////////////////

void UFuncKeep::createResources()
{
  keep = new UResKeep();
  addResource(keep, this);
}
