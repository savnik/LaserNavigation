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

#include "ufunccron.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncCron' with your classname, as used in the headerfile */
  return new UFuncCron();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncCron::~UFuncCron()
{ // possibly remove allocated variables here - if needed
  if (cron != NULL)
    delete cron;
}

///////////////////////////////////////////////////

bool UFuncCron::handleCommand(UServerInMsg * msg, void * extra)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 10000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool replySend = false;
  bool gotCmd;
  bool gotList;
  bool gotRes;
  int gotResValue = 0;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool result;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = strtol(val, NULL, 0);
    gotList = msg->tag.getAttValue("list", val, MVL);
    gotCmd = msg->tag.getAttValue("cmd", val, MVL);
    if (not gotCmd)
      gotCmd = msg->tag.getAttValue("flush", val, MVL);
    gotRes = msg->tag.getAttValue("res", val, MVL);
    if (gotRes)
      gotResValue = strtol(val, NULL, 0);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart( msg, "CRON");
    sendText( msg, "--- available CRON options\n");
    if (cron == NULL)
    {
      sendText( msg, "*** The needed CRON resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "log[=false]        Open or close cron logfile 'cron.log'"
             " - is open (%s)\n", bool2str(cron->isLogOpen()));
      sendText(msg, reply);
      sendText(msg,        "total=k or n=k     Stop push after k events (def: no stop)\n");
      sendText(msg,        "interval=T or t=T  Push with this interval (in seconds, def = 1.0)\n");
      sendText(msg,        "flush[=cmd]        Remove push command (default all)\n");
      sendText(msg,        "cmd=\"cmd\"          Do execute 'cmd' command on event\n");
      sendText(msg,        "call=method(par)   Do call method on event\n");
      snprintf(reply, MRL, "list               List active commands (%d commands)\n",
               cron->cmds.getPushCmdCnt(NULL, NULL));
      sendText(msg, reply);
      snprintf(reply, MRL, "res[=1]            List last results 1=last command else all (%d is available)\n",
               cron->getResultStringsCnt());
      sendText(msg, reply);
      snprintf(reply, MRL, "verbose[=false]    Output debug messages if true (is %s)\n",
               bool2str(cron->verbose));
      sendText(msg, reply);
      snprintf(reply, MRL, "stop               Stop any active jobs (%d is active)\n",
               cron->getActiveCnt());
      sendText(msg, reply);
      sendText(msg,        "help               This message\n");
      sendText(msg,        "--------\n");
      sendText(msg,        "see also           do cmd=""   for single commands,\n"
                           "                   var cron    for status and settings\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (cron == NULL)
  {
    sendWarning(msg, "no CRON resource to do that - try 'module list' for help");
    replySend = true;
  }
  else
  { // cron resource is available, so make a reply
    if (gotRes)
    {
      reply[MRL - 1] = '\0';
      cron->getResultStr(gotResValue, reply, MRL - 1);
      sendHelpStart(msg, "cron result list");
      sendText(msg, reply);
      sendHelpDone(msg);
      replySend = true;
    }
    if (gotCmd)
    {
      result = cron->cmds.addPushCommand(msg);
      if (result)
      {
        snprintf(reply, MRL, "push command succeded - now %d",
                 cron->cmds.getPushCmdCnt(NULL, NULL));
        sendInfo(msg, reply);
      }
      else
        sendWarning(msg, "push command failed");
      replySend = true;
    }
    if (gotList)
    {
      sendMsg(msg, "<help subject=\"cron command list\">\n");
      cron->cmds.print("cron", reply, MRL);
      sendText(msg, reply);
      sendHelpDone(msg);
      replySend = true;
    }
    if (gotVerbose)
    {
      cron->verbose = gotVerboseValue;
      sendInfo(msg, "done");
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo(msg, "no action performed");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////

void UFuncCron::createResources()
{
  cron = new UResCron();
  addResource(cron, this);
}
