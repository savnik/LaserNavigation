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

#include <stdio.h>
#include <unistd.h>

#include "uvarcalc.h"

#include "ufunctionif.h"

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionIf::UFunctionIf()
 : UFunctionBase()
{ // initialization of variables in class - as needed
  // main interface resource
  setCommand("gif gifOnConnect", "genif", "generic interface to inter server connection resource");
  resIf = NULL;
  resIfLocal = false;
  ifVar = NULL;
  ifVarLocal = false;
  dataTrapSerial = 0;
  dataTrapSerialHelp = 0;
  setAliasName("if");
}

///////////////////////////////////////////////////

UFunctionIf::~UFunctionIf()
{ // possibly remove allocated variables here - if needed
  if (resIf != NULL)
  {
    resIf->UClientHandler::stop(true);
    if (resIfLocal)
    {
      delete resIf;
      resIf = NULL;
    }
  }
  if (ifVar != NULL and ifVarLocal)
    delete ifVar;
}

///////////////////////////////////////////////////

const char * UFunctionIf::name()
{
  return "if (server interface client) (" __DATE__ " " __TIME__ " by Christian)";
}

///////////////////////////////////////////////////

void UFunctionIf::createResources()
{
  const int MSL = 100;
  char s[MSL];
  int serverPort = 0;
  int serverVersion = 2;
  double d;
  char s2[MSL];
  if (resIf == NULL)
  { // no cam pool loaded - create one now
    resIf = new UResIf(aliasName);
    resIf->setVerbose(false);
    if (getGlobalValue("core.port", &d))
      serverPort = roundi(d);
    if (getGlobalValue("core.version", &d))
      serverVersion = roundi(d);
    strncpy(s2, "failed", MSL);
    gethostname(s2, MSL);
    // make string with info on this server
    snprintf(s, MSL, "name=\"%s\" version=\"%d\" host=\"%s\" port=\"%d\"", 
             appName, serverVersion, s2, serverPort);
    resIf->setNamespace(resIf->getResID(), s);
    // start rx-loop
    resIf->start();
    // mark as locally created
    resIfLocal = (resIf != NULL);
    addResource(resIf, this);
  }
  if (ifVar == NULL)
  { // no pool - so (try to) create one
    const int MSL = 64;
    char s[MSL];
    snprintf(s, MSL, "if%s", aliasName);
    ifVar = new UResIfVar(s);
    ifVar->setVerbose(false);
    // handles 'var' tags by default - more may be added here
    ifVar->addTags("odoPose");
    ifVar->addTags("utmPose");
    ifVar->addTags("mapPose");
    ifVarLocal = (ifVar != NULL);
    addResource(ifVar, this);
  }
}

////////////////////////////////////////////////////

bool UFunctionIf::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  switch (getCmdIndex())
  {
    case 0: 
      result = handleIf(msg);
      break;
    case 1:
      result = handleIfPush(msg);
      break;
    default:
      sendDebug(msg, "Command not handled (by me)");
      break;
  }
  return result;
}

///////////////////////////////////////////////////

bool functionIfDataTrap(void * object, void * data)
{
  UFunctionIf * obj = (UFunctionIf *) object;
  USmlTag * tag = (USmlTag *) data;
  // call member function
  return obj->dataTrap(tag);
}

//----------------------------------------------------

bool UFunctionIf::handleIf(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 500;
  char attValue[VAL_BUFF_LNG];
  const int MCL = 500;
  char cmd[MCL] = "";
  const int MRL = 4000;
  char reply[MRL];
  const int MHL = 100;
  char host[MHL] = "";
  bool ask4help = false;
 // bool anAdd = false;
 // char addName[MAX_RESOURCE_ID_LENGTH];
  bool gotCmd = false;
  bool gotConnect = false;
  bool isConnected = false;
  bool gotTrap = false;
  bool aStatus = false;
  bool aVerbose = false;
  bool verbose = false;
  bool aHUP = false;
  bool aStxEtx = false;
  bool aReplay = false;
  bool aReplayValue = false;
  bool aShowWarning = false;
  bool aShowWarningValue = true;
  bool aShowInfo = false;
  bool aShowInfoValue = true;
  const int MBL = 100;
  char aBinNameValue[MBL] = "";
  bool aBinName = false;
  bool aAllIsBin = false;
  bool aAllIsBinValue = false;
  int gotPort = 0;
  bool result = false;
  char * p1, * p2;
  const char *p3;
  bool replyOK = false;
  UVarPool * vp;
  const int MSL = 40;
  char s[MSL];
  const char STX = 2;
  const char ETX = 3;
  //
  if (resIf != NULL)
  {
    gotPort = resIf->getPort();
    strncpy(host, resIf->getHost(), MHL);
    verbose = resIf->USmlSource::isVerbose();
    isConnected = resIf->isConnected();
  }
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  { // camera device
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "trap") == 0)
      gotTrap = true;
    else if (strcasecmp(attName, "status") == 0)
      aStatus = true;
    else if (strcasecmp(attName, "stx") == 0)
      aStxEtx = str2bool2(attValue, true);
    else if (strcasecmp(attName, "cmd") == 0)
    {
      snprintf(cmd, MCL, "%s\n", attValue);
      gotCmd = true;
      dataTrapClient = -1;
    }
//     else if (strcasecmp(attName, "add") == 0)
//     {
//       strncpy(addName, attValue, MAX_RESOURCE_ID_LENGTH);
//       //anAdd = true;
//     }
    else if (strcasecmp(attName, "connect") == 0)
    {
      gotConnect = true;
      p2 = attValue;
      p1 = strsep(&p2, ":");
      if (p2 != NULL)
      { // got a colon, so host:port format
        gotPort = strtol(p2, NULL, 0);
        strncpy(host, p1, MHL);
      }
    }
    else if (strcasecmp(attName, "verbose") == 0)
    {
      aVerbose = true;
      verbose = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "HUP") == 0)
      aHUP = str2bool2(attValue, true);
    else if (strcasecmp(attName, "replay") == 0)
    {
      aReplay = true;
      aReplayValue = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "warning") == 0)
    {
      aShowWarning = true;
      aShowWarningValue = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "info") == 0)
    {
      aShowInfo = true;
      aShowInfoValue = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "binname") == 0)
    {
      aBinName = true;
      strncpy(aBinNameValue, attValue, MBL);
    }
    else if (strcasecmp(attName, "allIsBin") == 0)
    {
      aAllIsBin = true;
      aAllIsBinValue = str2bool2(attValue, true);
    }
    else
    { // unused options - it is assumed to be to laser server
      gotCmd = true;
      // the rest of the options are assumed to be options to the command
      p3 = msg->tag.getNext();
      snprintf(cmd, MCL, "%s %s\n", attName, p3);
      break;
    }
  }
  if (ask4help)
  {
    snprintf(reply, MRL, "<help subject=\"%s\">\n", strnupper(s, aliasName, MSL));
    sendMsg(msg, reply);
    if (resIf != NULL)
    {
      snprintf(reply, MRL, "----------- available %s options\n", strnupper(s, aliasName, MSL));
      sendMsg(msg, reply);
      snprintf(reply, MRL, "connect[=\"%s:%d\"]   Connect to server (connected=%s)\n",
              host, gotPort, bool2str(isConnected));
      sendText(msg, reply);
      sendText(msg,        "cmd=\"cmd for server\"        Send command to server\n");
      snprintf(reply, MRL, "  e.g.: <%s cmd=\"module reslist\"/> to get resource list on server\n", aliasName);
      sendMsg(msg, reply);
      snprintf(reply, MRL, "  or:   <%s module reslist/> as a simpler command style \n", aliasName);
      sendMsg(msg, reply);
      snprintf(reply, MRL, "  or:    %s module reslist   as the brackets are not needed for manual commands\n", aliasName);
      sendMsg(msg, reply);
      sendText(msg,        "stx                         Pack command in STX ETX (for lms100)\n");
      sendText(msg,        "trap                        Send reply to this client (e.g. help text or images)\n");
      snprintf(reply, MRL, "  e.g.: <%s trap cmd=\"cam imageget all\"/> to get image bypassing the inbetween server\n", aliasName);
      sendMsg(msg, reply);
      sendText(msg,        "hup                         Disconnect (hangup)\n");
      snprintf(reply, MRL, "replay[=false               Should other end be notified "
               "on replay events (is %s)\n",  bool2str(resIf->isReplay()));
      sendMsg(msg, reply);
      sendText(msg,        "status                      Status for interface\n");
      snprintf(reply, MRL, "info[=false]                Print info messages to console (is %s)\n", bool2str(resIf->showInfo));
      sendText(msg, reply);
      snprintf(reply, MRL, "warning[=false]             Print warnings and errors to console (is %s)\n", bool2str(resIf->showWarnings));
      sendText(msg, reply);
      snprintf(reply, MRL, "verbose[=false]             Print more to server console - data handler determined (is %s)\n", bool2str(verbose));
      sendText(msg, reply);
/*      snprintf(reply, MRL, "add=var                     Add handler to copy server variables to %s.name in var pool\n", aliasName);
      sendText(msg, reply);*/
      sendText(msg,        "                            Handles also pose update for odoPose, utmPose and MapPose\n");
      snprintf(reply, MRL, "binname=NAME                Tag all received data as binary with NAME (is %s)\n",
                resIf->binName());
      sendText(msg, reply);
      snprintf(reply, MRL, "allIsBin=false|true         Is all received data to be treated as binary (is %s)\n",
                bool2str(resIf->binNameValid()));
      sendText(msg, reply);
    }
    else
    {
      sendText(msg,        "NB!                         Interface resource is missing (reload plug-in module)!!!\n");
    }
    sendText(msg,        "help                        This help tekst\n");
    sendText(msg,        "------\n");
    snprintf(reply, MRL, "See also '%sOnConnect'\n", aliasName);
    sendMsg(msg, reply);
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (resIf != NULL)
  {
    if (aHUP)
    {
      resIf->connectionHangUp();
      replyOK = sendInfo(msg, "HUP (hangup) requested");
    }
    if (gotConnect and not aHUP)
    { // do not connect if HUP on the same line - just set host and port
      resIf->openConnection(host, gotPort);
      snprintf(reply, MRL, "connection requested to %s:%d",
              resIf->getHost(), resIf->getPort());
      replyOK = sendInfo(msg, reply);
    }
    if (aVerbose)
    {
      resIf->UClientHandler::setVerbose(verbose);
      resIf->getVarPool()->setVerbose(verbose);
      replyOK = sendInfo(msg, "done");
    }
    if (gotCmd)
    {
      if (dataTrapSerial > 0)
      { // delete old trap
        resIf->delDataTrap(dataTrapSerial);
        dataTrapSerial = 0;
      }
      if (dataTrapSerialHelp > 0)
      { // delete old trap
        resIf->delDataTrap(dataTrapSerialHelp);
        dataTrapSerialHelp = 0;
      }
      // create new trap
      // get camera command type, this is the key to the reply
      if (gotTrap)
      { // reply data should be trapped and send to client
        dataTrapClient = msg->client;
        sscanf(cmd, "%s", reply);
        // add trap to catch reply to command
        dataTrapSerial = resIf->addDataTrap(reply,
                                            functionIfDataTrap,
                                            this);
        // also add trap for a help reply
        dataTrapSerial = resIf->addDataTrap("help",
                                            functionIfDataTrap,
                                            this);
      }
      // send command to server
      if (aStxEtx)
      { // sent packed in stx --- etx brackets
        snprintf(reply, MRL, "%c%s%c\n", STX, cmd, ETX);
        result = resIf->sendWithLock(reply);
      }
      else
      { // normal clear text - just append a newline
        if (strchr(cmd, '\n') == NULL)
          // ensure the command is finished off with a new-line
          strcat(cmd, "\n");
        result = resIf->sendWithLock(cmd);
      }
      replyOK = true;
      if (result)
      {
        if (resIf->USmlSource::isVerbose() or msg->client >= 0)
        { // want reply or is not (server) console
          p1 = strchr(cmd, '\n');
          if (p1 != NULL)
            *p1 = '\0';
          snprintf(reply, MRL, "send: %s\\n", cmd);
          sendInfo(reply);
        }
      }
      else
      { // terminate before end of line
        p1 = strchr(cmd, '\n');
        if (p1 != NULL)
          *p1 = '\0';
        snprintf(reply, MRL, "No connection to server - not send: %s\\n", cmd);
        sendWarning(reply);
      }
    }
    if (aStatus)
    {
      snprintf(reply, MRL, "<help subject=\"%s status\">\n", strnupper(s, aliasName, MSL));
      if (resIf == NULL)
        sendText(msg, "No interface resource is available! (no memory space?\n");
      else
      {
        snprintf(reply, MRL, "This interface is connected (%s) to %s %d\n",
                 bool2str(resIf->isConnected()), resIf->getHost(), resIf->getPort());
        sendText( msg, reply);
        vp = resIf->getVarPool();
        snprintf(reply, MRL,   " %s    Interface handler has (%d/%d var %d/%d structs %d/%d funcs)\n",
                 aliasName,
                 vp->getVarsCnt(), vp->getVarMax(),
                 vp->getStructCnt(), vp->getStructMax(),
                 vp->getMethodCnt(), vp->getMethodMax());
        sendText( msg, reply);
        //
        if (resIf == NULL)
          sendText(msg, " var   No var-pool interface for this interface (try add=var)\n");
        else
        {
          resIf->snprint(" var   ", reply, MRL);
          sendText(msg, reply);
        }
      }
      sendMsg(msg, "</help>\n");
      replyOK = sendInfo(msg, "done");
    }
    if (aReplay)
    {
      resIf->setReplay(aReplayValue);
      if (aReplayValue)
        replyOK = sendInfo(msg, "replay set true");
      else
        replyOK = sendInfo(msg, "replay set false");
    }
    if (aShowWarning)
    {
      resIf->showWarnings = aShowWarningValue;
      if (aShowWarningValue)
        replyOK = sendInfo(msg, "warnings will now be shown on console");
      else
        replyOK = sendInfo(msg, "warnings (some) will be hidden");
    }
    if (aShowInfo)
    {
      resIf->showInfo = aShowInfoValue;
      if (aShowInfoValue)
        replyOK = sendInfo(msg, "info messages will now be shown on console");
      else
        replyOK = sendInfo(msg, "info messages (most) will be hidden");
    }
    if (aBinName)
    {
      resIf->setBinName(aBinNameValue);
      replyOK = sendInfo("binary data handler set");
    }
    if (aAllIsBin)
    {
      resIf->setBinNameValid(aAllIsBinValue);
      if (aAllIsBinValue)
        replyOK = sendInfo("all data is handled as binary");
      else
        replyOK = sendInfo("data as handled as XML tags");
    }
    if (not replyOK)
      sendWarning(msg, "Unknown subject");
  }
  else
    sendWarning(msg, "No interface resource");
  return result;
}

//////////////////////////////////////////////

bool UFunctionIf::dataTrap(USmlTag * tag)
{
  bool result = false;
  const int MBL = 20;
  char buff[MBL];
  int buffCnt = MBL;
  USmlTag tagE;
  const int MRL = 200 + MBL;
  char reply[MRL];
  int n;
  bool binTag = false;
  int binTagSize = 0;
  int binTagCnt = 0;
  //
  n = tag->getTagCnt();
  n++;
  strncpy(reply, tag->getTagStart(), n);
  reply[n] = '\0';
  strcat(reply, "\n");
  sendMsgInt(dataTrapClient, reply);
  if (tag->isAStartTag())
  { // get all data to until end-tag
    result = true;
    while (result)
    {
      buffCnt = MBL;
      if (binTag)
      { // get binary data without check for tag
        buffCnt = mini(MBL, binTagSize - binTagCnt);
        result = resIf->getNBytes(buff, buffCnt, 400);
        if (result)
          binTagCnt += buffCnt;
        tagE.setValid(false);
        if (binTagCnt == binTagSize)
          // end of binary area
          binTag = false;
      }
      else
        result = resIf->getNextTag(&tagE, 400, NULL, buff, &buffCnt);
      if (result)
      {
        n = 0;
        if (buffCnt > 0)
        {  // copy any pre-tag data to reply buffer
          memmove(reply, buff, buffCnt);
          n = buffCnt;
        }
        if (tagE.isValid())
        { // this is the rest - add to reply (including terminating '>')
          memmove(&reply[n], tagE.getTagStart(), tagE.getTagCnt() + 1);
          n += tagE.getTagCnt() + 1;
          if (tagE.isTagA("bin") and tagE.isAStartTag())
          { // only a proper bin tag if a size attribute exist
            binTag = tagE.getAttValue("size", buff, MBL);
            if (binTag)
              binTagSize = strtol(buff, NULL, 10);
            binTagCnt = 0;
          }
        }
        result = sendMsg(dataTrapClient, reply, n);
        if (tagE.isTagAnEnd(tag->getTagName()))
        { // this is the end of this story - send a terminating linefeed
          result = sendMsg(dataTrapClient, "\n", 1);
          break;
        }
      }
    }
    result = true;
  }
  return result;
}

///////////////////////////////////////////////////

const char * UFunctionIf::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s interface resource (empty %s)\n",
             preString, bool2str(resIf == NULL));
  return buff;
}

//////////////////////s//////////////////////////////

//
////////////////////////////////////////////////////

bool UFunctionIf::handleIfPush(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4List = false;
  const int MRL = 10000;
  char reply[MRL];
  int cm, ca;
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4List = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"IfConnect\">\n");
    sendText(msg, "----------------Available OnConnect options:\n");
    sendText(msg, "flush=cmd       Remove 'cmd' command(s) defined by client (default all)\n");
    sendText(msg, "cmd=\"cmd\"       Do execute 'cmd' command on connect\n");
    sendText(msg, "list            List all defined commands (as help text)\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (resIf != NULL)
  {
    if (ask4List)
    {
      sendMsg(msg, "<help subject=\"OnConnect command list\">\n");
      resIf->UServerPush::print("interface ", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    { // push or flush command
      result = resIf->addPushCommand(msg);
      if (result)
      {
        resIf->getPushCmdCnt(&cm, &ca);
        snprintf(val, VBL, "push command succeded - now %d pushCmds and %d pushCalls",
                 cm, ca);
        sendInfo(msg, val);
      }
      else
        sendWarning(msg, "push command failed");
    }
  }
  else
    sendWarning(msg, "No interface resource");

  return result;
}
