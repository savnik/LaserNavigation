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

#include "ufuncrhdif.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncRhdIf' with your classname, as used in the headerfile */
  return new UFuncRhdIf();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncRhdIf::~UFuncRhdIf()
{ // possibly remove allocated variables here - if needed
  if (rhd != NULL)
    delete rhd;
}

///////////////////////////////////////////////////

bool UFuncRhdIf::handleCommand(UServerInMsg * msg, void * extra)
{
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("rhd"))
    result = handleRhd(msg);
  else if (msg->tag.isTagA("rhdConnect"))
    result = handlePushCommand(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;

}

///////////////////////////////////////////////////////

bool UFuncRhdIf::handleRhd(UServerInMsg * msg)
{ // send a reply back to the client requested the 'bark'
  const int MRL = 10000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool replySend = false;
  bool gotUrl = false;
  bool aKeep = true;
  const int MUL = 250;
  char url[MUL] = "";
  bool gotConnect = false;
  bool gotHup = false;
  bool gotWrite = false;
  char * p1;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotUrl = msg->tag.getAttValue("url", url, MUL) or
             msg->tag.getAttValue("host", url, MUL);
    gotConnect = msg->tag.getAttValue("connect", val, MVL);
    if (gotConnect and strlen(val) > 0)
    {
      gotUrl = true;
      strncpy(url, val, MUL);
    }
    gotHup  = msg->tag.getAttValue("hup",  val,  MVL);
    msg->tag.getAttBool("keep", &aKeep, true);
    if (msg->tag.getAttBool("write", &gotWrite, true) and rhd != NULL)
      rhd->setWriteable(gotWrite);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart( msg, "RHD");
    sendText( msg, "--- available RHD interface options\n");
    if (rhd == NULL)
    {
      sendText( msg, "*** The needed RHDIF resource is not available ***\n");
      sendText( msg, "help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "url=\"host:port\"    Connection details (is %s:%d)\n", rhd->getHost(), rhd->getPort());
      sendText(msg, reply);
      snprintf(reply, MRL, "connect            Try connect to RHD (connected=%s)\n",
              bool2str(rhd->isConnected()));
      snprintf(reply, MRL, "                   write access: rhd.write=%s rhd.writegranted=%s\n",
               bool2str(rhd->isWriteRequested()), bool2str(rhd->isWriteGranted()));
      sendText(msg,        "hup                Stop (hang-up) connection to RHD\n");
      sendText(msg, reply);
      sendText(msg,        "help               This message\n");
      sendText(msg,        "--------\n");
      sendText(msg,        "see also   var rhd    for status and settings\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (rhd == NULL)
  {
    sendWarning(msg, "no RHD resource to do that - try unload and reload plug-in");
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotUrl)
    {
      p1 = strchr(url, ':');
      if (p1 != NULL)
        *p1++ = '\0';
      if (strlen(url) > 0)
        rhd->setHost(url);
      if (p1 != NULL)
        rhd->setPort(strtol(p1,NULL, 0));
      replySend = sendInfo(msg, "done");
    }
    if (gotConnect)
    {
      if (rhd->isConnected())
      {
        sendInfo(msg, "connected already");
      }
      else if (rhd->tryConnect(true, aKeep))
      {
        snprintf(reply, MRL, "Connected to %s:%d", rhd->getHost(), rhd->getPort());
        sendInfo(msg, reply);
      }
      else
      {
        snprintf(reply, MRL, "Failed to connect to %s:%d", rhd->getHost(), rhd->getPort());
        sendWarning(msg, reply);
      }
      replySend = true;
    }
    if (gotHup)
    {
      rhd->tryConnect(false, false);
      replySend = sendInfo(msg, "done");
    }
  }
  if (not replySend)
    sendInfo(msg, "no action performed");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////

void UFuncRhdIf::createResources()
{
  rhd = new UResRhdIf();
  if (rhd != NULL)
    addResource(rhd, this);
}

////////////////////////////////////////////////////////////

bool UFuncRhdIf::handlePushCommand(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4list = false;
  const int MRL = 1000;
  char reply[MRL];
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4list = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  {
    sendMsg(msg, "<help subject=\"RHDConnect\">\n");
    sendText(msg, "----------------Available RHD connect settings:\n");
    sendText(msg, "flush[=cmd]       Remove push command from client (default all)\n");
    sendText(msg, "cmd=\"cmd\"         Do execute 'cmd' command on established connection\n");
    sendText(msg, "call=method(par)  Do call method on established connection\n");
    sendText(msg, "list              List all active push commands\n");
    sendText(msg, "See also PUSH (timed push), RHD\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (rhd != NULL)
  { // push command
    if (ask4list)
    {
      sendMsg(msg, "<help subject=\"rhdConnect command list\">\n");
      rhd->UServerPush::print("RhdConnect", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    {
      result = rhd->addPushCommand(msg);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d",
                 rhd->getPushCmdCnt(NULL, NULL));
        sendInfo(msg, val);
      }
      else
        sendWarning(msg, "push command failed");
    }
  }
  else
    sendWarning(msg, "No such device");

  return result;
}
