/***************************************************************************
 *   Copyright (C) 2008 by DTU Christian Andersen   *
 *   chrand@mail.dk   *
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
#include "ufuncplugbase.h"

UFuncPlugBase::UFuncPlugBase()
 : UFunctionBase()
{
}


UFuncPlugBase::~UFuncPlugBase()
{
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

UFuncPlugBasePush::UFuncPlugBasePush()
{
}

//////////////////////////////////////////////////////////////

UFuncPlugBasePush::~UFuncPlugBasePush()
{
}

//////////////////////////////////////////////////////////////

bool UFuncPlugBasePush::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  result  = UFunctionBase::setResource(resource, remove);
  result |= UServerPush::setResource(resource, remove);
  return result;
}


////////////////////////////////////////////////////////////

bool UFuncPlugBasePush::handlePush(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  bool ask4help = false;
  bool ask4List = false;
  const int MRL = 1000;
  char reply[MRL];
  int cm, ca;
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4List = msg->tag.getAttValue("list", val, VBL);
  // ignore all other attributes
  if (ask4help)
  { // convert command name to upper case
    const char * nam = msg->tag.getTagName();
    int n = mini(strlen(nam), VBL - 1);
    for (int i = 0; i < n; i++)
      val[i] = toupper(nam[i]);
    val[n] = '\0';
    // generate help text
    snprintf(reply, MRL, "<help subject=\"%s\">\n", val);
    sendMsg(msg, reply);
    snprintf(reply, MRL, "----------------Available %s options:\n", val);
    sendText(msg, reply);
    sendText(msg, "cmd=\"cmd\"          Do execute 'cmd' command on event\n");
    sendText(msg, "flush=cmd          Remove 'cmd' command(s) defined by this client (default all)\n");
    sendText(msg, "total=k or n=k     Stop after k commands (default: no stop)\n");
    sendText(msg, "interval=k or i=k  Push with this interval of events (default = 1)\n");
    sendText(msg, "list               List all defined commands (as help text)\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (UServerPush::gotCmdExe())
  {
    if (ask4List)
    {
      sendMsg("<help subject=\"Push command list\">\n");
      UServerPush::print(msg->tag.getTagName(), reply, MRL);
      sendText(reply);
      sendMsg("</help>\n");
      sendInfo("done");
    }
    else
    { // push or flush command
      result = addPushCommand(msg);
      getPushCmdCnt(&cm, &ca);
      if (result)
      {
        snprintf(reply, MRL, "push command succeded - now %d cmds and %d calls",
                 cm, ca);
        sendInfo(reply);
      }
      else
        sendWarning("push command failed");
    }
  }
  else
    sendWarning("No events implementor resource");

  return result;
}


