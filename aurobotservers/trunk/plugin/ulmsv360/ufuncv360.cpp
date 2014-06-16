/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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

#include "uresv360.h"

#include "ufuncv360.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  return new UFuncV360();
}


#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFuncV360::UFuncV360()
{
  setCommand("v360 v360push", "virtuel360", "implements a virtual 360 deg laser scanner (by jca " __DATE__ " " __TIME__ ")");
  v360 = NULL;
}


UFuncV360::~UFuncV360()
{
  if (v360 != NULL)
  {
    delete v360;
    v360 = NULL;
  }
}

///////////////////////////////////////////////////

void UFuncV360::createResources()
{
  v360 = new UResV360();
  addResource(v360, this);
}

/////////////////////////////////////////////////////////////

bool UFuncV360::handleCommand(UServerInMsg * msg, void * extra)
{
  bool result = false;
  //
  if (msg->tag.isTagA("v360"))
    result = handleV360Command(msg, (ULaserData *) extra);
  else if (msg->tag.isTagA("v360Push"))
    result = handleV360Push(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;

}

//////////////////////////////////////////////////////////////

bool UFuncV360::handleV360Command(UServerInMsg * msg, ULaserData * data)
{
  //bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 50;
  char attValue[VAL_BUFF_LNG];
  ULaserData laserDataBuff;
  int modeFake = 0;
  bool ask4help = false;
  bool doUpdate = false;
  bool unusedParams = false;
  bool aSrc = false;
  const int MCL = 30;
  char aSrcValue[MCL] = "";
  const int MRL = 1000;
  char reply[MRL];
  char s[MRL];
  char unused[MRL] = "";
  int unusedCnt=0;
  bool newLPx = false, newLPy = false, newLPth = false;
  UPose laserPose, laserPoseOld;
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "update") == 0)
      doUpdate = true;
    else if (strcasecmp(attName, "fake") == 0)
    {
      if (strcasecmp(attValue, "false") == 0)
        modeFake = 0;
      else if (strlen(attValue) == 0)
        modeFake = 1; // no value, just fake
      else
        modeFake = strtol(attValue, NULL, 10);
    }
    else if (strcasecmp(attName, "src") == 0)
    {
      aSrc = true;
      strncpy(aSrcValue, attValue, MCL);
    }
    else
    { // ignore unknown attributes
      unusedParams = true;
      snprintf(&unused[unusedCnt], MRL - unusedCnt - 1,
                " %s='%s'", attName, attValue);
      unusedCnt=strlen(unused);
    }
  }
  //
  if (ask4help)
  {
    sendHelpStart(msg, "V360");
    sendText(msg, "--------------Available V360 options:\n");
    snprintf(reply, MRL, "src=N         Source device number (is %d) (-1 for default)\n", v360->getDefDevice());
    sendText(reply);
    sendText(            "src=dev       Source device type (e.g. sick)\n");
    sendText(reply);
    sendText(            "update        Update virtual scan\n");
    sendText(            "help          This help text");
    sendText( "---");
    sendText( "-- See also scanGet set and get commands");
    sendHelpDone();
    //result = true;
  }
  else if (unusedParams)
  {
    snprintf(reply, MRL, "Unused parameters: %s\n", unused);
    sendWarning(reply);
  }
  else if (gotAllResources(s, MRL)) // laserpool is a must (v360 an option)
  {
    if (newLPx or newLPy or newLPth)
    {
      laserPoseOld = v360->getLaserPose();
      if (newLPx)
        laserPoseOld.x = laserPose.x;
      if (newLPy)
        laserPoseOld.y = laserPose.y;
      if (newLPth)
        laserPoseOld.h = laserPose.h * M_PI / 180.0;
      //
      v360->setLaserPose(laserPoseOld);
      snprintf(reply, MRL, "LaserPose %gx %gy %gth",
               laserPoseOld.x, laserPoseOld.y,
               laserPoseOld.h * 180.0 / M_PI);
      sendInfo(msg, reply);
    }
    if (aSrc)
    {
      if (strlen(aSrcValue) > 0)
        v360->setDefDevice(aSrcValue);
      snprintf(reply, MRL, "source device is %d", v360->getDefDevice());
      sendInfo(reply);
    }
    if (doUpdate)
    {
      v360->update(data, NULL, modeFake);
    }
  }
  else
    sendWarning(msg,"Missing v360 resource");
  return true;
}

////////////////////////////////////////////

bool UFuncV360::handleV360Push(UServerInMsg * msg)
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
    sendMsg(msg, "<help subject=\"V360PUSH\">\n");
    sendText(msg, "----------------Available V360PUSH settings:\n");
    sendText(msg, "total=k or n=k    Stop push after k events (def: no stop)\n");
    sendText(msg, "interval=k or i=k Push with this event interval (in images, def = 1)\n");
    sendText(msg, "flush[=cmd]       Remove push command from client (default all)\n");
    sendText(msg, "cmd=\"cmd\"         Do execute 'cmd' command for every event\n");
    sendText(msg, "call=method(par)  Do call method on event\n");
    sendText(msg, "list              List all active push commands\n");
    sendText(msg, "See also PUSH (timed push), SVS\n");
    sendText(msg, "---\n");
    sendMsg(msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (v360 != NULL)
  { // push command
    if (ask4list)
    {
      sendMsg(msg, "<help subject=\"svsPush command list\">\n");
      v360->UServerPush::print("svsPush", reply, MRL);
      sendText(msg, reply);
      sendMsg(msg, "</help>\n");
      sendInfo(msg, "done");
    }
    else
    {
      result = v360->addPushCommand(msg);
      if (result)
      {
        snprintf(val, VBL, "push command succeded - now %d",
                 v360->getPushCmdCnt(NULL, NULL));
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
