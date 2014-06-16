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

#include "ufunctiongps.h"

#ifdef LIBRARY_OPEN_NEEDED

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFunctionGps' with your classname, as used in the headerfile */
  return new UFunctionGps();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////

UFunctionGps::UFunctionGps()
{ // initialization of variables in class - as needed
  setCommand("gps", "gpsif", "interface to GPS (NMEA) extraction unit");
  gps = NULL;       // initially the resource is not created
}

///////////////////////////////////////////////////

UFunctionGps::~UFunctionGps()
{ // possibly remove allocated variables here - if needed
  if (gps != NULL)
    delete gps;
}

///////////////////////////////////////////////////


void UFunctionGps::createResources()
{
  gps = new UResGps();
  if (strcmp(gps->getResID(), aliasName) != 0)
  {  // give resource a different name
    gps->setResID(aliasName, 1385);
  }
  gps->replaySetBaseFileName(aliasName);
  addResource(gps, this);
}

///////////////////////////////////////////////////

bool UFunctionGps::handleCommand(UServerInMsg * msg, void * extra)
{  // message is to be handled here
  const int MRL = 2000;
  char reply[MRL];
  const int MVL = 100;
  char val[MVL];
  bool gotGetStatus = false;
  bool silent = false;
  // get parameter options
  if (msg->tag.getAttValue("help", val, MVL))
  { // create the reply in XML (html - like) format
    sendHelpStart(aliasName);
    if (gps == NULL)
    {
      sendText("--- available GPS options\n");
      sendText("*** The needed GPS resource is not available ***\n");
      sendText("The resource should be created and loaded with this plugin.\n");
      sendText("*** The needed GPS resource is not available ***\n");
      sendText("help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL,  "--- available %s options\n", aliasName);
      sendText(reply);
      snprintf(reply, MRL,  "device='dev'     Set serial device name - is '%s'\n",
                gps->getDeviceName());
      sendText(reply);
      snprintf( reply, MRL, "speed='bps'      Set serial speed - is %d bps\n",
                gps->getDeviceSpeed());
      sendText(reply);
      snprintf( reply, MRL, "open[=false]     Open or close serial port to GPS - is open (%s)\n",
                bool2str(gps->isConnected()));
      sendText(reply);
      snprintf( reply, MRL, "log[=false]      Open or close NMEA logfile (%s) - is open (%s)\n",
                gps->logGps.getLogFileName(), bool2str(gps->isLogGpsOpen()));
      sendText(reply);
      snprintf( reply, MRL, "alive            Returns time since last NMEA position (is %g sec)\n",
                gps->getLastPositionTime().getTimePassed());
      sendText(reply);
      snprintf( reply, MRL, "replay           Set replay mode (is %s sec)\n",
                bool2str(gps->isReplay()));
      sendText(reply);
      sendText(             "silent           suppress replay\n");
      sendText(             "status           Get status\n");
      sendText(             "help             This message\n");
    }
    sendHelpDone();
  }
  else
  { // gps resource is available, so make a reply
    int valueInt;
    bool valueBool;
    // general attributes
    gotGetStatus = msg->tag.getAttValue("status", val, MVL);
    silent = msg->tag.getAttValue("silent", val, MVL);
    // set replay mode
    if (msg->tag.getAttBool("replay", &valueBool, true))
    {
      gps->setReplay(valueBool);
      snprintf(reply, MRL, "<%s info=\"is in replay mode: %s\"/>\n",
               msg->tag.getTagName(), bool2str(gps->isReplay()));
      if (not silent)
        sendMsg(msg, reply);
    }
    // replay step
    if (msg->tag.getAttInteger("step", &valueInt, 1))
    {
      if (gps->isReplay())
      {
        gps->replayStep(valueInt);
        if (not silent)
          sendInfo("stepped");
      }
      else if (not silent)
        sendWarning("GPS is not in replay mode");
    }
    if (msg->tag.getAttInteger("speed", &valueInt, 4800))
    {
        gps->setSpeed(valueInt);
        if (not silent)
          sendInfo(msg, "speed set (for next open)");
    }
    if (msg->tag.getAttValue("device", val, MVL))
    {
      if (strlen(val) > 0)
      {
        gps->setDevice(val);
        if (not silent)
          sendInfo(msg, "device set (for next open)");
      }
      else
        sendWarning(msg, "not a valid device name");
    }
    if (msg->tag.getAttBool("open", &valueBool, true))
    {
      if (valueBool)
        gps->openPort();
      else
        gps->closePort();
      gotGetStatus = true;
    }
    if (msg->tag.getAttBool("log", &valueBool, true))
    {
      gps->openLogfile(valueBool);
      gotGetStatus = true;
    }
    if (msg->tag.getAttBool("openLog", &valueBool, true))
    { // legacy syntax still works
      gps->openLogfile(valueBool);
      gotGetStatus = true;
    }
    if (gotGetStatus)
    {
      snprintf(reply, MRL, "<%s connected=\"%s\" device=\"%s\" logOpen=\"%s\"/>\n",
               msg->tag.getTagName(), bool2str(gps->isConnected()),
                                   gps->getDeviceName(), bool2str(gps->isLogGpsOpen()));
      // send reply string to client
      sendMsg(msg, reply);
    }
    if (msg->tag.getAttBool("alive", &valueBool, true))
    {
      snprintf(reply, MRL, "<%s last=\"%f\"/>\n",
               msg->tag.getTagName(), gps->getLastPositionTime().getTimePassed());
      sendMsg(msg, reply);
    }
  }
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////
