/***************************************************************************
 *   Copyright (C) 2009 by DTU (Christian Andersen)                        *
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

#include "ufuncmapobst.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncMapObst' with your classname, as used in the headerfile */
  return new UFuncMapObst();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncMapObst::~UFuncMapObst()
{ // possibly remove allocated variables here - if needed
  if (mobs != NULL)
    delete mobs;
}

///////////////////////////////////////////////////

bool UFuncMapObst::handleCommand(UServerInMsg * msg, void * extra)
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
  bool map2localize = false;
  bool justObstacles = false;
  int n;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotOpenLog = msg->tag.getAttValue("log", val, MVL);
    if (gotOpenLog)
      gotOpenLogValue = strtol(val, NULL, 0);
    gotVerbose = msg->tag.getAttValue("verbose", val, MVL);
    if (gotVerbose)
      gotVerboseValue = str2bool2(val, true);
    map2localize = msg->tag.getAttValue("map2localize", val, MVL);
    if (msg->tag.getAttValue("justObstacles", val, MVL))
      justObstacles = str2bool2(val, true);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("MAPOBST");
    sendText("--- available mapobst options\n");
    if (mobs == NULL)
    {
      sendText("*** The needed UResMapObst resource is not available ***\n");
      sendText("help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL, "log[=false]        Open or close logfile 'mapobst.log'"
             " - is open (%s)\n", bool2str(mobs->isLogOpen()));
      sendText(reply);
      sendText(        "map2localize       Send all map lines to localizer using 'addline startx ...'\n");
      sendText(        "justObstacles      Used with map2localize limits the lines to those marked as obstacles\n");
      snprintf(reply, MRL, "verbose[=false]    Output debug messages if true (is %s)\n",
               bool2str(mobs->verbose));
      sendText(reply);
      sendText(        "help               This message\n");
      sendText(        "--------\n");
      sendText(        "see also 'VAR mapobst', MAPBASE and AVOID\n");
    }
    sendHelpDone(msg);
    replySend = true;
  }
  else if (mobs == NULL)
  {
    sendWarning(msg, "no MAPOBST resource to do that");
    replySend = true;
  }
  else
  { // mobs resource is available, so make a reply
    if (gotVerbose)
    {
      mobs->verbose = gotVerboseValue;
      sendInfo("done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      mobs->openLog(gotOpenLogValue);
      snprintf(reply, MRL, "done, open=%s file=%s",
               bool2str(mobs->isLogOpen()), mobs->ULogFile::getLogFileName());
      sendInfo(reply);
      replySend = true;
    }
    if (map2localize)
    {
      n = mobs->sendMapLinesToLocalizer(msg->client, justObstacles);
      snprintf(reply, MRL, "Send %d lines from map to localizer", n);
      sendInfo(reply);
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo("no action performed");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////////

void UFuncMapObst::createResources()
{
  mobs = new UResMapObst();
  addResource(mobs, this);
}
