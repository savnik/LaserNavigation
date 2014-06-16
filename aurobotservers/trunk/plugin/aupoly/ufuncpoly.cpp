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

#include "ufuncpoly.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncPoly' with your classname, as used in the headerfile */
  return new UFuncPoly();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncPoly::~UFuncPoly()
{ // possibly remove allocated variables here - if needed
  if (poly != NULL)
    delete poly;
}

///////////////////////////////////////////////////

bool UFuncPoly::handleCommand(UServerInMsg * msg, void * extra)
{
  const int MRL = 150000;
  char reply[MRL];
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool replySend = false;
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool gotVerbose;
  bool gotVerboseValue = false;
  bool gotAdd = false;
  bool gotDel = false;
  const int MNL = 100;
  char name[MNL] = "";
  bool gotList = false;
  bool isOK;
  bool gotX, gotY;
  double x, y;
  bool gotGet = false;
  bool gotUpdate = false;
  UTime t;
  int c;
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
    gotAdd = msg->tag.getAttValue("add", name, MNL);
    gotDel = msg->tag.getAttValue("del", name, MNL);
    gotList = msg->tag.getAttValue("list",  NULL, 0);
    gotX = msg->tag.getAttDouble("X",  &x);
    gotY = msg->tag.getAttDouble("Y",  &y);
    if (not gotX)
      gotX = msg->tag.getAttDouble("E",  &x);
    if (not gotY)
      gotY = msg->tag.getAttDouble("N",  &y);
    gotGet = msg->tag.getAttValue("get",  val, MVL);
    gotUpdate = msg->tag.getAttValue("update",  val, MVL);
    if (gotUpdate and strlen(val) > 0)
      gotUpdate = str2bool2(val, true);
  }
  // ask4help = false, i.e. no 'help' option.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("POLY");
    sendText("--- available POLY options (mission poly lines and points)\n");
    if (poly == NULL)
    {
      sendText("*** The needed POLY resource is not available ***\n");
      sendText("help       This message\n");
    }
    else
    { // full functionality
      snprintf(reply, MRL,
               "list               List all polygons (has %d)\n", poly->getPolysCnt());
      sendText(reply);
      sendText("get                 Get all items in one xml message\n");
      sendText("update              Get all updated items only - first time you get all\n");
      sendText("add=name            Add a new (empty) item with this name\n");
      sendText("add=name x=X y=Y    Add one new point point to an item\n");
      sendText("add=name E=X N=Y    Add one new point point to an item\n");
      sendText("add=name color=dddd Add color, up to 4 chars: 1=matlab color, 1=line width, 2=* vertex circle, 4=' ' no edge line\n");
      sendText("del=name            Delete item with this name (may hold wildcards (* and ?)\n");
      sendText("verbose[=false]     Posibly more messages to server console\n");
      sendText("log[=false]         Open [or close] logfile\n");
      sendText("help                This message\n");
      sendText("--------\n");
    }
    sendHelpDone();
    replySend = true;
  }
  else if (poly == NULL)
  {
    sendWarning("no POLY resource to do that - try unload and reload plug-in");
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotVerbose)
    {
      poly->verbose = gotVerboseValue;
      sendInfo("done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      poly->openLog(gotOpenLogValue);
      sendInfo("done");
      replySend = true;
    }
    if (gotList)
    {
      poly->getList("poly", reply, MRL);
      sendHelpStart("POLY list");
      sendText(reply);
      replySend = sendHelpDone(msg);
    }
    if (gotAdd and strlen(name) > 0)
    {
      if ((gotX or gotY) and (strchr(name, '*') == NULL) and (strchr(name, '?') == NULL))
      {
        isOK = poly->add(name, x, y);
      }
      if (msg->tag.getAttValue("color", val, MVL))
      {
        int p1; // next polygon index number
        UPolyItem * pi = poly->getNext(0, &p1, name);
        while (pi != NULL)
        { // fill in new color codes
          for (int i = 0; i < mini(5, strlen(val)); i++)
            pi->color[i] = val[i];
          isOK = true;
          if (p1 < 0)
            break;
          // get next polygon to color
          pi = poly->getNext(p1, &p1, name);
        }
      }
      if (isOK)
        replySend = sendInfo("added OK");
      else
        replySend = sendWarning("failed - no more space?");
      poly->gotNewData();
    }
    if (gotDel)
    {
      poly->del(name);
      replySend = sendInfo("deleted (or did not exist)");
      poly->gotNewData();
    }
    if (gotGet or gotUpdate)
    {
      c = msg->client + 20;
      if (gotUpdate and c >= 0)
        // get index to last update timestamp
        t = updateTimes[c];
      else
        // else all updated since start of time (1970)
        t.clear();
      updateTimes[c].now();
      for (int u = 0; u < poly->getPolysCnt(); u++)
      {
        if (poly->codePolyXml(msg->tag.getTagName(), reply, MRL, t, u))
          replySend = sendMsg(reply);
      }
    }
  }
  if (not replySend)
    sendInfo(msg, "no poly action performed (no command option?)");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////

void UFuncPoly::createResources()
{
  poly = new UResPoly();
  addResource(poly, this);
}
