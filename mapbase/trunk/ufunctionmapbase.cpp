/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 * ----------------------------------------------------------------------- *
 *   Plugin for aurobotservers version 2.206.                              *
 *   Contains map database						   *
 *   Edited by Peter Tjell (s032041) & Søren Hansen (s021751)              *
 * ----------------------------------------------------------------------- *
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
/**
@author Christian Andersen
@editor Peter Tjell / Søren Hansen
*/

#include <urob4/usmltag.h>
#include "ufunctionmapbase.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded

UFunctionBase * createFunc()
{ // create an object of this type
  //
  return new UFunctionMapbase();
}

#endif

///////////////////////////////////////////////////

UFunctionMapbase::~UFunctionMapbase()
{
  // possibly remove allocated variables here - if needed

  if(mapbase != NULL)
    delete mapbase;
}

///////////////////////////////////////////////////

void UFunctionMapbase::createResources()
{ // Creates new resource
  mapbase = new UResMapbase();
  addResource(mapbase, this);
  return;
}

///////////////////////////////////////////////////

bool UFunctionMapbase::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  // Test for the handled commands, and call a function to do the job
  // the tagname is not case sensitive - see the library documentation for
  // 'ServerInMsg' and tag of type 'USmlTag' - to get available function list.
  if (msg->tag.isTagA("mapbase"))
    result = handleMapbase(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionMapbase::handleMapbase(UServerInMsg * msg)
{ // send a short reply back to the client requested the 'bark'

  bool ask4help;
  const int MVL = 500;
  char val[MVL];
  const int MRL = 500;
  char reply[MRL];
  char filename[MAX_FILENAME_LENGTH];
  bool gotmapload;
  bool gotdisplay;
  bool gotgraphload;
  bool gotexport;
  bool gotMapLines;
  bool handled = false;

  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  if (not ask4help)
  { // get all other parameters
    gotmapload = msg->tag.getAttValue("mapload", filename, MAX_FILENAME_LENGTH);
    gotdisplay = msg->tag.getAttValue("list", val, MVL) or msg->tag.getAttValue("display", val, MVL);
    gotexport = msg->tag.getAttValue("export", val, MVL);
    gotMapLines = msg->tag.getAttValue("mapLines", val, MVL);
    gotgraphload = msg->tag.getAttValue("graphload", filename, MAX_FILENAME_LENGTH);
  }
  // ask4help = false, if no 'help' option were available.
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart(msg,"Mapbase");
    sendText( msg,       "--- available Mapbase options\n");
    sendText( msg,       "help             This message\n");
    sendText( msg,       "mapload=[file]   Loads data in file into database\n");
    sendText( msg,       "graphload=[file] Loads graph data into database\n");
    sendText( msg,       "export           Exports the entire map in XML format\n");
    sendText( msg,       "mapLines         Export map lines\n");
    sendText( msg,       "list          Display current contents of database\n");
    if (mapbase != NULL)
    {
      sendText(mapbase->print("status: ", reply, MRL));
    }
    sendHelpDone(msg);
  }
  else if (mapbase == NULL)
    sendWarning(msg, "no varPool resource to access resource functions - try 'module list' for help");
  else
  {
    if(gotmapload)
    {
      mapbase->mapload(filename);
      handled = sendInfo(msg, "map loaded");
    }

    if(gotgraphload)
    {
      mapbase->graphmapper.loadMap(filename);
      handled = sendInfo(msg, "graph loaded");
    }
    if(gotdisplay)
    {
      mapbase->display();
      handled = sendInfo(msg, "done");
    }
    if(gotexport)
    {
      //Send the string output to clients (or server console)
      sendMsg(msg, mapbase->exportMap().c_str());
      handled = sendInfo(msg, "map exported");
    }
    if (gotMapLines)
    {
      handled = sendMapLines();
    }
    if(!handled)
    {
      sendWarning(msg, "No matching command option!");
    }
  }
  return true;
}

////////////////////////////////////////////////////////////

const char * UFunctionMapbase::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s Mapbase functions\n", preString);
  return buff;
}

////////////////////////////////////////////////////////////

bool UFunctionMapbase::sendMapLines()
{
  int i;
  USmlTag nTag;
  const int MRL = 300;
  char reply[MRL];
  UPose pose;
  double h, l;
  node * nd;
  //
  if (mapbase != NULL)
  {
    if (mapbase->maplines.size() == 0)
      sendWarning("no map lines loaded");
    else
    { // send start tag with informative attributes
      snprintf(reply, MRL, "name=\"maplines\" nodeName=\"%s\" lineCnt=\"%d\"",
               mapbase->maplines[0].nodename, mapbase->maplines.size());
      sendStartTag(reply);
      // send also the map coordinate base
      nd = mapbase->getMapRefNode();
      if (nd != NULL)
        // the referenced node is assumed to hold the origin position in utmPose coordinates
        // this 
        pose.set(nd->x, nd->y, nd->th);
      else
        pose.clear();
      nTag.codePose(&pose, reply, MRL, "mapCoordinateBase", NULL);
      sendMsg(reply);
      // send all lines
      for(i = 0; i < (int) mapbase->maplines.size(); i++)
      { // make a line segment structure
        // heading of line
        h = atan2(mapbase->maplines[i].y_e - mapbase->maplines[i].y_s,
                  mapbase->maplines[i].x_e - mapbase->maplines[i].x_s);
        // length of line
        l = hypot(mapbase->maplines[i].y_e - mapbase->maplines[i].y_s,
                  mapbase->maplines[i].x_e - mapbase->maplines[i].x_s);
        // make additional attribute with line width (which is not strictly the same as a perimeter - but more correct)
        snprintf(reply, MRL, "<line x=\"%.3f\" y=\"%.3f\" th=\"%.3f\" l=\"%g\" width=\"%g\"/>\n",
                 mapbase->maplines[i].x_s, mapbase->maplines[i].y_s,
                 h, l, mapbase->maplines[i].perimeter);
        // send off to client
        sendMsg(reply);
      }
      sendEndTag();
    }
  }
  else
    sendWarning("no mapBase module");
  return true;
}
