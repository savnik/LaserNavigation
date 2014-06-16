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

#include "ufuncbin.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object

UFunctionBase * createFunc()
{ // create an object of this type
  //
  /** replace 'UFuncBin' with your classname, as used in the headerfile */
  return new UFuncBin();
}

#endif

///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////
///////////////////////////////////////////////////


UFuncBin::~UFuncBin()
{ // possibly remove allocated variables here - if needed
  if (bin != NULL)
    delete bin;
}

///////////////////////////////////////////////////

bool UFuncBin::handleCommand(UServerInMsg * msg, void * extra)
{
  bool ask4help;
  const int MVL = 1000;
  char val[MVL];
  bool replySend = false;
  bool gotOpenLog;
  bool gotOpenLogValue = false;
  bool gotVerbose;
  bool gotVerboseValue = false;
  UTime t;
  // check for parameters - one parameter is tested for - 'help'
  // the help value is ignored, e.g. if help="bark", then
  // the value "bark" will be in the 'helpValue' string.
  ask4help = msg->tag.getAttValue("help", val, MVL);
  // get all other parameters
  gotOpenLog = msg->tag.getAttValueBool("log", &gotOpenLogValue, true);
  gotVerbose = msg->tag.getAttValueBool("verbose", &gotVerboseValue, true);
  //
  if (ask4help)
  { // create the reply in XML-like (html - like) format
    sendHelpStart("BIN");
    sendText("--- available BIN options (mission poly lines and points)\n");
    sendText("verbose[=false]    Posibly more messages to server console\n");
    sendText("log[=false]        Open [or close] logfile\n");
    sendText("help               This message\n");
    sendText("--------\n");
    sendHelpDone();
    replySend = true;
  }
  else
  { // resource is available, so make a reply
    if (gotVerbose)
    {
      bin->verbose = gotVerboseValue;
      sendInfo("done");
      replySend = true;
    }
    if (gotOpenLog)
    {
      bin->openLog(gotOpenLogValue);
      sendInfo("done");
      replySend = true;
    }
  }
  if (not replySend)
    sendInfo(msg, "no poly action performed (no command option?)");
  // return true if the function is handled with a positive result
  return true;
}

////////////////////////////////////////////////////////

void UFuncBin::createResources()
{
  bin = new UResBin();
  if (strcmp(bin->getResID(), aliasName) != 0)
    bin->setResID(aliasName, 1385);
  addResource(bin, this);
}
