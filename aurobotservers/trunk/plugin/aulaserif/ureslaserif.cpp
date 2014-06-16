/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
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
#include <math.h>

#include <urob4/uvarcalc.h>

#include "ureslaserif.h"

// UResLaserIf::UResLaserIf()
// { // these first two lines is needed
//   // to save the ID and version number
//   setResID(getResID());
//   resVersion = getResVersion();
//   // other local initializations
//   createVarSpace(20, 0, 2, "Laser scanner server interface status");
//   createBaseVar();
//   // default port
//   port = 24919;
//   // default host
//   strncpy(host, "localhost", MAX_HOST_LENGTH);
//   // try to connect
//   tryHoldConnection = true;
// }


///////////////////////////////////////////

void UResLaserIf::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "Resource version");
    vp->addVar("connected", 0.0, "d", "Connected to laser scanner when '1'");
    vp->addMethod(this, "send", "s", "send this command to the laser scanner");
  }
}

///////////////////////////////////////////

UResLaserIf::~UResLaserIf()
{
}

///////////////////////////////////////////

const char * UResLaserIf::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
/*  snprintf(buff, buffCnt, "%s laserIf connected=%s to %s port %d\n", preString,
           bool2str(isConnected()), getHost(), getPort());*/
  return UClientHandler::snprint(preString, buff, buffCnt);
}

///////////////////////////////////////////

void UResLaserIf::connectionChange(bool connected)
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->setLocalVar("connected", connected, false, UVariable::b);
  }
  if (connected)
    // tell push system to execute on-connect commands
    setUpdated("");
}

///////////////////////////////////////////

bool UResLaserIf::setResource(UResBase * resource, bool remove)
{
  bool result = false;
  bool isUsed = false;
  //
  if (resource->isAlsoA(UCmdExe::getResClassID()))
  { // this ressource may hold variables that can be accessed by this module
    if (remove)
      UServerPush::setCmdExe(NULL);
    else
    { // resource is also a var-pool resource, so access as so.
      UServerPush::setCmdExe((UCmdExe *) resource);
    }
    result = true;
  }
  isUsed = UResVarPool::setResource(resource, remove);
  //
  return result or isUsed;
}

//////////////////////////////////////////////////////

bool UResLaserIf::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  bool isOK;
  // evaluate standard functions
  if ((strcasecmp(name, "send") == 0) and (strcasecmp(paramOrder, "s") == 0))
  {
    isOK = sendMsg(strings[0]);
    if (value != NULL)
      *value = isOK;
  }
  else
    result = false;
  return result;
}
