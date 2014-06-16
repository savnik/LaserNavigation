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


#include <urob4/uvarcalc.h>
#include "uclienthandlercamif.h"


// UResCamIf::UResCamIf()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   //
//   verboseMessages = true;
//   createVarSpace(10, 0, 0, "Camera server interface variables");
//   createBaseVar();
//   // connect
//   tryHoldConnection = true;
// }

////////////////////////////////////////////

UResCamIf::~UResCamIf()
{
}

///////////////////////////////////////////


void UResCamIf::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "Resource version");
    vp->addVar("connected", 0.0, "d", "Connected to camera server if '1'");
    vp->addMethod(this, "send", "s", "send this command to the camera scanner");
  }
}

///////////////////////////////////////////

// void UResCamIf::print(const char * preString, char * buff, int buffCnt)
// {
//   int i, m = 0;
//   char * p1;
//   UClientFuncBase * fu;
//   //
//   if (buff != NULL)
//   {
//     p1 = buff;
//       snprintf(p1, buffCnt, "%sCamera interface connected %s to %s %d)\n",
//                preString, bool2str(isConnected()), getHost(), getPort());
//     m = strlen(p1);
//     p1 = &buff[m];
//     if (isConnected())
//     {
//       snprintf(p1, buffCnt - m, " - Server to local time %f secs (valid %s)\n",
//             getServerToLocalTime(), bool2str(isServerToLocalTimeValid()));
//       m += strlen(p1);
//       p1 = &buff[m];
//     }
//     snprintf(p1, buffCnt - m, " - Client has %d functions:\n", getFuncCnt());
//     m += strlen(p1);
//     p1 = &buff[m];
//     for (i = 0; i < getFuncCnt(); i++)
//     {
//       fu = getFunc(i);
//       snprintf(p1, buffCnt - m, "     #%d is '%s', handling '%s'\n", i,
//               fu->name(), fu->commandList());
//       m += strlen(p1);
//       p1 = &buff[m];
//     }
//   }
// }

/////////////////////////////////////////////////////

bool UResCamIf::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResCamIf::setResource(UResBase * resource, bool remove)
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

///////////////////////////////////////////

void UResCamIf::connectionChange(bool connected)
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

//////////////////////////////////////////////////////

bool UResCamIf::methodCall(const char * name, const char * paramOrder,
                             char ** strings, const double * pars,
                             double * value,
                             UDataBase ** returnStruct,
                             int * returnStructCnt)
{
  bool result = true;
  bool isOK;
  // evaluate standard functions
  if ((strcasecmp(name, "send") == 0) and (strcasecmp(paramOrder, "s") == 0))
  { // ensure noone else is using the line
    isOK = sendWithLock(strings[0]);
    if (value != NULL)
      *value = isOK;
  }
  else
    result = false;
  return result;
}

