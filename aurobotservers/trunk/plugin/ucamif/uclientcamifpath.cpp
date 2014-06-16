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

#include "uclientcamifpath.h"


// UResCamIfPath::UResCamIfPath()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   verboseMessages = true;
//   createVarSpace(10, 0, 0, "Camera server road path detection variables");
//   createBaseVar();
// }

////////////////////////////////////////////

UResCamIfPath::~UResCamIfPath()
{
}

///////////////////////////////////////////

void UResCamIfPath::createBaseVar()
{
  varUpdTime = addVar("pTime", 0.0, "d", "Time when road otline were detected");
  varUpdPose = addVar("pose", "0.0 0.0 0.0" , "pose", "Pose of robot when robot when road outline were detected");
  varUpdNew = addVar("pNew", 0.0, "d", "Set to '1' when new data is available");
  //
  addMethod("getOutline", "", "Get latest road outline polygon");
}

//////////////////////////////////////////////

const char * UResCamIfPath::snprint(const char * preString, char * buff, int buffCnt)
{
  int m = 0;
  char * p1;
  UProbPoly * poly;
  const int MSL = 50;
  char s[MSL] = "(no-time)";
  bool valid;
  int n = 0;
  //
  if (buff != NULL)
  {
    p1 = buff;
    if (visData[0] != NULL)
    {
      poly = visData[0]->getPoly();
      valid = poly->isValid();
      poly->getPoseTime().getTimeAsString(s);
      n = poly->getPointsCnt();
    }
    else
      valid = false;
    snprintf(p1, buffCnt - m, "%sholds %d/%d path polygon area(s)\n", preString, visDataCnt, visDataMax);
    m = strlen(p1);
    p1 = &buff[m];
    snprintf(p1, buffCnt - m, "  -   Newest is valid (%s) with %d points at %s\n", bool2str(valid),  n, s);
  }
  return buff;
}

/////////////////////////////////////////////////////

// bool UResCamIfPath::gotAllResources(char * missingThese, int missingTheseCnt)
// { // just needs a pointer to core for event handling
//   bool result = true;
// /*  result = (cmdexe != NULL);
//   if ((not result) and (missingThese != NULL))
//     snprintf(missingThese, missingTheseCnt, " %s", UCmdExe::getResID());*/
//   return result;
// }

//////////////////////////////////////////////////////////////////

// bool UResCamIfPath::setResource(UResBase * resource, bool remove)
// {
//   bool result = false;
// /*  if (resource->isA(UCmdExe::getResID()))
//   { // delete any local
//     if (remove)
//       cmdexe = NULL;
//     else if (cmdexe != (UCmdExe *)resource)
//       cmdexe = (UCmdExe *)resource;
//     else
//       result = false;
//   }
//   else
//     result = false;*/
//   return result;
// }

//////////////////////////////////////////////////////////

void UResCamIfPath::newDataAvailable(UProbPoly * poly)
{
  UPose pose;
  //
  if (poly->isValid())
  {
    varUpdTime->setTime(poly->getPoseTime());
    pose = poly->getPoseOrg();
    varUpdPose->setPose(&pose);
    varUpdNew->setBool(true);
  }
}

//////////////////////////////////////////////////////////
