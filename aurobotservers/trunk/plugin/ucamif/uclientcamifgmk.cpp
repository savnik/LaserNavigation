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

#include "uclientcamifgmk.h"


// UResCamIfGmk::UResCamIfGmk()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   verboseMessages = true;
//   // create space for 10 variables, 5 structures and 10 functions
//   createVarSpace(10, 0, 0, "Camera server interface guidemark variables");
//   createBaseVar();
// }

////////////////////////////////////////////

UResCamIfGmk::~UResCamIfGmk()
{
}

///////////////////////////////////////////

void UResCamIfGmk::createBaseVar()
{
  UVarPool * vp;
  varUpd = addVar("upd", 0.0, "d", "(r/w) Set to 'true' on update from interface");
  varTime = addVar("time", 0.0, "d", "(r) Guidemark detection time (sec)");
  varPosX = addVar("posX", 0.0, "6d", "(r) Position of new guidemark - the closest in robot coordinates");
  varCode = addVar("id", 0.0, "d", "(r) Code inside this guidematk (integer)");
  varGmkCnt = addVar("cnt", 0.0, "d", "(r) The number of guidemarks found in image");
  varCallDisp = addVar("callDisp", 1.0, "d", "(w) Should 'disp' be called after a data update");
  //
  // add also a structure for a selected guidemark to watch
  varSelID = addVar("selID", 0.0, "d", "(w) Guidemark with this code is put into the 'sel' structure");
  vp = getVarPool()->addStructLocal("sel", "selected guidemark with the ID-code in 'selID'", false);
  varSelUpd  = vp->addVar("upd", 0.0, "d", "(r/w) Set to 'true' on update from interface");
  varSelTime = vp->addVar("time", 0.0, "d", "(r) Guidemark detection time (sec)");
  varSelPosX = vp->addVar("posX", 0.0, "6d", "(r) Position of newe guidemark - the closest.");
  varSelCode = vp->addVar("id", 0.0, "d", "(r) Code inside this guidematk (integer)");
}

///////////////////////////////////////////

const char * UResCamIfGmk::snprint(const char * preString, char * buff, int buffCnt)
{
  int i, m = 0, gCnt;
  char * p1;
  UGmk * gmk;
  const int MSL = 50;
  char s[MSL];
  UPosition pos;
  //
  if (buff != NULL)
  {
    p1 = buff;
    gCnt = gmkPool->getGmkCnt();
    snprintf(p1, buffCnt - m, "%sholds %d guidemarks\n", preString, gCnt);
    m = strlen(p1);
    p1 = &buff[m];
    for (i = 0; i < gCnt; i++)
    {
      gmk = gmkPool->getGmkNum(i);
      gmk->getTime().getTimeAsString(s);
      pos = *gmk->getPos();
      snprintf(p1, buffCnt - m, "     id %lu at %gx %gy at %s\n", gmk->getCodeInt(),  pos.x, pos.y, s);
      m += strlen(p1);
      p1 = &buff[m];
    }
  }
  return buff;
}

/////////////////////////////////////////////////////

// bool UResCamIfGmk::gotAllResources(char * missingThese, int missingTheseCnt)
// { // just needs a pointer to core for event handling
//   bool result = true;
//   return result;
// }
//
// //////////////////////////////////////////////////////////////////
//
// bool UResCamIfGmk::setResource(UResBase * resource, bool remove)
// {
//   bool result = false;
//   return result;
// }


void UResCamIfGmk::newDataAvailable(int updCnt, UTime updTime)
{
  UGmk * gmk;
  const int MSL = 50;
  char s[MSL];
  int i;
  double dMin = 1e10;
  double d, code, v;
  UGmk * gmkMin = NULL;
  UGmk * gmkSel = NULL;
  unsigned long selCode;
  char * dataType = (char *) "gmk";
  //
  if (verboseMessages)
  {
    updTime.getTimeAsString(s, true);
    printf("UResCamIfGmk::newDataAvailable: Got %d guidemarks at %s\n", updCnt, s);
  }
  if (updCnt > 0)
  {
    lock();
    // get specific code to watch
    selCode = roundi(varSelID->getValued());
    //
    for (i = 0; i < gmkPool->getGmkCnt(); i++)
    {
      gmk = gmkPool->getGmkNum(i);
      if (gmk->getTime() == updTime)
      {
        d = gmk->pos().dist();
        if (d < dMin)
        {
          dMin = d;
          gmkMin = gmk;
        }
        if (selCode == gmk->getCodeInt())
          gmkSel = gmk;
      }
    }
    if (gmkMin != NULL)
    { // mark as updated
      varUpd->setBool(true);
      // update time
      varTime->setTime(gmkMin->getTime());
      // position
      varPosX->set6D(gmkMin->getPosRot());
/*      varPosY->setValued(gmkMin->getY(), 0);
      varPosZ->setValued(gmkMin->getZ(), 0);
      varPosO->setValued(gmkMin->getOmega(), 0);
      varPosP->setValued(gmkMin->getPhi(), 0);
      varPosK->setValued(gmkMin->getKappa(), 0);*/
      // ID in guidemark
      code = gmkMin->getCodeInt();
      varCode->setInt(roundi(code), 0);
      // set number of guidemarks in this image (usually just 1)
      varGmkCnt->setInt(updCnt, 0);
      if (gmkSel != NULL)
      { // update the selected guidemark too.
        varSelUpd->setBool(true, 0);
        varSelTime->setTime(gmkSel->getTime());
        varSelPosX->set6D(gmkSel->getPosRot());
/*        varSelPosY->setValued(gmkSel->getY(), 0);
        varSelPosZ->setValued(gmkSel->getZ(), 0);
        varSelPosO->setValued(gmkSel->getOmega(), 0);
        varSelPosP->setValued(gmkSel->getPhi(), 0);
        varSelPosK->setValued(gmkSel->getKappa(), 0);*/
        varSelCode->setInt(gmkSel->getCodeInt(), 0);
      }
    }
    unlock();
  // tell display system of new image
    if (varCallDisp->getValueBool(0))
    {
      callGlobal("view.newData", "sd", &dataType, &code, &v, NULL, NULL);
      callGlobal("disp.newData", "sd", &dataType, &code, &v, NULL, NULL);
    }
  }
}
