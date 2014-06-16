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


#include <urob4/uvarcalc.h>

#include "urescamifcam.h"


// UResCamIfCam::UResCamIfCam()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   // create space for 10 variables, 5 structures and 10 functions
//   createVarSpace(10, 0, 0, "Camera server interface camera lens data handling");
//   createBaseVar();
// //  tick = 0;
// }

////////////////////////////////////////////

UResCamIfCam::~UResCamIfCam()
{
}

///////////////////////////////////////////

void UResCamIfCam::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    varUpdTime   = vp->addVar("time", 0.0, "d", "Latest update time");
    varCamDevNum = vp->addVar("lastDev", 0.0, "d", "Latest update is for this device");
    varUpdCnt    = vp->addVar("cnt", 0.0, "d", "Number of updates received");
    varCallDisp  = vp->addVar("callDisp", 1.0, "d", "Call display module on update");
  }
}

///////////////////////////////////////////

const char * UResCamIfCam::snprint(const char * preString, char * buff, int buffCnt)
{
  int k, i, n = 0;
  char * p1;
  const int MSL = 50;
  char s[MSL];
  UTime t;
  UClientCamData * cd;
  //
  if (buff != NULL)
  {
    p1 = buff;
    t = varUpdTime->getTime();
    t.getTimeAsString(s, true);
    snprintf(p1, buffCnt - n, "%sreceived %.0f updates last for camera %.0f at %s\n", preString,
             varUpdCnt->getValued(), varCamDevNum->getValued(), s);
    k = getCams()->getCamsCnt();
    for (i = 0; i < k; i++)
    {
      cd = getCams()->getCam(i);
      if (cd != NULL)
      {
        n += strlen(p1);
        p1 = &buff[n];
        cd->snprint("  -  ", p1, buffCnt - n);
      }
    }
  }
  return buff;
}

/////////////////////////////////////////////////////

// bool UResCamIfCam::gotAllResources(char * missingThese, int missingTheseCnt)
// { // just needs a pointer to core for event handling
//   bool result = true;
//   char * p1 = missingThese;
//   int n = 0;
//   //
//   if (imgPool == NULL)
//   {
//     strncpy(p1, UImagePool::getResID(), missingTheseCnt);
//     n += strlen(p1);
//     p1 = &missingThese[n];
//     result = false;
//   }
//   result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
//   return result;
// }

//////////////////////////////////////////////////////////////////

// bool UResCamIfCam::setResource(UResBase * resource, bool remove)
// {
//   bool result = true;
//   if (resource->isA(UImagePool::getResID()))
//   { // delete any local
//     if (remove)
//       imgPool = NULL;
//     else if (imgPool != resource)
//       imgPool = (UImagePool *)resource;
//     else
//       result = false;
//   }
//   else
//     result = false;
//   //
//   result |= UResVarPool::setResource(resource, remove);
//   return result;
// }

/////////////////////////////////////////////////////////////

void UResCamIfCam::gotNewData(int device)
{
  double dev = device;
  char * dataType = (char*)"cam";
  double v;
  const int MDSL = 20;
  char ds[MDSL];
  UVarPool * vp;
  UClientCamData * cam;
  UTime t;
  UPosRot pr;
  //
  //n = roundi(getLocalValue(varUpdCnt));
  varUpdCnt->add(1.0, 0);
  varCamDevNum->setInt(device);
  cam = getCams()->getCam(device);
  if (cam != NULL)
    // camera structure not yet available
    varUpdTime->setTime(cam->updTime);
  else
  {
    t.now();
    varUpdTime->setTime(t);
  }
  // set also global vars
  snprintf(ds, MDSL, "dev%d", device);
  vp = getVarPool()->getStruct(ds);
  if (vp == NULL)
  { // no struct available - so create one
    vp = getVarPool()->addStructLocal(ds, "Camera device details", false);
    if (vp != NULL)
    {
      varPosX = vp->addVar("posX", 0.0, "3d", "(r) Position of camera on robot");
      varPosY = vp->addVar("posY", 0.0, "3d", "");
      varPosZ = vp->addVar("posZ", 0.0, "3d", "");
      varPosO = vp->addVar("rotO", 0.0, "rot", "(r) orientation of camera");
      varPosP = vp->addVar("rotP", 0.0, "rot", "");
      varPosK = vp->addVar("rotK", 0.0, "rot", "");
      varFocal = vp->addVar("focal", 500.0, "d", "(r) Focal length in pixels (at current resolution)");
      varWidth = vp->addVar("width", 0.0, "d", "(r) Current image width (resolution)");
      varHeight  = vp->addVar("posY", 0.0, "d", "(r) Current image height");
    }
  }
  if (vp != NULL)
  {
    cam = getCams()->getCamData(device, false);
    pr.set(cam->pos, cam->rot);
    varPosX->set6D(&pr);
    varPosY->setDouble(cam->pos.y, 0);
    varPosZ->setDouble(cam->pos.z, 0);
    varPosO->setDouble(cam->rot.Omega, 0);
    varPosP->setDouble(cam->rot.Phi, 0);
    varPosK->setDouble(cam->rot.Kappa, 0);
    // this focal length is in current resolution (full resolution in cam-structure)
    varFocal->setDouble(cam->getFocalLength(), 0);
    varWidth->setInt(cam->width, 0);
    varHeight->setInt(cam->height, 0);
  }
  // tell display system of new image
  if (varCallDisp->getBool())
  {
    callGlobal("view.newData", "sd", &dataType, &dev, &v, NULL, NULL);
    callGlobal("disp.newData", "sd", &dataType, &dev, &v, NULL, NULL);
  }
  //
}

