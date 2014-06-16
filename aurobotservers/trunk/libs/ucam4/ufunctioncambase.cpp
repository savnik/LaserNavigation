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
#include "ufunctioncambase.h"

UFunctionCamBase::UFunctionCamBase()
 : UFunctionImgBase()
{
  camPool = NULL;
  camPoolLocal = false;
  //imgPool = NULL;
}

//////////////////////////////////////////////////////////////////

UFunctionCamBase::~UFunctionCamBase()
{
/*  if (camPool != NULL)
    delete camPool;*/
}

//////////////////////////////////////////////////////////////////

// const char * UFunctionCamBase::resourceList()
// {
//   return "camPool imgPool";
// }

//////////////////////////////////////////////////////////////////

bool UFunctionCamBase::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UCamPool::getResClassID()))
  {
    if (camPoolLocal)
      // do not set locally owned resources
      result = false;
    else if (remove)
      // resource module unloaded
      camPool = NULL;
    else if (camPool != (UCamPool *)resource)
      // resource changed or new
      camPool = (UCamPool *)resource;
    else
      // no change - i.e. not used
      result = false;
  }
  else
    // may be the base class needs it (or maybe not)
    result = UFunctionImgBase::setResource(resource, remove);
  //
  return result;
}

//////////////////////////////////////////////////////////////////

UCamPush * UFunctionCamBase::getCam(
                        int imgDevice,     /* camera device number */
                        const char * posName)      /* camera position name */
{
  UCamPush * cam = NULL;
  const char * defName = "dummy";
  //
  if (camPool != NULL)
  {
    if (imgDevice >= 0)
      cam = camPool->getCam(imgDevice);
    else if (strlen(posName) > 0)
      cam = camPool->getCam(posName);
    else
      // default is device 0
      cam = camPool->getCam(imgDevice);
    if (cam == NULL)
    { // no default cam devive - make one
      if (imgDevice < 0)
        imgDevice = 0;
      if (strlen(posName) == 0)
        posName = defName;
      camPool->makeDevice(imgDevice, posName);
      cam = camPool->getCam(imgDevice);
    }
  }
  //
  return cam;
}

//////////////////////////////////////////////////////////////////

int UFunctionCamBase::getDefaultCamDevice()
{
  int devNum = -1;
  if (camPool != NULL)
    devNum = camPool->getFirstCamDevice();
  return devNum;
}

//////////////////////////////////////////////////////////////////

UCamPush * UFunctionCamBase::getCam(int camDevNum)      /* camera position name */
{
  UCamPush * result = NULL;
  if (camPool != NULL)
  {
    result = camPool->getCam(camDevNum);
  }
  return result;
}

//////////////////////////////////////////////////////////////////

bool UFunctionCamBase::getCamAndRawImage(
                       UCamPush ** cam,      // result camera
                       UImage ** img,  // result image
                       int * imgDevice,     // camera device number
                       //bool * imgRemRad,    // should/is radial error remoced
                       void * imgBase,      // pushed image (YUV)
                       const char * posName,      // camera position name
                       int rectfiedImg // destination for rectified image
                       )
{
  bool result = false;
  bool rect = (rectfiedImg >= 0);
  //
  if (imgBase != NULL)
  { // image push, but get camera from image info
    if (*img != NULL)
      // a destination image is available - copy image to this buffer
      (*img)->copy((UImage *)imgBase);
    else
      *img = (UImage *)imgBase;
    *imgDevice = (*img)->camDevice;
    *cam = camPool->getCam(*imgDevice);
    // log image if needed
    (*cam)->logImage(*img);
    //*imgRemRad = (*img)->radialErrorRemoved;
    result = true;
  }
  else
  { // get directly from camera
    *cam = getCam(*imgDevice, posName);
    result = (*cam != NULL);
    if (result and (*img == NULL))
    { // get an image buffer from image pool
      *img = imgPool->getImage((*cam)->getDevNum(), true);
      result = *img != NULL;
    }
    if (result)
      result = (*cam)->getImageSnapshot(*img); //, *imgRemRad);
    if (result)
      *imgDevice = (*img)->camDevice;
  }
  if (result and rect)
  {
    UImage * rectImg;
    UCamRad * camr = *cam;
    rectImg = imgPool->getImage(rectfiedImg, true);
    if (rectImg != NULL)
    { // make a destination image (also converts source image if in Bayer format)
      camr->removeRadialError(*img, rectImg);
      *img = rectImg;
    }
  }

  return result;
}

////////////////////////////////////////////////////

// bool UFunctionCamBase::gotAllResources(char * missingThese, int missingTheseCnt)
// {
//   bool result;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   result = camPool != NULL;
//   if ((not result) and (p1 != NULL))
//   {
//     snprintf(p1, missingTheseCnt, " %s", UCamPool::getResClassID());
//     n = strlen(p1);
//     p1 = &missingThese[n];
//   }
//   if (result)
//   {
//     result = camPool->gotAllResources(p1, missingTheseCnt - n);
//     if (not result and (p1 != NULL))
//     {
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//   }
//   result &= UFunctionImgBase::gotAllResources(p1, missingTheseCnt - n);
//   return result;
// }

////////////////////////////////////////////////////

void UFunctionCamBase::createResources()
{
  UFunctionImgBase::createResources();
  if (camPool == NULL)
  {
    camPool = new UCamPool();
    if (camPool != NULL)
    { // find devices and ...
      camPool->findDevices();
      // mark as locally created (to delete when appropriate)
      // camPoolLocal = true;
    }
    addResource(camPool, this);
  }
}

// UResBase * UFunctionCamBase::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UCamPool::getResClassID()) == 0)
//   {
//     if (camPool == NULL)
//     { // no cam pool loaded - create one now
//       camPool = new UCamPool();
//       if (camPool != NULL)
//       { // find devices and ...
//         camPool->findDevices();
//         // mark as locally created (to delete when appropriate)
//         camPoolLocal = true;
//       }
//     }
//     result = camPool;
//   }
//   if (result == NULL)
//     // may need a diffrerene resource
//     result = UFunctionImgBase::getResource(resID);
//   //
//   return result;
// }

///////////////////////////////////////////////////

