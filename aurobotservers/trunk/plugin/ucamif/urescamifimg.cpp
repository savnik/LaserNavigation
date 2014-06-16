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
//#include <opencv/highgui.h>
#include <urob4/usmltag.h>

#include "urescamifimg.h"


// UResCamIfImg::UResCamIfImg()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   verboseMessages = true;
//   // pointer to external resources
//   imgPool = NULL;
//   // create space for 10 variables, 5 structures and 10 functions
//   createVarSpace(10, 0, 0, "Camera server interface image copy handling");
//   createBaseVar();
//   tick = 0;
// }

////////////////////////////////////////////

UResCamIfImg::~UResCamIfImg()
{
}

///////////////////////////////////////////

void UResCamIfImg::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    varImgTime = vp->addVar("time", 0.0, "d", "Latest image time received");
    varImgPoolNum = vp->addVar("pool", 0.0, "d", "Latest image pool number received");
    varImgCnt = vp->addVar("cnt", 0.0, "d", "Number of images received");
  }
}

///////////////////////////////////////////

const char * UResCamIfImg::snprint(const char * preString, char * buff, int buffCnt)
{
  int m = 0;
  char * p1;
  const int MSL = 50;
  char s[MSL];
  UTime t;
  //
  if (buff != NULL)
  {
    p1 = buff;
    t = varImgTime->getTime();
    t.getTimeAsString(s, true);
    snprintf(p1, buffCnt - m, "%sreceived %.0f images latest %.0f imgTime %s\n", preString,
             varImgCnt->getValued(), varImgPoolNum->getValued(), s);
  }
  return buff;
}

/////////////////////////////////////////////////////

bool UResCamIfImg::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  char * p1 = missingThese;
  int n = 0;
  //
  if (imgPool == NULL)
  {
    strncpy(p1, UImagePool::getResClassID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }
  result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

//////////////////////////////////////////////////////////////////

bool UResCamIfImg::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UImagePool::getResClassID()))
  { // delete any local
    if (remove)
      imgPool = NULL;
    else if (imgPool != resource)
      imgPool = (UImagePool *)resource;
    else
      result = false;
  }
  else
    result = false;
  //
  result |= UResVarPool::setResource(resource, remove);
  return result;
}

/////////////////////////////////////////////////////////////

UImage * UResCamIfImg::getImageBuffer(int poolNumber, int height, int width, int channels, int depth)
{ // could be overwritten to get a image buffer associated with this pool number
  UImage * result = NULL;
  bool resize = false;
  double imgNum = poolNumber;
  double v;
  //
  if (imgPool != NULL)
  {
    result = imgPool->getImage(poolNumber, false);
    if (result)
      // test for image size change
      resize = ((int)result->width() != width or (int)result->height() != height);
    if (resize)
    { // remove old image first on resize
      callGlobal("disp.resize", "d", NULL, &imgNum, &v, NULL, NULL);
    }
    if (resize or result == NULL)
      result = imgPool->getImage(poolNumber, true, height, width, channels, depth);
  }
  return result;
}

/////////////////////////////////////////////////////////////

void UResCamIfImg::gotNewImage(UImage * img, int poolNum, USmlTag * tag)
{
  double imgNum = poolNum;
  char * dataType = (char*) "img";
  double v;
  int dev;
  const int MRL = 500;
  char req[MRL];
  int n;
  //
  n = varImgCnt->getInt();
  varImgCnt->setInt(n + 1);
  varImgPoolNum->setInt(poolNum);
  varImgTime->setTime(img->imgTime);
/*  printf("Got new image of size h=%d w=%d (from pool number %d)\n",
         img->height(), img->width(), poolNum);*/
  // tell display system of new image
  img->used = false;
  callGlobal("disp.newData", "sd", &dataType, &imgNum, &v, NULL, NULL);
  // request camera data for this device too
  dev = img->camDevice;
  snprintf(req, MRL, "camget device=\"%d\" pos rot focal size fps name\n", dev);
  tag->outputData(req);
}

/*
bool UResVarPool::call(const char * name, const char * paramOrder,
                      char ** strings, double * doubles,
                      double * value,
                      UDataBase ** returnStruct,
                      int * returnStructCnt)
*/
//////////////////////////////////////////////////

void UResCamIfImg::doTimeTick()
{
  //Wait(0.1);
  tick++;
}
