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

#include <ugen4/uimage2.h>
#include "uimgpush.h"
#include "uimagepool.h"
//#include "urawimage.h"

void UImagePool::UImagePoolInit()
{
  int i;
  //setResID(getResID());
  cmdexe = NULL;
  for (i = 0; i < MAX_IMAGES_IN_POOL; i++)
    images[i] = NULL;
  imagesCnt = 0;
  resVersion = getResVersion();
  varShowDefault = NULL;
  varImgCnt = NULL;
  varImgList = NULL;
  varShowDefault = NULL;
}

///////////////////////////////////////////////////////

UImagePool::~UImagePool()
{
  release();
}

///////////////////////////////////////////////////////

UImage * UImagePool::getImage(unsigned int source, bool mayCreate,
    const unsigned int height, const unsigned int width,
    const int channels, const int depth)

{
  UImgPush * result = NULL;
  UVarPool * imgVar;
  const int MSL = 40;
  char s[MSL];
  if (source < MAX_IMAGES_IN_POOL)
  { // get image pointer
    result = images[source];
    if ((result == NULL) and mayCreate)
    { // create image
      images[source] = new UImgPush(maxi(1, height), maxi(1, width), channels, depth);
      result = images[source];
      result->setCmdExe(cmdexe);
      imagesCnt = maxi(imagesCnt, source + 1);
      // update summary)
      if (varImgCnt != NULL)
      { // there is global variable structures for this image - update 
        int n = varImgCnt->getInt();
        varImgCnt->add(1);
        varImgList->setInt(source, n, true);
        //
        snprintf(s, MSL, "img%d", source);
        imgVar = addStruct(s, "details for image");
        result->createLocalVariables(imgVar);
        result->setTcpPort(23000 + source);
        if (strcasestr(appName, "client") != NULL)
          // we are as client, so show images as default
          varShowDefault->setBool(true);
        if (varShowDefault->getBool())
          result->startShowing(true);
      }
    }
    if (result != NULL)
      result->source = source;
  }
  return result;
}

///////////////////////////////////////////////////////

int UImagePool::createdImages()
{
  int result = 0;
  int i;
  //
  for (i = 0; i < imagesCnt; i++) 
    if (images[i] != NULL)
      result++;
  //
  return result;
}

///////////////////////////////////////////////////////

void UImagePool::release()
{
  int i;
  //
  for (i = 0; i < imagesCnt; i++)
    if (images[i] != NULL)
    {
      delete images[i];
      images[i] = NULL;
    }
}

///////////////////////////////////////////////////////

const char * UImagePool::print(const char * preString, char * buff, int buffCnt)
{
  int i, m = 0;
  char * p1;
  //
  if (buff != NULL)
  {
    p1 = buff;
    if (imagesCnt == 0)
      snprintf(p1, buffCnt, "%s no images available (max %d)\n", preString, MAX_IMAGES_IN_POOL);
    else
    {
      snprintf(p1, buffCnt, "%s available images:", preString);
      m += strlen(p1);
      p1 = &buff[m];
      for (i = 0; i < imagesCnt; i++)
      {
        if (images[i] != NULL)
          snprintf(p1, buffCnt - m, " %d", i);
        m += strlen(p1);
        p1 = &buff[m];
      }
      snprintf(p1, buffCnt - m, " (max %d)\n", MAX_IMAGES_IN_POOL);
    }
  }
  return buff;
}

/////////////////////////////////////////////////////

bool UImagePool::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result;
  result = (cmdexe != NULL);
  if ((not result) and (missingThese != NULL))
    snprintf(missingThese, missingTheseCnt, " %s", UCmdExe::getResClassID());
  return result;
}

//////////////////////////////////////////////////////////////////

bool UImagePool::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UCmdExe::getResClassID()))
  { // delete any local
    if (remove)
      cmdexe = NULL;
    else if (cmdexe != (UCmdExe *)resource)
      cmdexe = (UCmdExe *)resource;
    else
      result = false;
  }
  else
    result = false;
  return result;
}

/// ////////////////////////////////////////////////

void UImagePool::createResources()
{
  createBaseVar();
}

/// /////////////////////////////////////////////////

  /* *
  Make the variables that will be available to other plugins */
  void UImagePool::createBaseVar()
  {
    setDescription("Image pool information");
    varImgCnt = addVarA("imgCnt", "0.0 0.0", "d", "(r) number if images in imagepool (of maksimum)");
    varImgCnt->setInt(MAX_IMAGES_IN_POOL, 1, false);
    varImgList = addVar("imgList", 0.0, "d", "(r) list of constructed images");
    varShowDefault = addVar("show", 0.0, "d", "(rw) should images be shown as default");
  }


/// /////////////////////////////////////////////////

