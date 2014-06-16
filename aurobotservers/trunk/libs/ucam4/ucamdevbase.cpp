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

#include <urob4/uvarpool.h>

#include "ucambase.h"
#include "ucamdevbase.h"
#include "urob4/uvarpool.h"

#include <pwc-ioctl.h>

///////////////////////////////////////////

UCamDevBase::UCamDevBase()
{
  int i;

  strncpy(camName, "noname", MAX_CAM_DEV_NAME_LENGTH);
  frameRate = -1;
  frameHeight = -1;
  frameWidth = -1;
  vgain = -1;
  vshutter = -1;
  vgamma = -1;
  vcolour = -1;
  vbrightness = -1;
  vcontrast = -1;
  vcompressionPref = -1;
  vcontour = -1;
  imCnt = 0;
  cam = NULL;
  imgBuffNext = 0;
  cam_fd = -1;
  devNum = -1;
  cameraOpen = false;
  imageNumber = 0;
  for (i = 0; i < RAW_IMAGEBUFFER_MAX_CNT; i++)
    imgBuff[i] = NULL;
  initialized = false;
  vars = NULL;
  varLog = NULL;
  varReplay = NULL;
  varCamName = NULL;
}

///////////////////////////////////////////

UCamDevBase::~UCamDevBase()
{
  if (imgBuff[1] != NULL)
  { // locally created buffers, so remove
    for (int i = 0; i < RAW_IMAGEBUFFER_MAX_CNT; i++)
      if (imgBuff[i] != NULL)
        delete imgBuff[i];
  }
  //printf("UCamDevBase buffers deleted\n");
}

//////////////////////////////////////////

void UCamDevBase::createVars()
{
  if (vars != NULL)
  {
    varLog = vars->addVar("log", 0.0, "d", "(r/w) is used images from this camera to be logged");
    varCamName = vars->addVarA("typeName", "noname", "s", "(r/w) Camera type name");    
  }
}


///////////////////////////////////////////

char * UCamDevBase::getWBModeAsString(int mode, char * strBuff)
{
  switch (mode)
  {
    case PWC_WB_AUTO:
      sprintf(strBuff, "auto     ");
      break;
    case PWC_WB_MANUAL:
      sprintf(strBuff, "manual   ");
      break;
    case PWC_WB_INDOOR:
      sprintf(strBuff, "indoor   ");
      break;
    case PWC_WB_OUTDOOR:
      sprintf(strBuff, "outdoor  ");
      break;
    case PWC_WB_FL:
      sprintf(strBuff, "flurocent");
      break;
    default:
      sprintf(strBuff, "unknown  ");
      break;
  }
  return strBuff;
}

////////////////////////////////////////////////////////

int UCamDevBase::getWBModeFromString(const char * mode)
{
  int result;
  //
  if (strncasecmp(mode, "auto", 4) == 0)
    result = PWC_WB_AUTO;
  else if (strncasecmp(mode, "manual", 4) == 0)
    result = PWC_WB_MANUAL;
  else if (strncasecmp(mode, "indoor", 4) == 0)
    result = PWC_WB_INDOOR;
  else if (strncasecmp(mode, "outdoor", 4) == 0)
    result = PWC_WB_OUTDOOR;
  else if (strncasecmp(mode, "flurocent", 4) == 0)
    result = PWC_WB_FL;
  else
    result = PWC_WB_AUTO;
  //
  return result;
}

/////////////////////////////////////////////

void UCamDevBase::gotNewImage(UImage * raw)
{
  if (cam != NULL)
    cam->gotNewImage(raw);
}

/////////////////////////////////////////////

void UCamDevBase::imgUpdated()
{
  if (cam != NULL)
    cam->imgUpdated();
}

/////////////////////////////////////////////

bool UCamDevBase::setDevice(const int width, const int height,
                           const int framesPerSec)
{
  frameWidth = width;
  frameHeight = height;
  frameRate = framesPerSec;
  return true;
}

////////////////////////////////////////////////

void UCamDevBase::imageSizeChanged(double iResFactor)
{
  if (cam != NULL)
    cam->imageSizeChanged(iResFactor);
}

////////////////////////////////////////////////

bool UCamDevBase::getLockedNewImage(UImage ** prgb)
{ // finds the most recent bufer with a timestamped image
  // and returns it locked.
  // opens the camera is closed.
  bool isOK;
  int bufNum;
  bool validAndLocked = false;
  int i = 0; //, j, n;
  //
  // if device is not open, or open in a wrong mode
  // (re)open in right mode
  isOK = isCameraOpen();
  // open camera
  if (not isOK)
    isOK = openDeviceDefault();
  // test for available images (image thread got first image)
  if (isOK and (imCnt == 0))
  { // wait for first image
    i = 0;
    while ((imCnt == 0) and (i < 40))
    { // no image is read yet, size is just changed and
      // this can take time (up to 2 seconds).
      Wait(0.05);
      i++;
    }
    isOK = imCnt > 0;
  }
  // debug
  // printf("Get new image cameraOpen=%s thread=%s imageCnt=%d\n",
  //        bool2str(cameraOpen), bool2str(threadRunning), imCnt);
  // debug end
  // get an image
  if (isOK)
  { // open and in right mode, now get image
    i = 0;
    do
    { // get buffer pointer just released by read thread
      bufNum = (imgBuffNext - 1 + RAW_IMAGEBUFFER_MAX_CNT)
          % RAW_IMAGEBUFFER_MAX_CNT;
      // get latest frame pointer
      //*praw = raw[bufNum];
      *prgb = imgBuff[bufNum];
      validAndLocked = false;
      if ((*prgb)->tryLock())
      { // lock frame to avoid reuse by read thread
/*        if (vpic.palette == VIDEO_PALETTE_YUV420P)
        validAndLocked = (*praw)->valid;
        else*/
        validAndLocked = (*prgb)->valid;
        if (not validAndLocked)
        { // image not available or already used
          (*prgb)->unlock(); // release
//          n = 0; // count valid frames
/*          for (j = 0; j < RAW_IMAGEBUFFER_SIZE; j++)
          {
          if (((vpic.palette == VIDEO_PALETTE_YUV420P)
          and (raw[j]->valid))
          or
          ((vpic.palette == VIDEO_PALETTE_RGB24)
          and (raw[j]->valid)))
          n++; // count number of usable frames
        }*/
/*          if ((i % 200) == 199) // should not take more than 2 s (5 frames/sec)
          {
          printf("*** UCamDevice:: "
          "did not get image after %d ms (%d of %d buffers valid)\n",
          i, n, RAW_IMAGEBUFFER_SIZE);
        }*/
          Wait(0.02); // max 3 waits at 30 frames / sec
        }
      }
      i += 20; // add wait time in miliseconds
      // if not valid, then wait for up to 2 seconds
    } while (i < 2000 and not validAndLocked);
    //
    // image is always in YUV format with
    // 4:1:1 relation between Y, U and V (YUV 4:2:0 Planer)
    if (validAndLocked)
    { // image is valid and locked
      //imageNumber++;
      //result->imageNumber = imageNumber;
    }
    else
    {
      //praw = NULL;
      prgb = NULL;
      printf("UCamDevice:: did not get an image");
    }
  }
  return validAndLocked;
}

////////////////////////////////////////////////

bool UCamDevBase::getImageSnapshot(UImage * image) //, bool remRadError)
{ // this is default behaviour - especially for PWC devices
  UImage * raw;
  bool result;
  //
  result = getLockedNewImage(&raw);
  if (result)
  { // copy content to users buffer
    if (image != NULL)
    {
      result = image->copy(raw);
    }
    // image should not be reused
    raw->valid = false;
    // unlock image
    raw->unlock();
  }
  return result;
}

////////////////////////////////////////////

bool UCamDevBase::needNewPushData()
{
  bool result = false;
  //
  if (cam != NULL)
    result = cam->needNewPushData();
  //
  return result;
}

/////////////////////////////////////////////

void UCamDevBase::setLog(bool value)
{
  if (varLog != NULL)
    varLog->setBool(value, 0, false);
}

/////////////////////////////////////////////

void UCamDevBase::setReplay(bool value)
{
  if (varReplay != NULL)
    varReplay->setBool(value, 0, false);
}

void UCamDevBase::setTypeName(const char * newName)
{ // space for 32 characters in camera name
  varCamName->setValues(newName, 0, true);
  int n = 0;
  while (newName[n] >= ' ' and n < (MAX_CAM_DEV_NAME_LENGTH - 1))
  { // name reduced to first line 
    camName[n] = newName[n];
    n++;
  }
  camName[n] = '\0';
};
