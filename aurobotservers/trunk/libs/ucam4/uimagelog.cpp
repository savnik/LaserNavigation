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

#include <ugen4/conf.h>
#include <ugen4/ucommon.h>
#include <ugen4/uimage2.h>

#include "uimagelog.h"
#include "ucammount.h"

UImageLog::UImageLog()
{
  //imgBuf = new UImage800();
  inPNG = true;
  saveImg = true;
}

///////////////////////////////////////////

UImageLog::~UImageLog()
{
  lock();
  closeLog();
  //delete imgBuf;
  unlock();
}

///////////////////////////////////////////////////////

bool UImageLog::openLog(bool doOpen)
{
  bool result;
  if (isOpen())
    closeLogging();
  lock();
  result = ULLogFile::openLog(doOpen);
  unlock();
  return result;
}

///////////////////////////////////////////////////////

bool UImageLog::openLogging(const char * name)
{
  //
  //lock();
  // now open a new file
  bool isOK = openLog(name);
  if (isOK)
    printf("Logging images to %s\n", getLogFileName());
  else
    printf("Not logging, failed to open %s\n", getLogFileName());
  //unlock();
  //
  return isOK;
}

///////////////////////////////////////////////////////

void UImageLog::closeLogging()
{ // should probably be mutex-locked
  int i;
  bool locked = false;
  //
  if (isOpen())
  {
    for (i = 0; i < 30; i++)
    {
      locked = tryLock();
      if (locked)
        break;
      Wait(0.01);
    }
    // close locked or not
    closeLog();
    printf("Closed image snapshot log %s\n", getLogFileName());
    if (locked)
      unlock();
  }
}

//////////////////////////////////////////////////////////

bool UImageLog::logImage(UImage * img, UCamMounted * cam)
{
  bool result = false;
  const int MTL = 30;
  char st[MTL];
  const int MFL = MAX_FILENAME_LENGTH;
  char imgFN[MFL];
  const int MNL = MAX_IMG_NAME_SIZE;
  char imgName[MNL];
  bool usePNG = inPNG;
  int isOK;
  //
  lock();
  if (isLogOpen() and img != NULL and cam != NULL)
  { // convert image time to string
    img->imgTime.getForFilename(st, true);
    // include camera device and time in imagename
    snprintf(imgName, MNL, "%s%08lu-cam%02d-%s",
             img->getColorTypeString(), img->imageNumber, img->camDevice, st);
    img->setName(imgName);
    // convert image to RGB
    if (saveImg)
    {
      //imgBuf->copy(img);
      //img->toRGB(imgBuf);
  #ifdef IMAGE_NO_PNG
      usePNG = false;
  #endif
      if (not usePNG)
      {
        snprintf(imgFN, MNL, "%s/%s.bmp", imagePath, imgName);
        // save
        result = img->saveBMP(imgFN);
      }
      else
      {
  #ifndef IMAGE_NO_PNG
      snprintf(imgFN, MNL, "%s/%s.png", imagePath, imgName);
      // save
      result = img->savePNG(imgFN);
  #endif
      }
    }
    else
    { // add file extension - assume png
      // image is probably saved by image client
      snprintf(imgFN, MNL, "%s.png", imgName);
    }
    //
    if (result)
    { // logfile open, note image data
      fprintf(getF(), "%lu.%06lu %lu %d %d %.2f %d %.3f %.3f %.3f %.4f %.4f %.4f %s\n",
              img->imgTime.getSec(), img->imgTime.getMicrosec(),
              img->imageNumber, img->getHeight(), img->getWidth(),
              cam->getCamPar()->getFocalLength(), cam->getDev()->getDeviceNumber(),
              cam->getPos().x, cam->getPos().y, cam->getPos().z,
              cam->getRot().Omega, cam->getRot().Phi, cam->getRot().Kappa,
              basename(imgFN));
    }
    else
    { // failed to save image -- close logging -- try to write to log
      printf("******** failed to log image (no space?)\n");
      isOK = system("df");
      result &= isOK;
      toLog("******** failed to log image (no space?)");
      closeLogging();
    }
  }
  unlock();
  return result;
}

///////////////////////////////////

void UImageLog::toLog(const char * txt)
{
  lock();
  ULLogFile::toLog(txt);
  unlock();
}


