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
#include <urob4/uimagepool.h>

#include "uimagelog.h"
#include "ucampwc.h"
#include "ucamdevgrabber.h"
#include "ucamdevieee1394.h"
#include "ucamdevgige.h"
#include "ucamdevguppy.h"
#include "ucamdevreplay.h"
#include "ucomcam.h"
#include "ucampool.h"

#ifdef USE_GUPPY
#include <libraw1394/raw1394.h>
#include <dc1394/dc1394.h>
#endif

#define FIRST_REPLAY_DEVICE_NUMBER 7
#define FIRST_GRABBER_DEVICE_NUMBER 16
#define FIRST_USERCAM_DEVICE_NUMBER 18

void UCamPool::UCamPoolInit()
{
  int i;
  // set name and version
  setResID(getResID(), 200);
  camCnt = 0;
  cmdExe = NULL;
  verboseMessages = false;
  for (i = 0; i < MAX_MOUNTED_CAMERAS; i++)
    cam[i] = NULL;
  imageLog = new UImageLog();
  resVersion = getResVersion();
  createBaseVar();
  replaySetFileName("image.log");
  imageLog->setLogName("image");
  imgPool = NULL;
}

void UCamPool::createBaseVar()
{

  varUseGuppy = addVar("useGuppy", 0.0, "d", "Is Guppy firewire camera support enabled (DC1394 ver 2 compiled in)");
  varUseIeeeOld = addVar("useIeeeOld", 0.0, "d", "Is firewire using old libs (DC1394 ver 1 compiled in)");
  varUseGigE = addVar("useGigE", 0.0, "d", "Is GigE support enabled (compiled in)");
#ifdef USE_GUPPY
  varUseGuppy->setBool(true, 0);
#endif
#ifdef USE_IEEE1394
  varUseIeeeOld->setBool(true, 0);
#endif
#ifdef USE_GIGE
  varUseGigE->setBool(true, 0);
#endif
  // this class uses replay, so create status replay vars
  createReplayVar(getVarPool());
  varCamsCnt = addVar("camsCnt", 0.0, "d", "Number of detected cameras");
  varCams = addVar("cams", 0.0, "d", "Device numbers of the detected cameras");
}

/////////////////////////////////////////////////

UCamPool::~UCamPool()
{
  int i;
  for (i = 0; i < camCnt; i++)
    if (cam[i] != NULL)
    { // delete both device and camera
      delete cam[i]->getDev();
      delete cam[i];
    }
  delete imageLog;
}

/////////////////////////////////////////////////

bool UCamPool::findDevices()
{
  UCamDevBase * testDev = NULL;
  UCamPush * testCam = NULL;
  int i;
  const int MKL = 30;
  char key[MKL];
  const int MSL = 130;
  char s[MSL];
  const char * posName = "none";
  UPosition pos;
  URotation rot;
  UComCam settings;
  int panPos, tiltPos;
  UVarPool * vpd, vpnew;
  enum UCamType {USB, REPLAY, IEEE1394, GIGE, GRABBER};
  UCamType camType;
  //
  lock();
  camType = USB;
  for (i = 0; i < MAX_MOUNTED_CAMERAS; i++)
  { // test if device with this number is known already
    switch (camType)
    { // set new device number within allocated number range
      case REPLAY:
        if (i < FIRST_REPLAY_DEVICE_NUMBER)
          i = FIRST_REPLAY_DEVICE_NUMBER;
        break;
      case IEEE1394:
        if (i < FIRST_IEEE1394_DEVICE_NUMBER)
          i = FIRST_IEEE1394_DEVICE_NUMBER;
        break;
      case GIGE:
        if (i < FIRST_GIGE_DEVICE_NUMBER)
          i = FIRST_GIGE_DEVICE_NUMBER;
        break;
      case GRABBER:
        if (i < FIRST_GRABBER_DEVICE_NUMBER)
          i = FIRST_GRABBER_DEVICE_NUMBER;
        break;
      default:
        break;
    }
    // is this device number available already
    testCam = getCam(i);
    if (testCam == NULL)
    { // not known to exist, so try make a new camera device
      switch (camType)
      { // need new structure
        case USB:
          testDev = new UCamPwc();
          break;
        case REPLAY:
          testDev = new UCamDevReplay();
          break;
        case IEEE1394:
#ifdef USE_GUPPY
            testDev = new UCamDevGuppy();
#else
            testDev = new UCamDevIeee1394();
#endif
          break;
        case GIGE:
          testDev = new UCamDevGigE();
          break;
        case GRABBER:
          testDev = new UCamDevGrabber();
          break;
        default: break;
      }
      //
      if (testDev == NULL)
      {
        if (camType >= GRABBER)
          break;
        // continue with next type
        camType = UCamType(camType + 1);
        continue;
      }
      // camera is not set before, so test
      testDev->setDeviceNumber(i);
      if (testDev->deviceExist())
      { // try open this device number
        testDev->openDeviceDefault();
      }
      if (not testDev->isCameraOpen())
        // no such camera type - advance to next
        camType = UCamType(camType + 1);
      else
      { // camera can be opened - register
        // debug print
        // printf("New device %d : %s\n", i, testDev->getCameraName());
        // debug end
        if (true)
        { // create a camera for the device
          testCam = new UCamPush(testDev);
          // allow push commands
          testCam->setCmdExe(cmdExe);
          // set image logger
          testCam->setImageLogging(imageLog);
          // get default lens and position data from camera type
          snprintf(key, MKL, "device%d", i);
          // load relevant settings to camera structure.
          settings.setFromCam(testDev, true, true);
          vpd = getVarPool()->getStruct(key);
          if (vpd == NULL)
          {
            snprintf(s, MSL, "a %s camera device", testDev->getCameraName());
            vpd = addStruct(key, s, true);
            if (vpd == NULL)
              printf("Failed to make global variable structure for camera\n");
          }
          if (vpd != NULL)
            testCam->setVarPool(vpd);
          //vpd->addVar("open", 0.0, "d", "is device open");
          //vpd->addVar("imageCnt", 0.0, "d", "number of images fetched");
          // set also pan-tilt position
          if (i < 7 and testCam->setPantiltStatus())
          { // reset to home
            testCam->pantiltToHomePosition();
            panPos = 0; // ini.intGet(posName, "panPos", 0);
            tiltPos = 0; //ini.intGet(posName, "tiltPos", 0);
            // debug
            // printf("Cam %s 1 pan %d, tilt %d\n", posName,
            //       testCam->getPanPos(), testCam->getTiltPos());
            // debug end
            if ((panPos != 0) or (tiltPos != 0))
            {
              Wait(3.5);
              testCam->pantiltSetPosition(false, panPos, tiltPos);
              // debug
              printf("Cam %s 2 pan %d, tilt %d\n", posName,
                    testCam->getPanPos(), testCam->getTiltPos());
              // debug end
            }
          }
        }
        testDev->closeDevice();
        testDev->setInitialized(true);
        // put in pool
        // set device list
        varCams->setInt(i, camCnt, true);
        cam[camCnt++] = testCam;
        varCamsCnt->setInt(camCnt);
        testDev = NULL;
      }
      if (testDev != NULL)
      {
        delete testDev;
        testDev = NULL;
      }
    }
  }
  //
  //
  unlock();
  if (verboseMessages)
    print("Cams:");
  //
  return camCnt > 0;
}

//////////////////////////////////////////////////

UCamPush * UCamPool::makeDevice(int devNum, const char * posName)
{
  bool result = true;
  UCamPush * testCam = NULL;
  int n;
  //Uconfig ini;
  UCamDevBase * dev = NULL;
  UVarPool * vpd;
  const int MKL = 30;
  char key[MKL];
  const int MSL = 100;
  char s[MSL];
  //
  lock();
  //ini.clear();
  // read camera configuration from config-file
  //err = ini.readConfig(configFileCam);
  //result = (err == 0);
  //
  if (posName != NULL)
  {
    testCam = getCam(posName);
    if (testCam != NULL)
    {
      n = testCam->getDev()->getDeviceNumber();
      if (n != devNum)
        printf("Device name error failed to make new device %s (%d) but with number %d!\n",posName, n, devNum);
      result = false;
    }
  }
  if (result)
  {
    testCam = getCam(devNum);
    result = (testCam == NULL);
    if (not result)
    { // camera known already
      printf("Trying to create a known device - device %d %s exists already!\n",
             devNum, testCam->getDev()->getCameraName());
    }
  }
  if (result and (camCnt < MAX_MOUNTED_CAMERAS))
  { // make new camera of type specified by device number
    if (devNum < 7)
      dev = new UCamPwc();
    else if (devNum < 10)
      dev = new UCamDevReplay();
    else if (devNum < 14)
#ifdef USE_GUPPY
      dev = new UCamDevGuppy();
#else
      dev = new UCamDevIeee1394();
#endif
    else if (devNum < FIRST_GRABBER_DEVICE_NUMBER)
      dev = new UCamDevGigE();
    else if (devNum < FIRST_USERCAM_DEVICE_NUMBER)
      dev = new UCamDevGrabber();
    else
      dev = new UCamDevReplay();
    dev->setDeviceNumber(devNum);
    if (not dev->deviceExist())
    {
      if (not dev->isThisA(UCamDevBase::CAM_DEV_REPLAY))
      { // not a real device, so make a replay device
        delete dev;
        dev = new UCamDevReplay();
        dev->setDeviceNumber(devNum);
      }
    }
    testCam = new UCamPush(dev);
    result = (testCam != NULL);
  }
  if (result)
  {
    testCam->setCmdExe(cmdExe);
    // get default lens and position data from camera type
    snprintf(key, MKL, "device%d", devNum);
    // load relevant settings to camera structure.
    vpd = getVarPool()->getStruct(key);
    if (vpd == NULL)
    {
      snprintf(s, MSL, "a %s camera device", dev->getCameraName());
      vpd = addStruct(key, s, true);
      if (vpd == NULL)
        printf("Failed to make global variable structure for camera\n");
    }
    testCam->setVarPool(vpd);
    if (posName != NULL)
      testCam->setMountName(posName);
    //testCam->loadCamSettings(&ini, posName);
    testCam->setImageLogging(imageLog);
    dev->closeDevice();
    dev->setInitialized(true);
    // put in pool
    // set device list
    varCams->setInt(devNum, camCnt, true);
    cam[camCnt++] = testCam;
    varCamsCnt->setInt(camCnt);
  }
  unlock();
  return testCam;
}

//////////////////////////////////////////////////

UCamPush * UCamPool::getCam(int device)
{
  UCamPush * result = NULL;
  int i;
  //
  for (i = 0; i < camCnt; i++)
  {
    result = cam[i];
    if (result != NULL)
    {
      if (device < 0)
        // use first device
        break;
      else if (device == result->getDev()->getDeviceNumber())
        break;
      else
        result = NULL;
    }
  }
  return result;
}

//////////////////////////////////////////////////

int UCamPool::getFirstCamDevice()
{
  int result = -1;
  //
  for (int i = 0; i < camCnt; i++)
  {
    if (cam[i] != NULL)
      result = cam[i]->getDev()->getDeviceNumber();
  }
  return result;
}

//////////////////////////////////////////////////

UCamPush * UCamPool::getCam(const char * posName)
{
  UCamPush * result = NULL;
  int i;
  const char * m = NULL;
  //
  for (i = 0; i < camCnt; i++)
  {
    result = cam[i];
    if (result != NULL)
    {
      m = result->getPosName();
      if (strcasecmp(posName, m) == 0)
        break;
      else
        result = NULL;
    }
  }
  return result;
}

///////////////////////////////////////////////////////

void UCamPool::print(const char * preString)
{
  const int MSL = 2000;
  char s[MSL];
  //
  print(preString, s, MSL);
  printf("%s", s);
}

///////////////////////////////////////////////////////

const char * UCamPool::print(const char * preString, char * buff, int buffCnt)
{
  int i, m = 0;
  UCamPush * c;
  const int SL = 80;
  char s[SL];
  char * p1 = buff;
  //
  snprintf(buff, buffCnt, "%s %d camera device(s) (max %d)\n", preString, camCnt, MAX_MOUNTED_CAMERAS);
  for (i = 0; i < camCnt; i++)
  {
    m += strlen(p1);
    p1 = &buff[m];
    c = cam[i];
    if (c != NULL)
    {
      snprintf(s, SL, " - cam#%d ", i);
      c->print(s, p1, buffCnt - m);
    }
    else
      snprintf(p1, buffCnt - m, " - cam#%d - not valid\n", i);
  }
  return buff;
}

///////////////////////////////////////////////////////

void UCamPool::flushClientCmds(int clientIdx)
{
  int i;
  for (i = 0; i < camCnt; i++)
    cam[i]->flushClientCmds(clientIdx);
}

///////////////////////////////////////////////////////

bool UCamPool::saveSettings(Uconfig * iniBase)
{
/*  int err = 0;
  Uconfig * ini = iniBase;
  Uconfig iniData;
  const int MKL = 30;
  char key[MKL];
  char val[MKL];
  UCamPush * cam;
  UCamPush * cam2 = NULL;
  int i, j;
  bool nameConflict;
  //
  if (ini == NULL)
  { // no open config file, so open default
    ini = &iniData;
    ini->clear();
    err = ini->readConfig(configFileCam);
  }
  // read camera configuration from config-file
  if (err == 0)
  {
    for (i = 0; i < MAX_MOUNTED_CAMERAS; i++)
    {
      cam = getCam(i);
      if (cam != NULL)
      {
        nameConflict = false;
        for (j = i - 1; j >= 0; j--)
        {
          cam2 = getCam(j);
          if (cam2 != NULL)
            nameConflict =
                (strcasecmp(cam->getPosName(), cam2->getPosName()) == 0);
          if (nameConflict)
            break;
        }
        if (nameConflict)
          snprintf(key, MKL, "pos%d", cam->getDev()->getDeviceNumber());
        else
          strncpy(key, cam->getPosName(), MKL);
        snprintf(val, MKL, "device%d", cam->getDev()->getDeviceNumber());
        ini->strPut("devices", val, key);
        cam->saveCamSettings(ini, key);
      }
    }
  }
//  if ((err == 0) and (iniBase == NULL))
  { // local ini-file, so save
    ini->saveConfig();
  }
  return  (err == 0);*/
  printf("UCamPool::saveSettings: camera configuration no longer saved - use '.ini' file to configure\n");
  return false;
}

///////////////////////////////////////////////////////

bool UCamPool::openImageLogging(const char * name)
{
  bool result = false;
  //
  if (imageLog != NULL)
    if (not imageLog->isOpen())
      result = imageLog->openLogging(name);
  //
  return result;
}

///////////////////////////////////////////////////////

void UCamPool::closeImageLogging()
{ // should probably be mutex-locked
  if (imageLog != NULL)
    if (imageLog->isOpen())
      imageLog->closeLogging();
}

/////////////////////////////////////////////////////

bool UCamPool::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result;
  result = (cmdExe != NULL);
  if ((not result) and (missingThese != NULL))
    snprintf(missingThese, missingTheseCnt, " %s", UCmdExe::getResClassID());
  return result;
}

//////////////////////////////////////////////////////////////////

bool UCamPool::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  int i;
  UImage * img;
  //
  if (resource->isA(UCmdExe::getResClassID()))
  { // ressource may change
    lock();
    if (remove)
      cmdExe = NULL;
    else if (cmdExe != (UCmdExe *)resource)
      cmdExe = (UCmdExe *)resource;
    else
      result = false;
    if (result)
    {
      for (i = 0; i < MAX_MOUNTED_CAMERAS; i++)
        if (cam[i] != NULL)
          cam[i]->setCmdExe(cmdExe);
    }
    unlock();
  }
  else if (resource->isA(UImagePool::getResClassID()))
  {
    lock();
    if (remove)
      imgPool = NULL;
    else if (imgPool != (UImagePool *)resource)
      imgPool = (UImagePool *)resource;
    else
      result = false;
    if (result)
    {
      for (i = 0; i < MAX_MOUNTED_CAMERAS; i++)
      {
        if (cam[i] != NULL)
        {
          if (remove)
            cam[i]->getDev()->setPushBuffer(NULL);
          else
          {
            img = imgPool->getImage(cam[i]->getDevNum(), true);
            if (img != NULL)
              cam[i]->getDev()->setPushBuffer(img);
          }
        }
      }
    }
    unlock();
  }
  else
    result = false;
  return result;
}

////////////////////////////////////////////////////////


bool UCamPool::decodeReplayLine(char * line)
{
  bool result = false;
  /*
  Replay images is in 'imgorg' subdirectory, and logfile has format:
  time              serial  height width focall device pose 6D x,y,z,O,P,K filename
  1266581865.438693 1965867 240 320 325.000000 0       0.4         0      0.87         0       0.3         0 /rhome/demo/log2/cam/img01965867-cam00-20100219_131745.438.bmp
  1266581866.437555 1965877 240 320 325.000000 0       0.4         0      0.87         0       0.3         0 /rhome/demo/log2/cam/img01965877-cam00-20100219_131746.437.bmp
  */
  const char * p1;
  char *p2;
  const int MNL = 100;
  char iname[MNL];
  UTime iTime;
  //double a, f;
  unsigned int sn;
  bool aBmp, aPng;
  int n, w, h, devNum;
  UImage * img;
  UImagePool * imgPool;
  UPosRot pose;
  const int MFL = 500;
  char fname[MFL];
  UCamMounted * camdev;
  bool isOK;
  UTime replayTime;
  //
  iTime.setTimeTod(replayLine);
  if (iTime.valid)
  {
    p1 = line;
    replayTime.setTimeTod(line);
    strtod(p1, &p2);
    sn = strtol(p2, &p2, 10);
    h = strtol(p2, &p2, 10);
    w = strtol(p2, &p2, 10);
    strtod(p2, &p2);
    devNum = strtol(p2, &p2, 10);
    pose.x = strtod(p2, &p2);
    pose.y = strtod(p2, &p2);
    pose.z = strtod(p2, &p2);
    pose.Omega = strtod(p2, &p2);
    pose.Phi = strtod(p2, &p2);
    pose.Kappa = strtod(p2, &p2);
    // get just filename from full path
    p2++;
    p2 = basename(p2);
    p1 = strrchr(p2, '.');
    aBmp = strncmp(p1, ".bmp", 4) == 0;
    aPng = strncmp(p1, ".png", 4) == 0;
    // look for first '/' to separate the filename
    if (aBmp or aPng)
    {
      strncpy(iname, p2, MNL);
      n = mini(p1 - p2 + 4, MNL - 1);
      iname[n] = '\0';
      imgPool = (UImagePool*) getStaticResource("imgPool", false, false);
      if (imgPool != NULL)
      { // there is a place for the image
        img = imgPool->getImage(devNum, true, h, w, 3, 8);
        // create fill folename from log-path
        snprintf(fname, MFL, "%s/imgorg/%s", replayPath, iname);
        if (aPng)
          isOK = img->loadPNG(fname);
        else
          isOK = img->loadBMP(fname);
        if (isOK)
        {
          img->setName(iname);
          img->imageNumber = sn;
          img->imgTime.setTimeTod(line);
          img->updated();
        }
      }
      camdev = getCam(devNum);
      if (camdev != NULL)
      { // set also camera pose - if not unset
        if (pose.getPos()->dist() > 0.001 or fabs(pose.Omega) > 0.001 or
            fabs(pose.Phi) > 0.001 or fabs(pose.Kappa) > 0.001)
        { // pose is not zero
          if (camdev->getPos().dist() < 0.01)
            camdev->setPosOnRobot(pose.getPos(), pose.getRot());
        }
      }
    }
    result = true;
  }
  return result;
}


