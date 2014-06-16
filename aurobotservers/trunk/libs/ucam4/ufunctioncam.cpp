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

#include <urob4/ucmdexe.h>

#include "ucampush.h"
#include "ufunctioncam.h"
#include "ucomcam.h"

UFunctionCam::UFunctionCam()
 : UFunctionCamBase()
{
  setCommand("camGet camSet camPush camsGet",
             "camCtrl", "interface to camera settings (by jca " __DATE__ " " __TIME__ ")");
}

///////////////////////////////////////////////////

// UFunctionCam::UFunctionCam(UCamPool * cams, UImagePool * images)
//  : UFunctionCamBase(cams, images)
// {
// }

///////////////////////////////////////////////////

UFunctionCam::~UFunctionCam()
{
}

///////////////////////////////////////////////////

bool UFunctionCam::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("camGet"))
    result = handleCamGetCommand(msg);
  else if (msg->tag.isTagA("camSet"))
    result = handleCamSetCommand(msg);
  else if (msg->tag.isTagA("camPush"))
    result = handleCamPushCommand(msg);
  else if (msg->tag.isTagA("camsGet"))
    result = handleCamsGetCommand(msg);
  else
    sendDebug(msg, "Command not handled (by me)");
  return result;
}

///////////////////////////////////////////////////

bool UFunctionCam::handleCamSetCommand(UServerInMsg * msg)
{
  bool result = true;
  UComCamSml camValues;
  UCamMounted * cam = NULL;
  const int MRL = 2000;
  char reply[MRL];
  bool ask4help;
  char * p1;
  bool trigger;
  bool triggerExt;
  bool triggerExtVal;
  bool logImg;
  bool logImgValue;
  bool replay;
  bool replayValue;
  bool replayStep;
  int replayStepValue;
  bool silent = false;
  //
  camValues.clear();
  ask4help = camValues.unpack(msg);
  trigger = msg->tag.getAttBool("trigger", NULL, true);
  triggerExt = msg->tag.getAttBool("triggerExt", &triggerExtVal, true);
  logImg = msg->tag.getAttBool("log", &logImgValue, true);
  replay = msg->tag.getAttBool("replay", &replayValue, true);
  replayStep = msg->tag.getAttInteger("step", &replayStepValue, true);
  msg->tag.getAttBool("silent", &silent, true);
  if (ask4help)
  {
    sendHelpStart(msg, "CAMSET");
    sendText( msg, "---- Available CAMSET functions (if supported by cam):\n");
    sendText( msg, "device=N            Set device 'N' (default is first device)\n");
    sendText( msg, "channel=N           Set data channel on this device (if supp.)\n");
    sendText( msg, "posName=name        Set camera 'name', e.g. left for left camera\n");
//    sendText( msg, "inCharge            Force you to be in charge of settings\n");
    sendText( msg, "gamma=X brightness=X colour=X contrast=X  Video controls (def.=32767)\n");
    sendText( msg, "width=W             Set image size to W (e.g. 160, 320 or 640)\n");
    sendText( msg, "fps=FR              Set frame rate (e.g 3.75, 15, 30)/sec\n");
    sendText( msg, "gain=X              Set gain -1 is automatic (else 1 to 65535)\n");
    sendText( msg, "shutter=X           Set shutter, -1 is automatic (else 3000 to 65535)\n");
    sendText( msg, "contour=X           Set edge enhancement 0 is none, C=65535 is max\n");
    sendText( msg, "whiteBal=auto | indoor | outdoor | manual | flurocent\n");
    sendText( msg, "whiteBalRed=RG whiteBalBlue=BG  Gain in manual mode (green ref 16384)\n");
    sendText( msg, "close open          Close or open camera explicitly\n");
    sendText( msg, "posX=xm posY=ym posZ=zm  Set camera position on robot in meter\n");
    sendText( msg, "rotOmega=rx rotPhi=ry rotKappa=rz Set rotation of axis in radians\n");
    sendText( msg, "serial=S            Set serial number (unsigned)\n");
    sendText( msg, "focalLength=pix K1=k1 K2=k2 headX=hx headY=hy  Set camera params\n");
    sendText( msg, "                    NB! values should be valid for 640x480 \n");
    sendText( msg, "panTiltHome         Pan-tilt set to home position\n");
    sendText( msg, "panPos=X tiltPos=Z  Set pan or tilt position absolute (deg*100)\n");
    sendText( msg, "panRel=X tiltRel=Z  Set pan or tilt position relative (deg*100)\n");
    sendText( msg, "trigger             Trigger new image(s) - if extrenal trigger is on\n");
    sendText( msg, "triggerExt[=false]  Set camera to external trigger\n");
    sendText(      "log[=false]         Set images from this device=N to be logged\n");
    snprintf(reply, MRL,"logPng[=false]      if false, then images are logged as .bmp files (logpng is %s)\n", bool2str(camPool->isLogPng()));
    sendText(reply);
    snprintf(reply, MRL,"replay[=false]      If true, all images in image.log will be replayed to "
                                        "image-pool (no logging and no campush on replayed images) (is %s)\n",
                                        bool2str(camPool->isReplay()));
    sendText(reply);
    sendText(      "step[=N]            step one image in logfile (logfile is image.log)\n");
    sendText( msg, "silent              Removes most replay to commands\n");
    sendText( msg, "---\n");
    sendText( msg, "See also CAMGET, CAMSGET and CAMPUSH\n");
    sendText( msg, "See also 'var campool' for device list and replay status\n");
    sendHelpDone(msg);
  }
  else if (replayStep)
  { // global functions
    replayStepValue = camPool->replayStep(replayStepValue);
    if (replayStepValue < 0)
      sendWarning("Replay step failed");
    else if (not silent)
    {
      UTime t = camPool->replayTimeNow;
      snprintf(reply, MRL, "Stepped to %lu.%06lu line %d in image.log", t.getSec(), t.GetMicrosec(), replayStepValue);
      sendInfo(reply);
    }
  }
  else
  { // device specific commands
    // get requested camera
    if (camValues.deviceValid)
      cam = camPool->getCam(camValues.deviceValue);
    else if (camValues.posNameValid)
      cam = camPool->getCam(camValues.posName);
    if (cam == NULL)
    { // create replay device if both
      // device and pos-name is valid
      if (camValues.deviceValid) // and camValues.posNameValid)
      {
        if (camValues.posNameValid)
          p1 = camValues.posName;
        else
          p1 = NULL;
        cam = camPool->makeDevice(camValues.deviceValue, p1);
      }
    }
    if (cam == NULL)
      // default is device first device
      cam = camPool->getCam(-1);
    // implement
    result = (cam != NULL);
    if (result)
    { // set in charge if requested
      bool bval = true;
      if (logImg)
      {
        cam->getDev()->setLog(logImgValue);
        if (logImgValue and not camPool->isImagelogOpen())
          camPool->openImageLogging();
      }
      if (msg->tag.getAttBool("logPng", &bval, true))
      { // set saved imageformat to PNG (else BMP)
        camPool->setLogPng(bval);
      }
      if (replay)
      {
        cam->getDev()->setReplay(replayValue);
        // set also general replay flag
        camPool->setReplay(replayValue);
      }
      if (camValues.inCtrlValid)
        cam->setClientInCharge(msg->client);
      // removed anoing incharge limitation
      //if (cam->isClientInCharge(msg->client))
      { // set all but trigger
        camValues.setCamDevice(cam);
        if (triggerExt)
          cam->getDev()->setExternalTrigger(triggerExtVal, NULL);
        if (trigger)
          cam->getDev()->makeTriggerPulse();
      }
/*      else
      {
        camValues.inCtrlValid = true;
        camValues.inCtrlValue = false;
        sendWarning(msg, "You are not in charge of device - (try 'camset inCharge' first)");
      }*/
    }
    if (not silent)
    { // send return message
      reply[0] = 0;
      if (result)
      {
        if ((trigger or triggerExt or logImg) and cam->getDev()->isCameraOpen())
          sendInfo("done");
        else if (cam->getDev()->isCameraOpen())
          sendStatusReply(msg, &camValues, cam);
        else
          snprintf(reply, MRL, "<%s result=\"%s\" camopen=\"%s\"/>\n",
            msg->tag.getTagName(),
            bool2str(true or cam->isClientInCharge(msg->client)),
            bool2str(cam->getDev()->isCameraOpen()));
      }
      else
        snprintf(reply, MRL, "<%s result=\"false\" msg=\"no such camera\"/>\n",
            msg->tag.getTagName());
      // send reply
      if (strlen(reply) > 0)
        sendMsg(msg, reply);
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////////

bool UFunctionCam::handleCamGetCommand(UServerInMsg * msg)
{
  bool result = true;
  UComCamSml camValues;
  UCamMounted * cam = NULL;
  const int MRL = 100;
  char reply[MRL];
  bool ask4help;
  //
  camValues.clear();
  if (msg->tag.getAttValue("all", NULL, 0))
  { // request all metric values
    camValues.relPosValid = true;
    camValues.relRotValid = true;
    camValues.sizeValid = true;
    camValues.camParValid = true;
  }
  // unpack other attributes (e.g. device number)
  ask4help = camValues.unpack(msg);
  if (ask4help)
  {
    sendHelpStart(msg, "CAMGET");
    sendText( msg, "device=N      from device 'N' (default is first device)\n");
    sendText( msg, "posName=name  from camera 'name', e.g. left for left camera\n");
    sendText( msg, "inCharge      Get in-charge (of cam settings) status\n");
    sendText( msg, "name          Get device name and position name\n");
    sendText( msg, "video         Get video control settings\n");
    sendText( msg, "channel       Get video data channel and name (1=tv 3=video)\n");
    sendText( msg, "size          Get image size\n");
    sendText( msg, "fps           Get framerate\n");
    sendText( msg, "gain          Get gain\n");
    sendText( msg, "shutter       Get shutter setting (if not automatic)\n");
    sendText( msg, "contour       Get edge enhancement setting 0=none\n");
    sendText( msg, "whiteBal      Get white balance settings\n");
    sendText( msg, "open          Get open status\n");
    sendText( msg, "pos rot       Get camera relative position (m) and rotation (rad)\n");
    sendText( msg, "serial        Get current image serial number\n");
    sendText( msg, "focal K1 K2   Get camera internal orientation / radial error etc.\n");
    sendText( msg, "panTilt       Get pan-tilt information\n");
    sendText( msg, "all           Get pos, rot, size and camera lens data information\n");
    sendText( msg, "---\n");
    sendHelpDone(msg);
  }
  else
  { // get requested camera
    if (camValues.deviceValid)
      cam = camPool->getCam(camValues.deviceValue);
    else if (camValues.posNameValid and (strlen(camValues.posName) > 0))
      cam = camPool->getCam(camValues.posName);
    else
      cam = camPool->getCam(-1);
    // implement
    result = (cam != NULL);
    if (result)
      sendStatusReply(msg, &camValues, cam);
    else
    {
      // camera not found
      snprintf(reply, MRL, "<%s result=\"false\" msg=\"no such camera\"/>\n",
          msg->tag.getTagName());
      // send reply
      sendMsg(msg, reply);
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////

bool UFunctionCam::sendStatusReply(UServerInMsg * msg,
                                   UComCamSml * camValues,
                                   UCamMounted * cam)
{
  const int MRL = 500;
  char reply[MRL];
  int n;
  //
  snprintf(reply, MRL, "<%s",
      msg->tag.getTagName());
  // add attributes
  n = strlen(reply);
  camValues->pack(cam, msg->client, &reply[n], MRL-n);
  // terminate message
  n = strlen(reply);
  if (strlen(camValues->notUsedPars) > 0)
  {
    snprintf(&reply[n], MRL-n, " unknown=\"%s\"", camValues->notUsedPars);
    n = strlen(reply);
  }
  snprintf(&reply[n], MRL-n, "/>\n");
  // send reply  reply[89]
  return sendMsg(msg, reply);
}

////////////////////////////////////////////////////////////

bool UFunctionCam::handleCamPushCommand(UServerInMsg * msg)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  int n;
  bool isAFlush = false;
  const int MRL = 1000;
  char reply[MRL];
  int deviceNum = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  UCamPush * cam;
  bool ask4help = false;
  bool ask4list = false;
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcmp(attName, "flush") == 0)
    { // flush
      isAFlush = true;
    }
    else if ((strcasecmp(attName, "device") == 0) or
              (strcasecmp(attName, "camera") == 0) or
              (strcasecmp(attName, "posName") == 0))
    { // either number or string
      n = sscanf(attValue, "%d", &deviceNum);
      if ((n != 1) and (strlen(attValue) > 0))
        strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
    }
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "list") == 0)
      ask4list = true;
    // else ignore attribute
  }
  //

  if (ask4help)
  {
    sendHelpStart(msg, "CAMPUSH");
    sendText( msg, "------------- Available CAMPUSH settings:\n");
    sendText( msg, "device=N        Push from device 'N'\n");
    sendText( msg, "posName=name    Push from camera 'name', e.g. left for left camera\n");
    sendText( msg, "good=k or g=k   Stop push after k good images (def: no stop)\n");
    sendText( msg, "total=k or n=k  Stop push after k images (def: no stop)\n");
    sendText( msg, "interval=k or i=k Push with this interval (in images, def = 1)\n");
    sendText( msg, "flush=cmd       Remove all push commands from client (default all)\n");
    sendText( msg, "cmd=\"cmd\"       Do execute 'cmd' command for every available image\n");
    sendText( msg, "call=method(pr) Do call method when new image is available\n");
    sendText( msg, "list            List active push commands\n");
    sendText( msg, "---\n");
    sendHelpDone(msg);
  }
  else
  {
    // get requested camera
    if (deviceNum >= 0)
      cam = camPool->getCam(deviceNum);
    else if (strlen(posName) > 0)
      cam = camPool->getCam(posName);
    else
      // default is first device
      cam = camPool->getCam(-1);
    if (cam != NULL)
    {
      if (ask4list)
      {
        sendHelpStart(msg, "camPush command list");
        cam->UServerPush::print("camPush", reply, MRL);
        sendText(msg, reply);
        sendHelpDone(msg);
      }
      else
      {
        n = cam->addCamPushCommand(msg);
        // ensure camera is open (else no images)
        if (not cam->getDev()->isCameraOpen())
          cam->getImageSnapshot(NULL); //, false);
        if (isAFlush)
        { // flush and send reply
          snprintf(reply, MRL, "<%s done=\"%s\" flushed=\"%d\"/>\n",
                msg->tag.getTagName(), bool2str(n > 0), n);
        }
        else
        { // a new push command
          snprintf(reply, MRL, "<%s done=\"%s\"/>\n",
                msg->tag.getTagName(), bool2str(n > 0));
        }
        sendMsg(msg, reply, strlen(reply));
      }
    }
    else
      sendWarning(msg, "Camera not found");
  }
  //
  return result;
}

////////////////////////////////////////////////////////////

bool UFunctionCam::handleCamsGetCommand(UServerInMsg * msg)
{
  bool result = false;
  bool ask4help = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  int n;
  bool getAll = false;
  bool foundCam = false;
  const int RL = 100;
  char reply[RL];
  int deviceNum = -1;
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  UCamPush * cam = NULL;
  //
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcmp(attName, "all") == 0)
      // flush
      getAll = true;
    else if ((strcasecmp(attName, "device") == 0) or
              (strcasecmp(attName, "camera") == 0) or
              (strcasecmp(attName, "posName") == 0))
    { // either number or string
      n = sscanf(attValue, "%d", &deviceNum);
      if ((n != 1) and (strlen(attValue) > 0))
        strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
    }
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendHelpStart(msg, "CAMSGET");
    sendText( msg, "---- Available CAMSGET options:\n");
    sendText( msg, "all            List all available (USB) video devices (and new)\n");
    sendText( msg, "device=N       Get name of (USB) video devices N\n");
    sendText( msg, "posName='left' Get name of device with position name 'left'\n");
    sendText( msg, "help           this message\n");
    sendText( msg, "---\n");
    sendHelpDone(msg);
  }
  if (getAll)
  {
    camPool->findDevices();
  }
  // default is device 0
  cam = camPool->getCamByPoolIndex(0);
  //
  if (not getAll)
  { // get a requested camera
    if (deviceNum >= 0)
      cam = camPool->getCam(deviceNum);
    else if (strlen(posName) > 0)
      cam = camPool->getCam(posName);
  }
  //
  for (n = 0; n < camPool->getDeviceCount(); n++)
  { // a new push command
    if (cam != NULL)
    {
      snprintf(reply, RL, "<%s device=\"%d\" name=\"%s\" posName=\"%s\" pushQueue=\"%d\"/>\n",
               msg->tag.getTagName(), cam->getDev()->getDeviceNumber(), cam->getDev()->getCameraName(),
            cam->getPosName(), cam->getPushCmdCnt(NULL, NULL));
      sendMsg(msg, reply, strlen(reply));
      foundCam = true;
    }
    if (not getAll)
      break;
    cam = camPool->getCamByPoolIndex(n+1);
  }
  if (not foundCam)
  {
    if (getAll)
      sendWarning(msg, "No cameras found");
    else
      sendWarning(msg, "Device not found");
  }
  //
  return result;
}

