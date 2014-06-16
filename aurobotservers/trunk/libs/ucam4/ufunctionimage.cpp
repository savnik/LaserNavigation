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
#include "ufunctionimage.h"
#include "ucampool.h"

UFunctionImage::UFunctionImage()
 : UFunctionCamBase()
{
  setCommand("imageGet", "image", "handles interface to image capture (by jca " __DATE__ " " __TIME__ ")");
}

// UFunctionImage::UFunctionImage(UCamPool * cams, UImagePool * images)
//  : UFunctionCamBase(cams, images)
// {
// }

UFunctionImage::~UFunctionImage()
{
}

///////////////////////////////////////////////////

bool UFunctionImage::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  //
  if (msg->tag.isTagA("imageGet"))
    result = handleImageGetCommand(msg, extra);
  else
    sendWarning(msg, "Command not handled (by me)");
  //
  return result;
}


////////////////////////////////////////////////////


bool UFunctionImage::handleImageGetCommand(UServerInMsg * msg, void * imgBase)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  char posName[MAX_MOUNT_NAME_SIZE] = "";
  const char * posGotName = NULL;
  UCamPush * cam = NULL;
  int imgWidth  = 0;
  int imgHeight = 0;
  int imgTop    = 0;
  int imgLeft   = 0;
  int imgDevice = -1; // video device
  int imgPoolNum = -1;
  bool imgGetTime = false;   // return image time
  bool imgGetTod = true;    // get time in timeofday format
  bool imgGetNum = false;    // return image number
  //bool imgRemRad = false; // remove radial error
  bool imgIsBW = false; // is requested image in BW only
  bool imgGetName = false;
  bool imgSave = false;
  bool ask4help = false;
  const int MNL = MAX_FILENAME_LENGTH;
  char imgName[MNL];
  char imgFN[MNL];
  const int MRL = 200;
  char reply[MRL];
  char s[MRL];
  double colourGain = 1.0;
  bool colourGainValid = false;
  bool silent = false;
  //URawImage * imgRaw = NULL;
  UImage * imgRgb = NULL;
  UImage * imgRgb2 = NULL;
  UTime imgTime;
  int rectImg = -1;
  bool doPoolImgCpy = false;
  //
  // debug
/*  if (not msg->tag.isTagEndFound())
  { // trap for deadlock error a "log" is found in message '<poolget img=9 save/>\n'
    printf("***unfinished tag?? '%s'\n", msg->message);
  }*/
  // debug end
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcasecmp(attName, "device") == 0)
      sscanf(attValue, "%d", &imgDevice);
    else if (strcasecmp(attName, "posName") == 0)
      strncpy(posName, attValue, MAX_MOUNT_NAME_SIZE);
    else if (strcasecmp(attName, "time") == 0)
      imgGetTime = str2bool(attValue)  or (strlen(attValue) == 0);
    else if (strcasecmp(attName, "all") == 0)
    {
      imgGetTime = true;
      imgGetTod = true;
      imgGetNum = true;
      imgGetTod = true;
      if (imgWidth == 0)
        imgWidth = 800;
      if (imgHeight == 0)
        imgHeight = 600;
    }
    else if (strcasecmp(attName, "tod") == 0)
    {
      imgGetTime = str2bool(attValue) or (strlen(attValue) == 0);
      imgGetTod = str2bool(attValue) or (strlen(attValue) == 0);
    }
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strncasecmp(attName, "serial", 3) == 0)
      imgGetNum = true;
//     else if (strncasecmp(attName, "remRad", 3) == 0)
//     {
//       if (strlen(attValue) == 0)
//         imgRemRad = true;
//       else
//         imgRemRad = str2bool(attValue);
//     }
    else if (strncasecmp(attName, "name", 3) == 0)
      imgGetName = true;
    else if (strncasecmp(attName, "width", 1) == 0)
      sscanf(attValue, "%d", &imgWidth);
    else if (strncasecmp(attName, "height", 1) == 0)
      sscanf(attValue, "%d", &imgHeight);
    else if (strcasecmp(attName, "y") == 0)
      sscanf(attValue, "%d", &imgTop);
    else if (strcasecmp(attName, "x") == 0)
      sscanf(attValue, "%d", &imgLeft);
    else if (strcasecmp(attName, "poolImg") == 0)
      sscanf(attValue, "%d", &imgPoolNum);
    else if (strcasecmp(attName, "save") == 0)
    {
      imgSave = true;
      strncpy(imgName, attValue, MNL);
    }
    else if (strcasecmp(attName, "log") == 0)
    {
      if (strcasecmp(attValue, "false") == 0)
        camPool->closeImageLogging();
      else if (strlen(attValue) == 0)
        camPool->openImageLogging();
      else
        camPool->openImageLogging(attValue);
    }
    else if (strcasecmp(attName, "colorGain") == 0)
    {
      colourGain = strtod(attValue, 0);
      colourGainValid = true;
    }
    else if (strcasecmp(attName, "logSaveImg") == 0)
      camPool->setLogSaveImg(str2bool(attValue));
    else if (strcasecmp(attName, "rectImg") == 0)
      rectImg = strtol(attValue, NULL, 0);
    else if (strcasecmp(attName, "silent") == 0)
      silent = str2bool2(attValue, true);
    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendMsg( msg, "<help subject=\"IMAGEGET help\">\n");
    sendText( msg, "--------- IMAGEGET options:\n");
    sendText( msg, "device=N      from device 'N' (n=0..30, default is first device\n");
//    sendText( msg, "posName=name  from camera 'name', e.g. left for left camera\n");
    sendText( msg, "time          get also image time in hh:mm:ss.ddd form\n");
    sendText( msg, "poolImg=D     Use pool image D as buffer for raw image (default=device number)\n");
    sendText( msg, "rectImg=U     Use pool image U as buffer for rectified image (U != D) (default is not rectified)\n");
//    sendText( msg, "tod           get time in sec.usec (timeofday format)\n");
    sendText( msg, "name          get also image name\n");
    sendText( msg, "all           get all pixels (default=none)\n");
    sendText( msg, "x=left y=top  get image from this top-left pixel position (default=0,0)\n");
    sendText( msg, "width=cols    get just 'cols' pixels wide image (default=0)\n");
    sendText( msg, "height=rows   get just 'rows' pixels high image (default=0)\n");
    sendText( msg, "serial        get image serial number\n");
    //sendText( msg, "remRad=true|false  Remove radial error (false on campush) default=false\n");
    sendText( msg, "codec=HEX|BIN|BMPBIN|BMPHEX coding og image (hex or binary (default=BIN) BMP adds BMP header)\n");
    sendText( msg, "colourGain=f  change the color gain before sending the image (default=1.0)\n");
    sendText( msg, "silent        do not reply to this command (especially for push commands)\n");
    //sendText( msg, "save=\"name\"   save image in servers default image path (.bmp)\n");
    if (camPool->isImagelogOpen())
    {
      snprintf(reply, MRL, "log=name|false  open logfile with name (is open='%s'), false=close\n",
               camPool->getImagelogFilename());
      sendText( msg, reply);
    }
    else
      sendText( msg, "log=name|false  open logfile with name (def=image.log), is closed\n");
    // save image while logging
    snprintf(reply, MRL, "logSaveImg=true|false  Save image while logging (%s) (else just log name)\n",
             bool2str(camPool->isLogSaveImg()));
    sendText( msg, reply);
    sendText( msg, "---\n");
     sendText( msg, "see also poolget and poollist for handling of acquired images\n");
   sendMsg( msg, "</help>\n");
    if (not silent)
      sendInfo(msg, "done");
  }
  else if (gotAllResources(s, MRL))
  { // get camera reference and image
    // source is a raw image either from camera or pushed
    if (imgDevice < 0)
    {
      if (imgBase != NULL)
        imgDevice = ((UImage*)imgBase)->camDevice;
      else
        imgDevice = getDefaultCamDevice();
    }
    // should image be copied to a image pool image
    doPoolImgCpy = (imgBase != NULL and imgPoolNum >= 0);
    // test if a camera exist
    result = imgDevice >= 0;
    if (result and imgPoolNum < 0)
      // missing image number - use device number
      imgPoolNum = imgDevice;
    if (result and imgBase == NULL)
    { // no image yet, so get buffer for image
      imgRgb = imgPool->getImage(imgPoolNum, true);
      result = imgRgb != NULL;
    }
    if (result)
      result = getCamAndRawImage(
                        &cam,         // result camera
                        &imgRgb,      // result image
                        &imgDevice,  // camera device number
                        //&imgRemRad,  // should/is radial error remoced
                        imgBase,     // pushed image (YUV)
                        posName,     // camera position name
                        rectImg
                        );
    //
    // debug
    if (not result and not silent)
      printf("ImageGet dev=%d result(%s)\n", imgDevice, bool2str(result));
    // debug end
    // all arameters are now checked
    if (result)
    { // get camera name
      if (cam != NULL)
        posGotName = cam->getPosName();
      if (colourGainValid)
        imgRgb->colourSaturate(colourGain);
      if (rectImg >= 0)
        imgPoolNum = rectImg;
      // send the image data
      result = sendImage(msg, //->client,
                  msg->tag.getTagName(),
                  imgRgb,  // image and format
                  imgLeft,     // left column to send
                  imgTop ,     // top row to send
                  imgWidth,    // max width
                  imgHeight,   // max height
                  imgPoolNum,         // image source - if from pool
                  true,        // default is binary coding
                  imgIsBW,         // send as BW-only
                  imgGetNum,       // send image number
                  imgGetTod,       // send time-of-day format
                  imgGetTime,      // send hh:mm:ss.ccc time format
                  posGotName, // send this camera position name
                  imgGetName,          // send image name (RGB format only)
                  silent
                  );
    }
    if (result and imgSave)
    {
      if (strlen(imgName) < 1)
      { // make image name
        snprintf(imgName, MNL, "img%08lu",
            imgRgb->imageNumber);
      }
      imgRgb->imgTime.getForFilename(attName, true);
      // save
      snprintf(imgFN, MNL, "%s/%s-cam%02d-%s.bmp",
                        imagePath, imgName, imgRgb->camDevice, attName);
      imgRgb2 = imgRgb;
      if (imgRgb2->saveBMP(imgFN))
      { // reply
        snprintf(reply, MRL, "Image saved as '%s' in default path", imgFN);
        if (not silent)
          sendInfo(msg, reply);
/*        if (logi != NULL)
        { // logfile open
          fprintf(logi, "%lu.%06lu %lu %d %d %f %9g %9g %9g %9g %9g %9g %s\n",
             imgRgb->imgTime.getSec(), imgRgb->imgTime.getMicrosec(),
             imgRgb->imageNumber, imgRgb->getHeight(), imgRgb->getWidth(),
             cam->getCamPar()->getFocalLength(),
             cam->getPos().x, cam->getPos().y, cam->getPos().z,
             cam->getRot().Omega, cam->getRot().Phi, cam->getRot().Kappa,
             imgFN);
        }*/
      }
      else
        sendWarning(msg, "Raw Image not saved");
    }
    //
    if (doPoolImgCpy)
    { // make an explicit copy to image pool number
      imgRgb2 = imgPool->getImage(imgPoolNum, true);
      if (imgRgb2 != NULL and imgRgb2 != imgRgb)
        imgRgb2->copy(imgRgb);
      imgRgb2->updated();
    }
    //
    if (not result and not silent)
    {
      if (not msg->serverPushCommand)
        sendWarning(msg, "No image available");
      else
        sendInfo(msg, "Image push failed - no image");
    }
  }
  else
  { // missing resources
    snprintf(reply, MRL, "Missing resources:%s", s);
    sendWarning(msg, reply);
  }
  //
  return result;
}


