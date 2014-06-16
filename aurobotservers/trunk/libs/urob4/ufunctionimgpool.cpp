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

#include <ugen4/uimage.h>
//#include <ugen4/urawimage.h>
#include "uimgpush.h"
#include "ufunctionimgpool.h"

///////////////////////////////////////////////////

UFunctionImgPool::~UFunctionImgPool()
{
  if (imgPool != NULL)
    delete imgPool;
}

///////////////////////////////////////////////////

void UFunctionImgPool::createResources()
{
  if (imgPool == NULL)
  {
    // initialize gstreamer
    // gst_init (&s_argc, &s_argv);
    // make image pool object
    imgPool = new UImagePool();
    // add image pool to server resource pool
    addResource(imgPool, this);
    // create image-pool variables
    createBaseVar();
  }
}

// const char * UFunctionImgPool::name()
// {
//   return "imgPool (" __DATE__ " jca@oersted.dtu.dk)";
// }

///////////////////////////////////////////////////

// const char * UFunctionImgPool::commandList()
// {
//   return "poolGet poolList poolSet poolPush";
// }

///////////////////////////////////////////////////

bool UFunctionImgPool::handleCommand(UServerInMsg * msg, void * extra)
{  // message is unhandled
  bool result = false;
  //
  switch (getCmdIndex())
  {
    case 0: // first command
      result = handleImageGetCommand(msg, extra);
      break;
    case 1:
      result = handleImageListCommand(msg);
      break;
    case 2:
      result = handleImageSetCommand(msg, extra);
      break;
    case 3:
      result = handlePoolPushCommand(msg);
      break;
    default:
      sendWarning(msg, "Command not handled (by me)");
      break;
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UFunctionImgPool::handleImageListCommand(UServerInMsg * msg)
{
  //URawImage * imgRaw;
  UImgPush * imgRgb;
  int i, j;
  bool found = false, result;
  const int MRL = 300;
  char reply[MRL];
  char s[MRL];
  UServerPushQueue * pq;
  UServerPushElement * pqe;
  //
  result = gotAllResources(s, MRL);
  if (not result)
  {
    snprintf(reply, MRL, "Missing resources:%s", s);
    sendWarning(msg, reply);
  }
  else
  {
    for (i = 0; i < imgPool->getImageCnt(); i++)
    {
      imgRgb = (UImgPush*)imgPool->getImage(i, false);
      // debug
      // printf("UFunctionImgPool::handleImageListCommand: img=%d (%d)\n",
      //        i, imgRgb);
      // debug end

      if (imgRgb != NULL)
      {
        pq = imgRgb->getPushQueue();
        snprintf(reply, MRL, "<%s img=\"%d\"  w=\"%d\" h=\"%d\" serial=\"%lu\" format=\"%s\" depth=\"%d\" channels=\"%d\" name=\"%s\"",
          msg->getTag()->getTagName(),
          i, imgRgb->width(), imgRgb->height(),
          imgRgb->imageNumber, imgRgb->getColorTypeString(), 
          imgRgb->getDepth(), imgRgb->getChannels(), imgRgb->name);
        if (pq->getPushCmdCnt() == 0)
          strncat(reply, "/>\n", MRL - strlen(reply));
        else
          strncat(reply, ">\n", MRL - strlen(reply));
        sendMsg(msg, reply);
        // send any queue elements
        for (j = 0; j < pq->getPushCmdCnt(); j++)
        {
          pqe = pq->get(j);
          if (pqe->activeCmd)
          {
            snprintf(reply, MRL, "<pushelement active=\"%s\" every=\"%g\" count=\"%d\" cmd=\"%s\"/>\n",
                    bool2str(pqe->activeCmd),
                    pqe->interval, pqe->countTotal,
                    pqe->toDo.message);
            sendMsg(msg, reply);
          }
          if (pqe->activeCall)
          {
            pqe->printCall("<pushelement ", reply, MRL - 4);
            strncat(reply, "/>\n", MRL);
            sendMsg(msg, reply);
          }
        }
        if (j > 0)
        {
          snprintf(reply, MRL, "</%s>\n", msg->getTag()->getTagName());
          sendMsg(msg, reply);
        }
        found = true;
      }
    }
    if (not found)
      sendWarning(msg, "No images in image pool");
  }
  return result;
}

////////////////////////////////////////////////////

bool UFunctionImgPool::handleImageGetCommand(UServerInMsg * msg, void * imgBase)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  unsigned int imgWidth  = 0;
  unsigned int imgHeight = 0;
  unsigned int imgTop    = 0;
  unsigned int imgLeft   = 0;
  //unsigned int w = 0, h = 0;
  int imgSource = -1; // image pool source
  bool imgGetTime = false;   // return image time
  bool imgGetTod = true;    // get time in timeofday format
  bool imgGetNum = false;    // return image number
  bool isFull = false; // image source
  bool imgGetName = true;
  bool imgSave = false;
  bool imgSavePng = false;
  bool imgSaveTxt = false;
  bool ask4help = false;
  bool imgDebug = false;
  bool silent = false;
  bool aToImg = false;
  int aToImgValue = 2;
  double aScaleValue = 1.0;
  bool aScale = false;
  bool aToFmt = false;
  char aToFmtValue[VAL_BUFF_LNG] = "BGR";
  bool aTest = false;
  int aTestValue = 4;
  const int MNL = 100;
  char imgName[MNL];
  const int MRL = 200;
  char reply[MRL];
  const int MFL = MAX_FILENAME_LENGTH;
  char filn[MFL] = "";
  //URawImage * imgRaw = NULL;
  UImage * imgRgb = NULL, *imgSrc, *imgCnv = NULL;
  UTime imgTime;
  char * p1;
  int imgRgbCreated = false;
  //

  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
    if (strcasecmp(attName, "time") == 0)
      imgGetTime = true;
    else if (strcasecmp(attName, "all") == 0)
    {
      imgGetTime = true;
      imgGetTod = true;
      imgGetNum = true;
      imgGetName = true;
      if (imgWidth == 0)
        imgWidth = 6400; // too big
      if (imgHeight == 0)
        imgHeight = 4800; // too big
    }
    else if (strcasecmp(attName, "tod") == 0)
    {
      imgGetTime = true;
      imgGetTod = str2bool2(attValue, true);
    }
    else if ((strncasecmp(attName, "pool", 4) == 0) or
             (strncasecmp(attName, "img", 2) == 0))
    {
      sscanf(attValue, "%d", &imgSource);
//      isRaw = false;
      isFull = true;
    }
/*    else if ((strncasecmp(attName, "poolRaw", 5) == 0) or
             (strncasecmp(attName, "raw", 3) == 0))
    {
      sscanf(attValue, "%d", &imgSource);
      isRaw = true;
      isFull = false;
    }*/
    else if ((strncasecmp(attName, "number", 3) == 0) or
             (strncasecmp(attName, "serial", 4) == 0))
      imgGetNum = true;
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strncasecmp(attName, "name", 3) == 0)
      imgGetName = str2bool2(attValue, true);
    else if (strcasecmp(attName, "w") == 0)
      sscanf(attValue, "%u", &imgWidth);
    else if (strcasecmp(attName, "h") == 0)
      sscanf(attValue, "%u", &imgHeight);
    else if (strcasecmp(attName, "y") == 0)
      sscanf(attValue, "%u", &imgTop);
    else if (strcasecmp(attName, "x") == 0)
      sscanf(attValue, "%u", &imgLeft);
    else if (strcasecmp(attName, "save") == 0)
    {
      imgSave = true;
      strncpy(imgName, attValue, MNL);
    }
    else if (strcasecmp(attName, "savePng") == 0)
    {
      imgSavePng = true;
      strncpy(imgName, attValue, MNL);
    }
    else if (strcasecmp(attName, "saveTxt") == 0)
    {
      imgSaveTxt = true;
      strncpy(imgName, attValue, MNL);
    }
    else if (strcasecmp(attName, "debug") == 0)
      imgDebug = str2bool(attValue);
    else if (strcasecmp(attName, "silent") == 0)
      silent = str2bool2(attValue, true);
    else if (strcasecmp(attName, "toImg") == 0)
    {
      aToImg = true;
      aToImgValue = strtol(attValue, NULL, 10);
    }
    else if (strcasecmp(attName, "scale") == 0)
    {
      aScale = true;
      aScaleValue = strtod(attValue, NULL);
    }
    else if (strcasecmp(attName, "fmt") == 0)
    {
      aToFmt = true;
      strncpy(aToFmtValue, attValue, VAL_BUFF_LNG);
    }
    else if (strcasecmp(attName, "test") == 0)
    {
      aTest = true;
      aTestValue = strtol(attValue, NULL, 10);
    }

    // else ignore attribute
  }
  //
  if (ask4help)
  {
    sendMsg( msg, "<help subject=\"POOLGET help\">\n");
    sendText( msg, "-------- Available POOLGET options\n");
    snprintf(reply, MRL, "img=num       from image pool number 'num' (default=0, max=%d)\n",
                  imgPool->getMaxImageCnt() - 1);
    sendText( msg, reply);
    sendText( msg, "time          get image time in hh:mm:ss.ddd form\n");
    sendText( msg, "tod           get time in sec.usec (timeofday) form\n");
    sendText( msg, "name          get image name (if available)\n");
    sendText( msg, "all           get all pixels (default=none)\n");
    sendText( msg, "x=left y=top  get image from this top-left pixel position (default=0,0)\n");
    sendText( msg, "width=cols    get just 'cols' pixels wide image (default=0)\n");
    sendText( msg, "height=rows   get just 'rows' pixels high image (default=0)\n");
    sendText( msg, "serial        get image serial number\n");
    sendText( msg, "remRad=true|false  Remove radial error (false on campush) default=true\n");
    sendText( msg, "codec=BIN|HEX|BMP|BMPHEX coding og image (hex or binary (default=BIN) BMP adds BMP header)\n");
    sendText( msg, "toImg=num     copy image to this image pool number\n");
    sendText( msg, "scale=factor  scale destination by this factor\n");
    sendText( msg, "fmt=colFmt    change color format to: RGB,BGR,gray,yuv,Bayer(gray)\n");
    sendText( msg, "save=\"name\"    save image in image path (.bmp)\n");
    sendText( msg, "savePNG=\"name\" save image in image path (.png)\n");
    sendText( msg, "saveTXT=\"name\" save pixel values in image path as text (.txt)\n");
    sendText( msg, "debug=true    Sets a debug flag, the effect is unknown\n");
    sendText( msg, "silent        no reply to client - (especially for push commands)\n");
    sendText( msg, "See also: POOLSET, POOLLIST, POOLPUSH and VER IMGPOOL\n");
    sendText( msg, "---\n");
    sendMsg( msg, "</help>\n");
    if (not silent)
      sendInfo(msg, "done");
  }
  else if (imgPool != NULL)
  { // get camera reference and image
    if (imgBase != NULL)
    { // is a pushed image
      imgRgb = (UImage*)imgBase;
      imgSource = imgRgb->source;
      result = imgRgb->valid;
    }
    else if (isFull)
    { // image is from image pool
      imgRgb = imgPool->getImage(imgSource, false);
      result = (imgRgb != NULL);
      if (result and imgDebug)
      { // change to full VGA size
//         w = imgRgb->width();
//         h = imgRgb->height();
        imgRgb->setSizeOnly(480, 640);
      }
    }
    //
    if (aTest)
    { // make poolimage a red square in BGGR format
      unsigned char * pp;
      for (int j = 0; j < 4; j++)
      {
        imgRgb = imgPool->getImage(imgSource + j, true);
        result = imgRgb != NULL;
        if (not result)
          continue;
        switch (j)
        {
          case 0: imgRgb->setSize(aTestValue, aTestValue, 1, 8, "BGGR"); break;
          case 1: imgRgb->setSize(aTestValue, aTestValue, 1, 8, "RGGB"); break;
          case 2: imgRgb->setSize(aTestValue, aTestValue, 1, 8, "GBRG"); break;
          case 3: imgRgb->setSize(aTestValue, aTestValue, 1, 8, "GRBG"); break;
        }
        for (int r = 0; r < aTestValue; r++)
        {
          pp = imgRgb->getUCharRef(r, 0);
          for (int c = 0; c < aTestValue; c+=2)
          {
            if (r % 2 == 0)
            { // blue green
              switch (j)
              {
                case 0: /*BG*/ pp[0] = 0x20; pp[1] = 0x77; break;
                case 1: /*RG*/ pp[0] = 0xee; pp[1] = 0x77; break;
                case 2: /*GB*/ pp[0] = 0x77; pp[1] = 0x20; break;
                case 3: /*GR*/ pp[0] = 0x77; pp[1] = 0xee; break;
              }
            }
            else
            { // green red
              switch (j)
              {
                case 0: /*GR*/ pp[0] = 0x77; pp[1] = 0xee; break;
                case 1: /*GB*/ pp[0] = 0x77; pp[1] = 0x20; break;
                case 2: /*RG*/ pp[0] = 0xee; pp[1] = 0x77; break;
                case 3: /*BG*/ pp[0] = 0x20; pp[1] = 0x77; break;
              }
            }
            pp += 2;
          }
        }
        imgRgb->imageNumber = 2077;
        imgRgb->imgTime.now();
        imgRgb->valid = true;
        imgRgb->setName("testimage-in-bggr");
        imgRgb->updated();
      }
    }
    // debug
    if (not result)
      printf("poolGet Full24bpp(%s) source=%d result(%s)\n",
                  bool2str(isFull), imgSource, bool2str(result));
    // debug end
    if (result and (aToFmt or (aScale and (imgRgb->isBayer() or imgRgb->isYUV420()))))
    { // scale only work on normal images
      int cf = imgRgb->toColFormatInt(aToFmtValue);
      if (strcasecmp(aToFmtValue, "bayer") == 0 and imgRgb->isBayer())
      {
        imgRgb->setColorType("gray");
      }
      else if (cf == PIX_PLANES_BW or cf == PIX_PLANES_RGB or
               cf == PIX_PLANES_BGR or cf == PIX_PLANES_YUV)
      {
        imgSrc = imgRgb;
        imgCnv = new UImage();
        switch (cf)
        {
          case PIX_PLANES_BW:
            imgSrc->toBW(imgCnv);
            break;
          case PIX_PLANES_RGB:
            imgSrc->toRGB(imgCnv);
            break;
          case PIX_PLANES_BGR:
            imgSrc->toBGR(imgCnv);
            break;
          case PIX_PLANES_YUV:
            imgSrc->toYUV(imgCnv);
            break;
        }
        imgRgb = imgCnv;
      }
      else
        sendWarning("poolget: unknown 'convert to' format");
    }
    if (result and (aScale or aToImg or aToFmt))
    {
      imgSrc = imgRgb;
      if (aToImg and aToImgValue != imgSource)
        imgRgb = imgPool->getImage(aToImgValue, true);
      else if (aScale)
      {
        imgRgb = new UImage();
        imgRgbCreated = true;
      }
      imgRgb->copyMeta(imgSrc, true);
      if (not aScale)
        imgRgb->copyJustImage(imgSrc);
      else
        imgRgb->copyAndScale(imgSrc, aScaleValue);
      imgRgb->updated();
      if (aToImg)
        imgSource = aToImgValue;
    }
    // all arameters are now checked
    if (result and not (imgSave or imgSavePng or imgSaveTxt))
    { // send the image data
      result = sendImage(msg, //->client,
                  msg->tag.getTagName(),
                  imgRgb,  // image and format
                  roundi(imgLeft * aScaleValue),     // left column to send
                  roundi(imgTop * aScaleValue),     // top row to send
                  roundi(imgWidth * aScaleValue),    // max width
                  roundi(imgHeight * aScaleValue),   // max height
                  imgSource,   // image source - if from pool
                  true,  // binatry is default
                  false,     // send as BW-only
                  imgGetNum,   // send image number
                  imgGetTod,   // send time-of-day format
                  imgGetTime,  // send hh:mm:ss.ccc time format
                  NULL,        // send this camera position name
                  imgGetName,   // send image name (RGB format only)
                  silent
                  );
    }
    //
    if (result and (imgSave or imgSavePng or imgSaveTxt) and imgRgb != NULL)
    { // save RGB image to file
      if (strlen(imgName) < 1)
      { // make image name
        if (strlen(imgRgb->name) < 1)
          snprintf(imgName, MNL, "imgRgb%08lu_",
              imgRgb->imageNumber);
        else
        {
          p1 = strrchr(imgRgb->name, '/');
          if (p1 == NULL)
            p1 = imgRgb->name;
          else
            p1++;
          snprintf(imgName, MNL, "%s_%08lu_", p1,
                   imgRgb->imageNumber);
        }
        imgRgb->imgTime.getForFilename(attName, true);
        snprintf(filn, MFL, "%s/%s_%02d%s", imagePath, imgName,
                 imgRgb->camDevice, attName);
        if (imgSavePng)
          strncat(filn, ".png", MFL);
        else if (imgSave)
          strncat(filn, ".bmp", MFL);
        else if (imgSaveTxt)
          strncat(filn, ".txt", MFL);
      }
      else
      {
        if (imgName[0] == '.' or imgName[0] == '/' or imgName[0] == '~')
          strncpy(filn, imgName, MFL);
        else
          // add default path
          snprintf(filn, MFL, "%s/%s", imagePath, imgName);
        if ((strstr(filn, ".bmp") != NULL) or (strstr(filn, ".BMP") != NULL))
        {
          imgSave = true;
          imgSavePng = false;
          imgSaveTxt = false;
        }
        else if ((strstr(filn, ".png") != NULL) or (strstr(filn, ".PNG") != NULL))
        {
          imgSavePng = true;
          imgSave = false;
          imgSaveTxt = false;
        }
        else if ((strstr(filn, ".txt") != NULL) or (strstr(filn, ".TXT") != NULL))
        {
          imgSaveTxt = true;
          imgSave = false;
          imgSavePng = false;
        }
        else if (imgSave)
          strcat(filn, ".bmp");
        else if (imgSavePng)
          strcat(filn, ".png");
        else if (imgSaveTxt)
          strcat(filn, ".txt");
      }
      // save
      if (imgSave)
      {
        UImage * tmp = imgRgb;
        if (tmp->saveBMP(filn))
        { // reply
          snprintf(reply, MRL, "saved BMP image as '%s' on server imagePath or specified path", tmp->name);
          if (not silent)
            sendInfo(reply);
        }
        else
        { // not saved to
          snprintf(reply, MRL, "save BMP image failed, to %s (bad path?)", filn);
          sendWarning(reply);
        }
      }
      if (imgSavePng)
      {
        UImage * tmp = imgRgb;
        if (tmp->savePNG(filn))
        { // reply
          snprintf(reply, MRL, "PNG saved to %s", filn);
          if (not silent)
            sendInfo(reply);
        }
        else
        { // not saved
          snprintf(reply, MRL, "PNG not saved %s (bad path?, no libPNG?)", filn);
          sendWarning(reply);
        }
      }
      if (imgSaveTxt)
      {
        UImage * tmp = imgRgb;
        if (tmp->saveTxt(filn))
        { // reply
          snprintf(reply, MRL, "TXT saved to %s", filn);
          if (not silent)
            sendInfo(reply);
        }
        else
        { // not saved
          snprintf(reply, MRL, "TXT not saved %s (bad path?, no libPNG?)", filn);
          sendWarning(reply);
        }
      }
// BGGR00000015-cam10-20110907_132524.360_00000015__1020110907_132524.360
    }
    if (not result)
    {
      if (not msg->serverPushCommand)
        sendWarning("No such image available");
      else
        sendInfo("Image push failed - no image");
    }
  }
  else
  {
    sendWarning("Missing imgPool resource");
  }
  // cleanup
  if (imgRgbCreated)
    delete imgRgb;
  if (imgCnv != NULL)
    delete imgCnv;
  //
  return result;
}


////////////////////////////////////////////////////

bool UFunctionImgPool::handleImageSetCommand(UServerInMsg * msg, void * imgBase)
{
  bool result = false;
  char attName[MAX_SML_NAME_LENGTH];
  const int VAL_BUFF_LNG = 100;
  char attValue[VAL_BUFF_LNG];
  const int MSL = 200;
  char s[MSL];
  int imgSource = 0; // image pool source
  bool loadPng = false;
  bool loadBmp = false;
  bool ask4help = false;
  bool aStream = false;
  bool vStream;
  bool aShow = false;
  bool vShow = false;
//  int camDevice = 0;
  const int MFL = MAX_FILENAME_LENGTH;
  char filn[MFL];
  char filn2[MFL];
  UImgPush * imgRgb = NULL;
  unsigned long todS, todUs;
  UTime imgTime;
  char * p1;
  int i, n;
  //
  imgTime.Now();
  while (msg->tag.getNextAttribute(attName, attValue, VAL_BUFF_LNG))
  {
/*    if ((strcasecmp(attName, "device") == 0))
      camDevice = strtol(attValue, NULL, 10);*/
    if (strcasecmp(attName, "img") == 0)
      imgSource = strtol(attValue, NULL, 10);
    else if (strcasecmp(attName, "help") == 0)
      ask4help = true;
    else if (strcasecmp(attName, "loadBmp") == 0)
    {
      loadBmp = true;
      strncpy(filn, attValue, MFL);
    }
    else if (strcasecmp(attName, "loadPng") == 0)
    {
      loadPng = true;
      strncpy(filn, attValue, MFL);
    }
    else if (strcasecmp(attName, "tod") == 0)
    {
      sscanf(attValue, "%lu.%lu", &todS, &todUs);
      imgTime.setTime(todS, todUs);
    }
    else if (strcasecmp(attName, "stream") == 0)
    {
      aStream = true;
      vStream = str2bool2(attValue, true);
    }
    else if (strcasecmp(attName, "show") == 0)
    {
      aShow = true;
      vShow = str2bool2(attValue, true);
    }
    // else ignore attribute
  }
  //
  if (ask4help)
  {
//    sendMsg( msg, "<help subject=\"module help\">\n");
//    sendMsg( msg, "</help>\n");
    sendMsg( msg, "<help subject=\"POOLSET help\">\n");
    sendText( msg, "---------- Options to POOLSET command\n");
    sendText( msg, "loadPng='filename'   Load this image - in PNG format\n");
    sendText( msg, "loadBmp='filename'   Load this image - in BMP format\n");
    sendText( msg, "img=N                Load to image pool number N (default=0) (also for pushed image)\n");
    sendText( msg, "stream[=false]       Start (or stop) streaming this image\n");
    sendText( msg, "show[=false]         Show (or hide) display of this image\n");
    sendText( msg, "device=D             Set as taken from camera device D (def=0)\n");
    sendText( msg, "tod=secs             Set as taken at time 's.d' (def = now)\n");
    sendText( msg, "serial=s             Set serial number (def = 0)\n");
    sendText( msg, "See also: POOLGET, POOLLIST and VAR IMGPOOL\n");
    sendText( msg, "---\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
  }
  else if (imgPool != NULL)
  { // get camera reference and image
    imgRgb = (UImgPush*) imgPool->getImage(imgSource, true);
    if (aStream and imgRgb != NULL)
    {
      imgRgb->startStreaming(vStream);
      if (vStream and not silent)
      {
        snprintf(s, MSL, "Strting stream on port %d", imgRgb->getTcpPort());
        sendInfo(s);
      }
      else
      {
        snprintf(s, MSL, "Strting stream on port %d", imgRgb->getTcpPort());
        sendInfo(s);
      }        
    }
    else if (aShow and imgRgb != NULL)
    {
      imgRgb->startShowing(vShow);
      sendInfo("done");
    }
    else if (imgRgb != NULL)
    {
      if (imgRgb->tryLock())
      {
        if (loadBmp)
        { // source is BMP file
          if (filn[0] != '/' and filn[0] != '.')
          {
            snprintf(filn2, MFL, "%s/%s", imagePath, filn);
            result = imgRgb->loadBMP(filn2);
          }
          else
          {
            result = imgRgb->loadBMP(filn);
            strcpy(filn2, filn);
          }
          //
          if (result)
          { // give image a name - from filename
            n = strlen(filn);
            p1 = &filn[n];
            for (i = 0; i < n; i++)
              if (*p1-- == '/')
              {
                p1++;
                break;
              }
            p1++;
            strncpy(imgRgb->name, filn, MAX_IMG_NAME_SIZE);
          }
          if (result)
          {
            snprintf(s, MSL, "img=%d loaded from %s", imgSource, filn2);
            sendInfo(s);
          }
          else
          {
            snprintf(s, MSL, "Image failed to load (source=%s)", filn2);
            sendWarning(msg, s);
          }
        }
        else if (loadPng)
        { // source is PNG file
          if (filn[0] != '/' and filn[0] != '.')
          { // relative name, so extend with image path
            snprintf(filn2, MFL, "%s/%s", imagePath, filn);
            result = imgRgb->loadPNG(filn2);
          }
          else
          { // fully qualified filename
            result = imgRgb->loadPNG(filn);
            strcpy(filn2, filn);
          }
          if (result)
          { // give image a name - from filename
            n = strlen(filn);
            p1 = &filn[n];
            for (i = 0; i < n; i++)
              if (*p1-- == '/')
              {
                p1++;
                break;
              }
            p1++;
            strncpy(imgRgb->name, filn, MAX_IMG_NAME_SIZE);
          }
          if (result)
          {
            snprintf(s, MSL, "img=%d loaded from %s", imgSource, filn2);
            sendInfo(msg, s);
          }
          else
          {
            snprintf(s, MSL, "Image failed to load (source=%s)", filn2);
                  sendWarning(msg, s);
          }
        }
        //
        imgRgb->imgTime = imgTime;
        int n;
        if (msg->tag.getAttInteger("serial", &n, 1))
          imgRgb->imageNumber = n;
        else
          imgRgb->imageNumber++;
        if (msg->tag.getAttInteger("device", &n, 10))
          imgRgb->camDevice = n;
        imgRgb->used = 0;
        imgRgb->updated();
        imgRgb->unlock();
      }
      else
        sendWarning(msg, "Image is locked (by another thread) and thus not updated");      
    }
    else
      sendWarning(msg, "Image pool number not valid (or no memory left)");
    // debug
    // printf("UFunctionImgPool:: pool count is now %d\n", imgPool->getImageCnt());
    // debug end
  }
  else
    sendWarning( msg, "Missing imgPool resource");
  //
  return result;
}

////////////////////////////////////////////////////////

bool UFunctionImgPool::handlePoolPushCommand(UServerInMsg * msg)
{ // get parameters
  bool result = false;
  const int VBL = 50;
  char val[VBL];
  const int MRL = 2000;
  char reply[MRL];
  int poolimg = 0;
  bool ask4help;
  bool ask4list;
  bool isAflush;
  UImgPush * img;
  int n, cm, ca;
  //
  // get relevalt attributes
  ask4help = msg->tag.getAttValue("help", val, VBL);
  ask4list = msg->tag.getAttValue("list", val, VBL);
  isAflush = msg->tag.getAttValue("flush", val, VBL);
  if (msg->tag.getAttValue("img", val, VBL))
   // get (non default) device
    poolimg = strtol(val, NULL, 0); // device number
  // ignore all other attributes - handled in ush structure
  if (ask4help)
  {
//    sendMsg( msg, "<help subject=\"module help\">\n");
//    sendMsg( msg, "</help>\n");
    sendMsg( msg, "<help subject=\"POOLPUSH help\">\n");
    sendText( msg, "---- Available POOLPUSH settings:\n");
    sendText( msg, "Executes push cmd's on update of img N.\n");
    sendText( msg, "img=N             Push from poolimage 'N' (else pool img 0)\n");
    sendText( msg, "good=k or g=k     Stop push after k good commands (def: no stop).\n");
    sendText( msg, "total=k or n=k    Stop push after k commands (def: no stop)\n");
    sendText( msg, "interval=k or i=k Push with this interval (in updates, def = 1)\n");
    sendText( msg, "flush[=cmd]       Remove push command(s) from client (default all)\n");
    sendText( msg, "cmd=\"cmd\"         Do execute 'cmd' command for every image update\n");
    sendText( msg, "list              List all active push commands\n");
    sendText( msg, "See also PUSH (timed push), POOLGET, POOLLIST, POOLSET, and VAR IMGPOOL\n");
    sendText( msg, "---\n");
    sendMsg( msg, "</help>\n");
    sendInfo(msg, "done");
    result = true;
  }
  else if (imgPool != NULL)
  { // push command
    img = (UImgPush*)imgPool->getImage(poolimg, false);
    if (img == NULL and not ask4list)
    {  // make a minimal image for the push command
      img = (UImgPush*)imgPool->getImage(poolimg, true, 60, 80, 3, 8);
      if (img != NULL)
        img->setName("empty");
    }
    //
    if (img != NULL)
    {
      if (ask4list)
      {
        snprintf(reply, MRL, "<help img=\"%d\" subject=\"image poolPush command list\">\n", poolimg);
        sendMsg(msg, reply);
        img->UServerPush::print("poolPush", reply, MRL);
        sendText(msg, reply);
        sendMsg(msg, "</help>\n");
        sendInfo(msg, "done");
      }
      else
      {
        n = img->addPoolPushCommand(msg);
        img->getPushQueue()->getPushCmdActiveCnt(&cm, &ca);
        if (isAflush)
        { // flush and senr reply
          snprintf(reply, MRL, "<%s done=\"%s\" flushed=\"%d\" pushcmds=\"%d\" pushcalls=\"%d\"/>\n",
                msg->tag.getTagName(), bool2str(n > 0), n, cm, ca);
        }
        else
        { // a new push command
          snprintf(reply, MRL, "<%s done=\"%s\" pushcmds=\"%d\" pushcalls=\"%d\"/>\n",
                msg->tag.getTagName(), bool2str(n > 0), cm, ca);
        }
        sendMsg(msg, reply);
      }
    }
    else
      sendWarning(msg, "No such image");
  }
  else
    sendWarning(msg, "No image pool");
  //
  return result;
}



