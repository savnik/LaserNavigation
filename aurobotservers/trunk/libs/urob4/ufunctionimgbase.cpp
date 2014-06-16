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
#include <stdlib.h>
//#include <ugen4/urawimage.h>

#include "ufunctionimgbase.h"

UFunctionImgBase::UFunctionImgBase()
 : UFunctionBase()
{
  imgPool = NULL;
//  imgPoolLocal = false;
}

////////////////////////////////////////////////////

// UFunctionImgBase::UFunctionImgBase(UImagePool * images)
//  : UFunctionBase()
// {
//   imgPoolLocal = false;
//   imgPool = images;
// }

////////////////////////////////////////////////////

UFunctionImgBase::~UFunctionImgBase()
{
/*  if (imgPoolLocal and (imgPool != NULL))
    delete imgPool;*/
}

//////////////////////////////////////////////////////////////////

bool UFunctionImgBase::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  //
  if (resource->isA(UImagePool::getResClassID()))
  { // delete any local
    if (remove)
      // resource module is unloaded
      imgPool = NULL;
    else if (imgPool != (UImagePool *)resource)
      // updated or new resource
      imgPool = (UImagePool *)resource;
    else
      // same as before
      result = false;
  }
  else
    // may be needed at lower levels
    result = UFunctionBase::setResource(resource, remove);
  //
  return result;
}

//////////////////////////////////////////////////////////////////

bool UFunctionImgBase::sendImage(UServerInMsg * msg, //int client,
               const char * tagName,
               /*URawImage * imgRaw, */
               UImage * imgRgb,  // image and format
               int imgLeft,     // left column to send
               int imgTop ,     // top row to send
               int imgWidth,    // max width
               int imgHeight,   // max height
               int imgSource,        // image source - if from pool
               bool isCodecBin,      // binatry or hex-coded
               bool imgIsBW,         // send as BW-only - not used
               bool imgGetNum,       // send image number
               bool imgGetTod,       // send time-of-day format
               bool imgGetTime,      // send hh:mm:ss.ccc time format
               const char * posName, // send this camera position name
               bool imgGetName,      // send image name
               bool silent
               )
{
  bool result = (imgRgb != NULL);
  bool isNone;
  int n, r, c;
  unsigned char * py;
  unsigned char * pu;
  unsigned char * pv;
  UPixel * px;
  const int MRL = UIMAGE_MAX_WIDTH * 8; // hex coded line (640*8=3840)
  char replyBuf[MRL];
  char * reply = replyBuf;
  const int MSL = 50;
  char s[MSL];
  char * pd;
  bool isBw = false, is420 = false;
  bool imgRemRad = false;
  UTime imgTime;
  unsigned long imgNumber = 0;
  const char * format ="unknown";
  int imgDevice = 0;
  const char * imgName = "";
  bool isBmpFmt = false;
  //
  if (msg->tag.getAttValue("codec", reply, 10))
  { // there is a codec attribute
    isBmpFmt = strcasestr(reply, "bmp") != NULL;
    isCodecBin = strcasestr(reply, "hex") == NULL;
  }
  // send the image data
  if (result)
  { // size check - raw image - reduce to be within image
    imgTop = maxi(0, mini(imgTop, imgRgb->height()));
    imgLeft = maxi(0, mini(imgLeft, imgRgb->width()));
    imgHeight = maxi(0, imgHeight);
    imgWidth = maxi(0, imgWidth);
    if ((imgTop + imgHeight) > (int)imgRgb->height())
      imgHeight = maxi(0, imgRgb->height() - imgTop);
    if ((imgLeft + imgWidth) > (int)imgRgb->width())
      imgWidth = maxi(0, imgRgb->width() - imgLeft);
    imgNumber = imgRgb->imageNumber;
    imgTime = imgRgb->imgTime;
    isBw = imgRgb->getChannels() == 1;
    imgRemRad = imgRgb->radialErrorRemoved;
    format = imgRgb->getColorTypeString();
    int stride = imgWidth * imgRgb->getDepth() / 8 * imgRgb->getChannels();
    if (stride * 2 > MRL)
      // default buffer too small - make more space (slower)
      reply = (char *) malloc(stride * 2);
    if (imgRgb->getColorType() ==  PIX_PLANES_YUV420)
      is420 = true;
    imgDevice = imgRgb->camDevice;
    imgName = imgRgb->name;
  }
  if (result)
  { // are pixels required
    isNone = (imgWidth == 0) or (imgHeight == 0);
    // send metadata first
    snprintf(reply, MRL-3, "<%s x=\"%d\" y=\"%d\" w=\"%d\" h=\"%d\" format=\"%s\" remRad=\"%s\" dev=\"%d\"",
       tagName,
       imgLeft, imgTop, imgWidth, imgHeight,
       format, bool2str(imgRemRad), imgDevice);
    n = strlen(reply);
    // debug
    if (n >= MRL-n-3)
      n = MRL-n-3;
    // debug end
    if (imgGetNum)
    {
      snprintf(&reply[n], MRL-n-3, " imgNum=\"%lu\"", imgNumber);
      n = strlen(reply);
    }
    if (imgGetTod)
    { // forward time in time-of-day format
      snprintf(&reply[n], MRL-n-3, " imgTod=\"%ld.%06ld\"",
                      imgTime.GetSec(), imgTime.GetMicrosec());
      n = strlen(reply);
    }
    if (imgGetTime)
    {
      imgTime.getTimeAsString(s, true);
      snprintf(&reply[n], MRL-n-3, " imgTime=\"%s\"", s);
      n = strlen(reply);
    }
    if (imgGetName and (imgName != NULL))
    {
        snprintf(&reply[n], MRL-n-3, " name=\"%s\"", imgName);
        n = strlen(reply);
    }
    if (imgSource >= 0)
    {
      snprintf(&reply[n], MRL-n-3, " pool=\"%d\"",
              imgSource);
      n = strlen(reply);
    }
    if (isNone)
    {
      strcat(reply, "/>\n");
      if (not silent)
        sendMsg(msg, reply);
    }
    else
    {
      strcat(reply, ">\n");
      sendMsg(msg, reply);
    }
    // now the image itself
    if (not isNone)
    {
      const int MBHL = 0x36; // 56 bytes
      n = imgWidth * imgHeight * imgRgb->getDepth()/8;
      if (is420 and not isBw)
        n += 2 * (((imgHeight + 1)/2) * ((imgWidth + 1)/2));
      else if (not is420 and not isBw)
        n *= imgRgb->getChannels();
      if (isCodecBin)
      { // binary use all 8-bit char
        if (isBmpFmt)
          snprintf(reply, MRL, "<bin size=\"%d\" codec=\"BMPBIN\">", n + MBHL);
        else
          snprintf(reply, MRL, "<bin size=\"%d\" codec=\"BIN\">", n);
      }
      else
      { // hex format - only in range "0123456789abcdef"
        if (isBmpFmt)
          snprintf(reply, MRL, "<bin size=\"%d\" codec=\"BMPHEX\">", 2 * (n + MBHL));
        else
          snprintf(reply, MRL, "<bin size=\"%d\" codec=\"HEX\">", 2 * n);
      }
      result = sendMsg(msg,reply);
      if (isBmpFmt and result)
      { // make bmp header
        char bmpHdr[MBHL];
        uint32_t * v32;
        uint16_t * v16;
        // BMP header (14 bytes)
        bmpHdr[0] = 'B';
        bmpHdr[1] = 'M';
        v32 = (uint32_t *)&bmpHdr[0x02];
        *v32++ = n + MBHL;
        *v32++ = 0;
        *v32   = MBHL;
        // 40 bytes of DIB Header (Bitmap Information Header)
        v32 = (uint32_t *)&bmpHdr[0x0E];
        *v32++ = 40;
        *v32++ = imgWidth; // width
        *v32   = imgHeight; // height
        // two 16 bit integers
        v16 = (uint16_t *)&bmpHdr[0x1A];
        *v16++ = 1; // color planes - set to 1
        *v16   = imgRgb->getChannels() * imgRgb->getDepth(); // bit per pixel
        // the rest is 32 bit integers
        v32 = (uint32_t *)&bmpHdr[0x1E];
        *v32++ = 0; // compression method 0=none
        *v32++ = n; // image size in bytes
        *v32++ = 0; // resolution width
        *v32++ = 0; // resolution height
        *v32++ = 0; // number of colors in palette - 0 is 2^n
        *v32   = 0; // important colors - 0 is all
        if (isCodecBin)
          // send as binary
          result = sendMsg(msg->client, bmpHdr, MBHL);
        else
        { // code header in hex format
          char * ph = bmpHdr;
          pd = reply;
          for (int i = 0; i < MBHL; i++)
          { // convert header to hex
            sprintf(pd, "%02x", *ph++);
            pd += 2;
          }
          result = sendMsg(msg->client, reply, MBHL * 2);
        }
      }
      if ((is420 or isBw) and result)
      {
        int widthBytes = imgWidth * imgRgb->getDepth()/8;
        for (r = 0; r < (int)imgHeight and result; r++)
        {
          py = &imgRgb->getYline(r + imgTop)[imgLeft];
          if (isCodecBin)
            result = sendMsg(msg->client, (char *)py, widthBytes);
          else
          { // hex
            pd = reply;
            for (c = 0; c < widthBytes; c++)
            {
              sprintf(pd, "%02x", *py++);
              pd += 2;
            }
            result = sendMsg(msg->client, reply, widthBytes * 2);
          }
        }
        if (not isBw)
        { // send also U and V part
          for (r = 0; r < (imgHeight+1)/2 and result; r++)
          {
            pu = &imgRgb->getUline(r + imgTop/2)[imgLeft/2];
            n = (imgWidth+1)/2;
            if (isCodecBin)
              result = sendMsg(msg->client, (char *)pu, n);
            else
            { // hex format
              pd = reply;
              for (c = 0; c < n; c++)
              {
                sprintf(pd, "%02x", *pu++);
                pd++; pd++;
              }
              result = sendMsg(msg->client, reply, n * 2);
            }
          }
          for (r = 0; r < (int)(imgHeight+1)/2 and result; r++)
          {
            pv = &imgRgb->getVline(r + imgTop/2)[imgLeft/2];
            n = (imgWidth+1)/2;
            if (isCodecBin)
              result = sendMsg(msg->client, (char *)pv, n);
            else
            { // hex format
              pd = reply;
              for (c = 0; c < n; c++)
              {
                sprintf(pd, "%02x", *pv++);
                pd++; pd++;
              }
              result = sendMsg(msg->client, reply, n * 2);
            }
          }
        }
      }
      else if (result)
      { // 2 or 3-plane image
        // debug
        if ((unsigned)imgHeight > imgRgb->height())
          // error
          imgHeight = imgRgb->height();
        if ((unsigned)imgWidth > imgRgb->width())
          imgWidth = imgRgb->width();
        // debug end
        for (r = 0; r < imgHeight and result; r++)
        {
          if (isBmpFmt)
            // BMP format takes bottom line first
            px = &imgRgb->getLine(imgHeight - (r + imgTop) - 1)[imgLeft];
          else
            // else top line first
            px = &imgRgb->getLine(r + imgTop)[imgLeft];
          if (isCodecBin)
            result = sendMsg(msg->client, (char *)px, imgWidth * imgRgb->getChannels());
          else
          { // hex
            unsigned char * ps = (unsigned char *) px;
            int psCnt = imgWidth * imgRgb->getChannels();
            pd = reply;
            for (c = 0; c < psCnt ; c++)
            {
              sprintf(pd, "%02x", *ps);
              ps++;
              pd += 2;
            }
            result = sendMsg(msg->client, reply, psCnt * 2);
          }
        }
      }
      if (result)
      { // send end tag
        snprintf(reply, MRL, "</bin>\n</%s>\n", tagName);
        result = sendMsg(msg, reply);
      }
    }
    // debug
    if (not result)
      printf("UFunctionImgBase::sendImage failed to send image %lu\n", imgRgb->imageNumber);
    // sendInfo(msg, "Image send");
    // debug end
  }
  if (reply != replyBuf)
    free(reply);
  return result;
}

////////////////////////////////////////////////////

// bool UFunctionImgBase::gotAllResources(char * missingThese, int missingTheseCnt)
// {
//   bool result;
//   int n = 0;
//   char * p1 = missingThese;
//   //
//   result = imgPool != NULL;
//   if ((not result) and (p1 != NULL))
//   {
//     snprintf(p1, missingTheseCnt, " %s", UImagePool::getResClassID());
//     n = strlen(p1);
//     p1 = &missingThese[n];
//   }
//   // image pool needs core pointer (cmdExe)
// /*  if (result)
//   {
//     result = imgPool->gotAllResources(p1, missingTheseCnt - n);
//     if (not result and (p1 != NULL))
//     {
//       n += strlen(p1);
//       p1 = &missingThese[n];
//     }
//   }*/
//   // function base needs core pointer too.
//   result &= UFunctionBase::gotAllResources(p1, missingTheseCnt - n);
//   return result;
// }

////////////////////////////////////////////////////

// UResBase * UFunctionImgBase::getResource(const char * resID)
// {
//   UResBase * result = NULL;
//   //
//   if (strcmp(resID, UImagePool::getResClassID()) == 0)
//   {
//     if (imgPool == NULL)
//     { // no pool - so (try to) create one
//       imgPool = new UImagePool();
//       imgPoolLocal = (imgPool != NULL);
//     }
//     result = imgPool;
//   }
//   //
//   if (result == NULL)
//     // base function may have requested resource
//     result = UFunctionBase::getResource(resID);
//   return result;
// }
// 
// ///////////////////////////////////////////////////
// 
// const char * UFunctionImgBase::resourceList()
// {
//   return "imgPool";
// }

///////////////////////////////////////////////////
