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

#include "usmltag.h"
#include "uclientfuncimage.h"


UClientFuncImage::UClientFuncImage()
{
  //rawImg = NULL;
  imgBuffer = NULL;
}

//////////////////////////////////////////

UClientFuncImage::~UClientFuncImage()
{
  if (imgBuffer != NULL)
    delete imgBuffer;
}

//////////////////////////////////////////

const char * UClientFuncImage::name()
{
  return "image data to imgPool resource (same pool number)";
}

//////////////////////////////////////////

const char * UClientFuncImage::commandList()
{
  return "imageGet poolGet poolList";
}

//////////////////////////////////////////

void UClientFuncImage::handleNewData(USmlTag * tag)
{ // distribute to sub-functions
  if (tag->isTagA("imageGet") or tag->isTagA("poolGet"))
    handleImages(tag);
  else if (tag->isTagA("poolList"))
    handleImageList(tag);
  else if (tag->isTagA("camget"))
    handleImageList(tag);
  else
    printReply(tag, "UClientFuncImage::handleNewData: not mine");
}


///////////////////////////////////////////////

void UClientFuncImage::changedNamespace(const char * newNamespace)
{ // set namespace
  strncpy(serverNamespace, newNamespace, MAX_SML_NAME_LENGTH);
  // set name space value
  if (strcmp(newNamespace, "camServer") == 0)
    serverNamespaceValue = 1;
  else if (strcmp(newNamespace, "rob4") == 0)
    // testserver in library
    serverNamespaceValue = 2;
  else
    // unsupported namespace
    serverNamespaceValue = 0;
}

//////////////////////////////////////////

void UClientFuncImage::handleImageList(USmlTag * tag)
{
  tag->print(" - ");
}

///////////////////////////////////////////////

void UClientFuncImage::handleImages(USmlTag * tag)
{ // handle images
  bool result = true;
  int width = 0;
  int height = 0;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  int n, m;
  int binSize = 0;
  long sec = 0, usec = 0;
  unsigned long imgNum = 0;
  bool radRem = false;
//  bool isRawFormat = false;
  bool isHex = false;
  bool isBMP = false;
  int imgDev = 0;
  int imgSource = -1;
  const int MFL = 10;
  char format[MFL] = "NONE";
  char imgName[MAX_IMG_NAME_SIZE] = "";
  USmlTag binTag;
  char * binDataDest = NULL;
  char * binDataSource;
  bool moreToCome = false;
  UImage * img = NULL;
  int poolNum = 3; // default pool-image number ("never" used)
  // debug
  //tag->print("handleImages got:");
  // debug end
  posName[0] = '\0';
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "w") == 0)
      sscanf(val, "%d", &width);
    else if (strcasecmp(att, "h") == 0)
      sscanf(val, "%d", &height);
    else if (strcasecmp(att, "imgTod") == 0)
      sscanf(val, "%ld.%ld", &sec, &usec);
    else if (strcasecmp(att, "remRad") == 0)
      radRem = str2bool(val);
    else if (strcasecmp(att, "imgNum") == 0)
      sscanf(val, "%lu", &imgNum);
    else if (strcasecmp(att, "format") == 0)
      strncpy(format, val, MFL);
    else if (strcasecmp(att, "dev") == 0)
      sscanf(val, "%d", &imgDev);
    else if (strcasecmp(att, "posName") == 0)
      snprintf(posName, MAX_MOUNT_NAME_SIZE, "%s", val);
    else if (strcasecmp(att, "name") == 0)
      snprintf(imgName, MAX_IMG_NAME_SIZE, "%s", val);
    else if (strcasecmp(att, "source") == 0)
      sscanf(val, "%d", &imgSource);
    else if (strcasecmp(att, "pool") == 0)
      poolNum = strtol(val, NULL, 0);
    else if ((strcasecmp(att, "warning") == 0) or
             (strcasecmp(att, "debug") == 0) or
             (strcasecmp(att, "info") == 0))
      result = false;
  }
  //
  if (not result)
  { // just a message
    if (verboseMessages)
      tag->print("");
  }
  else if (tag->isAStartTag())
  { // is FULL format type
    moreToCome = result;
    // get buffer for image (a small image should be OK)
    img = getImageBuffer(poolNum, height, width, 1, 8);
    //img = getImageBuffer(poolNum, height, width, 3, 8);
    result = (img != NULL);
    if (not result)
    { // nowhere to put image, so skip image part of message
      tag->skipToEndTag(300);
      printf("UClientFuncImage::handleImages: no buffer for received image - skipping!\n");
    }
  }
  if (tag->cnnVerbose())
    // print attributes if requested
    tag->print("");
  //
  if (result and moreToCome)
  {
    img->radialErrorRemoved = radRem;
    img->imageNumber = imgNum;
    img->imgTime.setTime(sec, usec);
    img->setColorType(format);
    img->setSize(height, width, img->getChannels(), img->getDepth());
    // debug
    //printf("Set color type to '%s' (%d)\n", format, img->getColorType());
    // debug end
    binDataDest = (char *)img->getData();
    img->valid = true;
    img->source = imgSource;
    img->camDevice = imgDev;
    if (strlen(imgName) < 19)
      snprintf(img->name, MAX_IMG_NAME_SIZE,
            "%s_d%d_s%d_%s_%s", format, img->camDevice,
            img->source, posName, imgName);
    else
      strncpy(img->name, imgName, MAX_IMG_NAME_SIZE);
    //
    if (tag->isAStartTag())
    { // next tag should be a <bin size=xxx codec="HEX"> type tag
      result = tag->getNextTag(&binTag, 200);
      moreToCome = result;
    }
  }
  if (result and moreToCome)
  {
    //
    // debug
    //binTag.print("handleImages got:");
    // debug end
    //
    result = (binTag.isTagA("bin"));
    if (not result)
    { // leave the rest to the main tag-reader
      if (verboseMessages)
        printf("UClientFuncImage::handleImages"
              " expected a 'bin' tag, but got a '%s' - discarded\n",
              binTag.getTagName());
    }
  }
  if (result and moreToCome)
  { // get binary part
    while (binTag.getNextAttribute(att, val, MVL))
    {
      if (strcasecmp(att, "size") == 0)
        sscanf(val, "%d", &binSize);
      else if ((strcasecmp(att, "codex") == 0) or (strcasecmp(att, "codec") == 0))
      { // one of these formats
        isHex = (strcasestr(val, "hex") != NULL);
        isBMP = (strcasestr(val, "bmp") != NULL);
      }
    }
    m=0;
    if (binSize > 0)
    { // get the binary data
      m = binSize;
      if (isBMP)
      { // BMP header needs to be removed
        // is always 54 bytes (or 108 in hex)
        if (isHex)
          n = tag->getNBytes(val, 108, 800);
        else
          n = tag->getNBytes(val, 54, 800);
      }
      if (isHex)
      { // get data in chuncks of MVL
        n = 0;
        while (m > 0) // and cnn->isConnected())
        { // wait max (5000) milisec
          // debug
          if (m <= 5759982 + MVL)
            binDataSource = val;
          // debug end
          n = tag->getNBytes(&val[n], mini(MVL - n, m), 500);
          if (n == 0)
            break;
          m -= n;
          binDataSource = val;
          for (int h = 0; h < n/2; h++)
          {
            *binDataDest++ = hex2int(binDataSource[0], binDataSource[1]);
            binDataSource++;
            binDataSource++;
          }
          n = n % 2;
          if (n == 1)
            // odd value received
            val[0] = val[n-1];
        }
      }
      else // (binary)
      { // get data with long timeout (may be up to 1 MB of data)
        while (m > 0) // and cnn->isConnected())
        { // try in limited steps
          n = tag->getNBytes(binDataDest, m, 5000);
          if (n == 0)
            break;
          m -= n;
          binDataDest += n;
        }
      }
    }
    if ((m != 0) and verboseMessages)
      printf("UClientFuncImage::handleImages"
          " did not get all pixels - missing %d of %d\n",
          m, binSize);
    // get end-tag </bin>
    result = tag->getNextTag(&binTag, 2000);
  }
  if (result and moreToCome)
  {
    if (binTag.isTagA("/bin"))
      // get also the </imageget> tag off the queue
      result = tag->getNextTag(&binTag, 500);
  }
  if (result and moreToCome)
  { //
    if (false and verboseMessages)
    {
      snprintf(val, MVL, "%s/got_%d_%06lu.bmp",
              imagePath, imgDev, imgNum);
      printf("UClientFuncImage::handleImages"
        " saving image to %s\n", val);
/*      if (isRawFormat)
        rawImg->saveBMP(val);
      else*/
      img->saveBMP(val);
    }
    if (binSize > 0)
    { // some data received - inform client
/*      if (isRawFormat)
        gotNewRawImage(rawImg);
      else*/
      img->imgUpdated();
      gotNewImage(img, poolNum, tag);
    }
  }
  if (moreToCome and verboseMessages and not result)
    printf("Failed to get full image\n");
}

//////////////////////////////////////////////////

void UClientFuncImage::gotNewImage(UImage * img, int poolNum, USmlTag * tag)
{
  printf("Got new image of size h=%d w=%d (from pool number %d)\n",
    img->height(), img->width(), poolNum);
}

////////////////////////////////////////////////

void UClientFuncImage::gotNewCamInfo(int device, UPosRot pose,
                                     double focalLength, double k1, double k2,
                                     const char * name)
{
  printf("Got new cam data device=%d, a %s\n",
         device, name);
  printf("Lens data: focalLength=%g (K1=%g K2=%g)\n",
        focalLength, k1, k2);
  pose.print("pose ");
}

////////////////////////////////////////////////

UImage * UClientFuncImage::getImageBuffer(int poolNumber, int height, int width, int channels, int depth)
{ // could be overwritten to get a image buffer associated with this pool number
  if (imgBuffer == NULL)
    imgBuffer = new UImage800();
  return imgBuffer;
}


