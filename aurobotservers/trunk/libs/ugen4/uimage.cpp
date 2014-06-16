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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <stddef.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#ifndef IMAGE_NO_PNG
#include <png.h>
#endif

#ifndef NO_ZLIB
#include <zlib.h>
#endif


#include "ucommon.h"
#include "ugbm.h"
#include "ugbmbmp.h"
#include "uimage2.h"

#ifdef OPENCV2
  #include <imgproc/types_c.h>
  //#ifndef NO_OPENCV_HIGHGUI
  //#include <highgui/highgui_c.h>
  //#endif
#else
#include <opencv/cv.h>
#endif

// source number for debug
#define evIMAGE 10
///////////////////////////////////////////////////////

// Main variable to hold RGB images
//UImages images;

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
// UIMAGE
////////////////////////////////////////////////////////

UImage::UImage()
{
  //
  img.imageData = NULL;
  img.imageDataOrigin = NULL;
  cam = NULL;
  bufferBytes = 0;
  //externalBuffer = NULL;
  //externalBufferCnt = 0;
  //pImage = vImage;
  cvInitImageHeader(&img, // image header
           cvSize(1,1),  // image size (w,h)
           IPL_DEPTH_8U, // 8-bit unsigned
           3);  // channels
  //height = 0;
  //width = 0;
  used = 0;
  radialErrorRemoved = false;
  cam = NULL;
  imageNumber = 0;
  name[0] = '\0';
  saveType[0] = '\0';
  pos.clear();
  rot.clear();
  imgTime.Now();
  colFormat = PIX_PLANES_BGR;
  valid = false;
  source = -1; // should be ignored by image copy!
  convertBuffer = NULL;
}

//////////////////////////////////////////////////////

UImage::~UImage()
{
  if (convertBuffer != NULL)
  {
    // debug
    // printf("deleting image convert buffer (image %lu)\n", imageNumber);
    // debug end
    delete convertBuffer;
    convertBuffer = NULL;
  }
  if (/*(img.imageDataOrigin != externalBuffer) and */(img.imageDataOrigin != NULL))
    free(img.imageDataOrigin);
  // debug
  // fprintf(stderr, "image released\n");
  // debug end
}

////////////////////////////////////////////////////////

bool UImage::initImage(const unsigned int iHeight, const unsigned int iWidth,
               char * data, const unsigned int bufferSize,
               const int channels /*= 3*/, const int depth /* = 8 */)
{
  bool result;
  // set maximum buffer size
  bufferBytes = bufferSize;
  // note that the buffer is externally allocated - and should no be released
  //externalBuffer = data;
  //externalBufferCnt = bufferSize;
  // set CV image header
  cvSetData( &img, data, iWidth * channels);
  // set image size
  result = setSize(iHeight, iWidth, channels, depth);
  // set image as valid
  valid = result;
  //
  return result;
}

//////////////////////////////////////////////////////

bool UImage::resizeBuffer(const unsigned int iHeight, const unsigned int iWidth,
                    const int channels , const int depth )
{
  bool result = true;
  int newBuffCnt;
  char * newBuffer;
  //
  newBuffCnt = iHeight * iWidth * channels * (depth / 8);
  if (newBuffCnt > int(bufferBytes))
  { // reallocate buffer memory
//     if (/*(img.imageDataOrigin == externalBuffer) or*/ (img.imageDataOrigin == NULL))
//       // the external allocated buffer is too small
//       // (or do not exist) allocate a new
//       newBuffer = (char *) malloc(newBuffCnt);
//     else
    { // buffer ia allocated on heap already, so just resize
      newBuffer = (char *) realloc(img.imageDataOrigin, newBuffCnt);
    }
    if (errno == ENOMEM)
    { // no more memory
      printf("!!!!!!! UImage::resize malloc error!!! errno=ENOMEM (= %d)!\n", errno);
      result = false;
    }
    else
    { // implement
      bufferBytes = newBuffCnt;
      img.imageData = newBuffer;
      img.imageDataOrigin = newBuffer;
    }
  }
  return result;
}

//////////////////////////////////////////////////////

bool UImage::resize(const unsigned int iHeight, const unsigned int iWidth,
                    const int channels /*= 3*/, const int depth /* = 8 */)
{
  bool result = true;
  int newBuffCnt;
  bool newChannelCnt = (channels != img.nChannels);
  //
  newBuffCnt = iHeight * iWidth * channels * (depth / 8);
  if (newBuffCnt > int(bufferBytes))
    result = resizeBuffer(iHeight, iWidth, channels, depth);
  if (result)
  { // set other image parameters
    if (newChannelCnt)
    { // change to default color format too
      switch (channels)
      {
        case 1:
          if (depth == 8)
            result = setSize(iHeight, iWidth, channels, depth, "BW");
          else // depth must be 16 (assumed signed value)
            result = setSize(iHeight, iWidth, channels, depth, "BW16S");
          break;
        case 3: result = setSize(iHeight, iWidth, channels, depth, "RGB"); break;
        default: result = setSize(iHeight, iWidth, channels, depth, "RGBA"); break;
      }
    }
    else
      result = setSize(iHeight, iWidth, channels, depth);
  }
  // set image as valid
  valid = result;
  //
  return result;
}

//////////////////////////////////////////////////////

bool UImage::setSize(const unsigned int iHeight, const unsigned int iWidth,
              const int channels /*= 3*/, const int depth /* = 8 */,
              const char * colorType /*= NULL*/)
{
  bool result;
  int ws = iWidth * channels * depth/8; // width step
  //
  result = (iHeight * ws) <= maxBytes();
  if (not result)
    result = resizeBuffer(iHeight, iWidth, channels, depth);
  if (result)
  { // no available function in opencv to
    // set size to part of an existing buffer
    img.width = iWidth;
    img.height = iHeight;
    img.nChannels = channels;
    img.imageSize = iHeight * ws;
    img.depth = depth;
    img.widthStep = ws;
    py = (unsigned char *)getData();
    pu = &py[img.width * img.height];
    pv = &pu[(img.width * img.height) / 4];
  }
  if (result and (colorType != NULL))
    setColorType(colorType);
  //
  return result;
}

//////////////////////////////////////////////////////

bool UImage::setSizeOnly(const unsigned int iHeight,
                         const unsigned int iWidth)
{
  bool result;
  int ws = iWidth * img.nChannels * img.depth/8; // width step
  //
  result = (iHeight * ws) <= maxBytes();
  if (not result)
    result = resizeBuffer(iHeight, iWidth, img.nChannels, img.depth);
  if (result)
  { // set size
    img.width = iWidth;
    img.height = iHeight;
    img.imageSize = iHeight * ws;
    img.widthStep = ws;
    py = (unsigned char *)getData();
    pu = &py[img.width * img.height];
    pv = &pu[(img.width * img.height) / 4];
  }
  return result;
}

//////////////////////////////////////////////////////

bool UImage::setMaxSize43()
{
  bool result;
  unsigned int w, h, x;
  //
  x = roundi(sqrt(maxPixels()/12));
  w = x * 4;
  h = (w * 3)/4;
  result = (w >= 4);
  if (result)
    result = setSize(h, w, 3, 8);
  return result;
}

//////////////////////////////////////////////////////

void UImage::setNameAndExt(const char * basename,
                           const char * ext)
{ // name
  if ((basename != NULL) and (strlen(basename) > 0))
  {
    strncpy(name, basename, MAX_IMG_NAME_SIZE);
    name[MAX_IMG_NAME_SIZE-1] = '\0';
  }
  if (strlen(name) == 0)
    snprintf(name, MAX_IMG_NAME_SIZE, "img%lu", imageNumber);
  // extension
  if ((ext != NULL) and (strlen(ext) > 0))
  {
    strncpy(saveType, ext, MAX_EXT_SIZE);
    saveType[MAX_EXT_SIZE-1] = '\0';
  }
  if (strlen(saveType) == 0)
    strcpy(saveType, "png");
}

//////////////////////////////////////////////////////

bool UImage::GetNewNonCameraImage()
{
  bool result;
  //
  result = (maxBytes() < (160 * 120));
    // no resasonable data buffer - not valid for any practical use.
  if (result)
  { // image has a size - set real size
    setMaxSize43();
    valid = true;
    used = 0;
    cam = NULL;
    imageNumber = 0;
    strcpy(name, "NonCamera");
    strcpy(saveType, "png");
    pos.clear();
    rot.clear();
    imgTime.Now();
  }
  return result;
}

//////////////////////////////////////////////////////////////
/*
UVector4 UImage::Conv3DToPixel(UPosition * pos, UMatrix4 * A,
                                                UMatrix4 * b)
{ // convert using this A matrix center image coordinates
  // to pixel values and b matrix to convert to top-left
  // oriented coordinates
  UVector4 P;
  UVector4 R;
  UVector4 C;
  //bool isImage1, isImage2;
  R = pos->asVector4();
  // R is size 4, A is size 3x4, b is size 3x3, P is size 3
  C = *A * R; // in camera coordinates
  P = *b * C; // in pixels
  // remove scale (w)
  P.normalize();
  return P;
};
*/
///////////////////////////////////////////////////////

void UImage::gridLine(float Pos, char axis,
                    float minH,
                    float maxH,
                    float minW,
                    float maxW,
                    char sw /*= 'y'*/,
                    int Color /*= 1*/)
{ // paint grid line in the image
  // within margins specified
  // axis    'x','y','z'
  // Plot sw:'y': x-z plot, H=x, W=z
  //         'z': x-y plot, H=y, W=x
  //         'x': y-z plot, H=y, W=z
  //         'n'; z-x (north = x = right) H=z, W=x
  // color by 1 : yellow (minor)
  //          2 : red  (major)
  //          3 : blue (zero)
  //          >3 : dot distance
  // returns error if not painted
  UPixel rgb;
  UPixel rgbMinor(255,200,255);
  UPixel rgbMajor(255,164,164);
  UPixel rgbZero(164,164,255);
  bool isX = false;
  int x1, x2;
  int y1, y2;
  //
  switch (sw)
  { // is pos a width value
    case 'z': isX = (axis == 'x');
              break;
    case 'y': isX = (axis == 'z');
              break;
    case 'x': isX = (axis == 'z');
              break;
    case 'n': isX = (axis == 'x');
              break;
    default:;
  }
  // minimum is left botttom
  if (isX)
  {
    x1 = roundi((Pos - minW)/(maxW - minW) * width());
    x2 = x1;
    y1 = 0;
    y2 = height()-1;
  }
  else
  {
    y1 = height() - roundi((Pos - minH)/(maxH - minH) * height());
    y2 = y1;
    x1 = 0;
    x2 = width() -1;
  }
  if (Color <4)
  {
    switch (Color)
    {
      case 1: rgb = rgbMinor;
              break;
      case 2: rgb = rgbMajor;
              break;
      case 3: rgb = rgbZero;
              break;
      default:;
    }
    cvLine( cvArr(), cvPoint(x1, y1), cvPoint(x2,y2),
            CV_RGB(rgb.p3, rgb.p2, rgb.p1),
            1, 8, 0 );
    //line(x1, y1, x2, y2, &rgb);
  }
  else
  {
    cvLine( cvArr(), cvPoint(x1, y1), cvPoint(x2,y2),
            CV_RGB(rgbMinor.p3, rgbMinor.p2, rgbMinor.p1),
            1, 4, 0 );
    //line(x1, y1, x2, y2, &rgbMinor, NULL, Color-3);
  }
}

/////////////////////////////////////////////////////////

void UImage::lineInGrid(UPosition pos1,
                             UPosition pos2,
                            float minH,
                            float maxH,
                            float minW,
                            float maxW,
                            char sw,
                            UPixel * rgb,
                            UMatrix4 * mA, /*= NULL*/
                            int lineWidth /* = 1 */)
{ // paint line in the image
  // shown from above within margins specified
  // axis    'x','y','z'
  // Plot sw:'y': x-z plot, H=x, W=y
  //         'z': x-y plot, H=y, W=x
  //         'x': y-z plot, H=y, W=z
  //         'n': (north up) H=z, W=x
  // color by 1 : yellow (minor)
  //          2 : red  (major)
  //          3 : blue (zero)
  // returns error if not painted
  float w1, w2, h1, h2;
  int x1, x2;
  int y1, y2;
  UPosition p1 = pos1;
  UPosition p2 = pos2;
  //
  if (mA != NULL)
  {
    p1.transfer(mA);
    p2.transfer(mA);
  }
  //
  switch (sw)
  { // translate to 2D view
    case 'z': // x-y view
              w1 = p1.x;
              w2 = p2.x;
              h1 = p1.y;
              h2 = p2.y;
              break;
    case 'y': // z-x view
              w1 = -p1.z; // front of camera is right
              w2 = -p2.z; // with robot in zero position
              h1 = -p1.x; // right is down
              h2 = -p2.x;
              break;
    case 'x': // z-y view
              w1 = -p1.z; // more front is right
              w2 = -p2.z; // with robot in pole position
              h1 = p1.y;
              h2 = p2.y;
              break;
    case 'n': // z-x view
              w1 = p1.x; // front of camera is right
              w2 = p2.x; // with robot in zero position
              h1 = -p1.z; // right is down
              h2 = -p2.z;
              break;
    default: w1 = 0.0; w2 = 0.0; h1=0.0; h2 = 0.0;
  }
  // minimum is left botttom
  // transfer to pixel view
  x1 = roundi((w1 - minW)/(maxW - minW) * width());
  x2 = roundi((w2 - minW)/(maxW - minW) * width());
  y1 = height() - roundi((h1 - minH)/(maxH - minH) * height());
  y2 = height() - roundi((h2 - minH)/(maxH - minH) * height());
  if (width() == 1)
    cvLine( cvArr(), cvPoint(x1, y1), cvPoint(x2,y2),
            CV_RGB(rgb->p3, rgb->p2, rgb->p1),
            1, 8, 0 );
    //line(x1, y1, x2, y2, rgb);
  else
    cvLine( cvArr(), cvPoint(x1, y1), cvPoint(x2,y2),
            CV_RGB(rgb->p3, rgb->p2, rgb->p1),
            lineWidth, 8, 0 );
    //line(x1, y1, x2, y2, rgb, NULL, lineWidth);
  //
}

/////////////////////////////////////////////////////////

void UImage::gridLines(float major, float minor,
                     char axis,
                     float minH, float maxH,
                     float minW, float maxW, char sw)
{ // paints a series of grid lines along one coordinate (axis)
  // axis specified as above. major is major line distance,
  // minor is minor line distance.
  float f;
  float minG, maxG;
  bool useH;
  float sign;
  //
  switch (sw)
  { // find axis to use
    case 'x': // H=y, W=z
      useH = (axis == 'y');
      break;
    case 'y': // H=x, W=z
      useH = (axis == 'x');
      break;
    case 'z': // H=y, W=x
      useH = (axis == 'y');
      break;
    case 'n': // H=z, W=x
      useH = (axis == 'z');
      break;
    default: // error
      useH = false;
  }
  //
  if (useH)
  { // get limits for axis to use
    minG = minH;
    maxG = maxH;
  }
  else
  {
    minG = minW;
    maxG = maxW;
  }
  //
  sign = (maxG - minG)/absf(maxG - minG);
  // paint minor lines dotted
  f = int(minG / minor) * minor;
  while ((f * sign) <= (maxG * sign))
  {
    gridLine(f, axis, minH, maxH, minW, maxW, sw, 6);
    f += minor * sign;
  }
  // paint major lines on top of minor
  f = int(minG / major) * major;
  while ((f * sign) <= (maxG * sign))
  {
    gridLine(f, axis, minH, maxH, minW, maxW, sw, 2);
    f += major * sign;
  }
  // paint zero line
  if ((minG * maxG) <= 0.0)
    gridLine(0.0, axis, minH, maxH, minW, maxW, sw, 3);
  //
}

////////////////////////////////////////////////////////

void UImage::tone(UPixel * rgb, int procent)
{
  int x,y;
  UPixel * b;
  char * d;
  for (y = 0; y < int(height()); y++)
  {
    b = getLine(y);
    for (x = 0; x < int(width()); x++)
    {
      b->p1 += (rgb->p1 - b->p1) * procent / 100;
      if (isBW())
      { // advance 1 byte only
        d = (char *)b;
        b = (UPixel *) ++d;
      }
      else
      {
        b->p2 += (rgb->p2 - b->p2) * procent / 100;
        b->p3 += (rgb->p3 - b->p3) * procent / 100;
        b++;
      }
    }
  }
}

////////////////////////////////////////////////////////

bool UImage::toRGB(UImage * dest)
{ // converts the image to RGB format
  int i;
  int pixels = int(width() * height());
  UPixel * d, *s;
  unsigned char * bw;
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    if (isRGB())
      dst->copy(this);
    else
    {
      dst->copyMeta(this, false);
      dst->setSize(height(), width(), 3, 8, "RGB");
    }
  }
  if (not isRGB())
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
      moveYUV420To(PIX_PLANES_RGB, dst);
    else if (colFormat == PIX_PLANES_YUV422)
    { // check if source and destination is the same
      UImage * src = this;
      if (dst == this)
      { // is the same, so use convert buffer
        if (convertBuffer == NULL)
          convertBuffer = new UImage800();
        src = convertBuffer;
        src->copyJustImage(this);
      }
      dest->yuv422toRGB((uint8_t *) src->getData(), src->height(), src->width());
    }
    else if (colFormat == PIX_PLANES_BGGR or
             colFormat == PIX_PLANES_RGGB or
             colFormat == PIX_PLANES_GBRG or
             colFormat == PIX_PLANES_GRBG)
      moveBGGRTo(PIX_PLANES_RGB, dst);
    else if (colFormat == PIX_PLANES_BW16S)
      moveBW16ToRGB(dst);
    else if (dst != this and colFormat == PIX_PLANES_BGR)
      cvCvtColor(cvArr(), dst->cvArr(), CV_BGR2RGB);
    else if (dst != this and colFormat == PIX_PLANES_RGBA)
      cvCvtColor(cvArr(), dst->cvArr(), CV_RGBA2RGB);
    else if (dst != this and colFormat == PIX_PLANES_BGRA)
      cvCvtColor(cvArr(), dst->cvArr(), CV_BGRA2RGB);
#ifdef OPENCV2
    else if (dst != this and colFormat == PIX_PLANES_YUV)
      cvCvtColor(cvArr(), dst->cvArr(), CV_YUV2RGB);
#endif
    else if (dst != this and colFormat == PIX_PLANES_BW)
      cvCvtColor(cvArr(), dst->cvArr(), CV_GRAY2RGB);
    else if (dst != this and colFormat == PIX_PLANES_BW)
      cvCvtColor(cvArr(), dst->cvArr(), CV_HSV2RGB);
    else
    {
      if (dst != this)
      {
        dst->copyMeta(this, false);
        dst->setSize(height(), width(), 3, 8, "RGB");
      }
      d = dst->getData();
      d += pixels - 1;
      s = getData();
      s += pixels - 1;
      bw = (unsigned char *) getData();
      bw += pixels - 1;
      if (isYUV())
      {
        for (i = 0; i < pixels; i++)
        {
          d->setYUVto(s->y, s->u, s->v, PIX_PLANES_RGB);
          d--;
          s--;
        }
      }
      else if (isBGR())
      {
        for (i = 0; i < pixels; i++)
        {
          d->set(s->p3, s->p2, s->p1);
          d--;
          s--;
        }
      }
      else if (isBW())
      {
        for (i = 0; i < pixels; i++)
        {
          d->set(*bw, *bw, *bw);
          d--;
          bw--;
        }
      }
    }
    //colFormat = PIX_PLANES_RGB;
    if (isBW() or isYUV420())
      dst->setSize(height(), width(), 3, 8, "RGB");
    else
      dst->setColorType("RGB");
  };
  return dst->isRGB();
}

////////////////////////////////////////////////////////

bool UImage::toBGR(UImage * dest)
{ // converts the image to BGR format
  int i;
  int pixels = int(width() * height());
  UPixel * d, *s;
  unsigned char * bw;
  UImage * dst = dest;
  bool isOK = true;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    if (isBGR())
      isOK = dst->copy(this);
    else
    {
      dst->copyMeta(this, false);
      isOK = dst->setSize(height(), width(), 3, 8, "BGR");
    }
  }
  if (isOK and not isBGR())
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
      moveYUV420To(PIX_PLANES_BGR, dst);
    else if (colFormat == PIX_PLANES_YUV422)
      dest->yuv422toBGR((uint8_t *) getData(), height(), width());
    else if (isBayer())
      moveBGGRTo(PIX_PLANES_BGR, dst);
    else if (colFormat == PIX_PLANES_BW16S)
    {
      moveBW16ToRGB(dst);
      dst->setColorType(PIX_PLANES_BGR);
    }
    else if (dst != this and colFormat == PIX_PLANES_RGB)
      cvCvtColor(cvArr(), dst->cvArr(), CV_RGB2BGR);
    else if (dst != this and colFormat == PIX_PLANES_RGBA)
      cvCvtColor(cvArr(), dst->cvArr(), CV_RGBA2BGR);
    else if (dst != this and colFormat == PIX_PLANES_BGRA)
      cvCvtColor(cvArr(), dst->cvArr(), CV_BGRA2BGR);
#ifdef OPENCV2
    else if (dst != this and colFormat == PIX_PLANES_YUV)
      cvCvtColor(cvArr(), dst->cvArr(), CV_YUV2BGR);
#endif
    else if (dst != this and colFormat == PIX_PLANES_BW)
      cvCvtColor(cvArr(), dst->cvArr(), CV_GRAY2BGR);
    else
    {
      d = dst->getData();
      s = getData();
      d += pixels - 1;
      s += pixels - 1;
      if (isYUV())
      {
        for (i = 0; i < pixels; i++)
        {
          d->setYUVto(s->y, s->u, s->v, PIX_PLANES_BGR);
          d--;
          s--;
        }
        if (not dst->isBGR())
          dst->setColorType("BGR");
      }
      else if (isRGB())
      {
        for (i = 0; i < pixels; i++)
        {
          d->set(s->p3, s->p2, s->p1);
          d--;
          s--;
        }
        if (not dst->isBGR())
          dst->setColorType("BGR");
      }
      else if (isBW())
      {
        bw = (unsigned char *) getData();
        bw += pixels - 1;
        for (i = 0; i < pixels; i++)
        {
          d->set(*bw, *bw, *bw);
          d--;
          bw--;
        }
        if (not dst->isBGR())
          dst->setSize(height(), width(), 3, 8, "BGR");
      }
    }
  }
  return isOK and dst->isBGR();
}

//////////////////////////////////////////////////////

void UImage::moveBW16ToRGB(UImage * dest)
{ // get last pixel BW
  uint16_t * up = (uint16_t *) getData() + width() * height() - 1;
  UPixel * pp;
  // change to destination color type
  UImage * dst = dest;
  if (dst == NULL)
    dst = this;
  if (dst != this)
    dst->copyMeta(this, false);
  // set destination new color type
  dst->setColorType(PIX_PLANES_RGB);
  pp = &dst->getLine(dst->height() - 1)[dst->width() - 1];
  for (int i = 0; i < int(width()*height()); i++)
  {
    int a, b;
    a = *up >> 8;
    b = *up & 0xff;
    pp->set(127, a, b);
    pp--;
    up--;
  }
}


////////////////////////////////////////////////////////

bool UImage::colourSaturate(double gain)
{ // magnify color part
  int i;
  const int pixels = int(width() * height());
  UPixel * d, yuv;
  unsigned char * uv;
  int fac = roundi(gain * 8.0);
  //
  if (not isBW())
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
    {
      uv = getUline(0);
      for (i = 0; i < pixels/2; i++)
      { // enhance both U and V
        *uv = mini(0xff, maxi(0, (((int(*uv) - 128) * fac) >> 3) + 128));
        uv++;
      }
    }
    else if (colFormat == PIX_PLANES_YUV422)
    {
      uv = getUCharRef(0,0);
      for (i = 1; i < pixels * 2; i += 2)
      { // enhance both U and V
        *uv = mini(0xff, maxi(0, (((int(*uv) - 128) * fac) >> 3) + 128));
        uv++;
      }
    }
    else
    {
      d = getData();
      if (isYUV())
      {
        for (i = 0; i < pixels; i++)
        {
          d->u = mini(0xff, maxi(0, (((int(d->u) - 128) * fac) >> 3) + 128));
          d->v = mini(0xff, maxi(0, (((int(d->v) - 128) * fac) >> 3) + 128));
          d++;
        }
      }
      else if (isRGB())
      {
        for (i = 0; i < pixels; i++)
        {
          yuv.setRGBto(d->p1, d->p2, d->p3, PIX_PLANES_YUV);
          yuv.u = mini(0xff, maxi(0, (((int(yuv.u) - 128) * fac) >> 3) + 128));
          yuv.v = mini(0xff, maxi(0, (((int(yuv.v) - 128) * fac) >> 3) + 128));
          d->setYUVto(yuv.y, yuv.u, yuv.v, PIX_PLANES_RGB);
          d++;
        }
      }
      else if (isBGR())
      {
        for (i = 0; i < pixels; i++)
        {
          yuv.setRGBto(d->p3, d->p2, d->p1, PIX_PLANES_YUV);
          yuv.u = mini(0xff, maxi(0, (((int(yuv.u) - 128) * fac) >> 3) + 128));
          yuv.v = mini(0xff, maxi(0, (((int(yuv.v) - 128) * fac) >> 3) + 128));
          d->setYUVto(yuv.y, yuv.u, yuv.v, PIX_PLANES_BGR);
          d++;
        }
      }
    }
  }
  return not isBW();
}

////////////////////////////////////////////////////////

bool UImage::toCromaBGR(UImage * dest)
{ // converts the image to BGR format
  int i;
  int pixels = int(width() * height());
  UPixel * d, *s;
  UPixel bgr;
  unsigned char * bw;
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    dst->copyMeta(this, false);
    dst->setSize(height(), width(), 3, 8, "BGR");
  }
  // convert all pixels
  if (colFormat == PIX_PLANES_YUV420)
    moveYUV420To(PIX_PLANES_BGR, dst);
  else if (colFormat == PIX_PLANES_YUV422 or isBayer())
    toBGR(dst);
  // then make to cromatisity values, BGR coded
  d = dst->getData();
  d += pixels - 1;
  s = getData();
  s += pixels - 1;
  bw = (unsigned char *) getData();
  bw += pixels - 1;
  for (i = 0; i < pixels; i++)
  {
    if (isYUV())
      bgr.setYUVto(s->y, s->u, s->v, PIX_PLANES_BGR);
    else if (isRGB())
      bgr.set(s->p3, s->p2, s->p1);
    else if (isBW())
      bgr.set(*bw, *bw, *bw);
    else
      bgr = *s;
    // convert to cromatisity (in BGR format)
    *d = bgr.asCromaBGR(PIX_PLANES_BGR);
    d--;
    s--;
    bw--;
  }
  //colFormat = PIX_PLANES_BGR;
  if (isBW() or isYUV420())
    dst->setSize(height(), width(), 3, 8, "BGR");
  else
    dst->setColorType("BGR");
  return dst->isBGR();
}

////////////////////////////////////////////////////////

bool UImage::toBW(UImage * dest)
{
  bool result;
  int i;
  int pixels = int(width() * height());
  UPixel *s;
  UPixel px;
  unsigned char * bw;
  word * bw16;
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  result = isBW();
  if (dst != this)
  {
    if (result)
      dst->copy(this);
    else
    { // set correct destination size and format
      dst->copyMeta(this, false);
      dst->setSize(height(), width(), 1, 8, "BW");
    }
  }
  if (not result)
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
      moveYUV420To(PIX_PLANES_BW, dst);
    else if (colFormat == PIX_PLANES_BGGR)
      moveBGGRTo(PIX_PLANES_BW, dst);
    else if (colFormat == PIX_PLANES_YUV422)
      dest->yuv422toBW(getUCharRef(0, 0), height(), width());
    else if ((dst != this) and (colFormat == PIX_PLANES_RGBA))
      cvCvtColor(cvArr(), dst->cvArr(), CV_RGBA2GRAY);
    else if ((dst != this) and (colFormat == PIX_PLANES_BGRA))
      cvCvtColor(cvArr(), dst->cvArr(), CV_BGRA2GRAY);
    else if ((dst != this) and (colFormat == PIX_PLANES_RGB))
      cvCvtColor(cvArr(), dst->cvArr(), CV_RGB2GRAY);
    else if ((dst != this) and (colFormat == PIX_PLANES_BGR))
      cvCvtColor(cvArr(), dst->cvArr(), CV_BGR2GRAY);
    else
    {
      //d = dst->getData();
      s = getData();
      bw = (unsigned char *)dst->getData();
      bw16 = (word *) getData();
      for (i=0; i < pixels; i++)
      {
        if (isRGB())
          px.setRGBto(s->p1, s->p2, s->p3, PIX_PLANES_BW);
        else if (isBGR())
          px.setRGBto(s->p3, s->p2, s->p1, PIX_PLANES_BW);
        else if (isYUV())
          px.setYUVto(s->y, s->u, s->v, PIX_PLANES_BW);
        else if (isBW16s())
          px.y = maxi(0, mini(255, *bw16 + 128));
        else if (isBW16u())
          px.y = maxi(0, mini(255, *bw16));
        else
          break;
        *bw = px.y;
        //d++;
        s++;
        bw++;
        bw16++;
      }
    }
    //colFormat = PIX_PLANES_YUV;
    if (dst != this)
      dst->setSize(height(), width(), 1, 8, "BW");
    result = true;
  };
  return result;
}

////////////////////////////////////////////////

bool UImage::toHalf()
{ // Downsample the image
  //bool result = false;
  //
  if (not isYUV())
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
      moveYUV420ToHalf();
    else if (colFormat == PIX_PLANES_YUV422)
      yuv422toRGBhalf(getUCharRef(0,0), height(), width());
    else if (colFormat == PIX_PLANES_RGGB)
    { // convert as if BGGR
      moveBGGRToHalfBGR();
      // and then correct color order setting
      setColorType(PIX_PLANES_RGB);
    }
    else if (colFormat == PIX_PLANES_BGGR or
             colFormat == PIX_PLANES_GBRG or
             colFormat == PIX_PLANES_GRBG)
      moveBGGRToHalfBGR();
    else if (colFormat == PIX_PLANES_BW)
      moveBWToHalf();
    else
      movePixToHalf();
  }
  return true;
}



////////////////////////////////////////////////

bool UImage::toYUV(UImage * dest)
{ // converts the image to RGB format
  bool result = false;
  int i;
  int pixels = int(width() * height());
  UPixel * d, *s;
  unsigned char * bw;
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    if (isYUV())
      dst->copy(this);
    else
    {
      dst->copyMeta(this, false);
      dst->setSize(height(), width(), 3, 8, "BGR");
    }
  }
  //
  if (not isYUV())
  { // convert all pixels
    if (colFormat == PIX_PLANES_YUV420)
      moveYUV420To(PIX_PLANES_YUV, dst);
    else if (colFormat == PIX_PLANES_YUV422)
    {
      UImage * src = this;
      if (dst == this)
      {
        if (convertBuffer == NULL)
          convertBuffer = new UImage800();
        src = convertBuffer;
        src->copyJustImage(this);
      }
      yuv422toYUV(src->getUCharRef(0, 0), src->height(), src->width());
    }
    else if (isBayer())
    {
      moveBGGRTo(PIX_PLANES_YUV, dst);
      //printf(".");
    }
    else
    { // either BW, RGB or BGR
      d = dst->getData();
      d += pixels - 1;
      s = getData();
      s += pixels - 1;
      if (isRGB())
      {
        for (i=0; i < pixels; i++)
        {
          *d = d->RGBtoYUV(s->p1, s->p2, s->p3);
//          d->setRGBto(s->p1, s->p2, s->p3, PIX_PLANES_YUV);
          d--;
          s--;
        }
      }
      else if (isBGR())
      {
        for (i=0; i < pixels; i++)
        {
          *d = d->RGBtoYUV(s->p3, s->p2, s->p1);
//          d->setRGBto(s->p3, s->p2, s->p1, PIX_PLANES_YUV);
          d--;
          s--;
        }
      }
      else if (isBW())
      {
        bw = (unsigned char *) getData();
        bw += pixels - 1;
        for (i=0; i < pixels; i++)
        {
          d->set(*bw, 128, 128);
          d--;
          bw--;
        }
      }
    }
    //colFormat = PIX_PLANES_YUV;
    if (img.nChannels != 3)
      dst->setSize(height(), width(), 3, 8, "YUV");
    else
      dst->setColorType("YUV");
    //
    result = dst->isYUV();
  }
  return result;
}

////////////////////////////////////////////////////////

bool UImage::invertY()
{ // invert intensity and keep color
  bool result;
  int i;
  int pixels = int(width() * height());
  UPixel* d;
  //
  result = not isBW();
  // convert all pixels
  if (result)
  {
    d = getData();
    for (i = 0; i < pixels; i++)
    {
      d->y = 255 - d->y;
      d++;
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////
// clear to this value
void UImage::clear(UPixel rgb)
{
  int i;
  UPixel * d;
  int n;
  //
  if (width() * height() > maxPixels())
  { // reduce size
    setMaxSize43();
    printf("UImage::clear: Image width error");
  }
  //
  d = getData();
  n = int(height() * width());
  for (i = 0; i < n; i++)
    *d++ = rgb;
  valid = true;
}

////////////////////////////////////////////////////////

void UImage::clear(void)
{ // clear to black (zero)
  bzero(getData(), imgBytes());
}

////////////////////////////////////////////////////////

void UImage::clear(int v)
{ // clear to byte value v
  memset(getData(), v, imgBytes());
  valid = true;
}

////////////////////////////////////////////////////////

void UImage::shiftRows(int rows, int byteFill)
{ // mowe rows up or down (+ mowes down)
  char * p1;
  char * p2;
  int bytes, i;
  bool fill = byteFill >= 0;
  unsigned char f = byteFill;
  //
  if ((absi(rows) < (int)height()) and (rows != 0))
  {
    p1 = (char *)getLine(0);
    p2 = (char *)getLine(absi(rows));
    bytes = (height() - absi(rows)) * width() * getChannels();
    if (rows > 0)
      memmove(p2, p1, bytes);
    else
      memmove(p1, p2, bytes);
    if (fill)
    {
      bytes = absi(rows) * width() * getChannels();
      if (rows < 0)
        p1 = (char *)getLine(height() - absi(rows));
      for (i = 0; i < bytes; i++)
        *p1++ = f;
    }
  }
}

////////////////////////////////////

void UImage::shiftCols(int cols, int byteFill)
{ // mowe cols left or right (+ mowes right)
  char * p1;
  char * p2;
  unsigned int bytes;
  unsigned int row, col;
  bool fill = byteFill >= 0;
  unsigned char f = byteFill;
  //
  if ((absi(cols) < (int)width()) and (cols != 0))
  {
    for (row = 0; row < height(); row++)
    {
      p1 = (char *)getLine(row);
      p2 = &p1[absi(cols) * getChannels()];
      bytes = (width() - absi(cols)) * getChannels();
      if (cols > 0)
        memmove(p2, p1, bytes);
      else
        memmove(p1, p2, bytes);
      if (fill)
      { // fill the empty part with fill byte
        bytes = absi(cols) * getChannels();
        if (cols < 0)
          p1 = &p1[(width() - absi(cols)) * getChannels()];
        for (col = 0; col < bytes; col++)
          *p1++ = f;
      }
    }
  }
}


////////////////////////////////////

/*
void UImage::clearAll(void)
{
  bufferBytes = 0;
  //pImage = vImage;
  cvInitImageHeader(&img, // image header
           cvSize(1,1),  // image size (w,h)
           IPL_DEPTH_8U, // 8-bit unsigned
           3);  // channels
  //height = 0;
  //width = 0;
  valid = true;
  used = 0;
  setColorType("BGR");
  isRGB = false;
  radialErrorRemoved = false;
  cam = NULL;
  imageNumber = 0;
  shutterSpeed = 3000;
  name[0] = '\0';
  saveType[0] = '\0';
  pos.clear();
  rot.clear();
  imgTime.Now();
}
*/
/////////////////////////////////////////////////////////////////

bool UImage::copyJustImage(UImage * source)
{ // copy raw image data from source image
  bool result = false;
  //
  if (source != NULL)
  {
    if (source->imgBytes() > maxBytes())
      result = resize(source->height(), source->width(),
                 source->getChannels(), source->getDepth());
    if (source->imgBytes() <= maxBytes())
    {
      result = setSize(source->height(), source->width(),
                 source->getChannels(), source->getDepth());
      memcpy(getData(), source->getData(), source->imgBytes());
      used = 0;
      //result = true;
    }
    else
      printf("UImage::copyJustImage: Image too big (%d bytes) for buffer of %d bytes)", source->imgBytes(), maxBytes());
  }
  return result;
}

///////////////////////////////////////////////////////////////////

bool UImage::copyToMaxRes(UImage * source)
{ // copy raw image data from source image and expand size
  // to max image resolution
  bool result = false;
  /*
  int i;
  UPixel * prs; // source pixel
  UPixel ps;
  UPixel * prd; // destination row
  int sr, dr; // source and destination row;
  int sc, dc; // source and destination column
  */
  //
  result = (source != NULL);
  if (result)
  { // copy name, format timestamp etc
    copyMeta(source, true);
    if (height() < 10)
      setSizeOnly(480, 640);
    while (height() < 400);
      setSizeOnly(height() * 2, width() * 2);
    cvResize(source->cvArr(), cvArr(), CV_INTER_NN);
  }
  return result;
}

///////////////////////////////////////////////////////////////////

bool UImage::copyAndScale(UImage * source, double factor)
{ // copy raw image data from source image and resize
  bool result = false;
  //
  result = (source != NULL);
  if (result)
    // copy name, format timestamp etc
    result = copyMeta(source, true);
  if (result and height() < 10)
      result = resize(240, 320, source->getChannels(), source->getDepth());
  else if (result)
      result = resize(roundi(height() * factor), roundi(width() * factor), source->getChannels(), source->getDepth());
  if (result)
    cvResize(source->cvArr(), cvArr(), CV_INTER_NN);
  return result;
}

//////////////////////////////////////////////////

bool UImage::copyScaleDown(UImage * source, int factor)
{ // copy raw image data from source image and expand size
  // to max image resolution
  bool result;
  UPixel * prs; // source pixel
  UPixel ps;
  UPixel * prd; // destination row
  int dr; // source and destination row;
  int dc; // source and destination column
  unsigned int w, h;
  //
  result = ((source != NULL) and (factor > 0));
  if (result)
  {
    w = source->width() / factor;
    h = source->height() / factor;
    result = setSize(h, w, source->getChannels(),
                           source->getDepth());
  }
  if (result)
  {
    imgTime = source->imgTime;
    cam = source->cam;
    imageNumber = source->imageNumber;
    strcpy(name, source->name);
    strcpy(saveType, source->saveType);
    colFormat = source->colFormat;
    pos = source->pos;
    rot = source->rot;
    radialErrorRemoved = source->radialErrorRemoved;
    // then the pixels
    for (dr = 0; dr < int(height()); dr++)
    { // get first row pointers
      prd = getLine(dr);
      prs = source->getLine(dr * factor + factor / 2);
      prs = &prs[factor/2];
      for (dc = 0; dc < int(width()); dc++)
      {
        *prd = *prs;
        prd++;
        prs = &prs[factor];
      }
    }
    used = 0;
    valid = true;
  }
  return result;
}

//////////////////////////////////////////////////

bool UImage::copyMeta(UImage * source, bool andSize)
{
  // copy just meta data
  if (andSize)
  { // not just name and position
    resize(source->height(), source->width(),
              source->getChannels(), source->getDepth());
  }
  setColorType(source->colFormat);
  pos = source->pos;
  rot = source->rot;
  cam = source->cam;
  camDevice = source->camDevice;
  hdrSet = source->hdrSet;
  readDelay = source->readDelay;
  strcpy(name, source->name);
  imageNumber = source->imageNumber;
  imgTime = source->imgTime + 0.000001;
  radialErrorRemoved = source->radialErrorRemoved;
  strcpy(saveType, source->saveType);
  used = 0;
  return source->getDataSize() <= getBufferSize();
}

//////////////////////////////////////////////////
/**
Copy all image data  regardless of resolution */
bool UImage::copy(UImage * source)
{
  bool result;
  //
  result = copyMeta(source, true);
  if (result)
    // copy image data
    result = copyJustImage(source);
  return result;
}

//////////////////////////////////////////

UPixel UImage::pixRGB(unsigned char r,
                unsigned char g,
                unsigned char b)
{
  UPixel result;
  //
  switch (colFormat)
  {
    case PIX_PLANES_BW:
      result.y = (r + g + b)/3;
      break;
    case PIX_PLANES_RGB:
      result.set(r, g, b);
      break;
    case PIX_PLANES_BGR:
      result.set(b, g, r);
      break;
    case PIX_PLANES_YUV:
      result = UPixel::RGBtoYUV(r, g, b);
      break;
    default:
      result.set(b, g, r);
      break;
  }
  return result;
}

//////////////////////////////////////////

bool UImage::setColorType(int colorFormat)
{
  bool result = true;
  //
  switch(colorFormat)
  {
    case PIX_PLANES_BW:
      colFormat = PIX_PLANES_BW;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      break;
    case PIX_PLANES_RGGB:
      colFormat = PIX_PLANES_RGGB;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      break;
    case PIX_PLANES_BGGR:
      colFormat = PIX_PLANES_BGGR;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      break;
    case PIX_PLANES_GBRG:
      colFormat = PIX_PLANES_GBRG;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      break;
    case PIX_PLANES_GRBG:
      colFormat = PIX_PLANES_GRBG;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      break;
    case PIX_PLANES_RGB:
      colFormat = PIX_PLANES_RGB;
      // set default color model
      strncpy(img.colorModel, "RGB", 4);
      strncpy(img.channelSeq, "RGB", 4);
      img.nChannels = 3;
      img.depth = 8;
      break;
    case PIX_PLANES_BGR:
      colFormat = PIX_PLANES_BGR;
      // set default color model
      strncpy(img.colorModel, "RGB", 4);
      strncpy(img.channelSeq, "BGR", 4);
      img.nChannels = 3;
      img.depth = 8;
      break;
    case PIX_PLANES_YUV:
      colFormat = PIX_PLANES_YUV;
      // set default color model
      strncpy(img.colorModel, "YUV", 4);
      strncpy(img.channelSeq, "YUV", 4);
      img.nChannels = 3;
      img.depth = 8;
      break;
    case PIX_PLANES_YUV420:
      colFormat = PIX_PLANES_YUV420;
      // set default color model
      strncpy(img.colorModel, "YUV2", 4);
      strncpy(img.channelSeq, "YUV", 4);
      img.nChannels = 3;
      img.depth = 8;
      break;
    case PIX_PLANES_YUV422:
      colFormat = PIX_PLANES_YUV422;
      // set default color model
      strncpy(img.colorModel, "YUV4", 4);
      strncpy(img.channelSeq, "YUV", 4);
      img.nChannels = 2;
      img.depth = 8;
      break;
    case PIX_PLANES_BGRA:
      colFormat = PIX_PLANES_BGRA;
      // set default color model
      strncpy(img.colorModel, "RGBA", 4);
      strncpy(img.channelSeq, "BGRA", 4);
      img.nChannels = 1;
      img.depth = 32;
      break;
    case PIX_PLANES_RGBA:
      colFormat = PIX_PLANES_RGBA;
      // set default color model
      strncpy(img.colorModel, "RGBA", 4);
      strncpy(img.channelSeq, "RGBA", 4);
      img.nChannels = 1;
      img.depth = 32;
      break;
    default:
      colFormat = colorFormat;
      // set color model to BW (not used)
      strncpy(img.colorModel, "GRAY",4);
      strncpy(img.channelSeq, "GRAY",4);
      img.nChannels = 1;
      img.depth = 8;
      //result = false;
      break;
  }
  img.widthStep = img.width * img.nChannels * (img.depth / 8);
  return result;
}

//////////////////////////////////////////

int UImage::setColorType(const char * col)
{
  //PIX_PLANES_RGGB
  if ((strcasecmp(col, "BW") == 0) or (strcasecmp(col, "GRAY") == 0))
  {
    colFormat = PIX_PLANES_BW;
    // set color model to BW
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else if (strcasecmp(col, "RGGB") == 0)
  { // bayer coded colour
    colFormat = PIX_PLANES_RGGB;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else if (strcasecmp(col, "BGGR") == 0)
  { // bayer coded colour
    colFormat = PIX_PLANES_BGGR;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else if (strcasecmp(col, "GRBG") == 0)
  { // bayer coded colour
    colFormat = PIX_PLANES_GRBG;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else if (strcasecmp(col, "GBRG") == 0)
  { // bayer coded colour
    colFormat = PIX_PLANES_GBRG;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else if (strcasecmp(col, "BW16S") == 0)
  {
    colFormat = PIX_PLANES_BW16S;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 16; // = IPL_DEPTH_16S;
  }
  else if (strcasecmp(col, "BW16U") == 0)
  {
    colFormat = PIX_PLANES_BW16U;
    // set color model to BW (not used)
    strncpy(img.colorModel, "GRAY",4);
    strncpy(img.channelSeq, "GRAY",4);
    img.nChannels = 1;
    img.depth = 16; // = IPL_DEPTH_16S;
  }
  else if (strcasecmp(col, "RGB") == 0)
  {
    colFormat = PIX_PLANES_RGB;
    // set color model
    strncpy(img.colorModel, "RGB", 4);
    strncpy(img.channelSeq, "RGB", 4);
    img.nChannels = 3;
    img.depth = 8;
  }
  else if (strcasecmp(col, "BGR") == 0)
  {
    colFormat = PIX_PLANES_BGR;
    // set color model
    strncpy(img.colorModel, "RGB", 4);
    strncpy(img.channelSeq, "BGR", 4);
    img.nChannels = 3;
    img.depth = 8;
  }
  else if (strcasecmp(col, "YUV") == 0)
  {
    colFormat = PIX_PLANES_YUV;
    // set color model
    strncpy(img.colorModel, "YUV", 4);
    strncpy(img.channelSeq, "YUV", 4);
    img.nChannels = 3;
    img.depth = 8;
  }
  else if (strcasecmp(col, "YUV420") == 0)
  {
    colFormat = PIX_PLANES_YUV420;
    // set color model
    strncpy(img.colorModel, "YUV2", 4);
    strncpy(img.channelSeq, "YUV", 4);
    img.nChannels = 3;
    img.depth = 8;
  }
  else if (strcasecmp(col, "YUV422") == 0)
  {
    colFormat = PIX_PLANES_YUV422;
    // set color model
    strncpy(img.colorModel, "YUV4", 4);
    strncpy(img.channelSeq, "YUV", 4);
    img.nChannels = 2;
    img.depth = 8;
  }
  else if (strcasecmp(col, "RGBA") == 0)
  {
    colFormat = PIX_PLANES_RGBA;
    // set color model
    strncpy(img.colorModel, "RGBA", 4);
    strncpy(img.channelSeq, "RGBA", 4);
    img.nChannels = 4;
    img.depth = 8;
  }
  else if (strcasecmp(col, "BGRA") == 0)
  {
    colFormat = PIX_PLANES_BGRA;
    // set color model
    strncpy(img.colorModel, "RGBA", 4);
    strncpy(img.channelSeq, "BGRA", 4);
    img.nChannels = 4;
    img.depth = 8;
  }
  else if (strcasecmp(col, "bayer") == 0)
  {
    colFormat = PIX_PLANES_BAYER;
    // set color model
    strncpy(img.colorModel, "GRAY", 4);
    strncpy(img.channelSeq, "GRAY", 4);
    img.nChannels = 1;
    img.depth = 8;
  }
  else
  { // unknown format - assume the worst, i.e. 4 bytes per pixel
    colFormat = PIX_PLANES_BGRA;
    // set default color model
    strncpy(img.colorModel, "RGBA", 4);
    strncpy(img.channelSeq, "BGRA", 4);
    img.nChannels = 4;
    img.depth = 8;
  }
  return colFormat;
}

///////////////////////////////////////////

int UImage::toColFormatInt(const char * col)
{
  int colFormat;
  if ((strcasecmp(col, "BW") == 0) or (strcasecmp(col, "GRAY") == 0))
    colFormat = PIX_PLANES_BW;
  else if (strcasecmp(col, "RGGB") == 0)
    // bayer coded colour
    colFormat = PIX_PLANES_RGGB;
  else if (strcasecmp(col, "BGGR") == 0)
    colFormat = PIX_PLANES_BGGR;
  else if (strcasecmp(col, "GBRG") == 0)
    colFormat = PIX_PLANES_GBRG;
  else if (strcasecmp(col, "GRBG") == 0)
    colFormat = PIX_PLANES_GRBG;
  else if (strcasecmp(col, "BW16S") == 0)
    colFormat = PIX_PLANES_BW16S;
  else if (strcasecmp(col, "RGB") == 0)
    colFormat = PIX_PLANES_RGB;
  else if (strcasecmp(col, "BGR") == 0)
    colFormat = PIX_PLANES_BGR;
  else if (strcasecmp(col, "YUV") == 0)
    colFormat = PIX_PLANES_YUV;
  else if (strcasecmp(col, "YUV420") == 0)
    colFormat = PIX_PLANES_YUV420;
  else if (strcasecmp(col, "YUV422") == 0)
    colFormat = PIX_PLANES_YUV422;
  else if (strcasecmp(col, "RGBA") == 0)
    colFormat = PIX_PLANES_RGBA;
  else if (strcasecmp(col, "BGRA") == 0)
    colFormat = PIX_PLANES_BGRA;
  else if (strcasecmp(col, "BAYER") == 0)
    colFormat = PIX_PLANES_BAYER;
  else
    colFormat = PIX_PLANES_BW;
  return colFormat;
}


//////////////////////////////////////////

const char * UImage::getColorTypeString()
{
  const char * result = NULL;
  //PIX_PLANES_RGGB
  switch (colFormat)
  {
    case  PIX_PLANES_BW:
      result = "GRAY"; break;
    case PIX_PLANES_RGGB:
      result = "RGGB"; break;
    case PIX_PLANES_BGGR:
      result = "BGGR"; break;
    case PIX_PLANES_GBRG:
      result = "GBRG"; break;
    case PIX_PLANES_GRBG:
      result = "GRBG"; break;
    case PIX_PLANES_BW16S:
      result = "BW16S"; break;
    case PIX_PLANES_RGB:
      result = "RGB"; break;
    case PIX_PLANES_BGR:
      result = "BGR"; break;
    case PIX_PLANES_YUV:
      result = "YUV"; break;
    case PIX_PLANES_YUV420:
      result = "YUV420"; break;
    case PIX_PLANES_YUV422:
      result = "YUV422"; break;
    case PIX_PLANES_RGBA:
      result = "RGBA"; break;
    case PIX_PLANES_BGRA:
      result = "BGRA"; break;
    case PIX_PLANES_BAYER:
      result = "BAYER"; break;
    default:
      result="unknown"; break;
  }
  return result;
}


//////////////////////////////////////////

UPixel UImage::pixYUV(unsigned char y,
                unsigned char u,
                unsigned char v)
{
  UPixel result;
  //
  switch (colFormat)
  {
    case PIX_PLANES_BW:
      result.y = y;
      break;
    case PIX_PLANES_RGB:
      result = UPixel::YUVtoRGB(y, u, v);
      break;
    case PIX_PLANES_BGR:
      result = UPixel::YUVtoBGR(y, u, v);
      break;
    case PIX_PLANES_YUV:
      result.set(y, u, v);
      break;
    default:
      result.set(y, u, v);
      break;
  }
  return result;
}

//////////////////////////////////////////

#ifndef NO_OPENCV_HIGHGUI

////////////////////////////////////////////////////////

bool UImage::save(const char * basename, const char * ext, const char * path)
{
  bool result;
  char fn[MAX_FILENAME_SIZE] = "";
  //
  result = valid;
  if (result)
  { // make filename
    setNameAndExt(basename, ext);
    if (path != NULL)
      snprintf(fn , MAX_FILENAME_SIZE, "%s/%s.%s", path, name, saveType);
    else
      // save to work dir
      snprintf(fn , MAX_FILENAME_SIZE, "%s.%s", name, saveType);
  }
  // save image
  if (result)
    cvSaveImage(fn, cvArr());
  //
  return result;
}

////////////////////////////////////////////////////////


bool UImage::load(const char * basename, const char * ext, const char * path)
{
  bool result;
  char fn[MAX_FILENAME_SIZE] = "";
  IplImage * im = NULL;
  unsigned int w, h;
  //
  result = valid;
  if (result)
  { // construct filename
    setNameAndExt(basename, ext);
    if (path != NULL)
      snprintf(fn , MAX_FILENAME_SIZE, "%s/%s.%s", path, name, saveType);
    else
      // use work dir
      snprintf(fn , MAX_FILENAME_SIZE, "%s.%s", name, saveType);
  }
  // load image
  if (result)
  {  // load image to temp storage
    im = cvLoadImage(fn, -1); // load both BW and color
    result = (im != NULL);
  }
  // copy image to local buffer
  if (result)
  { // save image copy to local buffer
    if (im->nChannels == 1)
      setColorType("BW");
    else
      setColorType("BGR");
    w = im->width;
    h = im->height;
    if ((w * h * im->nChannels * im->depth/8) <= maxBytes())
    {  // space for full image
      setSize(h, w, im->nChannels, im->depth);
      // copy pixels (all)
      cvCopy(im, &img);
    }
    else
    { // limit size to before load size - or less
      w = mini(width(), w);
      h = mini(height(), h);
      setSize(h, w, im->nChannels, im->depth);
      // set part to copy
      cvSetImageROI(im, cvRect( 0, 0, w, h));
      // copy pixels (ROI only)
      cvCopy(im, &img);
      // release allocated ROI structure.
      cvResetImageROI(im);
    }
  }
  //
  if (result)
  { // save image name
    strncpy(name, basename, MAX_IMG_NAME_SIZE);
    // ensure zero terminated
    name[MAX_IMG_NAME_SIZE-1] = '\0';
  }
  // release dynmically allocated image storage space
  cvReleaseImage(&im);
  //
  return result;
}

#endif
///////////////////////////////////////////////////////////////////

bool UImage::saveBMP(const char path[], const char basename[],
                     const int num,  const char nameAdd[])
{
  const int FN_SIZE = 500;
  char fn[FN_SIZE];
  const char * add = nameAdd;
  //
  if (add == NULL)
    add = "";
  if (num == -1)
    snprintf(name, MAX_IMG_NAME_SIZE, "%s%s", basename, add);
  else
    snprintf(name, MAX_IMG_NAME_SIZE, "%s-%02d%s", basename, num, add);
  // ensure nameto be terminated
  name[MAX_IMG_NAME_SIZE-1] = '\0';
  //
  snprintf(fn, FN_SIZE, "%s/%s.bmp", path, name);
  //
  return saveBMP(fn);
}

///////////////////////////////////////////////////////////////////

bool UImage::loadBMP(const char path[], const char basename[],
                     const int num,  const char nameAdd[])
{
  const int FN_SIZE = 500;
  char fn[FN_SIZE];
  // e.g. saveBMP("/home/chr", "img0045", 3, "Gaus3x3"); is equvivalent with
  //      saveBMP("/home/chr/img0045-03Gaus3x3.bmp");
  if (num == -1)
    snprintf(name, MAX_IMG_NAME_SIZE, "%s%s", basename, nameAdd);
  else
    snprintf(name, MAX_IMG_NAME_SIZE, "%s-%02d%s", basename, num, nameAdd);
  // ensure nameto be terminated
  name[MAX_IMG_NAME_SIZE-1] = '\0';
  //
  snprintf(fn, FN_SIZE, "%s/%s.bmp", path, name);
  //
  return loadBMP(fn);
}

///////////////////////////////////////////////////////////////////

bool UImage::saveBMP(const char filename[])
{
  // @todo this (URawImage::save()) function is untested
  bool result = true;
  int i, j, l;
  GBM gbm;     // w,h,bpp (width, height bit per pixel + buffer)
  unsigned char * dataBuf = NULL;
  int dataBufSize;
  unsigned char * data; // pointer to data
  int fd = -1; // file descriptor
  UPixel yuv;
  UPixel rgb;
  UPixel * pix;
  int lineW;
  UImage * tmp = this;
  const char * fn = filename;
  char * fnBuff = NULL;
  const char *p2 = NULL;
  //
  // copy image data to buffer in right order
  result = valid and (filename != NULL);
  if (result)
  { // allocate buffer space + 3 extra bytes each line to allow 24bit format for BW image
    dataBufSize = getWidth() * getHeight() * getChannels() * getDepth() / 8 + 3 * getHeight(); //  + 3;
    dataBuf = (unsigned char *)malloc(dataBufSize);
    result = (dataBuf != NULL);
  }
  if (result)
  { // change odd color formats
/*    if (getColorType() == PIX_PLANES_RGGB or
        getColorType() == PIX_PLANES_BGGR or
        getColorType() == PIX_PLANES_GBRG or
        getColorType() == PIX_PLANES_GRBG)
    {
      tmp = new UImage();
      tmp->setSize(height(), width(), 3, 8, "RGB");
      toBGR(tmp);
    }*/
    if (isBayer() or tmp->getDepth() > 8)
    {
      const char * p1;
      p1 = strrchr(filename, '/');
      if (p1 == NULL)
        p1 = filename;
      else
        p1++;
      if ((getColorType() == PIX_PLANES_BGGR) and (strncmp(p1, "BGGR", 4) != 0))
        p2 = "BGGR";
      else if ((getColorType() == PIX_PLANES_RGGB) and (strncmp(p1, "RGGB", 4) != 0))
        p2 = "RGGB";
      else if ((getColorType() == PIX_PLANES_GBRG) and (strncmp(p1, "GBRG", 4) != 0))
        p2 = "GBRG";
      else if ((getColorType() == PIX_PLANES_GRBG) and (strncmp(p1, "GRBG", 4) != 0))
        p2 = "GRBG";
      else if ((getColorType() == PIX_PLANES_BW16S) and (strncmp(p1, "BW16S", 5) != 0))
        p2 = "BW16S";
      else if ((getColorType() == PIX_PLANES_BW16U) and (strncmp(p1, "BW16U", 5) != 0))
        p2 = "BW16U";
      if (p2 != NULL)
      {
        int n = strlen(filename) + strlen(p2) + 1;
        char * p3;
        int m = p1 - filename; // length of path part of filename
        fnBuff = (char *)malloc(n);
        strncpy(fnBuff, filename, n);
        p3 = fnBuff + m; // points to start of new filename
        // add format of BW image as first part of filename
        snprintf(p3, n - m, "%s%s", p2, p1);
        // modify the name
        setName(p3);
        fn = fnBuff;
      }
    }
    // set flags and
    // copy image data to file buffer
    // can save in 24-bit format only
    gbm.bpp = 24; // tmp->getDepth() * tmp->getChannels(); // bit per pixel
    // if BW or Bayer, then the image will look rather slim, but can be recovered OK
    gbm.w = (tmp->width() * tmp->getDepth() * tmp->getChannels()) / gbm.bpp;
    gbm.h = tmp->height();
    // line width in bytes
    lineW = tmp->width() * tmp->getChannels() * tmp->getDepth() / 8;
    // copy data
    const char * pl = (const char*)tmp->getData();
    // width of BMP-image line buffer - must be along a 4-byte border
    int ww = gbm.w * (gbm.bpp / 8);
    // add the extra bytes (0..3)
    if ((ww % 4) != 0)
      ww += 4 - (ww % 4);
    for (i = 0; i < int(tmp->height()); i++)
    { // in reverse line order
      int idxd = i * ww; // gbm.w * (gbm.bpp / 8); // destination index (bytes)
      int rows = tmp->height() - i - 1;   // source row
      int idxs = rows * lineW;            // source buffer index (bytes)
      l = tmp->height() - i - 1;
      pix = tmp->getLine(l);
      data = &dataBuf[idxd];
      if (idxd > dataBufSize - gbm.w * 3)
        printf("Buffer overflow! row %d of a %dx%d image, %d > %d\n", i, gbm.h, tmp->width(), idxd, dataBufSize - gbm.w * 3);
      if (isBW() or isBGR() or isBayer())
      { // is packed as a RGB image - as driver do not seem to accept 8-bit images
        memcpy(data, &pl[idxs], lineW);
      }
      else if (colFormat == PIX_PLANES_YUV420 or colFormat == PIX_PLANES_YUV422)
      { // 12 bit (average) format - convert to BGR
        for (j = 0; j < int(width()); j++)
        { // color order is BGR
          rgb = tmp->getPix(l, j, PIX_PLANES_BGR);
          *(UPixel *)data = rgb;
          // advance pointers
          data += 3;
        }
      }
      else
      { // all 24bit formats
        for (j = 0; j < int(tmp->width()); j++)
        { // color order is BGR in file
          rgb = pix->asBGR(colFormat);
          //rgb = pix->asRGB(colFormat);
          *(UPixel *)data = rgb;
          // advance pointers
          data += 3;
          pix++;
        }
      }
    }
  }
  //
  //fd = def_create(FileName, O_WRONLY|O_BINARY); pix.r
  if (result)
  {
    fd = open(fn, O_CREAT|O_TRUNC|O_WRONLY,
                   S_IREAD|S_IWRITE|S_IROTH|S_IWOTH);
    result = (fd >= 0);
  }
  // write file
  if (result)
    result = (bmp_w(fn, fd, &gbm, NULL, dataBuf, NULL) == 0);
  // close
  if (fd >= 0)
    close(fd);
  if (dataBuf != NULL)
    free(dataBuf);
  if (tmp != this)
    delete tmp;
  if (fnBuff != NULL)
    free(fnBuff);
  //
  return result;
}


//////////////////////////////////////////////////////////////

bool UImage::loadBMP(const char filename[])
{
  // @todo this (URawImage::load()) test this function
  bool result = true;
  int fd, i, k, l, stride;
  unsigned int bytes = 0;
  GBMFT gbmft; // typename - not used ??
  GBM gbm;     // w,h,bpp (width, height bit per pixel + buffer)
  GBMRGB  gbmrgb[0x100]; // RGB struc buffer (size 256)
  unsigned char * dataBuf = NULL;
  int dataBufSize;
  UPixel rgb;
  UPixel * pix;
  const char proc[] = "UImage::loadBMP: ";
  const char * colorTyp = NULL;
  //
  fd = open(filename, O_RDONLY); // | O_BINARY);
  result = (fd >= 0);
  if (result)
    result = (bmp_rhdr(filename, fd, &gbm, "") == 0);
  if (result)
    bmp_qft(&gbmft);
  if (result)
  {
    switch ( gbm.bpp )
    {
      case 32:
//          flag = GBM_FT_W32;
          setColorType("RGBA");
          break;
      case 24:
//          flag = GBM_FT_W24;
          setColorType("BGR");
          break;
      case 8:
//          flag = GBM_FT_W8;
          setColorType("BW");
          break;
//       case 4:
//         flag = GBM_FT_W4;
//         break;
//       case 1:
//         flag = GBM_FT_W1;
//         break;
      default:
        result = false;
        break;
    }
  }
  if (result)
    result = (bmp_rpal(fd, &gbm, gbmrgb) == 0);
  if (result)
  { // set size (and reposition pointers) for this image
    setSize(gbm.h, gbm.w, gbm.bpp/8, 8, colorTyp);
    // get needed temp buffer size
    stride = ( ((gbm.w * gbm.bpp + 31)/32) * 4 );
    bytes = stride * gbm.h;
    result = (bytes <= maxBytes());
  }
  // allocate buffer space
  if (result)
  {// allocate temp buffer
    dataBufSize = bytes + 3;
    dataBuf = (unsigned char *) malloc(dataBufSize);
    result = (dataBuf != NULL);
  }
  if (result)
    // read data to buffer
    result = (bmp_rdata(fd, &gbm, dataBuf) == 0);
  // close file;
  if (fd >= 0)
    close(fd);
  if (result)
    result = ((gbm.w * gbm.h * (gbm.bpp / 8)) <= int(maxBytes()));
  if (result)
  { // set image name to "bmp" why?
    // strncpy(name, "bmp", MaxImageNameLength);
    // copy buffer data to image
    int sw = gbm.w * (gbm.bpp / 8);
    // source buffer is extended in 4-byte units
    if ((sw % 4) != 0)
      sw += 4 - (sw % 4);
    for (i = 0; i < int(height()); i++)
    {
      l = height() - i - 1;
      pix = getLine(l);
      k = i * sw;
      memcpy((char *)pix, &dataBuf[k], width() * getChannels());
    }
    valid = true;
  }
  if (result)
  { // use first part of filename as format - if bayer or BW16
    const char * p1;
    int w = gbm.w;
    int h = gbm.h;
    p1 = strrchr(filename, '/');
    if (p1 == NULL)
      p1 = filename;
    else
      p1++;
    if (strncmp(p1, "BGGR", 4) == 0)
      result = setSize(h, w * 3, 1, 8, "BGGR");
    else if (strncmp(p1, "RGGB", 4) == 0)
      result = setSize(h, w * 3, 1, 8, "RGGB");
    else if (strncmp(p1, "GBRG", 4) == 0)
      result = setSize(h, w * 3, 1, 8, "GBRG");
    else if (strncmp(p1, "GRBG", 4) == 0)
      result = setSize(h, w * 3, 1, 8, "GRBG");
    else if (strncmp(p1, "BW16S", 5) == 0)
      result = setSize(h, (w * 3) / 2, 1, 16, "BW16S");
    else if (strncmp(p1, "BW16U", 5) == 0)
      result = setSize(h, (w * 3) / 2, 1, 16, "BW16U");
  }
  // delete buffer memory
  if (dataBuf != NULL)
    delete dataBuf;
  // report error
  if (not result)
    printf("%s file read error: %s\n", proc, filename);
  return result;
}

//////////////////////////////////////////////////////////

UPixel UImage::sobel(UPixel * p1, UPixel * p2, UPixel * p3)
{
  UPixel result;
  int g1;
  int g2;
  int g3;
  int g4;
  //
  // red p3[2].p3
  // y
  g1 = (p1[0].p3 + 2 * p1[1].p3 + p1[2].p3) - (p3[0].p3 + 2 * p3[1].p3 + p3[2].p3);
  // x
  g2 = (p1[0].p3 + 2 * p2[0].p3 + p3[0].p3) - (p1[2].p3 + 2 * p2[2].p3 + p3[2].p3);
  // +45
  g3 = (p2[0].p3 + 2 * p1[0].p3 + p1[1].p3) - (p3[1].p3 + 2 * p3[2].p3 + p2[2].p3);
  // -45
  g4 = (p1[1].p3 + 2 * p1[2].p3 + p2[2].p3) - (p2[0].p3 + 2 * p3[0].p3 + p3[1].p3);
  //
  result.p3 = mini(absi(g1) + absi(g2) + absi(g3) + absi(g4), 255);
  // green
  // y
  g1 = (p1[0].p2 + 2 * p1[1].p2 + p1[2].p2) - (p3[0].p2 + 2 * p3[1].p2 + p3[2].p2);
  // x
  g2 = (p1[0].p2 + 2 * p2[0].p2 + p3[0].p2) - (p1[2].p2 + 2 * p2[2].p2 + p3[2].p2);
  // +45
  g3 = (p2[0].p2 + 2 * p1[0].p2 + p1[1].p2) - (p3[1].p2 + 2 * p3[2].p2 + p2[2].p2);
  // -45
  g4 = (p1[1].p2 + 2 * p1[2].p2 + p2[2].p2) - (p2[0].p2 + 2 * p3[0].p2 + p3[1].p2);
  result.p2 = mini(absi(g1) + absi(g2) + absi(g3) + absi(g4), 255);
  // blue
  // y
  g1 = (p1[0].p1 + 2 * p1[1].p1 + p1[2].p1) - (p3[0].p1 + 2 * p3[1].p1 + p3[2].p1);
  // x
  g2 = (p1[0].p1 + 2 * p2[0].p1 + p3[0].p1) - (p1[2].p1 + 2 * p2[2].p1 + p3[2].p1);
  // +45
  g3 = (p2[0].p1 + 2 * p1[0].p1 + p1[1].p1) - (p3[1].p1 + 2 * p3[2].p1 + p2[2].p1);
  // -45
  g4 = (p1[1].p1 + 2 * p1[2].p1 + p2[2].p1) - (p2[0].p1 + 2 * p3[0].p1 + p3[1].p1);
  result.p1 = mini(absi(g1) + absi(g2) + absi(g3) + absi(g4), 255);
  //
  return result;
}

/////////////////////////////////////////////////////////////////////

UPixel UImage::sobel(UPixel * p1, UPixel * p2, UPixel * p3, UPixel * pixRef, int reducFactor)
{ // signed from source omage
  UPixel result;
  //               g1         g2          g3         g4
  int g1; //p1  +1 +2 +1    +1  0 -1   +2 +1  0    0 +1 +2
  int g2; //p2   0  0  0    +2  0 -2   +1  0 -1   -1  0 +1
  int g3; //p3  -1 -2 -1    +1  0 -1    0 -1 -2   -2 -1  0
  int g4;
  //
  // red p3[2].p3
  // y
  g1 = (p1[0].p3 + 2 * p1[1].p3 + p1[2].p3) - (p3[0].p3 + 2 * p3[1].p3 + p3[2].p3);
  // x
  g2 = (p1[0].p3 + 2 * p2[0].p3 + p3[0].p3) - (p1[2].p3 + 2 * p2[2].p3 + p3[2].p3);
  // +45
  g3 = (p2[0].p3 + 2 * p1[0].p3 + p1[1].p3) - (p3[1].p3 + 2 * p3[2].p3 + p2[2].p3);
  // -45
  g4 = -(p1[1].p3 + 2 * p1[2].p3 + p2[2].p3) + (p2[0].p3 + 2 * p3[0].p3 + p3[1].p3);
  //
  result.p3 = mini(maxi(0, pixRef->p3 + (g1 + g2 + g3 + g4)/reducFactor), 255);
  // green
  // y
  g1 = (p1[0].p2 + 2 * p1[1].p2 + p1[2].p2) - (p3[0].p2 + 2 * p3[1].p2 + p3[2].p2);
  // x
  g2 = (p1[0].p2 + 2 * p2[0].p2 + p3[0].p2) - (p1[2].p2 + 2 * p2[2].p2 + p3[2].p2);
  // +45
  g3 = -(p2[0].p2 + 2 * p1[0].p2 + p1[1].p2) + (p3[1].p2 + 2 * p3[2].p2 + p2[2].p2);
  // -45
  g4 = -(p1[1].p2 + 2 * p1[2].p2 + p2[2].p2) + (p2[0].p2 + 2 * p3[0].p2 + p3[1].p2);
  result.p2 = mini(maxi(0, pixRef->p2 + (g1 + g2 + g3 + g4)/reducFactor), 255);
  // blue
  // y
  g1 = (p1[0].p1 + 2 * p1[1].p1 + p1[2].p1) - (p3[0].p1 + 2 * p3[1].p1 + p3[2].p1);
  // x
  g2 = (p1[0].p1 + 2 * p2[0].p1 + p3[0].p1) - (p1[2].p1 + 2 * p2[2].p1 + p3[2].p1);
  // +45
  g3 = (p2[0].p1 + 2 * p1[0].p1 + p1[1].p1) - (p3[1].p1 + 2 * p3[2].p1 + p2[2].p1);
  // -45
  g4 = (p1[1].p1 + 2 * p1[2].p1 + p2[2].p1) - (p2[0].p1 + 2 * p3[0].p1 + p3[1].p1);
  result.p1 = mini(maxi(0, pixRef->p1 + (g1 + g2 + g3 + g4)/reducFactor), 255);
  //
  return result;
}

////////////////////////////////////////////////////

void UImage::edgeSobel(UImage * dest, bool absoluteValue, int reducFactor)
{
  UPixel * p1;
  UPixel * p2;
  UPixel * p3;
  UPixel * ps;
  unsigned int r, c;
  UPixel * d;
  // make destination the same size
  dest->copyMeta(this, true);
  for (r = 0; r < height(); r++)
  {
    d = dest->getLine(r);
    d->set(255,255,255);
    p1 = getLine(maxi(r-1, 0));
    p2 = getLine(r);
    ps = p2;
    p3 = getLine(mini(r+1, height()-1));
    // first value is as second
    if (absoluteValue)
      *d++ = sobel(p1, p2, p3);
    else
      *d++ = sobel(p1, p2, p3, ps, reducFactor);
    ps++;
    for (c = 1; c < width() - 1; c++)
    {
      if (absoluteValue)
        *d = sobel(p1, p2, p3);
      else
        *d = sobel(p1, p2, p3, ps, reducFactor);
      p1++;
      p2++;
      p3++;
      d++;
      ps++;
    }
    // last in line as the previous
    d--;
    d[1] = *d;
  }
}


//////////////////////////////////////////////////////////

int UImage::pixCmp(UPixel * p1, UPixel * p2)
{ // p1-
  int a,b;
  a = p1->p1 + p1->p2 + p1->p3;
  b = p2->p1 + p2->p2 + p2->p3;
  return a - b;
}

/////////////////////////////////////////////////////////////////

UPixel * UImage::pixSort(UPixel *** ps, int pixCnt)
{
  UPixel ** p;
//  UPixel * px;
  int i, j;
  // boble sort
  for (i = 1; i < pixCnt; i++)
  {
    for (j = i; j > 0; j--)
    {
      if (pixCmp(*ps[j], *ps[j-1]) < 0)
      { // swap
        p = ps[j];
        ps[j] = ps[j-1];
        ps[j-1] = p;
      }
      else
        break;
    }
  }
  return *ps[pixCnt >> 1];
}

/////////////////////////////////////////////////////////////////

bool UImage::median(UImage * dimg, // destination image
              unsigned int row1, unsigned int col1,  // top left of area
              unsigned int row2, unsigned int col2,  // bottol right of area
              unsigned int fW, unsigned int fH     // filter width and height
              )
{
  bool res = (dimg != NULL);
  const unsigned int MAX_MASK_H = 11;
  const unsigned int MAX_PIXS = 200;
  unsigned int r, c, i, sl;
  unsigned int mw, mwh;// mask width and half width
  unsigned int mh;     // mask height
  UPixel * pixs[MAX_MASK_H];
  UPixel * pixd;
  UPixel ** pps[MAX_PIXS * MAX_MASK_H];
  UPixel * ps[MAX_PIXS * MAX_MASK_H];
  unsigned int pixn;
  int cols; // start column
  int cole; // end column
  int col = 0;
  //
  if (res)
    res = (fH <= MAX_MASK_H) and ((fW * fH) <= MAX_PIXS);
  if (res)
  {
    for (c = 0; c < MAX_PIXS * MAX_MASK_H; c++)
      // make medan sort pointers to point af buffer elements
      pps[c] = &ps[c];
    // set mask size
    mh = fH;
    mw = fW;
    // ensure mask height is odd
    if (mh % 2 == 0)
      mh++;
    // make sure mask width is odd
    if (mw % 2 == 0)
        mw++;
    // start and end column
    mwh = mw >> 1;
    cols = col1 - mwh;
    cole = mini(width(), col2 + mwh);
    // filter all relevant rows
    for (r = row1; r < row2; r++)
    { // fill median buffer at start of line
      for (i = 0; i < mh; i++)
      { // get filter source line
        sl = maxi(0, r - 1 + i);
        if (sl >= height())
          sl = height()-1;
        pixs[i] = getLine(sl);
        col = cols;
        if (col > 0)
          // advance to first pixel to use in row
          pixs[i] = &pixs[i][col];
        for (c = 0; c < mw; c++)
        { // add pixel to sort buffer
          ps[c + i * mw] = pixs[i];
          if ((col >= 0) and (col < cole))
            // advance to next pixel - else reuse edge pixel
            pixs[i]++;
          col++;
        }
      }
      // set pointer to next element to replace
      pixn = 0;
      // get pointer to destination element
      pixd = &dimg->getLine(r)[col1];
      // do all needed columns
      for (c = col1; c < col2; c++)
      { // save result in destination image
        // of median pixel
        *pixd++ = *pixSort(pps, mw * mh);
        // fill next pixels into median buffer
        for (i = 0; i < mh; i++)
        {
          ps[pixn + i * mw] = pixs[i];
          if ((col >= 0) and (col < cole))
            // advance to next pixel - else reuse edge pixel
            pixs[i]++;
        }
        col++;
        // advance pointer to oldest pixel
        pixn++;
        if (pixn >= mw)
          pixn = 0;
      }
    }
  }
  return res;
}

//////////////////////////////////////////////////////////

void UImage::hLine(unsigned int line,
             unsigned int c1, unsigned int c2,
             int p1, int p2, int p3)
{
  unsigned int c;
  UPixel * pix;
  //
  if ((line >= 0) and (line < height()))
  {
    c1 = maxi(mini(c1, width()), 0);
    c2 = maxi(mini(c2, width()), 0);
    p1 = mini(p1, 255);
    p2 = mini(p2, 255);
    p3 = mini(p3, 255);
    if (c1 < c2)
    {
      pix = &getLine(line)[c1];
      for (c = c1; c < c2; c++)
      {
        if (p1 >= 0)
          pix->p1 = p1;
        if (p2 >= 0)
          pix->p2 = p2;
        if (p3 >= 0)
          pix->p3 = p3;
        pix++;
      }
    }
  }
}

///////////////////////////////////////////////////

void UImage::vLine(unsigned int col,
             unsigned int r1, unsigned int r2,
             int p1, int p2, int p3)
{
  unsigned int r;
  UPixel * pix;
  //
  if ((col >= 0) and (col < width()))
  {
    r1 = maxi(mini(r1, height()), 0);
    r2 = maxi(mini(r2, height()), 0);
    p1 = mini(p1, 255);
    p2 = mini(p2, 255);
    p3 = mini(p3, 255);
    if (r1 < r2)
    {
      pix = &getLine(r1)[col];
      for (r = r1; r < r2; r++)
      {
        if (p1 >= 0)
          pix->p1 = p1;
        if (p2 >= 0)
          pix->p2 = p2;
        if (p3 >= 0)
          pix->p3 = p3;
        pix += width();
      }
    }
  }
}

///////////////////////////////////////////////////


bool UImage::loadPNG(const char * filename)
{
  bool result = false;
#ifndef IMAGE_NO_PNG
  const int MHL = 8;
  png_byte head[MHL];
  png_bytep header = head;
  FILE * fp;
  png_structp png_ptr = NULL;
  png_infop info_ptr = NULL;
  png_infop end_info = NULL;
  png_bytep * row_pointers;
  png_uint_32 w, h;
  int bit_depth, color_type, channels;
  png_bytep row;
  int i;
  //
  fp = fopen(filename, "rb");
  result = (fp != NULL);
  if (result)
  {
    i = fread(header, 1, MHL, fp);
    result = png_sig_cmp(header, 0, MHL) == 0;
  }
  if (result)
  {
    png_ptr = png_create_read_struct
        (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
         //(png_voidp)user_error_ptr,
         //user_error_fn, user_warning_fn);
    result = (png_ptr != NULL);
  }
  if (result)
  {
    info_ptr = png_create_info_struct(png_ptr);
    result = (info_ptr != NULL);
    if (not result)
      png_destroy_read_struct(&png_ptr,
                               (png_infopp)NULL, (png_infopp)NULL);
  }
  if (result)
  {
    end_info = png_create_info_struct(png_ptr);
    result = (end_info != NULL);
    if (not result)
      png_destroy_read_struct(&png_ptr, &info_ptr,
                               (png_infopp)NULL);
  }
  if (result)
  { // initialize and tell that 8 bytes are missing
    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, MHL);
    // limit the allowed image size
    png_set_user_limits(png_ptr, 2000, 2000);
    // read the file
    png_read_png(png_ptr, info_ptr,
                 //PNG_TRANSFORM_IDENTITY |     // No transformation
                 //PNG_TRANSFORM_STRIP_16 |  //   Strip 16-bit samples to 8 bits
                 PNG_TRANSFORM_STRIP_ALPHA | // Discard the alpha channel
                 PNG_TRANSFORM_PACKING   //   Expand 1, 2 and 4-bit  samples to bytes
                 //PNG_TRANSFORM_PACKSWAP |  //   Change order of packed  pixels to LSB first
                 //PNG_TRANSFORM_EXPAND |    //   Perform set_expand()
                 //PNG_TRANSFORM_INVERT_MONO | //  Invert monochrome images
                 //PNG_TRANSFORM_SHIFT |     //   Normalize pixels to the sBIT depth
                 //PNG_TRANSFORM_BGR | //         Flip RGB to BGR, RGBA to BGRA
                 //PNG_TRANSFORM_SWAP_ALPHA | //  Flip RGBA to ARGB or GA to AG
                 //PNG_TRANSFORM_INVERT_ALPHA | // Change alpha from opacity to transparency
                 //PNG_TRANSFORM_SWAP_ENDIAN //  Byte-swap 16-bit samples
                     , NULL);
    row_pointers = png_get_rows(png_ptr, info_ptr);
    png_get_IHDR(png_ptr, info_ptr, &w, &h,
                 &bit_depth, &color_type, NULL,
                 NULL, NULL);
    channels = png_get_channels(png_ptr, info_ptr);
    result = channels == 1 or ((color_type == PNG_COLOR_TYPE_RGB) and (channels >= 3));
    fclose(fp);
  }
  if (result)
  {
    // debug
    //printf("size is %lux%lu %d channels and color_type = %d\n",
    //      w, h, channels, color_type);
    // debug end
    if (color_type == PNG_COLOR_TYPE_GRAY)
    { // use first part of filename as format - if bayer
      const char * p1;
      p1 = strrchr(filename, '/');
      if (p1 == NULL)
        p1 = filename;
      else
        p1++;
      if (strncmp(p1, "BGGR", 4) == 0)
        result = setSize(h, w, channels, bit_depth, "BGGR");
      else if (strncmp(p1, "RGGB", 4) == 0)
        result = setSize(h, w, channels, bit_depth, "RGGB");
      else if (strncmp(p1, "GBRG", 4) == 0)
        result = setSize(h, w, channels, bit_depth, "GBRG");
      else if (strncmp(p1, "GRBG", 4) == 0)
        result = setSize(h, w, channels, bit_depth, "GRBG");
      else if (strncmp(p1, "BW16S", 5) == 0)
        result = setSize(h, w, channels, bit_depth, "BW16S");
      else if (strncmp(p1, "BW16U", 5) == 0)
        result = setSize(h, w, channels, bit_depth, "BW16U");
      else
        result = setSize(h, w, channels, bit_depth, "BW");
      if (not result)
        printf("Image too big (%lux%lu) or wrong number of "
               "channels (%d) for image buffer (%d bytes)\n",
               h, w, channels, maxBytes());
    }
    else if (color_type == PNG_COLOR_TYPE_RGB)
    {
      result = setSize(h, w, channels, 8, "RGB");
      if (not result)
        printf("Image too big (%lux%lu) or wrong number of "
               "channels (%d) for image buffer (%d bytes)\n",
               h, w, channels, maxBytes());
    }
    else
    { // unknown color type
      printf("Unknown color type (not RGB-8 or grayscale) in png file %s\n", filename);
      result = false;
    }
  }
  if (result)
  { // move unpacked image
    for (i = 0; i < int(h); i++)
    {
      row = row_pointers[i];
      //unsigned char * ps = row_pointers[i];
      unsigned char * pd = (unsigned char *) getLine(i);
      if (bit_depth != 16)
        // assume the bytes are in the right order
        memmove(pd, row, w * channels * bit_depth / 8);
      else if (bit_depth == 16)
      { // at 16 bit depth are byte order - so far - wrong and must be swapped
        for (int j = 0; j < int(width()); j++)
        { // change byte order to destination image
          *pd++ = row[1];
          *pd++ = row[0];
          row += 2;
        }
      }
    }
    //saveBMP("test.bmp");
  }
  if (png_ptr != NULL)
    png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
  //
#endif
  return result;
}

//////////////////////////////////////////////////////////////////

bool UImage::savePNG(const char * filename)
{
  bool result = false;
#ifndef IMAGE_NO_PNG
  FILE * fp;
  png_structp png_ptr = NULL;
  png_infop info_ptr = NULL;
  png_bytep * row_pointers = NULL;
  int i;
  UImage * img = this;
  //
  fp = fopen(filename, "wb");
  result = (fp != NULL);
  if (result)
  {
    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL,
        NULL, NULL);
    result = (png_ptr != NULL);
  }
  if (result)
  {
    info_ptr = png_create_info_struct(png_ptr);
    result = (info_ptr != NULL);
    if (not result)
      png_destroy_write_struct(&png_ptr,
                                (png_infopp)NULL);
  }
  if (result)
  {
    int colType;
    if (img->isBW16s())
    {
      colType = PNG_COLOR_TYPE_GRAY;
      // need to convert byte order in image buffer
      if (convertBuffer == NULL)
        convertBuffer = new UImage800();
      img = convertBuffer;
      img->copyMeta(this, true);
      img->setSize(height(), width(), 1, 16, "BW16s");
      unsigned char * ps, *pd;
      ps = getUCharRef(0,0);
      pd = img->getUCharRef(0,0);
      for (int i = 0; i < int(width()*height()); i++)
      { // change byte order to destination image
        *pd++ = ps[1];
        *pd++ = ps[0];
        ps += 2;
      }
      // printf("Changed byte order on BW16s %dx%d image\n", img->height(), img->width());
    }
    else if (img->isBW())
    {
      colType = PNG_COLOR_TYPE_GRAY;
    }
    else if (colFormat != PIX_PLANES_RGB)
    {
      if (not img->isRGB())
      { // a color conversion is needed
        if (convertBuffer == NULL)
        { // allocate a buffer for the new image format
          convertBuffer = new UImage800();
          // debug
          // printf("UImage::savePNG: Created image buffer for PNG vonversion of %s\n", filename);
          // debug end
        }
        img = convertBuffer;
        img->setSize(height(), width(), 3, 8, "RGB");
        toRGB(img);
      }
      colType = PNG_COLOR_TYPE_RGB;
    }
    else
      colType = PNG_COLOR_TYPE_RGB;
    result = colType >= 0;
    if (result)
    { // initialize file header information
      png_set_IHDR(png_ptr, info_ptr, img->width(), img->height(),
                  img->getDepth(), colType,
                  PNG_INTERLACE_NONE,
                  PNG_COMPRESSION_TYPE_DEFAULT,
                  PNG_FILTER_TYPE_DEFAULT);
      png_init_io(png_ptr, fp);
      //png_set_tIME(png_ptr, info_ptr, mod_time);
      // make row pointers structure of the right size
      row_pointers = (png_bytep *)malloc(sizeof(void *) * img->height());
      result = row_pointers != NULL;
    }
  }
  if (result)
  {
    for (i = 0; i < int(img->height()); i++)
      row_pointers[i] = (png_byte *) img->getLine(i);
    png_set_rows(png_ptr, info_ptr, row_pointers);
    png_write_png(png_ptr, info_ptr,
                  PNG_TRANSFORM_IDENTITY,
                  NULL);
    png_destroy_info_struct(png_ptr, &info_ptr);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    //png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    free(row_pointers);
    fclose(fp);
  }
  //
// IMAGE_NO_PNG
#endif
  return result;
}

///////////////////////////////////////////////////

bool UImage::saveTxt(const char * filename)
{
  FILE * txt;
  txt = fopen(filename, "w");
  uchar * ps;
  if (txt != NULL)
  {
    fprintf(txt, "%s\n", filename);
    fprintf(txt,"width=%d height=%d depth=%d\n", width(), height(), getDepth());
    for (int r = 0; r < (int)height(); r++)
    {
      ps = getUCharRef(r,0);
      for (int c = 0; c < (int)width()*getDepth(); c++)
      {
        fprintf(txt, " %3i", *ps++);
      }
      fprintf(txt, "\n");
    }
    fclose(txt);
  }
  return txt != NULL;
}

////////////////////////////////////////////////////

int UImage::packZLIB(char * buffer, int bufferSize)
{

  int byteCnt = -1;
  byteCnt = packZlib((char*)getData(), imgBytes(), buffer, bufferSize);
  return byteCnt;
}

/////////////////////////////////////////////////////////////


bool UImage::unpackZLIB(int rows, int cols, const char * colFormat, char * buffer, int bufferCnt)
{
  bool result = false;
  unsigned int have;
  //
  if (bufferCnt > 0)
  {
    if (strcasecmp(colFormat, "BW") == 0)
      result = setSize(rows, cols, 1, 8, colFormat);
    else
      result = setSize(rows, cols, 3, 8, colFormat);
    if (not result)
      printf("UImage::unpackZLIB: No space for this (%dx%d) image in %d bytes\n",
             rows, cols, maxBytes());
  }
  if (result)
  { // unpack
    have = unpackZlib(buffer, bufferCnt, (char*)getData(), maxBytes());
    // test number of bytes
    if (have != imgBytes())
      printf("UImage::unpackZLIB: Image size (%dx%dx%d=%d) do not fit data (%d bytes)\n",
             height(), width(), getChannels(), imgBytes(), have);
  }
  //
  return result;
}

////////////////////////////////////////////////////

bool UImage::moveYUV420To(int toColFormat, UImage * dest)
{
  bool result = true;
  unsigned int i; // row
  unsigned int j; // col
  unsigned char y;
  unsigned char u;
  unsigned char v;
  //long intensity=0;
  UPixel * pix;
  unsigned char * lpy; // u-start
  unsigned char * lpu = NULL; // u-start
  unsigned char * lpv = NULL; // v-start
  unsigned char * rpy; // Y-row
  unsigned char * rpu = NULL; // U-row
  unsigned char * rpv = NULL; // V-row
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    dst->copyMeta(this, true);
  }
  switch (toColFormat)
  {
    case PIX_PLANES_BW:
      // just maintain the intensity part
      result = dst->setSize(height(), width(), 1, 8, "BW");
      break;
    case PIX_PLANES_RGB:
    case PIX_PLANES_BGR:
    case PIX_PLANES_YUV:
      result = true;
      break;
    default:
      result = false;
      break;
  }
  if (result and colFormat != PIX_PLANES_BW)
  { // copy image to temp buffer
    if (dst != this)
      lpy = (unsigned char *)getData();
    else
    { // make temporary buffer
      lpy = (unsigned char*)malloc(getDataSize());
      // copy data
      memmove(lpy, getData(), getDataSize());
    }
    // set pointers to U and V data
    lpu = &lpy[height() * width()];
    lpv = &lpu[(height() * width()) / 4];
    //
    for (i = 0; i < height(); i++)
    { // for all rows
      pix = dst->getLine(i);
      rpy = &lpy[i * width()];
      // move to next row for U and V index
      rpu = &lpu[(i >> 1) * (width() >> 1)];
      rpv = &lpv[(i >> 1) * (width() >> 1)];
      // move row pixels
      for (j = 0; j < width(); j++)
      { // for all columns in this row
        // get pixel values for pixel (i,j)
        y = *rpy++;
        u = *rpu;
        v = *rpv;
        // convert to desired format and store in image
        pix->setYUVto(y, u, v, toColFormat);
        // advance to next pixel
        pix++;
        // advance to next u and v intensity
        if ((j % 2) == 1)
        { // increase pointer every other time
          rpu++;
          rpv++;
        }
      }
    }
    // Set new size and color format
    dst->setSize(height(), width(), 3, 8);
    dst->setColorType(toColFormat);
    if (dst == this)
      free(lpy);
  }
  return result;
}

/////////////////////////////////////////////////////////////////

bool UImage::yuv422toBW(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  // to      111222333444   3 bytes per pixel
  //      1  YUVYUVYUVYUV
  //      2  YUVYUVYUVYUV
  //      3  YUVYUVYUVYUV

  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  uint8_t * pix; // destination
  uint8_t *rpy; // source
  //
  if (result)
    result = setSize(h, w, 1, 8, "BW");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_BW);
    for (int i = 0; i < h; i++)
    { // for all rows
      pix = (uint8_t*) getLine(i);
      rpy = &source[i * w * 2];
      for (int j = 0; j < w; j++)
      { // for all columns in this row
        *pix++ = *rpy;
        // advance to next source
        rpy += 2; // rpy advanced to next Y source
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////

bool UImage::yuv422toYUV(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  // to      111222333444   3 bytes per pixel
  //      1  YUVYUVYUVYUV
  //      2  YUVYUVYUVYUV
  //      3  YUVYUVYUVYUV

  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy, *rpu, *rpv;
  //
  if (result)
    result = setSize(h, w, 3, 8, "YUV");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_YUV);
    for (int i = 0; i < h; i++)
    { // for all rows
      pix = getLine(i);
      rpy = &source[i * w * 2];
      // move to next row for U and V index
      rpu = rpy + 1;
      rpv = rpy + 3;
      pix->set(*rpy, *rpu, *rpv);
      pix++; // first pixel a bit off color
      // do remaining pixels
      for (int j = 1; j < w; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        pix->set(*rpy, *rpu, *rpv);
        // advance to next pixel
        pix++;
        // advance to next source
        rpy += 2; // rpy advanced to next Y source
        // advance to next u and v source intensity
        if ((j % 2) == 1)
          // increase pointer every other time
          rpu += 4;
        else
          rpv += 4;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////

bool UImage::yuv422toYUVhalf(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy1, *rpy2;
  //
  if (result)
    result = setSize(h/2, w/2, 3, 8, "YUV");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_YUV);
    for (int i = 0; i < h; i+=2)
    { // for all rows
      pix = getLine(i / 2);
      rpy1 = &source[i * w * 2]; // first line
      rpy2 = &source[(i + 1) * w * 2]; // following line
      for (int j = 1; j < w/2; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        int y = (int(rpy1[0]) + int(rpy1[2]) + int(rpy2[0]) + int(rpy2[2]))/4;
        int u = (int(rpy1[1]) + int(rpy2[1])) / 2;
        int v = (int(rpy1[3]) + int(rpy2[3])) / 2;
        pix->set(y,u,v);
        // advance to next pixel
        pix++;
        // advance to next source pixel
        rpy1 += 4; // rpy advanced to next Y source
        rpy2 += 4; // rpy advanced to next Y source
      }
    }
  }
  return result;
}

/////////////////////////////////////////////

bool UImage::yuv422toRGBhalf(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy1, *rpy2;
  //
  if (result)
    result = setSize(h/2, w/2, 3, 8, "RGB");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_RGB);
    for (int i = 0; i < h; i += 2)
    { // for all rows
      pix = getLine(i / 2);
      rpy1 = &source[i * w * 2]; // source line i
      rpy2 = &source[(i + 1) * w * 2]; // following line (i+1)*2
      for (int j = 0; j < w / 2; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        int y = (int(rpy1[0]) + int(rpy1[2]) + int(rpy2[0]) + int(rpy2[2])) / 4;
        int u = (int(rpy1[1]) + int(rpy2[1])) / 2;
        int v = (int(rpy1[3]) + int(rpy2[3])) / 2;
        *pix = pix->YUVtoRGB(y, u, v);
        // advance to next pixel
        pix++;
        // advance to next source pixel
        rpy1 += 4; // rpy advanced to next Y source
        rpy2 += 4; // rpy advanced to next Y source
      }
    }
  }
  return result;
}

/////////////////////////////////////////////

bool UImage::yuv422toBGRhalf(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy1, *rpy2;
  //
  if (result)
    result = setSize(h/2, w/2, 3, 8, "RGB");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_RGB);
    for (int i = 0; i < h; i += 2)
    { // for all rows
      pix = getLine(i / 2);
      rpy1 = &source[i * w * 2]; // source line i
      rpy2 = &source[(i + 1) * w * 2]; // following line (i+1)*2
      for (int j = 0; j < w / 2; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        int y = (int(rpy1[0]) + int(rpy1[2]) + int(rpy2[0]) + int(rpy2[2])) / 4;
        int u = (int(rpy1[1]) + int(rpy2[1])) / 2;
        int v = (int(rpy1[3]) + int(rpy2[3])) / 2;
        *pix = pix->YUVtoBGR(y, u, v);
        // advance to next pixel
        pix++;
        // advance to next source pixel
        rpy1 += 4; // rpy advanced to next Y source
        rpy2 += 4; // rpy advanced to next Y source
      }
    }
  }
  return result;
}

//////////////////////////////////////////////////

bool UImage::yuv422toRGB(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  // to      111222333444   3 bytes per pixel
  //      1  RGBRGBRGBRGB
  //      2  RGBRGBRGBRGB
  //      3  RGBRGBRGBRGB

  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy, *rpu, *rpv;
  //
  if (result)
    result = setSize(h, w, 3, 8, "RGB");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_RGB);
    for (int i = 0; i < h; i++)
    { // for all rows
      pix = getLine(i);
      rpy = &source[i * w * 2];
      // move to next row for U and V index
      rpu = rpy + 1;
      rpv = rpy + 3;
      *pix = pix->YUVtoRGB(*rpy, *rpu, *rpv);
      pix++; // first pixel a bit off color
      // do remaining pixels
      for (int j = 1; j < w; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        *pix = pix->YUVtoRGB(*rpy, *rpu, *rpv);
        // advance to next pixel
        pix++;
        // advance to next source
        rpy += 2; // rpy advanced to next Y source
        // advance to next u and v source intensity
        if ((j % 2) == 1)
          // increase pointer every other time
          rpu += 4;
        else
          rpv += 4;
      }
    }
  }
  return result;
}

bool UImage::yuv422toBGR(uint8_t * source, int h, int w)
{
  // Y for every pixel, U and V for every other pixel
  // from    112233445566   2 bytes per pixel
  //         01234567890
  //      1  YUYVYUYVYUYV
  //      2  YUYVYUYVYUYV
  //      3  YUYVYUYVYUYV
  // to      111222333444   3 bytes per pixel
  //      1  RGBRGBRGBRGB
  //      2  RGBRGBRGBRGB
  //      3  RGBRGBRGBRGB

  bool result = (h >= 0) and (w >= 0) and (source != NULL);
  UPixel * pix;
  uint8_t *rpy, *rpu, *rpv;
  //
  if (result)
    result = setSize(h, w, 3, 8, "RGB");
  if (result)
  { // OK to continue
    setColorType(PIX_PLANES_RGB);
    for (int i = 0; i < h; i++)
    { // for all rows
      pix = getLine(i);
      rpy = &source[i * w * 2];
      // move to next row for U and V index
      rpu = rpy + 1;
      rpv = rpy + 3;
      *pix = pix->YUVtoRGB(*rpy, *rpu, *rpv);
      pix++; // first pixel a bit off color
      // do remaining pixels
      for (int j = 1; j < w; j++)
      { // for all columns in this row
        // set pixel where we have an Y value and either an U or V,
        // using neighbor value for the other - left or right
        *pix = pix->YUVtoBGR(*rpy, *rpu, *rpv);
        // advance to next pixel
        pix++;
        // advance to next source
        rpy += 2; // rpy advanced to next Y source
        // advance to next u and v source intensity
        if ((j % 2) == 1)
          // increase pointer every other time
          rpu += 4;
        else
          rpv += 4;
      }
    }
  }
  return result;
}

/////////////////////////////////////////////////////////////////

bool UImage::moveBGGRTo(int toColFormat, UImage * dest)
{
  bool result;
  if (toColFormat == PIX_PLANES_BW)
    result = moveBGGRtoBW(dest);
  else if (toColFormat == PIX_PLANES_RGB)
    result = moveBGGRtoRGB(dest);
  else if (toColFormat == PIX_PLANES_BGR)
  {
    result = moveBGGRtoBGR(dest);
//     if (result)
//       dest->toBGR(dest);
  }
  else if (toColFormat == PIX_PLANES_YUV)
  {
    result = moveBGGRtoRGB(dest);
    if (result)
      dest->toYUV(dest);
  }
  else
  {
    printf("This conversion of a BGGR (Bayer) to '#%d' color format is not implemented\n", toColFormat);
    result = false;
  }
  return result;
}

 /////////////////////////////////////////////////////////////////

bool UImage::moveBGGRtoBW(UImage * dest)
{
  bool result = false;
  int i; // row
  int j; // col
  int n, w;
  unsigned char *lp3, *lp2, *lp1, *lp0; // red  -- 1 is lower (this quad) 1 is higher (1 quad higher)
  unsigned char *p3use, *p3save;
  unsigned char *px1, *px0;
  unsigned int g0, g1, g2, g3, g4, g5;
  const int MRW = UIMAGE_MAX_WIDTH;
  unsigned char row3a[MRW];
  unsigned char row3b[MRW];
  //char * pLine;
  UImage * dst = dest;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
  {
    dst->copyMeta(this, false);
    dst->setSize(height(), width(), 1, 8, "BW");
  }
  // set pointer to start of last pixel 2x2 info (in RGGB image)
  // line width in chars
  w = getWidth();     // 1 line in RGGB image
  // get pointer to last pixel
  lp2 = (unsigned char*)img.imageData + (w * getHeight() - 1);
  // get pointer to last pixel in last but 1, 2 and 3 row
  lp1 = lp2 - w;
  lp0 = lp1 - w;
  lp3 = lp1; // first time reuse line with g at same position
  // byte count to last pixel
  n = w * getHeight() - 1;
  // get pointer to last pixel in destination image
  px1 = (unsigned char*)dst->getData() + n;
  // and last pixel of line just before
  px0 = px1 - w;
  // copy row 3 to buffer - to avoid overwriting usefull data
  // from start of row lp1
  memcpy(row3a, lp0 + 1, w);
  memcpy(row3b, row3a, w);
  p3use = row3a;
  p3save = row3b;
  //
  for (i =(int)height()/2; i > 0; i--)
  { // for all rows
    //pLine = (char*)getLine((int)height() - (i*2 - 1));
    lp3 = &p3use[w-1];
    g4 = *lp1;
    for (j = w/2; j > 0; j--)
    { // for all columns in this quad-row
      // red uses uses also red values outside (over and left)
      // of current quad set   //lp0:      g2
      lp2--;                  //lp1:  g3  B  G0    |
      g1 = *lp2--;             //lp2:      G1 R  g4 |
      g0 = *lp1--;             //lp3:         g5
      g3 = *lp1--;             //         |-----|
      lp0--;
      g2 = *lp0--;
      g5 = *lp3--;
      lp3--;
      // write result to current quad
      // debug
      if (false and j == 2)
      {
        printf( "Near j=%3d  GRGRG %3d %3d %3d %3d %3d\n"
            " quad -> |  BGBGB %3d %3d %3d %3d %3d  |\n"
            "apriori  |  GRGRG %3d %3d %3d %3d %3d  |\n"
            "            BGBGB %3d %3d %3d %3d %3d\n"
            "                          |------|\n",
            j, lp0[-1], lp0[0], lp0[1], lp0[2], lp0[3],
            lp1[-1], lp1[0], lp1[1], lp1[2], lp1[3],
            lp2[-1], lp2[0], lp2[1], lp2[2], lp2[3],
            lp3[-1], lp3[0], lp3[1], lp3[2], lp3[3]);
      }
      // debug end
      // upper row in quad
      *px0-- = g0;
      if ((absi(g3 - g0) > absi(g2-g1)) or (j == 0))
        *px0-- = g2;
      else
        *px0-- = g3;
      // lower row in quad
      if (absi(g1 - g4) > absi(g0-g5))
        *px1-- = g0;
      else
        *px1-- = g1;
      *px1-- = g1;
      g4 = g1;
      // debug
      if (false and j == 2)
      {
        printf( "New BW quad               %3d %3d\n"
                "                          %3d %3d\n---\n", px0[1], px0[2], px1[1], px1[2]);
        printf( "buf r  save BGBG %3d %3d %3d %3d %3d == row3a %s\n"
                "        use BGBG %3d %3d %3d %3d %3d == row3a %s\n",
                p3save[0], p3save[1], p3save[2], p3save[3], p3save[4], bool2str(row3a == p3save),
                p3use[0],  p3use[1],  p3use[2],  p3use[3],  p3use[4],  bool2str(row3a == p3use));
        printf("---end of line %d and %d-----------------------\n", i * 2 - 2, i * 2 - 1);
      }
      // debug end
    }
    //move to next 2 RGGB lines
    lp3 = p3save;
    p3save = p3use;
    p3use = lp3;
    lp2 -= w;
    lp1 -= w;
    if (i > 2)
    { // get line above current quad
      lp0 -= w;
      memcpy(p3save, lp0 + 1, w);
    }
    else
      // use same red in top row
      lp0 = lp2;
    // move to next 2 RGB rows
    px1 -= w;
    px0 -= w;
  }
  // Set new size and color format
  dst->setSize(height(), width(), 1, 8);
  dst->setColorType(PIX_PLANES_BW);
  result = true;
  //
  return result;
}

/////////////////////////////////////////////////////////////////


bool UImage::moveBGGRtoRGB(UImage * dest)
{
  UImage * dst = dest;
  UImage * src = this;
  bool result = true;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
    dst->copyMeta(src, true);
  if (dst == this)
  {
    if (convertBuffer == NULL)
      convertBuffer = new UImage();
    src = convertBuffer;
    //src->setSize(height(), width(), getChannels(), getDepth());
    src->copy(this);
  }
  dst->setSize(height(), width(), 3, 8, "RGB");
  switch (src->getColorType())
  {
    case PIX_PLANES_BGGR:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerRG2RGB);
      break;
    case PIX_PLANES_RGGB:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerBG2RGB);
      break;
    case PIX_PLANES_GBRG:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerGR2RGB);
      break;
    case PIX_PLANES_GRBG:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerGB2RGB);
      break;
    default:
      result = false;
      break;
  }
//   if (src != this)
//     delete src;
  return result;
}

/////////////////////////////////////////////////////

bool UImage::moveBGGRtoBGR(UImage * dest)
{
  UImage * dst = dest;
  UImage * src = this;
  bool result = true;
  //
  if (dst == NULL)
    dst = this;
  if (dst != this)
    dst->copyMeta(src, true);
  if (dst == this)
  {
    if (convertBuffer == NULL)
      convertBuffer = new UImage();
    src = convertBuffer;
    //src->setSize(height(), width(), getChannels(), getDepth());
    src->copy(this);
  }
  dst->setSize(height(), width(), 3, 8, "BGR");
  switch (src->getColorType())
  {
    case PIX_PLANES_BGGR:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerRG2BGR);
      break;
    case PIX_PLANES_RGGB:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerBG2BGR);
      break;
    case PIX_PLANES_GBRG:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerGR2BGR);
      break;
    case PIX_PLANES_GRBG:
      cvCvtColor(src->cvArr(), dst->cvArr(), CV_BayerGB2BGR);
      break;
    default:
      result = false;
      break;
  }
  return result;
}

// bool UImage::moveBGGRtoBGR(UImage * dest)
// {
//   bool result = false;
//   int i; // row
//   int j; // col
//   int n, m, w;
//   unsigned char *lp2, *lp1, *lp0; // red  -- 1 is lower (this quad) 1 is higher (1 quad higher)
//   unsigned char * px1, *px0;
//   unsigned int r0, r1, r2 = 0, r3 = 0, g0, g1, g2, b;
//   int oldFormat = colFormat;
//   UImage * dst = dest;
//   //
//   if (dst == NULL)
//     dst = this;
//   if (dst != this)
//     dst->copyMeta(this, false);
//   // resize image buffer (if needed)
//   dst->setSize(height(), width(), 3, 8);
//   // set pointer to start of last pixel 2x2 info (in RGGB image)
//   // line width in chars
//   w = getWidth();     // 1 line in RGGB image
//   // get pointer to last pixel
//   lp2 = (unsigned char*)img.imageData + (w * getHeight() - 1);
//   // get pointer to last pixel in last but 1 row
//   lp1 = lp2 - w;
//   // get pointer to last pixel in last but 2 row
//   lp0 = lp1 - w;
//   // byte count to last pixel
//   m = getWidth() * 3; // 1 line in clolor image
//   n = m * getHeight();
//   if (n <= (int)dest->getBufferSize())
//   { // there should always be buffer space
//     px1 = (unsigned char*)dst->getData() + n;
//     px0 = px1 - m;
//     for (i =(int)height()/2; i > 0; i--)
//     { // for all rows
//       //pLine = (char*)getLine((int)height() - (i*2 - 1));
//       for (j = w/2; j > 0; j--)
//       { // for all columns in this odd row
//         // red uses uses also red values outside (over and left)
//         //                                  BGGR           RGGB
//         // of current quad set   //lp0:  R2  G2 R0       lp0   B2 G2 B0
//         r1 = *lp2--;             //lp1:      B  G0  |    lp1      R  G0
//         g1 = *lp2--;             //lp2:  R3  G1 R1  |    lp2   B3 G1 B1
//         g0 = *lp1--;             //         |-----|
//         b  = *lp1--;
//         r0 = *lp0--;
//         g2 = *lp0--;
//         if (j > 1)
//         { // reuse last values for left-most quad
//           r2 = *lp0;
//           r3 = *lp2;
//         }
//         // write result to current quad
//         if (oldFormat == PIX_PLANES_BGGR)
//         { // upper row in quad
//           *px0-- = r0; *px0-- = g0;  *px0-- = b;
//           *px0-- = r2; *px0-- = g2;  *px0-- = b;
//           // lower row in quad
//           *px1-- = r1; *px1-- = g0;  *px1-- = b;
//           *px1-- = r3; *px1-- = g1;  *px1-- = b;
//         }
//         else
//         { // upper row in quad
//           // upper row in quad (debug g<-->r)
//           *px0-- = b; *px0-- = g0;  *px0-- = r1;
//           *px0-- = b; *px0-- = g1;  *px0-- = r1;
//           // lower row in quad
//           *px1-- = b; *px1-- = g0;  *px1-- = r1;
//           *px1-- = b; *px1-- = g1;  *px1-- = r1;
//         }
//         if (false and j == 2)
//         { // left edge
//           printf("Near j=%3d  GRGR %3d %3d %3d %3d (row=%d)\n"
//               " quad -> |  BGBG %3d %3d %3d %3d       BGR %3d %3d %3d      %3d %3d %3d\n"
//               "         v  GRGR %3d %3d %3d %3d       BGR %3d %3d %3d      %3d %3d %3d\n",
//               j, lp0[-1], lp0[0], lp0[1], lp0[2], i,
//               lp1[-1], lp1[0], lp1[1], lp1[2],    px0[1], px0[2], px0[3], px0[4], px0[5], px0[6],
//               lp2[-1], lp2[0], lp2[1], lp2[2],    px1[1], px1[2], px1[3], px1[4], px1[5], px1[6]);
//         }
//       }
//       //move to next 2 RGGB lines
//       lp2 -= w;
//       lp1 -= w;
//       if (i > 2)
//         lp0 -= w;
//       else
//         // use same red in top row
//         lp0 = lp2;
//       // move to next 2 RGB rows
//       px1 -= m;
//       px0 -= m;
//     }
//     // Set new size and color format
//     dst->setSize(height(), width(), 3, 8);
//     dst->setColorType(PIX_PLANES_BGR);
//     result = true;
//   }
//   return result;
// }

/////////////////////////////////////////////////////////////////

bool UImage::setQuadPix(unsigned int line, unsigned int column,
                           unsigned char I11, // top left
                           unsigned char I12, // bottom left
                           unsigned char I21, // top right
                           unsigned char I22, // botom right
                           unsigned char U,
                           unsigned char V)
{
  bool result;
  unsigned char * ipy;
  unsigned char * ipu;
  unsigned char * ipv;
  unsigned int col = (column >> 1);
  unsigned int row = (line >> 1);
  //
  if (colFormat != PIX_PLANES_YUV420)
    fprintf(stderr, "setQuadPix() NB! wrong color format (should be YUV420)\n");
  result = ((line < height()) and (column < width()));
  if (result)
  { // get even Y line
    ipy = &getYline(row << 1)[col << 1];
    *ipy = I11;
    ipy++;
    *ipy = I12;
    ipy = &getYline((row << 1) + 1)[col << 1];
    *ipy = I21;
    ipy++;
    *ipy = I22;
    // color part
    ipu = &getUline(row)[col];
    ipv = &getVline(row)[col];
    *ipu = U;
    *ipv = V;
  }
  return result;
}

///////////////////////////////////////////////////////

bool UImage::getQuadPix(unsigned int line, unsigned int column,
                           unsigned char * I11, // top left
                           unsigned char * I12, // bottom left
                           unsigned char * I21, // top right
                           unsigned char * I22, // botom right
                           unsigned char * U,
                           unsigned char * V)
{
  bool result;
  unsigned char * ipy;
  unsigned char * ipu;
  unsigned char * ipv;
  unsigned int col = (column >> 1);
  unsigned int row = (line >> 1);
  //
  if (colFormat != PIX_PLANES_YUV420)
    fprintf(stderr, "getQuadPix() NB! wrong color format (should be YUV420)\n");
  result = ((line < height()) and (column < width()));
  if (result)
  { // get even Y line
    ipy = &getYline(row << 1)[col << 1];
    *I11 = *ipy;
    ipy++;
    *I12 = *ipy;
    ipy = &getYline((row << 1) + 1)[col << 1];
    *I21 = *ipy;
    ipy++;
    *I22 = *ipy;
    // color part
    ipu = &getUline(row)[col];
    ipv = &getVline(row)[col];
    *U = *ipu;
    *V = *ipv;
  }
  return result;
}

///////////////////////////////////////////////////////

unsigned char * UImage::getYline(unsigned int line)
{
  unsigned char * result;
  if (line < height())
    result = &py[line * img.widthStep];
  else
    result = NULL;
  return result;
}

///////////////////////////////////////////////////////

unsigned char * UImage::getUline(unsigned int line)
{
  unsigned char * result;
  if (line <= (height() >> 1))
    result = &pu[line * (width() >> 1)];
  else
    result = NULL;
  return result;
}

///////////////////////////////////////////////////////

unsigned char * UImage::getVline(unsigned int line)
{
  unsigned char * result;
  if (line <= (height() >> 1))
    result = &pv[line * (width() >> 1)];
  else
    result = NULL;
  return result;
}

///////////////////////////////////////////////////////

UPixel UImage::getPix(unsigned int line,
                      unsigned int column)
{
  UPixel result;
  unsigned char * pp;
  switch (colFormat)
  {
    case PIX_PLANES_BW:
      result.set(getYline(line)[column], 128, 128);
      break;
    case PIX_PLANES_YUV420:
      result.set(getYline(line)[column],
                 getUline(line >> 1)[column >> 1],
                 getVline(line >> 1)[column >> 1]);
      break;
    case PIX_PLANES_YUV422:
      pp = (unsigned char *) getData();
      pp += width() * 2 * line + column * 2;
      if (column %2 == 0)
        result.set(pp[0], pp[1], pp[-1]);
      else
        result.set(pp[0], pp[-1], pp[1]);
      break;
    default:
      // use format as is
      result = *getPixRef(line, column);
      break;
  }
  return result;
}

///////////////////////////////////////////////////////

int UImage::getPixInt(unsigned int line,
                      unsigned int column)
{
  UPixel * pix;
  int pixi;
  unsigned char * pp;
  switch (colFormat)
  {
    case PIX_PLANES_YUV420:
      // use YUV format
      pixi = (getYline(line)[column] << 16) +
             (getUline(line >> 1)[column >> 1] << 8) +
              getVline(line >> 1)[column >> 1];
      break;
    case PIX_PLANES_YUV422:
      pp = (unsigned char *) getData();
      pp += width() * 2 * line + column * 2;
      if (column %2 == 0)
        pixi = (pp[0] << 16) + (pp[1] << 8 )+  pp[-1];
      else
        pixi = (pp[0] << 16) + pp[1] +  (pp[-1] << 8);
      break;
    case PIX_PLANES_BW:
      // use YUV format
      pixi = getYline(line)[column];
      break;
    default:
      // use format as is
      pix = getPixRef(line, column);
      pixi = (pix->p1 << 16) + (pix->p2 << 8) + pix->p3;
      break;
  }
  return pixi;
}

///////////////////////////////////////////////////////

UPixel UImage::getPix(unsigned int line,
                      unsigned int column,
                      int inColFormat)
{
  UPixel asIs = getPix(line, column);
  UPixel result;
  int fromFormat;
  // short formats uses YUV
  if ((colFormat == PIX_PLANES_YUV420) or
       (colFormat == PIX_PLANES_BW))
    fromFormat = PIX_PLANES_YUV;
  else if (colFormat == PIX_PLANES_YUV422)
  {
    asIs = getPix(line, column);
    fromFormat = PIX_PLANES_YUV;
  }
  else
    fromFormat = colFormat;
  //
  switch (inColFormat)
  {
    case PIX_PLANES_YUV420:
    case PIX_PLANES_BW:
      result = asIs.asYUV(fromFormat);
      break;
    case PIX_PLANES_RGB:
      result = asIs.asRGB(fromFormat);
      break;
    case PIX_PLANES_BGR:
      result = asIs.asBGR(fromFormat);
      break;
    default:
      // error - use format as is
      result = asIs;
      break;
  }
  return result;
}

////////////////////////////////////////////////////////

void UImage::imgUpdated()
{
  // and set the update time - if others are polling
  // for potential updates
  imgUpdateTime.now();
  used = 0;
}

/////////////////////////////////////////////

void UImage::moveYUV420ToHalf()
{
  unsigned char *py1, *py0;
  unsigned char *pu = NULL, *pv = NULL;
  UPixel *pd; // destination
  int sum;
  int w, h, r, c, k;
  const int M2RW = UIMAGE_MAX_WIDTH * 2;
  unsigned char row01[M2RW];
  //
  k = width(); // current width
  w = width() / 2; // new width
  h = height() / 2; // new height
  // n = w * h;
  // pointer to the Y-value in full resolution
  // first 2 lines (line 0 and 1) will be overwritten
  // by the destination data and source data must be copied
  // to buffer first.
  memcpy(row01, img.imageData, k * 2);
  py0 = row01;
  py1 = py0 + k;
  // destination
  pd = getData();
  for (r = 0; r < h; r++)
  { // do average for all rows 2 by 2
    if (r == 1)
    { // move row pointers back to image buffer at first chance
      py0 = getYline(2);
      py1 = getYline(3);
    }
    // make average for quad pixel set
    for (c = 0; c < w; c++)
    {
      sum  = *py0++ + *py1++;
      sum += *py0++ + *py1++;
      pd->p1 = sum / 4;
      pd->p2 = *pu++;
      pd->p3 = *pv++;
      pd++;
    }
    // advance one (further) line to next quad set
    py0 += w;
    py1 += w;
  }
  setSize(h, w, 3, 8, "YUV");
}

/////////////////////////////////////////////

void UImage::moveBGGRToHalfBGR()
{
  unsigned char *pb1, *pb0, *pls;
  UPixel *pd, *pld; // destination
  int w, h, r, c, k;
  const int M2RW = UIMAGE_MAX_WIDTH * 2;
  unsigned char row01[M2RW];
  //
  k = width(); // current width
  w = width() / 2; // new width
  h = height() / 2; // new height
  //n = w * h;
  // first 2 lines (line 0 and 1) og Bayer data will be overwritten
  // by the destination data and source data must be copied
  // to buffer first.
  memcpy(row01, img.imageData, k * 2);
  pb0 = row01;
  pb1 = pb0 + k;
  // destination
  pls = (unsigned char *)getData();
  pld = getData();
  pd = pld;
  for (r = 0; r < h; r++)
  { // do average for all rows 2 by 2
/*    if (r == 1)
    { // move row pointers back to image buffer at first chance
      pb0 = (unsigned char *)img.imageData + k * 2;
      pb1 = pb0 + k;
    }*/
    // make average for quad pixel set
    for (c = 0; c < w; c++)
    { // make destination BGR pixel (3 bytes)
      if (colFormat == PIX_PLANES_BGGR)
      {
        pd->p1  = *pb0++;                  // blue
        pd->p2 = (*pb0++ + *pb1++) / 2;    // green
        pd->p3 = *pb1++;                   // red
      }
      else if (colFormat == PIX_PLANES_RGGB)
      {
        pd->p3  = *pb0++;                  // red
        pd->p2 = (*pb0++ + *pb1++) / 2;    // green
        pd->p1 = *pb1++;                   // blue
      }
      else if (colFormat == PIX_PLANES_GBRG)
      {
        pd->p3  = *pb1++;                  // red
        pd->p2 = (*pb0++ + *pb1++) / 2;    // green
        pd->p1 = *pb0++;                   // blue
      }
      else if (colFormat == PIX_PLANES_GRBG)
      {
        pd->p1 = *pb1++;                   // blue
        pd->p2 = (*pb0++ + *pb1++) / 2;    // green
        pd->p3 = *pb0++;                  // red
      }
      pd++;
    }
    // advance destination - on line border
    pld += w;
    pd = pld;
    // advance source lines
    pls += 2 * k;
    pb0 = pls;
    pb1 = pls + k;
  }
  setSize(h, w, 3, 8, "BGR");
}

/////////////////////////////////////////////

void UImage::moveBWToHalf()
{
  unsigned char *px1, *px0;
  unsigned char *pd; // destination
  int sum;
  int w, h, r, c, k;
  //
  k = width(); // current width
  w = width() / 2; // new width
  h = height() / 2; // new height
  // n = w * h;
  px0 = (unsigned char*)img.imageData;
  px1 = px0 + k;
  pd = px0;
  for (r = 0; r < h; r++)
  {
    for (c = 0; c < w; c++)
    {
      sum  = *px0++ + *px1++;
      sum += *px0++ + *px1++;
      *pd++ = sum / 4;
    }
    // advance one (further) line to next quad set
    px0 += k;
    px1 += k;
  }
  setSizeOnly(h, w);
}

/////////////////////////////////////////////

void UImage::movePixToHalf()
{
  UPixel *px1, *px0;
  UPixel *pd; // destination
  int sP1, sP2, sP3; // sum of values
  int w, h, r, c, k;
  //
  k = width(); // current width
  w = width() / 2; // new width
  h = height() / 2; // new height
  //n = w * h;
  px0 =   pd = getData();
  px1 = px0 + k;
  pd = px0;
  for (r = 0; r < h; r++)
  {
    for (c = 0; c < w; c++)
    {
      sP1  = px0->p1 + px1->p1;
      sP2  = px0->p2 + px1->p2;
      sP3  = px0->p3 + px1->p3;
      px0++; px1++;
      sP1 += px0->p1 + px1->p1;
      sP2 += px0->p2 + px1->p2;
      sP3 += px0->p3 + px1->p3;
      px0++; px1++;
      pd->p1 = sP1 / 4;
      pd->p2 = sP2 / 4;
      pd->p3 = sP3 / 4;
      pd++;
    }
    // advance one (further) line to next quad set
    px0 += k;
    px1 += k;
  }
  setSizeOnly(h, w);
}

/////////////////////////////////////////////////

void UImage::paintGridAligned(const int r0, const int c0, const double pixPerM,
                              const int strongTicEvery)
{
  CvPoint p1, p2;
  const CvScalar blue = CV_RGB(100, 100 , 255);
  const CvScalar gray1 = CV_RGB(175, 175 , 175);
  const CvScalar gray2 = CV_RGB(100, 100 , 100);
  CvScalar col;
  int w, h, m, x, y;
  //
  w = getWidth();
  h = getHeight();
  if (pixPerM > 1.5)
  { // find leftmost meter line
    m = 0;
    x = c0;
    while (x >= 0)
    {
      m--;
      x = c0 + roundi(m * pixPerM);
    }
    m++;
    p1.y = 0;
    p2.y = h - 1;
    p1.x = c0 + roundi(m * pixPerM);
    while (p1.x < w)
    { // paint all vertical lines
      p1.x = c0 + roundi(m * pixPerM);
      p2.x = p1.x;
      if (p1.x == c0)
        col = blue;
      else if (m % strongTicEvery == 0)
        col = gray2;
      else
        col = gray1;
      cvLine(cvArr(), p1, p2, col, 1);
      m++;
      p1.x = c0 + roundi(m * pixPerM);
    }
    // find leftmost meter line
    m = 0;
    y = r0;
    while (y >= 0)
    {
      m--;
      y = r0 + roundi(m * pixPerM);
    }
    m++;
    // do the lines
    p1.x = 0;
    p2.x = w - 1;
    p1.y = r0 + roundi(m * pixPerM);
    while (p1.y < h)
    { // paint all horizontal lines
      if (p1.y == r0)
        col = blue;
      else if (m % strongTicEvery == 0)
        col = gray2;
      else
        col = gray1;
      p2.y = p1.y;
      cvLine(cvArr(), p1, p2, col, 1);
      m++;
      p1.y = r0 + roundi(m * pixPerM);
    }
  }
}

////////////////////////////////////////////////////////

bool UImage::toRaibow(UImage * gray)
{
  bool result = setSize(gray->height(), gray->width(), 3, 8, "RGB");
  UPixel * pixc = getData();
  unsigned char * pixg = gray->getUCharRef(0, 0);
  //
  int n = gray->height() * gray->width();
  for (int i = 0; i < n; i++)
  {
    pixc++->grayToColor(*pixg++);
  }
  return result;
}

////////////////////////////////////////////////////////

bool UImage::toRaibowInv(UImage * gray)
{
  bool result = setSize(gray->height(), gray->width(), 3, 8, "RGB");
  UPixel * pixc = getData();
  unsigned char * pixg = gray->getUCharRef(0, 0);
  //
  int n = gray->height() * gray->width();
  for (int i = 0; i < n; i++)
  {
    pixc++->grayToColorInv(*pixg++);
  }
  return result;
}
