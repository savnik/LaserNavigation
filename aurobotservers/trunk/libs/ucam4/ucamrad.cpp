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

#include <string.h>
#include <urob4/uvarpool.h>
#include "ucamrad.h"

#define resultDecimalFactor 10

///////////////////////////////////////////////////////

UCamRad::UCamRad(UCamDevBase * device)
  : UCamPanTiltDevice(device)
{ // constructor
  //valid = false;
  //mP.init(3,3,1);
  radialOffsetValid = false;
  defaultRemRadErr = false;
  defaultColFormat = PIX_PLANES_YUV;
  varIntrinsic = NULL;
  varDistortion = NULL;
  mapx = NULL;
  mapy = NULL;
}

//////////////////////////////////////////////////////

UCamRad::~UCamRad()
{ // Destructor
  if (mapx != NULL)
    cvReleaseImage(&mapx);
  if (mapy != NULL)
    cvReleaseImage(&mapy);
}

///////////////////////////////////////////////////////

bool UCamRad::getImageSnapshot(UImage * image) //, bool remRadErr)
{
  bool result = true;
  //
  if (not dev->isCameraOpen())
    result = dev->openDeviceDefault();
  if (result)
  {
    result = dev->getImageSnapshot(image); //, remRadErr);
    if (image != NULL)
    { // save for later replay
      if (result)
        // log just captured image if log is open
        logImage(image);
    }
  }
  return result;
}

/////////////////////////////////////////////

void UCamRad::imageSizeChanged(double iResFactor)
{ // always recalculate - may be just a radial error change
  // camDevice
  // pixSize = iResFactor;
  // parameter
  par.setPixelSize(iResFactor);
  radialOffsetValid = false;
  updateLensParams(0, 0);
  setRadialErrorMatrix();
}

//////////////////////////////////////////////////////////////

bool UCamRad::removeRadialError(UImage * source, UImage * destination)
{ // remove radial error and move result to destination image
  bool result = false;
  const int width = source->getWidth();
  const int height = source->getHeight();
  // copy image meta data to destination
  if (lensParChanged() or mapx == NULL or height != (int)getDev()->getHeight())
    updateLensParams(height, width);
  if (par.isValid() and mapx != NULL)
  { // take one plane at a time
    switch (source->getColorType())
    {
      case PIX_PLANES_YUV420:
        destination->copyMeta(source, true);
        result = removeRadialErrorOnePlane(source->getYline(0),
                                    destination->getYline(0),
                                    height, width, par.getPixelSize(), false);
        if (result)
          result = removeRadialErrorOnePlane(source->getUline(0),
                                      destination->getUline(0),
                                      height/2, width/2, par.getPixelSize() * 2.0, true);
        if (result)
          result = removeRadialErrorOnePlane(source->getVline(0),
                                      destination->getVline(0),
                                      height/2, width/2, par.getPixelSize() * 2.0, true);
        break;
/*      case PIX_PLANES_BW:
        result = removeRadialErrorOnePlane(source->getYline(0),
                                           destination->getYline(0),
                                           height, width, par.getPixelSize(), false);
        break;*/
      default: // all other 3-plane images
        // can not rectify a bayer coded image, nor a YUV420
#if (CV_MAJOR_VERSION >= 1)
        if (source->getColorType() == PIX_PLANES_BGGR or
            source->getColorType() == PIX_PLANES_RGGB or
            source->getColorType() == PIX_PLANES_GBRG or
            source->getColorType() == PIX_PLANES_GRBG or
            source->getColorType() == PIX_PLANES_YUV420)
          source->toBGR(source);
        destination->copyMeta(source, true);
        cvRemap(source->cvArr(), destination->cvArr(), mapx, mapy ); // undistort image
        result = true;
#else
        result = false;
#endif
        //result = removeRadialErrorPixels(source->getLine(0),
        //                              destination->getLine(0),
        //                              height, width, par.getPixelSize());
        break;
    }
  }
  // debug
  /*
  printf("UCamRad::removeRadialError: raw - pixSize %f, image height %d\n",
     par.getPixelSize(), source->getHeight());
  par.print("cam-par");
  if (source == destination)
    printf("UCamRad::removeRadialError: raw Source == destnation!\n");
  */
  // debug end
  if (result)
  {
    destination->radialErrorRemoved = true;
    destination->updated();
  }
  return result;
}

//////////////////////////////////////////////////////////////

bool UCamRad::removeRadialErrorOnePlane(unsigned char ps[],
                                       unsigned char pd[],
                                       unsigned int height,
                                       unsigned int width,
                                       float pixSize, bool halfRes)
{
  int result = true;
  UXYoffset oxy; // offset x and y
  UXYoffset * oxyh; // line with ofset values
  int decimalFactor = roundi(resultDecimalFactor * pixSize);
  int w, h, i;
  int rhx, rhy;
  double dhx, dhy;
  int ix, iy; // pixel offset in Intensity resolution
  int r1, r2, c1, c2;
  int i1, i2; // intensity (decimal part) from pix 1 and 2
  unsigned char * pss; // source pixel pointer intensity (Y)
  unsigned char * pdd; // destination pixel pointer Intensity
  unsigned char p1 = 0, p2 = 0, p3 = 0, p4 = 0, pt, pb, pr; // pixel values
  unsigned char gray = 128;
  bool outside, outsideByOne;
  //
  dhx = par.getHx();
  dhy = par.getHy();
  if (halfRes)
  { // head point in actual resolution
    dhx /= 2.0;
    dhy /= 2.0;
  }
  rhx = int(dhx);
  rhy = int(dhx);
  // correct all lines
  for (h = 0; h < int(height); h++)
  { // get destination
    pdd = &pd[h * width];
    // get line in offset table
    i = absi(int(pixSize * (double(h) - dhy)));
    // get pointer to first line of offsets
    oxyh = radialOffset[i];
    for (w = 0; w < int(width); w++)
    { // get index to offset value on this line
      i = absi(int(pixSize * (double(w) - dhx)));
      // get offset for this pixel
      oxy = oxyh[i];
      // corrext for right quadrant - matrix is for lower-right
      if (w < rhx)
        oxy.dx = -oxy.dx;
      if (h < rhy)
        oxy.dy = -oxy.dy;
      // if not exact right find 4 pixels to interpolate
      if ((oxy.dx != 0) or (oxy.dy != 0))
      { // get top left pixel offset
        ix = oxy.dx / decimalFactor; // includes pixel size
        if (oxy.dx < 0) ix--;
        iy = oxy.dy / decimalFactor;
        if (oxy.dy < 0) iy--;
        // limit to edge of image (reuse border pixels)
        r1 = h + iy;
        r2 = r1 + 1;
        c1 = w + ix;
        c2 = c1 + 1;
        outside = (r1 < 0) or (r2 >= int(height)) or
                  (c1 < 0) or (c2 >= int(width));
        if (outside)
          // may be outside by just one pixel
          outsideByOne = (((r2 == 0) or (r2 == int(height))) and
                                (c2 > 0) and (c2 < int(width))) or
                         (((c2 == 0) or (c2 == int(width))) and
                                (r2 > 0) and (r2 < int(height)));
        else
          outsideByOne = false;
        //
        if (outside and not outsideByOne)
            // both outside - use gray
            *pdd = gray;
        else
        { // not (completely) outside
          // get pixels
          i = r1 * width + c1;
          pss = &ps[i];
          p1 = pss[0];
          p2 = pss[1];
          p3 = pss[width];
          p4 = pss[width + 1];
          if (outsideByOne)
          { // just outside by one, so adjust index to inside
            if (r2 == 0)
            { // top row missing
              p1 = p3;
              p2 = p4;
            }
            if (r2 == int(height))
            { // bottom row missing
              p3 = p1;
              p4 = p2;
            }
            if (c2 == 0)
            { // left column missing
              p1 = p2;
              p3 = p4;
            }
            if (c2 == int(width))
            { // right column missing
              p2 = p1;
              p4 = p3;
            }
          }
          outside = false;
        }
        if (not outside)
        { // get index to top-left of pixels in question
          // get 4 pixels in question
          // get left and right share
          i2 = oxy.dx - ix * decimalFactor; // part of pixel in position (ix, iy)
          i1 = decimalFactor - i2;          // part of (ix+1, iy+1)
          // average for top set of pixels
          pt = (unsigned char)((int(p1) * i1  + int(p2) * i2)/decimalFactor);
          // average for bottom set of pixels
          pb = (unsigned char)((int(p3) * i1  + int(p4) * i2)/decimalFactor);
          // get shares of top and bottom
          i2 = oxy.dy - iy * decimalFactor; // part of pixel in position (ix, iy)
          i1 = decimalFactor - i2; // part of (ix, iy+1)
          // find result pixel intensity
          pr = (unsigned char)((int(pt) * i1  + int(pb) * i2)/decimalFactor);
          // implement
          *pdd = pr;
        }
      }
      else
      { // no change, so set destination to source
        pss = &ps[h * width];
        *pdd = pss[w];
      }
      pdd++;
    }
  }
  return result;
}

///////////////////////////////////////////////////////////////////////

bool UCamRad::removeRadialErrorPixels(UPixel ps[],
                                      UPixel pd[],
                                      unsigned int height,
                                      unsigned int width,
                                      float pixSize)
{ //
  int result = true;
  UXYoffset oxy; // offset x and y
  UXYoffset * oxyh; // line with ofset values
  int decimalFactor = roundi(resultDecimalFactor * pixSize);
  int w, h, i;
  int rhx, rhy;
  double dhx, dhy;
  int ix, iy; // pixel offset in Intensity resolution
  int r1, r2, c1, c2;
  int i1, i2; // intensity (decimal part) from pix 1 and 2
  UPixel * pss; // source pixel pointer intensity (Y)
  UPixel * pdd; // destination pixel pointer Intensity
  UPixel p1, p2, p3, p4, pt, pb, pr; // pixel values
  UPixel gray(128,128,128);
  bool outside, outsideByOne;
  //
  // head point in actual resolution
  dhx = par.getHx();
  dhy = par.getHy();
  rhx = int(dhx);
  rhy = int(dhx);
  // correct all lines
  for (h = 0; h < int(height); h++)
  { // get destination
    pdd = &pd[h * width];
    // get line in offset table
    i = absi(int(pixSize * (double(h)- dhy)));
    // get pointer to first line of offsets
    oxyh = radialOffset[i];
    for (w = 0; w < int(width); w++)
    { // get index to offset value on this line
      i = absi(int(pixSize * (double(w) - dhx)));
      // get offset for this pixel
      oxy = oxyh[i];
      // corrext for right quadrant - matrix is for lower-right
      if (w < rhx)
        oxy.dx = -oxy.dx;
      if (h < rhy)
        oxy.dy = -oxy.dy;
      // if not exact right find 4 pixels to interpolate
      if ((oxy.dx != 0) or (oxy.dy != 0))
      { // get top left pixel offset
        ix = oxy.dx / decimalFactor;
        if (oxy.dx < 0) ix--;
        iy = oxy.dy / decimalFactor;
        if (oxy.dy < 0) iy--;
        // limit to edge of image (reuse border pixels)
        r1 = h + iy;
        r2 = r1 + 1;
        c1 = w + ix;
        c2 = c1 + 1;
        outside = (r1 < 0) or (r2 >= int(height)) or
                  (c1 < 0) or (c2 >= int(width));
        if (outside)
          // may be outside by just one pixel
          outsideByOne = (((r2 == 0) or (r2 == int(height))) and
                                (c2 > 0) and (c2 < int(width))) or
                         (((c2 == 0) or (c2 == int(width))) and
                                (r2 > 0) and (r2 < int(height)));
        else
          outsideByOne = false;
        //
        if (outside and not outsideByOne)
            // both outside - use gray
            *pdd = gray;
        else
        { // not (completely) outside
          // get pixels
          i = r1 * width + c1;
          pss = &ps[i];
          p1 = pss[0];
          p2 = pss[1];
          p3 = pss[width];
          p4 = pss[width + 1];
          if (outsideByOne)
          { // just outside by one, so adjust index to inside
            if (r2 == 0)
            { // top row missing
              p1 = p3;
              p2 = p4;
            }
            if (r2 == int(height))
            { // bottom row missing
              p3 = p1;
              p4 = p2;
            }
            if (c2 == 0)
            { // left column missing
              p1 = p2;
              p3 = p4;
            }
            if (c2 == int(width))
            { // right column missing
              p2 = p1;
              p4 = p3;
            }
          }
          outside = false;
        }
        if (not outside)
        { // get index to top-left of pixels in question
          // get 4 pixels in question
          // get left and right share
          i2 = oxy.dx - ix * decimalFactor; // part of pixel in position (ix, iy)
          i1 = decimalFactor - i2;          // part of (ix+1, iy+1)
          // average for top set of pixels
          pt.y = (unsigned char)((int(p1.y) * i1  + int(p2.y) * i2)/decimalFactor);
          pt.u = (unsigned char)((int(p1.u) * i1  + int(p2.u) * i2)/decimalFactor);
          pt.v = (unsigned char)((int(p1.v) * i1  + int(p2.v) * i2)/decimalFactor);
          // average for bottom set of pixels
          pb.y = (unsigned char)((int(p3.y) * i1  + int(p4.y) * i2)/decimalFactor);
          pb.u = (unsigned char)((int(p3.u) * i1  + int(p4.u) * i2)/decimalFactor);
          pb.v = (unsigned char)((int(p3.v) * i1  + int(p4.v) * i2)/decimalFactor);
          // get shares of top and bottom
          i2 = oxy.dy - iy * decimalFactor; // part of pixel in position (ix, iy)
          i1 = decimalFactor - i2; // part of (ix, iy+1)
          // find result pixel intensity
          pr.y = (unsigned char)((int(pt.y) * i1  + int(pb.y) * i2)/decimalFactor);
          pr.u = (unsigned char)((int(pt.u) * i1  + int(pb.u) * i2)/decimalFactor);
          pr.v = (unsigned char)((int(pt.v) * i1  + int(pb.v) * i2)/decimalFactor);
          // implement
          *pdd = pr;
        }
      }
      else
      { // no change, so set destination to source
        pss = &ps[h * width];
        *pdd = pss[w];
      }
      pdd++;
    }
  }
  return result;
}

///////////////////////////////////////////////////////////////////////

void UCamRad::saveRadialCorrectionMatrix(char * filename)
{
  const int w = int(dev->MAX_IMAGE_WIDTH / par.getPixelSize());
  const int h = int((w * 3) / 4);
  FILE * f;
  int x, y;
  UXYoffset oxy; // offset x and y
  //
  if (radialOffsetValid)
  {
    f = fopen(filename, "w");
    if (f != NULL)
    {
      fprintf(f, "Radial error (x,y) pixel offset, with %d decimal offset\n", resultDecimalFactor);
      fprintf(f, "k1: %e, k2: %e, hx: %f, hy: %f\n",
      par.getK1(), par.getK2(), par.getHx(), par.getHy());
      fprintf(f, "first 20 in each corner and +/- 10 around center axis\n");
      for (y = 0; y < 20; y++)
      {
        fprintf(f, "%03d:", y);
        for (x = 0; x < 20; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w/2 - 10; x < w/2 + 10; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w - 20; x < w; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f,"\n");
      }
      fprintf(f, " ------------- \n");
      for (y = h/2 - 10; y < h/2 + 10; y++)
      {
        fprintf(f, "%03d:", y);
        for (x = 0; x < 20; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w/2 - 10; x < w/2 + 10; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w - 20; x < w; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f,"\n");
      }
      fprintf(f, " ------------- \n");
      for (y = h - 20; y < h; y++)
      {
        fprintf(f, "%03d:", y);
        for (x = 0; x < 20; x++)
        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w/2 - 10; x < w/2 + 10; x++)

        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f, " -- ");
        for (x = w - 20; x < w; x++)

        {
          oxy = radialOffset[y][x];
          fprintf(f, " %4d,%4d", oxy.dx, oxy.dy);
        }
        fprintf(f,"\n");
      }
      fclose(f);
    }
  }
}

/////////////////////////////////////////////////////////

bool UCamRad::setCameraParameters(float hx, float hy,
                  float k1, float k2,  float focalLng)
{
  bool result;
  double pixSize = dev->getPixelSize();
  //
  result = par.setCameraParameters(hx, hy, k1, k2, focalLng, pixSize);
  if (varIntrinsic != NULL)
    createVars();
  if (varDistortion != NULL)
  { // then the rest is probably OK too
    varDistortion->setDouble(k1, 0);
    varDistortion->setDouble(k2, 1);
    varIntrinsic->setDouble(focalLng / pixSize, 0);
    varIntrinsic->setDouble(focalLng / pixSize, 4);
    varIntrinsic->setDouble(hx / pixSize, 2);
    varIntrinsic->setDouble(hy / pixSize, 5);
  }
  if (result)
  {
    updateLensParams(0, 0);
    result = setRadialErrorMatrix();
  }
  return result;
}

///////////////////////////////////////////////////////

bool UCamRad::setRadialErrorMatrix()
{ // Removes radial error according to
  // calibration information in the referenced camera
  // removal follows the guidelines in Image Analysis,
  // Vision and Computer Graphics DTU/IMM publication
  // by Jens Michael Carstensen Lyngby 2001, page 75.
  //
  // error removal matrix is build in actual resolution.
  // head-point (hx and hy) and correction factors (k1 and k2)
  // are always in max resolution.
  bool result = true;
  UXYoffset oxy; // offset x and y
  float oldResFactor = par.getPixelSize();
  // force remake always
  radialOffsetValid = false;
  //
  if (not radialOffsetValid)
  { //assume max image size
    int x,y; // pixel position
    // matrix is always in full resolution
    par.setPixelSize(1.0);
    const int w = int(MAX_IMAGE_WIDTH_RAD_SQUARE);
    const int h = int(MAX_IMAGE_HEIGHT_RAD_SQUARE);
    // UPixel pix;
    for (y = 0; y < h; y++)
    {
      for (x = 0; x < w; x++)
      { // get offset value ...
        //
        // debug
        if ((y == (h-1)) and (x == (w-1)))
          oxy.dx = -30;
        // debug end
        //
        oxy = getRadialU2DOffsetInt(x + int(par.getHx()), y + int(par.getHy()));
        // and store in matrix
        radialOffset[y][x] = oxy;
      }
    }
    radialOffsetValid = true;
    /*
    printf("Radial error maksimum offset x=%1.2f, y=%1.2f (res fac %f)\n",
                float(mxy.dx) / float(resultDecimalFactor),
                float(mxy.dy) / float(resultDecimalFactor), ResFactor);
    */
  }
  // return to previous size
  par.setPixelSize(oldResFactor);
  return result;
}

//////////////////////////////////////////////////

UXYoffset UCamRad::getRadialU2DOffsetInt(int x, int y)
{ // Gets radial error offset according to
  // calibration information in the camera.
  // Follows the guidelines in Image Analysis,
  // Vision and Computer Graphics DTU/IMM publication
  // by Jens Michael Carstensen Lyngby 2001, page 75.
  //
  UXYoffset oxy; // offset x and y
  float dx, dy; // distance from center
  //
  getRadialU2DOffset(x, y, &dx, &dy);
  //
  oxy.dx = roundi(dx * resultDecimalFactor);
  oxy.dy = roundi(dy * resultDecimalFactor);
  //
  return oxy;
}

/////////////////////////////////////////////////////////////////

bool UCamRad::getRadialU2DOffset(float xu, float yu,
                  float * dxd, float * dyd)
{ // Gets radial error offset from undistorted to distorted image
  // in pixels
  bool result;
  float x,y;
  result = par.getRadialU2D(xu, yu, &x, &y);
  *dxd = x - xu;
  *dyd = y - yu;
  return result;
}

/////////////////////////////////////////////////////////////////


void UCamRad::createVars()
{
  if (vars != NULL)
  {
    UCamPanTiltDevice::createVars();
    //
    varIntrinsic = vars->addVarA("intrinsic", "709   0  320;"
                              "  0 709  240;"
                              "  0   0    1", "m",
                              "(r) Camera intrinsic matrix holding focal length (f) and headpoint (c)"
                              " in pixels: [0,0]=fx [1,1]=fy. Headpoint [0,2]=cx, [1,2]=cy");
    varDistortion = vars->addVarA("distortion","-0.207  0.2813  0.0013  0.0036  -0.3", "m",
                              "(r/w) lens distortion: [k1, k2, p1, p2, k3], k is radial and p is tangential values (see openCV manual)");
  }
}

/////////////////////////////////////////////

bool UCamRad::lensParChanged()
{
  bool result = false;
  double f, cx, cy, k1, k2;
  if (varIntrinsic != NULL)
  {
    f = ((varIntrinsic->getValued(0) + varIntrinsic->getDouble(4))/2.0);
    cx = varIntrinsic->getDouble(2);
    cy = varIntrinsic->getDouble(5);
    k1 = varDistortion->getDouble(0)/f;
    k2 = varDistortion->getDouble(0)/(f*f*f);
    result = par.getK1() != k1 or
             par.getK2() != k2 or
             par.getFocalLength() != f or
             par.getHx() != cx or
             par.getHy() != cy;
  }
  return result;
}

/////////////////////////////////////////////

void UCamRad::updateLensParams(const int height, const int width)
{
  double f, cx, cy, k1, k2, s;
  CvSize imageSize;
  bool redo;
  UMatrix4 intrinsic, distortion;
  //
  if (varIntrinsic != NULL)
  {
    f = ((varIntrinsic->getValued(0) + varIntrinsic->getDouble(4))/2.0);
    cx = varIntrinsic->getDouble(2);
    cy = varIntrinsic->getDouble(5);
    k1 = varDistortion->getDouble(0)/f;
    k2 = varDistortion->getDouble(0)/(f*f*f);
    s = getDev()->getPixelSize();
    redo = (par.getK1() != k1 or
             par.getK2() != k2 or
             par.getFocalLength() != f or
             par.getHx() != cx or
             par.getHy() != cy);
    if (redo)
      par.setCameraParameters(cx, cy, k1, k2, f, s);
    //
    //
    imageSize = cvSize(width, height);
    if (imageSize.width > 0 and imageSize.height > 0)
    { // and image size is available, so continue
      if (mapx == NULL)
      {
        mapx = cvCreateImage(imageSize, IPL_DEPTH_32F, 1);
        mapy = cvCreateImage(imageSize, IPL_DEPTH_32F, 1);
        redo = true;
      }
      else if (imageSize.width != mapx->width)
      {
        cvReleaseImage(&mapx);
        cvReleaseImage(&mapy);
        mapx = cvCreateImage(imageSize, IPL_DEPTH_32F, 1);
        mapy = cvCreateImage(imageSize, IPL_DEPTH_32F, 1);
        redo = true;
      }
      if (redo)
      {
        intrinsic.setMat(3, 3, varIntrinsic->getValuesd());
        distortion.setMat(5, 1, varDistortion->getValuesd());
#if (CV_MAJOR_VERSION >= 1)
        cvInitUndistortMap( intrinsic.cvMat(), distortion.cvMat(), mapx, mapy );
#else
        printf("UCamRad::updateLensParams: used cvInitUndistortMap(...) is incompatible with openCV version\n");
#endif
      }
    }
  }
}

