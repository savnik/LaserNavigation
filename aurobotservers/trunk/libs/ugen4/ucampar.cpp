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
#include "ucampar.h"

////////////////////////////////////////
// camera internal parameters

UCamPar::UCamPar()
{
  parValid = false;
  // typical 1050.0, 1.4e-7, 8.4e-13, 320.0, 240.0
  focalLength = 1050.0;
  radialK1 = 1.4e-7;
  radialK2 = 8.4e-13;
  headX = 320.0;
  headY = 240.0;
  //
  resFactor = 1.0;
}

////////////////////////////////////////

UCamPar::~UCamPar()
{
}

////////////////////////////////////////

bool UCamPar::getRadialD2U(float xd, float yd,
                           float * xu, float * yu)
{ // Gets radial error offset according to
  // calibration information in the camera.
  // Follows the guidelines in Image Analysis,
  // Vision and Computer Graphics DTU/IMM publication
  // by Jens Michael Carstensen Lyngby 2001, page 75.
  //
  float rx, ry; // distance from center
  float r2;  // radial distance from center
  float rcp; // radial correction factor
  //
  // calculate position in deformed image at
  // max resolution (at resFactor = 1)
  rx = xd * resFactor - headX;
  ry = yd * resFactor - headY;
  //
  // radius squared in pixels
  r2 = rx*rx + ry*ry;
  // radial error offset size in
  // pixels at used resolution
  rcp = (radialK1*r2 + radialK2*r2*r2) / resFactor;
  // report result
  *xu = xd + rx * rcp;
  *yu = yd + ry * rcp;
  //
  return true;
}

//////////////////////////////////////////

bool UCamPar::getRadialU2D(const float xu, const float yu,
                          float * xd, float * yd)
{ // get undistorted image coordinates
  int i;
  // real (undistorted) pixel position
  float rhx = headX / resFactor;
  float rhy = headY / resFactor;
  // radius
  float r;
  // real radius for distorted image
  float rr = hypot(xu - rhx, yu - rhy);
  float x,y;
  //
  *xd = xu; // initial estimate
  *yd = yu;
  getRadialD2U(*xd, *yd, &x, &y);
  for (i = 0; i < 5; i++)
  { // iterate to clean pixel position
    *xd = xu - (x - *xd);
    *yd = yu - (y - *yd);
    getRadialD2U(*xd, *yd, &x, &y);
    // get position in undistorted image
    r = hypot(x - rhx, y - rhy);
    // stop if error is below 0.05 pixel
    if (absf(r - rr) < 0.01)
      break;
  }
  return true;
}

///////////////////////////////////////////////////////

bool UCamPar::setCameraParameters(float hx, float hy,
                  float k1, float k2,  float focalLng,
                  float pixSizeFactor)
{
  headX = hx;
  headY = hy;
  radialK1 = k1;
  radialK2 = k2;
  focalLength = focalLng;
  resFactor = pixSizeFactor;
  parValid = setMatrices();
  //
  return parValid;
}

///////////////////////////////////////////////////////

bool UCamPar::setPixelSize(float pixSize)
{
  bool result;
  //
  resFactor = pixSize;
  // recalculate conversion matrices
  result = setMatrices();
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPar::setCameraParameters(UCamPar * source)
{
  float rf = source->getPixelSize();
  //
  return setCameraParameters(
                      source->getHx() * rf,
                      source->getHy() * rf,
                      source->getK1(), source->getK2(),
                      source->getFocalLength() * rf,
                      rf);
}

///////////////////////////////////////////////////////

bool UCamPar::setMatrices(void)
{
  bool result;
  //
  result = ((focalLength > 0) and (resFactor > 0));
  if (result)
  { // make P matrix
    mP.init(3, 3, 1.0); // unit matrix
    mP.setRC(2, 2, -1.0 / focalLength * resFactor);
    // now from image plane x,y to image pixel x,y

    // here y is down, and 0,0 is top left
    mb.init(3, 3, 1.0);
    mb.setRC(1, 1, -1.0); // turns Y direction down
    mb.setRC(0, 2, headX / resFactor);
    mb.setRC(1, 2, headY / resFactor);
    // conversion from Image coordinates to pixel coordinate
    mItoP = mb * mP;
    // and from pixel to image
    //mPtoI = mItoP.inversed();
    //err = mPtoI.err;
    mCtoPRob = getCtoPRob();
  }
  //
  return result;
}

///////////////////////////////////////////////////////

UMatrix4 UCamPar::getCtoPRob()
{ // add conversion from 3d robot (z is up) to 3d (z is back)
  // to image to pixel coordinates
  UMatrix4 result(3,4);
  UMatrix4 mRtoI(3,4,0.0);
  //
  mRtoI.setRow(0,  0.0, -1.0,  0.0,  0.0);
  mRtoI.setRow(1,  0.0,  0.0,  1.0,  0.0);
  mRtoI.setRow(2, -1.0,  0.0,  0.0,  0.0);
  //
  result = mItoP * mRtoI;
  //
  return result;
}

///////////////////////////////////////////////////////

bool UCamPar::savePar(Uconfig * ini, const char * key)
{
  bool result;
  //
  result = ((ini != NULL) and (key != NULL));
  if (result)
  {
      ini->doublePut(key, "focusLength", focalLength);
      ini->doublePut(key, "radialErrorK1", radialK1);
      ini->doublePut(key, "radialErrorK2", radialK2);
      ini->doublePut(key, "headPointX", headX);
      ini->doublePut(key, "headPointY", headY);
  }
  //
  return result;
}

/////////////////////////////////////////////////////////

bool UCamPar::setCameraParameters(Uconfig * ini, const char * key)
{ // set parameters from config file
  bool result;
  float ifl, ik1, ik2, iHx, iHy;
  //
  result = ((ini != NULL) and (key != NULL));
  if (result)
  {
    ifl = ini->doubleGet(key, "focusLength", focalLength);
    ik1 = ini->doubleGet(key, "radialErrorK1", radialK1);
    ik2 = ini->doubleGet(key, "radialErrorK2", radialK2);
    iHx = ini->doubleGet(key, "headPointX", headX);
    iHy = ini->doubleGet(key, "headPointY", headY);
    setCameraParameters(iHx, iHy, ik1, ik2, ifl, resFactor);
  }
  return result;
}

///////////////////////////////////////////////////////

int UCamPar::snprint(const char * preString, char * dest, const int maxLength)
{
  snprintf(dest, maxLength, "%s K1:%g, k2:%g, c:%g, hx:%.1f, hy:%.1f\n",
                   preString,
                   radialK1, radialK2,
                   focalLength, headX, headY);
  return strlen(dest);
}

///////////////////////////////////////////////////////
/*
URPos UCamPar::getCtoPRob(UPosition pos3D)
{
  URPos result;
  UMatrix4 res(3,1);
  UMatrix4 v3d;
  //
  v3d = pos3D.asVector4();
  v3d.transpose();
  //v3d.print("UCamPar::getCtoPRob.v3d");
  // get pixel position in homogene coordinates
  res = mCtoPRob * v3d;
  //res.print("UCamPar::getCtoPRob.res");
  // make [xw, yw ,w] to [x, y, 1.0]
  res.normalize();
  //res.print("UCamPar::getCtoPRob.res.normalized");
  // get position
  result.x = res.get(0,0);
  result.y = res.get(1,0);
  //
  return result;
}
*/
URPos UCamPar::getCtoPRob(UPosition pos3D)
{
  URPos res;
  double s, xc, yc;
  //
  s = - pos3D.x / focalLength * resFactor;
  if (absd(s) > 1e-30)
  {
    xc = pos3D.y / s;
    yc = pos3D.z / s;
    res.x = xc + headX / resFactor;
    res.y = yc + headY / resFactor;
  }
  else
  { // no result (too close to camera x = 0)
    res.x = 0;
    res.y = 0;
  }
  //
  //
  return res;
}


///////////////////////////////////////////////////////

UPosition UCamPar::getPtoCRob(int x, int y, float distance)
{
  UPosition result;
  //
  result.y = double((headX/resFactor - float(x)) * 
                        distance/focalLength*resFactor); // x left
  result.z = double((headY/resFactor - float(y)) * 
                        distance/focalLength*resFactor); // y up
  result.x = distance;
  //
  return result;
}

///////////////////////////////////////////////////////

UPosition UCamPar::getPtoCRob(float x, float y, float distance)
{
  UPosition result;
  //
  result.y = double((headX/resFactor - x) * 
      distance/focalLength*resFactor); // x left
  result.z = double((headY/resFactor - y) * 
      distance/focalLength*resFactor); // y up
  result.x = distance;
  //
  return result;
}

///////////////////////////////////////////////////////

void UCamPar::print(const char * prestring)
{
  printf("%s a3:%8.2e, a5:%8.2e, c:%7.2f, hx:%6.1f, hy:%6.1f",
     prestring, radialK1, radialK2, focalLength, headX, headY);
  printf(" - valid(%s),  pixel size %f\n", bool2str(parValid), resFactor);
  if (parValid)
  {
    mP.print("mP");
    mb.print("mb");
    mCtoPRob.print("mCtoPRob");
    mItoP.print("mItoP");
  }
}
