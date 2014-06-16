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
#include "ucamparest.h"

////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////
// camera internal parameters
/*
UCamPar::UCamPar()
{
  valid = false;
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
};

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
                  float pixSizeFactor,
                  const char * name)
{
  int err = 0;
  //
  headX = hx;
  headY = hy;
  radialK1 = k1;
  radialK2 = k2;
  focalLength = focalLng;
  resFactor = pixSizeFactor;
  if (name != NULL)
    strncpy(camName, name, MAX_CAMERA_NAME_LENGTH);
  else
    strcpy(camName, "noname");
  err = setMatrices();
  valid = (err == 0);
  //
  return valid;
}

///////////////////////////////////////////////////////

bool UCamPar::setPixelSize(float pixSize)
{
  bool result;
  int err;
  //
  resFactor = pixSize;
  // recalculate conversion matrices
  err = setMatrices();
  result = (err == 0);
  //
  return result;
};

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
                      rf,
                      source->camName);
}

///////////////////////////////////////////////////////

int UCamPar::setMatrices(void)
{
  int err;
  //
  err = ((focalLength > 0) and (resFactor > 0));
  if (err != 0)
  { // make P matrix
    mP.init(3,3,1.0); // unit matrix
    mP.set(2, 2, -1.0 / focalLength * resFactor);
    // now from image plane x,y to image pixel x,y

    // here y is down, and 0,0 is top left
    mb.init(3,3,1.0);
    mb.set(1, 1, -1); // turns Y direction down
    mb.set(0, 2, headX / resFactor);
    mb.set(1, 2, headY / resFactor);
    // conversion from Image coordinates to pixel coordinate
    mItoP = mb * mP;
    // and from pixel to image
    mPtoI = mItoP.inversed();
    err = mPtoI.err;
  }
  //
  return err;
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
    setCameraParameters(iHx, iHy, ik1, ik2, ifl, resFactor, key);
  }
  return result;
}

///////////////////////////////////////////////////////

int UCamPar::snprint(char * dest, const char * preString, const int maxLength)
{
  snprintf(dest, maxLength, "%s a3:%8.2e, a5:%8.2e, c:%7.2f, hx:%6.1f, hy:%6.1f",
                   preString,
                   radialK1, radialK2,
                   focalLength, headX, headY);
  return strlen(dest);
}
*/
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

UCamParEst::UCamParEst()
{
  mA = new UMatrixBig(40,35);
  vK = new UMatrixBig(40);
}

///////////////////////////////////////////////////////

UCamParEst::~UCamParEst()
{
  delete mA;
  delete vK;
}

///////////////////////////////////////////////////////

void UCamParEst::initParEst()
{
  mA->init(40,5,0.0);
  vK->init(40,1, 0.0);
}

///////////////////////////////////////////////////////

void UCamParEst::initParEst35()
{
  mA->init(40,35,0.0);
  vK->init(40,1, 0.0);
}

///////////////////////////////////////////////////////

bool UCamParEst::setEstMatrix(
             int m,   // equation element
             float pixX,  // pix position
             float pixY,  //
             UPosition posChart, // position on chart (z = 0)
             UPosition posB, // translation of barcode
             URotation rotB) // rotation of barcode
{
  bool result;
  const int row1 = m;
  const int row2 = m + 1;
  const double so = sin(rotB.Omega);
  const double co = cos(rotB.Omega);
  const double sp = sin(rotB.Phi);
  const double cp = cos(rotB.Phi);
  const double sk = sin(rotB.Kappa);
  const double ck = cos(rotB.Kappa);
  const double cosk = co * sk;
  const double cock = co * ck;
  const double cpck = cp * ck;
  const double cpsk = cp * sk;
  const double spck = sp * ck;
  const double spsk = sp * sk;
  const double csc = cp * so * ck;
  const double css = cp * so * sk;
  const double sss = sp * so * sk;
  const double ssc = sp * so * ck;
  const double xc = posChart.x;
  const double yc = posChart.y;
  const double xh = getHx();
  const double yh = getHy();
  const double c = getFocalLength();
  const double xt = posB.x;
  const double yt = posB.y;
  const double zt = posB.z;
  const double xm = pixX;
  const double ym = pixY;
  const double a3 = getK1();
  const double a5 = getK2();
  const double r2 = sqr(xm-xh) + sqr(-ym+yh);
  const double aa3a5 = 1.0 + a3*r2 + a5*sqr(r2);
  int i;
  const int MSL = 250;
  char s[MSL];
  char s2[MSL];
  //
  result = (m < int(mA->rows() - 1));
  if (result)
  { // constant value at liniarization point
    vK->setRC(row1, 0, -(xc*cpck + xc*sss - yc*cpsk + yc*ssc + xt)*c /
                 (-xc*spck + xc*css + yc*spsk + yc*csc + zt) -
                 (xm-xh) * aa3a5);
    vK->setRC(row2, 0, -(cosk*xc + cock*yc + yt)*c /
                 (-xc*spck + xc*css + yc*spsk + yc*csc + zt) -
                 (-ym+yh) * aa3a5);
    // jacobian for vector [a3, a5, c, xh, yh]
    // a3 elements
    mA->setRC(row1, 0, -( xm-xh)*r2);
    mA->setRC(row2, 0, -(-ym+yh)*r2);
    // a5 elements
    mA->setRC(row1, 1, -( xm-xh)*sqr(r2));
    mA->setRC(row2, 1, -(-ym+yh)*sqr(r2));
    // focal length c
    mA->setRC(row1, 2, -(xc*cpck + xc*sss - yc*cpsk + yc*ssc + xt) /
                    (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    mA->setRC(row2, 2, -(cosk*xc + cock*yc + yt) /
                    (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    // head point x
    mA->setRC(row1, 3, 1.0 + a3*r2 + a5*sqr(r2) - ( xm-xh)*(a3*(-2.0*xm+2.0*xh) +
                       2.0*a5*r2*(-2.0*xm+2.0*xh)));
    mA->setRC(row2, 3, -(-ym+yh)*(a3*(-2.0*xm+2.0*xh) + 2.0*a5*r2*(-2.0*xm+2.0*xh)));
    // head point y
    mA->setRC(row1, 4, -( xm-xh)*(a3*(-2.0*ym+2.0*yh) + 2.0*a5*r2*(-2.0*ym-2.0*yh)));
    mA->setRC(row2, 4, -1.0 - a3*r2 - a5*sqr(r2) -(-ym+yh)*(a3*(-2.0*ym+2.0*yh) +
                       2.0*a5*r2*(-2.0*ym+2.0*yh)));
  }
  // debug (2 datasets)
  if (false and (row1 == 0 or row1 == 8))
  { /*
    snprintf(s, MAX_CAM_INFO_SIZE, "xc: x:%f, y:%f, z%f", xc, yc, posChart.z);
    sVinfo(s, edebug, 0);
    snprintf(s, MAX_CAM_INFO_SIZE, "xm: x:%f, y:%f", xm, ym);
    sVinfo(s, edebug, 0);
    snprintf(s, MAX_CAM_INFO_SIZE, "xt: x:%f, y:%f, z:%f", xt, yt, zt);
    sVinfo(s, edebug, 0);
    */
    strcpy(s, "row1:");
    for (i = 0; i < 5; i++)
    {
      snprintf(s2, MSL, "%f ", mA->get(row1,i));
      strcat(s, s2);
    }
    snprintf(s2, MSL, " : %f ", vK->get(row1));
    strcat(s, s2);
    //
    toLog(s, 3);
    strcpy(s, "row2:");
    for (i = 0; i < 5; i++)
    {
      snprintf(s2, MSL, "%f ", mA->get(row2,i));
      strcat(s, s2);
    }
    snprintf(s2, MSL, " : %f ", vK->get(row2));
    strcat(s, s2);
    //
    toLog(s, 3);
  }
  // debug end
  //
  return result;
}

////////////////////////////////////////////////////////////////////

bool UCamParEst::setInitialValues35(
             unsigned int m,       // dataset element
             float pixX,  // pix position
             float pixY,  //
             UPosition posChart, // position on chart (z = 0)
             UPosition posB, // translation of barcode
             URotation rotB, // rotation of barcode
             unsigned int set)        // position set
{
  bool result;
  //
  result = (m < 20) and (set < 5);
  if (result)
  { // pis position
    pixPosX[m] = pixX;
    pixPosY[m] = pixY;
    posOnGmk[m] = posChart;
    // gmk position
    posGmk[set] = posB;
    rotGmk[set] = rotB;
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UCamParEst::setEstMatrix35(
             int m,   // equation element
             float pixX,  // pix position
             float pixY,  //
             UPosition posChart, // position on chart (z = 0)
             UPosition posB, // translation of barcode
             URotation rotB, // rotation of barcode
             int set)        // position set
{
  bool result;
  const int row1 = m*2;
  const int row2 = m*2 + 1;
  const double so = sin(rotB.Omega);
  const double co = cos(rotB.Omega);
  const double sp = sin(rotB.Phi);
  const double cp = cos(rotB.Phi);
  const double sk = sin(rotB.Kappa);
  const double ck = cos(rotB.Kappa);
  const double sosk = so * sk;
  const double sock = so * ck;
  const double cosk = co * sk;
  const double cock = co * ck;
  const double spsk = sp * sk;
  const double spck = sp * ck;
  const double cpsk = cp * sk;
  const double cpck = cp * ck;
  const double csc = cp * so * ck;
  const double css = cp * so * sk;
  const double sss = sp * so * sk;
  const double ssc = sp * so * ck;
  const double scs = sp * co * sk;
  const double scc = sp * co * ck;
  const double ccs = cp * co * sk;
  const double ccc = cp * co * ck;
  const double xc = posChart.x;
  const double yc = posChart.y;
  const double xh = getHx();
  const double yh = getHy();
  const double c = getFocalLength();
  const double xt = posB.x;
  const double yt = posB.y;
  const double zt = posB.z;
  const double xm = pixX;
  const double ym = pixY;
  const double a3 = getK1();
  const double a5 = getK2();
  const double r2 = sqr(xm-xh) + sqr(-ym+yh);
  const double aa3a5 = 1.0 + a3*r2 + a5*sqr(r2);
  double n1, t1;
  int i;
  const int MSL = 250;
  char s[MSL];
  char s2[MSL];
  //
  result = (m < int(mA->rows() - 1));
  if (result)
  { // constant value at liniarization point
    vK->setRC(row1, 0, -(xc*cpck + xc*sss - yc*cpsk + yc*ssc + xt)*c /
                 (-xc*spck + xc*css + yc*spsk + yc*csc + zt) -
                 (xm-xh) * aa3a5);
    vK->setRC(row2, 0, -(cosk*xc + cock*yc + yt)*c /
                 (-xc*spck + xc*css + yc*spsk + yc*csc + zt) -
                 (-ym+yh) * aa3a5);
    // jacobian for vector [a3, a5, c, xh, yh]
    // a3 elements
    mA->setRC(row1, 0, -( xm-xh)*r2);
    mA->setRC(row2, 0, -(-ym+yh)*r2);
    // a5 elements
    mA->setRC(row1, 1, -( xm-xh)*sqr(r2));
    mA->setRC(row2, 1, -(-ym+yh)*sqr(r2));
    // focal length c
    mA->setRC(row1, 2, -(xc*cpck + xc*sss - yc*cpsk + yc*ssc + xt) /
                    (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    mA->setRC(row2, 2, -(cosk*xc + cock*yc + yt) /
                    (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    // head point x
    mA->setRC(row1, 3, 1.0 + a3*r2 + a5*sqr(r2) - ( xm-xh)*(a3*(-2.0*xm+2.0*xh) +
                       2.0*a5*r2*(-2.0*xm+2.0*xh)));
    mA->setRC(row2, 3, -(-ym+yh)*(a3*(-2.0*xm+2.0*xh) + 2.0*a5*r2*(-2.0*xm+2.0*xh)));
    // head point y
    mA->setRC(row1, 4, -( xm-xh)*(a3*(-2.0*ym+2.0*yh) + 2.0*a5*r2*(-2.0*ym-2.0*yh)));
    mA->setRC(row2, 4, -1.0 - a3*r2 - a5*sqr(r2) -(-ym+yh)*(a3*(-2.0*ym+2.0*yh) +
                       2.0*a5*r2*(-2.0*ym+2.0*yh)));
    // xt for set
    mA->setRC(row1, 5 + 6 * set, -c / (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    mA->setRC(row2, 5 + 6 * set, 0);
    // yt for set
    mA->setRC(row1, 6 + 6 * set, 0);
    mA->setRC(row2, 6 + 6 * set, -c / (-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    // yt for set
    mA->setRC(row1, 7 + 6 * set, (cosk*xc + cock*yc + yt)*c/
                              sqr(-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    mA->setRC(row2, 7 + 6 * set, (xc*cpck + xc*sss - yc*cpsk + yc*ssc + xt)*c/
                              sqr(-xc*spck + xc*css + yc*spsk + yc*csc + zt));
    // omega for set
    n1 = -xc*spck + xc*css + yc*spsk + yc*csc + zt;
    t1 = xc*cpck + xc*sss - yc*cpsk + yc*ssc;
    mA->setRC(row1, 8 + 6 * set, -(xc*scs + yc*scc)*c/(n1) +
                    (t1 + xt)*c*(xc*ccs + yc*ccc) /
                    sqr(n1));
    mA->setRC(row2, 8 + 6 * set, -(-sosk*xc - sock*yc)*c/(n1) +
                    (cosk*xc + cock*yc + yt)*c*(xc*ccs + yc*ccc) / sqr(n1));
    // phi for set
    mA->setRC(row1, 9 + 6 * set, -(-xc*spck + xc*css + yc*spsk + yc*csc)*c/n1 +
                              (t1 + xt)*c*(-t1)/sqr(n1));
    mA->setRC(row2, 9 + 6 * set, (cosk*xc + cock*yc + yt)*c*(-t1)/sqr(n1));
    // kappa for set
    mA->setRC(row1, 10 + 6 * set, (-xc*cpsk + xc*ssc - yc*cpck - yc*sss)*c/n1 +
                       (t1 + xt)*c*(xc*spsk + xc*csc + yc*spck - yc*css)/sqr(n1));
    mA->setRC(row2, 10 + 6 * set, (cock*xc - cosk*yc)*c/n1 +
                      (cosk*xc + cock*yc + yt)*c*(xc*spsk + xc*csc + yc*spck - yc*css)/
                      sqr(n1));
  }
  // debug (2 datasets)
  if (false and (row1 == 0 or row1 == 8))
  { /*
    snprintf(s, MAX_CAM_INFO_SIZE, "xc: x:%f, y:%f, z%f", xc, yc, posChart.z);
    sVinfo(s, edebug, 0);
    snprintf(s, MAX_CAM_INFO_SIZE, "xm: x:%f, y:%f", xm, ym);
    sVinfo(s, edebug, 0);
    snprintf(s, MAX_CAM_INFO_SIZE, "xt: x:%f, y:%f, z:%f", xt, yt, zt);
    sVinfo(s, edebug, 0);
    */
    strcpy(s, "row1:");
    for (i = 0; i < 5; i++)
    {
      snprintf(s2, MSL, "%f ", mA->get(row1,i));
      strcat(s, s2);
    }
    snprintf(s2, MSL, " : %f ", vK->get(row1));
    strcat(s, s2);
    //
    toLog(s, 4);
    strcpy(s, "row2:");
    for (i = 0; i < 5; i++)
    {
      snprintf(s2, MSL, "%f ", mA->get(row2,i));
      strcat(s, s2);
    }
    snprintf(s2, MSL, " : %f ", vK->get(row2));
    strcat(s, s2);
    //
    toLog(s, 4);
  }
  // debug end
  //
  return result;
}

////////////////////////////////////////////////////////////

bool UCamParEst::adjustParameters(bool debug)
{
  bool result;
  UMatrix4 mR(6);
  double dK1;
  double dK2;
  double dC;
  double dHx;
  double dHy;
  double dJ = 0.0, sumK = 0.0;
  const int MSL = 250;
  char s[MSL];
  const double redFactor = 1.0; // parameter adjustment reduction factor
  unsigned int i;
  //
  result = solveLeastSquare(&mR);     // mA
  if (result)
  {
    dK1 = mR.get(0) / redFactor;
    dK2 = mR.get(1) / redFactor;
    dC  = mR.get(2) / redFactor;
    dHx = mR.get(3) / redFactor;
    dHy = mR.get(4) / redFactor;
    // weighed sum
    // typical 1050.0, 1.4e-7, 8.4e-13, 320.0, 240.0
    if (debug)
    { // old values
      /*
      snprintf(s, MAX_CAM_INFO_SIZE, "Par new    K1:%7.1e, K2:%7.1e, C:%6.1f, "
                    "hx:%5.1f, hy:%5.1f",
                    radialK1, radialK2, focalLength, headX, headY);
      sVinfo(s, edebug, 0);
      */
      // get a summed change value
      dJ = absd(dK1) / 1.4e-7 + absd(dK2) / 1e-12 +
                 dC / 1000.0 + dHx / 320.0 + dHy / 240.0;
      dJ *= redFactor;
      //
      sumK = 0;
      for (i = 0; i < vK->size(); i++)
        sumK += sqr(vK->get(i));
      //
    }
    // implement change
    radialK1 -= dK1;
    radialK2 -= dK2;
    focalLength -= dC/20.0;
    headX -= dHx;
    headY -= dHy;
    if (debug)
    { // print new values
      snprintf(s, MSL, "Par change K1:%10.2e, K2:%10.2e, C:%7.2f, "
                    "hx:%6.2f, hy:%6.2f, tot%5.3f sumK:%5.3f",
                     dK1, dK2, dC, dHx, dHy, dJ, sumK);
      toLog(s, 3);
      snprintf(s, MSL, "Par new    K1:%10.2e, K2:%10.2e, C:%7.2f, "
                    "hx:%6.2f, hy:%6.2f",
                    radialK1, radialK2, focalLength, headX, headY);
      toLog(s, 3);
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////////////

bool UCamParEst::adjustParameters35(bool debug)
{
  bool result;
  UMatrix4 vR(6);
  double dK1;
  double dK2;
  double dC;
  double dHx;
  double dHy;
  double dJ = 0.0, sumK = 0.0;
  const int MSL = 250;
  char s[MSL];
  const double redFactor = 0.2;   // parameter adjustment (reduction) factor
  unsigned int i, loop, set, d;
  UPosition dPosChart[5];
  URotation dRotChart[5];
  UPosition dsumPosChart[5]; // zero initial value
  URotation dsumRotChart[5]; // zero initial value
  //
  //
  for (loop = 0; loop < 5; loop++)
  {
    // load linearized matrix with current estimate
    d = 0;
    for (set = 0; set < 5; set++)
      for (i = 0; i < 4; i++)
      {
        setEstMatrix35(d, pixPosX[d], pixPosY[d], posOnGmk[d],
                       posGmk[set] + dsumPosChart[set],
                       rotGmk[set] + dsumRotChart[set], set);
        d++;
      }
    // solve for adjusted parameters
    result = solveLeastSquare(&vR); // mA     vK
    if (result)
    {
      dK1 = vR.get(0) * redFactor;
      dK2 = vR.get(1) * redFactor;
      dC  = vR.get(2) * redFactor;
      dHx = vR.get(3) * redFactor;
      dHy = vR.get(4) * redFactor;
      d = 6;
      for (set = 0; set < 5; set++)
      {
        dPosChart[set].set(vR.get(d  ), vR.get(d+1), vR.get(d+2));
        dRotChart[set].set(vR.get(d+3), vR.get(d+4), vR.get(d+5));
        // "implement"
        dsumPosChart[set] = dsumPosChart[set] - dPosChart[set].scaled(redFactor);
        dsumRotChart[set] = dsumRotChart[set] - dRotChart[set].scaled(redFactor);
        d += 6;
      }
      // implement change
      radialK1 -= dK1;
      radialK2 -= dK2;
      focalLength -= dC / 10.0;
      headX -= dHx;
      headY -= dHy;
      //
      if (debug)
      { // print new values
        snprintf(s, MSL, "Par change K1:%10.2e, K2:%10.2e, C:%7.2f, "
                      "hx:%6.2f, hy:%6.2f, tot%5.3f sumK:%5.3f",
                       dK1, dK2, dC, dHx, dHy, dJ, sumK);
        toLog(s, 3);
        snprintf(s, MSL, "Par new    K1:%10.2e, K2:%10.2e, C:%7.2f, "
                      "hx:%6.2f, hy:%6.2f",
                      radialK1, radialK2, focalLength, headX, headY);
        toLog(s, 3);
      }
    }
  }
  //
  return result;
}

////////////////////////////////////////////////////////////

bool UCamParEst::solveLeastSquare(UMatrix4 * R)
{
  bool result;
  //
  mA->solve(vK, R);
  result = (R->err == 0);
  /*
  UMatrixBig BB(40);
  //
  BB = mA.transposed() * mA;
  result = (BB.err == 0);
  if (result)
  {
    BB.inverse();
    result = (BB.err == 0);
  }
  if (result)
  {
    *R = BB * mA.transposed() * vK;
    result = (R->err == 0);
  }
  */
  return result;
}


