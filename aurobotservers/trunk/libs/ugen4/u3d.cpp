/***************************************************************************
 *   Copyright (C) 2010 by DTU (Christian Andersen)                        *
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

//#include <iostream.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "u3d.h"
#include "uline.h"
#include "u2dline.h"

////////////////////////////////////////////
// UPos
////////////////////////////////////////////

void UPos::show(const char * prestring)
{
  printf("%s: x:%d, y:%d\n", prestring, x, y);
}

////////////////////////////////////////////
// URPos
////////////////////////////////////////////

void URPos::show(const char * prestring)
{
  printf("%s: x:%f, y:%f\n", prestring, x, y);
}

////////////////////////////////////////////
// POSITION
////////////////////////////////////////////


UPosition::UPosition() : x(0), y(0), z(0)
{
}

////////////////////////////////////////////

UPosition::UPosition(double ix, double iy, double iz) :
x(ix), y(iy), z(iz)
{
}

////////////////////////////////////////////

UMatrix4 UPosition::asVector3(bool column)
{
  UMatrix4 pos(3);
  double * d;
  //
  if (column)
    pos.setSize(3, 1);
  else
    pos.setSize(1, 3);
  d = pos.getData();
  *d++ = x;
  *d++ = y;
  *d   = z;
  return pos;
}

////////////////////////////////////////////

UMatrix4 UPosition::asVector4(bool column)
{
  UMatrix4 pos(4);
  double * d;
  //
  if (column)
    pos.setSize(4, 1);
  else
    pos.setSize(1, 4);
  d = pos.getData();
  *d++ = x;
  *d++ = y;
  *d++ = z;
  *d   = 1.0;
  return pos;
}

////////////////////////////////////////////

void UPosition::println(const char * leadString)
{
  printf("%s  x:%9.4f  y:%9.4f  z:%9.4f\n", leadString, x,y,z);
}

////////////////////////////////////////////

void UPosition::sprint(char * s, const char * leadString)
{
  sprintf(s, "%s  x:%9.4f  y:%9.4f  z:%9.4f", leadString, x,y,z);
}

////////////////////////////////////////////

void UPosition::snprint(const char * leadString, char * s, const int bufferLength)
{
  snprintf(s, bufferLength, "%s  x:%9.4f  y:%9.4f  z:%9.4f\n",
           leadString, x,y,z);
}

////////////////////////////////////////////

void UPosition::show(const char * leadString /* = NULL */)
{
  if (leadString != NULL)
    printf("%s  x:%9.4f  y:%9.4f  z:%9.4f\n", leadString, x, y, z);
  else
    printf("x:%9.4f y:%9.4f z:%9.4f\n", x, y, z);
}

////////////////////////////////////////////

UMatrix4 UPosition::asMatrix4x4()
{ // returns translation 4x4 matrix with this distance
  UMatrix4 T(4, 4, 1.0);
  T.setRC(0, 3, -x);
  T.setRC(1, 3, -y);
  T.setRC(2, 3, -z);
  return T;
}

////////////////////////////////////////////
/*
UVector4 UPosition::GetPixelPos(UMatrix4 * A, UMatrix4 * b)
{ // convert using this A matrix center image coordinates
  // to pixel values and b matrix to convert to top-left
  // oriented coordinates
  UVector4 P;
  UVector4 R;
  UVector4 C;
  //
  R = asVector4();
  // R is size 4, A is size 3x4, b is size 3x3, P is size 3
  C = *A * R; // in camera coordinates
  P = *b * C; // in pixels
  // remove scale (w)
  P.normalize();
  return P;
};
*/
////////////////////////////////////////////

UPosition UPosition::operator = (UMatrix4 vec)
{
  copy(&vec);
  return *this;
}

////////////////////////////////////////////

UPosition UPosition::operator = (const URotation rot)
{
  copy(&rot);
  return *this;
}

////////////////////////////////////////////

UPosition UPosition::operator = (const U2Dpos val)
{
  x = val.x;
  y = val.y;
  z = 0.0;
  return *this;
}


////////////////////////////////////////////
/*
int UPosition::copy(UMatrix4 pos)
{ // assign position from vector
  return copy(&pos);
}
*/
////////////////////////////////////////////

int UPosition::copy(UMatrix4 * pos)
{
  double * d;
  //
  d = pos->getData();
  //
  if (pos->size() == 3)
  {
    x = d[0];
    y = d[1];
    z = d[2];
    return 0;
  }
  else if ((pos->size() == 4) && (fabs(d[3]) > 1e-10))
  {
    x = d[0] / d[3];
    y = d[1] / d[3];
    z = d[2] / d[3];
    return 0;
  }
  else
    return -1;
}

////////////////////////////////////////////

int UPosition::copy(const URotation * rot)
{
  x = rot->Omega;
  y = rot->Phi;
  z = rot->Kappa;
  return 0;
}

////////////////////////////////////////////

void UPosition::clear(void)
{
  set(0.0, 0.0, 0.0);
}

////////////////////////////////////////////

UPosition UPosition::position(double ix, double iy, double iz)
{
  UPosition res(ix, iy, iz);
  return res;
}

////////////////////////////////////////////

void UPosition::add(const UPosition * pos)
{
  x += pos->x;
  y += pos->y;
  z += pos->z;
}

////////////////////////////////////////////

void UPosition::add(const UPosition pos)
{
  x += pos.x;
  y += pos.y;
  z += pos.z;
}

////////////////////////////////////////////

void UPosition::add(const double ix, const double iy, const double iz)
{
  x += ix;
  y += iy;
  z += iz;
}

////////////////////////////////////////////

void UPosition::scale(double val)
{
  x *= val;
  y *= val;
  z *= val;
}

////////////////////////////////////////////

UPosition UPosition::added(UPosition * pos)
{
  UPosition P;
  P.x = x + pos->x;
  P.y = y + pos->y;
  P.z = z + pos->z;
  return P;
}

////////////////////////////////////////////

UPosition UPosition::added(UPosition pos)
{
  UPosition P;
  P.x = x + pos.x;
  P.y = y + pos.y;
  P.z = z + pos.z;
  return P;
}

////////////////////////////////////////////

UPosition UPosition::operator+ (UPosition pos)
{ // return this added to pos
  return added(pos);
}

////////////////////////////////////////////

UPosition UPosition::subtracted(UPosition * pos)
{ // returns the difference from this to pos
  UPosition P;
  P.x = x - pos->x;
  P.y = y - pos->y;
  P.z = z - pos->z;
  return P;
}

////////////////////////////////////////////

UPosition UPosition::subtracted(UPosition pos)
{ // returns the difference from this to pos
  UPosition P;
  P.x = x - pos.x;
  P.y = y - pos.y;
  P.z = z - pos.z;
  return P;
}

////////////////////////////////////////////

UPosition UPosition::operator- (UPosition pos)
{ // return this position minus pos: (result = this - pos)
  return subtracted(pos);
}

////////////////////////////////////////////

void UPosition::subtract(const UPosition * pos)
{ // returns the difference from this to pos
  x -= pos->x;
  y -= pos->y;
  z -= pos->z;
}

////////////////////////////////////////////

UPosition UPosition::scaled(const double val)
{
  UPosition P;
  P.x = x * val;
  P.y = y * val;
  P.z = z * val;
  return P;
}

////////////////////////////////////////////

UPosition UPosition::operator* (const double scalar)
{ // return this position scaled with scalar
  return scaled(scalar);
}

////////////////////////////////////////////

void UPosition::operator*= (const double scalar)
{ // scale with scalar
  scale(scalar);
}

////////////////////////////////////////////

void UPosition::operator+= (const UPosition pos)
{ // add to this
  add(pos);
}

////////////////////////////////////////////

void UPosition::operator-= (const UPosition pos)
{ // subtract
  subtract(&pos);
}

////////////////////////////////////////////

//int UPosition::SaveToReg(const char * subject, const char * key)
//{
//  return SaveToReg(&conf, subject, key);
//}

////////////////////////////////////////////

int UPosition::SaveToReg(Uconfig * ini, const char * subject, const char * key)
{ // saves all three values in one line in reg
  //int kl;
//  int err = 0;
  char s[MaxCharPerLine - MaxKeyLength] = "";
  char * ps = &s[0];
  char s2[MaxCharPerLine - MaxKeyLength] = "";
  char * ps2 = &s2[0];
  //
  //kl = strlen(subject);
//   if ((kl == 0) or (kl >= 100))
//     err=-1;
  //kl = strlen(key);
//   if ((kl == 0) or (kl >= 100))
//     err=-2;
  ps = doubleToKeyStr(x, ps);
  ps2 = doubleToKeyStr(y, ps2);
  ps = strncat(ps, ", ", MaxCharPerLine - MaxKeyLength);
  ps = strncat(ps, ps2, MaxCharPerLine - MaxKeyLength);
  ps2 = doubleToKeyStr(z, ps2);
  ps = strncat(ps, ", ", MaxCharPerLine - MaxKeyLength);
  ps = strncat(ps, ps2, MaxCharPerLine - MaxKeyLength);
  ini->strPut(subject, key, ps);
  return 0;
}

////////////////////////////////////////////

//int UPosition::LoadFromReg(const char * subject, const char * key)
//{
//  int err = 0;
//  err = LoadFromReg(&conf, subject, key);
//  return err;
//}

////////////////////////////////////////////

int UPosition::LoadFromReg(Uconfig * ini, const char * subject, const char * key)
{ // gets a string with 3 values as stored above
  int err = 0;
  int n;
//  char s[MaxCharPerLine - MaxKeyLength] = "";
  const char * ps;
  //
  ps = ini->strGet(subject, key, NULL);
  if (ps != NULL)
    n = sscanf(ps, "%lg,%lg,%lg", &x, &y, &z);
  else
    n = 0;
  if (n != 3)
  {
    err=-1;
    x=0;
    y=0;
    z=0;
  }
  return err;
}

////////////////////////////////////////////

double UPosition::distSq(const UPosition * pTo)
{
  double result;
  result = sqr(x - pTo->x) + sqr(y - pTo->y) + sqr(z - pTo->z);
  return result;
}

////////////////////////////////////////////

int UPosition::toUnitVector()
{ // convert position to a unit vector
  // returns 0 is succesfull
  int err = 0;
  double lng = dist();
  if (lng > 1e-50)
    scale(1.0/lng);
  else
    err = -1;
  return err;
}

//////////////////////////////////////////////////

UMatrix4 UPosition::getPixelPos(UMatrix4 * A,
                                UMatrix4 * b)
{ // convert using this A matrix center image coordinates
  // to pixel values and b matrix to convert to top-left
  // oriented coordinates
  UMatrix4 P(4,1);
  UMatrix4 R(4,1);
  UMatrix4 C(4,1);
  //bool isImage1, isImage2;
  R = asVector4(true);
  // R is size 4x1, A is size 3x4, b is size 3x3, P is size 3
  C = *A * R; // in camera coordinates
  P = *b * C; // in pixels
  // remove scale (w)
  P.normalize();
  return P;
}

////////////////////////////////////////////

double UPosition::dist(const UPosition * pTo)
{ // distance to another point
  double result = distSq(pTo);
  result = sqrt(result);
  return result;
}

/////////////////////////////////////////////

void UPosition::transfer(UMatrix4 * mA)
{ // transfers this position as described by mA
  // mA must be a 4x4 matrix
  UMatrix4 v(this->asVector4(true));
  UMatrix4 P(4,1);
  //v.transpose();
  P = *mA * v;
  this->copy(&P);
}

////////////////////////////////////////////

UPosition UPosition::transferred(UMatrix4 * mA)
{ // returnes this position transferred by mA
  // this position is unaffected
  // mA must be a 4x4
  UMatrix4 v(this->asVector4(true));
  UMatrix4 P(4);
  UPosition pos;
  //
  //v.transpose();
  P = *mA * v;
  pos.copy(&P);
  return pos;
}

////////////////////////////////////////////

bool UPosition::save(FILE * fmap, const char * key)
{
  bool result = (fmap != NULL);
  //
  fprintf(fmap, "<%s=\"%g,%g,%g\">", key, x, y, z);
  //
  return result;
}

////////////////////////////////////////////

bool UPosition::load(const char * valueString)
{
  bool result;
  int n;
  //
  n = sscanf(valueString, "%le,%le,%le", &x, &y, &z);
  result = (n == 3);
  //
  return result;
}

////////////////////////////////////////////

UPosition::~UPosition()
{}

////////////////////////////////////////////////////////

UPosition UPosition::getTangentPointXY(
                          const UPosition c,
                          const double bc,
                          const bool rightSide,
                          bool * valid, double * ang)
{ // return tangent point, where a line from this point
  // touches a circle with centre in c and radius bc.
  // if 'rightSide', then angle c-this-b is negative
  UPosition result = c;
  bool isOK = true;
  double ac; // distance
  double ACB = 0.0; // angle
  double CA; // angle in coordinate system
  double CB = 0.0; // angle in coordinate system
  // get distance to circle centre
  ac = hypot(c.x - x, c.y - y);
  // this point must be outside circle
  isOK = (ac > bc);
  if (isOK)
  { // OK get angle from tangebt point (b) to ref point (a)
    ACB = acos(bc/ac);
    // get angle from centre (c) to a in coordinate system
    CA = atan2(y - c.y, x - c.x);
    // add/subtract angle
    if (rightSide)
      CB = CA + ACB;
    else
      CB = CA - ACB;
    // add a bc-distance in direction of b
    result.x += bc * cos(CB);
    result.y += bc * sin(CB);
  }
  if (valid != NULL)
    *valid = isOK;
  if (ang != NULL)
    *ang = CB;
  return result;
}

///////////////////////////////////////////////////////

double UPosition::dot(UPosition v)
{
  double result;
  result = x * v.x + y * v.y + z * v.z;
  return result;
}

///////////////////////////////////////////////////////

UPosition UPosition::cross(UPosition v)
{
  UPosition result;
  result.x = y * v.z - z * v.y;
  result.y = z * v.x - x * v.z;
  result.z = x * v.y - y * v.x;
  return result;
}

//////////////////////////////////////////////////////

const char * UPosition::codeXml(char * buf, const int bufCnt, const char * extraAttr)
{
  const char * es = "";
  //
  if (extraAttr != NULL)
    es = extraAttr;
  // make string
  snprintf(buf, bufCnt, "<pos3d x=\"%.14g\" y=\"%.14g\" z=\"%.14g\" %s/>\n",
           x, y, z, es);
  return buf;
}

////////////////////////////////////////////

const char * UPosition::codeXml(const char * name, char * buff, const int buffCnt,
                               const char * extra)
{
  bool andName = false;
  const int MSL = 40000;
  char ns[MSL] = "";
  char * p1 = ns;
  int n;
  //
  if (name != NULL)
    andName = (strlen(name) > 0);
  if (andName)
    snprintf(p1, MSL, " name=\"%s\"", name);
  if (extra != NULL)
  {
    if (strlen(extra) > 0)
    {
      n = strlen(p1);
      p1 = &ns[n];
      snprintf(p1, MSL - n, " %s", extra);
    }
  }
  return codeXml(buff, buffCnt, ns);
}

////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////
// ROTATION
////////////////////////////////////////////

URotation::URotation() : Omega(0), Phi(0), Kappa(0)
{
}

////////////////////////////////////////////

URotation::URotation(double iOmega, double iPhi, double iKappa) :
Omega(iOmega), Phi(iPhi), Kappa(iKappa)
{
}

////////////////////////////////////////////

URotation::~URotation()
{}

////////////////////////////////////////////

UMatrix4 URotation::asVector3()
{
  UPosition pos;
  pos.copy(this);
  return pos.asVector3(true);
}

////////////////////////////////////////////

UMatrix4 URotation::asVector4()
{
  UPosition pos;
  pos.copy(this);
  return pos.asVector4(true);
}

////////////////////////////////////////////

UMatrix4 URotation::asUnitZVector3CtoW()
{
  UMatrix4 v1(1, 3, 0.0);
  UMatrix4 v2(3);
  // unit vector in -z direction
  v1.setAt(2, -1.0);
  v2 = this->asMatrix3x3CtoW() * v1;
  return v2;
}

////////////////////////////////////////////

UMatrix4 URotation::asUnitZVector4CtoW()
{
  UMatrix4 v2(3);
  //
  v2 = this->asUnitZVector3CtoW();
  v2.init(4, true);
  v2.setAt(3, 1.0);
  return v2;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3O()
{
  UMatrix4 o(3,3,1.0);
  o.setRC(1, 1, cos(Omega));
  o.setRC(2, 2, o.get(1, 1));
  o.setRC(1, 2, sin(Omega));
  o.setRC(2, 1, -o.get(1, 2));
  return o;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3Ot()
{
  UMatrix4 o(3,3,1.0);
  o.setRC(1, 1, cos(Omega));
  o.setRC(2, 2, o.get(1, 1));
  o.setRC(1, 2, -sin(Omega));
  o.setRC(2, 1, -o.get(1, 2));
  return o;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3P()
{
  UMatrix4 p(3,3,1.0);
  p.setRC(0, 0, cos(Phi));
  p.setRC(2, 2,  p.get(0, 0));
  p.setRC(0, 2, -sin(Phi));
  p.setRC(2, 0, -p.get(0, 2));
  return p;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3Pt()
{
  UMatrix4 p(3,3,1.0);
  p.setRC(0, 0, cos(Phi));
  p.setRC(2, 2,  p.get(0, 0));
  p.setRC(0, 2, sin(Phi));
  p.setRC(2, 0, -p.get(0, 2));
  return p;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3K()
{
  UMatrix4 k(3,3,1.0);
  k.setRC(0, 0, cos(Kappa));
  k.setRC(1, 1,  k.get(0, 0));
  k.setRC(0, 1,  sin(Kappa));
  k.setRC(1, 0,  -k.get(0, 1));
  return k;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3Kt()
{
  UMatrix4 k(3,3,1.0);
  k.setRC(0, 0, cos(Kappa));
  k.setRC(1, 1,  k.get(0, 0));
  k.setRC(0, 1,  -sin(Kappa));
  k.setRC(1, 0,  -k.get(0, 1));
  return k;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3RtoM()
{ // robot rotation is designed for azimuth (left right) (Kappa) before
  // up-down - elevation (Phi) before tilt left-right (Omega). i.e
  // Mpos = Kt * Pt * Ot * Rpos
  UMatrix4 result(3,3,1.0);
  result = asMatrix3x3Kt() *
           asMatrix3x3Pt() *
           asMatrix3x3Ot();
  return result;
}

////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3MtoR()
{ // robot rotation is designed for azimuth (left right) (Kappa) before
  // up-down - elevation (Phi) before tilt left-right (Omega). i.e
  // Rpos = O * P * K * Mpos
  UMatrix4 result(3,3,1.0);
  result = asMatrix3x3O() *
           asMatrix3x3P() *
           asMatrix3x3K();
  return result;
}

/////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4MtoR()
{ // resize to 4x4
  UMatrix4 R(3,3);
  //
  R = asMatrix3x3MtoR();
  R.expand(4,4, 1.0);
  return R;
}

/////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4RtoM()
{ // resize to 4x4
  UMatrix4 R(3,3);
  //
  R = asMatrix3x3RtoM();
  R.expand(4,4, 1.0);
  return R;
}

////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3CtoW()
{ // rotation matrix.
  // rotated position is then Xr = R * X
  // Phi   (P) is y-axis (rotate camera to the left is positive)
  // Omega (O) is x-axis (tilt camera up towards cealing is positive)
  // Kappa (K) is z axis (rotate camera ccv is positive)
  // Usage: from camera to world coordinates W = (T(-pos) *) P' * O' * K'
  UMatrix4 o(3,3,1.0);
  UMatrix4 p(3,3,1.0);
  UMatrix4 k(3,3,1.0);
  // x axis (Omega) transposed
  o.setRC(1, 1, cos(Omega));
  o.setRC(2, 2, o.get(1, 1));
  o.setRC(1, 2, -sin(Omega));
  o.setRC(2, 1, -o.get(1, 2));
  //o.transpose();
  // y axis (Phi) transposed
  p.setRC(0, 0, cos(Phi));
  p.setRC(2, 2,  p.get(0, 0));
  p.setRC(0, 2, sin(Phi));
  p.setRC(2, 0, -p.get(0, 2));
  //p.transpose();
  // z axis (Kappa) transposed
  k.setRC(0, 0, cos(Kappa));
  k.setRC(1, 1,  k.get(0, 0));
  k.setRC(0, 1,  -sin(Kappa));
  k.setRC(1, 0,  -k.get(0, 1));
  //k.transpose();
  return (p * o * k);
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix3x3WtoC()
{ // rotation matrix with first on Phi then Omega then Kappa
  // Phi   (P) is y-axis (panorate camera to the left is positive)
  // Omega (O) is x-axis (tilt camera up towards cealing is positive)
  // Kappa (K) is z axis (rotate camera ccv is positive)
  // usage: from world to camera coordinates K * O * P (* T)
  UMatrix4 o(3,3,1.0);
  UMatrix4 p(3,3,1.0);
  UMatrix4 k(3,3,1.0);
  // x axis (Omega)
  o.setRC(1, 1, cos(Omega));
  o.setRC(2, 2, o.get(1, 1));
  o.setRC(1, 2, sin(Omega));
  o.setRC(2, 1, -o.get(1, 2));
  // y axis (Phi)
  p.setRC(0, 0, cos(Phi));
  p.setRC(2, 2,  p.get(0, 0));
  p.setRC(0, 2, -sin(Phi));
  p.setRC(2, 0, -p.get(0, 2));
  // z axis (Kappa)
  k.setRC(0, 0, cos(Kappa));
  k.setRC(1, 1,  k.get(0, 0));
  k.setRC(0, 1,  sin(Kappa));
  k.setRC(1, 0,  -k.get(0, 1));
  return (k * o * p);
}

/////////////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4MtoR(UPosition * pos)
{ // Phi   (P) is y-axis (robot nose up or down down is positive)
  // Omega (O) is x-axis (tilt robot left or right, right is positive)
  // Kappa (K) is z axis (turn robot left or right, left is positive)
  // pos   (T) is is position of robot in world coordinates (meter)
  UMatrix4 T(4,4);
  UMatrix4 R(4,4);
  T = pos->asMatrix4x4();
  R = asMatrix4x4MtoR();
  return R * T;
}

/////////////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4RtoM(UPosition * pos)
{ // Phi   (P) is y-axis (robot nose up or down down is positive)
  // Omega (O) is x-axis (tilt robot left or right, right is positive)
  // Kappa (K) is z axis (turn robot left or right, left is positive)
  // pos   (T) is is position of robot in world coordinates (meter)
  UMatrix4 T(4,4);
  UMatrix4 R(4,4);
  UPosition posi = *pos;
  posi.scale(-1);
  T = posi.asMatrix4x4();
  R = asMatrix4x4RtoM();
  return T * R;
}

/////////////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4WtoC(UPosition * pos)
{ // rotation matrix with first on Phi then Omega then Kappa
  // Phi   (P) is y-axis (turn camera to the left is positive)
  // Omega (O) is x-axis (tilt camera up towards cealing is positive)
  // Kappa (K) is z axis (rotate camera ccv is positive) (radians)
  // pos   (T) is is position of camera in world coordinates (meter)
  // usage: from world to camera coordinates K * O * P * T
  // so: Xc = asMatrix4x4WtoC(&pos) * Xw
  UMatrix4 T(4,4);
  UMatrix4 R(4,4);
  T = pos->asMatrix4x4();
  R = this->asMatrix4x4WtoC();
  return R * T;
}

///////////////////////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4CtoW(UPosition * pos)
{ // rotation matrix with first on Phi then Omega then Kappa
  // Phi   (P) is y-axis (turn camera to the left is positive)
  // Omega (O) is x-axis (tilt camera up towards cealing is positive)
  // Kappa (K) is z axis (rotate camera ccv is positive) (radians)
  // pos   (T) is is position of camera in world coordinates (meter)
  // usage: from camera to world coordinates: T(-pos) * P' * O' * K'
  // so Xw = asMatrix4x4CtoW(&pos) * Xc
  UMatrix4 T(4,4);
  UMatrix4 R(4,4);
  UPosition posi = *pos;
  posi.scale(-1);
  T = posi.asMatrix4x4();
  R = this->asMatrix4x4CtoW();
  return T * R;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4CtoW()
{ // rotation matrix with first on Omega then Phi then Kappa
  // i.e.: R = Kappa * Phi * Omega
  // rotated position is then Xr = R * X
  // Kappa is z axis (rotate camera ccv is positive)
  // Omega is x-axis (tilt camera up towards cealing is positive)
  // Phi   is y-axis (rotate camera to the left is positive)
  UMatrix4 R(3,3);
  //
  R = asMatrix3x3CtoW();
  R.expand(4, 4, 1.0);
  return R;
}

////////////////////////////////////////////

UMatrix4 URotation::asMatrix4x4WtoC()
{ // Kappa is z axis (rotate camera ccv is positive)
  // Omega is x-axis (tilt camera up towards cealing is positive)
  // Phi   is y-axis (rotate camera to the left is positive)
  UMatrix4 R(3,3);
  //
  R = asMatrix3x3WtoC();
  R.expand(4,4, 1.0);
  return R;
}

////////////////////////////////////////////

URotation URotation::operator = (UMatrix4 vec)
{
  copy(&vec);
  return *this;
}

////////////////////////////////////////////

URotation URotation::operator+ (URotation rot)
{ // return this added to rot
  return added(&rot);
}

////////////////////////////////////////////

URotation URotation::operator- (URotation rot)
{ // return this subtracted by rot
  return subtracted(&rot);
}

////////////////////////////////////////////

URotation URotation::operator* (double val)
{ // return this scaled with val
  return scaled(val);
}

////////////////////////////////////////////

void URotation::operator+= (URotation rot)
{ // add rot
  add(rot);
}

////////////////////////////////////////////

void URotation::operator-= (URotation rot)
{ // subtract rot
  subtract(&rot);
}

////////////////////////////////////////////

void URotation::operator*= (double val)
{ // scale with a value
  scale(val);
}

////////////////////////////////////////////
/*
int URotation::copy(UMatrix4 * rot)
{ // copy value of size 3 or 4 Vector
  UPosition pos;
  int err;
  err = pos.copy(rot);
  if (err == 0)
  {
    Omega = pos.x;
    Phi   = pos.y;
    Kappa = pos.z;
  }
  return err;
}
*/
////////////////////////////////////////////

int URotation::copy(UMatrix4 * rot)
{ // copy value of size 3 or 4 Vector
  UPosition pos;
  int err;
  err = pos.copy(rot);
  if (err == 0)
  {
    Omega = pos.x;
    Phi   = pos.y;
    Kappa = pos.z;
  }
  return err;
}

////////////////////////////////////////////

int URotation::copy(UPosition * pos)
{ // copy
  Omega = pos->x;
  Phi   = pos->y;
  Kappa = pos->z;
  return 0;
}

////////////////////////////////////////////

void URotation::add(URotation * rot)
{ // add
  Omega += rot->Omega;
  Phi += rot->Phi;
  Kappa += rot->Kappa;
  LimitToPi();
}

////////////////////////////////////////////

void URotation::add(URotation rot)
{ // add tothis
  Omega += rot.Omega;
  Phi += rot.Phi;
  Kappa += rot.Kappa;
  LimitToPi();
}

////////////////////////////////////////////

URotation URotation::added(URotation * rot)
{ // return a version rotis added
  URotation R;
  R.Omega = Omega + rot->Omega;
  R.Phi = Phi + rot->Phi;
  R.Kappa = Kappa + rot->Kappa;
  R.LimitToPi();
  return R;
}

////////////////////////////////////////////

URotation URotation::subtracted(URotation * rot)
{ // subtract
  URotation R;
  R.Omega = Omega - rot->Omega;
  R.Phi = Phi - rot->Phi;
  R.Kappa = Kappa - rot->Kappa;
  R.LimitToPi();
  return R;
}

////////////////////////////////////////////

void URotation::subtract(URotation * rot)
{
  Omega -= rot->Omega;
  Phi   -= rot->Phi;
  Kappa -= rot->Kappa;
  LimitToPi();
}

////////////////////////////////////////////

void URotation::set(double iOmega, double iPhi, double iKappa)
{
  Omega = iOmega;
  Phi   = iPhi;
  Kappa = iKappa;
  LimitToPi();
}

/////////////////////////////////////////////

void URotation::add(double iOmega, double iPhi, double iKappa)
{ // add these values
  Omega += iOmega;
  Phi += iPhi;
  Kappa += iKappa;
  LimitToPi();
}

///////////////////////////////////////////////

void URotation::clear(void)
{
  set(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////

void URotation::LimitToPi()
{ // limit to range
  // more than pi
  while (Omega >= M_PI)
    Omega -= 2 * M_PI;
  while (Phi >= M_PI)
    Phi -= 2 * M_PI;
  while (Kappa >= M_PI)
    Kappa -= 2 * M_PI;
  // less than -pi
  while (Omega < -M_PI)
    Omega += 2 * M_PI;
  while (Phi < -M_PI)
    Phi += 2 * M_PI;
  while (Kappa < -M_PI)
    Kappa += 2 * M_PI;
}

////////////////////////////////////////////

URotation URotation::scaled(double val)
{ // return a scaled copy
  URotation R;
  R.Omega = Omega * val;
  R.Phi   = Phi * val;
  R.Kappa = Kappa * val;
//  R.LimitToPi();
  return R;
}

////////////////////////////////////////////

void URotation::scale(double val)
{ // scale this
  Omega *= val;
  Phi *= val;
  Kappa *= val;
//  LimitToPi();
}

////////////////////////////////////////////

void URotation::print(const char * leadString)
{ // in radians only
  printf("%s  O:%9.4f  P:%9.4f  K:%9.4f\n", leadString, Omega, Phi, Kappa);
}

////////////////////////////////////////////

void URotation::sprint(char * s, const char * leadString, bool inDegree /*= true*/)
{ // in radians or degrees
  if (inDegree)
    sprintf(s, "%s  O:%9.4f  P:%9.4f  K:%9.4f (deg)",
          leadString, Omega*180.0/M_PI, Phi*180.0/M_PI,
          Kappa*180.0/M_PI);
  else
    sprintf(s, "%s  O:%9.4f  P:%9.4f  K:%9.4f (rad)",
          leadString, Omega, Phi, Kappa);
}

////////////////////////////////////////////

void URotation::snprint(const char * leadString,
                        bool inDegree, char * s, const int bufferLength)
{ // in radians or degrees
  if (inDegree)
    snprintf(s, bufferLength, "%s  O:%9.4f  P:%9.4f  K:%9.4f (deg)\n",
          leadString, Omega*180.0/M_PI, Phi*180.0/M_PI,
          Kappa*180.0/M_PI);
  else
    snprintf(s, bufferLength, "%s  O:%9.4f  P:%9.4f  K:%9.4f (rad)\n",
          leadString, Omega, Phi, Kappa);
}

////////////////////////////////////////////

int URotation::SaveToReg(Uconfig * ini, const char * subject, const char * key)
{ // to specified configuration file
  UPosition p;
  p.copy(this);
  return p.SaveToReg(ini, subject, key);
}


////////////////////////////////////////////

int URotation::LoadFromReg(Uconfig * ini, const char * subject, const char * key)
{ // from specified configuration file
  UPosition p;
  int err;
  err = p.LoadFromReg(ini, subject, key);
  if (err==0)
    copy(&p);
  else
  {
    Omega=0.0;
    Phi=0.0;
    Kappa=0.0;
  }
  return err;
}

////////////////////////////////////////////

bool URotation::save(FILE * fmap, const char * key)
{
  bool result = (fmap != NULL);
  //
  fprintf(fmap, "<%s=%e,%e,%e>", key,
    Omega, Phi, Kappa);
  //
  return result;
}

////////////////////////////////////////////

bool URotation::load(const char * valueString)
{
  bool result;
  int n;
  //
  n = sscanf(valueString, "%le,%le,%le", &Omega, &Phi, &Kappa);
  result = (n == 3);
  //
  return result;
}

///////////////////////////////////////////////////////

void URotation::setFromYZ(UPosition posY, UPosition posZ)
{
  UPosition px, pz;
  UMatrix4 mR;
  //
  //make normal vector to YZ plane
  px = posY.cross(posZ);
  // make it a unit vector
  px.toUnitVector();
  // azimuth orientation is then the vector in
  // x-y coordinates
  Kappa = atan2(px.y, px.x);
  // the elevation (+/- Pi/2) is found
  // from the height of the vector (positive is down)
  Phi = asin(-px.z);
  // the Z vector is then rotated with
  // the found azimuth and elevation
  Omega = 0.0;
  px.clear();
  mR = asMatrix4x4MtoR(&px);
  pz = mR * posZ;
  // omega is then the Z vector seen in the YZ plane
  // positive 1q if y<0 and z>0
  Omega = atan2(-pz.y, pz.z);
}

//////////////////////////////////////////////////////

const char * URotation::codeXml(char * buff, const int buffCnt, const char * extra)
{
  const char * es = "";
  //
  if (extra != NULL)
    es = extra;
  // make string
  snprintf(buff, buffCnt, "<rot3d Omega=\"%.14g\" Phi=\"%.14g\" Kappa=\"%.14g\" %s/>\n",
           Omega, Phi, Kappa, es);
  return buff;
}

////////////////////////////////////////////////////

const char * URotation::codeXml(const char * name, char * buff, const int buffCnt, const char * extra)
{
  bool andName = false;
  const int MSL = 40000;
  char ns[MSL] = "";
  char * p1 = ns;
  int n;
  //
  if (name != NULL)
    andName = (strlen(name) > 0);
  if (andName)
    snprintf(p1, MSL, " name=\"%s\"", name);
  if (extra != NULL)
  {
    if (strlen(extra) > 0)
    {
      n = strlen(p1);
      p1 = &ns[n];
      snprintf(p1, MSL - n, " %s", extra);
    }
  }
  return codeXml(buff, buffCnt, ns);
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

int DoFindPeak(     const double x1, const double y1,
                    const double x2, const double y2,
                    const double x3, const double y3,
                    double * xr, double * yr)
{ // finds the peak, by matrix evaluation of
  // A x X = B
  // The three (x,y) points is fitted on
  // a second order curve d = ax + bx*x + c
  // the x-value of the peak is returned in xr,
  // and y-value at that peay is returned in yr.
  // dy/dx = a + 2*b*x = 0 => x=-a/(2*b)
  // if no solution then err < 0
  // if peak is a max, then err = +1.
  bool result;
  UMatrix4 A(3, 3);
  UMatrix4 X(3);
  UMatrix4 B(3, 1);
  //
  A.setRow(0, x1, sqr(x1),  1.0);
  A.setRow(1, x2, sqr(x2),  1.0);
  A.setRow(2, x3, sqr(x3),  1.0);
  B.set(y1, y2, y3);
  //
  result = A.solve(&B, &X);
  if (result and (absd(X.get(1))> 1e-15))
  {
    *xr = -X.get(0)/(X.get(1)*2.0);
    *yr = X.get(0) * *xr + X.get(1) * sqr(*xr) + X.get(2);
    if (*yr >= y1)
      result = false;
  }
  return not result;
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////////////////////////////////////////////////////

UOriPlane::UOriPlane()
{
  clear();
}

////////////////////////////////////////////////////////

UOriPlane::~UOriPlane()
{}

////////////////////////////////////////////////////////

void UOriPlane::clear()
{
  poso.set(0.0, 0.0, 0.0);
  posx.set(1.0, 0.0, 0.0);
  posy.set(0.0, 1.0, 0.0);
}

////////////////////////////////////////////////////////

void UOriPlane::set(UPosition pos, URotation rot)
{
  UMatrix4 mT;
  //
  clear();
  mT = rot.asMatrix4x4RtoM(&pos);
  poso = pos;
  posx = posx.transferred(&mT) - pos;
  posy = posy.transferred(&mT) - pos;
}

////////////////////////////////////////////////////////

void UOriPlane::print(const char * prestring)
{
  UPosition pz;
  //
  pz = posx.cross(posy);
  poso.print(prestring);
  posx.print("   x-vec");
  posy.print("   y-vec");
  pz.print("   z-vec");
}

/////////////////////////////////////////////////////

void UOriPlane::norm()
{ // normalize vectors
  posx.toUnitVector();
  posy.toUnitVector();
}



//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////

