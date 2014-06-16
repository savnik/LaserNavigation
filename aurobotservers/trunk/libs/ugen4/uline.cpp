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

#include "uline.h"

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
// ULine

ULine::ULine()
{
  clear();
}

////////////////////////////////////////////////////////

ULine::ULine(double Px, double Py, double Pz,
      double Vx, double Vy, double Vz)
{
  pos.x = Px;
  pos.y = Py;
  pos.z = Pz;
  vec.x = Vx;
  vec.y = Vy;
  vec.z = Vz;
  vec.toUnitVector();
}


////////////////////////////////////////////////////////

ULine::~ULine()
{}

////////////////////////////////////////////////////////

void ULine::clear()
{ // clear line info
  pos.clear();
  vec.clear();
}

////////////////////////////////////////////////////////

bool ULine::isValid()
{ // line is valid if 'vec' vector is 1.0.
  return (absd(1.0 - vec.dist()) < 1e-4);
}

////////////////////////////////////////////////////////

double ULine::getDistanceSq(UPosition * point)
{ // Get distance from this line to point.
  double t, dE, dH, dN, result;
  // parameter line: [e,h,n] = [A,B,C]*t + [e0,h0,n0];
  // find point on parameter line closest to point
  // (assumes [A,B,C] is a unit vector).
  t = getPositionOnLine(point);
  // get vector from line to point.
  dE = vec.x * t + pos.x - point->x;
  dH = vec.y * t + pos.y - point->y;
  dN = vec.z * t + pos.z - point->z;
  // get distance
  result = sqr(dE) + sqr(dH) + sqr(dN);
  return result;
}

////////////////////////////////////////////////////////

double ULine::getPositionOnLine(UPosition * point)
{ // Get get parameter value for nearest position on
  // line to this point.
  // parameter is the 't' in line equation [x,y,z] = vec * t + pos.
  // assumed that 'vec' is a unit vector
  // Returns t value
  return -(vec.x * (pos.x - point->x) +
           vec.y * (pos.y - point->y) +
           vec.z * (pos.z - point->z));
}

////////////////////////////////////////////////////////

UPosition ULine::getPositionOnLine(const double t)
{ // Get position with this parameter value.
  // Given a 't' value for a parametized line [x,y,z] = vec * t + pos,
  // The [x,y,z] position is returned.
  UPosition result;
  //
  result.x = vec.x * t + pos.x;
  result.y = vec.y * t + pos.y;
  result.z = vec.z * t + pos.z;
  //
  return result;
}

////////////////////////////////////////////////////////

double ULine::getPositionOnLineXY(UPosition * point)
{ // Get get parameter value for nearest position on
  // line to this point.
  // parameter is the 't' in line equation [x,y] = vec * t + pos.
  // assumed that 'vec' is a unit vector
  // Returns t value
  return -(vec.x * (pos.x - point->x) +
           vec.y * (pos.y - point->y));
}

////////////////////////////////////////////////////////

UPosition ULine::getPositionOnLineXY(const double t)
{ // Get position with this parameter value.
  // Given a 't' value for a parametized line [x,y,z] = vec * t + pos,
  // The [x,y,z] position is returned.
  UPosition result;
  //
  result.x = vec.x * t + pos.x;
  result.y = vec.y * t + pos.y;
  result.z = 0.0;
  //
  return result;
}

///////////////////////////////////////////////////////////

void ULine::setFromPoints(const UPosition * pos1, const UPosition * pos2)
{ // create 3D line from 2 points
  pos = *pos1;
  vec.x = pos2->x - pos1->x;
  vec.y = pos2->y - pos1->y;
  vec.z = pos2->z - pos1->z;
  vec.toUnitVector();
}

///////////////////////////////////////////////////////////

void ULine::setFromPoints(UPosition pos1, UPosition pos2)
{ // create 3D line from 2 points
  pos = pos1;
  vec.x = pos2.x - pos1.x;
  vec.y = pos2.y - pos1.y;
  vec.z = pos2.z - pos1.z;
  vec.toUnitVector();
}

///////////////////////////////////////////////////////////

void ULine::setFromPoints(double x1, double y1, double z1, double x2, double y2, double z2)
{ // create 3D line from 2 points
  pos.set(x1, y1, z1);
  vec.x = x2 - x1;
  vec.y = y2 - y1;
  vec.z = z2 - z1;
  vec.toUnitVector();
}

///////////////////////////////////////////////////////////

void ULine::set(UPosition iPos, UPosition iVec)
{
  pos = iPos;
  vec = iVec;
  vec.toUnitVector();
}

///////////////////////////////////////////////////////////

void ULine::set2D(double x, double y, double heading)
{
  pos.x = x;
  pos.y = y;
  pos.z = 0.0;
  vec.x = cos(heading);
  vec.y = sin(heading);
  vec.z = 0.0;
}

///////////////////////////////////////////////////////////

void ULine::show(const char * prestring /* = NULL*/)
{ // show values to console
  if (prestring != NULL)
    printf("%s:\n", prestring);
  else
    printf("3DLine:\n");
  pos.show(" - pos");
  vec.show(" - vec");
}

///////////////////////////////////////////////////////////

void ULine::snprint(const char * prestring, char * buff, const int buffCnt)
{
  snprintf(buff, buffCnt, "%s pos(%g, %g, %g) vec(%g, %g, %g)\n",
           prestring, pos.x, pos.y, pos.z, vec.x, vec.y, vec.z);
}

///////////////////////////////////////////////////////////

UPosition ULine::getPlaneLineCrossing(ULine crossingLine, bool * notParallel)
{ // this is the plane
  double A, B, C, D;
  double nom, denom;
  UPosition P = crossingLine.pos;
  UPosition V = crossingLine.vec;
  bool isOK;
  UPosition result;
  double t;
  // get plane in Ax + By + Cz + D = 0 form
  A = -vec.x;
  B = -vec.y;
  C = -vec.z;
  D = vec.x * pos.x + vec.y * pos.y + vec.z * pos.z;
  // now line in parameter form is [X] = [P] + t[V]
  // [X] (=[x,y,z]) are inserted into plane and t is found.
  // t = -(A Px + B Py + C Pz + D)/(A Vx + B Vy + C Vz);
  denom = A * V.x + B * V.y + C * V.z;
  isOK = absd(denom) > 1e-40;
  if (isOK)
  {
    nom = -(A * P.x + B * P.y + C * P.z + D);
    t = nom/denom;
    result = crossingLine.getPositionOnLine(t);
  }
  else
    result = crossingLine.getPositionOnLine(1e30);
  if (notParallel != NULL)
    *notParallel = isOK;
  return result;
}

///////////////////////////////////////////////////////////

int ULine::getCylinderCrossings(UPosition center, double r,
                          double * t1, double * t2)
{ // calculated from
  // ((Px + Vx * t) - Cx)^2 + ((Px + Vx * t) - Cx)^2 = r^2
  // and solve this 2nd order equation
  // (Vx� + Vy�)t� + 2*((Px-Cx)Vx + (Py-Cy)Vy)t + (Px-Cx)� + (Py - Cy)� - r� = 0
  double A, B, C;
  double kx, ky; // difference from line origin to circle center
  double d; // determinant
  int result;
  //
  A = sqr(vec.x) + sqr(vec.y);
  kx = pos.x - center.x;
  ky = pos.y - center.y;
  B = 2.0 * (kx * vec.x + ky * vec.y);
  C = sqr(kx) + sqr(ky) - sqr(r);
  d = sqr(B) - 4.0 * A * C;
  if (d < 0.0)
    result = 0;
  else
  {
    result = 2;
    d = sqrt(d);
    *t1 = (-B + d)/(2.0 * A);
    *t2 = (-B - d)/(2.0 * A);
  }
  return result;
}

///////////////////////////////////////////////////////////

int ULine::getSphereCrossings(UPosition center, double r,
                          double * t1, double * t2)
{ // calculated from
  // ((Px + Vx * t) - Cx)^2 + ((Px + Vx * t) - Cx)^2 = r^2
  // define kx = Px - Cx;
  // define ky = Py - Cy;
  // define kz = Pz - Cz;
  // and solve this 2nd order equation
  // (Vx� + Vy� + Vz�)t� + 2*(kx*Vx + ky*Vy + kz*Vz)t + kx� + ky� + kz� - r� = 0
  double A, B, C;
  double kx, ky, kz; // difference from line origin to circle center
  double d; // determinant
  int result;
  //
  A = sqr(vec.x) + sqr(vec.y) + sqr(vec.z);
  kx = pos.x - center.x;
  ky = pos.y - center.y;
  kz = pos.z - center.z;
  B = 2.0 * (kx * vec.x + ky * vec.y + kz * vec.z);
  C = sqr(kx) + sqr(ky) + sqr(kz) - sqr(r);
  d = sqr(B) - 4.0 * A * C;
  if (d < 0.0)
    result = 0;
  else
  {
    result = 2;
    d = sqrt(d);
    *t1 = (-B + d)/(2.0 * A);
    *t2 = (-B - d)/(2.0 * A);
  }
  return result;
}

///////////////////////////////////////////////////////////

bool ULine::getXYCrossing(ULine other, UPosition * crossingXY)
{
  U2Dlined l1, l2;
  double x, y;
  bool result;
  //
  l1.setPV(pos.x, pos.y, vec.x, vec.y);
  l2.setPV(other.pos.x, other.pos.y, other.vec.x, other.vec.y);
  result = l1.getCrossing(l2, &x, &y);
  if (crossingXY != NULL)
    crossingXY->set(x, y, 0.0);
  return result;
}

///////////////////////////////////////////////////////////

double ULine::getXYsignedDistance(UPosition toPos)
{
  U2Dlined lin2d;
  double d;
  //
  lin2d.setPV(pos.x, pos.y, vec.x, vec.y);
  d = lin2d.distanceSigned(toPos.x, toPos.y);
  //
  return d;
}

///////////////////////////////////////////////////////////

void ULine::turn90degXY(bool left)
{
  double a = vec.x;
  // swap to normal vector
  vec.x = vec.y;
  vec.y = a;
  // change sign on one of them
  if (left)
    vec.x = -vec.x;
  else
    vec.y = -vec.y;
}

///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////

void ULineSegment::clear()
{
  ULine::clear();
  length = 0.0;
}

///////////////////////////////////////////////////////////

void ULineSegment::setFromPoints(const UPosition * pos1, const UPosition * pos2)
{
  ULine::setFromPoints(pos1, pos2);
  length = sqrt(sqr(pos1->x - pos2->x) + sqr(pos1->y - pos2->y) + sqr(pos1->z - pos2->z));
}

///////////////////////////////////////////////////////////

void ULineSegment::setFromPoints(double x1, double y1, double z1, double x2, double y2, double z2)
{
  ULine::setFromPoints(x1, y1, z1, x2, y2, z2);
  length = sqrt(sqr(x2 - x1) + sqr(y2 - y1) + sqr(z2 - z1));
}

///////////////////////////////////////////////////////////

void ULineSegment::setFromPoints(UPosition pos1, UPosition pos2)
{
  ULine::setFromPoints(pos1, pos2);
  length = pos1.dist(pos2);
}

///////////////////////////////////////////////////////////

void ULineSegment::show(const char * prestring)
{ // show values to console
  if (prestring != NULL)
    ULine::show(prestring);
  else
    ULine::show("3D line segment");
  printf(" - length %e\n", length);
}

////////////////////////////////////////////////////////

void ULineSegment::snprint(const char * prestring, char * buff, const int buffCnt)
{
  char *p1;
  int n;
  ULine::snprint(prestring, buff, buffCnt);
  p1 = strrchr(buff, '\n');
  if (p1 != NULL)
  {
    n = p1 - buff;
    snprintf(p1, buffCnt - n, " length %g\n", length);
  }
}

///////////////////////////////////////////////////////////

double ULineSegment::getDistanceXYSigned(UPosition point, int * where)
{
  U2Dlined linxy;
  UPosition p1, p2;
  double dist;
  double t;
  int w;
  ULineSegment seg;
  //
  p1 = pos;
  p1.z = 0.0;
  p2 = getOtherEnd();
  p2.z = 0.0;
  linxy.set2P(pos.x, pos.y, p2.x, p2.y);
  dist =  linxy.distanceSigned(point.x, point.y);
  w = 0;
  // get closest point on line as line parameter t
  seg.setFromPoints(p1, p2);
  p1 = point;
  p1.z = 0.0;
  t = seg.getPositionOnLine(&p1);
  if (t <= 0.0)
  { // closest to ref point
    dist = hypot(pos.x - point.x, pos.y - point.y) * signofd(dist);
    w = 1; // to end 1
  }
  else if (t > length)
  { // closest to other end
    dist = hypot(p2.x - point.x, p2.y - point.y) * signofd(dist);
    w = 2; // to end 2
  }
  if (where != NULL)
    *where = w;
  return dist;
}

///////////////////////////////////////////////////////////

double ULineSegment::getDistanceFromSegSq(UPosition point, int * where /* = NULL */)
{
  double t;
  double distSq;
  UPosition p2;
  int w;
  //
  t = getPositionOnLine(&point);
  if (t < 0)
  { // closest to ref point
    distSq = pos.distSq(&point);
    w = 1; // to end 1
  }
  else if (t > length)
  { // closest to other end
    p2 = getPositionOnLine(length);
    distSq = p2.distSq(&point);
    w = 2; // to end 2
  }
  else
  { // closest to point on line segment
    p2 = getPositionOnLine(t);
    distSq = p2.distSq(&point);
    w = 0; // to line segment
  }
  if (where != NULL)
    *where = w;
  return distSq;
}

/////////////////////////////////////////////////////

bool ULineSegment::getSegmentCrossingXY(ULineSegment * other, UPosition * crossing)
{
  bool result;
  double t;
  UPosition a, b;
  //
  result = getXYCrossing(*other, crossing);
  if (result)
  { // is crossing inside this line
    t = getPositionOnLineXY(crossing);
    result = (t >= 0) and (t <= length);
  }
  if (result)
  { // is crossing inside other line
    t = other->getPositionOnLineXY(crossing);
    result = (t >= 0) and (t <= other->length);
  }
//   if (result)
//   { // crossing is inside this x,y line segment
//     b = getOtherEnd();
//     result = crossing->x >= fmin(b.x, pos.x) and crossing->x <= fmax(b.x, pos.x) and
//              crossing->y >= fmin(b.y, pos.y) and crossing->y <= fmax(b.y, pos.y);
// /*    t = getPositionOnLine(crossing);
//     result = (t >= 0) and (t <= length);*/
//   }
//   if (result)
//   { // is crossing inside other x,y segment
//     a = other->pos;
//     b = other->getOtherEnd();
//     result = crossing->x >= fmin(b.x, a.x) and crossing->x <= fmax(b.x, a.x) and
//              crossing->y >= fmin(b.y, a.y) and crossing->y <= fmax(b.y, a.y);
// /*    t = other->getPositionOnLine(crossing);
//     result = (t >= 0) and (t <= other->length);*/
//   }
  return result;
}
