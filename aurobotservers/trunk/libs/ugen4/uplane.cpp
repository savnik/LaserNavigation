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
#include "uplane.h"

///////////////////////////////////

UPlane::UPlane()
{
  clear();
}

///////////////////////////////////

UPlane::~UPlane()
{
}

///////////////////////////////////

void UPlane::set(UPosition p1, UPosition p2, UPosition p3)
{
  UPosition n1;
  //
  n1.cross(p2 - p1, p3 - p1);
  // normalize normal vector
  n1.toUnitVector();
  // set normalized parameters
  a = n1.x;
  b = n1.y;
  c = n1.z;
  // calculate d from one of the points
  d = -a*p1.x - b*p1.y - c*p1.z;
  if (d < 0.0)
  {
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }
}

///////////////////////////////////

void UPlane::set(UPosition p1, UPosition n1)
{ // normalize normal vector
  n1.toUnitVector();
  // set normalized parameters
  a = n1.x;
  b = n1.y;
  c = n1.z;
  // calculate d from one of the points
  d = -a*p1.x - b*p1.y - c*p1.z;
  if (d < 0.0)
  {
    a = -a;
    b = -b;
    c = -c;
    d = -d;
  }
}

///////////////////////////////////

void UPlane::set(UPosition p1)
{ // normalize normal vector
  UPosition n1 = p1;
  //
  n1.toUnitVector();
  // set normalized parameters
  a = -n1.x;
  b = -n1.y;
  c = -n1.z;
  d = p1.dist();
}

///////////////////////////////////

void UPlane::normalize()
{ // scale values, so that (a,b,c) is a unit vector and d is positive
  double u = sqrt(sqr(a) + sqr(b) + sqr(c));
  //
  if (u > 1e-50)
    u = 1.0/u;
  if (d < 0.0)
    u = -u;
  a *= u;
  b *= u;
  c *= u;
  d *= u;
}

///////////////////////////////////

const char * UPlane::print(const char * preStr, char * buff, const int buffCnt)
{
  snprintf(buff, buffCnt, "%sa=%g b=%g c=%g d=%g\n", preStr, a, b, c, d);
  return buff;
}

///////////////////////////////////

UPosition UPlane::getOnPlane(UPosition p1)
{
  UPosition n1(a,b,c); // normalized normal
  // the point must be distance times the normal vector away from the point.
  return p1 - n1 * distSigned(p1);
}

///////////////////////////////////

UPosition UPlane::getLineCrossing(ULine crossingLine, bool * notParallel)
{ // this is the plane
  double nom, denom;
  UPosition P = crossingLine.pos;
  UPosition V = crossingLine.vec;
  bool isOK;
  UPosition result;
  double t;
  // now line in parameter form is [X] = [P] + t[V]
  // [X] (=[x,y,z]) are inserted into plane and t is found.
  // t = -(A Px + B Py + C Pz + D)/(A Vx + B Vy + C Vz);
  denom = a * V.x + b * V.y + c * V.z;
  isOK = absd(denom) > 1e-40;
  if (isOK)
  {
    nom = -(a * P.x + b * P.y + c * P.z + d);
    t = nom/denom;
    result = crossingLine.getPositionOnLine(t);
  }
  else
    result = crossingLine.getPositionOnLine(1e30);
  if (notParallel != NULL)
    *notParallel = isOK;
  return result;
}

//////////////////////////////////////

ULine UPlane::getPlaneCrossing(UPlane plane2, bool * notParallel)
{
  UPosition n1(a,b,c);
  UPosition n2;
  UMatrix4 mA(2,2); // left side matrix
  UMatrix4 mB(2,1); // right side vector
  UMatrix4 mX(2,1); // solution
  int i,j,m;
  ULine result;
  bool isOK = true;
  //
  n2 = plane2.getNormal();
  // the line orientation vector is easy
  result.vec = n1.cross(n2);
  // vector needs to be normalized - to folow definition of line
  result.normalize();
  // now to a position on the line
  // the line will xross either x=0 or y=0 or z=0
  if (fabs(result.vec.x) > 0.3)
    // x is set to 0, as line vector
    // will cross x=0 at some point
    m = 0;
  else if (fabs(result.vec.y) > 0.2)
    m = 1; // y is set to 0
  else if (fabs(result.vec.z) > 0.2)
    m = 2; // set z to null
  else
    isOK = false;
  if (isOK)
  {
    j = 0;
    // mA mX = mB
    for (i = 0; i < 3; i++)
    { // make equation (A matrix) from remaining vectors
      if (m != i)
      { // set 2 values only
        mA.setRC(0, j, n1.get(i));
        mA.setRC(1, j, n2.get(i));
        j++;
      }
    }
    mB.set(-d, -plane2.d);
    mA.solve(&mB, &mX);
    //
    j = 0;
    result.pos.clear();
    for (i = 0; i < 3; i++)
    { // return result into line position
      if (m != i)
      { // set remining 2 values
        result.pos.set(i, mX.get(j));
        j++;
      }
    }
  }
  if (notParallel != NULL)
    *notParallel = isOK;
  //
  return result;
}

