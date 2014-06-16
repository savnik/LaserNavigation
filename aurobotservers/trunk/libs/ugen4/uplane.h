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
#ifndef UPLANE_H
#define UPLANE_H

#include "u3d.h"
#include "uline.h"

/**
 * General definition of a 3D plane, defined bu ax + by +cz + d = 0.
 * The normal vector (a,b,c) is assumed (and set) to be a unit vector, and d will be positive or zero
 * (The distance to the plane (from origo) is then d)
 * to ease distance calculations.
 * @author Christian Andersen <jca@oersted.dtu.dk> */
class UPlane
{
public:
  /**
   * Constrctor */
  UPlane();
  /**
   * Destructor */
  ~UPlane();
  /**
   * clear sets the plane to span the x-y axis zeros. */
  void clear()
  { // a plane that spans the x,y axes
    a = 0.0;
    b = 0.0;
    c = 1.0;
    d = 0.0;
  }
  /**
   * Set the plane from three 3D positions.
   * Using a = A1, b=A2, c = A3, d = A,
   * and Ai = det(x1, x2, x3), where x1 is p1 coulumn, x2 is x2 column x3 is p3 column,
   * except for column i wich is set to [1 1 1]'.
   * A uses the full matrix.
   * \param p1 is a 3d position on plane
   * \param p2 is a 3d position on plane
   * \param p3 is a 3d position on plane
   * All positions must be different, else the result will be unpredicted, and isValid() will return false. */
  void set(UPosition p1, UPosition p2, UPosition p3);
  /**
   * Print result to a string */
  const char * print(const char * preStr, char * buff, const int buffCnt);
  /**
   * Set plane from a position and a normal vector.
   * \param p1 is a 3d position on the plane
   * \param n1 is a 3d normal vector (need not be a unit vector)
   * normal vector must not be a null vector (else unpredicted result). */
  void set(UPosition p1, UPosition n1);
  /**
   * Set plane from the position on the plane closest to origin.
   * \param p1 is a the closest 3d position on the plane that is closest to the origin
   * the position must not be a null vector (else unpredicted result). */
  void set(UPosition p1);
  /**
   * Set plane from the position on the plane closest to the origin.
   * \param p1 is the 3d position on the plane that is closest to the origin
   * the position must not be a null vector (else unpredicted result). */
  inline void set(double x, double y, double z)
  {
    UPosition v(x,y,z);
    set(v);
  };
  /**
   * Adjust plane parameters to make (a,b,c) a unit vector and 'd' a positive value. */
  void normalize();
  /**
   * Set plane values direct, the a,b,c must not form a null vector.
   * \param a,b,c,d is the plane parameters (will be normalixed to have a,b,c as a unit vector)
   * normal vector must not be a null vector (else unpredicted result). */
  inline void set(double ia, double ib, double ic, double id)
  { a=ia; b=ib; c=ic; d=id; normalize(); };
  /**
   * The plane is defined valid, if the normal vector defined as (a,b,c) has length 1.
   * \returns true if valid plane. */
  bool isValid()
  { return (fabs(sqr(a) + sqr(b) + sqr(c) - 1.0) < 1e-10); };
  /**
   * Get normal vector */
  UPosition getNormal()
  {
    UPosition result(a, b, c);
    return result;
  }
  /**
   * \brief Get signed distance from point to plane.
   * The distance is positive if the position is at the origin-side of the plane
   * \param p1 is the 3d point
   * \return distance to point */
  inline double distSigned(UPosition p1)
  { return a*p1.x + b*p1.y + c*p1.z + d; };
  /**
   * \brief Get distance from point to plane.
   * \param p1 is the 3d point
   * \return distance to point */
  inline double dist(UPosition p1)
  { return fabs(distSigned(p1)); };
  /**
   * Get point on plane nearest to a point off the plane.
   * \param p1 is the position off the plane.
   * */
  UPosition getOnPlane(UPosition p1);
  /**
   * Get position on plane, where the line is crossing the plane.
   * \param crossingLine is the line that is assumed to cross the line
   * \param notParallel is true if the crossing is OK, otherwise the point is far away, in the right
   * direction and on the line, but not on the plane.
   * \returns the common point on the line and plane. */
  UPosition getLineCrossing(ULine crossingLine, bool * notParallel);
  /**
   * Get the line from intersection of two planes.
   * \param plane2 is the second plane.
   * \param notParallel is true if the result is OK.
   * \returns the intersection line. */
  ULine getPlaneCrossing(UPlane plane2, bool * notParallel);

  
public:
  /** parameter values for a general plane */
  double a, b, c, d;
};

#endif
