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

#ifndef ULINE_H
#define ULINE_H

#include "u2dline.h"
#include "u3d.h"

/**
  * @author Christian Andersen
  * This is a general purpose class for 3D lines and 3D planes, with a few simple
  * functions to create and use the line.
  */
class ULine : public UDataBase
{
public:
  /**
  Constructor */
  ULine();
  /**
  Constructr from position and vector */
  ULine(double Px, double Py, double Pz,
        double Vx, double Vy, double Vz);
  /**
  Destructor */
  virtual ~ULine();
  /**
  Get (end) type string for this structure */
  virtual const char * getDataType()
  {
    return "line";
  };
  /**
  Clear */
  virtual void clear();
  /**
  Test to if line is valid. Line is valid if length of vector is 1.0. */
  bool isValid();
  /**
  Get distance from this line to point. */
  inline double getDistance(UPosition * point)
  {
    return sqrt(getDistanceSq(point));
  }
  /**
  Get distance squared from this line to point. */
  double getDistanceSq(UPosition * point);
  /**
  Get distance squared from this line to point. */
  inline double getDistanceSq(UPosition point)
  { return getDistanceSq(&point); };
  /**
  Get get parameter value for nearest position on
  line to this point.
  parameter is the 't' in line equation [x,y,z] = vec * t + pos.
  Returns t value */
  double getPositionOnLine(UPosition * point);
  /**
  Get get parameter value for nearest position on
  line to this point.
  parameter is the 't' in line equation [x,y,z] = vec * t + pos.
  Returns t value */
  inline double getPositionOnLine(UPosition point)
  { return getPositionOnLine(&point); };
  /**
  Get position with this parameter value.
  Given a 't' value for a parametized line [x,y,z] = vec * t + pos,
  The [x,y,z] position is returned. */
  UPosition getPositionOnLine(const double t);
  /**
  Get get parameter value for nearest position on
  line to this point.
  parameter is the 't' in line equation [x,y,z] = vec * t + pos.
  Returns t value */
  double getPositionOnLineXY(UPosition * point);
  /**
  Get get parameter value for nearest position on
  line to this point.
  parameter is the 't' in line equation [x,y,z] = vec * t + pos.
  Returns t value */
  inline double getPositionOnLineXY(UPosition point)
  { return getPositionOnLineXY(&point); };
  /**
  Get position with this parameter value.
  Given a 't' value for a parametized line [x,y,z] = vec * t + pos,
  The [x,y,z] position is returned. */
  UPosition getPositionOnLineXY(const double t);
  /**
  Set line from two points.
  Position pos1 is used as base point and vector
  is set pointing towards pos2. */
  virtual void setFromPoints(const UPosition * pos1, const UPosition * pos2);
  /**
  Set line from two points.
  Position pos1 is used as base point and vector
  is set pointing towards pos2. */
  virtual void setFromPoints(UPosition pos1, UPosition pos2);
  /**
   * Set line from points. a line is a start point and a direction vector.
   * \param x1,y1,z1  defines the first (start) point
   * \param x2,y2,z2 defines the end point */
  void setFromPoints(double x1, double y1, double z1,
                     double x2, double y2, double z2);
  /**
  Set from a position and a vector */
  void set(UPosition iPos, UPosition iVec);
  /**
  Set line from 2D (x,y,th) information.
  Other values are set to zero. */
  void set2D(double x, double y, double heading);
  /**
  Show values of line to console */
  void show(const char * prestring = NULL);
  inline void print(const char * prestring = NULL)
  { show(prestring); };
  /**
  Print values of line to string */
  virtual void snprint(const char * prestring, char * buff, const int buffCnt);
  /**
  Get position on this plane, where the line 'crossingLine' is
  crossing. This is a plane with a point 'pos' and a normal vector 'vec'.
  If the line is not too close to parallel 'notParallel' is true and
  crossing point is returned. Otherwise a very large position in direction
  of the line vector is returned. */
  UPosition getPlaneLineCrossing(ULine crossingLine, bool * notParallel = NULL);
  /**
  Get 2D crosings with a circle (3d cylinder along Z axis) of
  radius r and center (x,y only) at 'center'.
  The results are in t1, t2 (vector distance on line) and if solutions are found).
  Returns the number of solutions - either 0 or 2. If only one
  solution is found, then this solution in in both t1 and t2.
  Solved from the equations:
  Line = P + V*t (or  P(t) = [P_x + V_x * t, P_y + V_y * t])<br>
  Circle (x - C_x)^2 + (y - C_y)^2 = r^2 <br>
  so by inserting line into circle:<br>
  ((P_x + V_x * t) - C_x)^2 + ((P_x + V_x * t) - C_x)^2 = r^2 <br>
  define k_x = P_x - C_x; <br>
  define k_y = P_y - C_y; <br>
  and isolate t-terms and solve this 2nd order equation:<br>
  (V_x^2 + V_y^2)t^2 + 2(k_x V_x + k_y V_y)t + k_x^2 + k_y^2 - r^2 = 0 */
  int getCylinderCrossings(UPosition center, double r,
                          double * t1, double * t2);
  /**
  Get 3D crosings of this line and a sphere of radius r and center (x,y,z) at 'center'.
  The results are in t1, t2 (vector distance on line) and if solutions are found).
  Returns the number of solutions - either 0 or 2. If only one
  solution is found, then this solution in in both t1 and t2.
  Solved from the equations:
  Line = P + V*t (or  P(t) = [P_x + V_x * t, P_y + V_y * t, P_z * V_z * t])<br>
  Circle (x - C_x)^2 + (y - C_y)^2 + (z - C_z)^2 = r^2 <br>
  so by inserting line into circle:<br>
  ((P_x + V_x * t) - C_x)^2 + ((P_x + V_x * t) - C_x)^2 + ((P_z + V_z * t) - C_z)^2= r^2 <br>
  define k_x = P_x - C_x; <br>
  define k_y = P_y - C_y; <br>
  define k_z = P_z - C_z; <br>
  and isolate t-terms and solve this 2nd order equation:<br>
  (V_x^2 + V_y^2 + V_z^2)t^2 + 2(k_x V_x + k_y V_y + k_z V_z)t + k_x^2 + k_y^2 + k_z^2 - r^2 = 0
  */
  int getSphereCrossings(UPosition center, double r,
                          double * t1, double * t2);
  /**
  This functio finds the crossing of the two lines projected to the
  Z plane, i.e. the x and y position is used only, and the z position
  returned is always zero.
  Returns true if crossing is found. */
  bool getXYCrossing(ULine other, UPosition * crossingXY);
  /**
  Get signed distance from the x,y part of this position to
  this line (projected to the z-plane).
  Returnes the distance, with sign indicating the side of the line. */
  double getXYsignedDistance(UPosition pos);
  /**
  Get vector XY heading in radians */
  inline double getXYHeading()
  { return atan2(vec.y, vec.x); };
  /**
  Turn the vector in this line to its normal vector in the (XY plane only).
  This is done by swapping the x and y values and xhanging sign on the x value if 'left' is true,
  otherwise change sign on y-value. */
  void turn90degXY(bool left);
  /**
   * Normalize the line, to make a valid line (vector needs to be a unit vector) */
  inline void normalize()
  { vec.toUnitVector(); };

public:
  /**
  Position on line, if a line segment then this is the
  (or one of) end point. */
  UPosition pos;
  /**
  Direction of line. This is a unit vector */
  UPosition vec;
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
Class to describe a 3D line using a 3D position, a unit vector and a length. */
class ULineSegment : public ULine
{
public:
  /**
  Get (end) type string for this structure */
  virtual const char * getDataType()
  {
    return "linesegment";
  };
  /**
  Clear */
  virtual void clear();
  /**
  Set line from two points.
  Reference position is set from pos1, and other end is
  set to pos2 so that "pos1 + vec * length" ends at pos2.  */
  virtual void setFromPoints(const UPosition * pos1, const UPosition * pos2);
  /**
  Set line from two points.
  Reference position is set from pos1, and other end is
  set to pos2 so that "pos1 + vec * length" ends at pos2.  */
  virtual void setFromPoints(UPosition pos1, UPosition pos2);
  /**
   * Set line segment from points. a line segment is a start point, a direction vector and a length.
   * \param x1,y1,z1 defines the first (start) point
   * \param x2,y2,z2 defines the end point */
  void setFromPoints(double x1, double y1, double z1,
                     double x2, double y2, double z2);
  /**
  Show values of line to console */
  void show(const char * prestring = NULL);
  /**
  Print values of line to string */
  virtual void snprint(const char * prestring, char * buff, const int buffCnt);
  /**
  Same as abowe */
  inline void print(const char * prestring = NULL)
  { show(prestring); };
  /**
  Get distance from a 3D point to a line segemnt, i.e.
  distance to the closest end or if closer to a point on the
  line segment, then this distance. the distance is squared.
  The optional second parameter returns the closest point (if != NULL)
  where = 0 if on line segment, 1 if closest to 'pos', and 2 if
  closest to 'other end' */
  double getDistanceFromSegSq(UPosition point, int * where = NULL);
  /**
  Get distance to line segment from a point (not squared).
  where = 0 if on line segment, 1 if closest to 'pos', and 2 if
  closest to 'other end' */
  inline double getDistanceFromSeg(UPosition point, int * where = NULL)
    { return sqrt(getDistanceFromSegSq(point, where)); };
  /**
  Get position of other end of line segment */
  inline UPosition getOtherEnd()
    { return getPositionOnLine(length); };
  /**
  Is this line segment crossing the other segment,
  Returns true if crossing (within ends), and
  The crossing (x,y-only) is returned in 'crossing'
  If no crossing is found, 'crossing' may be changed. */
  bool getSegmentCrossingXY(ULineSegment * other,
                            UPosition * crossing);
  /**
  Get --- in XY space --- the signed distance of 'point'
  relative to this segment.
  Positive is to the right of segment - seen from first end
  in direction towards other end.
  Returnes distance to either end or to a point on line
  whatever is closest. If 'where' is not NULL, then
  the closest part is returned here as: 0=point on line, 
  1= first point, 2= other end. */
  double getDistanceXYSigned(UPosition point, int * where);

public:
  /**
  Length of line segment */
  double length;
};

#endif
