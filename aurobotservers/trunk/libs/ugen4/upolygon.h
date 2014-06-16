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
#ifndef UPOLYGON_H
#define UPOLYGON_H

#include <stdlib.h>

#include "ucommon.h"
#include "u3d.h"
#include "udatabase.h"
#include "ulock.h"
#include "uline.h"


/**
General polygon (or polyline or an ordered list of positions) with no data.
* The class should normally be used as a UPolygon40 or UPolygon400.
@author Christian Andersen */
class UPolygon : public UDataBase, public ULock
{
public:
  /**
  Constructor */
  UPolygon()
  {
    points = NULL;
    ppoints = NULL;
    pointsCnt = 0;
    ppointsCnt = 0;
    pointsMax = 0;
    aPolygon = true;
    setCogXYvalid(false);
    strncpy(color, "dddd", 8);
  };
  /**
   * set max size of polygon, the size if never reduced.
   * The current points and other settings are not changed
   * \param maxPoints is maximum number of 3d positions that defines the polygon or polyline 
   * \returns true if polygon after the call has at least maxPoints points */
  bool setSize(int maxPoints);
  /**
  Clear the polygon */
  virtual void clear()
  {
    pointsCnt = 0;
    ppointsCnt = 0;
    setCogXYvalid(false);
    aPolygon = true;
  };
  /**
  Destructor */
  virtual ~UPolygon()
  {
    if (points != NULL)
      free(points);
    if (ppoints != NULL)
      free(ppoints);
  };
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "polygon";  };
  /**
  Function to test if the class or one of its ancestors is of a specific type */
  virtual bool isAlsoA(const char * typeString);
  /**
  Print points to console */
  void print(const char * prestring);
  /**
  Print points to string buffer */
  virtual void snprint(const char * prestring, char * buff, const int buffCnt);
  /**
  Print sorted after ppoints */
  void printSort(const char * prestring);
  /**
  Add a point to end of polygon */
  bool add(UPosition point);
  /**
  * Add a point to end of polygon */
  bool add(UPosition * point);
  /**
    Add all points - or as many as there is space for.
    to this polygon.
    Returns true if there were space. */
    bool add(UPolygon * poly);
  /**
  Push a point to end of polygon.
  (Same as add, but takes a pointer) */
  inline bool push(UPosition * point)
  { return add(point); };
  /**
  Add a point to end of polygon */
  inline bool add(double x, double y, double z = 0)
  {
    UPosition p(x, y, z);
    return add(p);
  };
  /**
   * Make X,Y based convex hull polygon from array of points in this polygon.
   * The ppoints list is modified by the operation.
   * \param destination is the destination polygon, the z-values of the
   * new vertex points are maintained from the source.
   * \Returns true if space in destination polygon. */
  bool extractConvexTo(UPolygon * destination);
  /**
  Get center of gravity using x,y only.
  COG and max distance vill be calculated if not valid already. */
  UPosition getCogXY();
  /**
  Get area, using A = 0.5 * sum(x_i*y_i+1 - x_i+1*y_i) for i=1..n
  where point n+1 is taken as the first point */
  double getXYarea();
  /**
  Is centre of gravity valid */
  inline bool isCogXYvalid()
  { return cogXYvalid; };
  /**
  Set centre of gravity flag */
  inline void setCogXYvalid(bool value)
  { cogXYvalid = value; };
  /**
  Set centre of gravity flag */
  inline void setPointsCnt(int value)
  { pointsCnt = mini(value, pointsMax); };
  /**
  Get max distance to Centre of gravity.
  If not calculated already it will be calculated */
  double getCogXYmaxDist();
  /**
  Get number of verteces that are closer than this distance */
  int getCloseVertexCnt(double dist);
  /**
  Remove near vertex if distance to next vertex is less than
  'dist'. The count of removed verteces is returned. */
  int removeNearVertex(double dist);

  /**
  Get a pointer to the element that is lower left in x,y coordinates,
  that is the element with minimum y, if more than one, then the one with
  the minimum x of these. */
  UPosition * getLowerLeftXY();
  /**
  Sort the polygon elements according to angle to the position
  in 'pc'. The polygon positions are returned in order
  in the '*pdata[]' array.
  The array must be able to hold the number of
  points in the array. */
  virtual void sortByAngleXYTo(UPosition * pc);
  /**
  Get points array - raw data, all points are in this array of UPosition */
  inline UPosition * getPoints()
  { return points; };
  /**
  Get orderd pointers array, this is a pointer array of pointers to UPosition */
  inline UPosition ** getPPoints()
  { return ppoints; };
  /**
  Get the latest position in the polygon list
  and remove delete the point from the polygon.
  If there is no points left in the polygon
  a (0.0, 0.0, 0.0) position is returned. */
  UPosition pop();
  /**
  Get point by index.
  NB! no range check. */
  inline UPosition getPoint(int index)
  { return points[index]; };
  /**
  Get last osition in line.
  NB! no range check. */
  inline UPosition getLastPoint()
  { return points[pointsCnt - 1]; };
  /**
  Get sorted point by index.
  NB! no range check. */
  inline UPosition getPPoint(int index)
  { return *ppoints[index]; };
  /**
  Get the position from top of points
  list, at this distance from top.
  If not enough points, then NULL is returned. */
  UPosition * getFromTop(int fromTop);
  /**
  Get number of points in polygon */
  inline int getPointsCnt()
  { return pointsCnt; };
  /**
  Copy the sorted polygon data to destination (and copies color spec too).
  \Returns true if space for elements.
  If not space, then the first elements are copied
  until no more space. */
  bool copyTo(UPolygon * destination);
  /**
  Copy points in this polygon to a destination polygon - but points only (not color)
  \param destination is the destination polygon.
  \returns true if space in destination, else desitnation is filled with as many points as possible. */
  bool copyPointsTo(UPolygon * destination);
  /**
  Copy a polygon in image pixels to this polygon */
  bool copy(CvPoint * source, int sourceCnt);
  /**
  Get x and y limits of the points in the
  points list.
  Returns true if at least one point is present. */
  bool getLimits(double * minx, double * miny,
                  double * maxx, double * maxy);
  /**
   * Is this polygon overlapping with 'poly2'.
   * Uses crossing line-segment for analysis, and returns true, if there is crossing lines.
   * if count is not NULL then number of crosses are counted.
   * assumes 1 cross - are just touching (not safe - numeric problems)
   *         2 crosses slight overlap.
   *         3 crosses overlap and touches the other side (not safe - nomeric problems)
   *         4 crosses ibstacles are crossing each other
   *         >4 crosses should not be possible, but commen sides may give false crosses.
   \param poly2 is the other obstacle
   \param xcnt is an optional parameter that will count the crossings.
   \param xes[] is the crossing positions for the xrossings
   \param xesCnt is the size of the array for crossings.
   \returns true if overlap,
   \returns NB! false if one is embedded into the other - use isInsideConvex(poly2)
   \returns false if they are totally separated */
  bool isOverlappingXY(UPolygon * poly2, int * count = NULL, UPosition xes[] = NULL, const int xesCnt = 0);
  /**
   * Is this polygon overlapping with 'poly2'.
   * Uses crossing line-segment for analysis, and returns true, if there is crossing lines.
   * if count is not NULL then number of crosses are counted.
   * assumes 1 cross - are just touching (not safe - numeric problems)
   *         2 crosses slight overlap.
   *         3 crosses overlap and touches the other side (not safe - nomeric problems)
   *         4 crosses ibstacles are crossing each other
   *         >4 crosses should not be possible, but commen sides may give false crosses.
   \param poly2 is the other obstacle
   \param xcnt is an optional parameter that will count the crossings.
   \param xes[] is the crossing positions for the xrossings
   \param xesCnt is the size of the array for crossings.
   \param type is an array where crossing type is returned:
            crossing type is seen in positive x direction - used in cell decomposition.\n
            type = 4 is a dead end as higher x-values will be inside both polygons \n
            type = 5 is a new opening as higher x-values will be outside both polygons \n
            type = 6 is an upper xrossing (positive y side of both polygons) \n
            type = 7 is a lower xrossing (negative y-side of both polygons)
   \returns true if overlap,
   \returns NB! false if one is embedded into the other - use isInsideConvex(poly2)
   \returns false if they are totally separated */
  bool isOverlappingXYtype(UPolygon * poly2, int * xcnt, UPosition xes[], const int xesCnt, int type[]);
  /**
  Is this polygon overlapping with 'poly2'.
  Returns true if so.
  Margin may be positive or negative, if
  positive they are overlappint if ine polygon has a
  vertex closer than the margin to the other.
  Uses signed distancefrom vertex to edge lines.
  If closeThis or closePoly2 are not NULL, then
  one of them will be set to the (first) vertex
  that is inside margin distance of the other polygon.
  This function tests vertex positions only and will not detect that two
  obstacles may overlap if no vertex is within margin, as may happen
  if two long square polygons overlaps far from vertices - e.g. in a cross. */
  bool isOverlappingXYconvex(UPolygon * poly2, double margin,
                             int * closeThis = NULL, int * closePoly2 = NULL);
  /**
  This function uses line segments when determining nearness of polygons
   * \param poly2 the other polygon
   * \param margin a positive value (or 0.0) is the distance the polygons may be apart, and still count as overlapping.
   * \param close a position on one of the polygons near the other polygon (if the position pointer is not NULL).
   * \Returns true if the closest distance is less than the provided 'margin' */
  bool isOverlappingXYconvex2(UPolygon * poly2, double margin,
                             UPosition * close = NULL);
  /**
  Find the index of first and last vertex that is within this margin of the other polygon.
  At the same time find wich poins from the other obstacle that is too close to this obstacle.
  at maximum 3 set of vertex indices are found.
  The indexes are returned in pairs, so that at maximum 6 index numbers are returned. */
  bool isOverlappingXYconvexSeg(UPolygon * poly2, double margin,
                                int closeThis[6], int * closeThisCnt, int closePoly2[6], int * closePoly2Cnt);
  /**
  Is this line segment crossing this polygon.
  Assuming the polygon is convex the number of crossings is maximum 4, if the crossing is exactly at a vertex, then
  no more than 4 crossings should be reported, but this is not tested much. 
  If not convex the number of crossings are unlimited. */
  int getCrossings(ULineSegment * seg, UPosition xes[], const int xesCnt);
  /**
  Is this position (in X,Y) inside the polygon
  defined by the points in this polygon.
  The polygon is assumed to be convex and
  with the points listed counter clockwise.
  A point on one of the lines is deemed to be inside.
  The margin may be set to a positiv value, if
  a position this distance from the polygon
  is counted as inside.
  (if negative it must be deper than the margin
  to be counted as inside). */
  bool isInsideConvex(double x, double y, double margin = 0.0);
  /**
  Get max number of available points */
  inline int getPointsMax()
  { return pointsMax; };
  /**
  Get line segment. */
  ULineSegment getSegment(int index);
  /**
  NB! do not use this function - it fails in some situations!
  Use getClosestDistance(...) instead.
  Get distance from polygon side or vertex. Negative is inside, if
  points in polygon are ordered counter clockwise (x,y points).
  'idx' is set to closest vertex or side, returns -1 if polygon is empty,
  'idx' may be NULL.
  'posOnPolygon' is set to the closest position on polygon (if not NULL).
  'vertex' is set true if closest position is to the vertex (not the side).
  'vertex' may be NULL.
  All is calculated in x,y only.*/
  double getDistanceXYsigned2(UPosition pos, int * idx,
                             UPosition * posOnPolygon, bool * vertex);
  /**
  Get position of most distant vertes relative to the position
  in 'x, y' to the direction in 'side', where side=1 to the left and
  side=0 is to the right seen in the 'h' direction.
  Calculation is in X,Y only.
  Returns position of most distant vertex in 'posOnPolygon' and the
  (positive) distance as returned value (negative if to the other side). */
  double getMostDistantVertexXY(double x, double y, double h,
                                int side, UPosition * posOnPolygon);
  /**
  Is the points describing a closed polygon */
  inline bool isPolygon()
  { return aPolygon; };
  /**
  Is the points describing an open polyline */
  inline bool isPolyline()
  { return not aPolygon; };
  /**
  Set line as polygon */
  inline void setAsPolygon(bool value = true)
  { aPolygon = value; };
  /**
  Set line as polygon */
  inline void setAsPolyline(bool value = true)
  { aPolygon = not value; };
  /**
  Remove one position in polygon at 'index' */
  void remove(int index);
  /**
  Insert a new position at an index position - moving the other parts up one step.
  \param pos is the new 3D position.
  \param index is the index position for the new point.
  \returns false if no more space (last point is removed) */
  bool insert(UPosition pos, int index);
  /**
  Get the most distant poit from this line. If 'leftSide' is true the point is the leftmost
  (but may actually be to the right of the line), otherwise the rightmost point is found.
  if 'insideSegment', then only points inside the band perpendicular to the line segment is used.
  \Returns -1 if no point is inside the band (when 'insideSegment' is true).
  \Returns index to the most extreme point (when found).
  If 'distance' is not NULL, then the founed (signed) distance from the line is returned. positive is to the right.
  The point with the index 'exclude' is not evaluated.
  \param line is the line segment to be tested against.
  \param leftSide - find leftmost point relative to line segment vector direction (else rightmost)
  \param insideSegment valid only, if direct left or right of line segment
  \param distance is the found distance to the point (positive to the left of line segment).
  \param exclude a specific point may be excluded from the search (if not -1)
  \param tPos is the position on the line for the found point, from start of line in m */
  int getMostDistantXY(ULineSegment * line, bool leftSide,
                       bool insideSegment, double * distance,
                       int exclude = -1, double * tPos = NULL);
  /**
  Get distance from this point to the polygon.
  It is tested at the most positive distance when taking the signed distance
  to all the edges.
  The result is zero or negative if the position is inside the polygon.
  If 'closeSide' != NULL then the side number is returned her.
  Side number 1 connects vertex 1 and 2.
  If the distance is closer to a vertex than a side (and closeVertex != NULL)
  then that vertext number is returned in 'closeVertex'.
  If closer to a side, then '*closeVertex' is -1. '*closeSide' will
  always be a valid edge number (if polygon has at least 1 point). */
  double getDistance(double x, double y,
                     int * closeSide = NULL, int * closeVertex = NULL);
  /**
  Is the other polygon embedded in this polygon.
   * \param other polygon
   * \param maxDist returns the maximum distance from the inner to the outher polygon.
   * \Returns true if the maximum distance from any point in other is negative.*/
  bool isEmbedded(UPolygon * other, double * maxDist = NULL);
  /**
  Get closest point on polygon to this point.
  \param toX,toY is the position to compare against
  \param maxRelevantDist ignore (no action), if distance is further away than this.
  \param closest is the position on the polygon closest to toX,toY (if relevant).
  \param atVertex returns true if closest point is a vertes and false if it is a side (if relevant).
  \Returns the distance to the polygon if it is within the max relevant distance
  'maxRelevantDist' and the actual closest position in '*closest'.
  Otherwise the function returns a distance above maxRelevantDist and no change to 'closest' and 'atVertex'. */
  double getClosestDistance(double toX, double toY, double maxRelevantDist, UPosition * closest, bool * atVertex = NULL);
  /**
   * Code this structure in XML format. The open tag includes
   * any additional XML attributes from the codeXmlAttributes function call
   * \param buf is the character buffer where the attributes are stored
   * \param bufCnt is the amount of buffer space allocated
   * \returns a pointer to the buffer */
  virtual char * codeXml(char * buf, const int bufCnt, const char * extraAttr);
  /**
  Code polygon using the tagname 'polygon'.
  Handles any number of vertices, but require a buffer of
  about 90 characters per vertice (+ about 50 for frame)
  * \param name is an optional extra name attribute value - is added as name value in quotes.
  * \param buf is the buffer to code to,
  * \param bufCnt is the length of the buffer,
  * \param extra is a full string of extra attributes (assumed to be coded to legal xml standard, i.e. aaa="bbb" ccc="555" ....
  * \returns pointer to buffer */
  char * codeXml(const char * name, char * buf, int bufCnt,
                 const char * extra);
  /**
  Cut this polygon at this line (x,y only) into a left and right polygon.
  Both destination polygons are modified by the call (if not NULL)
  Destination is concave if source is concave.
  source may be the same as one of the destination polygons.
  Both destinations may be NULL - to test if polygon is crossed by line.
  \param line is a cutting line segment
  \param source is the source polygon
  \param dstL is the left side polygon (may be NULL)
  \param dstR is the right side polygon (may be NULL)
  \returns true if cut, else one of the destination polygons is a copy of the source.*/
  bool cut(U2Dlined * line, UPolygon * source, UPolygon * dstL, UPolygon * dstR);
  /**
  Find the cut points on this polygon and this line (x,y only).
  \param line is a cutting line segment
  \param xPnts is where the result is placed
  \param xPntsCnt is the number of positions in the xPnt array (no more than this is used).
  \returns number of crossings (max 2 if convex) - returns also crossing count even if not enough crossing points in xPnts.*/
  int cutPoints(U2Dlined * line, U2Dpos * xPnts, int xPntsCnt);
  /**
  Set position of a point.
  \param idx is the position in the points array where to set the position.
  \param pos is the new vertex position.
  \returns true id position is set. */
  inline bool setPoint(int idx, UPosition pos)
  {
    if (idx >= 0 and idx < pointsMax)
    {
      points[idx] = pos;
      if (idx < pointsCnt)
        cogXYvalid = false;
      return true;
    }
    else
      return false;
  };
  /**
  Convert this polygon to a convex polygon with point order CCV.
  \returns true if space for the resulting polygon within a 40 point polygon. */
  bool toConvex();
  /**
   * Set paint hint for polygon.
   * \param colorStr is paint hint info:
      1st character is color of the polygon on display.
      Assumed to be like MATLAC/SCILAB 'r' = red, 'k' = black 'g'=green ....
      Or using default color if color='d'.
      2nd chararcter is width of line in pixels '1' is one pixel, 'd' is default.
      Third character is vertex marking, i.e. 'o' is circle, 'd' is none.
      Fourth character is 'd' for solid line, '-' for dashed, '.' for dotted and ' ' (space) for invisible.
      (dashed and dotted is pt. not implemented in auclient).
      5th-to-7th character is unused - set to '\0'.
      String is transferred as string, so must be terminated by a zero character and zero as information is thus not allowed. */
  void setColor(const char * colorStr)
  { strncpy(color, colorStr, 7); }


protected:
  /**
  Copy the sorted polygon data (using unchecked ppoint list) to destination.
  NB! requires that a sort function is called first, e.g.
  \Returns true if space for elements.
  If not space, then the first elements are copied
  until no more space. */
  bool copySortedTo(UPolygon * destination);

  
protected:
  /**
  Edge points in polygon */
  UPosition * points;
  /**
  Number of points used */
  int pointsCnt;
  /**
  Edge points in polygon */
  UPosition ** ppoints;
  /**
  Number of used points in pointer array,
  The ppoints array may hold just a subset of
  the points. */
  int ppointsCnt;
  /**
  Max number of available points */
  int pointsMax;
  /**
  Center of gravity in x,y only - valid onlly when cogXYvalid is true. */
  UPosition cogXY;
  /**
  Is centre of gravity and most distant vertex (cogXYmaxDist) valid */
  bool cogXYvalid;
  /**
  Is the line a closed polygon (else it is assumed to be a poly-line) */
  bool aPolygon;
  /**
  Distance from cog to most remote vertex - valid only, when cogXYvalid is true */
  double cogXYmaxDist;
public:
  /**
  1st character is color of the polygon on display.
  Assumed to be like MATLAC/SCILAB 'r' = red, 'k' = black 'g'=green ....
  Or using default color if color='d'.
  2nd chararcter is width of line in pixels '1' is one pixel, 'd' is default.
  Third character is vertex marking, i.e. 'o' is circle, 'd' is none.
  Fourth character is 'd' for solid line, '-' for dashed, '.' for dotted and ' ' for invisible.
  5th-to-8th character is unused - set to '\0'.
  String is transferred as string, so must be terminated by a zero character and zero as information is thus not allowed. */
  char color[8];
};

///////////////////////////////////////////////

/**
Polygon with up to 40 vertices. */
class UPolygon40 : public UPolygon
{
  public:
  //size of polygon
    static const int MAX_POINTS = 40;
  public:
  /**
    Constructor */
    UPolygon40()
    {
      setSize(40);
//       pointsMax = MAX_POINTS;
//       points = data;
//       ppoints = pdata;
    }

public:
  /**
  Sort the polygon elements according to angle to the position
  in 'pc'. The polygon positions are returned in order
  in the '*pdata[]' array.
  The array must be able to hold the number of
  points in the array.
  If 'pc' points to one of the points in the polygon,
  then this point is not included in ppoints. */
  virtual void sortByAngleXYTo(UPosition * pc);

//   protected:
//   /**
//     Array of points */
//     UPosition data[MAX_POINTS];
//   /**
//     Array of pointers for sorting etc */
//     UPosition * pdata[MAX_POINTS];
};

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/**
Polygon with up to 400 vertices. */
class UPolygon400 : public UPolygon
{
public:
  //size of polygon
    static const int MAX_POINTS = 400;
public:
  /**
  Constructor */
  UPolygon400()
  {
    setSize(400);
//     pointsMax = MAX_POINTS;
//     points = data;
//     ppoints = pdata;
  }

public:
  /**
  Sort the polygon elements according to angle to the position
  in 'pc'. The polygon positions are returned in order
  in the '*pdata[]' array.
  The array must be able to hold the number of
  points in the array.
  If 'pc' points to one of the points in the polygon,
  then this point is not included in ppoints. */
  virtual void sortByAngleXYTo(UPosition * pc);

//   protected:
//   /**
//     Array of points */
//     UPosition data[MAX_POINTS];
//   /**
//     Array of pointers for sorting etc */
//     UPosition * pdata[MAX_POINTS];
};

#endif
