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
#ifndef UPROBPOLY_H
#define UPROBPOLY_H

#include <ugen4/uimage.h>
#include <urob4/userverqueue.h>
#include <ugen4/upolygon.h>

#include "upose.h"

/**
Maximum number of points in each polygon. */
//#define MAX_POLYGON_SIZE 900
/**
Maximum number of stored path polygons */
//#define MAX_POLYGON_COUNT 30

/**
Class to hold one passable polygon and the related
information */
class UProbPoly : public UPolygon
{
public:
  //size of polygon
    static const int MAX_POINTS = 900;
  /**
  Constructor */
  UProbPoly();
  /**
  Destructor */
  ~UProbPoly();
  /**
  Clear data in polygon */
  virtual void clear();
  /**
  Add a new position to the polygon.
  Returns true if space for new position */
  bool addPos(UPosition pos, bool hardEdge);
  /**
  Move position back corresponding so that all points now are
  seen from this pose. */
  bool moveToPose(UPose poseNew, UTime poseTime);
  /**
  Move the positions in the polygon to
  map coordinates, assuming coordinates are relatiive
  to this pose. */
  bool moveToMap(UPose toPose);
  /**
  Allocates new polygon space and sets pose and time.
  The polygon count is set to buffer size. */
  bool setPoly(UPose robPose, UTime poseTime);
  /**
  Get pointer to first element in polygon */
/*  inline UPosition * getPoly()
     { return poly; };*/
  /**
  Get pointer to first boolean flag for edge not
  beeing on an image edge */
  inline bool * getIsObst()
     { return isObst; };
  /**
  Get number of vertices in polygon */
/*  inline int getPolyCnt()
     { return polyCnt; };*/
  /**
  Get pointer to number of vertices in polygon.
  Used, when adjusting number of valid polygons. */
/*  inline int getPolyCntMax()
  { return MAX_POLYGON_SIZE; };*/
  /**
  Get pose used in last projection. */
  inline UPose getPoseNow()
    { return poseNow; };
  /**
  Get pose used in last projection. */
  inline UPose getPoseOrg()
    { return poseOrg; };
  /**
  Get time for dataset */
  inline UTime getPoseTime()
    { return poseOrgTime; };
  /**
  Set the polygon count - if values are set
  from the outside. */
/*  inline void setPolyCnt(int cnt)
    { polyCnt = cnt; };*/
  /**
  Set polygon origin time */
  inline void setTime(UTime t)
    { poseOrgTime = t; };
  /**
  Set current pose */
  inline void setPoseNow(double x, double y, double h)
    { poseNow.set(x, y, h); };
  /**
  Set current pose */
  inline void setPoseNow(UPose pose)
    { poseNow = pose; };
  /**
  Set position when data were captured (Original pose) */
  inline void setPoseOrg(UPose pose)
  { poseOrg = pose; };
  /**
  Set current pose */
  inline void setPoseOrg(double x, double y, double h)
    { poseOrg.set(x, y, h); };
  /**
  Paint robot pose in image (in blue) prior to
  painting probability grid in green plane.
  The refX, refY is the center position, where robot is now.
  cellSize is scale inmeters per pixel. */
  bool paintRobot(UImage * img, int refX, int refY, double cellSize);
  /**
  finds the rectangular limits in 2 dimentions (X and Y) of this polygon.
  Returns true if there is any points in the polygon and
  all 4 limit value pointers are provided */
/*  bool getLimits(double * minX, double * maxX,
                 double * minY, double * maxY);*/
  /**
  Get get polygon crossings with this x-line.
  The xrossings are returned to an array of Y values.
  The buffer size is bufSize.
  No more than bufCnt crossings are returned.
  Returns the number of found crossings.
  Returns -1 if buffer is NULL and 0 if
  no polygon or no crossings.
  The crossings are returnned in the order found,
  while traversing polygon forward.
  If 'isYanObst' != NULL, then this array is
  filled with information about crossing is with
  an image edge or an assumed obstacle.
  Returns bufSize if at least bufSize crossings are found. */
  int getCrossingsAtX(double atX, // xrossings of this x value
                     double * ys,  // buffer for results
                     bool * isYAnObst,
                     int bufSize);       // buffer size)
  /**
  Get crossings at this x-value and sort result in 
  decreasing y order.
  Returns number of crossings */
  int getCrossingsAtXinYorder(double atX, // xrossings of this x value
                                         double * ys,  // buffer for results
                                         bool * isYAnObst,
                                         int bufSize);  // buffer size
  /**
  Same as GetCrossingsAtX, but for crossings of an Y-line.
  The crossings are sorted in increasing x order.*/
  int getCrossingsAtY(double atY, // xrossings of this x value
                     double * xs,  // buffer for results
                     bool * xIsAnObst, // is crossing an obstacle
                     int bufSize);  // buffer size
  /**
  Print polygon to console */
  void print(const char * prestring, bool verbose);
  /**
  Send polygon packed in SML to client.
  The pack is framed with the message tag name and with
  a 2D polygon frame:\n
  <tagname name='name' type=UProbPoly>\n
  <UProbPoly count=N hasEdge=true>\n
    <poly2db x=1.2e0 y=1.2e1 edge=true/false/>\n
    ...\n
  </UProbPoly></tagname>.
  Returns true if send.  */
  //bool sendSml(UServerInMsg * msg, const char * name);

  /**
  Set valid flag */
  void setValid(bool value)
    { valid = value; };
  /**
  Get valid flag */
  bool isValid()
    { return valid; };
  /**
  Set the value seedCromaSD to this value */
  inline void setCromaSD(double value)
  { seedCromaSD = value; };
  /**
  Set the value seedCromaSD to this value */
  inline double getCromaSD()
  { return seedCromaSD; };

protected:
  /**
  Is polygon valid, i.e. is all received */
  bool valid;
  /**
  Points describing the passabel path */
//  UPosition poly[MAX_POLYGON_SIZE];
  /**
  Number of points in polygon */
//  int polyCnt;
  /**
  Flag to mark if the corresponding point
  were on image edge */
  bool isObst[MAX_POINTS];
  /**
  Original pose for this polygon. */
  UPose poseOrg;
  /**
  Pose that is correct for the
  current polygon projection. */
  UPose poseNow;
  /**
  Time of original pose polygon */
  UTime poseOrgTime;
  /**
  If the polygon is from a path analysis, there is a standard deviation of the seed
  area cromatisity, that might be relevant. This can be stored in this variable. */
  double seedCromaSD;
  
private:

  /**
  Array of points */
  UPosition data[MAX_POINTS];
  /**
  Array of pointers for sorting etc */
  UPosition * pdata[MAX_POINTS];
};

////////////////////////////////////////

#endif
