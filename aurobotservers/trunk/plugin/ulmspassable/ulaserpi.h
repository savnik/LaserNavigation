/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@elektro.dtu.dk
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ULASERPI_H
#define ULASERPI_H

#include <ugen4/u2dline.h>
#include <ugen4/uline.h>
#include <umap4/upose.h>

//#include "ulaserscan.h"
#include "ulaserobst.h"

class ULaserScan;
class UResPassable;

/**
Specification of one passable interval in
a series of laser scan values.
Number of first and last point is stored only. */
class ULaserPi
{
  public:
  /**
    Constructor */
    ULaserPi();
  /**
    Destructor */
  virtual ~ULaserPi();
  /**
    Set interval.
    With these limits (both inclusive)
    and with this minimum variance */
    void setInterval(int right, int left,
                     ULaserScan * sourceScan,
                     double varMin,
                     double varMin2);
  /**
    Set center position (parameter on line segment, i.e
    distance from right edge) and validity */
    void setCenter(double segmentCenter, bool valid);
  /**
    Get left data index */
    inline int getLeft()
    { return left; };
  /**
    Get left data index */
    inline int getRight()
    { return right; };
  /**
    Get scan (parent) of this passable interval */
    inline ULaserScan * getScan()
    { return scan; };
  /**
    Get detect time */
    virtual UTime getDetectTime();
  /**
    Get pointer to left obstacle structure */
    inline ULaserObst * getLeftObst()
    { return &obstLeft; };
  /**
    Get pointer to right obstacle structure */
    inline ULaserObst * getRightObst()
    { return &obstRight; };
  /**
    Get passable line as a 2D line */
    U2Dline get2DLine();
  /**
    Get fit variance */
    inline double getFitVariance()
    { return variance; };
  /**
    Get minimum half-robot-width variance in interval with min limit */
    inline virtual double getVarMin()
    { return varianceMin; };
  /**
    Get minimum half-robot-width variance in interval, no min limit */
    inline virtual double getVarMin2()
    { return varianceMin2; };
  /**
    Get left position  - not smoothed to best fit line. */
    UPosition getLeftPos();
  /**
    Get right position - not smoothed to best fit line. */
    UPosition getRightPos();
  /**
    Get center position of found road.
    Center position should be highest point.
    This is the measured position and not on the fitted line */
    virtual UPosition getCenterPos();
  /**
  Get distance to centerPosition from segment origin */
  inline double getCenter()
  { return centerPosition; };
  /**
  Get valid flag for center position, center position is not valid
  if the segment is too flat. In this case the center position
  is set to half line width. */
  inline bool getCenterValid()
  { return centerValid; };
  /**
    Get position this distance from right edge */
    UPosition getTop();
  /**
    Is top (center) position valid */
    inline bool isTopValid()
    { return centerValid; };
  /**
    get road tilt - relative to robot */
    inline double getTilt()
    { return tilt; };
  /**
    Calculate best-fit line */
    ULineSegment * makeBestFit();
  /**
    Find edge obstacles (left and right)
    for this passable interval.
  * Further needs acces to opther passable intervals. 
    Returns number of obstacles found. */
    int findEdgeObstacles(FILE * logo, UResPassable * resp);
  /**
    Move fitted line segment to map coordinates
    from this robot pose */
    void moveToMap(UPose * odoPose);
  /**
  Print status for passable interval - i.e.
  scannumber left and right extreme and such */
  void print(const char * preString);
  /**
  Print status to this string */
  void print(const char * preString, char * buff, const int buffCnt);
  /**
    Get height of top position relative to best-fit line.
    This is calculated from the distance to the robot
    and the tilt of the laser-plane. */
    double getTopHeight(double laserTile);
  /**
    Get the distance to the crossing (in 2D x,y only)
    between this passable interval
    and this 'line', as seen from the provided 'fromPose'.
    If the found crossing is behind the pose, then behind is true.
    If the lines are parallel 0.0 is returned and
    'behind' is (brobably) false.
    Else the distance is returned.
    'behind' may be NULL, if no need */
    double get2DDistToCross(U2Dline line,
                            UPose fromPose,
                            bool * behind);
  /**
    Get segment width perpendicular to robot.
    When robot follows road this is an estimate of road width */
    inline double getYWidth()
    { return yw; };
  /**
    Get segment euclidian width - straight line distance. */
    inline double getWidth()
    { return segment.length; };
  /**
    Get passable line segment */
    inline ULineSegment * getSegment()
    { return &segment; };
  
  private:
  /**
    Parent scan */
    ULaserScan * scan;
  /**
    Left extreme of passable interval */
    int left;
  /**
    Right extreme of passable interval */
    int right;
  /**
    Variance relative to segment */
    double variance;
  /**
    Minimum half robot width variance (within minimum limit) */
    double varianceMin;
  /**
    Minimum half robot width variance based on at leat 10 measurements, no min limit. */
    double varianceMin2;
  /**
    Road tilt value relative to robot.
    This value is set from road edge-points */
    double tilt;
  /**
    Segment width perpendiculat ro robot.
    Could be taken as road width. */
    double yw;
  /**
    Distance from right (reference) end of segment to
    top of road (center position). */
    double centerPosition;
  /**
    Is top position valid - i.e. is a significant top found */
    bool centerValid;
  /**
    Obstacle to the left */
    ULaserObst obstLeft;
  /**
    Obstacle to the right */
    ULaserObst obstRight;
  /**
    Line segment desribing the interval */
    ULineSegment segment;
};

#endif
