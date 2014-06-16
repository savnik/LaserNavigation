/***************************************************************************
 *   Copyright (C) 2007 by DTU (Christian Andersen)
 *   jca@oersted.dtu.dk
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
#ifndef ULASEROBST_H
#define ULASEROBST_H

#include <umap4/upose.h>
#include <ugen4/uline.h>

#include "ulaserpoint.h"

class ULaserScan;
class ULaserPi;

/**
Class that holds an obstacle as seen by the lase scanner

@author Christian Andersen
 */
class ULaserObst
{
  public:
  /**
    Constructor */
    ULaserObst();
  /**
    Destructor */
    ~ULaserObst();
  /**
    find obstacle at edge of passable interval */
    bool findEdgeObstacle(int refIdx, bool toTheLeft,
                          ULaserScan * scanRef,
                          ULaserPi * neighbourPi);
  /**
    Is obstacle valid */
    bool isValid();
  /**
    Make obstacle invalid */
    inline void setInvalid()
    { leftIdx = 0; };
  /**
    Get left index into laserscan */
    inline int getLeftIndex()
    { return leftIdx; };
  /**
  Get max index into laserscan */
  int getIndexMax();
  /**
    Get right index into laserscan */
    inline int getRightIndex()
    { return rightIdx; };
  /**
    Get pointer to fitted line segment */
    inline ULineSegment * getLineSegment()
    { return &lineFit; };
  /**
    Get variance for fitted line */
    inline double getLineSegmentVar()
    { return lineFitVariance; };
  /**
    Move coordinates from robot to map coordinates, using this
    pose */
    void moveToMap(UPose odoPose);
  /**
    Get distance to closeby passable interval */
    inline double getClosebyPiDist()
    {  return closePiDist; };
  /**
    Get distance to closeby passable interval */
    ULaserPi * getClosebyPi();
  
  private:
  /**
    leftmost laser measurement of obstacle */
    int leftIdx;
  /**
    rightmost laser measurement of obstacle */
    int rightIdx;
  /**
    Reference to the scan detected the obstacle */
    ULaserScan * scan;
  /**
    Line segment best fitting the obstacle */
    ULineSegment lineFit;
  /**
    Line fit variance */
    double lineFitVariance;
  /**
    Center of obstacle */
    UPosition center;
  /**
    Is this edge just another passable interva
    then this pointer is to the other PI */
    ULaserPi * closePi;
  /**
    Distance to end of neighbohs end position */
    double closePiDist;
};

#endif
