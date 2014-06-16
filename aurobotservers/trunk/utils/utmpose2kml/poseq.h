/***************************************************************************
 *   Copyright (C) 2008 by Christian Andersen   *
 *   chrand@mail.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef POSEQ_H
#define POSEQ_H

#include <ugen4/ucommon.h>
#include <ugen4/utime.h>
#include <math.h>
#include <string.h>
#include <expat.h>

/**
	@author Christian Andersen <chrand@mail.dk>
*/
class PoseQ
{
  public:
  /**
    Constructor */
    PoseQ();
  /**
    Constructor with initial value */
    void set(double ix, double iy, double ih)
    {
      x=ix; y=iy; h=ih;
    };
  /**
    Destructor */
    virtual ~PoseQ();
  /**
    Clear to zero position and heading zero. */
    inline void clear()
    {
      x = 0.0;
      y = 0.0;
      h = 0.0;
    };
  /**
    Get distance to another pose. */
    inline double getDistance(PoseQ other)
    { return hypot(other.x - x, other.y - y); };
  /**
    Get distance to another pose. */
    inline double getDistance(PoseQ * other)
    { return hypot(other->x - x, other->y - y); };
  /**
    Get heading difference to another pose. I.e.
    how much angle should I add th this heading to get the other
    (return limitToPi(other.h - h)). */
    inline double getHeadingDiff(PoseQ other)
    { return limitToPi(other.h - h); };
  /**
    Convert this local pose position coordinate to map (global) coordinates.
    This is done by rotating with the heading and translating
    with the pose position.
    See also getMapToPosePose(PoseQ * mapPose)*/
    PoseQ getPoseToMapPose(PoseQ poseLocal);
    /**
     * convert this local position to map coordinates */
    PoseQ getPoseToMap(double localX, double localY);

  public:
  /**
    x position (forward or north) */
    double x; //
  /**
    y position (left or west) */
    double y; //
  /**
    heading (Theta) zero in x direction and positive towards y.
    That is right hand coordinates with z pointing up. */
    double h; //
    /**
     * pose quality or line number is > 100 */
    double q;
    /**
     * width of line - taken from map perimeter */
    double w;
};

#endif
