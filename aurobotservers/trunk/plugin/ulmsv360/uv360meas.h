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

#ifndef UV360MEAS_H
#define UV360MEAS_H

#include <ugen4/u3d.h>
#include <umap4/upose.h>

/**
One measurements in the virtual laser scanner

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UV360Meas
{
public:
  /**
  Constructor */
  UV360Meas();
  /**
  Destructor */
  ~UV360Meas();
  /**
  Set these data as measurement */
  void set(double rng, double ang, UTime t);
  /**
  Set these data as measurement */
  void set(UPosition pos, double rng, double ang, UTime t);
  /**
  Get update time */
  inline UTime getT()
  { return updTime; };
  /**
  Get update time */
  inline double  getRange()
  { return range; };
  /**
  Get angle */
  inline double  getAngle()
  { return angle; };
  /**
  Get position */
  inline UPosition getPos()
  { return position; };

protected:
  /**
  measurements position in this sector */
  UPosition position;
  /**
  Measurement time for this measurement */
  UTime updTime;
  /**
  Range in meters */
  double range;
  /**
  Angle in radians*/
  double angle;
};

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

/**
One sector in the virtual laser scanner
  @author Christian Andersen <jca@oersted.dtu.dk>
 */
class UV360Sect
{
public:
  /**
  Constructor. */
  UV360Sect();
  /**
  Destructor. */
  ~UV360Sect();
  /**
  Update position to this new pose. */
  void updatePosition(UPose newRelPose);
  /**
  Clear all measurements in this sector */
  void clear()
  { measCnt = 0; };
  /**
  Add this measurement to the sector. */
  void addMeas(double range, double angle, UTime t);
  /**
  Add this measurement to the sector.
  * Returns false if no space to add measurement.
    (too old or too far away). */
  bool addMeas(UPosition pos, double range, double angle, UTime t);
  /**
  Get number of measurements in sector */
  inline int getMeasCnt()
  { return measCnt; };
  /**
  Get measurements in sector.
  NB! no range check for n. n must be in range [0,measMaxCnt[ */
  inline UV360Meas * getMeas(int n)
  { return &meas[n]; };
  /**
  Get number of measurements slots in sector */
  inline static const int getMeasMaxCnt()
  { return measMaxCnt; };
  /**
  Get range for this sector */
  double getRange();

protected:
  /**
  * Find the measurement slot to add this measurement based
    on time and range.
  * Returns an empty slot for the new measurement.
  * Returns NULL if too old or too far out. */
  UV360Meas * getMeasSlot(double rng, UTime t);


protected:
  /**
  Max stored measurements in this sector. */
  static const int measMaxCnt = 6;
  /**
  Measurements positions in this sector. */
  UV360Meas meas[measMaxCnt];
  /**
  Valid measurements in this sector. */
  int measCnt;
};


#endif
