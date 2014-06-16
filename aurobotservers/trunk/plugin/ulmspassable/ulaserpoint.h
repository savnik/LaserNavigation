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
#ifndef ULASERPOINT_H
#define ULASERPOINT_H

#include <ugen4/u3d.h>
#include <ugen4/u2dline.h>

/**
Definition of max range for laserscanner - this is not optimal, ad should be probably be more dynamic */
#define LASER_MAX_RANGE 8.0

/**
Classification types for a laser point - and control intervals */
enum UPassQual {PQ_EASY, PQ_ROUGH, PQ_NOT, PQ_OBST, PQ_UNKNOWN};

/**
Class that holds one laser scanner measurement point
and the classification category */
class ULaserPoint
{
  public:
  /**
    Constructor */
    ULaserPoint();
  /**
    Constructor */
    ~ULaserPoint();
  /**
  *  Set measurement in known koordinates. */
  inline void setPosXYZ(double x, double y, double z, double rng, bool rngValid)
  {
    pos.set(x, y, z);
    range = rng;
    rangeValid = rngValid;
  };
  /**
    Set laser point from basic information.
    angle is in radians and 0.0 is forward, and positive is CCV.
    LaserTilt is in radians and negative value relative
    to horizontal. range is in meter, sensor height in meter.
  * Sets set position relative to sensor position but adjusted using
    sensor tilt (Phi - around Y-axis) and height only. */
  void setPosRA(double range, double angle, double laserTilt,
              double sensorHeight, bool rngValid);
  /**
  * Set passable quality - one of PQ_EASY, PQ_ROUGH, PQ_NOT ... */
  inline void setQ(UPassQual value)
  { passQ = value; };
  /**
  * Get passable quality - one of PQ_EASY, PQ_ROUGH, PQ_NOT, ... */
  inline UPassQual getQ()
  { return passQ; };
  /**
  * Is the laser range valid - i.e. not dazzeled and not morrored
    too far away */
  inline bool isValid()
  { return rangeValid; };

  public:
  /**
  Is range measurement valid */
  bool rangeValid;
  /**
    3D laser measurement point */
    UPosition pos;
  /**
    Original laser range */
    double range;
  /**
    Passage quality */
    UPassQual passQ;
  /**
    Variance around here (fixed mask)  */
  //  double var;
  /**
    Line fit for points to and including 'varToL'  */
    U2Dline varLine;
  /**
    Signed distance from fittet line of (last valid) left-most position (varToL) */
    double distLeft;
  /**
    Signed distance from fittet line of (last valid) right-most position (this) */
    double distRight;
  /**
    distance between last two points in interval on the left side */
    double distLeftPkt;
  /**
    Distance between this point and the previous (to the right) */
    double distRightPkt;
  /**
    Variance in left neighborhood  */
    double varL;
  /**
    Curve tilt in degrees, set from edge points.
    NB! set from peacewise line fit, and line-fit
    is not valid close top 90 deg. */
    double tilt;
  /**
    variance is calculated to this right point
    (max-min range measurements are not used) */
    int varToL;
};

#endif
