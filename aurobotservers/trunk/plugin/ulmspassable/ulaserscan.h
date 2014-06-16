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
#ifndef ULASERSCAN_H
#define ULASERSCAN_H

#include <umap4/uposev.h>
#include <ugen4/ulock.h>
#include <ugen4/uposrot.h>
#include <ulms4/ulaserdata.h>

#include "ulaserpi.h"
#include "ulaserobst.h"
#include "ulaserpoint.h"
//#include "uobstaclepool.h"

/**
Maximum number of passable intervals per scan */
#define MAX_PASSABLE_INTERVALS_PR_SCAN 40
/**
Maximum height above ground allowed as passable (meter) */
#define MAX_ALLOWED_HEIGHT 0.20
/**
Minimum height above ground allowed as passable (meter) */
#define MIN_ALLOWED_HEIGHT -0.5
/**
Minimum measurements for an allowed passable interval */
# define MIN_MEASUREMENTS_PER_PI 5


class ULaserPi;
class UObstaclePool;

/**
A full laser scan */
class ULaserScan : public ULock
{
  public:
  /**
    Constructor */
    ULaserScan();
  /**
    Clear scan to empty */
    void clear();
  /**
    Set position, speed and detection time */
    void setPoseAndTime(UTime laserTime,
                        double odoX,
                        double odoY,
                        double odoTheta);
  /**
    Set passage quality for one position.
    -  to one of PQ_EASY, PQ_ROUGH, PQ_NOT, PQ_OBST, PQ_UNKNOWN */
    void setQ(int index, UPassQual value);
  /**
  *  Set passage quality for one position.
  * -  to one of PQ_EASY, PQ_ROUGH, PQ_NOT, PQ_OBST, PQ_UNKNOWN.
  * in the interval to and including 'throught' */
  void setQ(int index, int through, UPassQual value);
  /**
    Get passable quality for this measurement */
    UPassQual getQ(int index);
  /**
  * Returns average of variance to th left and right of this point.
  * That is approx in a robot width area. */
  double getVarHere(int index);
  /**
  * Set vscan is valid and increment the scan-counter */
  void setValid(bool value);
  /**
  * Calculate variance of an interval using line-fit method.
  * The variance is saved at the right
    extreme of the interval, and will not interfere
    with the fixed-mask variance.
  * An integrated (running average) of a larger interval
    is an attempt to get a better estimate of the smootheness
    of a larger passable (e.g. asphalt) area. This value starts decreasing
    from maxIntegratedVariance, and is never below the found fit variance. */
  void setVariance(double width, int minCnt);
  /**
  *  Get 3D position of a given data point - by index number */
  inline UPosition getPos(int index)
  {
    if ((index >= 0) and (index < dataCnt))
      return data[index].pos;
    else
      return data->pos;
  };
  /**
  * Get range for this angle.
    angle is in radians and 0.0 is in front, positive is CCV. */
  double getRange(double angle);
  /**
  *  Get scan serial number */
  inline unsigned long getSerial()
    { return serial; };
  /**
  * Get number og measurements */
  inline int getDataCnt()
    { return dataCnt; };
  /**
  * Get pointer to laser measured data.
  * NB! no range check*/
  inline ULaserPoint * getData(int index)
  { return &data[index]; };
  /**
  Get maximum number of slots in point buffer */
  inline int getDataMax()
  { return MAX_RANGE_DATA_CNT; };
  /**
    Get scan serial number */
    void setSerial(const unsigned long scanSerial)
    { serial = scanSerial; };
  /**
  * Find line segment returning a best-fit line (2D in x,y)
    with the endpoints taken as the left and right
    measuremnt projected to the line.
  * If the variance is not NULL, the variance of the fit is returned. */
  ULineSegment getLineSegmentFit(int left, int right,
                                 double * variance,
                                 UPosition * center);
  /**
  * Count number of valid measurements from 'fromThis'
    up to, but not including the 'upToThis' measurement.
  * Valid measurements is found by calling 'isValid()'.
  * Returns n or above.
    'fromThis' must be lower or equal to 'upToThis', or
    no count is erformed */
  int countValidPoints(int fromThis, int upToThis);
  /**
    Find near obstacles - closer than maxRng
   * \param obsts obstacle pool, where obstacles are to be delivered.
   * \param odopose pose of robot at scantime
   * \param maxRngL range to use in left side of scan
   * \param maxRngR range to use in right side of scan
   * \param minRange minimum range to be considered an obstacle - could remove glass refelctions
   * \param outdoorContext use outdoor settings in detecting obstacle separations
   * \param horizontalScan assume scan is from a horizontal laserscan source
   * \param ignoreIfFixed ignore obstacles that overlap fixed (mapped) obstacles.
   * \param searchExt extra search range added to the range limits
   * \returns true */
  bool findNearObstacles(UObstaclePool * obsts,
                           UPoseTime odoPose,
                           double maxRngL, double maxRngR, double minRange,
                           bool outdoorContext, bool horizontalScan, bool ignoreIfFixed,
                           double searchExt);
  /**
  Get variance in the area to the right */
  double getVarRight(int index);
  /**
  Get index into scan that has this index as left limit */
  int getRight(int index);
  /**
  Get variance in the area to the right */
  double getVarLeft(int index);
  void  setMaxValidRange(double value)
  { rangeMax =value; };
  /**
  Set scan measurements from this source structure. */
  bool setScan(ULaserData * source, UPose odoPose, UPosRot *laserPose);
  /**
  Get sensor pose */
  inline UPosRot * getSensorPose()
  { return &sensorPose; };
  /**
  Get robot pose */
  inline UPose * getRobotPose()
  { return &robotPose; };

public:
  /**
  Maximum number of points in one laserscan */
  static const int MAX_RANGE_DATA_CNT = 800;
  /**
  *  Laser scan time */
  UTime time;
  /**
  * Laser scan data */
  ULaserPoint data[MAX_RANGE_DATA_CNT];
  /**
  *  Number of valid laser data points
  *  @todo function is not tested with anything other than 181 measurements. */
  int dataCnt;
  /**
  Is scan data valid */
  bool valid;

private:
  /**
  Scan serial number */
  unsigned long serial;
  /**
  Logfile */
  FILE * logpi;
  /**
  Max (valid) range for this scan */
  double rangeMax;
  /**
  Robot pose at scantime */
  UPose robotPose;
  /**
  Sensor pose rlative to robot */
  UPosRot sensorPose;
};

#endif
