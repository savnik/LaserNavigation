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
#ifndef UV360SCAN_H
#define UV360SCAN_H

#include <ulms4/ulaserdata.h>

#include "uv360meas.h"


#define MAX_MEASUREMENTS_PER_SCAN 360
/**
Class with a virtual 360 deg scan.

        @author Christian Andersen <jca@oersted.dtu.dk>
*/
class UV360Scan
{
public:
  /**
  Constructor */
  UV360Scan();
  /**
  Destructor */
  ~UV360Scan();
  /**
  Update virtual scanner with this scan.
  All data above max range or below 2 cm are not used as
  valid measurement.
   * \param data is the source scandata.
   * \param dataPose is robot odometry pose at scantime
   * \param notValid is an array with 4 doubles x,y of fwd-left and back-right, any measurement within this area is not used.
   * \returns true if updated */
  bool update(ULaserData * data, UPoseTime dataPose, U2Dpos notValid[]);
  /**
  Get current pose */
  inline UPoseTime getCurrentPose()
  { return currentPose; };
  /**
  Get the newest scan info as virtual laser scan */
  bool getScan(ULaserData * laserData);
  /**
  Get position of laser scanner on robot. */
  inline UPose getLaserPose()
  { return laserPose; };
  /**
  Set position of laser scanner on robot. */
  inline void setLaserPose(UPose newLaserPose)
  { laserPose = newLaserPose; };
  /**
  Get serial number of last scan */
  inline unsigned long getScanSerial()
  { return sourceSerial;};
  /**
  Get number of measurement sectors in a scan */
  inline int getSectorCnt()
  { return scanCnt; };
  /**
  Get total number of saved measurements is newest scan */
  int getMeasurementCnt();
  /**
  Get number of sectors each scan */
  inline double getResolution()
  { return angleVirtualRes; };
  
protected:
  /**
  Set index to next scan */
  void advanceScan();
  /**
  Get pointer to first sector in newest scan */
  inline UV360Sect * getNewestScan()
  { return &scan[newest];};
  /**
  Get pointer to first sector in next scan
  buffer (work in progress buffer) */
  UV360Sect * getNextScan();
  /**
  Get relative index to the sector in this direction
  The angle should be in the range [-Pi,Pi].
  * Returns index in legal range. */
  int getIndex(double angle);
  /**
  Get pointer to the sector with this angle */
  UV360Sect * getSector(int ref, double angle);
  
protected:
  /**
  Number of measurements in two virtual scans (current and next) */
  static const int scanMaxCnt = MAX_MEASUREMENTS_PER_SCAN * 2;
  /**
  Virtual scan */
  UV360Sect scan[scanMaxCnt];
  /**
  measurements in one virtual scan */
  int scanCnt;
  /**
  index to newest scan */
  int newest;
  /**
  Index to next scan (following newest) */
  int next;
  /**
  Use from this angle (in radians and range [-PI, Pi[) */
  double angleVirtualMin;
  /**
  The virtual scan has this resolution (in radians) */
  double angleVirtualRes;
  /**
  Use fresh data to this angle (radians) */
  double angleMax;
  /**
  Use fresh data to this angle (radians) */
  double angleMin;
  /**
  Latest robot pose at scantime (and update time) */
  UPoseTime currentPose;
  /**
  Serial number of latest source scan */
  unsigned long sourceSerial;
  /**
  Laser position on robot.
  Pose of laserscanner in robot perspective. */
  UPose laserPose;
  /**
   * Max valid range in source data */
  double maxValidRange;
};

#endif
