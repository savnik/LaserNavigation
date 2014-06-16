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
#ifndef ULASERDATASET_H
#define ULASERDATASET_H

#include <ugen4/utime.h>
#include <ugen4/usmltagin.h>
#include <ugen4/u3d.h>
#include <umap4/upose.h>
#include <ugen4/uline.h>
#include <urob4/uclientport.h>
#include <urob4/usmltag.h>

/**
Number of possible measurements in one scan */
#define MAX_DATA_COUNT 1000
/**
Maximum number of passable intervals in one laser scan */
#define MAX_PIS_COUNT 15
/**
Storage for laser scans */
#define MAX_STORED_LASER_SCANS 5000
/**
Maximum storage space for passable intervals in a detected path
based on laser returns in front of the robot */
#define MAX_PASSABLE_LINE_SEGMENTS 50
/**
Length of planner string */
#define MAX_PLANNER_LINE_SIZE 200
/**
Length of text-version of stop criteria */
#define MAX_PLANNER_STOP_CRIT_SIZE 50
/**
Text status length for planner stop criteria */
#define MAX_STOP_CRIT_STATUS_LENGTH 200
/**
Max number of stop criteias */
#define MAX_STOP_CRIT_COUNT 20
/**
Number of planner variables that can be stored for display */
#define MAX_VARS_STORED 100

/**
Number of dimensions in a status variable */
#define MAX_VALUE_DIMENSIONS 4

// typedef enum valueType
// {
//       VALTYP_D,  //(double)
//       VALTYP_DS, // double speed (m/s)
//       VALTYP_DQ, // (double with quality (val, qual))
//       VALTYP_TIME, // (time (as double)
//       VALTYP_3D, //   (3D position x,y,z)
//       VALTYP_POSE // (x,y,h) */
// };


/**
Class to hold one measurement of a laser scan */
class UClientLaserData
{
public:
  /**
  Constructor */
  UClientLaserData();
  /**
  Set values from 2-byte data */
  void setValue(double ang,
           unsigned char lsb, unsigned char msb,
           double measurementUnits);
  /**
  Set values from tag data */
  void setValue(double ang, double distance,
           bool inA, bool inB, bool inZ);
  /**
  Get measurement angle */
  inline double getAngle()
    { return angle; };
  /**
  Get measured distance */
  inline double getDistance()
    { return range; };
  /**
  Is measurement inside zone A */
  inline bool isValid()
    { return range > 0.02; };
  /**
  Is measurement inside zone B */
//  inline bool inZoneB()
//    { return (flag % 10) == 2; };
  /**
  Is measurement dazzeled (measurement is not valid) */
//  inline bool isDazzled()
//    { return (flag % 10) == 3; };
  /**
  Is measurement dazzeled (measurement is not valid) */
  inline int getFlag()
  { return flag; };
  /**
  Get measurement variance to the left */
  inline double getVarL()
    { return varL; };
  /**
  Get measurement deviation to the left */
  inline double getSdL()
    { return sqrt(varL); };
  /**
  Get measurement variance to the right */
  inline double getTilt()
    { return tilt; };
  /**
  Print values of this measurement */
  void print(char * preString);
  /**
  Set values from a message tag */
  bool setFromTag(USmlTag * tag);
  /**
  Get position as 3D relative to robot,
  when laser is positioned at laserPos - assuming same orientation */
  UPosition getPosition(UPosition laserPos, double cosLaserTilt);
  /**
  Get position in robot coordinates using full pose set */
  UPosition getPosition(UMatrix4 * movLtoR);

private:
  /**
  Measuremnt is in zone A or otherwise */
  int flag;
  /**
  Distance in meter */
  double range;
  /**
  Variance */
  double varL;
  /**
  tilt of line-fit */
  double tilt;
  /**
  Angle of measurement in radians */
  double angle;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/**
class to hold a passable interval position in a laserscan. */
class UClientLaserPi
{
public:
  /**
  Constructor */
  UClientLaserPi();
  /**
  Destructor */
  ~UClientLaserPi();
  /**
  Clear and set positions to 0.0 */
  void clear();
  /**
  Set passable interval from tag */
  bool setFromTag(USmlTag * tag);
  /**
  Get left position */
  inline UPosition getLeftPos()
    { return leftPos; };
  /**
  Get right position */
  inline UPosition getRightPos()
    { return rightPos; };
  /**
  Get top position */
  inline UPosition getTopPos()
    { return topPos; };
  /**
  Get right side of road (half robot distance from edge) */
  inline UPosition getRightSide()
    { return rightSide; };
  /**
  Get left side of road (half robot distance from edge) */
  inline UPosition getLeftSide()
    { return leftSide; };

private:
  /**
  Right angle of interval */
  double rightAngle;
  /**
  Left angle of interval */
  double leftAngle;
  /**
  3D position of right edge of interval */
  UPosition rightPos;
  /**
  3D Position of left edge of interval */
  UPosition leftPos;
  /**
  Right side (half robot distance) */
  UPosition leftSide;
  /**
  Left side (half robot distance) */
  UPosition rightSide;
  /**
  Top of road */
  UPosition topPos;
  /**
  Number of links from this passable interval to
  other passable intervals in previous scan */
  int linksCnt;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/**
Holds a set of laser data

@author Christian Andersen
*/
class ULaserDataSet
{
public:
  /**
  Construcor */
  ULaserDataSet();
  /**'
  Destructor */
  ~ULaserDataSet();
  /**
  Clear laserscan */
  void clear();
  /**
  Set static data */
  void setData(
            unsigned int serial,
            int firstVal, int dataInterval,
            int dataCount, UTime time,
            double measurementUnit,
            bool statDataValid,
            double statWindowWidth,
            double laserTilt,
            double validRange
              );
  /**
  Get pointer to first data value */
  inline UClientLaserData * getData()
    { return data; };
  /**
  Get dataset time */
  inline UTime getTime()
    { return dataTime; };
  /**
  Get count of measurements */
  inline int getCount()
    { return count; };
  /**
  Is statistics data valid for the scan */
  inline bool isStatValid()
    { return statValid; };
  /**
  Get pointer to first passable interval */
  inline UClientLaserPi * getPis()
    { return pis; };
  /**
  Get number of available passable intervals */
  inline int getPisCnt()
    { return pisCnt; };
  /**
  Get scantime */
  inline UTime getScanTime()
  { return dataTime; };
  /**
  Get pose of robot, when this scan were recorded */
  inline UPose getPose()
    { return odoPose; };
  inline double getLaserTilt()
  { return laserTilt; };
  /**
  Set odometry pose for this scan */
  void setPose(UPose pose);
  /**
  Add a determined passable control interval - used by MMRD only (deprecated) */
  void addPassableInterval(UClientLaserPi * passableInterval);
  /**
  Add a detected passable interval in this laserscan - including center position
  as a number of meter from start of segment */
  void addPiSeg(ULineSegment * seg, double centerPos);
  /**
  Print dataset
  if verbose, then also data values */
  void print(char * preString, bool verbose);
  /**
  Get scan serial number */
  inline unsigned int getSerial()
    { return scanSerial; };
  /**
  Get reference to sensor pose */
  UPosRot * getSensorPose()
  { return &sensorPose; };
  /**
   * Get max valid range */
  inline double getMaxValidRange()
  { return maxValidRange; }

private:
  /**
  First data value in original dataset */
  int first;
  /**
  Interval valid measurements from original dataset */
  int interval;
  /**
  Number of valid data values */
  int count;
  /**
  Dataset time (at recetion from serial port */
  UTime dataTime;
  /**
  Original data unit value (in meter) */
  double unit;
  /**
  Data measurements */
  UClientLaserData data[MAX_DATA_COUNT];
  /**
  Width of statistics window */
  double statWidth;
  /**
  Is laserscan statistics valid */
  bool statValid;
  /**
  Pose at time of scan */
  UPose odoPose;
  /**
  Is odometry position valid */
  bool odoValid;
  /**
  Passabel intervals in scan */
  UClientLaserPi pis[MAX_PIS_COUNT];
  /**
  Number of used PIs */
  int pisCnt;
  /**
  Max number of passable line segments for any scan */
  static const int MAX_PISEG_COUNT = 15;
  /**
  Scan serial number */
  unsigned long scanSerial;
  /**
  Laser tilt value in degrees (phi) */
  double laserTilt;
  /**
  Laser sensor pose (3D) */
  UPosRot sensorPose;
  /**
   * Maximum valid range */
  double maxValidRange;
};

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

/**
History of laser scans */
class ULaserDataHistory
{
public:
  /**
  Constructor */
  ULaserDataHistory();
  /**
  Destructor */
  ~ULaserDataHistory();
  /**
  Clear all history scans */
  void clear();
  /**
  Get space for new scan */
  ULaserDataSet * getNewScan();
  /**
  Get pointer to newest data */
  ULaserDataSet * getNewest();
  /**
  Get a laser scan indexed after its age in the
  history buffer (up to MAX_STORED_LASER_SCANS - 1) */
  ULaserDataSet * getScan(int histNum);
  /**
  Get number of saved scans */
  inline int getScansCnt()
    { return scansCnt; };
private:
  /**
  Storage of scans */
  ULaserDataSet * scans[MAX_STORED_LASER_SCANS];
  /**
  Number of used scan positions */
  int scansCnt;
  /**
  Newest available scan */
  int newest;
};


#endif
