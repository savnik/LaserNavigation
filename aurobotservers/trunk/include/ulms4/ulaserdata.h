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
#ifndef ULASERDATA_H
#define ULASERDATA_H

#include <stdio.h>
#include <math.h>

#include <ugen4/utime.h>
#include <ugen4/ulock.h>
#include <umap4/uposev.h>
#include <ugen4/uposrot.h>
#include <ugen4/uline.h>
#include <ugen4/upolygon.h>

/**
Maximum number of stored range values */
#define MAX_RANGE_VALUES 1000
/**
Measurement unit string length */
#define MAX_UNIT_SIZE 5

class UResPoseHist;
class UResVarPool;

/**
 * Class with fixed fak map used by fake laser scanner device */
class UFakeMap
{
public:
  UFakeMap();
  /// reset to initial position
  void reset();
  /// set original map as polygons
  bool copyToPoly();
  /**
  Get fake range using fake=2 option 
  \param angle is direction to look - as device angle 
  \param maxR is maximum range 
  \returns detected distance in meters (with added noise) */
  double getFake2range(double angle, double maxR, double minR);
  /**
  Advance a fake robot position (fake=2 or 3) a delta time of 'dt' in seconds.
  The state and startPose should be saved between calls.
  * Returns the new pose after the advance (and this should be used as
  the new start pose at the next call. */
  UPoseTVQ fakeAdvancePose(double dt);
private:
  /**
   * Simple zero mean radom noice
   * maximum width is +/- width*times/2 */
  double getPink(double width, int times);
  /**
  Advance robot position towards target pose in total 'dt' seconds.
  * Returns the distance to the the target position. */
  double fakeAdvanceControl(UPoseTVQ * currentPose, UPose targetPose, double dt);
  /**
  Make fake map and path */
  void initFakeMap();

public:
  /// fake path waypoint max count
  static const int FMPC = 9;
  /// fake path poses
  UPose fakeTargetPose[FMPC];
  /// fake waypoint count
  int fakeTargetPoseCnt;
  /// fake map max count
  static const int FMMC = 40;
  /// fake map
  ULineSegment fakeMap[FMMC];
  /// count of line part of fake scan
  int fakeWallOnlyCnt;
  /// fake map number of lines
  int fakeMapCnt;
  /// polygon description of map;
  UPolygon ** polyMap;
  /// number of valid polygons
  int polyMapCnt;
  /// global variable entry resource
  UResVarPool * varPool;
  /// pointer to odo pose history
  UResPoseHist * resOdo;
  /// pointer to odo pose history
  UResPoseHist * resUTM;
  /// current start in fake path
  int state;
  /// current robot pose (and sensor pose)
  UPoseTVQ currentTruePose;
  /// current pose of the laser scanner (updated with currentTruePose)
  UPose currentLaserPose;
  /// pose with offset and error
  UPoseTVQ currentOdoPose;
  /// distance error for laser scanner range (in m)
  double rangeError;
  /// distance error per meter moved
  double distError;
  /// heading error per meter moved
  double headError;
  /// heading offset per meter moved
  double headOffset;
  /// distance offset per meter moved;
  double distOffset;
  /// update UTM pose with true pose
  bool keepUTMpose;
  /// update odoPose with noisy pose
  bool keepODOpose;
  /// pose of laser device relative to currentTruePose
  UPosRot devicePose;
};

extern UFakeMap fakeMap;

/**
Laser values in integer array, with flag support
@author Christian Andersen
*/
class ULaserData : public UDataBase, public ULock 
{
public:
  /**
  Constructor */
  ULaserData();
  /**
  Destructor */
  ~ULaserData();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  {
    return "laserdata";
  };
  /**
  Print metadata for this set: */
  void print(char * prestring);
  /**
  Get pointer to range measurement with this index.
  NB! no range check on index. */
  inline int * getRange(int index)
  {  return &range[index]; };
  /**
  Get range measurement in meter with this index.
  Returns range in meter (same call as below, but different spelling). */
  inline double getRangeMetre(int index, bool * rangeValid = NULL)
  { return getRangeMeter(index, rangeValid); };
  /**
  Get range measurement in metre with this index.
  Returns range in metre. */
  double getRangeMeter(int index, bool * rangeValid = NULL);
  /**
  Get range measurement as x,y coordinate in laser scanner coordinates.
  \param int index to measurement (up to getRangeCnt() - 1)
  Returns x,y in metre, or 0,0 if not a valid measurement. */
  U2Dpos get2d(int index);
  /**
  Get range measurement in metre with this index.
  Returns range in metre. */
//  double getRangeValid(int index);
  /**
  Get pointer to measurement flag values with
  this index.
  NB! no range test for index (or value) */
  inline int * getFlags(int index)
  { return &flags[index]; };
  /**
  Set unit
  measurement unit 0 = cm, 1 = mm, 2 = 10cm */
  inline void setUnit(int toUnit)
  { unit = toUnit; };
  /**
  Get unit
  measurement unit 0 = cm, 1 = mm, 2 = 10cm */
  inline int getUnit()
  { return unit; };
  /**
  Set scantime  */
  inline void setScanTime(UTime value)
  { scanTime = value; };
  /**
  Get scantime  */
  inline UTime getScanTime()
  { return scanTime; };
  /**
  Set angle start and resolution
  start angle is for first measurement.
  res is increment for each measurement.
  Both measurements are in degrees.
  zero angle is in front.
  e.g. start=-90.0 res=1.0 fits for 181
  measurements with 1 deg resolution for a SICK laser */
  inline void setAngleResAndStart(double start, double res)
  {
    angleResolution = res;
    angleStart = start;
    fake = false;
  };
  /**
  Get start angle (in degrees, center is 0, right is negative).
  * If scanner is mounted upside down then the start angle is positive,
  and the resolution negative. */
  inline double getAngleStart()
  { return angleStart; };
  /**
  Get start angle (in radians, center is 0, left is positive).
  * If scanner is mounted upside down then the start angle is positive,
  and the resolution negative.*/
  inline double getAngleStartRad()
  { return angleStart * M_PI / 180.0; };
  /**
  Get resolution angle (in degrees).
  * If scanner is mounted upside down then the start angle is positive,
  and the resolution negative.*/
  inline double getAngleResolutionDeg()
  { return angleResolution; };
  /**
  Get resolution angle (in radians)
  Resolution is the angular interval
  between measurements.
  If scanner is mounted upside down then the start angle is positive,
  and the resolution negative. */
  inline double getAngleResolutionRad()
  { return angleResolution * M_PI / 180.0; };
  /**
  Get angle for this measurement.
  NB! no range check for index.
  Returns angle in degrees, 0 is front, left is positive. */
  inline double getAngleDeg(int index)
  { return angleStart + angleResolution * double(index); };
  /**
  Get angle for this measurement in radians.
  NB! no range check for index.
  Returns angle in radians, 0 is front, left is positive. */
  inline double getAngleRad(int index)
  { return (angleStart + angleResolution * double(index)) * M_PI / 180.0; };
  /**
  Set measurement count */
  inline void setRangeCnt(int value)
  { rangeCnt = value; };
  /**
  Get measurement count */
  inline int getRangeCnt()
  { return rangeCnt; };
  /**
  Set valid flag */
  inline void setValid(bool value)
  {  valid = value; };
  /**
  Get valid flag */
  inline bool isValid()
  { return valid; };
  /**
  Is the data based on one of the fixed fake scenarios */
  inline bool isFake()
  { return fake; };
  /**
  Copy data to another structure */
  void copy(ULaserData * source);
  /**
  Set serial number (scan number) */
  inline void setSerial(const unsigned long value)
  { serial = value; };
  /**
  Get serial number (scan number) */
  inline unsigned long getSerial()
  { return serial; };
  /**
  Fill random simulated data into structure
  NB! min- maxAng and resolution in degrees - range [-180,180] */
  void setSimData(double minAng,
                  double maxAng,
                  double resolution,
                  double minRange,
                  double maxRange,
                  UPose sourcePose,
                  UPosRot sensorPose,
                  int fake);
  /**
  Fill random simulated data into structure
  all angles in radians.
  * fakeMode is: 0 = not fake data.
  *              1 = random data.
  *              2,3,4 = fake corridor data. */
  void setFakeDataRad(double minAng,
                  double maxAng,
                  double resolution,
                  double minRange,
                  double maxRange,
                  UPose sourcePose,
                  UPosRot sensorPose,
                  int fakeMode);
  /**
  Set data value */
  void setValue(int idx,  // angle index
                unsigned char lsb,
                unsigned char msb);
  /**
  Write data to logfile.
  'logfile' must be open in write mode.
  Returns true if successfull. */
  bool saveToLogFile(FILE * logfile);
  /**
  Set data to the mirror if value is true.
  This corresponds to turning the laser scanner upside down. */
  void setMirror(bool value);
  /**
  Is the dataset mirrored. This is set to be true if the resolution value is
  set negative (corresponding to the missor flag is set true) */
  bool isMirror()
  { return (angleResolution < 0.0); };
//   /**
//   Advance a fake robot position (fake=2 or 3) a delta time of 'dt' in seconds.
//   The state and startPose should be saved between calls.
//   * Returns the new pose after the advance (and this should be used as
//   the new start pose at the next call. */
//   UPoseTVQ fakeAdvancePose(int * state, UPoseTVQ startPose, double dt);
  /**
  Set the maximum (valid) sensor range for sensor data */
  void setMaxValidRange(double value)
  { maxValidRange = value; };
  /**
  Set the maximum (valid) sensor range for sensor data */
  double getMaxValidRange()
  { return maxValidRange; };
  /**
  Get faked pose - only valid if the data actually is faked, and the fake method
  uses a pose to generate the data */
  inline UPose getFakePose()
  { return fakePose; };
  /**
  Get faked pose - only valid if the data actually is faked, and the fake method
  uses a pose to generate the data */
  inline UPoseTVQ getFakePoseTime()
  {
    UPoseTVQ result;
    result.set(&fakePose);  
    result.t = scanTime;
    result.q = 1.0;
    return result; };
  /**
  Set faked pose - used to set a pose relevant to the scandata. */
  inline void setFakePose(UPose pose)
  { fakePose = pose; };
  /**
  Get device number for source device */
  inline int getDeviceNum()
  { return device; };
  /**
  Set device number for source device */
  inline void setDeviceNum(int value)
  { device = value; };
  /**
  Get minimum (rightmost) angle */
  double getMinAngleDeg();
  /**
  Get maximum (leftmost) angle */
  double getMaxAngleDeg();

// protected:
//   /**
//   Get fake range using fake=2 option */
//   double getFake2range(UPose sourcePose, double angle, double maxR, double cosTilt);
//   /**
//   Advance robot position towards target pose in total 'dt' seconds.
//   * Returns the distance to the the target position. */
//   double fakeAdvanceControl(UPoseTVQ * currentPose, UPose targetPose, double dt);
//   /**
//   Make fake map and path */
//   void initFakeMap();

protected:
  /**
  Measured range values */
  int range[MAX_RANGE_VALUES];
  /**
  Flag value related to range measurements */
  int flags[MAX_RANGE_VALUES];
  /**
  Number of valid measurements */
  int rangeCnt;
  /**
  Scantime */
  UTime scanTime;
  /**
  measurment unit:
  0 = cm, 1 = mm, 2 = 10cm */
  int unit;
  /**
  angle resolution in degrees */
  double angleResolution;
  /**
  Start angle - zero is front */
  double angleStart;
  /**
  Is data valid */
  bool valid;
  /**
  Scannumber */
  unsigned long serial;
  /**
  Are the current range data from a fake source - set true on fake update and false on
  setAngleResAnsStart() */
  bool fake;
  /**
  Fake robot pose (for fake = 2 or 3) */
  UPoseV fakePose;
  /**
  Maximum (valid) laser range. for sick laser it should be set some distance
  less than max range to allow to assert range validity. */
  double maxValidRange;
  /**
  Device number for data */
  int device;
};

#endif
