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
Name length of a variable */
#define MAX_VARIABLE_NAME_SIZE 30
/**
Number of planner variables that can be stored for display */
#define MAX_VARS_STORED 100

/**
Number of dimensions in a status variable */
#define MAX_VALUE_DIMENSIONS 4

enum valueType
{
      VALTYP_D,  //(double)
      VALTYP_DS, // double speed (m/s)
      VALTYP_DQ, // (double with quality (val, qual))
      VALTYP_TIME, // (time (as double)
      VALTYP_3D, //   (3D position x,y,z)
      VALTYP_POSE // (x,y,h) */
};


/**
Class to hold one measurement of a laser scan */
class ULaserData
{
public:
  /**
  Constructor */
  ULaserData();
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
  inline bool inZoneA()
    { return (flag % 10) == 1; };
  /**
  Is measurement inside zone B */
  inline bool inZoneB()
    { return (flag % 10) == 2; };
  /**
  Is measurement dazzeled (measurement is not valid) */
  inline bool isDazzled()
    { return (flag % 10) == 3; };
  /**
  Is measurement dazzeled (measurement is not valid) */
  inline int getFlag()
  { return flag; };
  /**
  Get measurement variance */
/*  inline double getVar()
    { return var; };*/
  /**
  Get measurement variance to the left */
  inline double getVarL()
    { return varL; };
  /**
  Get measurement deviation to the left */
  inline double getSdL()
    { return sqrt(varL); };
  /**
  Get combined variance to the left */
/*  inline double getVarLC()
    { return varLC; };*/
  /**
  Get combined deviation to the left */
/*  inline double getSdLC()
    { return sqrt(varLC); };*/
  /**
  Get measurement variance to the left */
/*  inline double getVarI()
    { return varI; };*/
  /**
  Get measurement variance to the right */
  inline double getTilt()
    { return tilt; };
  /**
  Get measurement edge value */
/*  inline double getEdge()
    { return edge; };*/
  /**
  Get measurement curvature value */
/*  inline double getCurv()
    { return curv; };*/
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
  Measuremnt is in zone A */
  int flag;
  /**
  Measuremnt is in zone B */
  //bool zoneB;
  /**
  Measuremnt is dazzled - not valid e.g. in sunshine */
  //bool dazzle;
  /**
  debug variable - not used */
  //bool dummy;
  /**
  Distance in meter */
  double range;
  /**
  Variance */
//  double var;
  /**
  Variance */
  double varL;
  //double varI;
  /**
  Combined variance and curvature */
  //double varLC;
  /**
  tilt of line-fit */
  double tilt;
  /**
  Edge value */
  //double edge;
  /**
  Curvature value */
  //double curv;
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
class ULaserPi
{
public:
  /**
  Constructor */
  ULaserPi();
  /**
  Destructor */
  ~ULaserPi();
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
            double laserTilt);
  /**
  Get pointer to first data value */
  inline ULaserData * getData()
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
  inline ULaserPi * getPis()
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
  Add a determined passable control interval */
  void addPassableInterval(ULaserPi * passableInterval);
  /**
  Print dataset
  if verbose, then also data values */
  void print(const char * preString, bool verbose);
  /**
  Get scan serial number */
  inline unsigned int getSerial()
    { return scanSerial; };
  /**
  Get reference to sensor pose */
  UPosRot * getSensorPose()
  { return &sensorPose; };

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
  ULaserData data[MAX_DATA_COUNT];
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
  ULaserPi pis[MAX_PIS_COUNT];
  /**
  Number of used PIs */
  int pisCnt;
  /**
  Scan serial number */
  unsigned long scanSerial;
  /**
  Laser tilt value in degrees (phi) */
  double laserTilt;
  /**
  Laser sensor pose (3D) */
  UPosRot sensorPose;
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

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

/**
Class to hold result of laser scanner path evaluation.
All data is in (odometer) map coordinates */
class ULaserPathResult
{
public:
  /**
  Constructor */
  ULaserPathResult();
  /**
  Destructor */
  ~ULaserPathResult();
  /**
  Get number of passable lines in path */
  inline int getPassLinesCnt()
    { return passLinesCnt; };
  /**
  Get passable line with this index.
  Returns pointer to passable line segment. */
  inline ULineSegment * getPassLine(int index)
    { return &passLines[index]; };
  /**
  Get pointer to edge */
  inline ULineSegment * getEdgeLeft()
    { return &edgeLeft; };
  /**
  Get validity of edge */
  inline bool getEdgeLeftValid()
    { return edgeLeftValid; };
  /**
  Get pointer to edge */
  inline ULineSegment * getEdgeRight()
    { return &edgeRight; };
  /**
  Get validity of edge */
  inline bool getEdgeRightValid()
    { return edgeRightValid; };
  /**
  Get pointer to edge */
  inline ULineSegment * getEdgeTop()
    { return &edgeTop; };
  /**
  Get validity of edge */
  inline bool getEdgeTopValid()
    { return edgeTopValid; };
  /**
  Get obstacle avoidance route value */
  inline UPosition * getRoute(int index)
    { return &route[index]; };
  /**
  get count of route points */
  inline int getRouteCnt()
    { return routeCnt; };
  /**
  Is this route actually used for control */
  inline bool isPathUsed()
    { return pathUsed; };
  /**
  Is this route terminated in a crash */
  inline bool isPathCrashed()
  { return isACrash; };
  /**
  Decode a full path starting with the provided tag, and the
  rest taken from the provided connection */
  bool setFromTag(USmlTag * tag);
  /**
  Is route passing a wall */
  inline bool isPassingAWall()
    {  return isAWall; };
  /**
  Get manoeuvre sequence */
  inline UManSeq * getManSeq()
  { return man; };
private:
  /**
  Passable line segments */
  ULineSegment passLines[MAX_PASSABLE_LINE_SEGMENTS];
  /**
  Count of valid passable line segments */
  int passLinesCnt;
  /**
  Edge of path */
  ULineSegment edgeLeft;
  /**
  Is edge valid */
  bool edgeLeftValid;
  /**
  Left of path */
  ULineSegment edgeRight;
  /**
  Is edge valid */
  bool edgeRightValid;
  /**
  Left of path */
  ULineSegment edgeTop;
  /**
  Is edge valid */
  bool edgeTopValid;
  /**
  Is path passing a wall, i.e. at least
  2 measurements where reply is stationary, while
  robot moves */
  bool isAWall;
  /**
  A debug flag to assist analysis of crash paths */
  bool isACrash;
  /**
  Detected passable route */
  UPosition route[MAX_PASSABLE_LINE_SEGMENTS];
  /**
  Valid route points */
  int routeCnt;
  /**
  This is the used path */
  bool pathUsed;
  /**
  Most recent received manoeuvre list */
  UManSeq * man;
};

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/**
Class to hold information about one stop criteria */
class UPlannerStopCrit
{
public:
  /**
  Stop criteris name */
  char name[MAX_PLANNER_STOP_CRIT_SIZE];
  /**
  Is stop criteria valid */
  bool active;
  /**
  Status for stop criteria */
  char status[MAX_STOP_CRIT_STATUS_LENGTH];
};

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/**
Class to hold information on sensor values and variables */
class UPlannerValue
{
public:
  UPlannerValue()
  {
    int i;
    valid = false;
    type = VALTYP_D;
    name[0] = '\0';
    for (i = 0; i < MAX_VALUE_DIMENSIONS; i++)
      value[i] = 0;
  };
  /**
  Set value from tag */
  bool setFromTag(USmlTag * tag,
                 UPoseTime * poseTime);
  /**
  Format value ready for presentation for special values
  Some specific manes are shown differentlt than others, e.g.
  time and road values.
  Returns false if not a recognized special value type. */
  bool getAsString(char * buffer, int bufferLng);
  /**
  Clear variable to invalid and negative quality */
  void clear();
  /**
  Get pointer to name */
  inline const char * getName()
    { return name; };
  /**
  Get value of variable */
  inline double getValue()
    { return value[0]; };
  /**
  Get value of variable with index idx */
  inline double getValue(int idx)
  { return value[idx]; };
  /**
  Get variable type */
  inline valueType getType()
  { return type; };
  /**
  Is the value marked as valid */
  inline bool isValid()
    { return valid; };
  /**
  Get measurement quality figure. -1.0 means
  no quality is available */
  inline double getQuality()
    { return value[1]; };
protected:
  /**
  Name of variable */
  char name[MAX_VARIABLE_NAME_SIZE];
  /**
  Is value valid - may be present, but invalid - i.e. road width */
  bool valid;
  /**
  Value of variable */
  double value[MAX_VALUE_DIMENSIONS];
  /**
  Quality of estimation, a value in [0.0 to 1.0] interval */
  //double quality;
  /**
  Type of value, one of
  VALTYP_D (double)
  VALTYP_DQ (double with quality (val, qual))
  VALTYP_TIME (time (as double)
  VALTYP_3D   (3D position x,y,z)
  VARTYP_POSE (x,y,h) */
  valueType type;
};

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/**
Class to hold status information of mission planner */
class UPlannerData
{
public:
  UPlannerData()
  {
    stopCritCnt = 0;
    snprintf(cmdLine, MAX_PLANNER_LINE_SIZE, "none");
    snprintf(cmdLineLast, MAX_PLANNER_LINE_SIZE, "none");
    snprintf(cmdStoppedBy, MAX_PLANNER_STOP_CRIT_SIZE, "none");
    snprintf(cmdFile, MAX_PLANNER_LINE_SIZE, "closed");
    running = false;
    isIdle = false;
    varsCnt = 0;
  };
  /**
  Decode from tag, i.e. decode all messages starting with 'get name= .../>'.
  Returns true if in right format */
  bool setVarFromTag(USmlTag * tag,
                    UPoseTime * odoPose);
  /**
  Get pointer to a specific variable with this name.
  Returns NULL if variable do not exist */
  UPlannerValue * findValue(const char * name);
  /**
  Extract three values and store as pose.
  Returns true if all three values exist in stored set of values. */
  bool findPose(const char * nameX, const char * nameY, const char * nameH, UPose * poseDest);

public:
  /**
  Planned data - actual line */
  char cmdLine[MAX_PLANNER_LINE_SIZE];
  /**
  Command line number (in source file) */
  int cmdLineNum;
  /**
  Is command line a manual override */
  bool cmdLineOverride;
  /**
  Planned data - actual line */
  char cmdLineLast[MAX_PLANNER_LINE_SIZE];
  /**
  Last line finished by this event - one of a fixed number of strings */
  char cmdStoppedBy[MAX_PLANNER_STOP_CRIT_SIZE];
  /**
  Planner source file name (relative to server) */
  char cmdFile[MAX_PLANNER_LINE_SIZE];
  /**
  Is plan thread running */
  bool running;
  /**
  Is planner idle (no mission active) */
  bool isIdle;
  /**
  Stop criteria status for current command line */
  UPlannerStopCrit stopCrit[MAX_STOP_CRIT_COUNT];
  /**
  Count of valid stop criterias */
  int stopCritCnt;
  /**
  Is run simulated */
  bool simulated;
  /**
  Simulation source directory */
  char simSubDir[MAX_FILENAME_SIZE];
  /**
  Monitored values from planner */
  UPlannerValue vars[MAX_VARS_STORED];
  /**
  Number of vars available */
  int varsCnt;
};

#endif
