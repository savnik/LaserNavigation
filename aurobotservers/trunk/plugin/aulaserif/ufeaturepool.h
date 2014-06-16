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
#ifndef USFPOOL_H
#define USFPOOL_H

#include <ugen4/uline.h>
#include <ugen4/ulock.h>
#include <umap4/upose.h>
#include <urob4/uclientfuncbase.h>
#include <ugen4/uposrot.h>

class UFeatureData : public ULock
{
public:
  /**
  Constructor */
  UFeatureData();
  /**
  Destructor */
  ~UFeatureData();
  /**
  add segment */
  bool addSegment(ULineSegment * seg, int segInt, double segVal, const char * idStr);
  /**
  Get segment count */
  inline int getSegsCnt()
  { return segsCnt; };
  /**
  Clear segments */
  void clear();
  /**
  Get segment scan time */
  UTime getScanTime()
  { return scanTime; };
  /**
  Move data to odometry pose */
  void moveLocalToOdo(UPose scanPose, UPosRot sensorPose);
  /**
  Get max number of characters in type string */
  inline int getTypeMax()
  { return MAX_TYPE_LENGTH; };
  /**
  Get max number of segments allowed */
  inline int getSegsMax()
  { return MAX_SEGMENTS; };
  /**
  Is scan data valid */
  inline bool isValid()
  { return valid; };
  /**
  Get line data type */
  inline const char * getDataType()
  { return type; };
  /**
  Get line segment */
  inline ULineSegment * getSegs()
  { return segs; };
  /**
  Compare this type with a key string.
  Compare is case sensitive.
  Returns true if type match. */
  inline bool isA(const char * key)
  { return (strcmp(type, key) == 0); };
  /**
  Get line integer array */
  inline int * getSegsInt()
  { return segsInt; };
  /**
  Get (double) value array */
  inline double * getSegsVal()
  { return segsValue; };

  
public:

  /**
  Max number of segments */
  static const int MAX_SEGMENTS = 40;
  /**
  Max length of string value for a segment */
  static const int MAX_SEG_STR_LENGTH = 16;
  /**
    Segments */
  ULineSegment segs[MAX_SEGMENTS];
  /**
  Data integer for general use, e.g. for road lines (left=0, center=1 right=2) */
  int segsInt[MAX_SEGMENTS];
  /**
  Data string for general use */
  char segsStr[MAX_SEGMENTS][MAX_SEG_STR_LENGTH];
  /**
  An extra double value associated to each feature */
  double segsValue[MAX_SEGMENTS];
  /**
  Number of segments in array*/
  int segsCnt;
  /**
  Timestamp for scan */
  UTime scanTime;
  /**
  Robot pose at scantime */
  UPose pose;
  /**
  Is the dataset valid */
  bool valid;
  /**
  Maximum string length of segment type */
  static const int MAX_TYPE_LENGTH = 5;
  /**
  Segment type (c-string) typically holding the XML tag name */
  char type[MAX_TYPE_LENGTH];
  /**
  Is data the newest */
  bool isNewest;
  /**
   * Is the dataset valid in robot coordinates or anothe grid system
   * valid values are
   * -1 = local robot coordinates
   * 0 odometry coordinates
   * 1 UTM coordinates
   * 2 map coordinates */
  int coordinateRef;
};



/**
Holds history of scanfeatures

	@author Christian Andersen <jca@oersted.dtu.dk>
*/
class UFeaturePool : public UOnEvent
{
public:
  /**
  Constructor */
  UFeaturePool();
  /**
  Destructor */
  ~UFeaturePool();
  /**
  Get pointer to scan */
  UFeatureData * getScan(int age);
  /**
  Remove all data */
  void clear();
  /**
  Add a new set of data to the pool */
  bool addData(UFeatureData * data);
  /**
  Get max number of scans in pool */
  inline int getScansMax()
  { return MAX_SEGMENT_SCANS; };
  /**
  Get max number of scans in pool */
  inline int getScansCnt()
  { return scansCnt; };
  /**
  Mark all entries of this type as not new, until
  an enty is found that is not new already. */
  void markAsNotNew(const char * dataType);
  /**
  Set default sensor pose - used to convert sensor positions to
  robot coordinates */
  inline void setSensorPose(UPosRot value)
  {
    defaultSensorPose = value;
    defaultSensorPoseOK = true;
  };
  /**
  Get the default sensor pose - used to convert sensor positions to
  robot coordinates */
  inline UPosRot getSensorPose()
  { return defaultSensorPose; };
  /**
  Is default sensor pose set from a laser scanner message */
  inline bool gotSensorPose()
  { return defaultSensorPoseOK; };
  
protected:
    
  /**
  Max number of segment scans */
  static const int MAX_SEGMENT_SCANS = 400;
  /**
  Segments */
  UFeatureData scans[MAX_SEGMENT_SCANS];
  /**
  Number of received scans */
  int scansCnt;
  /**
  Newest scan index */
  int newest;
  /**
  Default laser scanner posr - if not included in sensor data set */
  UPosRot defaultSensorPose;
  /**
  Is default sensor pose set */
  bool defaultSensorPoseOK;
};

#endif
