/***************************************************************************
 *   Copyright (C) 2004 by Christian Andersen                              *
 *   jca@oersted.dtu.dk                                                    *
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
#ifndef UMAPSEGMENT_H
#define UMAPSEGMENT_H

#include <list.h>
#include <ugen4/u3d.h>
#include "umapobj.h"
#include "umaprel.h"

/* forward declartion */
class UMapObj;
class UMapGmk;
class UMapZGmk;
class UMapRobot;



/**
Map segment, that holds a number of map elements. */
class UMapSegment
{
public:
  /**
  Constructor */
  UMapSegment(UMapObj * parent);
  /**
  Destructor */
  ~UMapSegment();
  /**
  Remove all objects from map.
  Do not recalculate limits. */
  void clear();
  /**
  Test iterator and stuff */
  bool test();
  /**
  Add map element */
  list<UMapObj>::iterator add();
  /**
  Add map element */
  list<UMapObj>::iterator add(double x, double y, // position
                 double h,           // heading
                 UMatrix4 * zQ,      // measurement squared error estimate
                 UTime atTime        // position time
                 );
  /**
  Add new empty map object and return pointer to new object */
  UMapObj * addObj();
  /**
  Add new map object and return pointer to new object */
  UMapObj * addObj(double x, double y, // position
                 double h,           // heading
                 UMatrix4 * zQ,      // measurement squared error estimate
                 UTime atTime        // position time
                 );
  /**
  Add a new object to be a copy of this object including
  any specifications like robot, guidemark or 3d info.
  Do not copy history or relations. */
  UMapObj * addObjCopy(UMapObj * source);
  /**
  Add an empty object relation */
  UMapRel * addRel();
  /**
  Add an object relation between these
  two objects (must be within this segment).
  The position constraint must be added
  after the relation is added. */
  UMapRel * addRel(UMapObj * ref1, UMapObj * ref2);
  /**
  Update robot position with theis odometer position data */
  //bool updateRobot(UMapRobot * measured);
  /**
  Initialize new robot in map segment */
  bool initializeRobot(UMapRobot * posInMap, UMapRobot * lastUpdate);
  /**
  Find in this map (segment) a segment with this ID.
  Returns a pointer to the segment object, or NULL
  if not found. */
  UMapSegment * findSegment(unsigned long segmentID);
  /**
  Find an object from type and ID in this map segment.
  If type is unknown, then ID is assumed to be serial number. */
  list<UMapObj>::iterator find(unsigned long ID,
                          mapObjType type);
  /**
  Find a map object with this ID and type in this segment.
  Returns NULL if object is not found. */
  UMapObj * findObj(unsigned long ID,
                          mapObjType type = motUnknown);
  /**
  Find object nearest to this position, within a given margin.
  Returns NULL if no object were found. */
  UMapObj * find(double x, double y, double margin, bool andInHistory = true);
  /**
  Update with robot detected guidemark.
  Returns true if robot exist. */
  bool updateGmk(unsigned long robotID, UMapZGmk * measured);
  /**
  Get first object in an iterator */
  list<UMapObj>::iterator begin() { return mapList.begin();};
  /**
  Get first object in an iterator */
  list<UMapObj>::iterator end() { return mapList.end();};
  /**
  Get first object in relation iterator */
  list<UMapRel>::iterator beginRel() { return mapRelations.begin();};
  /**
  Get end object in relation iterator */
  list<UMapRel>::iterator endRel() { return mapRelations.end();};
  /**
  Get serial number */
  unsigned long getNextSerial() {return nextSerial;};
  /**
  Set next serial - that is initialize. */
  inline void setNextSerial(unsigned long next) { nextSerial = next;};
  /**
  Set map limits - that is initialize.
  Map limits are assumed to be in meter. */
  inline void setMapLimits(double xmin, double xmax, double ymin, double ymax)
  {
    minX = xmin; maxX = xmax; minY = ymin; maxY = ymax;
  };
  /**
  Update limits with this element */
  void updateLimits(double x, double y);
  /**
  Update limits with all elements */
  void updateLimits();
  /**
  Get segment ID.
  This ID must be unique among segment ID's.
  The parent segment (map) must assign sequential ID numbers. */
  inline unsigned long getID() { return ID;};
  /**
  Set segment ID.
  This ID must be unique among segment ID's.
  The parent segment (map) must assign sequential ID numbers. */
  inline void setID(unsigned long serial) { ID = serial;};
  /**
  Get segment name.
  Name is descriptive for the segment, but is not assumed to be unique. */
  inline char * getName() { return name;};
  /**
  Set segment name.
  Name has a maximum length of UMapSegment::MAX_SEGMENT_NAME_LNG.
  If given name is longer, then it is truncated,
  Name is descriptive for the segment, but is not assumed to be unique. */
  void setName(char * toName);
  /**
  Get limit for objects on map. */
  inline double getMinX() {return minX;};
  /**
  Get limit for objects on map. */
  inline double getMaxX() {return maxX;};
  /**
  Get limit for objects on map. */
  inline double getMinY() {return minY;};
  /**
  Get limit for objects on map. */
  inline double getMaxY() {return maxY;};
  /**
  Save map in a html-like format to this qualified
  filename (and path).
  Returns true if saved. */
  bool save(const char * filename, const char * path);
  /**
  Save this segment and all the elements
  in the segment to this open file.
  Returns true is saved (or nothing to save). */
  bool save(Uxml3D * fmap);
  /**
  Load map from this file.
  Returns true if loaded. */
  bool load(const char * filename, const char * path);
  /**
  Load map segment from this xml-file class.
  Returns true if loaded. */
  bool load(Uxml3D * fxmap);

public:
  /**
  Length of name buffer */
  static const int MAX_SEGMENT_NAME_LNG = 100;

protected:
  /**
  Parent object if this is a local map
  If top level map, then parent == NULL. */
  UMapObj * parent;
  /**
  Map segment ID */
  unsigned long ID;
  /**
  List structure with map. */
  list<UMapObj> mapList;
  /**
  next available serial number. */
  unsigned long nextSerial;
  /**
  Map limits in local coordinates */
  double minX, maxX, minY, maxY;
  /**
  Segment name */
  char name[MAX_SEGMENT_NAME_LNG];
  /**
  Relations, that further determines the position
  relations of items in map.
  This is the owner of the relations. */
  list<UMapRel> mapRelations;
public:
  /**
  Has this map segment a known position
  relative to map?
  For top level segment this should be false.
  For local maps this can be false, until
  local map is localized in higher level map. */
  bool hasPosition;
};







#endif
