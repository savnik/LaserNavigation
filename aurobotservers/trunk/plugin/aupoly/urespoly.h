/***************************************************************************
 *   Copyright (C) 2006 by DTU (by Christian Andersen, Lars m.fl.)         *
 *   jca@elektro.dtu.dk                                                    *
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

#ifndef URESPOLY_H
#define URESPOLY_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>
#include <ugen4/upolygon.h>
#include <urob4/uresifbase.h>


/**
 * Named polygon for mission lines */
class UPolyItem : public UPolygon40
{
public:
  /**
   * constructor */
  UPolyItem()
  {
    name[0] = '\0';
    relPoseUse = false;
    cooSys = 1; // 1=utm
    valid = true;
    updateTime.clear();
    relPose.clear();
  };
  /**
   * Destructor */
  ~UPolyItem() {};
  /**
   * Test name of this poly item
   * \param thisname is the name to be tested against
   * \returns true if name is identical - is not case sensitive*/
  bool isA(const char * thisname);
  /**
  * Test name of this poly item against a name, that may hold one final wildcard (a *)
  * i.e. 'foo*' will match 'footprint' and 'foo', but not 'fo'
  * and  'foo*print' will match the same as 'foo*'
  * a '*' will match all
  * \param wildname is the name to be tested against (with optionally one wildcard '*')
  * \returns true if name matches 'thisname' - is not case sensitive*/
  bool match(const char * wildname);
  /**
   * Set name of this poly item
   * \param newname is the new name for the polygon - default is empty - max 32 characters (MNL) */
  void setName(const char * newname);
  /**
   * Print current status to this buffer string
   * \param preStr start by inserting this string into buffer
   * \param buff start of buffer
   * \param bufCnt length of buffer
   * \returns a pointer to the buffer */
  const char * print(const char * preStr, char * buff, const int buffCnt);
  /**
   * code this poly item in XML format
   * \param buff start of buffer
   * \param bufCnt length of buffer
   * \returns a pointer to the buffer */
  const char * codeXML(char * buf, const int bufCnt);
  /**
   * set polygon as updated, i.e. set the update time to now. */
  inline void setUpdated()
  { updateTime.now(); };


public:
  /** size of names */
  static const int MNL = 32;
  /** polygon name */
  char name[MNL];
  /** coordinate system 0=odo, 1=utm, 2=map */
  int cooSys;
  /** if relative pose, the coordinates are relative to this
   * pose (in the specified coordinate system) */
  UPose relPose;
  /** use the relative pose */
  bool relPoseUse;
  /**
   * Is this item valid (or deleted) */
  bool valid;
  /**
   * Update time for this item, so that updated only can be send */
  UTime updateTime;
};


/**
This is the shared resource class.
It must enherit from the resource base class (or one of its decendents) as shown.

@author Christian Andersen
*/
class UResPoly : public UResIfBase, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResPoly) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResPoly()
  { // set name and version
    setResID("poly", 410);
    UResPolyInit();
  };
  /**
  Destructor */
  virtual ~UResPoly();
  /**
   * Initialize resource */
  void UResPolyInit();
  /**
  Function, that shall return a string with all handled commands,
  i.e. should return "gmk gmk2d guidemark", if commands
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList()
  { return "poly"; };
  /**
  Got fresh data from a stream destined to this function.
  that is, the received XML tag is in the commandList() for this resource. */
  virtual void handleNewData(USmlTag * tag);
  /**
   * Create any resources that this modules needs
   * This method is called after the resource is registred by the server. */
  virtual void createResources();
  /**
  print status to a string buffer */
  virtual const char * print(const char * preString, char * buff, int buffCnt);
  /**
  The varPool has methods, and a call to one of these is needed.
  Do the call now and return (a double sized) result in 'value' and
  return true if the method call is allowed.
  * If the returnStruct and returnStructCnt is not NULL, then
    a number (no more than initial value of returnStructCnt) of
    structures based on UDataBase may be returned into returnStruct array
    of pointers. The returnStructCnt should be modified to the actual
    number of used pointers (if needed).
  * If the call is allowed, but the result is invalid for some reason, the
    return value 'value' can be set to som agreed value, (e.g. 0.0 (false) and 1.0 for true). */
  virtual bool methodCall(const char * name, const char * paramOrder,
                  char ** strings, const double * pars,
                  double * value,
                  UDataBase ** returnStruct,
                  int * returnStructCnt);
  /**
   * A varPool method with this class as implementor is called.
   * \param name is the name of the called function
   * \param paramOrder is a string with one char for each parameter in the
   * call - d is double, s is string, c is class object.
   * \param params is an array of variable pointers with the
   * actual parameters, in the order specified by order
   * \param returnStruct is an array of class object pointers that can
   * return values or objects (may be NULL) if no result value is needed (a procedure call)
   * \param returnStructCnt is the number of objects in the returnStruct buffer
   * \return true if the method is recognised (exists),
   * when false is returned a invalid name or parameter list is detected. */
  virtual bool methodCallV(const char * name, const char * paramOrder,
                           UVariable * params[],
                           UDataBase ** returnStruct = NULL,
                           int * returnStructCnt = NULL);
public:
  // Public functions that this resource provides for all users.
  /**
   * Get poly item pointer
   * \param name is the name of the poly
   * \returns NULL if a poly with this name is not found */
  UPolyItem * getItem(const char * name);
  /**
   * get an item by index, NB no range check
   * \param idx the index to the items available, should be
   * in the range 0..getPolysCnt()-1.
   * \returns a pointer to the iten, NB! the pointer may be NULL */
  UPolyItem * getItem(int idx)
  { return polys[idx]; };
  /**
  * Get poly item pointer starting at index 'p1'
  * \param p1 is the first polygon to search for the next name that matches 'name'
  * \param p2 is set (if not NULL) to the next polygon after the found
  * \param name is the name of the poly
  * \returns NULL if a poly with this name is not found */
  UPolyItem * getNext(int p1, int * p2, const char * name);
  
  /**
   * Delete a poly, that is, set vertex count to 0
   * This marks that the polygon is empty and the polygon slot may be reused.
   * \param name is the name of the poly
   * \returns true if deleted poly, false if not existed. */
  bool del(const char * name);
  /**
   * Add a new poly item
   * \param name is the name of the new item. The name must be unique,
   * \returns a pointer to the new item, or null if it exist already, or could not be added. */
  UPolyItem * add(const char * name);
  /**
   * Add a new poly item
   * \param name is the name of a new or existing item. Names are unique,
   * \param x,y,z is a position that is added to the end of the item points, z is optional.
   * \returns a pointer to the item, or null if it do not exist and could not be added. */
  UPolyItem * add(const char * name, const double x, const double y, const double z = 0.0);
  /**
   * Get number of current poly processes
   * \returns number of polys, active or not */
  int getPolysCnt()
  { return polysCnt; };
  /**
   * Get list of current polys
   * \param preStr is a short string, that is added at the start of the buffer.
   * \param buff is the buffer, where to write the list.
   * \param buffCnt is the size of the buffer
   * \returns a pointer to buff */
  const char * getList(const char * preStr, char * buff, const int buffCnt);
  /**
   * code all polygons to an XML message into this buffer
   * \param tagName is the name to the tag, in wich the polygons are to be listed.
   * \param buff is the buffer, where to write the code.
   * \param buffCnt is the size of the buffer
   * \param updatedSince only items updated later than this is XML packed
   * \returns a pointer to buff */
  const char * codePolysXml(const char * tagName,
                            char * buf,
                            const int bufCnt,
                            UTime updatedSince);
  /**
   * code all polygons to an XML message into this buffer
   * \param tagName is the name to the tag, in wich the polygons are to be listed.
   * \param buff is the buffer, where to write the code.
   * \param buffCnt is the size of the buffer
   * \param updatedSince only items updated later than this is XML packed
   * \param idx is index to the polygon to code.
   * \returns true if the update time is later than updatedSince (else not coded) */
  bool codePolyXml(const char * tagName,
                    char * buf,
                    const int bufCnt,
                    UTime updatedSince,
                    int idx);
  /**
  Tell a display plugin that new data exist - for a potential redraw. */
  void gotNewData();


protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();
  

public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /// maximum number of polys
  static const int MAX_POLYS_CNT = 1000;
  /// array of poly handles
  UPolyItem * polys[MAX_POLYS_CNT];
  /// Number of polys allocated
  int polysCnt;
  //
  /// number of established poly items
  UVariable * varPolyCnt;
  /// inform a potential display plugin, that new data is available
  UVariable * varCallDispOnNewData;
  /// last update time for polygons
  UVariable * varUpdTime;
};

#endif

