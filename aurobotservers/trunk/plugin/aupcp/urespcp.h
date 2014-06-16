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

#ifndef URESPCP_H
#define URESPCP_H

#include <cstdlib>

#include <urob4/uresvarpool.h>
#include <urob4/uresposehist.h>
#include <urob4/userverpush.h>
#include <urob4/ulogfile.h>
#include <urob4/uclienthandler.h>
#include <urob4/uvarcalc.h>
#include <ugen4/upolygon.h>
#include <urob4/uresifbase.h>

#define BOOST_SYMBOL_VISIBLE
#include <pcl/point_types.h>

#include "upcpitem.h"

/**
This is the shared resource class.

This class implements a point cloud pool (PCP)
All pool items has an ID - a text string (< 32 chars) and is also used by the AUVIEW opengl 3D viewer)
The items can be retrieved by other plugins for on-line viewing/saving through this plugin

@author Christian Andersen
*/
class UResPcp : public UResIfBase, public ULogFile
{ // NAMING convention recommend that a resource class
  // starts with URes (as in UResPcp) followed by
  // a descriptive extension for this specific resource
public:
  /**
  Constructor */
  UResPcp()
  { // set name and version
    setResID("pcp", 1893);
    UResPcpInit();
  };
  /**
  Destructor */
  virtual ~UResPcp();
  /**
   * Initialize resource */
  void UResPcpInit();
  /**
  Function, that shall return a string with all handled XML tags,
  i.e. should return "gmk gmk2d guidemark", if incoming XML tags
  starting with any of these three keywords are handled
  by this function */
  virtual const char * commandList()
  { return "pcp"; };
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
   * Get pcp item pointer
   * \param name is the name of the pcp
   * \returns NULL if a pcp with this name is not found */
  UPcpItem * getItem(const char * name);
  /**
   * Get pcp item pointer
   * \param name is the name of the pcp
   * \returns a pointer to an existing or if not found, a pointer to an empty item of this name */
  UPcpItem * getItemXYZ(const char * name);
  /**
   * get an item by index, NB no range check
   * \param idx the index to the items available, should be
   * in the range 0..getPolysCnt()-1.
   * \returns a pointer to the iten, NB! the pointer may be NULL */
  UPcpItem * getItem(int idx)
  { return pcps[idx]; };
  /**
  * Get pcp item pointer starting at index 'p1'
  * \param p1 is the first pcpgon to search for the next name that matches 'name'
  * \param p2 is set (if not NULL) to the next pcpgon after the found
  * \param name is the name of the pcp
  * \returns NULL if a pcp with this name is not found */
  UPcpItem * getNext(int p1, int * p2, const char * name);
  
  /**
   * Delete a pcp, that is, set vertex count to 0
   * This marks that the pcpgon is empty and the pcpgon slot may be reused.
   * \param name is the name of the pcp
   * \returns true if deleted pcp, false if not existed. */
  bool del(const char * name);
  /**
   * Add a new pcp item
   * \param name is the name of the new item. The name must be unique,
   * \returns a pointer to the new item, or null if it exist already, or could not be added. */
  UPcpItem * add(const char * name);
  /**
   * make a new cloud (XYZRGB) for this source
   * \param source is the source cloud to copy
   * \param name is the name of the new item. The name must be unique,
   * \returns a pointer to the new item, or null if it exist already, or could not be added. */
  UPcpItem * copy(pcl::PointCloud<pcl::PointXYZ> * source, const char * name)
  {
    UPcpItem * pi = getItem(name);
    if (pi == NULL)
    {
      pi = add(name, UPcpItem::PointXyz);
      pi->copy(source);
    }
    return pi;
  }
  /**
   * make a new cloud (XYZRGB) for this source
   * \param source is the source cloud to copy
   * \param name is the name of the new item. The name must be unique,
   * \returns a pointer to the new item, or null if it exist already, or could not be added. */
  UPcpItem * copy(pcl::PointCloud<pcl::PointXYZRGB> * source, const char * name);
  /**
   * Add a new XYZ position to this cloud, creates cloud if none exist.
   * \param name is the name of a new or existing item. Names are unique,
   * \param x,y,z is a position that is added to the end of the item points, z is optional.
   * \returns a pointer to the item, or null if it do not exist and could not be added. */
  UPcpItem * add(const char * name, const double x, const double y, const double z = 0.0);
  /**
   * Add a new (empty) cloud item
   * \param[in] name is the name of a new or existing item. Names are unique,
   * \param[in] type is type of new cloud 
   * \returns pointer to new item */
  UPcpItem * add(const char * name, UPcpItem::POINT_CLOUD_TYPE type);
  /**
   * Get number of current pcp processes
   * \returns number of pcps, active or not */
  int getPolysCnt()
  { return pcpsCnt; };
  /**
   * Get list of current pcps
   * \param preStr is a short string, that is added at the start of the buffer.
   * \param buff is the buffer, where to write the list.
   * \param buffCnt is the size of the buffer
   * \returns a pointer to buff */
  const char * getList(const char * preStr, char * buff, const int buffCnt);
  /**
   * code all pcpgons to an XML message into this buffer
   * \param tagName is the name to the tag, in wich the pcpgons are to be listed.
   * \param buff is the buffer, where to write the code.
   * \param buffCnt is the size of the buffer
   * \param updatedSince only items updated later than this is XML packed
   * \returns a pointer to buff */
//   const char * codePolysXml(const char * tagName,
//                             char * buf,
//                             const int bufCnt,
//                             UTime updatedSince);
  /**
   * code one pcp to an XML message into this buffer
   * \param tagName is the name to the tag, in wich the pcpgons are to be listed.
   * \param buff is the buffer, where to write the code.
   * \param buffCnt is the size of the buffer
   * \param updatedSince only items updated later than this is XML packed
   * \param idx is index to the pcpgon to code.
   * \returns true if the update time is later than updatedSince (else not coded) */
  bool codePcpXml(const char * tagName,
                    char * buf,
                    const int bufCnt,
                    int idx);
  /**
  Tell a display plugin that new data exist - for a potential redraw. */
  void gotNewData();
  /**
   * Save a cloud to a pcd-file */
  bool savePCD(const char * idName, const char * filename)
  {
    UPcpItem * pi = getItem(idName);
    if (pi != NULL)
      return pi->savePCD(filename);
    else
      return false;
  }

protected:
  /**
  Make the variables that will be available to other plugins */
  void createBaseVar();


public:
  /**
   * Is the resource to output verbose message to console */
  bool verbose;

protected:
  /// maximum number of pcps
  static const int MAX_POLYS_CNT = 1000;
  /// array of pcp handles
  UPcpItem * pcps[MAX_POLYS_CNT];
  /// Number of pcps allocated
  int pcpsCnt;
  //
  /// number of established pcp items
  UVariable * varPolyCnt;
  /// inform a potential display plugin, that new data is available
  UVariable * varCallDispOnNewData;
  /// last update time for pcpgons
  UVariable * varUpdTime;
};

#endif

