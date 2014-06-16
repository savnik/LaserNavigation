/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
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

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>
#include <urob4/usmltag.h>

#include "urespcp.h"

#include <pcl/io/pcd_io.h>



///////////////////////////////////////////

bool UPcpItem::isA(const char * thisname)
{
  return (strcasecmp(name, thisname) == 0);
}

///////////////////////////////////////////

bool UPcpItem::match(const char * thisname)
{
  const char * p1;
  bool result;
  int n;
  //
  p1 = strchr(thisname, '*');
  if (p1 != NULL)
  {
    n = p1 - thisname;
    if ((unsigned int)n <= strlen(name))
      result = strncasecmp(name, thisname, n) == 0;
    else
      result = false;
  }
  else
    result = isA(thisname);
  return result;
}

///////////////////////////////////////////

void UPcpItem::setName(const char * newname)
{
  strncpy(name, newname, MNL);
}

/////////////////////////////////////////////////////////

const char * UPcpItem::print(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt - n, "%s %s has %d points, closed=%s, color=%s\n", 
           preStr, name, getPointsCnt(), bool2str(isPolygon()), bool2str(hasColor()));
  //
  return buff;
}

//////////////////////////////////////////////////////////

const char * UPcpItem::codeXML(char * buf, const int bufCnt)
{
  char * p1 = buf;
  int i, n = 0;
  const char * TAG_NAME = "pcpitem";
  const char * cType = "xyz";
  //
  if (bufCnt < getNeededBufferSize())
    fprintf(stderr, "UPcpItem::codeXML: buffer too small just %d bytes left, needs %d - skipping %s\n", 
            bufCnt, getNeededBufferSize(), name);
  else 
  {
    if (isXYZRGB())
      cType = "xyzrgb";
    snprintf(p1, bufCnt - n, "<%s name=\"%s\" cnt=\"%d\" type=\"%s\" cooSys=\"%d\" "
             "relPoseUse=\"%s\" pose6d=\"%.3f %.3f %.3f %.5g %.5g %.5g\" tod=\"%lu.%06lu\">\n",
             TAG_NAME, name, getPointsCnt(), cType,
             cooSys, bool2str(relPoseUse), relPose.x, relPose.y, relPose.z, relPose.Omega, relPose.Phi, relPose.Kappa, 
             sensorTime.getSec(), sensorTime.getMicrosec());
    n += strlen(buf);
    p1 = &buf[n];
    //
    for (i = 0; i < getPointsCnt(); i++)
    {
      if (isXYZRGB())
        snprintf(p1, bufCnt - n, "<pnt v=\"%.3f %.3f %.3f 0x%x\"/>\n", 
                 xyzrgb->points[i].x, xyzrgb->points[i].y, xyzrgb->points[i].z, xyzrgb->points[i].rgba);
      else
        snprintf(p1, bufCnt - n, "<pnt v=\"%.3f %.3f %.3f\"/>\n", 
                 xyz->points[i].x, xyz->points[i].y, xyz->points[i].z);
      n += strlen(p1);
      p1 = &buf[n];
      if (n > bufCnt - (12 + 50))
        // not room for more points
        break;
    }
    // end tag
    snprintf(p1, bufCnt - n, "</%s>\n", TAG_NAME);
    if (n >= bufCnt - 1)
      fprintf(stderr, "Point cloud pack overflow, missed %d points\n", getPointsCnt() - i);
  }
  //
  return buf;
}

///////////////////////////////////////////////////////

bool UPcpItem::copy(UPcpItem * source)
{
  if (not source->isEmpty())
  {
    makeCloud(source->getCloudType());
    switch (cloudType)
    {
      case PointXyz:
        xyz->points.resize(source->getPointsCnt());
        for (int i = 0; i < source->getPointsCnt(); i++)
          xyz->points[i] = source->xyz->points[i];
        break;
      case PointXyzRgb:
        xyzrgb->points.resize(source->getPointsCnt());
        for (int i = 0; i < source->getPointsCnt(); i++)
          xyzrgb->points[i] = source->xyzrgb->points[i];
        break;
      default:
        break;
    }
    strncpy(name, source->name, MNL);
    relPose = source->relPose;
    relPoseUse = source->relPoseUse;
    cooSys = source->cooSys;
    updateTime = source->updateTime;
    sensorTime = source->sensorTime;
  }
  return true;
}

////////////////////////////////////////////////

void UPcpItem::copy(pcl::PointCloud<pcl::PointXYZ> * source)
{
  makeCloud(PointXyz);
  xyz->points.resize(source->points.size());
  for (int i = 0; i < (int)source->points.size(); i++)
    xyz->points[i] = source->points[i];
}

////////////////////////////////////////////////

void UPcpItem::copy(pcl::PointCloud<pcl::PointXYZRGB> * source)
{
  makeCloud(PointXyzRgb);
  xyzrgb->points.resize(source->points.size());
  for (int i = 0; i < (int)source->points.size(); i++)
    xyzrgb->points[i] = source->points[i];
}

/////////////////////////////////////////////////////

bool UPcpItem::savePCD(const char * filename)
{
  int err;
  if (cloudType == PointXyzRgb and xyzrgb != NULL)
    err = pcl::io::savePCDFileASCII (filename, *xyzrgb);
  else if (xyz != NULL)
    err = pcl::io::savePCDFileASCII (filename, *xyz);
  else
    err = -1;
  return err == 0;
}


/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void UResPcp::UResPcpInit()
{
  setLogName("pcp");
  //openLog();
  // create status variables
  createBaseVar();
  verbose = false;
  pcpsCnt = 0;
}

///////////////////////////////////////////

UResPcp::~UResPcp()
{
  int i;
  for (i = 0; i < pcpsCnt; i++)
  {
    delete pcps[i];
  }
}

///////////////////////////////////////////

void UResPcp::createResources()
{
  //UCmdExe * core;
  //
  //core = getCorePointer();
  //UResBase::createResources();
}

///////////////////////////////////////////

const char * UResPcp::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  //
  getList(preString, buff, buffCnt);
  return buff;
}

///////////////////////////////////////////

void UResPcp::createBaseVar()
{
  varPolyCnt = addVar("pcpCnt",  0.0, "d", "(r) number of established pcp items");
  varCallDispOnNewData = addVar("callDispOnNewData", 1.0, "d", "(rw) make a call to 'disp.newData()' and 'view.newData' when any cloud has changed (if true).");
  varUpdTime = addVar("updateTime", 0.0, "d", "(r) time of last cloud update");
  //
  //  methods
  addMethod("addPoint", "sdd", "Add a pcpline point to a pcp-item. If the item do not exist it is created. First parameter is the item name, next 2 is x,y (or (E,N)) position");
  addMethod("getPoint", "sd", "Get a point from a pcp-item. First parameter is the item name, second is the position number (index). "
      "The point is returned into a 'UVariable' structure");
  addMethod("getPointCnt", "s", "Get a point count for a pcp-item");
  addMethodV("addPoint", "sc", "Add a point to a pcp-item. If the item do not exist it is created. First parameter is the item name, next is taken as a 2D position - (x,y) or (E,N)");
  addMethodV("del", "s", "Delete a pcp item with this name.");
  addMethodV("delPoint", "sd", "Delete a point from a pcp-item. "
      "First parameter is the item name, next is index [0..cnt-1] to "
          "the item to delete.");
  addMethodV("getPoint", "sd", "Get a pcpline point from a pcp-item. "
      "First parameter is the item name, next is the item to delete");
//   addMethodV("isInside", "sc", "Is this pose inside the cloud, "
//       "1st parameter is item name, 2nd is robot pose in item coordinates. "
//       "returns true if pose is inside the (convex) cloud.");
//   addMethodV("isInside", "scc", "Is this position relative to robot inside the cloud. "
//       "1st parameter is item name, 2nd is robot pose in item coordinates, 3rd is a position relative to robot. "
//       "returns true if position is inside the (convex) cloud.");
  addMethodV("defined", "s", "Is this pcp-item defined.");
  addMethodV("setRefCoord", "sd", "Set the relative coordinate system for "
      "this pcp item (0=odoPose, 1=utmPose, 2=mapPose). "
      "NB! must be defined with at least one point.");
  addMethodV("setOpen", "s", "Set the item coordinates as a pcp-line. NB! must be defined");
  addMethodV("setClosed", "s", "Set the item coordinates as a cloud - i.e. connect first and last point");
  addMethodV("setCloud", "sc", "Set named cloud from this source cloud (any existing pologon of this name is deleted)");
  addMethodV("setCloud", "scd", "Set named cloud and coordinate reference ('d' is 0=odo, 1=utm, 2=map) from this source cloud (any existing pologon of this name is deleted)");
  addMethodV("getCloud", "s", "Get pointer to a named cloud owned by point cloud pool (PCP) - will be created if none exist");  
  addMethodV("getCloud", "sd", "Get pointer to a named cloud owned by PCP ('d' is 0=as is, 1=PointXYZ cloud, 2=PointXYZRGB cloud type)");  
  addMethodV("updated", "s", "Set point cloud pool as updated - should be called if a point cloud is updated by another plugin");
}

//////////////////////////////////////////////

bool UResPcp::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  UPcpItem * pi;
  int i;
  UPosition pos;
  UVariable * var;
  bool isOK;
  // evaluate standard functions
  if ((strcasecmp(name, "addPoint") == 0) and (strcmp(paramOrder, "sdd") == 0))
  {
    pi = add(strings[0], pars[0], pars[1]);
    // return result - if a location is provided
    if (value != NULL)
      *value = (pi != NULL);
      // it is good praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
      *returnStructCnt = 0;
  }
  else if ((strcasecmp(name, "getPoint") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    isOK = false;
    pi = getItem(strings[0]);
    if (pi != NULL)
    {
      i = roundi(pars[0]);
      if (i < pi->getPointsCnt() and i >= 0)
      {
        isOK = true;
        pos = pi->getPoint(i);
      }
    }
    if (not isOK)
      pos.clear();
    // return result - if a location is provided
    if (value != NULL)
      *value = isOK;
      // it is goot praktice to set count of returned structures to 0
    if (returnStructCnt != NULL)
    {
      if (returnStruct[0]->isA("var"))
      {
        var = (UVariable*)returnStruct[0];
        var->set3D(&pos);
        *returnStructCnt = 1;
      }
    }
  }
  else
    // call name is unknown
    result = false;
  return result;
}

///////////////////////////////////////////////////////////////

bool UResPcp::methodCallV(const char * name, const char * paramOrder,
                                UVariable * params[],
                                UDataBase ** returnStruct,
                                int * returnStructCnt)
{
  bool result = true;
  UPcpItem * pi;
  UPcpItem * pcp;
  UDataBase * db;
  int i, retCnt = 0;
  UPosition pos;
  UVariable buffer;
  UVariable * returnVar = &buffer;
  bool isOK;
  UPose po1, po2;
  // evaluate standard functions
  if (returnStruct != NULL and returnStructCnt != NULL)
  { // there is a return pointerlist - get size
    retCnt = *returnStructCnt;
    if (returnStruct[0] != NULL)
    { // pointer has an item already, if a standard variable, then convert.
      if (returnStruct[0]->isA("var"))
      { // return variable for a (rule) function call
        returnVar = (UVariable*)returnStruct[0];
        // in this case one return calue is expected only
        *returnStructCnt = 1;
      }
    }
  }
  //
  // test available methods
  if ((strcasecmp(name, "addPoint") == 0) and (strcmp(paramOrder, "sc") == 0))
  {
    pos = params[1]->get3D();
    pi = add(params[0]->getValues(), pos.x, pos.y, pos.z);
    isOK = pi != NULL;
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "updated") == 0) and (strcmp(paramOrder, "s") == 0))
  { // a point cloud is updated by another plugin - update variable structure
//     pi = getItem(params[0]->getValues());
//     isOK = pi != NULL;
//     if (isOK)
    {
      // pi->setUpdated();
      gotNewData();
    }
  }
  else if ((strcasecmp(name, "del") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    isOK = del(params[0]->getValues());
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "delPoint") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    pi = getItem(params[0]->getValues());
    isOK = pi != NULL;
    if (isOK)
      pi->remove(params[1]->getInt());
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "getPoint") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    isOK = false;
    pi = getItem(params[0]->getValues());
    if (pi != NULL)
    { // item is found, so get position
      i = roundi(params[1]->getInt());
      if (i < pi->getPointsCnt() and i >= 0)
      {
        isOK = true;
        pos = pi->getPoint(i);
      }
    }
    if (not isOK)
      pos.clear();
    returnVar->set3D(&pos);
  }
//   else if ((strcasecmp(name, "isInside") == 0) and (strcmp(paramOrder, "sc") == 0))
//   {
//     isOK = false;
//     pos = params[1]->get3D();
//     pi = getItem(params[0]->getValues());
//     if (pi != NULL)
//     { // item is found, test
//       isOK = pi->isInsideConvex(pos.x, pos.y, 0.0);
//     }
//     returnVar->setBool(isOK);
//   }
//   else if ((strcasecmp(name, "isInside") == 0) and (strcmp(paramOrder, "scc") == 0))
//   {
//     isOK = false;
//     po1 = params[1]->getPose(); // pose of robot
//     po2 = params[2]->getPose(); // position relative to robot
//     po1 = po1 + po2;
//     pi = getItem(params[0]->getValues());
//     if (pi != NULL)
//     { // item is found, test
//       isOK = pi->isInsideConvex(po1.x, po1.y, 0.0);
//     }
//     returnVar->setBool(isOK);
//   }
  else if ((strcasecmp(name, "defined") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    pi = getItem(params[0]->getValues());
    isOK = (pi != NULL);
    returnVar->setBool(isOK);
  }
  else if ((strcasecmp(name, "setRefCoord") == 0) and (strcmp(paramOrder, "sd") == 0))
  {
    pi = getItem(params[0]->getValues());
    isOK = pi != NULL;
    if (isOK)
      pi->cooSys = params[1]->getInt();
    returnVar->setBool(isOK);
  }
  else if ((strcasecmp(name, "setOpen") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    pi = getItem(params[0]->getValues());
    isOK = pi != NULL;
    if (isOK)
      pi->setAsPolyline();
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "setClosed") == 0) and (strcmp(paramOrder, "s") == 0))
  {
    pi = getItem(params[0]->getValues());
    isOK = pi != NULL;
    if (isOK)
      pi->setAsPolygon();
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "setCloud") == 0) and
           (strcmp(paramOrder, "sc") == 0 or
            strcmp(paramOrder, "scd") == 0))
  {
    isOK = false;
    pi = getItem(params[0]->getValues());
    db = params[1];
    if (db->isAlsoA("pcpitem"))
      pcp = (UPcpItem*) db;
    else
      pcp = NULL;
    isOK = pcp != NULL;
    if (isOK and pi == NULL)
    { // item is found, so get position
      pi = add(params[0]->getValues());
      isOK = pi != NULL;
    }
    if (isOK)
    {
      isOK = pi->copy(pcp);
      if (strlen(paramOrder) >= 3)
        // get also coordinate system reference
        pi->cooSys = params[2]->getInt();
      pi->setUpdated();
    }
    returnVar->setBool(isOK);
    gotNewData();
  }
  else if ((strcasecmp(name, "getCloud") == 0) and
           (strcmp(paramOrder, "sd") == 0 or strcmp(paramOrder, "s") == 0))
  {
    isOK = false;
    pi = getItem(params[0]->getValues());
    if (pi == NULL)
      pi = add(params[0]->getValues());
    if (pi != NULL and retCnt > 0)
    { // a cloud structure is available return pointer available
      if (strlen(paramOrder) == 2)
      { // type value available - so set
        int typ = params[1]->getInt();
        if (typ > 0)
        { // type must be exact, so make sure
          if (typ == 2)
            pi->makeCloud(UPcpItem::PointXyzRgb);
          else
            pi->makeCloud(UPcpItem::PointXyz);
        }
      }
      returnStruct[0] = pi;
    }
  }
  else
    // call name is unknown (in conflict with announced methods)
    result = false;
  //
  return result;
}

///////////////////////////////////////////////////////////

UPcpItem * UResPcp::add(const char * name)
{
  UPcpItem * result;
  int i;
  //
  lock();
  result = getItem(name);
  if (result == NULL)
  { // does not exist, so create new pcp
    for (i = 0; i < pcpsCnt; i++)
    {
      if (pcps[i]->getPointsCnt() == 0)
      {
        result = pcps[i];
        break;
      }
    }
    if (result == NULL and pcpsCnt < MAX_POLYS_CNT)
    {
      result = new UPcpItem();
      pcps[pcpsCnt++] = result;
      varPolyCnt->setInt(pcpsCnt);
    }
    if (result != NULL)
    { // initialize
      result->setAsPolyline();
      result->setName(name);
    }
  }
  else if (result->getPointsCnt() == 0)
    // exist, but is empty - reuse
    result->setAsPolyline();
  unlock();
  //
  return result;
}

//////////////////////////////////////////////

UPcpItem * UResPcp::add(const char * name,
                          const double x, const double y, const double z)
{
  UPosition pos(x, y, z);
  UPcpItem * result;
  int i;
  //
  lock();
  result = getItem(name);
  if (result == NULL)
  { // does not exist, so create new pcp
    for (i = 0; i < pcpsCnt; i++)
    {
      if (pcps[i]->getPointsCnt() == 0)
      {
        result = pcps[i];
        if (not result->isXYZ())
          // wrong type
          result->deleteCloud();
        break;
      }
    }
    if (result == NULL and pcpsCnt < MAX_POLYS_CNT)
    {
      result = new UPcpItem();
      pcps[pcpsCnt++] = result;
      varPolyCnt->setInt(pcpsCnt);
    }
    if (result != NULL)
    { // initialize
      result->setAsPolyline();
      result->setName(name);
    }
  }
  if (result != NULL)
  {
    if (not result->isXYZ())
      result->makeCloud(UPcpItem::PointXyz);
    result->add(pos.x , pos.y, pos.z);
    result->setUpdated();
  }
  unlock();
  return result;
}

//////////////////////////////////////////////////////////

UPcpItem * UResPcp::add(const char * name, UPcpItem::POINT_CLOUD_TYPE type)
{
  UPcpItem * result;
  int i;
  //
  lock();
  result = getItem(name);
  if (result == NULL)
  { // does not exist, so create new pcp
    for (i = 0; i < pcpsCnt; i++)
    {
      if (pcps[i]->getPointsCnt() == 0)
      {
        result = pcps[i];
        break;
      }
    }
    if (result == NULL and pcpsCnt < MAX_POLYS_CNT)
    {
      result = new UPcpItem();
      pcps[pcpsCnt++] = result;
      varPolyCnt->setInt(pcpsCnt);
    }
    if (result != NULL)
    { // initialize
      result->setAsPolyline();
      result->setName(name);
    }
  }
  if (result != NULL)
  {
    if (result->getCloudType() != type)
    { // wrong type
      result->clear();
      result->makeCloud(type);
    }
    result->setUpdated();
  }
  unlock();
  return result;
}

//////////////////////////////////////////////

UPcpItem * UResPcp::getItem(const char * name)
{
  int i;
  UPcpItem ** ki = pcps;
  UPcpItem * result = NULL;
  //
  for (i = 0; i < pcpsCnt; i++)
  {
    if ((*ki)->isA(name))
    {
      result = *ki;
      break;
    }
    ki++;
  }
  return result;
}

//////////////////////////////////////////////

UPcpItem * UResPcp::getItemXYZ(const char * name)
{
  int i;
  UPcpItem ** ki = pcps;
  UPcpItem * result = NULL;
  //
  for (i = 0; i < pcpsCnt; i++)
  {
    if ((*ki)->isA(name))
    {
      result = *ki;
      break;
    }
    ki++;
  }
  if (result == NULL)
    result = add(name);
  if (result != NULL)
  {
    result->makeCloud(UPcpItem::PointXyz);
    result->setName(name);
  }
  return result;
}

//////////////////////////////////////////////

UPcpItem * UResPcp::getNext(int p1, int * p2, const char * name)
{
  int i;
  UPcpItem * ki;
  UPcpItem * result = NULL;
  const char * pw;
  bool hasWildcard;
  bool found;
  //
  pw = strchr(name, '*');
  hasWildcard = (pw != NULL);
  for (i = p1; i < pcpsCnt; i++)
  {
    ki = pcps[i];
    if (hasWildcard)
      found = ki->match(name);
    else
      found = ki->isA(name);
    if (found)
    {
      result = ki;
      break;
    }
  }
  i++;
  if (p2 != NULL)
  {
    if (i < pcpsCnt)
      *p2 = i;
    else
      *p2 = -1;
  }
  return result;
}

//////////////////////////////////////////////

bool UResPcp::del(const char * name)
{
  UPcpItem * pi = NULL;
  int ni;
  //
  lock();
  ni = 0;
  while (ni >= 0)
  {
    pi = getNext(ni, &ni, name);
    if (pi != NULL)
      pi->clear();
  }
  unlock();
  return true;
}

/////////////////////////////////////////////

const char * UResPcp::getList(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  int i;
  UPcpItem ** ki = pcps;
  //
  snprintf(p1, buffCnt, "%s - %d items\n", preStr, pcpsCnt);
  n += strlen(p1);
  p1 = &buff[n];
  for (i = 0; i < pcpsCnt; i++)
  {
    (*ki)->print("  - ", p1, buffCnt - n);
    n += strlen(p1);
    p1 = &buff[n];
    ki++;
  }
  return buff;
}

//////////////////////////////////////////////////////////////

void UResPcp::handleNewData(USmlTag * tag)
{ // handle pcp message
  const int MSL = 200;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  USmlTag nTag, pTag;
  int cntItems = 0;
  const int MNL = 32;
  char name[MNL];
  UPcpItem * pi;
  //
  while (tag->getNextAttribute(att, val, MSL))
  {
    if ((strcasecmp(att, "warning") == 0) or
         (strcasecmp(att, "info") == 0))
    {
      // no data printf("*** failed: %s\n", val);
      break;
    }
//     else if (strcasecmp(att, "cnt") == 0)
//     {
//       cnt = strtol(val, NULL, 10);
//     }
  }
  if (tag->isAStartTag())
  { // there is nested structures
    while (tag->getNextTag(&nTag, 200))
    {
      if (nTag.isTagA("pcpitem"))
      { // get all support data
        pi = NULL;
        if (nTag.getAttValue("name", name, MNL))
        { // must have a name
          // get cloud item to unpack to
          pi = getItem(name);
          if (pi == NULL)
            pi = add(name);
        }
        if (pi != NULL)
        { // optional coordinate system
          int cnt = 0;
          pi->lock();
          pi->clear();
          nTag.getAttInteger("cooSys", &pi->cooSys, 1);
          if (nTag.getAttValue("pose6d", val, MSL))
          {
            UPosRot p;
            int n = sscanf(val, "%lf %lf %lf %lf %lf %lf", &p.x, &p.y, &p.z, &p.Omega, &p.Phi, &p.Kappa);
            if (n == 6)
              pi->relPose = p;
          }
          nTag.getAttBool("relPoseUse", &pi->relPoseUse, false);
          if (not nTag.getAttTime("tod", &pi->updateTime, 0))
            pi->sensorTime.now();
          nTag.getAttInteger("cnt", &cnt, 0);
          nTag.getAttValue("type", val, MSL);
          if (strcmp(val, "xyzrgb") == 0)
            pi->makeCloud(UPcpItem::PointXyzRgb);
          else
            pi->makeCloud(UPcpItem::PointXyz);
          pi->updateTime.now();
          // get rest of points
          while (nTag.getNextTag(&pTag, 200))
          { // add points
            if (pTag.isTagA("pnt"))
            { // get all support data
              pTag.getAttValue("v", val, MSL);
              if (pi->cloudType == UPcpItem::PointXyzRgb)
              {
                pcl::PointXYZRGB p;
                sscanf(val, "%f %f %f %x", &p.x, &p.y, &p.z, &p.rgba);
                pi->xyzrgb->push_back(p);
              }
              else
              {
                pcl::PointXYZ p;
                sscanf(val, "%f %f %f", &p.x, &p.y, &p.z);
                pi->xyz->push_back(p);
              }
            }
            else if (pTag.isAStartTag())
              // is an unwanted tag goup - skip
              pTag.skipToEndTag(1000);
            // remember to test for endflag too
            else if (pTag.isTagAnEnd(nTag.getTagName()))
              break;
          }
          pi->unlock();
          cntItems++;
        }
      }
      else if (nTag.isAStartTag())
        // is an unwanted tag goup - skip
        nTag.skipToEndTag(1000);
      // remember to test for endflag too
      else if (nTag.isTagAnEnd(tag->getTagName()))
        break;
    }
  }
  if (cntItems > 0)
  {
/*    if (pcpsCnt != cnt)
      printf("Has %d pcpItems (%d updated), but sender has %d!\n", pcpsCnt, cntItems, cnt);*/
    varPolyCnt->setInt(pcpsCnt);
    gotNewData();
  }
}

/////////////////////////////////////////////////////

// const char * UResPcp::codePolysXml(const char * tagName,
//                                     char * buf, const int bufCnt, UTime updatedSince)
// {
//   char * p1 = buf;
//   int n = 0, i;
//   UPcpItem * pi;
//   //
//   snprintf(p1, bufCnt - n, "<%s name=\"pcpItems\" cnt=\"%d\">\n", tagName, pcpsCnt);
//   n += strlen(p1);
//   p1 = &buf[n];
//   for (i = 0; i < pcpsCnt; i++)
//   {
//     pi = pcps[i];
//     if (pi != NULL)
//     {
//       if (pi->updateTime >= updatedSince)
//       {
//         pi->codeXML(p1, bufCnt - n);
//         n += strlen(p1);
//         p1 = &buf[n];
//       }
//     }
//     if (bufCnt - n < 80)
//       break;
//   }
//   snprintf(p1, bufCnt - n, "</%s>\n", tagName);
//   return buf;
// }

//////////////////////////////////////////////////

bool UResPcp::codePcpXml(const char * tagName,
                                    char * buf, const int bufCnt, int idx)
{
  char * p1 = buf;
  int n = 0;
  UPcpItem * pi;
  bool isOK = false;
  //
  pi = pcps[idx];
  if (pi != NULL)
  {
    snprintf(p1, bufCnt - n, "<%s name=\"pcpItems\" cnt=\"1\">\n", tagName);
    n += strlen(p1);
    p1 = &buf[n];
    pi->codeXML(p1, bufCnt - n);
    n += strlen(p1);
    p1 = &buf[n];
    snprintf(p1, bufCnt - n, "</%s>\n", tagName);
    isOK = true;
  }
  return isOK;
}

/////////////////////////////////////////////

void UResPcp::gotNewData()
{ //
  char * dataType = (char*)"pcp";
  double v = 0.0;
  //
  if (varCallDispOnNewData->getBool())
  { // tell display system of new image
    callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
  }
  varUpdTime->setTimeNow();
}

////////////////////////////////////////////////

