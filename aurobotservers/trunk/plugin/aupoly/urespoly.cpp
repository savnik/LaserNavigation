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

#include "urespoly.h"

///////////////////////////////////////////

bool UPolyItem::isA(const char * thisname)
{
  return (strcasecmp(name, thisname) == 0);
}

///////////////////////////////////////////

bool UPolyItem::match(const char * wildname)
{
  const char * p1;
  bool result;
  int n;
  //
  if (true)
  {
    result = wildcmp(wildname, name);
  }
  else
  {
    p1 = strchr(wildname, '*');
    if (p1 != NULL)
    {
      n = p1 - wildname;
      if ((unsigned int)n <= strlen(name))
        result = strncasecmp(name, wildname, n) == 0;
      else
        result = false;
    }
    else
      result = isA(wildname);
  }
  return result;
}

///////////////////////////////////////////

void UPolyItem::setName(const char * newname)
{
  strncpy(name, newname, MNL);
}

/////////////////////////////////////////////////////////

const char * UPolyItem::print(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt - n, "%s %s has %d points, closed=%s, color=%s\n", preStr, name, pointsCnt, bool2str(isPolygon()), color);
  //
  return buff;
}

//////////////////////////////////////////////////////////

const char * UPolyItem::codeXML(char * buf, const int bufCnt)
{
  const int MEL = 100;
  char e[MEL];
  USmlTag nTag;
  //
  if (bufCnt < (100 + 30 * pointsCnt))
    printf("UPolyItem::codeXML: buffer too small just %d bytes left - skipping %s\n", bufCnt, name);
  else if (valid)
  {
    snprintf(e, MEL, "cooSys=\"%d\" relPoseUse=\"%s\" x=\"%.3f\" y=\"%.3f\" h=\"%.5f\" valid=\"true\"",
            cooSys, bool2str(relPoseUse), relPose.x, relPose.y, relPose.h);
    nTag.codePolygon(this, buf, bufCnt, name, e);
  }
  else
  {
    snprintf(buf, bufCnt, "<polygon name=\"%s\" valid=\"false\"/>\n", name);
  }
  return buf;
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void UResPoly::UResPolyInit()
{
  setLogName("poly");
  //openLog();
  // create status variables
  createBaseVar();
  verbose = false;
  polysCnt = 0;
}

///////////////////////////////////////////

UResPoly::~UResPoly()
{
  int i;
  for (i = 0; i < polysCnt; i++)
  {
    delete polys[i];
  }
}

///////////////////////////////////////////

void UResPoly::createResources()
{
  //UCmdExe * core;
  //
  //core = getCorePointer();
  //UResBase::createResources();
}

///////////////////////////////////////////

const char * UResPoly::print(const char * preString, char * buff, int buffCnt)
{ // print status for resource (short 1-3 lines of text followed by a line feed (\n)
  //
  getList(preString, buff, buffCnt);
  return buff;
}

///////////////////////////////////////////

void UResPoly::createBaseVar()
{
  varPolyCnt = addVar("polyCnt",  0.0, "d", "(r) number of established poly items");
  varCallDispOnNewData = addVar("callDispOnNewData", 1.0, "d", "(rw) make a call to 'disp.newData()' when any polygon has changed (if true).");
  varUpdTime = addVar("updateTime", 0.0, "d", "(r) time of last polygon update");
  //
  //  methods
  addMethod("addPoint", "sdd", "Add a polyline point to a poly-item. If the item do not exist it is created. First parameter is the item name, next 2 is x,y (or (E,N)) position");
  addMethod("getPoint", "sd", "Get a point from a poly-item. First parameter is the item name, second is the position number (index). "
      "The point is returned into a 'UVariable' structure");
  addMethod("getPointCnt", "s", "Get a point count for a poly-item");
  addMethodV("addPoint", "sc", "Add a polyline point to a poly-item. If the item do not exist it is created. First parameter is the item name, next is taken as a 2D position - (x,y) or (E,N)");
  addMethodV("del", "s", "Delete a poly item with this name.");
  addMethodV("delPoint", "sd", "Delete a polyline point from a poly-item. "
      "First parameter is the item name, next is index [0..cnt-1] to "
          "the item to delete.");
  addMethodV("getPoint", "sd", "Get a polyline point from a poly-item. "
      "First parameter is the item name, next is the item to delete");
  addMethodV("isInside", "sc", "Is this pose inside the polygon, "
      "1st parameter is item name, 2nd is robot pose in item coordinates. "
      "returns true if pose is inside the (convex) polygon.");
  addMethodV("isInside", "scc", "Is this position relative to robot inside the polygon. "
      "1st parameter is item name, 2nd is robot pose in item coordinates, 3rd is a position relative to robot. "
      "returns true if position is inside the (convex) polygon.");
  addMethodV("defined", "s", "Is this poly-item defined.");
  addMethodV("setRefCoord", "sd", "Set the relative coordinate system for "
      "this poly item (0=odoPose, 1=utmPose, 2=mapPose). "
      "NB! must be defined with at least one point.");
  addMethodV("setOpen", "s", "Set as a poly-line. NB! must be defined");
  addMethodV("setClosed", "s", "Set as a polygon - i.e. connect first and last point");
  addMethodV("setPolygon", "sc", "Set named polygon from this source polygon (any existing pologon of this name is deleted)");
  addMethodV("setPolygon", "scd", "Set named polygon and coordinate reference ('d' = 0=odo, 1=utm, 2=map) from this source polygon (any existing pologon of this name is deleted)");  /*
  else if ((strcasecmp(name, "setRefCoord") == 0) and (strcmp(paramOrder, "sd") == 0))
  else if ((strcasecmp(name, "setOpen") == 0) and (strcmp(paramOrder, "s") == 0))
  else if ((strcasecmp(name, "setClosed") == 0) and (strcmp(paramOrder, "s") == 0))
  */
}

//////////////////////////////////////////////

bool UResPoly::methodCall(const char * name, const char * paramOrder,
                                 char ** strings, const double * pars,
                                 double * value,
                                 UDataBase ** returnStruct,
                                 int * returnStructCnt)
{
  bool result = true;
  UPolyItem * pi;
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

bool UResPoly::methodCallV(const char * name, const char * paramOrder,
                                UVariable * params[],
                                UDataBase ** returnStruct,
                                int * returnStructCnt)
{
  bool result = true;
  UPolyItem * pi;
  UPolygon * poly;
  UDataBase * db;
  int i;
  UPosition pos;
  UVariable buffer;
  UVariable * returnVar = NULL;
  bool isOK;
  UPose po1, po2;
  // evaluate standard functions
  if (returnStruct[0]->isA("var"))
  {
    returnVar = (UVariable*)returnStruct[0];
    *returnStructCnt = 1;
  }
  else
    returnVar = &buffer;
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
  else if ((strcasecmp(name, "isInside") == 0) and (strcmp(paramOrder, "sc") == 0))
  {
    isOK = false;
    pos = params[1]->get3D();
    pi = getItem(params[0]->getValues());
    if (pi != NULL)
    { // item is found, test
      isOK = pi->isInsideConvex(pos.x, pos.y, 0.0);
    }
    returnVar->setBool(isOK);
  }
  else if ((strcasecmp(name, "isInside") == 0) and (strcmp(paramOrder, "scc") == 0))
  {
    isOK = false;
    po1 = params[1]->getPose(); // pose of robot
    po2 = params[2]->getPose(); // position relative to robot
    po1 = po1 + po2;
    pi = getItem(params[0]->getValues());
    if (pi != NULL)
    { // item is found, test
      isOK = pi->isInsideConvex(po1.x, po1.y, 0.0);
    }
    returnVar->setBool(isOK);
  }
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
  else if ((strcasecmp(name, "setPolygon") == 0) and
           (strcmp(paramOrder, "sc") == 0 or
            strcmp(paramOrder, "scd") == 0))
  {
    isOK = false;
    pi = getItem(params[0]->getValues());
    db = params[1];
    if (db->isAlsoA("polygon"))
      poly = (UPolygon*) db;
    else
      poly = NULL;
    isOK = poly != NULL;
    if (isOK and pi == NULL)
    { // item is found, so get position
      pi = add(params[0]->getValues());
      isOK = pi != NULL;
    }
    if (isOK)
    {
      isOK = poly->copyTo(pi);
      if (strlen(paramOrder) >= 3)
        // get also coordinate system reference
        pi->cooSys = params[2]->getInt();
      pi->setUpdated();
    }
    returnVar->setBool(isOK);
    gotNewData();
  }
  else
    // call name is unknown
    result = false;
  // set result if just a boolean
  if (result)
  { // set return struct value to isOK
    if (returnStruct != NULL and returnStructCnt != NULL)
      *returnStructCnt = 1;
  }
  return result;
}

///////////////////////////////////////////////////////////

UPolyItem * UResPoly::add(const char * name)
{
  UPolyItem * result;
  int i;
  //
  lock();
  result = getItem(name);
  if (result == NULL)
  { // does not exist, so create new poly
    for (i = 0; i < polysCnt; i++)
    {
      if (polys[i]->getPointsCnt() == 0)
      {
        result = polys[i];
        break;
      }
    }
    if (result == NULL and polysCnt < MAX_POLYS_CNT)
    {
      result = new UPolyItem();
      polys[polysCnt++] = result;
      varPolyCnt->setInt(polysCnt);
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
  else
    // exist already - not empty - return false
    result = NULL;
  unlock();
  //
  return result;
}

//////////////////////////////////////////////

UPolyItem * UResPoly::add(const char * name,
                          const double x, const double y, const double z)
{
  UPosition pos(x, y, z);
  UPolyItem * result;
  int i;
  //
  lock();
  result = getItem(name);
  if (result == NULL)
  { // does not exist, so create new poly
    for (i = 0; i < polysCnt; i++)
    {
      if (polys[i]->getPointsCnt() == 0)
      {
        result = polys[i];
        result->clear();
        break;
      }
    }
    if (result == NULL and polysCnt < MAX_POLYS_CNT)
    {
      result = new UPolyItem();
      polys[polysCnt++] = result;
      varPolyCnt->setInt(polysCnt);
    }
    if (result != NULL)
    { // initialize
      result->setAsPolyline();
      result->setName(name);
    }
  }
  if (result != NULL)
  {
    result->add(pos);
    result->setUpdated();
  }
  unlock();
  return result;
}

//////////////////////////////////////////////

UPolyItem * UResPoly::getItem(const char * name)
{
  int i;
  UPolyItem ** ki = polys;
  UPolyItem * result = NULL;
  //
  for (i = 0; i < polysCnt; i++)
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

UPolyItem * UResPoly::getNext(int p1, int * p2, const char * name)
{
  int i;
  UPolyItem * ki;
  UPolyItem * result = NULL;
  bool found;
  //
  for (i = p1; i < polysCnt; i++)
  {
    ki = polys[i];
    found = ki->match(name);
    if (found)
    {
      result = ki;
      break;
    }
  }
  i++;
  if (p2 != NULL)
  {
    if (i < polysCnt)
      *p2 = i;
    else
      *p2 = -1;
  }
  return result;
}

//////////////////////////////////////////////

bool UResPoly::del(const char * name)
{
  UPolyItem * pi = NULL;
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

const char * UResPoly::getList(const char * preStr, char * buff, const int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  int i;
  UPolyItem ** ki = polys;
  //
  snprintf(p1, buffCnt, "%s - %d items\n", preStr, polysCnt);
  n += strlen(p1);
  p1 = &buff[n];
  for (i = 0; i < polysCnt; i++)
  {
    (*ki)->print("  - ", p1, buffCnt - n);
    n += strlen(p1);
    p1 = &buff[n];
    ki++;
  }
  return buff;
}

//////////////////////////////////////////////////////////////

void UResPoly::handleNewData(USmlTag * tag)
{ // handle poly message
  const int MSL = 200;
  char att[MAX_SML_NAME_LENGTH];
  char val[MSL];
  USmlTag nTag;
  int cntItems = 0;
  const int MNL = 32;
  char name[MNL];
  UPolyItem * pi;
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
      if (nTag.isTagA("polygon"))
      { // get all support data
        pi = NULL;
        if (nTag.getAttValue("name", name, MNL))
        { // must have a name
          // get polygon item to unpack to
          pi = getItem(name);
          if (pi == NULL)
            pi = add(name);
        }
        if (pi != NULL)
        { // optional coordinate system
          nTag.getAttBool("valid", &pi->valid, true);
          if (pi->valid)
          {
            nTag.getAttInteger("cooSys", &pi->cooSys, 1);
            nTag.getAttDouble("x", &pi->relPose.x, 0.0);
            nTag.getAttDouble("y", &pi->relPose.y, 0.0);
            nTag.getAttDouble("h", &pi->relPose.h, 0.0);
            nTag.getAttBool("relPoseUse", &pi->relPoseUse, false);
            // get rest of polygon
            nTag.getPolygon(pi, NULL);
          }
          pi->updateTime.now();
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
/*    if (polysCnt != cnt)
      printf("Has %d polyItems (%d updated), but sender has %d!\n", polysCnt, cntItems, cnt);*/
    varPolyCnt->setInt(polysCnt);
    gotNewData();
  }
}

/////////////////////////////////////////////////////

const char * UResPoly::codePolysXml(const char * tagName,
                                    char * buf, const int bufCnt, UTime updatedSince)
{
  char * p1 = buf;
  int n = 0, i;
  UPolyItem * pi;
  //
  snprintf(p1, bufCnt - n, "<%s name=\"polyItems\" cnt=\"%d\">\n", tagName, polysCnt);
  n += strlen(p1);
  p1 = &buf[n];
  for (i = 0; i < polysCnt; i++)
  {
    pi = polys[i];
    if (pi != NULL)
    {
      if (pi->updateTime >= updatedSince)
      {
        pi->codeXML(p1, bufCnt - n);
        n += strlen(p1);
        p1 = &buf[n];
      }
    }
    if (bufCnt - n < 80)
      break;
  }
  snprintf(p1, bufCnt - n, "</%s>\n", tagName);
  return buf;
}

//////////////////////////////////////////////////

bool UResPoly::codePolyXml(const char * tagName,
                                    char * buf, const int bufCnt, UTime updatedSince, int idx)
{
  char * p1 = buf;
  int n = 0;
  UPolyItem * pi;
  bool isOK = false;
  //
  pi = polys[idx];
  if (pi != NULL)
  {
    if (pi->updateTime >= updatedSince)
    {
      snprintf(p1, bufCnt - n, "<%s name=\"polyItems\" cnt=\"1\">\n", tagName);
      n += strlen(p1);
      p1 = &buf[n];
      pi->codeXML(p1, bufCnt - n);
      n += strlen(p1);
      p1 = &buf[n];
      snprintf(p1, bufCnt - n, "</%s>\n", tagName);
      isOK = true;
    }
  }
  return isOK;
}

/////////////////////////////////////////////

void UResPoly::gotNewData()
{ //
  char * dataType = (char*)"poly";
  double v = 0.0;
  //
  if (varCallDispOnNewData->getBool())
  { // tell display system of new image
    callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
  }
  varUpdTime->setTimeNow();
}

