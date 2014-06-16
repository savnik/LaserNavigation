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

#include <urob4/usmltag.h>
#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>

#include "ureslaserifsf.h"

// UResLaserIfSf::UResLaserIfSf()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   createVarSpace(20, 0, 2, "Laser scanner server obstacle detection results");
//   createBaseVar();
//   sfPool = new UFeaturePool();
//   callDispOnNewData = true;
// }


UResLaserIfSf::~UResLaserIfSf()
{
  if (sfPool != NULL)
    delete sfPool;
}

// const char * UResLaserIfSf::name()
// {
//   return "scanfeature_data (jca@elektro.dtu.dk)";
// }

//////////////////////////////////////////

const char * UResLaserIfSf::commandList()
{
  return "sf pass line mapbase road lobst ";
}

////////////////////////////////////////////////////

void UResLaserIfSf::createBaseVar()
{
  UVarPool * vp;
  //
  vp = getVarPool();
  if (vp != NULL)
  {
    vp->addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varSerial = vp->addVar("serial", -1.0, "d", "latest scan serial number");
    varTime = vp->addVar("time", -1.0, "d", "Time for latest update");
    varSfGrps = vp->addVar("groups", -1.0, "d", "Number of scanfeature groups");
  }
}

////////////////////////////////////////////////////

void UResLaserIfSf::snprint(const char * preString, char * buff, int buffCnt)
{
  const int MSL = 30;
  char s[MSL];
  double v;
  UTime t;
  char * p1 = buff;
  int n = 0;
  //
  snprintf(p1, buffCnt - n, "%s Handles interface tags: %s\n",
           preString, commandList());
  n = strlen(p1);
  p1 = &buff[n];
  //
  v = varTime->getValued();
  t.setTime(v);
  t.getTimeAsString(s, true);
  snprintf(p1, buffCnt - n, "%s Holds %d feature groups, last update at %s\n",
           preString, sfPool->getScansCnt(), s);
}

//////////////////////////////////////////////////////////////////

bool UResLaserIfSf::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // delete any local
    if (remove)
      poseHist = NULL;
    else if (poseHist != resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
  }
  else
    result = false;
  //
  result |= UResVarPool::setResource(resource, remove);
  //
  return result;
}

//////////////////////////////////////////

void UResLaserIfSf::handleNewData(USmlTag * tag)
{ // print if desired
  bool isInfo;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  // het first attribute
  tag->getNextAttribute(att, val, MVL);
  tag->reset();
  isInfo = (strcmp(att, "warning") == 0) or
      (strcmp(att, "info") == 0) or
      (strcmp(att, "debug") == 0) or
      (strcmp(att, "error") == 0);
  if (verboseMessages)
    tag->print("");
  else if (isInfo)
    tag->print("");
  // distribute to sub-functions
  if (not isInfo)
  {
    if (tag->isTagA("sf") or
        tag->isTagA("line") or
        tag->isTagA("mapbase") or
        tag->isTagA("lobst"))
      handleSF(tag);
    else if (tag->isTagA("pass"))
      handlePass(tag);
    else if (tag->isTagA("road"))
      handleRoad(tag);
    else
      printReply(tag, "UClientFuncLaser::handleNewData: not mine");
  }
}

//////////////////////////////////////

bool UResLaserIfSf::handleSF(USmlTag * sfTag)
{
  UFeatureData dataBuffer;
  UFeatureData * dp = &dataBuffer;
  bool result;
  const int MVL = 30;
  char val[MVL];
  char att[MAX_SML_NAME_LENGTH];
  //int n;
  double x, y, th, length, width = 0.0;
  ULineSegment seg;
  USmlTag nTag;
  UPosRot sensorPose;
  bool gotSensorPos = false;
  bool gotSensorRot = false;
  bool gotRobotPose = false;
  UPoseTime poset;
  const int MSSL = UFeatureData::MAX_SEG_STR_LENGTH;
  char segStr[MSSL];
  int age, ns;
  //
  result = sfTag->isAStartTag();
  if (result)
  { // read size main attributes
    if (sfTag->getAttValue("timestamp", val, MVL) or
        sfTag->getAttValue("tod", val, MVL))
      dp->scanTime.setTimeTod(val);
    else
      dp->scanTime.now();
    // get data type
    if (not sfTag->getAttValue("name", dp->type, dp->MAX_TYPE_LENGTH))
      // no name attribute, so use tag name
      strncpy(dp->type, sfTag->getTagName(), dp->MAX_TYPE_LENGTH);
    // now get the guts of the message
    // that is the line segments
    // n = 0;
    while (sfTag->getNextTag(&nTag, 200))
    { // <line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\" />\n",
      if (nTag.isTagA("line"))
      { // is a line segment - read attributes
        x = 0.0;
        y=0.0;
        th = 0.0;
        length = 0.0;
        //msq = 0.0;
        segStr[0] = '\0';
        age = -2;
        while (nTag.getNextAttribute(att, val, MVL))
        { // read 2d position with obstacle flag
          if (strcasecmp(att, "x") == 0)
            x = strtod(val, NULL);
          else if (strcasecmp(att, "y") == 0)
            y = strtod(val, NULL);
          else if ((strcasecmp(att, "th") == 0) or (strcasecmp(att, "h") == 0))
            th = strtod(val, NULL);
          else if (strcasecmp(att, "l") == 0)
            length = strtod(val, NULL);
//           else if (strcasecmp(att, "msq") == 0)
//             // line fit variance [m^2]
//             msq = strtod(val, NULL);
          else if (strcasecmp(att, "name") == 0)
            strncpy(segStr, val, MSSL);
          else if (strcasecmp(att, "age") == 0)
            age = strtol(val, NULL, 10);
          else if (strcasecmp(att, "width") == 0)
            width = strtol(val, NULL, 10);
        }
        if (length > 0.01)
        { // valid line
          seg.set2D(x, y, th);
          seg.length = length;
          if (age != -2)
          { // add age to id string
            ns = strlen(segStr);
            snprintf(&segStr[ns], MSSL - ns, " %d", age);
          }
          result = dp->addSegment(&seg, 0, width,  segStr);
        }
      }
      else if (nTag.isTagA("pose"))
      { // is robot pose for the data
        gotRobotPose = nTag.getPose(&dp->pose);
        if (nTag.getAttValue("name", val, MVL))
        {
          if (strcasecmp(val, "mapCoordinateBase") == 0)
            dp->coordinateRef = 2;
        }
      }
      else if (nTag.isTagA("pos3d"))
      { // device position on robot
        gotSensorPos = nTag.getPosition(sensorPose.getPos());
      }
      else if (nTag.isTagA("rot3d"))
      { // device roattion on robot
        gotSensorRot = nTag.getRotation(sensorPose.getRot());
      }
      else if (nTag.isAnEndTag())
          break;
      else if (nTag.isAStartTag())
          // extra grouped info
          nTag.skipToEndTag(200);
      else
        ; // just ignore
    }
    if (sfPool != NULL)
    { // move the scandata to map coordinates
      // ensure we know where the sensor is
      if (gotSensorPos and gotSensorRot)
        sfPool->setSensorPose(sensorPose);
      else if (sfPool->gotSensorPose())
        sensorPose = sfPool->getSensorPose();
      else
        // assume already in robot coordinates
        sensorPose.clear();
      // add to pose history if data
      if (not gotRobotPose and poseHist != NULL)
      { // get robot pose
        dp->pose = poseHist->getPoseAtTime(dp->scanTime);
        gotRobotPose = true;
      }
/*      if (gotRobotPose and poseHist != NULL)
      { // update pose hist if needed
        poset.setPt(dp->pose, dp->scanTime);
        poseHist->addIfNeeded(poset, 0.0, -7);
      }*/
      // change to odometry coordinates
      if (gotRobotPose)
        dp->moveLocalToOdo(dp->pose, sensorPose);
      // add new data to segment pool
      sfPool->addData(dp);
      // make a data update event
      sfPool->event("lasif", "sf", NULL);
      varSfGrps->setInt(sfPool->getScansCnt());
      varTime->setTime(dp->scanTime, 0);
      // tell others
      newDataAvailable();
    }
  }
  return result;
}

//////////////////////////////////////

bool UResLaserIfSf::handleRoad(USmlTag * sfTag)
{
  UFeatureData dataBuffer;
  UFeatureData * dp = &dataBuffer;
  bool result;
  const int MVL = 30;
  char val[MVL];
//  char att[MAX_SML_NAME_LENGTH];
  int segInt;
  //double msq;
  ULineSegment seg;
  USmlTag nTag;
  const int MSSL = UFeatureData::MAX_SEG_STR_LENGTH;
  char segStr[MSSL];
  //
  result = sfTag->isAStartTag();
  if (result)
  { // read size main attributes
    if (sfTag->getAttValue("timestamp", val, MVL))
      dp->scanTime.setTimeTod(val);
    strncpy(dp->type, sfTag->getTagName(), dp->MAX_TYPE_LENGTH);
    // now get the guts of the message
    // that is the line segments
    //n = 0;
    while (sfTag->getNextTag(&nTag, 200))
    { // <line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\" />\n",
      segInt = 0; // to avoid compiler warning
      if (nTag.isTagA("lineSeg"))
      { // is a line segment - read attributes
        if (nTag.getAttValue("name", val, MVL))
        {
          if (strcmp(val, "left") == 0)
            segInt = 0;
          else if (strcmp(val, "right") == 0)
            segInt = 2;
          else // center position
            segInt = 1;
          strncpy(segStr, val, MSSL - 2);
        }
//         if (nTag.getAttValue("q", val, MVL))
//           msq = strtod(val, NULL);
        if (nTag.getAttValue("best", val, MVL))
        { // is this the best road estimate
          if (str2bool(val))
            segInt += 10;
          strcat(segStr, " *");
        }
        if (nTag.getLineSegment(&seg))
        { // valid line
          result = dp->addSegment(&seg, segInt, 0.0, segStr);
        }
      }
      else if (nTag.isAnEndTag())
        break;
      else if (nTag.isAStartTag())
          // extra grouped info
        nTag.skipToEndTag(200);
      else
        ; // just ignore
    }
    // make new pool if none exist
    if (sfPool != NULL)
    { // add new data to segment pool
      sfPool->addData(dp);
      // make a data update event
      sfPool->event("lasif", "road", NULL);
      varSfGrps->setInt(sfPool->getScansCnt());
      varTime->setTime(dp->scanTime, 0);
      // tell others
      newDataAvailable();
    }
  }
  return result;
}

//////////////////////////////////////

bool UResLaserIfSf::handlePass(USmlTag * sfTag)
{
  /* coded sample
  <pass cnt="2" tod="1174394449.843994">
  <pose name="robot" x="2.291" y="0.189" h="0.098"/>
  <pos3d name="device" x="0.4" y="0" z="0.4"/>
  <rot3d name="device" Omega="0" Phi="0.15" Kappa="0"/>
  <lineSeg name="pis" length="2.67834477551156e+00"
      center="1.339" centerValid="false" var="1.0668730e-02" varMin="1.0000000e+00">
  <pos3d name="start" x="1.5425000190735" y="-2.5471987724304" z="0.1986313180858"/>
  <pos3d name="vector" x="0.46499693999433" y="0.88207919965073" z="-0.075591873501314"/>
  </lineSeg>
  <lineSeg name="pis" length="2.15102156353428e+00"
      center="1.076" centerValid="false" var="4.0055499e-03" varMin="1.0000000e+00">
  <pos3d name="start" x="2.7314646244049" y="-0.0044137896038592" z="-0.004018962262829"/>
  <pos3d name="vector" x="-0.3346800227981" y="0.94078903768517" z="0.053900546484067"/>
  </lineSeg>
  </pass>
  */
  UFeatureData dataBuffer;
  UFeatureData * dp = &dataBuffer;
  bool result;
  const int MVL = 30;
  char val[MVL];
  char att[MAX_SML_NAME_LENGTH];
  int n;
  ULineSegment seg;
  USmlTag nTag;
  UPosRot sensorPose;
  bool gotSensorPos = false;
  bool gotSensorRot = false;
  bool gotRobotPose = false;
  int cnt = 0;
  double center;
//  bool centerValid;
  //double var;
  //double varMin;
  unsigned long serial = 0;
  //
  result = sfTag->isAStartTag();
  if (result)
  { // read size main attributes
    if (sfTag->getAttValue("tod", val, MVL))
      dp->scanTime.setTimeTod(val);
    if (sfTag->getAttValue("scan", val, MVL))
      serial = strtol(val, NULL, 0);
    strncpy(dp->type, sfTag->getTagName(), dp->MAX_TYPE_LENGTH);
    if (sfTag->getAttValue("cnt", val, MVL))
      cnt = strtol(val, NULL, 10);
    // now get the guts of the message
    // that is the line segments
    n = 0;
    while (sfTag->getNextTag(&nTag, 200))
    { // <line x=\"%g\" y=\"%g\" th=\"%g\" l=\"%g\" msq=\"%g\" />\n",
      if (nTag.isTagA("lineSeg"))
      { // is a line segment - read secondary attributes
        center = 0.0;
//        centerValid = false;
        //var = 0.0;
        //varMin = 0.0;
        while (nTag.getNextAttribute(att, val, MVL))
        { // read 2d position with obstacle flag
          if (strcasecmp(att, "var") == 0)
            ; //var = strtod(val, NULL);
          else if (strcasecmp(att, "varMin") == 0)
            ; //varMin = strtod(val, NULL);
          else if (strcasecmp(att, "center") == 0)
            center = strtod(val, NULL);
//           else if (strcasecmp(att, "centerValid") == 0)
//             centerValid = str2bool(val);
        }
        // get the segment itself
        if (nTag.getLineSegment(&seg))
        {
          result = dp->addSegment(&seg, 0, center, "");
        }
        n++;
      }
      else if (nTag.isTagA("pose"))
      { // is robot pose for the data
        gotRobotPose = nTag.getPose(&dp->pose);
      }
      else if (nTag.isTagA("pos3d"))
      { // device position on robot
        gotSensorPos = nTag.getPosition(sensorPose.getPos());
      }
      else if (nTag.isTagA("rot3d"))
      { // device roattion on robot
        gotSensorRot = nTag.getRotation(sensorPose.getRot());
      }
      else if (nTag.isAnEndTag())
        break;
      else if (nTag.isAStartTag())
          // extra grouped info
        nTag.skipToEndTag(200);
      else
        ; // just ignore the rest
    }
    // test for line count
    if ((n != cnt) and (verboseMessages))
      printf("Expected %d but got %d lines\n", cnt, n);
    // make new pool if none exist
    if (sfPool != NULL)
    {
      // move the scandata to map coordinates
      if (gotSensorPos and gotSensorRot)
        // set also default sensor pose
        sfPool->setSensorPose(sensorPose);
      else if (sfPool->gotSensorPose())
        sensorPose = sfPool->getSensorPose();
      // change to robot (odometry) coordinates
      // if info is not OK then keep sensor coordinates
      if (gotRobotPose and sfPool->gotSensorPose())
        dp->moveLocalToOdo( dp->pose, sensorPose);
      // add new data to segment pool
      sfPool->addData(dp);
      // make a data update event
      sfPool->event("lasif", "pass", NULL);
      // set local variables
      varSerial->setInt(serial, 0);
      varTime->setTime(dp->scanTime, 0);
      varSerial->setInt(sfPool->getScansCnt(), 0);
      //
      newDataAvailable();
    }
  }
  return true;
}

///////////////////////////////////////////

void UResLaserIfSf::newDataAvailable()
{
  char * dataType = (char *)"laser";
  double v;
  bool isOK;
  //
  //setLocalVar(varScan, scan->getSerial());
  if (callDispOnNewData)
  {
    // tell display system of new image
    v = varSerial->getValued();
    isOK = callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    isOK = callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
    //printf("UResLaserIfObst::newDataAvailable: now %d obstacle group lines\n",
    //     groupsCnt);
    callDispOnNewData = isOK;
  }
}

