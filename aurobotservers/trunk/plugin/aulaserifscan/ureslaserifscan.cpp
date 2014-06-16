/***************************************************************************
 *   Copyright (C) 2007 by Christian Andersen   *
 *   jca@elektro.dtu.dk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "ureslaserifscan.h"

#include <urob4/uvarcalc.h>
#include <urob4/uresposehist.h>

// UResLaserIfScan::UResLaserIfScan()
// {
//   setResID( getResID());
//   resVersion = getResVersion();
//   //
//   poseHist = NULL;
//   //
//   createVarSpace(20, 0, 2, "Laser scanner basic data interface");
//   createBaseVar();
//   callDispOnNewData = true;
//   strncpy(dataSourceName, "(none)", MAX_DATA_SOURCE_NAME);
// }

/////////////////////////////////////////

UResLaserIfScan::~UResLaserIfScan()
{
}

//////////////////////////////////////////

// const char * UResLaserIfScan::name()
// {
//   return "LaserIfScan (Christian)";
// }

//////////////////////////////////////////

const char * UResLaserIfScan::commandList()
{
  return "scanGet";
}


///////////////////////////////////////////

void UResLaserIfScan::createBaseVar()
{
    addVar("version", getResVersion() / 100.0, "d", "Resource version");
    varSensorX =  addVar("sersorX", 0.0, "3d", "Sensor 3D position (holds also rotation - 6D)");
    //addVar("sersorY", 0.0, "3d", "");
    //addVar("sersorZ", 0.0, "3d", "");
    varSensorOmega = addVar("sersorO", 0.0, "rot", "Sensor 3D orientation (Omega (x), Phi (y), Kappa (z))");
    //addVar("sersorP", 0.0, "rot", "");
    //addVar("sersorK", 0.0, "rot", "");
    varScan = addVar("scan", 0.0, "d", "Scan serial number for latest scan");
    //vp->addMethod(this, "rightLine", "", roadGetLine);
}

//////////////////////////////////////////////////////////////////

bool UResLaserIfScan::setResource(UResBase * resource, bool remove)
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

/////////////////////////////////////////////////////

bool UResLaserIfScan::gotAllResources(char * missingThese, int missingTheseCnt)
{ // just needs a pointer to core for event handling
  bool result = true;
  int n = 0;
  char * p1 = missingThese;
  //
  if (poseHist == NULL)
  {
    strncpy(p1, UResPoseHist::getOdoPoseID(), missingTheseCnt);
    n += strlen(p1);
    p1 = &missingThese[n];
    result = false;
  }
  result &= UResVarPool::gotAllResources(p1, missingTheseCnt - n);
  return result;
}

/////////////////////////////////////////

const char * UResLaserIfScan::snprint(const char * preString, char * buff, int buffCnt)
{
  char * p1 = buff;
  int n = 0;
  ULaserDataSet * ls = NULL;
  const int MSL = 50;
  char s[MSL];
  UTime t;
  UPose  pose;
  //
  snprintf(p1, buffCnt - n, "%s Handles XML tags: %s\n",
           preString, commandList());
  n += strlen(p1);
  p1 = &buff[n];
  snprintf(p1, buffCnt - n, "%s holds %d scan(s), latest scan number %d (from %s)\n",
      preString, scanHist.getScansCnt(), varScan->getInt(), dataSourceName);
  ls = scanHist.getNewest();
  if (ls != NULL)
  {
    t = ls->getScanTime();
    t.getTimeAsString(s, true);
    pose = ls->getPose();
    n += strlen(p1);
    p1 = &buff[n];
    snprintf(p1, buffCnt - n, "%s from %.2fx %.2fy %.4frad %d measurements at %s\n",
             preString, pose.x, pose.y, pose.h, ls->getCount(), s);
  }
  return buff;
}

/////////////////////////////////////////

void UResLaserIfScan::handleNewData(USmlTag * tag)
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
  if (tag->isTagA("scanGet"))
    handleLaserScan(tag);
  else
    printReply(tag, "UClientFuncLaser::handleNewData: not mine");
}

//////////////////////////////////////////////

bool UResLaserIfScan::handleLaserScan(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  long sec = 0, usec = 0;
  USmlTag binTag;
  int dataFirst = 0;
  int dataCount = 0;
  int dataInterval = 1;
  Units dataUnit = DM;
  Format dataCodec = HEX;
  double dataMin;
  double dataMax;
  UTime t;
  double unit = 0.01, ang, intAng = 0.0;
  int n = 0, m, h;
  UClientLaserData * pd;
  int binSize, valCnt, got;
  unsigned char lsb, msb;
  unsigned char * binDataSource;
  double statW = 0.0;
  double laserTilt = 0.0;
  bool statValid = false;
  UPose pose;
  UPoseTime pt;
  UClientLaserPi pi;
  unsigned long serial = 0;
  ULaserDataSet * scan = NULL;
  bool gotX = false;
  bool gotY = false;
  bool gotTh = false;
  UPosition pos;
  URotation rot;
  UPosRot pr;
  double maxValidRange = 8.1;
  // debug
  //tag->print("handleImages got:");
  // debug end
  // <scanGet first=0 interval=4 count=90 tod=1108828477.170399 unit=mm min=-90.00 max=88.00 codec=TAG>
  // <lval a=0 b=0 z=0 ang=-90.00 dist=1.647/>
  // <lval a=0 b=0 z=0 ang=-88.00 dist=1.600/>
  // ...
  // </scanGet>
  // <scanget first=0 interval=1 count=361 tod=1108833433.670426 unit=mm min=-90.00 max=90.00 codec=HEX>
  // <bin size=722 codec=HEX>6f0660064e06460642063b064106 ...
  // 40594058b058b0589058205</bin>
  // </scanget>
  // from mmrsr2:
  // <scanGet first=0 interval=1 count=181 tod=1112802415.035491 unit=mm codec=TAG
  //                  statValid=true statWidth=0.350 x=0.000y=0.000 h=0.00000 pisCnt=2>

  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "serial") == 0)
      sscanf(val, "%lu", &serial);
    else if (strcasecmp(att, "first") == 0)
      sscanf(val, "%d", &dataFirst);
    else if (strcasecmp(att, "interval") == 0)
      sscanf(val, "%d", &dataInterval);
    else if (strcasecmp(att, "count") == 0)
      sscanf(val, "%d", &dataCount);
    else if (strcasecmp(att, "tod") == 0)
      sscanf(val, "%ld.%ld", &sec, &usec);
    else if (strcasecmp(att, "unit") == 0)
    {
      if (strcasecmp(val, "mm") == 0)
        dataUnit = MM;
      else if (strcasecmp(val, "cm") == 0)
        dataUnit = CM;
      else if (strcasecmp(val, "10cm") == 0)
        dataUnit = DM;
    }
    else if (strcasecmp(att, "min") == 0)
      sscanf(val, "%lf", &dataMin);
    else if (strcasecmp(att, "max") == 0)
      sscanf(val, "%lf", &dataMax);
    else if ((strcasecmp(att, "codex") == 0) or (strcasecmp(att, "codec") == 0))
    {
      if (strcasecmp(val, "TAG") == 0)
        dataCodec = TAG;
      else if (strcasecmp(val, "HEX") == 0)
        dataCodec = HEX;
      else if (strcasecmp(val, "BIN") == 0)
        dataCodec = BIN;
    }
    else if (strcasecmp(att, "statWidth") == 0)
      sscanf(val, "%lf", &statW);
    else if (strcasecmp(att, "statValid") == 0)
      statValid = str2bool(val);
    else if (strcasecmp(att, "x") == 0)
    {
      sscanf(val, "%lf", &pose.x);
      gotX = true;
    }
    else if (strcasecmp(att, "y") == 0)
    {
      sscanf(val, "%lf", &pose.y);
      gotY = true;
    }
    else if (strcasecmp(att, "h") == 0)
    {
      sscanf(val, "%lf", &pose.h);
      gotTh = true;
    }
    else if (strcasecmp(att, "laserTilt") == 0)
      laserTilt = strtod(val, NULL);
    else if (strcasecmp(att, "maxValidRange") == 0)
      maxValidRange = strtod(val, NULL);
    else if ((strcasecmp(att, "warning") == 0) or
              (strcasecmp(att, "debug") == 0) or
              (strcasecmp(att, "info") == 0))
      result = false;
  }
  //
  if (not result)
  { // no data, so just show message (if not shown already)
/*    if (not verboseMessages)
    tag->print("Reply:");*/
  }
  if (result)
  { // put data in buffer
    scan = scanHist.getNewScan();
    if (scan == NULL)
      printf("UClientFuncLaser::handleLaserScan: Data allocation error\n");
    else
    {
      t.setTime(sec, usec);
      switch (dataUnit)
      {
        case MM: unit = 0.001; break;
        case CM: unit = 0.01;  break;
        case DM: unit = 0.1;   break;
        default: unit = 0.001; break;
      }
      scan->clear();
      scan->setData(serial, dataFirst, dataInterval, dataCount, 
                    t, unit, statValid, statW, laserTilt, maxValidRange);
      if (gotX and gotY and gotTh)
      {
        scan->setPose(pose);
/*        if (poseHist != NULL)
        {
          pt.setPt(pose, t);
          poseHist->addIfNeeded(pt, 0.0, -6);
        }*/
      }
      else
      { // use the cleared pose
        pose.clear();
        scan->setPose(pose);
      }
    }
    // calculate angle interval
    if (dataCount > 1)
      intAng = (dataMax - dataMin)/((double)dataCount - 1.0);
    else
      intAng = 1.0;
  }
  if ((scan != NULL) and tag->isAStartTag())
  { // there is more - get the rest
    // get next tag - either a measurement or a binary start tag
    n = 0;
    pd = scan->getData();
    while(tag->getNextTag(&binTag, 300))
    { // message structure:
      // <scanget ... >
      // <pose name="robot" x="0.599" y="0.027" h="0.076"/>
      // <pos3d name="device" x="3.3" y="0.01" z="0.81"/>
      // <rot3d name="device" Omega="1.2" Phi="0" Kappa="0.77"/>
      // <bin ...> HEX or binary scan data </bin> or
      // [<lval ... />]*  // laser range values
      // [<Pi ... />]*    // passable intervals
      // </scanget>
      // may be a laser value
      if (binTag.isTagA("lval"))
      { // laser value tag
        pd->setFromTag(&binTag);
        if (n < dataCount)
          // advance to next set
          pd++;
        n++;
      }
      else if (binTag.isTagA("Pi"))
      { // passable interval
        if (pi.setFromTag(&binTag))
          scan->addPassableInterval(&pi);
      }
      else if (binTag.isTagA("lineSeg"))
      { // passable interval
        if (pi.setFromTag(&binTag))
          scan->addPassableInterval(&pi);
      }
      else if (binTag.isTagA("pose"))
      { // robot pose at scantime
        if (binTag.getPose(&pose))
          scan->setPose(pose);
/*        if (poseHist != NULL)
        {
          pt.setPt(pose, t);
          poseHist->addIfNeeded(pt, 0.0, -6);
        }*/
      }
      else if (binTag.isTagA("pos3d"))
      { // device position on robot
        if (binTag.getPosition(&pos))
          scan->getSensorPose()->setPos(pos);
      }
      else if (binTag.isTagA("rot3d"))
      { // device roattion on robot
        if (binTag.getRotation(&rot))
          scan->getSensorPose()->setRot(rot);
      }
      else if (binTag.isTagA("bin"))
      { // data format is BIN or HEX
        binSize = 0;
        while (binTag.getNextAttribute(att, val, MVL))
        {
          if (strcasecmp(att, "size") == 0)
            sscanf(val, "%d", &binSize); // size in bytes of 'binary' data
          else if ((strcasecmp(att, "codex") == 0) or (strcasecmp(att, "codec") == 0))
            ; // known already
        }
        if (binSize > 0)
        { // get data
          m = binSize;
          valCnt = 0;
          n = 0;
          while (m > 0)
          {
            got = tag->getNBytes(&val[valCnt], mini(MVL, m), 200);
            if (got == 0)
              break;
            m -= got;
            valCnt += got;
            binDataSource = (unsigned char *)val;
            if (dataCodec == HEX)
            {
              for (h = 0; h < valCnt/4; h++)
              {
                lsb = hex2int(binDataSource[0], binDataSource[1]);
                binDataSource += 2;
                msb = hex2int(binDataSource[0], binDataSource[1]);
                binDataSource += 2;
                ang = double(n) * intAng +  dataMin;
                pd->setValue(ang, lsb, msb, unit);
                // debug
                //pd->print("debug");
                // debug end
                if (n < dataCount)
                  pd++;
                n++;
              }
              // any remaining odd data is saved
              if (((valCnt % 4) != 0) and (valCnt > 3))
                // move remaining part to start of buffer
                memcpy(val, &val[valCnt - (valCnt % 4)], valCnt % 4);
              valCnt = valCnt % 4;
            }
            else if (dataCodec == BIN)
            {
              for (h = 0; h < valCnt/2; h++)
              {
                lsb = binDataSource[0];
                msb = binDataSource[1];
                binDataSource += 2;
                ang = double(n) * intAng +  dataMin;
                pd->setValue(ang, lsb, msb, unit);
                if (n < dataCount)
                  pd++;
                n++;
              }
              // any remaining odd data is saved
              if (((valCnt % 2) != 0) and (valCnt > 1))
                // move remaining part to start of buffer
                memcpy(val, &val[valCnt - (valCnt % 2)], valCnt % 2);
              valCnt = valCnt % 2;
            }
          }
        }
      }
      else if (binTag.isAStartTag())
        // extra grouped info
        binTag.skipToEndTag(200);
      else if (binTag.isTagAnEnd("bin"))
        ; // end of binary message -- that is OK
      else if (binTag.isTagAnEnd(tag->getTagName()))
        // end of message
        break;
      else
        // something else - just ignore
        printf("UClientFuncLaser::handleLaserScan: no recognized data???\n");
    }
  }
  // announce the new data
  if (result and scan != NULL)
  {
    pr.set(pos, rot);
    gotNewData(scan);
    varSensorX->set6D(&pr);
    //setLocalVar(varSensorX+1, pos.y, 0);
    //setLocalVar(varSensorX+2, pos.z, 0);
    varSensorOmega->setRot(&rot);
    //setLocalVar(varSensorOmega+1, rot.Phi, 0);
    //setLocalVar(varSensorOmega+2, rot.Kappa, 0);
    varScan->setInt(serial, 0);
    if (tag->getIfName() != NULL)
      strncpy(dataSourceName, tag->getIfName(), MAX_DATA_SOURCE_NAME);
    else
      strncpy(dataSourceName, "unknown", MAX_DATA_SOURCE_NAME);
  }
  //printf("UClientFuncLaser::handleLaserScan: Finished %s with %d measurements\n", bool2str(result), n);
  return result;
}

//////////////////////////////////////////////////////

void UResLaserIfScan::gotNewData(ULaserDataSet * scan)
{ // 
  char * dataType = (char*)"laser";
  double v;
  bool isOK;
  //
  if (callDispOnNewData)
  { //setLocalVar(varScan, scan->getSerial());
    // tell display system of new image
    v = scan->getSerial();
    isOK  = callGlobal("view.newData", "sd", &dataType, &v, &v, NULL, NULL);
    isOK |= callGlobal("disp.newData", "sd", &dataType, &v, &v, NULL, NULL);
    // is no display to call, then stop doing so
    callDispOnNewData = isOK;
  }
}

