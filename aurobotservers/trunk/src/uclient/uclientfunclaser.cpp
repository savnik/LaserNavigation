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
#include "uclientfunclaser.h"

ULaserWpc::ULaserWpc()
{
  speedCurrent = 0.0;
  speedPlanned = 0.0;
  wpPosCnt = 0;
}

///////////////////////////////////////////////////

////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////

UClientFuncLaser::UClientFuncLaser()
 : UClientFuncBase()
{
  int i;
 // ciCnt = 0;
 // ciCorrCnt = 0;
  poseHistCnt = 0;
  ekfuSkipd = 0;
  ekfuUsed = 0;
  pathsCnt = 0;
  freePolyCnt = 0;
  freePolyNewest = -1;
  for (i = 0; i < MAX_NEW_PATH_CANDIDATES; i++)
    paths[i] = NULL;
}

//////////////////////////////////////////

UClientFuncLaser::~UClientFuncLaser()
{
  int i;

  for (i = 0; i < MAX_NEW_PATH_CANDIDATES; i++)
  { // delete all allocated paths
    if (paths[i] != NULL)
      delete paths[i];
    else
      break;
  }
}

//////////////////////////////////////////

const char * UClientFuncLaser::name()
{
  return "SICK_Laser_data_client-1.0 (jca@oersted.dtu.dk)";
}

//////////////////////////////////////////

const char * UClientFuncLaser::commandList()
{
  return "scanGet wpfGet wpcGet odoGet ekfGet planGet pathGet set get push var poseHist avoid roaddrive drivepos";
}

//////////////////////////////////////////

void UClientFuncLaser::changedNamespace(const char * newNamespace)
{ // set namespace
  strncpy(serverNamespace, newNamespace, MAX_SML_NAME_LENGTH);
  // set name space value
  if (strcmp(newNamespace, "mmrd") == 0)
    serverNamespaceValue = NAMESPACE_MMRD;
  else if (strcmp(newNamespace, "mmrsr2") == 0)
    serverNamespaceValue = NAMESPACE_MMRSR2;
  else if (strcmp(newNamespace, "laserServer") == 0)
    serverNamespaceValue = NAMESPACE_LASER;
//  else
//    not a valid namespace for me.
//    printf("Unknown namespace (%d) '%s'\n", serverNamespaceValue, laserNamespace);
}

//////////////////////////////////////////

void UClientFuncLaser::handleNewData(USmlTag * tag)
{ // print if desired
  bool isInfo;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  UPlannerValue pv;
  bool result;
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
  if ((tag->isTagA("get")) or (tag->isTagA("var")))
  { // save variable
    result = false;
    if (tag->isAStartTag())
      // data is more comples that a single variable
      result = handleGetComplexData(tag);
    else if (not isInfo)
      result = planner.setVarFromTag(tag, &odoPose);
    if (result)
      gotNewData(NULL);
  }
  else if (tag->isTagA("poseHist"))
    handleVarData(tag);
  else if (tag->isTagA("scanGet"))
    handleLaserScan(tag);
  else if (tag->isTagA("wpfGet"))
    handleWpf(tag);
  else if (tag->isTagA("wpcGet"))
    handleWpc(tag);
  else if (tag->isTagA("odoGet"))
    handleOdo(tag);
  else if (tag->isTagA("ekfGet"))
    handleEkf(tag);
  else if (tag->isTagA("planGet"))
    handlePlan(tag);
  else if (tag->isTagA("pathGet"))
    handlePath(tag);
  else if (tag->isTagA("avoid"))
    handlePath(tag);
  else if (tag->isTagA("roaddrive"))
    handlePath(tag);
  else if (tag->isTagA("drivepos"))
    handlePath(tag);
  else if (tag->isTagA("set"))
    ; // just show reply
  else if (tag->isTagA("push"))
  { // just a print message
    if (not verboseMessages)
      tag->print("");
  }
  else
    printReply(tag, "UClientFuncLaser::handleNewData: not mine");
}

//////////////////////////////////////////

bool UClientFuncLaser::handleLaserScan(USmlTag * tag)
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
  ULaserData * pd;
  int binSize, valCnt, got;
  unsigned char lsb, msb;
  unsigned char * binDataSource;
  double statW = 0.0;
  double laserTilt = 0.0;
  bool statValid = false;
  UPose pose;
  ULaserPi pi;
  unsigned long serial = 0;
  ULaserDataSet * scan = NULL;
  bool gotX = false;
  bool gotY = false;
  bool gotTh = false;
  UPosition pos;
  URotation rot;
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
      scan->setData(serial, dataFirst, dataInterval, dataCount, t, unit, statValid, statW, laserTilt);
      if (gotX and gotY and gotTh)
      {
        scan->setPose(pose);
        setPoseIfNewer(pose, t);
      }
      else if (poseHistCnt > 0)
        scan->setPose(poseHist[poseHistNewest]);
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
      else if (binTag.isTagA("pose"))
      { // robot pose at scantime
        if (binTag.getPose(&pose))
          scan->setPose(pose);
        setPoseIfNewer(pose, t);
      }
      else if (binTag.isTagA("pos3d"))
      { // device position on robot
        if (binTag.getPosition(&pos))
        {
          scan->getSensorPose()->setPos(pos);
          laserPose.setPos(pos);
        }
      }
      else if (binTag.isTagA("rot3d"))
      { // device roattion on robot
        if (binTag.getRotation(&rot))
        {
          scan->getSensorPose()->setRot(rot);
          laserPose.setRot(rot);
        }
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
    gotNewData(scan);
  //printf("UClientFuncLaser::handleLaserScan: Finished %s with %d measurements\n", bool2str(result), n);
  return result;
}

//////////////////////////////////////////////////////

bool UClientFuncLaser::handleWpc(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag ciTag;
  UTime t;
  bool wpOK = false;
  int cnt = 0;
  int n;
  UPosition pos;
  // <wpcGet curWpX=5.00 curWpY=0.00 curWpOK=true wpCnt=100 motorLeft=0.500000 motorRight=0.500000>
  // <waypoint n=0 x=5.000 y=0.000/>
  // <waypoint n=1 x=5.000 y=0.000/>
  // <waypoint n=2 x=5.000 y=0.000/>
  // <waypoint n=3 x=5.000 y=0.000/>
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "motorLeft") == 0)
      sscanf(val, "%lf", &wpcData.motorLeft);
    else if (strcasecmp(att, "motorRight") == 0)
      sscanf(val, "%lf", &wpcData.motorRight);
    else if (strcasecmp(att, "curWpX") == 0)
      sscanf(val, "%lf", &wpcData.wpPos[0].x);
    else if (strcasecmp(att, "curWpY") == 0)
      sscanf(val, "%lf", &wpcData.wpPos[0].y);
    else if (strcasecmp(att, "curWpOK") == 0)
      wpOK = str2bool(val);
    else if (strcasecmp(att, "wpCnt") == 0)
      sscanf(val, "%d", &cnt);
    else if (strcasecmp(att, "speed") == 0)
      wpcData.speedCurrent = strtod(val, NULL);
    else if (strcasecmp(att, "speedPlanned") == 0)
      wpcData.speedPlanned = strtod(val, NULL);
    else if (strcasecmp(att, "speedAdvice") == 0)
      wpcData.speedAdvice = strtod(val, NULL);
  }
  if (not wpOK)
    wpcData.wpPosCnt = 0;
  else
    wpcData.wpPosCnt = 1;
  // zero count of control intervals
  // get new control intervals
  if (tag->isAStartTag())
  { // get next tag - should be a control interval
    wpcData.wpPosCnt = 0;
    while (result)
    {
      result = tag->getNextTag(&ciTag, 400);
      if (not result)
        break;
      else if (ciTag.isTagA("waypoint"))
      { // get waypoint attributes
        // <waypoint n=7 x=3.098 y=0.782/>
        while (ciTag.getNextAttribute(att, val, MVL))
        { // decode message
          if (strcasecmp(att, "n") == 0)
            sscanf(val, "%d", &n);
          else if (strcasecmp(att, "x") == 0)
            sscanf(val, "%lf", &pos.x);
          else if (strcasecmp(att, "y") == 0)
            sscanf(val, "%lf", &pos.y);
        }
        if (wpcData.wpPosCnt < MAX_WP_POSITIONS)
        {
          wpcData.wpPos[wpcData.wpPosCnt] = pos;
          wpcData.wpPosCnt++;
        }
      }
      else if (ciTag.isTagAnEnd(tag->getTagName()))
        // no more data -- should exit here
        break;
      else
        // unknown, just print and skip
        ciTag.print("");
    }
  }
  // announce the new data
  if (result)
    ;//gotNewData(NULL);
  else
    printf("UClientFuncLaser::handleWpc: "
             "failed to decode WPC data (no close tag found)\n");
  return result;
}

////////////////////////////////////////////////////

bool UClientFuncLaser::handleWpf(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag ciTag;
  UTime t;
  // debug
  //tag->print("handleImages got:");
  // debug end
// <srget wpf=true odoX=0.00 odoY=0.00 odoDeg=0.00 toX=30.00 toY=0.00 find=true gotoWaypoint=true>
// <wpfCi leftX=1.423 leftY=-1.542 rightX=2.295 rightY=-0.040/>
// <wpfCi leftX=3.098 leftY=0.782 rightX=2.948 rightY=3.088/>
// <srget/>
//  ciCnt = 0;
//  ciCorrCnt = 0;

  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "odoX") == 0)
      sscanf(val, "%lf", &odoPose.x);
    else if (strcasecmp(att, "odoY") == 0)
      sscanf(val, "%lf", &odoPose.y);
    else if (strcasecmp(att, "odoH") == 0)
      sscanf(val, "%lf", &odoPose.h);
    else if (strcasecmp(att, "toX") == 0)
      sscanf(val, "%lf", &toPose.x);
    else if (strcasecmp(att, "toY") == 0)
      sscanf(val, "%lf", &toPose.y);
    else if (strcasecmp(att, "laserPosX") == 0)
      laserPose.setX(strtod(val, NULL));
    else if (strcasecmp(att, "laserPosY") == 0)
      laserPose.setY(strtod(val, NULL));
    else if (strcasecmp(att, "laserPosZ") == 0)
      laserPose.setZ(strtod(val, NULL));
  }
  // zero count of control intervals
  // get new control intervals
  if (tag->isAStartTag())
    printf("UClientFuncLaser::handleLaserSr: wpfCi intervals are no longer valid\n");
/*  { // get next tag - should be a control interval
    result = cnn->getNextTag(&ciTag, 400);
    while (result)
    { // after a srGet there can be a number of tag  types:
      if (ciTag.isTagA("wpfCi"))
        result = handleControlIntervals(&ciTag);
      else if (ciTag.isTagAnEnd(tag->getTagName()))
        // no more data
        break;
      else
      { // unknown, just print and skip
        ciTag.print("");
        result = cnn->getNextTag(&ciTag, 400);
      }
    }
  }*/
  // announce the new data
  if (result)
    ; //gotNewData(NULL);
  else
    printf("UClientFuncLaser::handleLaserSr: "
             "no new tag within timeout period\n");
  return result;
}

///////////////////////////////////////////////////////////

// bool UClientFuncLaser::handleControlIntervals(USmlTag * ciTag)
// {
//   bool result = ciTag != NULL;
//   bool isACi;    // raw control interval candidate
//   bool isACiB;   // support point for raw control interval 0.0 is none
//   bool isACiCorr; // modified control interval
//   bool isACiResult; // produced 5.0 m waypoint
//   bool gotData;
//   const int MVL = 320; //
//   char att[MAX_SML_NAME_LENGTH];
//   char val[MVL];
//   UPosition posL, posR;
//   int scan;
//   int scanCi;
//   int n;
//   //
//   if (result)
//   {
//     ciCnt = 0;
//     ciCorrCnt = 0;
//     while (ciTag->isTagA("wpfCi") and (ciCnt < MAX_CI_INTERVALS))
//     { // laser value tag
//       // <wpfCi leftX=3.098 leftY=0.782 rightX=2.948 rightY=3.088/>
//       gotData = false;
//       isACi = false;
//       isACiCorr = false;
//       isACiResult = false;
//       isACiB = false;
//       scan = -1;
//       scanCi = -1;
//       while (ciTag->getNextAttribute(att, val, MVL))
//       { // decode message
//         if (strcasecmp(att, "ciRaw") == 0)
//           isACi = true;
//         else if (strcasecmp(att, "ciRawImp") == 0)
//           isACiB = true;
//         else if (strcasecmp(att, "ciCorr") == 0)
//         {
//           isACiCorr = true;
//           n = sscanf(val, "%d", &scan);
//           if (n == 1)
//             scanCi++;
//         }
//         else if (strcasecmp(att, "ciResult") == 0)
//           isACiResult = true;
//         else if (strcasecmp(att, "leftX") == 0)
//           sscanf(val, "%lf", &posL.x);
//         else if (strcasecmp(att, "leftY") == 0)
//         {
//           sscanf(val, "%lf", &posL.y);
//           gotData = true;
//         }
//         else if (strcasecmp(att, "rightX") == 0)
//           sscanf(val, "%lf", &posR.x);
//         else if (strcasecmp(att, "rightY") == 0)
//         {
//           sscanf(val, "%lf", &posR.y);
//           gotData = true;
//         }
//       }
//       if (gotData)
//       {
//         if (isACi)
//         { // raw control interval
//           if (ciCnt < MAX_CI_INTERVALS)
//           {
//             ciLeft[ciCnt] = posL;
//             ciRight[ciCnt] = posR;
//             ciLeftB[ciCnt].clear();
//             ciRightB[ciCnt].clear();
//             ciCnt++;
//           }
//         }
//         else if (isACiB)
//         { // raw control interval
//           if ((ciCnt < MAX_CI_INTERVALS) and
//               (ciCnt > 0))
//           { // CI comes before ciB
//             ciLeftB[ciCnt-1] = posL;
//             ciRightB[ciCnt-1] = posR;
//           }
//         }
//         else if (isACiCorr)
//         { // checked control interval
//           if (ciCorrCnt < MAX_CI_INTERVALS)
//           {
//             ciCorrLeft[ciCorrCnt] = posL;
//             ciCorrRight[ciCorrCnt] = posR;
//             ciCorrScan[ciCorrCnt] = scan;
//             ciCorrIdx[ciCorrCnt] = scanCi;
//             ciCorrCnt++;
//           }
//         }
//         else if (isACiResult)
//         { // added waypoint and current goal position
//           ciResult = posL;
//           ciGoal = posR;
//         }
//       }
//       result = cnn->getNextTag(ciTag, 200);
//     }
//   }
//   return result;
// }

//////////////////////////////////////////////////////

bool UClientFuncLaser::gotNewData(ULaserDataSet * scan)
{
  scan->print("Got new data", true);
  return true;
}


///////////////////////////////////////////////////

void UClientFuncLaser::setPose(UPoseTime poseTime)
{
  odoPose = poseTime;
  // add to history
  if (poseHistCnt < MAX_POSE_HIST)
    poseHistNewest = poseHistCnt++;
  else
  {
    poseHistNewest++;
    if (poseHistNewest >= MAX_POSE_HIST)
      poseHistNewest = 0;
  }
  poseHist[poseHistNewest] = poseTime;
}

////////////////////////////////////////////////////

void UClientFuncLaser::setPoseIfNewer(UPose pose, UTime t)
{
  UPoseTime pt(pose, t);
  //
  if (poseHistCnt > 0)
  {
    if ((t - poseHist[poseHistNewest].t) > 0.0)
      setPose(pt);
  }
  else
    setPose(pt);
}

////////////////////////////////////////////////////

bool UClientFuncLaser::handleOdo(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag ciTag;
  UTime t;
  long int sec, usec;
  //bool running = false;
  //bool log = false;
  UPoseTime pose;
/*<odoGet running=true log=true tod=1111254673.775733 x=0.000 y=0.000 h=0.00000>
  <poseHist tod=1111254673.7757 x=0.000000 y=0.000000 h=0.000000/>
  <poseHist tod=1111254673.7757 x=0.000000 y=0.000000 h=0.000000/>
  <poseHist tod=1111254673.7757 x=0.000000 y=0.000000 h=0.000000/>
  <poseHist tod=1111254673.7757 x=0.000000 y=0.000000 h=0.000000/>
  <poseHist tod=1111254673.7757 x=0.000000 y=0.000000 h=0.000000/>
  </odoGrt>*/
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "tod") == 0)
      sscanf(val, "%ld.%ld", &sec, &usec);
    else if (strcasecmp(att, "running") == 0)
      ;//running = str2bool(val);
    else if (strcasecmp(att, "log") == 0)
      ; //log = str2bool(val);
    else if (strcasecmp(att, "x") == 0)
      sscanf(val, "%lf", &odoPose.x);
    else if (strcasecmp(att, "y") == 0)
      sscanf(val, "%lf", &odoPose.y);
    else if (strcasecmp(att, "h") == 0)
      sscanf(val, "%lf", &odoPose.h);
  }
  //
  odoPose.t.setTime(sec, usec);
  // get new control intervals
  if (tag->isAStartTag())
  { // get next tag - should be a control interval
    poseHistCnt = 0;
    while (result)
    {
      result = tag->getNextTag(&ciTag, 400);
      if (not result)
        break;
      else if (ciTag.isTagA("poseHist"))
      { // get waypoint attributes
        // <waypoint n=7 x=3.098 y=0.782/>
        while (ciTag.getNextAttribute(att, val, MVL))
        { // decode message
          if (strcasecmp(att, "tod") == 0)
            sscanf(val, "%ld.%ld", &sec, &usec);
          else if (strcasecmp(att, "x") == 0)
            sscanf(val, "%lf", &pose.x);
          else if (strcasecmp(att, "y") == 0)
            sscanf(val, "%lf", &pose.y);
          else if (strcasecmp(att, "h") == 0)
            sscanf(val, "%lf", &pose.h);
        }
        pose.t.setTime(sec, usec);
        if (poseHistCnt < MAX_POSE_HIST)
        {
          poseHist[poseHistCnt] = pose;
          poseHistCnt++;
        }
      }
      else if (ciTag.isTagAnEnd(tag->getTagName()))
        // no more data -- should exit here
        break;
      else
        // unknown, just print and skip
        ciTag.print("");
    }
  }
  // announce the new data
  if (result)
    ; //gotNewData(NULL);
  else
    printf("UClientFuncLaser::handleOdo: "
             "failed to decode odo data (no close tag found)\n");
  return result;
}

////////////////////////////////////////////////////////

bool UClientFuncLaser::handleEkf(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag ciTag;
  UTime t;
  long int sec, usec;
  //bool running = false;
  //bool log = false;
  UPose pose;
/*
<ekfGet running=true log=false uSkip=673 uUsed=0 tod=1111988817.860070 x=0.000 y=0.000 h=0.00000/>
*/
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "tod") == 0)
      sscanf(val, "%ld.%ld", &sec, &usec);
    else if (strcasecmp(att, "running") == 0)
      ; //running = str2bool(val);
    else if (strcasecmp(att, "log") == 0)
      ; //log = str2bool(val);
    else if (strcasecmp(att, "x") == 0)
      sscanf(val, "%lf", &ekfPose.x);
    else if (strcasecmp(att, "y") == 0)
      sscanf(val, "%lf", &ekfPose.y);
    else if (strcasecmp(att, "h") == 0)
      sscanf(val, "%lf", &ekfPose.h);
    else if (strcasecmp(att, "uSkip") == 0)
      sscanf(val, "%d", &ekfuSkipd);
    else if (strcasecmp(att, "uUsed") == 0)
      sscanf(val, "%d", &ekfuUsed);
  }
  //
  ekfTime.setTime(sec, usec);
  // get new control intervals
  // announce the new data
  if (result)
    gotNewEkfData();
  else
    printf("UClientFuncLaser::handleEkf: "
             "failed to decode odo data (no close tag found)\n");
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::gotNewEkfData()
{
  // nothing to do here - used if GUI part
  return true;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::gotNewFreePolyData()
{
  // nothing to do here
  return true;
}

////////////////////////////////////////////////////////

bool UClientFuncLaser::handlePlan(USmlTag * tag)
{
  enum Units {MM, CM, DM};
  enum Format {TAG, HEX, BIN};
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  UPose pose;
  USmlTag nTag;
  UPlannerStopCrit * stopCrit;
  char * p1;
/*
<planget running=true file="mission.txt" usingFile=true cmd="gotowaypoint odo 10.0 0.0  (get GPS)" isIdle=false/>
*/
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "running") == 0)
      planner.running = str2bool(val);
    else if (strcasecmp(att, "file") == 0)
      strncpy(planner.cmdFile, val, MAX_PLANNER_LINE_SIZE);
    else if (strcasecmp(att, "cmd") == 0)
    { // and terminate at line-feed
      strncpy(planner.cmdLine, val, MAX_PLANNER_LINE_SIZE);
      p1 = strchr(planner.cmdLine, '\n');
      if (p1 != NULL)
        *p1 = '\0';
    }
    else if (strcasecmp(att, "cmdLineNum") == 0)
      planner.cmdLineNum = strtol(val, NULL, 0);
    else if (strcasecmp(att, "cmdLineManual") == 0)
      planner.cmdLineOverride = str2bool(val);
    else if (strcasecmp(att, "cmdLast") == 0)
      strncpy(planner.cmdLineLast, val, MAX_PLANNER_LINE_SIZE);
    else if (strcasecmp(att, "lastStopBy") == 0)
      strncpy(planner.cmdStoppedBy, val, MAX_PLANNER_STOP_CRIT_SIZE);
    else if (strcasecmp(att, "isIdle") == 0)
      planner.isIdle = str2bool(val);
    else if (strcasecmp(att, "simulated") == 0)
      planner.simulated = str2bool(val);
    else if (strcasecmp(att, "simSubDir") == 0)
      strncpy(planner.simSubDir, val, MAX_FILENAME_SIZE);
  }
  if (tag->isAStartTag())
  { // there is more tags - stop criterias
    planner.stopCritCnt = 0;
    stopCrit = planner.stopCrit;
    while (result)
    {
      result = tag->getNextTag(&nTag, 400);
      if (not result)
        break;
      else if (nTag.isTagA("stopCrit"))
      { // get waypoint attributes
        // <waypoint n=7 x=3.098 y=0.782/>
        while (nTag.getNextAttribute(att, val, MVL))
        { // decode message
          if (strcasecmp(att, "name") == 0)
            strncpy(stopCrit->name, val, MAX_PLANNER_STOP_CRIT_SIZE);
          else if (strcasecmp(att, "active") == 0)
            stopCrit->active = str2bool(val);
          else if (strcasecmp(att, "status") == 0)
            strncpy(stopCrit->status, val, MAX_STOP_CRIT_STATUS_LENGTH);
        }
        if (planner.stopCritCnt < MAX_STOP_CRIT_COUNT)
        {
          stopCrit++;
          planner.stopCritCnt++;
        }
      }
      else if (nTag.isTagAnEnd(tag->getTagName()))
        // no more data -- should exit here
        break;
      else
        // unknown, just print and skip
        nTag.print("");
    }
  }
  // announce the new data
  if (result)
    gotNewPlanData();
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::gotNewPlanData()
{
  // nothing to do here - possibly used in GUI part
  return true;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::handlePath(USmlTag * tag)
{
  bool result = true;
  const int MVL = 320; //
  char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag nTag;
  int cnt = 0;
  bool gotData = false;
  UTime t;
  UPoseV pv;
  bool setPose;
/*
<pathget pathCnt=2>
<path intervals=5 path=false edgeLeft=true edgeRight=true edgeTop=true>
<lineSeg name="leftEdge" length=1.95030689239502e+00>
<pos3d name="start" x=2.15716743469238e+00 y=-1.88146317005157e+00 z=1.80217644064948e-01/>
<pos3d name="vector" x=9.99599030828172e-01 y=5.72597424506551e-03 z=-2.77303761985254e-02/>
<lineSeg/>
<manseq cnt="2" endV="0.455" dt="24.215">
<posev name="start" x="0.034" y="-0.001" h="-0.062" v="0.115"/>
<posev name="end" x="3.873" y="-4.784" h="-0.062" v="0.800"/>
<manoeuvre typ="arc" rad="2.073" arc="-1.6653" acc="0.0140562" endVel="  0.8"/>
<manoeuvre typ="arc" rad="2.073" arc="1.6653" acc="0.0140562" endVel="  0.8"/>
</manseq>
  ...
</path>
</pathget>
*/
  pathsCnt = 0;
  setPose = false;
  t.clear();
  while (tag->getNextAttribute(att, val, MVL))
  {
    if (strcasecmp(att, "pathCnt") == 0)
      cnt = strtol(val, NULL, 0);
    if (strcasecmp(att, "cnt") == 0)
      cnt = strtol(val, NULL, 0);
  }
  if (tag->isAStartTag())
  {
    while (result)
    {
      result = tag->getNextTag(&nTag, 400);
      if (result and nTag.isTagA("path"))
      { // there is a path
        if (nTag.getAttValue("tod", val, MVL) and not setPose)
          t.setTimeTod(val);
        if (pathsCnt < MAX_NEW_PATH_CANDIDATES)
        {
          if (paths[pathsCnt] == NULL)
            paths[pathsCnt] = new ULaserPathResult();
          result = (paths[pathsCnt] != NULL);
          if (result)
            gotData |= paths[pathsCnt]->setFromTag(&nTag);
        }
        if (not setPose and (t.getSec() > 0))
        { // set robot pose if not set already by other data sources
          setPose = true;
          if (t > odoPose.t)
          {
            pv = paths[pathsCnt]->getManSeq()->getStartPoseV();
            odoPose.setPt(pv.x, pv.y, pv.h, t);
          }
        }
        if (result and (pathsCnt < MAX_NEW_PATH_CANDIDATES))
          pathsCnt++;
      }
      else if (nTag.isTagAnEnd(tag->getTagName()))
        // end of all paths for this time.
        break;
    }
  }
  if ((cnt != pathsCnt) and verboseMessages)
    // @todo some paths are counter, but not valid and not send
    printf("UClientFuncLaser::handlePath: expected %d paths got %d\n",
      cnt, pathsCnt);
  // announce the new data
  if (gotData)
    gotNewData(NULL);
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::gotNewPathData()
{

  //
  // debug
  int i;
  printf("Received %d paths\n", pathsCnt);
  for (i = 0; i < pathsCnt; i++)
  {
    if (paths[i] != NULL)
      printf(" -- #%d has %d lines\n", i, paths[i]->getPassLinesCnt());
  }
  // debug end
  //
  // nothing to do here - used if GUI part
  return true;
}


/////////////////////////////////////////////////////////////////////////

UTime UClientFuncLaser::getLaserTime(int histNum)
{
  UTime result;
  ULaserDataSet * lp;
  //
  lp = scanHist.getScan(histNum);
  if (lp != NULL)
    result = lp->getTime();
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UClientFuncLaser::handleGetComplexData(USmlTag * tag)
{
  bool result;
  const int MVL = 320; //
  //char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag nTag;
  UProbPoly * poly;
  int n = freePolyNewest + 1;
  bool isOldNews;
  int m = 0;
  /*
  <get name="FreeArea" type="UProbPoly">
  <UProbPoly count="44" obstFlags="true">
  <time name="imgTime" sec="1128975788" usec="193675"/>
  <pose name="org" x="7.82321" y="-0.428225" h="0.0818329"/>
  <pos2db x="412.681" y="3.5506" obst="false"/>
  <pos2db x="413.125" y="3.42498" obst="true"/>
  <pos2db x="412.691" y="3.58566" obst="false"/>
  <pos2db x="412.916" y="4.3494" obst="false"/>
  <pos2db x="417.881" y="4.45332" obst="false"/>
  <pos2db x="418.16" y="4.37376" obst="false"/>
  </UProbPoly>
  </get>
  */
  pathsCnt = 0;
  tag->getAttValue("name", val, MVL);
  result = (strcasecmp(val, "FreeArea") == 0);
  while (result)
  {
    result = tag->getNextTag(&nTag, 400);
    if (result and nTag.isTagA("UProbPoly"))
    { // it is a polygon - decode
      if (n >= MAX_FREE_POLY_HIST)
        n = 0;
      poly = &freePoly[n];
      poly->setValid(false);
      result = nTag.getProbPoly(poly);
      m = poly->getPointsCnt();
      if (result)
      {
        isOldNews = freePolyNewest >= 0;
        if (isOldNews)
          isOldNews = (absf(poly->getPoseTime() -
              freePoly[freePolyNewest].getPoseTime())) < 0.001;
        if (not isOldNews)
        {
          freePolyNewest = n;
          if (n >= freePolyCnt)
            freePolyCnt = n + 1;
        }
      }
    }
    else if (nTag.isTagAnEnd(tag->getTagName()))
      // end of all paths for this time.
      break;
    else if (nTag.isAStartTag())
      // unknown group - skip
      nTag.skipToEndTag(400);
  }
  if (verboseMessages)
    // @todo some paths are counter, but not valid and not send
    printf("UClientFuncLaser::handleGetComplexData: got new data (polygon %d points)\n", m);
  // announce the new data
  gotNewFreePolyData();
  //
  return result;
}

//////////////////////////////////////////////////////////////

bool UClientFuncLaser::handleVarData(USmlTag * tag)
{
  bool result;
  const int MVL = 320; //
  //char att[MAX_SML_NAME_LENGTH];
  char val[MVL];
  USmlTag nTag;
  /*
  <get name="FreeArea" type="UProbPoly">
  <poseHist">
  <pose name="newest" x=123.34 y=456.78 h=1.2345"/>
  <pose name="org" x="7.82321" y="-0.428225" h="0.0818329"/>
  </poseHist>
  */
  pathsCnt = 0;
  tag->getAttValue("name", val, MVL);
  result = tag->isAStartTag();
  while (result)
  {
    result = tag->getNextTag(&nTag, 400);
    if (nTag.isTagA("pose") or nTag.isTagA("poseTime"))
    { // it is a polygon - decode
      planner.setVarFromTag(&nTag, &odoPose);
    }
    else if (nTag.isTagAnEnd(tag->getTagName()))
      // end of all paths for this time.
      break;
    else if (nTag.isAStartTag())
      // unknown group - skip
      nTag.skipToEndTag(400);
  }
  if (verboseMessages)
    // @todo some paths are counter, but not valid and not send
    printf("UClientFuncLaser::handleVarData a '%s' got new data\n", tag->getTagName());
  // announce the new data
  result = tag->isAStartTag();
  if (result)
    gotNewData(NULL);
  //
  return result;
}
