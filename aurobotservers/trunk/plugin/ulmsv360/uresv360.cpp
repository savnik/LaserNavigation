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

#include <urob4/uresposehist.h>

#include "uresv360.h"


UResV360::~UResV360()
{
  if (v360 != NULL)
    delete v360;
}

//////////////////////////////////////////////////////

bool UResV360::gotAllResources(char * missingThese, int missingTheseCnt)
{ // save pointers as needed
  bool gotPoseHist, gotLasPool;
  int n = 0;
  char * p1 = missingThese;
  //
  gotPoseHist = (poseHist != NULL);
  if ((not gotPoseHist) and (p1 != NULL))
    // missing the 'poseHist' resource
    snprintf(p1, missingTheseCnt - n, " %s", UResPoseHist::getOdoPoseID());
  gotLasPool = (lasPool != NULL);
  if ((not gotLasPool) and (p1 != NULL))
  { // missing the 'lasPool' resource
    n = strlen(p1);
    p1 = &missingThese[n];
    snprintf(p1, missingTheseCnt - n, " %s", ULaserPool::getResClassID());
  }
  return gotLasPool and gotPoseHist;
}

//////////////////////////////////////////////////////////////////

bool UResV360::setResource(UResBase * resource, bool remove)
{
  bool result = true;
  if (resource->isA(UResPoseHist::getOdoPoseID()))
  { // this ressource - use as appropriate
    lock();
    if (remove)
      poseHist = NULL;
    else if (poseHist != (UResPoseHist *)resource)
      poseHist = (UResPoseHist *)resource;
    else
      result = false;
    unlock();
  }
  else if (resource->isA(ULaserPool::getResClassID()))
  { // this ressource - use as appropriate
    lock();
    if (remove)
      lasPool = NULL;
    else if (lasPool != (ULaserPool *)resource)
      lasPool = (ULaserPool *)resource;
    else
      result = false;
    unlock();
  }
  else
  {
    result = UServerPush::setResource(resource, remove);
  }
  return result;
}

////////////////////////////////////////////////////////

void  UResV360::createBaseVar()
{
  var.notValidArea = addVar("notValidArea", "0 0 0 0", "d", "(r/w) Area in new scans that are to be considered invalid (x,y,g,h), where (x,y) is top-left corner, (g,h) is back-right corner relative to scanner coordinates");
  var.lastSerial = addVar("lastSerial", 0.0, "d", "(r) last scan serial number");
  var.updateCnt = addVar("updateCnt", 0.0, "d", "(r) Number of updates received by virtual scanner");
  var.updateTime = addVar("updateTime", 0.0, "t", "(r) Timestamp of last update");
  var.resolution = addVar("resolution", 0.0, "d", "(r) measurement resolution in degrees");
}

/////////////////////////////////////////////////////

bool UResV360::getNewestData(ULaserData * laserData,
            int last, int fake)
{
  bool result;
  UPoseTime pt;
  UTime scantime;
  unsigned long newest;
  //
  result = gotAllResources(NULL, 0);
  // create scan if not created yet
  if (v360 == NULL)
  { // try to obtain a scan from default source
    result = update(NULL, NULL, fake);
  }
  //
  if (result)
  { // get newest data
    lock();
    newest = v360->getScanSerial();
    if (last <= 0 or last < (int)newest)
      result = v360->getScan(laserData);
    else
      result = false;
    unlock();
  }
  //
  return result;
}

/////////////////////////////////////////////////////

const char * UResV360::print(const char * preString, char * buff, int buffCnt)
{
  UPoseTime pt;
  //
  if (v360 != NULL)
  {
    pt = v360->getCurrentPose();
    snprintf(buff, buffCnt,
           "%s scan %lu at pose %.2fx %.2fy %.1fdeg (%d pos in %d sectors)\n",
           preString,
           v360->getScanSerial(),
           pt.x, pt.y, pt.h * 180.0 / M_PI,
            v360->getMeasurementCnt(),
            v360->getSectorCnt());
  }
  else
  {
    snprintf(buff, buffCnt, "%s %s is yet unsed\n",
            preString, getResID());
  }
  return buff;
}

/////////////////////////////////////////////////////

bool UResV360::getSourceData(ULaserData * laserData, int last, int fake)
{
  ULaserDevice * ld;
  bool result;
  //
  result = (lasPool != NULL);
  if (result)
  { // get default device
    if (defDevice == -1)
      ld = lasPool->getDefDevice();
    else
      ld = lasPool->getDevice(defDevice);
    result = (ld != NULL);
  }
  if (result)
  { // get data
    result = ld->getNewestData(&sourceData, last, fake);
  }
  //
  return result;
}

///////////////////////////////////////////////////////

UPose UResV360::getLaserPose()
{
  UPose result;
  if (v360 == NULL)
    v360 = new UV360Scan();
  if (v360 != NULL)
    result = v360->getLaserPose();
  return result;
}

///////////////////////////////////////////////////////

void UResV360::setLaserPose(UPose newLaserPose)
{
  if (v360 == NULL)
    v360 = new UV360Scan();
  if (v360 != NULL)
    v360->setLaserPose(newLaserPose);
}

///////////////////////////////////////////////////////

bool UResV360::update(ULaserData * laserData, ULaserDevice * laserDevice, int fake)
{
  bool result;
  UPoseTime pt;
  UTime scantime;
  ULaserData * lasData = laserData;
  UPose devPose;
  UPosRot fullPose;
  U2Dpos notValid[2];
  //
  
  lock();
  result = gotAllResources(NULL, 0);
  // get not valid square
  for (int i = 0; i < 2; i++)
  {
    notValid[i].x = var.notValidArea->getDouble(i*2);
    notValid[i].y = var.notValidArea->getDouble(i*2 + 1);
  }
  // create scan if not created yet
  if (result)
  {
    lasDev = laserDevice;
    if (v360 == NULL)
      v360 = new UV360Scan();
    if (lasDev == NULL)
    {
      if (lasData != NULL)
      {
        sourceData.setDeviceNum(lasData->getDeviceNum());
        lasDev = lasPool->getDevice(lasData->getDeviceNum());
      }
      else if (defDevice == -1)
        lasDev = lasPool->getDefDevice();
      else
        lasDev = lasPool->getDevice(defDevice);
    }
    result = lasDev != NULL and v360 != NULL;
  }
  //
  if (result and lasData == NULL)
  {
    lasData = &sourceData;
    result = lasDev->getNewestData(lasData, lasData->getSerial(), fake);
  }
  if (result and lasData->isValid())
  {  // get fresh data
    scantime = lasData->getScanTime();
    if (fake == 0)
      pt.setPt(poseHist->getPoseAtTime(scantime), scantime);
    else
      pt = lasData->getFakePose();
    // process data
    fullPose = lasDev->getDevicePose();
    // full pose 6deg-pose is not supported
    devPose.set(fullPose.x, fullPose.y, fullPose.Kappa);
    // reduce to 3deg-pose
    v360->setLaserPose(devPose);
    // update scan
    result = v360->update(lasData, pt, notValid);
    //
    if (result)
    {
      var.lastSerial->setValued(lasData->getSerial());
      var.updateCnt->add(1.0);
      var.lastSerial->setValued(lasData->getSerial());
      var.updateTime->setValued(scantime.GetDecSec());
      var.resolution->setValued(v360->getResolution());
      // make a push event
      setUpdated("");
    }
  }
  unlock();
  //
  return result;
}

////////////////////////////////////////////////////////

int UResV360::setDefDevice(const char * devString)
{
  int m;
  char * p1;
  ULaserDevice * dev;
  //
  m = strtol(devString, &p1, 0);
  if (devString == p1)
  { // string is probably a name rather than a number
    dev = lasPool->getDevice(devString);
    if (dev != NULL)
      defDevice = dev->getDeviceNum();
  }
  else
    defDevice = m;
  return defDevice;
}

/////////////////////////////////////

void UResV360::callGotNewDataWithObject()
{
  bool gotData;
  //
  lock();
  gotData = v360->getScan(&pushData);
  // set device as source device, to get device pose etc.
  pushData.setDeviceNum(sourceData.getDeviceNum());
  unlock();
  if (gotData)
  { // start the push handling
    gotNewData(&pushData);
  }
}
