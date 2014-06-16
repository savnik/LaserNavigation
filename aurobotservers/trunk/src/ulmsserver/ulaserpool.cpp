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

#include <urob4/uvariable.h>
#include <urob4/uvarcalc.h>

#include "ureplaydevice.h"
#include "ulaserpool.h"

//////////////////////////////////////////
#include <ulms4/uresv360.h>

/// actual fake map and robot position in the map
UFakeMap fakeMap;


////////////////////////////////////////////////////////////


void ULaserPool::ULaserPoolInit()
{
  int i;
  // remaining initialization
  lasDevsCnt = 0;
  for (i = 0; i < MAX_LASER_DEVS; i++)
  {
    lasDevs[i] = NULL;
  }
  //defaultDevice = 0;
  // this is not realy a replay device, but set as such
  // to get information on replay time advances:
  replay = true;
  //
  cmdExe = NULL;
  //allow fake map to export map using plug-in calls
  fakeMap.varPool = this;
}

//////////////////////////////////////////

ULaserPool::~ULaserPool()
{
  int i;
  //
  for (i = 0; i < lasDevsCnt; i++)
  {
    if (lasDevs[i] != NULL)
      delete lasDevs[i];
  }
}

void ULaserPool::createBaseVar()
{
  var.deviceCnt = addVar("deviceCnt", 0.0, "d", "(r) number of laser scanner devices devices");
  var.devices = addVar("devices", "", "s", "(r) list of valid device names");
  var.def = addVar("default", 0.0, "d", "(r) default laser scanner, if not explicit specified");
}

///////////////////////////////////////////

ULaserDevice * ULaserPool::getDevice(int deviceNumber)
{
  if ((deviceNumber >= 0) and
       (deviceNumber < lasDevsCnt))
    return lasDevs[deviceNumber];
  else
    return NULL;
}

///////////////////////////////////////////

ULaserDevice * ULaserPool::getDevice(const char * deviceID)
{
  ULaserDevice * ld;
  int n, i, m;
  char * p1;
  //
  n = strtol(deviceID, &p1, 10);
  if (p1 == deviceID)
  { // ID is no number, may be device type
    n = MAX_LASER_DEVS; // not valid number
    for (i = 0; i < lasDevsCnt; i++)
    {
      ld = lasDevs[i];
      if (strcasecmp(deviceID, ld->getDeviceName()) == 0)
      { // device type match (sim, urg, sick, log) - use first name match
        n = i;
        break;
      }
      if (strcasecmp(deviceID, ld->getName()) == 0)
      { // device source name match (e.g. /dev/ttyUSB0 or localhost:19005)
        n = i;
        break;
      }
    }
  }
  else if (n > MAX_LASER_DEVS)
  { // may be a sim laser port number
    for (i = 0; i < lasDevsCnt; i++)
    {
      ld = lasDevs[i];
      p1 = strchr(ld->getDeviceName(), ':');
      if (p1 != NULL)
      { // a sim device with a port number
        m = strtol(++p1, NULL, 10);
        if (m == n)
        { // device is found - set n to device number
          n = i;
          break;
        }
      }
    }
  }
  if ((n >= 0) and
       (n < lasDevsCnt))
    return lasDevs[n];
  else
    return NULL;
}

/////////////////////////////////////////////

bool ULaserPool::addDevice(ULaserDevice * newDevice)
{
  bool result = false;
  int n;
  const int MSL = 12;
  char s[MSL];
  const int MCL = 150;
  char sc[MCL];
  UVarPool * devVars;
  //
  if (lasDevsCnt < MAX_LASER_DEVS)
  {
    lasDevs[lasDevsCnt] = newDevice;
    newDevice->setCmdExe(cmdExe); // server push
    newDevice->setCore(cmdExe);   // debug extra (odoPose fix)
    newDevice->setDeviceNum(lasDevsCnt);
    snprintf(s, MSL, "dev%d", lasDevsCnt);
    lasDevsCnt++;
    // update global variables
    var.deviceCnt->setValued(lasDevsCnt, 0);
    n = var.devices->getSize();
    if (n > 0)
      n++;
    var.devices->setValues(newDevice->getName(), n, true);
    // create new local structure with data for device
    snprintf(sc, MCL, "Laser scanner device, a %s at %s",
             newDevice->getName(), newDevice->getDeviceName());
    devVars = addStruct(s, sc, true);
    // let device create own global variables
    newDevice->setVarStructure(devVars);
    newDevice->createBaseVars();
    //
    result = true;
  }
  return result;
}

/////////////////////////////////////////////

int ULaserPool::getDeviceNumber(ULaserDevice * device)
{
  int n, result = -1;
  for (n = 0; n < lasDevsCnt; n++)
  {
    if (lasDevs[n] == device)
    {
      result = n;
      break;
    }
  }
  return result;
}

/////////////////////////////////////////////

void ULaserPool::setCmdExe(UCmdExe * server)
{ // for push commands
  int i;
  //
  for (i = 0; i < lasDevsCnt; i++)
    lasDevs[i]->setCmdExe(server);
}

//////////////////////////////////////////////////////////////////

bool ULaserPool::setResource(UResBase * res, bool remove)
{
  bool result = true;
  //
  if (res->isA(UCmdExe::getResClassID()))
  {
    if (remove)
      cmdExe = NULL;
    else if (cmdExe != (UCmdExe *)res)
      cmdExe = (UCmdExe *)res;
    else
      result = false;
  }
  else
    result = UResBase::setResource(res, remove);
  return result;
}

///////////////////////////////////////////////////////

const char * ULaserPool::print(const char * preString, char * buff, int buffCnt)
{
  int i, m = 0;
  ULaserDevice * d;
  const int SL = 80;
  char s[SL];
  char * p1 = buff;
  //
  snprintf(buff, buffCnt, "%s %d laser device(s)\n", preString, lasDevsCnt);
  for (i = 0; i < lasDevsCnt; i++)
  {
    m += strlen(p1);
    p1 = &buff[m];
    d = lasDevs[i];
    if (d != NULL)
    {
      snprintf(s, SL, " - dev %d ", i);
      d->print(s, p1, buffCnt - m);
    }
    else
      snprintf(p1, buffCnt - m, " - dev %d - not valid\n", i);
  }
  return buff;
}

///////////////////////////////////////////////////////

bool ULaserPool::gotAllResources(char * missingThese, int missingTheseCnt)
{
  bool result, isOK;
  int n = 0;
  char * p1 = missingThese;
  //
  result = (cmdExe != NULL);
  if ((not result) and (p1 != NULL))
  {
    snprintf(p1, missingTheseCnt, " %s", UCmdExe::getResClassID());
    n = strlen(p1);
    p1 = &missingThese[n];
  }
  isOK = UResBase::gotAllResources(p1, missingTheseCnt - n);
  return result and isOK;
}

///////////////////////////////////////////////////////


void ULaserPool::setDefDevice(int value)
{
  int defaultDevice = maxi(0, mini(lasDevsCnt - 1, value));
  var.def->setValued(defaultDevice, 0);
}

///////////////////////////////////////////////////////

bool ULaserPool::setDefDevice(const char * device)
{
  int i, n = -1;
  ULaserDevice * dv;
  //
  for (i = 0; i < lasDevsCnt; i++)
  {
    dv = getDevice(i);
    if (strcasecmp(dv->getName(), device) == 0)
      n = i;
  }
  if (n >= 0)
  {
    int defaultDevice = n;
    var.def->setValued(defaultDevice, 0);
  }
  return (n >= 0);
}

////////////////////////////////////////////////////

ULaserData * ULaserPool::getScan(const int device,
                                 ULaserDevice ** las,
                                 ULaserData * dataBuff,
                                 ULaserData * pushData,
                                 const int fake)
{
  ULaserData * scan = NULL;
  int n = device;
  unsigned long lastSerial;
  int lastDevice;
  ULaserDevice * theDevice;
  UResV360 * resV360 = NULL;
  //
  // get laser scanner device
  if (pushData != NULL)
    // device number is in scan data
    n = pushData->getDeviceNum();
  else if (device < 0)
    // use default device
    n = getDefDeviceNumber();
  else if (device == 360)
  { // get device number from virtual laser - if available
    resV360 = (UResV360 *)getStaticResource("v360", false, false);
    if (resV360 == NULL)
      n = getDefDeviceNumber();
  }
  // get laser device pointer
  if (n == 360)
    theDevice = resV360->getLastDevice();
  else
    theDevice = getDevice(n);
  // is push-data available
  if (pushData != 0)
  { // use this data
    scan = pushData;
    // save information on last used scan
    if (dataBuff != NULL)
    { // save info on last used scan
      dataBuff->setDeviceNum(n);
      dataBuff->setSerial(scan->getSerial());
    }
  }
  else if (dataBuff != NULL)
  { // no pushdata, so
    // get data from (default) device
    scan = dataBuff;
    // clear scannumber if new device
    lastDevice = dataBuff->getDeviceNum();
    if (n != lastDevice)
      lastSerial = 0;
    else
      lastSerial = dataBuff->getSerial();
    // get new scandata from device
    if (theDevice != NULL)
    { // get data
      if (n == 360)
        resV360->getNewestData(scan, lastSerial, fake);
      else
        theDevice->getNewestData(scan, lastSerial, fake);
    }
  }
  if (las != NULL)
    // return also a pointer to the laser device
    *las = theDevice;
  return scan;
}

///////////////////////////////////////////////////

bool ULaserPool::replayToTime(UTime untilTime)
{
  bool result = true;
  int i;
  ULaserDevice * dv;
  UReplayDevice * rd;
  //
  for (i = 0; i < lasDevsCnt; i++)
  { // allow all devices to advance
    dv = getDevice(i);
    if (dv->isReplayDevice())
    {
      rd = (UReplayDevice*) dv;
      result &= rd->replayToTime(untilTime);
    }
  }
  return result;
}

////////////////////////////////////////////

void ULaserPool::replayAdvanceTime(UTime untilTime)
{
  replayToTime(untilTime);
  UResBase::replayAdvanceTime(untilTime);
}

////////////////////////////////////////////

void ULaserPool::stop(bool andWait)
{
  int i;
  for (i = 0; i < lasDevsCnt; i++)
  { // stop all connections and stop any running threads
    if (lasDevs[i] != NULL)
      lasDevs[i]->stop(false);
  }
}

///////////////////////////////////////////////

ULaserData * ULaserPool::getScan(const char * device,
                                 ULaserDevice ** las,
                                 ULaserData * dataBuff,
                                 ULaserData * pushData,
                                 const int fake)
{
  int dev = -1;
  int n;
  ULaserDevice * scanner;
  //
  if (device != NULL)
  {
    if (strlen(device) > 0)
    {
      n = sscanf(device, "%d", &dev);
      if (n == 0)
      { // may be v360 scanner
        if (strcasecmp(device, "v360") == 0)
          dev = 360;
        else
        { // get device number
          scanner = getDevice(device);
          if (scanner != NULL)
            dev = getDeviceNumber(scanner);
        }
      }
    }
  }
  return getScan(dev, las, dataBuff, pushData, fake);
}
