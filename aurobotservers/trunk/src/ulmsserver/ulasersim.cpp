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

#include "ulasersim.h"

ULaserSim::ULaserSim()
  : ULaserDevice(), UClientHandler()
{
  namespaceUse = false;
  repeatGetScan = true;
  dataTime.Now();
  newData = false;
  scan = new ULaserData();
  scanf = new UClientFuncSimScan();
  rxSerial = 0;
  if ((scan != NULL)  and (scanf != NULL))
  { // add the data extraction function to this connection
    scanf->setLaserData(scan);
    addFunction(scanf, false);
  }
  strncpy(name, "sim", MAX_NAME_LNG);
  maxValidRange = 8.05;
  core = NULL;
}


ULaserSim::~ULaserSim()
{
  if (scanf != NULL)
    delete scanf;
  if (scan != NULL)
    delete scan;
}

//////////////////////////////////////

bool ULaserSim::openPort()
{
  bool result;
  const char *p1;
  const int MNL = 50;
  char h[MNL]; // host name
  int p;
  int n;
  //
  if (isConnected())
    closeConnection();
  p1 = strchr(devName, ':');
  // there must be a : in the host:port device name
  result = (p1 != NULL);
  if (result)
  { // get host name and port number from device name
    // format: host:port, e.g.: nyquist:19000
    n = mini(MNL-1, p1 - devName);
    strncpy(h, devName, n);
    h[n] = '\0';
    p1++;
    p = strtol(p1, NULL, 0);
    result = (p > 1000);
  }
  if (result)
  {
    setHost(h);
    setPort(p);
    tryHoldConnection = true;
    result = openConnection();
  }
  if (result and not isThreadRunning())
    result = UClientHandler::start();
  return result;
}

//////////////////////////////////////

void ULaserSim::closePort()
{
  tryHoldConnection = false;
  if (isConnected())
    closeConnection();
}

//////////////////////////////////////

bool ULaserSim::sendToDevice(const char * msg, int lng)
{
  const int MCL = 2000;
  char s[MCL];
  int n;
  bool isOK = false;
  //
  if (msg[lng-2] < ' ')
    isOK = blockSend(msg, lng);
  else
  {
    n = mini(MCL, lng);
    snprintf(s, n + 1, "%s", msg);
    while ((n > 0) and (msg[n-1] < ' '))
      n--;
    s[n++] = '\r';
    s[n++] = '\n';
    s[n] = '\0';
    isOK = blockSend(s, n);
  }
  return isOK;
}

//////////////////////////////////////

const char * ULaserSim::getNameFromDevice()
{
  return "SIM";
}

//////////////////////////////////////

bool ULaserSim::getNewestData(ULaserData * dest,
                             unsigned long lastSerial,
                             int fake)
{
  bool result = (dest != NULL);
  int to = 700; // timeout cycles
  // request new data
  // wait for data
  // return data
  if (fake > 0)
  {
    if (result)
    {
      if (fake == 2)
        dest->setMaxValidRange(4.1);
      else
        dest->setMaxValidRange(8.1);
      getFakeScan(dest, lastSerial, fake);
    }
  }
  else
  {
    if (not isConnected())
    { // connect to simulation port
      ULaserDevice::start();
    }
    if (result and (scan != NULL) and not repeatGetScan)
    {
      if (scan->getSerial() <= lastSerial)
      { // request new data
        scan->setSerial(lastSerial);
        result = sendMsg("scanget\r\n");
      }
    }
    while (result and (scan->getSerial() <= lastSerial))
    { // wait for data
      Wait(0.01);
      to--;
      if (to == 0)
        result = false;
    }
    if (result)
    { // wait for lock, if all data has not arrived
      scan->lock();
      dest->copy(scan);
      dest->setMirror(mirrorData);
        // save to logfile (if open)
      if (datalogUsedScans)
        logThisScan(dest);
      scan->unlock();
    }
  }
  if (result)
    dest->setDeviceNum( deviceNum);
  return result;
}

////////////////////////////////////////

bool ULaserSim::receiveData()
{
  bool gotData = false;
  const char * getscan = "scanget";
//  UResPoseHist * odoPose;
//  UPoseTime pt;
//  double vel;
  //
  if (modeSimulated)
  {
  //gotData = receiveSimulatedData(&length);
    Wait(0.1);
  }
  else if (isConnected())
  { // expecting data
    if (scan->getSerial() != rxSerial)
    {
      rxSerial = scan->getSerial();
      scan->setDeviceNum(deviceNum);
      scan->setValid(true);
      scan->setMirror(mirrorData);
      scan->setMaxValidRange(maxValidRange);
      // debug - tell odopose about sim-time
/*      if (core != NULL)
      { // trying to fis simulator problem
        odoPose = (UResPoseHist*)core->getStaticResource("odoPose", false, true);
        pt = odopose->getNewest(&vel);
        if ((scan->getScantime() - pt.t) > 0.1)
        {
          
        }
      }*/
      // debug end
      gotNewScan(scan);
      Wait(0.005);
      newData = true;
      gotData = true;
      dataTime.Now();
    }
    if (gotData or (dataTime.getTimePassed() > 5.0))
      sendToDevice(getscan, strlen(getscan));
  }
  return gotData;
}

///////////////////////////////////////////

bool ULaserSim::changeMode(int scanangle, double resolution)
{
  bool result = false;
  //
  if (true)
  { // simulator has no options - not supported
    angleResolution = resolution;
    modeAngleScan = scanangle;
    printf(" - device '%s' mode change not supported (%d deg, %g deg/meas.)\n", getDeviceName(), scanangle, resolution);
    result = true;
  }
  return result;
}

///////////////////////////////////////////////////

