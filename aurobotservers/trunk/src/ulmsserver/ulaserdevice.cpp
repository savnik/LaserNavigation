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

#include <pthread.h>
#include <urob4/uvarpool.h>

#include "ulaserdevice.h"

ULaserDevice::ULaserDevice()
{
  verbose = false;
  threadStop = true;
  threadRunning = false;
  datalogInterval = 0;
  datalogUsedScans = false;
  datalogSeq = 0;
  angleResolution = 1.0;
  modeAngleScan = 180;
  modeSimulated = false;
  statGoodCnt = 0;
  statBadCnt = 0;
  statMsgRate = 0.0;
  // set default device
  setDeviceName("/dev/ttyS4");
  //setDeviceName("/dev/ttyS0");
  sendNewData = false;
  sendStrCnt = 0;
  loopCnt = 0;
//  repeatGetScan = true; //false;
  serial = 0;
  mirrorData = false;
//   fakeState = 0;
//   fakePose.clear();
  maxValidRange = 80.0;
  pushData = NULL;
  vars = NULL;
  var.name = NULL;
  var.type = NULL;
  var.serial = NULL;
  var.scanwidth = NULL;
  var.scanres = NULL;
  var.maxRange = NULL;
  var.scanDelay = NULL;
}

///////////////////////////////////////////////////

ULaserDevice::~ULaserDevice()
{
  logFileClose();
  if (pushData != NULL)
    delete pushData;
}

///////////////////////////////////////////////////

void * threadRunSick(void * obj)
{ // call the hadling function in provided object
  ULaserDevice * ce = (ULaserDevice *)obj;
  ce->threadRunLoop();
  pthread_exit((void*)NULL);
  return NULL;
}

///////////////////////////////////////////////////

bool ULaserDevice::start()
{
  bool result = true;
  pthread_attr_t  thAttr;
  //
  if ((not modeSimulated) and (not isPortOpen()))
  { // open port
    result = openPort();
  }
  //
  if ((isPortOpen() or modeSimulated) and not threadRunning)
  {
    pthread_attr_init(&thAttr);
    //
    threadStop = false;
    // create socket server thread
    result = (pthread_create(&threadHandle, &thAttr,
                  &threadRunSick, (void *)this) == 0);
    pthread_attr_destroy(&thAttr);
  }

  return result;
}

///////////////////////////////////////////////////

void ULaserDevice::stop(bool justClosePort)
{
  if (isPortOpen())
  { // stop contiious mode and close serial port
    closePort();
  }
  if (threadRunning and not justClosePort)
  { // stop and join thread
    threadStop = true;
    pthread_join(threadHandle, NULL);
  }
  // close log too (if open)
  datalog.closeLog();
}

///////////////////////////////////////////////////

void ULaserDevice::threadRunLoop()
{
  int STAT_INTERVAL = 1000;
  //struct timeval t1, t2;
  double dt;
  bool gotData;
  UTime t;
  //
  threadRunning = true;
  loopCnt = 0;
  t.Now();
  while (not threadStop)
  {
    // send new data to device
    if (sendNewData)
    { // debug
      printf("ULaserDevice::threadRunLoop sending to device '%s'\n", sendStr);
      // debug end
      sendToDevice(sendStr, sendStrCnt);
      sendNewData = false;
    }
    // receive data from device
    // may be measurements or not
    gotData = receiveData();
    // do other things as appropriate
    if (not isPortOpen())
      Wait(0.1);
    if (not gotData)
      Wait(0.001); // changed to 1 ms
    if(int(statGoodCnt + statBadCnt) == STAT_INTERVAL )
    {
      dt = t.getTimePassed();
      t.Now();
      if (verbose)
      {
        printf("Got %d Good and %d Bad packets (total %d) %3.1f%% good\n",
                statGoodCnt, statBadCnt, statGoodCnt + statBadCnt,
                (float)(statGoodCnt) /
                (float)(statGoodCnt + statBadCnt) * 100.0);
        printf("in %7.3f seconds or %6.2f scan/sec\n",
                dt, (float)(statGoodCnt)/dt);
      }
      //
      statMsgRate = double(statGoodCnt)/dt;
      statGoodCnt = 0;
      statBadCnt = 0;
    }
    loopCnt++;
  }
  threadRunning = false;
}

//////////////////////////////////////////////////

void ULaserDevice::print(char * preString)
{
  const int MSL = 1000;
  char s[MSL];
  //
  print(preString, s, MSL);
  printf("%s", s);
}

///////////////////////////////////////////////////

const char * ULaserDevice::print(const char * preString, char * buff, int buffCnt)
{
  snprintf(buff, buffCnt, "%s %s on %s open=%s w=%ddeg, "
           "res=%4.2fdeg, rate=%.1f/s scan=%lu\n", preString,
           getName(), devName, bool2str(isPortOpen()),
           modeAngleScan, angleResolution,
           getMsgRate(), getSerial());
  return buff;
}

///////////////////////////////////////////////////

bool ULaserDevice::openPort()
{
  printf("virtual ULaserDevice::openPort: No real device to open\n");
  return false;
}
///////////////////////////////////////////////////

bool ULaserDevice::receiveData()
{
  printf("virtual ULaserDevice::receiveData: No real device\n");
  return false;
}

///////////////////////////////////////////////////

void ULaserDevice::closePort()
{
  printf("virtual ULaserDevice::closePort: No real device to close\n");
}

///////////////////////////////////////////////////

bool ULaserDevice::isPortOpen()
{
  printf("virtual ULaserDevice::isPortOpen: No real device\n");
  return false;
}

///////////////////////////////////////////////////

bool ULaserDevice::changeMode(int scanangle, double resolution)
{
  printf("virtual ULaserDevice::changeMode: no device\n");
  return false;
}

///////////////////////////////////////////////////

bool ULaserDevice::getNewestData(ULaserData * dest,
                                 unsigned long lastSerial,
                                 int fake)
{
  printf("virtual ULaserDevice::getNewestData: no device\n");
  return false;
}

///////////////////////////////////////////////////

void ULaserDevice::send(char * msg)
{
  int v;
  char * s, *d;
  //
  //strncpy(sendStr, msg, MAX_SEND_MSG_LNG);
  s = msg;
  d = sendStr;
  while (*s != '\0')
  {
    if (*s == ':')
    {
      v = strtol(&s[1], &s, 0);
      if (*s == ':')
        s++;
      *d = char(v);
      d++;
    }
    else
      *d++ = *s++;
  }
  *d = '\0';
  sendStrCnt = (d - sendStr);
  sendNewData = true;
  if (not isPortOpen())
    start();
}

///////////////////////////////////////////////////

bool ULaserDevice::sendToDevice(const char * msg, int lng)
{
  printf("virtual ULaserDevice::sendToDevice: nor send\n");
  return 0;
}

///////////////////////////////////////////////////

const char * ULaserDevice::getNameFromDevice()
{
  printf("ULaserDevice::getNameFromDevice: no device\n");
  return getName();
}

///////////////////////////////////////////////////

void ULaserDevice::gotNewScan(ULaserData * gotData)
{ // got new data - check for push commands
  // data is available if needed by a getNewestData(ULaserData *) call
  ULaserData * data = gotData;
  bool isOK;
  //
  // Do we have a scan buffer
  if (pushData == NULL)
    // create empty buffer for scanpush commands
    pushData = new ULaserData();
  //
  if (getPushQueue()->getPushCmdCnt() > 0)
  { // push commands exist
    // is the buffer in use (by a scanpush command)
    if (pushData->tryLock())
    { // not in use, so refresh
      if (data == NULL)
      { // request data from native coded queue
        data = pushData;
        // unpack to local buffer
        isOK = getNewestData(data, 0, false);
      }
      else
      { // data is available as parameter, that is OK
        isOK = true;
        // copy to scanpush buffer
        pushData->copy(data);
      }
      if (isOK)
      { // data is available - ask cmdExe to
        // execute push command (when time allows)
        setUpdated("");
      }
      pushData->unlock();
    }
  }
  // check for log needs
  if (datalogInterval > 0)
  {
    if (datalogSeq == 0)
    {
      if (data == NULL)
      { // especially sick do not decode to ULaserData format
        // so do it now - use push-data buffer
        if (pushData->tryLock())
        { // scanbuffer is available get data
          // unpack to ULaserData format
          if (getNewestData(pushData, 0, false))
            logThisScan(pushData);
          pushData->unlock();
        }
      }
      else
        // no resource problem
        logThisScan(data);
    }
    datalogSeq = (datalogSeq + 1) % datalogInterval;
  }
  // update global variables
  if (data != NULL)
    updateScanData(data->getScanTime());
}

/////////////////////////////////

void ULaserDevice::callGotNewDataWithObject()
{
  if (pushData != NULL)
  {
    pushData->lock();
    // start the push handling
    gotNewData(pushData);
    // finished with the data structure, so release.
    pushData->unlock();
  }
}

/////////////////////////////////

bool ULaserDevice::logFileOpen()
{
  const int MSL = 100;
  char s[MSL];
  //
  snprintf(s, MSL, "laser_%d", deviceNum);
  datalog.setLogName(s);
  datalog.openLog();
  //
  return datalog.isOpen();
}

/////////////////////////////////

char * ULaserDevice::getLogFileName(char * buffer, int bufferCnt)
{
  snprintf(buffer, bufferCnt, "%s/laser_%d.log", dataPath, deviceNum);
  return buffer;
}

/////////////////////////////////

void ULaserDevice::logFileClose()
{
  datalog.closeLog();
  //printf("Closed laserscanner logfile %s\n", datalog.getLogFileName());
}

/////////////////////////////////

void ULaserDevice::logThisScan(ULaserData * scan)
{
  datalog.logLock();
  scan->saveToLogFile(datalog.getF());
  datalog.logUnlock();
}

/////////////////////////////////

void ULaserDevice::getFakeScan(ULaserData * dest,
                                unsigned long lastSerial,
                                int fake, double fakeDt)
{
  bool advancePose;
  UTime t;
  double da;
  UPosRot pr;
  UPoseTVQ fakePose;
  //
  t.now();
  advancePose = lastSerial >= serial;
  da = double(modeAngleScan/2);
  if (advancePose)
  { // get new scan - using fixed time increment of 0.2 sec
    fakePose = fakeMap.fakeAdvancePose(fakeDt);
    serial++;
    dest->setScanTime(fakePose.t);
  }
  else
    dest->setScanTime(fakeMap.currentTruePose.t);
  pr = getDevicePose();
  dest->setSimData(-da, da, angleResolution, 0.02, maxValidRange + 0.01, fakePose.getPose(), pr, fake);
  dest->setMirror( mirrorData);
  dest->setSerial(serial);
  dest->setMaxValidRange(maxValidRange);
  dest->setDeviceNum(deviceNum);
  // allow push commands to use faked data too (experimental)
  if (advancePose)
  {
    gotNewScan(dest);
    if (datalogUsedScans)
      logThisScan(dest);
    if (verbose)
      fakePose.print("FakePose");
  }
}

///////////////////////////////////

void ULaserDevice::createBaseVars()
{
  double d;
  if (vars != NULL)
  {
    var.maxRange = vars->addVar("maxRange", maxValidRange, "d", "(r) Max valid range for device");
    var.name = vars->addVarA("name", name, "s", "(r) device name");
    var.versionInfo = vars->addVarA("versionInfo", name, "s", "(r) version info from device (if available)");
    var.scanres = vars->addVar("scanRes", angleResolution, "d", "(r) resoulution in degrees between scans.");
    var.scanwidth = vars->addVar("scanWidth", modeAngleScan, "d", "(r) width of scan - in xy plane symmetric around x-axis");
    var.serial = vars->addVar("scannumber", serial, "d", "(r) next unused scan number");
    var.type = vars->addVarA("device", devName, "s", "(r) device source name");
    var.pose = vars->addVarA("pose", "0 0 0 0 0 0", "6d", "(r/w) pose (x,y,z,a,p,k) relative to robot. 2D pose is (x,y,k)");
    d = isPortOpen();
    var.isOpen = vars->addVar("isOpen", d, "d", "(r) is device open (delivers data)");
    var.framerate = vars->addVar("framerate", 1.0, "d", "(r) delivered framerate (Hz) - may not work for all devices");
    var.scanDelay = vars->addVar("scanDelay", 0.0, "d", "(r/w) Delay time from laser scan to timestamp (approx 1/(2 * framerate))");
  }
}

///////////////////////////////////

void ULaserDevice::updateScanData(UTime scanTime)
{
  double f;
  if (var.framerate != NULL)
  {
    f = 1 / (scanTime - lastScanTime);
    var.framerate->setDouble(f, 0);
    var.serial->setInt(serial, 0);
    var.scanres->setDouble(angleResolution, 0);
    var.scanwidth->setInt(modeAngleScan, 0);
    var.maxRange->setDouble(maxValidRange, 0);
    lastScanTime = scanTime;
  }
}

/////////////////////////////////////

void ULaserDevice::setDevicePose(UPosRot * newPose)
{
  if (var.pose != NULL)
    var.pose->set6D(newPose);
};

///////////////////////////////////

UPosition ULaserDevice::getDevicePos()
{
  UPosition pos;
  if (var.pose != NULL)
    pos = var.pose->get3D();
  return pos;
}

///////////////////////////////////

URotation ULaserDevice::getDeviceRot()
{
  UPosRot pr;
  if (var.pose != NULL)
    pr = var.pose->get6D();
  return *pr.getRot();
}

////////////////////////////////////

UPosRot ULaserDevice::getDevicePose()
{
  UPosRot pr;
  if (var.pose != NULL)
    pr = var.pose->get6D();
  return pr;
}


