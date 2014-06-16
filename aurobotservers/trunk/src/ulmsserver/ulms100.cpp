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

#include <termios.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>

#include "ulms100.h"

ULms100::ULms100()
 : ULaserDevice()
{
  dataCnt = 0;
  dataBuf[0] = '\0';
  angleResolution = 0.5;
  modeAngleScan = 240;
  modeSimulated = false;
  setDeviceName("lms100");
  strncpy(name, "unknown", MAX_NAME_LNG);
  lasData = NULL;
  laslog.openLog("lms100");
  strncpy(name, "lms100", MAX_NAME_LNG);
  maxValidRange = 15.096;
  finishedScan = false;
}

/////////////////////////////////////

ULms100::~ULms100()
{
  stop(false);
  if (laslog.isOpen())
  {
    finishedScan = false;
    laslog.closeLog();
  }
  if (isConnected())
    closePort();
  //printf("Laser lms100 destructor\n");
}

/////////////////////////////////////

bool ULms100::openPort()
{
  bool result;
  int lms100Port;
  const int MHL = 100;
  char h[MHL];
  char * p1;
  //
  strncpy(h, devName, MHL);
  p1 = strchr(h, ':');
  if (p1 != NULL)
  {
    *p1 = '\0';
    p1++;
    lms100Port = strtol(p1, NULL, 0);
  }
  else
    //use the default port number
    lms100Port = 2111;
  setHost(h);
  setPort(lms100Port);
  printf("open_port: Trying to open port %s\n", devName);
  result = tryConnect();
  if (result)
  {
    printf("open_port:: Sucessfully opened %s\n", devName);
    toLog("Open port", 9, "**");
    // make sure device is ready to handle data
    Wait(0.1);
    // request scandata
    finishedScan = true;
  }
  return result;
}

//////////////////////////////////////////////////////////////////////////

const char * ULms100::getNameFromDevice()
{
  // device has no name, so return the default 'noname'
  return getName();
}

//////////////////////////////////////////////////////////////////////////

void ULms100::closePort()
{
  lock();
  closeConnection();
  unlock();
  toLog("Close port", 9, "**");
}

////////////////////////////////////////////////////////////////////////

bool ULms100::isPortOpen()
{
  return (isConnected());
}

///////////////////////////////////////////////////

bool ULms100::getNewestData(ULaserData * dest,
                            unsigned long lastSerial,
                            int fake)
{
  bool result = false;
//  UTime t;
  //
  result = (dest != NULL);
  if (result)
  {
    if (fake > 0)
      getFakeScan( dest, lastSerial, fake);
    else
    { // live data
      if (not isPortOpen())
      { // open port (in get scan mode)
        // and start read thread
        start();
      }
      if (isPortOpen())
      { // get a full scan of data
//        t = dataRxTime;
      }
      result = (lasData != NULL) and (dest != NULL);
      if (result)
      {
        lasData->lock();
        //printf("Last serial = %lu, got data fromserial %lu, valid=%s\n",
        //       lastSerial, lasData->getSerial(), bool2str(lasData->isValid()));
        result = lasData->isValid() and // and ((lasData->getScanTime() - t) > 0.001);
                (lasData->getSerial() > lastSerial);
        if (result)
        {
          dest->copy(lasData);
          // save to logfile (if open)
          if (datalogUsedScans)
            logThisScan(lasData);
        }
        else
          dest->setValid(false);
        lasData->unlock();
      }
      else if (dest != NULL)
        dest->setValid(false);
    }
    if (result)
      dest->setDeviceNum(deviceNum);
  }
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool ULms100::getDataTo(char * msg, ULaserData * dest)
{
  bool result = (dest != NULL);
  int i, j;
  int d0, d1, e, n, m;
  int status;
  int * r = NULL;
  int * f = NULL;
  char * p1;
  //int time;
  int sa; // start angle (in deg/10000)
  int da; // angular step width
  int dataType;
  const int MSL = 5;
  char s[MSL];
  //
  if (result)
  {
    p1 = &msg[15];
    d0 = strtol(p1, &p1, 16); // version
    //printf("lms100: vesion: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // device ID
    //printf("lms100: device ID: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // factory serial number
    //printf("lms100: serial number: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // status of sensor (first byte)
    status = strtol(p1, &p1, 16); // status of sensor
    //printf("lms100: status: %d%d\n", d0, status);
    if (status > 0)
    {
      switch (status)
      {
        case 2:
          printf("lms100 device (serial %d) reports dirt on sensor (warning)\n", d0);
          break;
        case 3:
          printf("lms100 device (serial %d) reports dirt on sensor (error)\n", d0);
          break;
        default:
          printf("lms100 device (serial %d) reports device error\n", d0);
          break;
      }
    }
    d0 = strtol(p1, &p1, 16); // message counter
    //printf("lms100: message: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // scan counter
    //printf("lms100: scan: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // time since power in us
    //printf("lms100: time since power up: %d us\n", d0);
    strtol(p1, &p1, 16); // time since start scanning (us)
    //printf("lms100: time since start scan: %d us\n", time);
    d0 = strtol(p1, &p1, 16); // input status LSB
    d1 = strtol(p1, &p1, 16); // input status (3=all on)
    //printf("lms100: input status: %d%d (3=all)\n", d1, d0);
    d0 = strtol(p1, &p1, 16); // output status LSB
    d1 = strtol(p1, &p1, 16); // output status (7=all on)
    //printf("lms100: output status: %d%d (7=all)\n", d1, d0);
    d0 = strtol(p1, &p1, 16); // reserved
    //printf("lms100: reserved: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // scan rate in Hz/100 (2500 (=25Hz) .. 5000)
    //printf("lms100: scan rate: %d\n", d0);
    d0 = strtol(p1, &p1, 16); // measurement frequency an 100Hz
    //printf("lms100: measurement frequency: %d 00 Hz\n", d0);
    e = strtol(p1, &p1, 16); // encoders
    //printf("lms100: number of encoders: %d\n", e);
    result = (e < 4);
  }
  if (result)
  {
    for (i = 0; i < e; i++)
    {
      d0 = strtol(p1, &p1, 16); // encoder position (in tick)
      d0 = strtol(p1, &p1, 16); // encoder speed (tick/mm)
    }
    m = strtol(p1, &p1, 16); // measurement channels
    //printf("lms100: number of measurement sets: %d\n", m);
    result = (m < 5);
  }
  if (result)
  { // skip space before type
    p1++;
    for (i = 0; i < m; i++)
    {
      if (strncmp(p1, "DIST1", 4) == 0)
        dataType = 0;
      else if (strncmp(p1, "RSSI1", 5) == 0)
        dataType = 1;
      else if (strncmp(p1, "DIST2", 5) == 0)
        dataType = 2;
      else if (strncmp(p1, "RSSI2", 5) == 0)
        dataType = 3;
      else
        dataType = 4;
      if (dataType > 1)
        // discard all but distance measurement
        break;
      // printf("lms100: Data type is: %d (0=range, 1=intens)\n", dataType);
      //
      // scaling and angles
      p1 = &p1[5];
      d0 = strtol(p1, &p1, 16); // scaling factor
//      printf("lms100: scaling factor (real): %d\n", d0);
      d0 = strtol(p1, &p1, 16); // scaling offset
//      printf("lms100: scaling offset (real): %d\n", d0);
      // get signed start angle - right is 0, positive is CCV
      strncpy(s, ++p1, MSL);
      s[4] = '\0';
      d1 = strtol(s, NULL, 16); // most significant 16 bit
      p1 += 4;
      d0 = strtol(p1, &p1, 16); // least significant 16 bit
      sa = d1 << 16 | d0; // start angle in deg/10000 (-55 .. +125)
      //printf("lms100: start angle: %g deg (sa=%x d0=%x d1=%x)\n",
      //       sa / 10000.0,  sa, d0, d1);
      da = strtol(p1, &p1, 16); // angle step width in deg/10000
      //printf("lms100: angle step: %g deg\n", da / 10000.0);
      if (dataType == 0)
      { // distance measurements
        double startAng = sa / 10000.0 - 90.0;
        double angleRes = da / 10000.0;
        int skip = roundi((fabs(startAng) - modeAngleScan/2.0)/angleRes);
        int cnt = roundi(modeAngleScan/angleRes);
        r = dest->getRange(0);
        n = strtol(p1, &p1, 16); // number of measurements
        if (skip < 0 or cnt > n)
        {
          skip = 0;
          cnt = n;
          dest->setAngleResAndStart(startAng, angleRes);
        }
        else
          dest->setAngleResAndStart(-modeAngleScan/2.0, angleRes);
        //printf("lms100: range measurement count: %d\n", n);
        dest->setRangeCnt(cnt);
        for (j = 0; j < skip + cnt; j++)
        {
          *r = strtol(p1, &p1, 16); // range in mm
          //printf("lms100: range %d: %d mm\n", j, *r);
          if (j >= skip)
           r++;
        }
      }
      else if (dataType == 1)
      {
        f = dest->getFlags(0);
        n = strtol(p1, &p1, 16); // number of measurements
        printf("lms100: intens measurement count: %d\n", n);
        for (j = 0; j < n; j++)
        {
          *f = strtol(p1, &p1, 16); //RSSI intensity in digits
          printf("lms100: intensity %d: %d\n", j, *f);
          f++;
        }
      }
    }
    // rest of meassage is discarded.
    //dest->setScanTime();
  }
  // printf("lms100: measurement end %d bytes used\n", p1 - msg);
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////


bool ULms100::receiveData()
{
  bool gotData = false;
  int n;
  char * dend = NULL;
  char * dstart = NULL;
  const char * getScandata = "sRN LMDscandata";
  //
  if (modeSimulated)
  {
    //gotData = receiveSimulatedData(&length);
    Wait(0.1);
  }
  else
  {
    if (dataCnt > 0)
    { // test for old data
      if (dataTime.getTimePassed() > 1.0)
      {
        printf("Discarded '%s'\n", dataBuf);
        for (n = 0; n < dataCnt; n++)
          printf("%02x,", dataBuf[n]);
        printf("\n");
        printf("Discarded %d bytes of old data\n", dataCnt);
        statBadCnt++;
        laslog.toLog("discarded read data #:", statBadCnt, dataCnt, "bytes");
        dataCnt = 0;
        finishedScan = true;
      }
    }
    lock();
    if (isPortOpen())
    {
      // set timeout to ~30 ms
      n = getDataFromLine(&dataBuf[dataCnt], MAX_LMS100_LNG - dataCnt - 1, 20);
      if (dataCnt > 0)
        dstart = strchr(dataBuf, STX);
      if (dstart == NULL)
        dataRxTime.now();
      if (n == 0 and dataTime.getTimePassed() > 0.5)
      { // no data - for some reason
        finishedScan = true;
        laslog.toLog("no data in 0.5 sec", " re-request scan");
      }
    }
    else
      n = 0;
    //
    if (finishedScan)
    { // request new scan
      finishedScan = false;
      // ask for new data
      sendToDevice(getScandata, -1);
    }
    // finished using socket
    unlock();
    if (n == 0)
      Wait(0.01);
    else
    {
      dataCnt += n;
      dataBuf[dataCnt] = '\0';
      dstart = strchr(dataBuf, STX);
    }
  }
  //
  while (dstart != NULL)
  { // look for start of next message
    dstart++;
    // returnes first unused character
    dend = decodeData(dstart, dataRxTime);
    // any used data
    gotData |= dend != dstart;
    // look for new start of next message
    dstart = strchr(dend, STX);
    if (dstart > dend)
    { // STX found
      n = dataCnt - (dstart - dataBuf);
      // move the rest forward, and also terminating '\0'
      memmove(dataBuf, dstart,  n + 1);
      // reduce available data count
      dataCnt = n;
      // test for more data in buffer
      dstart = strchr(dataBuf, STX);
      if (dstart != NULL)
      { // debug
        printf("****Got more messages in one read (its OK, but indicate latency)\n");
        laslog.toLog("got more messages in one read,", " OK, but indicate latency");
        // debug end
      }
    }
    else if (dend > dstart)
    { // data used, and no new message start
      // remove all data
      dataCnt = 0;
    }
  }
  return gotData;
}

///////////////////////////////////////

char * ULms100::decodeData(char * msg, UTime rxTime)
{
  bool result = true;
  const char * aScan = "sEN LMDscandata";
  bool gotScan = false;
  const char * aStartMeas = "sAN LMCstartmeas";
  bool gotStartMeas = false;
  const char * aStatus = "sRA STlms";
  bool gotStatus = false;
  char * p1 = msg;
  char * pEnd;
  const int MSL = 30;
  char s[MSL+1];
  // ensure buffer is available
  dataTime.Now();
  if (lasData == NULL)
    lasData = new ULaserData();
  //
  result = (lasData != NULL);
  if (result)
  { // test received format
    // test also next 3 characters
    if (strncmp(p1, aScan, strlen(aScan)))
      gotScan = true;
    else if (strncmp(p1, aStartMeas, strlen(aStartMeas)))
      gotStartMeas = true;
    else if (strncmp(p1, aStatus, strlen(aStatus)))
      gotStatus = true;
    else
    { // unknown message - print
      strncpy(s, p1, MSL);
      s[MSL] = '\0';
      // debug print unknown messages
      printf("ULms100::decodeData: unknown msg: %s\n", s);
      laslog.toLog("ULms100::decodeData: unknown msg:", s);
      // debug end
      // mark message as used.
      result = true;
    }
  }
  pEnd = strchr(p1, ETX);
  result = pEnd != NULL;
  if (result)
  {
    if (gotScan)
    { // save data into decoded buffer
      lasData->lock();
      lasData->setDeviceNum(deviceNum);
      lasData->setMaxValidRange(maxValidRange);
      // copy data to this scan structure buffer
      result = getDataTo(p1, lasData);
      if (result)
      { // 0 = cm, 1=mm, 2 = 10cm
        lasData->setUnit(1);
        // subtract 1/2 scantime (50 scans per sec) plus
        // a bit for communication
        lasData->setScanTime(dataRxTime - var.scanDelay->getValued());
        lasData->setSerial(serial++);
        lasData->setValid(true);
        statGoodCnt++;
      }
      lasData->setMirror(mirrorData);
      lasData->unlock();
      //
      if (result)
        // do pending push commands.
        // No need to set data as locked, as
        // a client scanGet may use data at the same time as a scanpush,
        // as none of these will modify the 'lasData' structure.
        gotNewScan(lasData);
      finishedScan = true;
    }
    else if (gotStatus)
    {
      printf("%s\n", p1);
      result = true;
    }
    else if (gotStartMeas)
    {
      printf("%s\n", p1);
      result = true;
    }
  }
  else
    // error, not a full message - wait for more.
    pEnd = msg;
  return pEnd;
}

///////////////////////////////////////

bool ULms100::decodeName(char * msg)
{
  bool result = true;
  strncpy(name, msg, MAX_DEVICE_NAME_LNG - 1);
  return result;
}

///////////////////////////////////////

bool ULms100::changeMode(int scanangle, double resolution)
{
  //
  if (resolution < 0.4)
    angleResolution = 0.25;
  else
    angleResolution = 0.5;
  modeAngleScan =  mini(270,scanangle);
  printf("New width=%d res=%g\n", modeAngleScan, angleResolution);
  return true;
}

////////////////////////////////////////////////////

bool ULms100::sendToDevice(const char * msg, int lng)
{
  bool result;
  const int MSL = 100;
  char s[MSL];
  char * p1;
  //
  if (lng < 0)
    lng = strlen(msg);
  result = lng < (MSL - 4);
  if (result and lng > 0)
  {
    s[0] = STX;
    strncpy(&s[1], msg, lng);
    s[lng+1] = '\0';
    // remove trailing new-line
    p1 = strrchr(s, '\n');
    if (p1 == NULL)
      p1 = &s[lng+1];
    // append ETX (and a newline)
    *p1++ = ETX;
    *p1++ = '\n';
    *p1 = '\0';
    result = blockSend(s, p1 - s);
    if (result)
      toLog(msg, lng, "to laser ");
    else
      printf("Write failed to device %s\n", name);
  }
  return result;
}

////////////////////////////////////////////////////

void ULms100::toLog(const char * data, int length, const char * pre)
{
  const int MSL = MAX_LMS100_LNG + 1;
  char sd[MSL];
  int i;
  const char * cp = data;
  char * dp = sd;
//  const int MTL = 30;
//  char s[MTL];
//  UTime tim;
  //
  if (laslog.isOpen())
  {
    for (i = 0; i < length; i++)
    {
      if (*cp < ' ')
      {
        *dp++ = '<';
        cp++;
      }
      else
        *dp++ = *cp++;
    }
    *dp = '\0';
    laslog.toLog(pre, sd);
/*    tim.Now();
    tim.getTimeAsString(s, true);
    fprintf(laslog.getF(), "%s%s(%2d)%s\n", &s[3], pre, length, sd);*/
  }
}


