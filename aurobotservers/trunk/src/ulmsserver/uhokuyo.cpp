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
#include <ugen4/ucommon.h>


#include "uhokuyo.h"

UHokuyo::UHokuyo()
 : ULaserDevice()
{
  dataCnt = 0;
  dataBuf[0] = '\0';
  hfd = -1;
  angleResolution = 360.0/1024.0;
  modeAngleScan = 240;
  modeSimulated = false;
  setDeviceName("/dev/ttyACM0");
  strncpy(name, "unknown", MAX_NAME_LNG);
  lasData = NULL;
  laslog.setLogName("urg");
  strncpy(name, "urg", MAX_NAME_LNG);
  maxValidRange = 4.096;
  badSeries = 0;
  versionInfoCnt = 0;
}

/////////////////////////////////////

UHokuyo::~UHokuyo()
{
  stop(false);
  laslog.closeLog();
}

/////////////////////////////////////

bool UHokuyo::openPort()
{
  bool result;
  UTime t;
/*  const int MSL = 30;
  char s[MSL];*/
  //
  if (hfd < 0)
  {
    /* fd = open(SERIAL_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY); */
    printf("open_port: Trying to open port %s\n", devName);
    // debug 
    //int err = setDeviceSpeed(devName, 115200);
    int err = setDeviceSpeed(devName, 9600);
    if (err != 0)
      printf(" - failed to ser speed on serial to USB device\n");
    //
    hfd = open(devName, O_RDWR | O_NOCTTY );
    if (hfd == -1)
    { /* Could not open the port. */
      perror("open_port:: Unable to open port\n");
    }
    else
    { /* fcntl(fd, F_SETFL, FNDELAY);   non blocking by using FNDELAY */
      //fcntl(hfd, F_SETFL, 0);      /* Blocking */
      printf("open_port:: Sucessfully opened %s\n", devName);
      //tcflush(hfd, TCIFLUSH); /* flush unread data */
      //
/*      Wait(0.1);
      // request name
      snprintf(s, MSL, "\n");
      sendToDevice(s);
      Wait(0.1);
      // request name
      snprintf(s, MSL, "V\n");
      sendToDevice(s);*/
      t.now();
      toLog("Open port", 9, "**", t);
    }
  }
  result = hfd >= 0;
  return result;
}

//////////////////////////////////////////////////////////////////////////

const char * UHokuyo::getNameFromDevice()
{
  int i;
  //
  if (isPortOpen())
  {
    closePort();
  }
  // open portand ensure that receive thread is running
  start();
  if (isPortOpen())
  { // wait for port to open ...
    lock();
    Wait(0.1);
    printf("Name request [V\\n]...\n");
    // request forst message - vill probably be lost
    sendToDevice("V\n", 2);
    printf("Waiting for reply ...\n");
    Wait(0.1);
    // request again
    sendToDevice("V\n", 2);
    // wait for reply
    i = 0;
    unlock();
    while (strlen(getName()) < 10)
    { // wait for reply
      Wait(0.1);
      i++;
      if (i > 20)
        break;
    }
    printf("Finished after <= %g secs\n", 0.1 * i);
    closePort();
  }
  return getName();
}

//////////////////////////////////////////////////////////////////////////

void UHokuyo::closePort()
{
  int err;
  UTime t;
  //
  if (hfd >= 0)
  {
    lock();
//    repeatGetScan = false;
    err = close(hfd);
    if (err != 0)
      perror("Serial laser port close error:");
    else
      printf("Port %s closed\n", getDeviceName());
    hfd = -1;
    t.now();
    toLog("Close port", 9, "**", t);
    unlock();
  }
}

////////////////////////////////////////////////////////////////////////

bool UHokuyo::isPortOpen()
{
  return (hfd >= 0);
}

///////////////////////////////////////////////////

bool UHokuyo::getNewestData(ULaserData * dest,
                            unsigned long lastSerial,
                            int fake)
{
  bool result = false;
  UTime t;
  bool isOK;
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
        if (isPortOpen())
        { // wait for port to open
          printf("Opening port for scan ...\n");
          Wait(0.1);
          lock();
          sendToDevice("\nV\n", 3);
          //isOK = sendToDevice("\nG04572501\n", 11);
          unlock();
          //printf("Waiting for reply 1 to [G38438501\\n] ...\n");
          Wait(0.1);
          lock();
          isOK = sendToDevice("\nG04572501\n", 11);
          unlock();
          //printf("Waiting for reply 2 to [G38438501\\n] ...\n");
          Wait(0.3);
          if (isOK)
            printf("Opening port for scan ... is OK\n");
          badSeries = 0;
          versionInfoCnt = 0;
        }
      }
      if (isPortOpen())
      { // get a full scan of data
        t = dataRxTime;
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
      dest->setDeviceNum( deviceNum);
  }
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UHokuyo::getDataTo(char * msg, const int msgCnt, ULaserData * dest)
{
  bool result = (dest != NULL);
  int i, j;
  int val;
  int block;
  int d0, d1, d2;
  int status;
  char * sd = msg;
  int * r = NULL;
  int * f = NULL;
  const int MSL = 200;
  char s[MSL];
  double resA, firstA;
  bool char2 = true; // 12 bit coding in 2 chars (else 18bit coding in 3 chars)
  int valueCnt = 0;
  //
  result = sd[0] == 'G';
  if (not result)
    printf("Failed: first data byte is not 'G'\n");
  if (result)
  {
    sd++;
    strncpy(s, sd, 8);
    s[8] = '\0';
    d2 = strtol(&s[6], NULL, 10); // interval 1 = 0.35 deg
    s[6] = '\0';
    d1 = strtol(&s[3], NULL, 10); // end index
    s[3] = '\0';
    d0 = strtol(s, NULL, 10); // start index
    valueCnt = d1 - d0 + 1;
    result = (valueCnt > 12); // more that 12 values send
    // the message holds the msg echo (12 chars) a newline every 64 characters and the value bytes (2 or 3 chars)
    int expected3MsgCnt = valueCnt * 3 + 12 + ((valueCnt * 3)/64);
    char2 = msgCnt < expected3MsgCnt;
    // debug
/*    printf("UHokuyo::getDataTo: 3char length expects %d bytes, got %d", expected3MsgCnt, msgCnt);
    if (char2)
      printf(" is 2 char encoding\n");
    else
      printf(" is 3 char encoding\n");*/
    //strncpy(s, sd, 8);
    //s[8] = '\0';
    //printf("Got G start %d end %d interval %d from:'%s'\n",
    //       d0, d1,d2, s);
    // debug end
    sd += 9;
    // debug
    if (not result and verbose)
    {
      printf("UHokuyo::getDataTo - Failed on too few data\n");
    }
    // debug end
  }
  if (result)
  {
    resA = double(d2) * 360.0 / 1024.0;
    //firstA = (d1 - d0)* resA / 2.0;
    // First angle in degrees
    firstA = d0 * 360.0 / 1024.0 - 135.0;
    // printf("Got data from measurement %d to %d at %d interval -- first %g res %g\n", d0, d1, d2, -firstA, resA);
    // set in structure
    dest->setAngleResAndStart(firstA, resA);
  }
  if (result)
  { // get status
    strncpy(s, sd, 10);
    s[10] = '\0';
    status = strtol(sd, &sd, 0);
    // debug
    //  printf("Got status %d from '%s'\n", status, s);
    // debug end
    result = (status == 0);
    r = dest->getRange(0);
    f = dest->getFlags(0);
    // debug
    if (not result and verbose)
    {
      printf("UHokuyo::getDataTo - Failed status byte is not 0\n");
    }
    // debug end
  }
  i = 0;
  block = 0;
  j = 0;
  val = 0;
  int cCnt = 0;
  while (*sd < ' ' and *sd > '\0')
    sd++;
  while (result)
  { // get data values
    if ((*sd <= ' ') or j >= 64)
    {
      if (*sd > ' ')
      { // this happens on the used interface - it seems like
        // 64 bytes were lost making one buffer 1 byte too long
        result = false;
        // debug
        if (not result and verbose)
        {
          printf("UHokuyo::getDataTo - reply too long (j >= 64)\n");
        }
        // debug end
        break;
      }
      while (*sd > ' ')
        // skip surplus linefeed characters
        sd++;
      sd++;
      block++;
      if (*sd <= ' ')
        // end of data
        break;
      j = 0;
    }
    // add char to value
    val = (val << 6) + *sd - 0x30;
    // advance
    cCnt++;
    j++;
    sd++;
    if ((cCnt % 2 == 0 and char2) or (cCnt % 3 == 0 and not char2))
    { // a full value is assembled
      if (val < 20)
        // a flag value
        *f++ = val;
      else
        // no error
        *f++ = 0;
      // save distance value
      *r++ = val;
      // increase value count
      val = 0;
      i++;
      if (i > MAX_RANGE_VALUES)
      {
        result = false;
        // debug
        if (not result and verbose)
        {
          printf("UHokuyo::getDataTo - too many range values\n");
        }
        // debug end
        break;
      }
    }
  }
  if (result)
  {
    dataRxTime.getTimeAsString(s, true);
    //printf("Got %lu: (%d blocks) %d values at %s\n", serial, block, i, s);
  }
  else if (verbose)
  {
    snprintf(s, MSL, "UHokuyo::getDataTo: Decode data failed (after %d blocks and %d values)\n", block, i);
    printf("%s\n", s);
    laslog.toLog("err:", s);
  }
  if (result)
    dest->setRangeCnt(i);
  if (result and i != valueCnt)
  {
    result = false;
    snprintf(s, MSL, "UHokuyo::getDataTo: too few/many values, expected %d got %d. in %d blocks (each 64) last block had j=%d chars\n",
             valueCnt, i, block, j);
    printf("%s\n", s);
    laslog.toLog("err:", s);
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////////////////

bool UHokuyo::requestMoreData()
{
  const int MSL = 30;
  char s[MSL];
  // make data request string
  makeGetDataString(s, MSL);
  //printf("mode=%d, res=%g, sending:%s", modeAngleScan, angleResolution, s);
  // request data
  return sendToDevice(s, 11);
}

/////////////////////////////////////////////////////////////////////////

char * UHokuyo::makeGetDataString(char * cmdStr, const int cmdStrCnt)
{
  int clust;
  // full scan for device
  int first = 44; // 0   is -135 deg (44 approx -120 deg)
  int last = 725; // 768 is +135 deg (725 approx +120 deg)
  const double baseRes = 360.0/1024.0;
  const int MSL = 12; // minimum string buffer length (exclusive terminating zero)
  bool result;
  //
  result = (cmdStr != NULL) and (cmdStrCnt > MSL);
  //
  clust = roundi(angleResolution / baseRes);
  if (modeAngleScan <= 270)
  { // less than full scan requested - get symmetric values
    first = 384 - roundi(double(modeAngleScan) / baseRes / 2.0 - 1);
    last = 384 + roundi(double(modeAngleScan) / baseRes / 2.0);
  }
  if (result)
    snprintf(cmdStr, cmdStrCnt, "\nG%03d%03d%02d\n", first, last, clust);
  //printf("mode=%d, res=%g, sending:%s", modeAngleScan, angleResolution, s);
  return cmdStr;
}

/////////////////////////////////////////////////////////////////////////

bool UHokuyo::receiveData()
{
  bool gotData = false;
  int n, i = 0;
  char * dend = NULL;
  UTime t;
  bool gotRngData = false;
  const int MSL = 500;
  char s[MSL];
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
        dataCnt = 0;
      }
    }
    lock();
    if (isPortOpen())
    {
      // set timeout to ~30 ms
      n = receiveFromDevice(&dataBuf[dataCnt], MAX_DATA_LNG - dataCnt - 1, 0.015);
      if (/*repeatGetScan and*/ (n > 0))
      { // request more data
        requestMoreData();
      }
      //if (n > 0)
      //  printf("(%d bytes)", n);
    }
    else
      n = 0;
    unlock();
    if (n > 0)
    {
      dataCnt += n;
      dataBuf[dataCnt] = '\0';
      dend = strstr(dataBuf, "\n\n");
      // debug
/*      if (verbose and (datalog != NULL))
      { // log the received part of the buffer
       dataBuf[n] = '\0';
       fprintf(datalog, "%4d (n=%3d) got:'%s'\n", dataCnt, n, dataBuf);
      }*/
      // debug end
      gotData = (dend != NULL);
    }
  }
  //
  while (gotData)
  { // message length
    badSeries++;
    n = dend - dataBuf;
    // debug
    if (n > dataCnt)
    {
      printf("UHokuyo::receiveData message end found after buffer end? n=%d dataCnt=%d\n", n, dataCnt);
    }
    if (verbose)
    {
      snprintf(s, MSL, "In %d 'gotData' dataCnt=%d n=%d  %c%c%c%c%c%c%c%c%c... after %g sec\n", i, dataCnt, n,
            dataBuf[0], dataBuf[1], dataBuf[2],dataBuf[3], dataBuf[4], dataBuf[5], dataBuf[6], dataBuf[7], dataBuf[8],
            dataTime.getTimePassed());
      laslog.toLog("urg:", s);
    }
    //printf("msg:%s", dataBuf);
    i++;
    // debug end
    switch (dataBuf[0])
    {
    case 'V':
      dend[1] = '\0';
      if (dataBuf[1] == '\n')
      { // some garbage may also start with a V
        if (versionInfoCnt < 5 or verbose)
          printf("Got version (%d) info ((grp %d) %d chars):'%s'\n", versionInfoCnt, i, dataCnt, dataBuf);
        decodeName(dataBuf);
        versionInfoCnt++;
        if (verbose)
          laslog.toLog(" - V:", &dataBuf[4]);
      }
      break;
    case 'G':
      if (dataBuf[10] != '0')
      { // must be '0 to be valid scandata
        // printf("Bad data set laser may be off (bdSeries=%d) - send an on-command (\\nL1\\n)\n", badSeries);
        sendToDevice("\nL1\n", 4);
        break;
      }
      badSeries = 0;
      if (lasData == NULL)
        lasData = new ULaserData();
      if (lasData != NULL)
      { // save data into decoded buffer
        lasData->lock();
        lasData->setDeviceNum(deviceNum);
        gotRngData = decodeData(dataBuf, n, lasData);
        lasData->setMaxValidRange(maxValidRange);
        //if (repeatGetScan and not gotRngData)
        //{ // error message - request new data
        //  // send request for more data right away
        //  lock();
        //  //sendToDevice("00038401\n", 9);
        //  //sendToDevice("00076801\n", 9);
        //  unlock();
        //}
        if (gotRngData)
          statGoodCnt++;
        lasData->setMirror(mirrorData);
        lasData->unlock();
        //
        if (gotRngData)
          // do pending push commands.
          // No need to set data as locked, as
          // a client scanGet may use data at the same time as a scanpush,
          // as none of these will modify the 'lasData' structure.
          gotNewScan(lasData);
        if (verbose)
        { // debug logging
          snprintf(s, MSL, "UHokuyo::receiveData: scan=%6s %drngs In %2d msgCnt=%4d/%4d : %c%c%c%c%c%c%c%c%c %c %c%c... after %g sec\n",
            bool2str(gotRngData), lasData->getRangeCnt(),  i, n, dataCnt,
            dataBuf[0], dataBuf[1], dataBuf[2],dataBuf[3], dataBuf[4], dataBuf[5], dataBuf[6], dataBuf[7], dataBuf[8],
            dataBuf[10], dataBuf[12], dataBuf[13],
            dataTime.getTimePassed());
          laslog.toLog(" - G:", s);
          // to screen also
          printf("%s", s);
        }
        dataTime.Now();
      }
      break;
    default:
      // got error
      // discard to first newline
/*      char * nl = strchr(dataBuf, '\n');
      if (nl < dend and nl > dataBuf)
      { // newline found,
        // set new data end (dend) to one earlier, as if it was
        // a real "\n\n" command termination.
        // this is to avoid a situation where OK scans
        // are part of an error-respond.
        dend = --nl;
      }*/
      // close string and restart
      dend[1] = '\0';
      // debug
      if (verbose)
      {
        snprintf(s, MSL, "UHokuyo::receiveData: garbage msg %4d of %4d chars in buffer:'%c%c%c...'\n", n, dataCnt, dataBuf[0], dataBuf[1], dataBuf[2]);
        laslog.toLog(" - ", s);
      }
      //printf("UHokuyo::receiveData: ignored garbage (%d chars):'%s'\n", dataCnt, dataBuf);
      //rp = true; //repeatGetScan;
      // no other than G???... should be
      // received in repeat mode, well
      //    sometimes a LFLF is in the error message
      //repeatGetScan = false;
        /*closePort();
        printf("Got unknown garbage:--------Closing port\n");
        Wait(0.2);
        if (rp)
        { // repeat is enabled - request dummy data
          printf("-------------restarting in repeat mode\n");
          getNewestData(NULL, 0, false);
          //repeatGetScan = true;
        }*/
      statBadCnt++;
      break;
    }
    // discard the used (or unknown) data
    n = dataCnt - (dend - dataBuf) - 2;
    memmove(dataBuf, dend + 2,  n);
    // reduce available data count
    dataCnt = n;
    dataBuf[dataCnt] = '\0';
    // test for more data in buffer
    gotData = dataCnt > 3;
    if (gotData)
    { // is it a full message
      dend = strstr(dataBuf, "\n\n");
      gotData = (dend != NULL);
      //printf("****Got more messages in one read (OK, but indicate latency)\n");
    }
  }
  return gotRngData;
}

///////////////////////////////////////

bool UHokuyo::decodeData(char * msg, const int msgCnt, ULaserData * dest)
{
  bool result = true;
  const int MSL = 300;
  char gs[MSL];
  char s[MSL];
  //
  result = (dest != NULL);
  if (result)
  {
    makeGetDataString( gs, MSL);
    // compare command with expected data
    result = (strncmp(msg, &gs[1], 9) == 0);
    // debug
    if (verbose)
    {
      if (result)
      {
        //fprintf(stderr, "UHokuyo::decodeData is OK : expected 'G%s...' got '%s'\n", gs, msg);
        if (false and datalog.isOpen())
          datalog.toLog("UHokuyo::decodeData is OK : expected ", 1, gs, msg);
      }
      else
      {
        fprintf(stderr, "UHokuyo::decodeData failed: expected 'G%s...' got '%s'\n", gs, msg);
/*        if (false and datalog != NULL)
          fprintf(datalog, "UHokuyo::decodeData failed: expected 'G%s...' got '%s'\n", gs, msg);*/
      }
    }
    //result = true;
    // debug End
    if (result)
    { // test also next 3 characters
      result = ((msg[9] == '\n') and (msg[10] == '0') and (msg[11] == '\n'));
      if (not result and verbose)
      {
        snprintf(s, MSL, " - UHokuyo::decodeData failed on return status msg[9,10,11] should be 0x0a000a but is 0x%2x%2x%2x",
                 msg[9], msg[10], msg[11]);
        laslog.toLog(" - ", s);
        printf("%s\n", s);
      }
    }
  }
  if (result)
  {
    result = getDataTo(msg, msgCnt, dest);
    if (not result and verbose)
    {
      laslog.toLog(" - ", "transfer of data failed");
      printf(" - UHokuyo::decodeData: transfer of data to buffer failed\n");
    }
  }
  if (result)
  {
      // 0 = cm, 1=mm, 2 = 10cm
    dest->setUnit(1);
    // subtract 1/2 scantime (10 scans per sec) plus
    // a bit for communication
    dest->setScanTime(dataRxTime - var.scanDelay->getValued()); // (0.5/10.0 + 0.01));
    dest->setSerial(serial++);
    dest->setValid(true);
  }
    // send message to port to remove
    // some garbage data
    // and request more data
  return result;
}

///////////////////////////////////////

bool UHokuyo::decodeName(char * msg)
{
  bool result = true;
  strncpy(name, msg, MAX_DEVICE_NAME_LNG - 1);
  name[MAX_DEVICE_NAME_LNG-1] = '\0';
  // debug
  // printf("UHokuyo::decodeName: set version info (length=%d)\n", strlen(msg));
  // debug end
  var.versionInfo->setValues(name, 0, true);
  // debug
  // printf("UHokuyo::decodeName: set version info (length=%d) OK\n", strlen(msg));
  // debug end
  return result;
}

///////////////////////////////////////

bool UHokuyo::changeMode(int scanangle, double resolution)
{
  const double baseRes = 360.0 / 1024.0;
  //
  angleResolution = roundi(resolution / baseRes) * baseRes;
  modeAngleScan =  mini(270,scanangle);
  printf("New width=%d res=%g\n", modeAngleScan, angleResolution);
  return true;
}

////////////////////////////////////////////////////

bool UHokuyo::sendToDevice(const char * msg, int lng)
{
  int t;
  bool result;
  UTime ts;

  //tcflush(fd, TCIFLUSH); /*Empty input buffer, not sure if this is a good idea*/
  t = write(hfd, msg, lng);            /* Write to filedescripter */
  if (laslog.isOpen())
  {
    ts.now();
    toLog(msg, lng, "->", ts);
  }
  result = (t >= 0);
/*  if (result)
    printf("Message send %d bytes: %s", lng, msg);
  else*/
  if (not result)
    printf("Write failed in to device %s\n", name);

  return result;
}

////////////////////////////////////////////////////

void UHokuyo::toLog(const char * data, int length, const char * pre, UTime ts)
{
  const int MSL = MAX_DATA_LNG + 1;
  char sd[MSL];
  int i;
  const char * cp = data;
  char * dp = sd;
  const int MTL = 30;
  char s[MTL];
  //
  if (laslog.isLogOpen())
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
    ts.getTimeAsString(s, true);
    fprintf(laslog.getF(), "%lu,%06lu %s%s(%2d)%s\n", ts.getSec(), ts.getMicrosec(), &s[3], pre, length, sd);
  }
}

////////////////////////////////////////////////////

int UHokuyo::receiveFromDevice(char * start,
                    int maxLng,
                    double timeoutSec)
{ // poll wait time
  int pollTime = roundi(timeoutSec * 1000.0); // ms
  int n, m = maxLng;
  struct pollfd sickstruct;
  //
  sickstruct.fd = hfd;
  sickstruct.events = POLLIN;
  bool timeout;
  int result = 0;
  char * bp = start;
  //
  if (verbose and not laslog.isOpen())
    laslog.openLog();
  else if (laslog.isOpen() and not verbose)
    laslog.closeLog();
  // receive a number of data that should include a header
  /* get new data */
  timeout = false;
  while ((m > 0) and not timeout)
  { // Wait for data in up to 100 ms
    if (poll(&sickstruct, 1, pollTime) != POLLIN)
      // timeout or other error - return
      timeout = true;
    else
    { /* read up to a full message */
      n = read(hfd, bp, m);
      if (n == -1)
      { // error
        perror("Error in read from serial line");
        break;
      }
      else
      { // allow log
        if (result == 0)
        { // first data - timestamp
          dataRxTime.now();
        }
        if ((n > 0) and laslog.isLogOpen())
          toLog(bp, n, "<-", dataRxTime);
        //
        bp[n] = '\0';
        m -= n;
        result += n;
        if (strstr(bp, "\n\n") != NULL)
          // a full message is received.
          break;
        bp = &start[result];
      }
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////

void UHokuyo::createBaseVars()
{
  ULaserDevice::createBaseVars();
  if (var.scanDelay != NULL)
    // set default value for scandelay for urg scanner
    var.scanDelay->setValued(0.07);
}

