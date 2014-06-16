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

#include <iostream>
#include <sys/utsname.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <unistd.h>

#include "usmrcl.h"

USmrOdoState::USmrOdoState()
{
  dh = 0.0;
  velocity = 0.0;
  dist = 0.0;
  dt = 0.0; // delta time since last update
  readFlags = 0;
}

//////////////////////////////////////////////

void USmrOdoState::logState(FILE * logf)
{
  if (logf != NULL)
  {
    fprintf(logf,
      // time     x      y     h     v   dist   dt    dh   readflags
      "%lu.%03lu %9.4f %9.4f %9.4f %9.4f %9.4f %9.3f %6.3f %x\n",
      pose.t.getSec(),
      pose.t.GetMilisec(),
      pose.x, pose.y, pose.h,
      velocity, dist, dt, dh, readFlags);
  }
}

//////////////////////////////////////////////

void USmrOdoState::print(const char * prestring)
{
  printf("%s %.3fx %.3fy %.2fdeg %.2fm/s %.2fm dt=%.4fs\n",
         prestring, pose.x, pose.y, pose.h * 180.0 / M_PI,
         velocity, dist, dt);
}

//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////
//////////////////////////////////////////////

USmrCl::USmrCl()
{
  //
//  odoLog = NULL;
  connected = false;
//  stop = true;
  sock = -1;
  port = 31001;
  strcpy(host, "localhost");
  tryHoldConnection = true;
  readCnt = 0;
  errCnt = 0;
  txCnt = 0;
  connCnt = 0;
  reply[0] = '\0';
  replyCnt = 0;
  replyLine[0] = '\0';
  evalReq = 0;
  evalRes = 0;
  evalResult[0] = '\0';
  //
  cmdLineQueued = -1;
  cmdLineStarted = -1;
  cmdLineFinished = -1;
  cmdLineStopCnd = -1;
  cmdLineSyntaxError = -1;
  cmdLineUserEvent = -1;
  // odometry every 40 ms.
//  simulated = false;
  for (int i = 0; i < maxGpsVals; i++)
    gpsVals[i] = 0.0;
  //
  //logIO = NULL;
  logIOMode = 2;
  logIO.setLogName("smrcl");
  //
  streamShowGpsEvery = 0; // 0 is off
  streamShowOdoEvery = 0; // 0 is off
  streamShowOdoCnt = 0;
  streamShowGpsCnt = 0;
//  running = false;
//  odoEvery = -1.0;
  streamInsTime = -1.0;
  verbose=false;
  streamImu = false;
  running = false;
  start();
}

///////////////////////////////////////////////

USmrCl::~USmrCl()
{ // Destructor
  doDisconnect();
  printf("stopped MRC connection\n");
  stop(true);
/*  if (connected)
  { // stop thread
    stop = true;
    if (running)
    {
      printf("USmrCl termination thread ... ");
      fflush(NULL);
      pthread_join(thClient, NULL);
      printf("[OK]\n");
    }
  }*/
/*  if (odoLog != NULL)
    fclose(odoLog);*/
  if (logIO.isOpen())
  {
    logIO.toLog("smrcl.log closed");
    logIO.closeLog();
  }
}

//////////////////////////////////////////////

void USmrCl::print(const char * prestring)
{
  printf("%s host='%s' port=%d connected=%s (%u times) tx %u, rx %u, err %u\n",
         prestring, host, port, bool2str(connected),
         connCnt, txCnt, readCnt, errCnt);
}

///////////////////////////////////////////

// bool USmrCl::measureOdo(double timeoutSec, UPoseTime * poseTime)
// {
//   bool result;
//   const int OC = 6;
//   double val[OC];
//   UTime t;
//   UPoseTime poseOld;
//   bool handled = true;
//   const char * repl = NULL;
//   int n = 0;
//   //
//   result = isConnected();
//   if (result)
//   {
// /*    result = sendSMR("eval $time; $odox; $odoy; $odoth; $odovelocity; $ododist",
//         double(timeoutMs) / 1000.0, &repl);*/
//     result = sendOnly("eval $time; $odox; $odoy; $odoth; $odovelocity; $ododist");
//     t.Now();
//     while (connected)
//     { // read data until
//       result = listen(30, &repl, &handled, NULL, NULL, NULL);
//       // debug
//       // printf("USmrCl::measureOdo: got (%s) replyLine %s\n",
//       //      bool2str(result), replyLine);
//       // debug end
//       n++;
//       if (result and not handled)
//         break;
//       if (t.getTimePassed() > timeoutSec)
//         break;
//     }
//     if (result and not handled)
//     { // get values from string
//       n = sscanf(repl, "%lf%lf%lf%lf%lf%lf",
//              &val[0], &val[1], &val[2],
//              &val[3], &val[4], &val[5]);
//       result = (n == 6);
//       if (result)
//       {
//         odoState.lock();
//         poseOld = odoState.pose;
//         odoState.pose.set(val[1], val[2], val[3]);
//         odoState.pose.t.setTime(val[0]);
//         odoState.dt = val[0] - poseOld.t.getDecSec();
//         odoState.dh = limitToPi(val[3] - poseOld.h);
//         odoState.velocity = val[4];
//         odoState.dist = val[5];
//         if (poseTime != NULL)
//           *poseTime = odoState.pose;
//         odoState.unlock();
//         eventPoseUpdated(false);
//       }
//     }
//   }
//   else
//   { // not connected
//     // debug printout
//     printf("smr not connected: (%s)\n",
//            "eval $time; $odox; $odoy; $odoth; $odovelocity; $ododist");
//     // debug end
//   }
//   //
//   return result;
// }

//////////////////////////////////////

void USmrCl::eventPoseUpdated(bool streamSource)
{ // nothing done here - should be overwrited to be meaningfull
  odoState.print("odo");
  ;
}

///////////////////////////////////////

void USmrCl::eventGpsUpdate(UPoseTime odoState,
                 double easting, double northing, double heading,
                 double quality, double satellites, double dop)
{
  printf("USmrCl::eventGpsUpdate: %lu.%06lu %.2fE, %.2fN, "
      "%fQ, %.0fsats, %.1f dop\n", odoState.t.getSec(), odoState.t.getMicrosec(),
      easting, northing,
        quality, satellites, dop);
}

///////////////////////////////////////

void USmrCl::eventInsUpdate(UPoseTime odoState,
                            double accx, double accy, double accz,
                            double roll, double tilt, double pan, double insTime)
{
  UTime t;
  t.setTime(insTime);
  printf("USmrCl::eventInsUpdate: %lu.%06lu %gaccx %gaccy %.2faccz %gdroll %gdtilt %gdpan"
      "\n", t.getSec(), t.getMicrosec(), accx, accy, accz, roll, tilt, pan);
}

///////////////////////////////////////////////////

void USmrCl::info(const char * msg, int type /*= scInfo*/)
{
  if (type == scInfo) // and verbose)
  {
    printf("%s\n", msg);
    toLog(msg, 3);
  }
  else if (type == scWarning)
  {
    printf("USmrCL:: WARN: %s\n", msg);
    toLog(msg, 0);
  }
  else if (type == scError)
  {
    printf("USmrCL:: ERR: %s\n", msg);
    toLog(msg, 0);
  }
  else // if (verbose)
  {
    printf("USmrCL:: DEBUG: %s\n", msg);
    toLog(msg, 0);
  }
}

////////////////////////////////////

bool USmrCl::tryConnect()
{
  int err = 0;
  struct sockaddr_in name;
  char s[MAX_SC_INFO_SIZE];
  struct hostent * hostEntry;
  char * hostaddr;
  struct in_addr IPadr;
  bool valid;
  //
  // if connected already, then disconnect
/*  if (connected)
    doDisconnect();*/
  // maintain connection, as MRC connection otherwise may get lost
  //
  if (not connected)
  {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1)
    {
      info(strerror(errno), scWarning);
      err = -1;
    }
    if (err == 0)
    { // resolve name
      //info("Socket created ... ", scInfo);
      // try convert host name to an IP number
      valid = inet_aton(host, &IPadr);
      if (not valid)
      { // host name is not an IP number, so try resolve name
        // try resolve host name to IP number
        h_errno = 0;
        hostEntry = gethostbyname(host);
        if (hostEntry != NULL)
        { // name to IP resolution succeded
          hostaddr = hostEntry->h_addr_list[0]; // first address in net byte order
          IPadr = * (struct in_addr *) hostaddr;
          snprintf(s, MAX_SC_INFO_SIZE, "%s has addr %s - connecting ...",
                hostEntry->h_name,
                inet_ntoa(IPadr));
          info(s, scInfo);
          // save IP address in place of host name (may be equal)
          strncpy(host, inet_ntoa(IPadr), MAX_HOST_LENGTH);
        }
        else
        { // h_errno should be set by the gethostname() function
          switch (h_errno)
          { // show error text
            case HOST_NOT_FOUND:
              info("Host not found", scWarning);
              break;
            case NO_DATA:
            //case NO_ADDRESS: - same as NO_DATA
              info("No IP adress found", scWarning);
              break;
            case NO_RECOVERY:
              info("Error while resolving name (no recovery)", scWarning);
              break;
            case TRY_AGAIN:
              info("Error at name server - try later.", scWarning);
              break;
            default:
              info("Unknown error occured while resolving host mane.", scWarning);
          }
          err = -1;
        }
      }
    }
  }
  //
  if (not connected and (err == 0))
  { // set up structure for connect
    name.sin_family = AF_INET;
    name.sin_port = htons(port);
    name.sin_addr.s_addr = inet_addr(host);
    // set to non-blocking, to be able to stop
    //fcntl(sock, F_SETFL, O_NONBLOCK);
    // try a connect
    err = connect(sock, (struct sockaddr *) &name, sizeof(name));
    //
    /* if (err != 0)
    { // connection error
      snprintf(s, MAX_SC_INFO_SIZE, "Connect error (Hang up ?)");
      info(s, scWarning);
    }*/
    //
    if (err == 0)
    { // create thread to handle connection
      //stop = false;
      snprintf(s, MAX_SC_INFO_SIZE, "... connected OK");
      info(s, scDebug);
      //connected = startThread();
      connected = true;
      errCnt = 0;
      txCnt = 0;
      readCnt = 0;
      connCnt++;
      connectionChange(true);
    }
    else
    {
      close(sock);
      sock = -1;
    }
  }
  return connected;
}

//////////////////////////////////////////////////////////

// bool USmrCl::startThread()
// {
//   bool result;
//   pthread_attr_t  thConAttr;
//   int err;
//   //
//   stop = false;
//   // Starts socket client thread 'runSockClient'
//   pthread_attr_init(&thConAttr);
//   // create socket client thread
//   err = pthread_create(&thClient, &thConAttr, &runSmrLoop, (void *)this);
//   result = (err == 0);
//   return result;
// }

///////////////////////////////////////////////////////////

// bool USmrCl::stopThread(bool andWait)
// {
//   stop = true;
//   if (andWait and running)
//     // wait for client read thread to stop
//     pthread_join(thClient, NULL);
//   //
//   return running;
// }

///////////////////////////////////////////////////////////

bool USmrCl::doDisconnect()
{ // terminate connection and stop read thread
  //
  if (connected)
    closeConnection();
  //
  return not connected;
}

/////////////////////////////////////////////////////////////////////

bool USmrCl::handleLineData(const int timeoutMs)
{
  bool result;
  //lock();
  result = listen(timeoutMs, NULL, NULL,
                  NULL, NULL, NULL);
  //unlock();
  return result;
}

/////////////////////////////////////////////////////////////////////

bool USmrCl::handleGetevents(const int timeoutMs)
{
  bool result = true;
  bool handled;
  bool handledID;
  bool handledEventTimeout;
  bool handledStream;
  const char * repl;
  int i=0, j;
  bool gotEventTimeout;
  bool gotGeteventReply;
  bool finished;
  const int MSL = 350;
  char s[MSL];
  const int EMPTY_LOOPS = 150;
  //
  gotEventTimeout = false;
  while (result)
  { // get all pending data until an eventtimeout
    i++;
    j = 0;
    sendString("getevent\n");
    gotGeteventReply = false;
    finished = false;
    while (not finished)
    { // read all, until reply on getevent
      j++;
      result = listen(timeoutMs, &repl, &handled,
                      &handledID, &handledEventTimeout, &handledStream);
      //
      if (handled and not handledStream)
      { // either an IDXX started or IDXX stopcond or IDXX syntax or
        //           IDXX flushed or eventtimeout or motioncontrol
        gotGeteventReply = true;
        if (handledEventTimeout)
          gotEventTimeout = true;
      }
      if (not isMessageInBuffer())
        // do not finish if more data in buffer
        // probably stream odometry
        finished = gotGeteventReply;
      if (j > EMPTY_LOOPS)
      { // more than 100 messages, but no eventtimeout
        //errCnt++;
        snprintf(s, MSL, "error %d more than %d empty loops, but no eventtimeout", errCnt, EMPTY_LOOPS);
        toLog(s, 1);
        break;
      }
    }
    if (gotEventTimeout)
      break;
    if (i > 10)
    { // handles up to 10 messages (without eventTimeout)
      //errCnt++;
      snprintf(s, MSL, "error %d more than 10 timeouts of each %dms but no eventtimeout", errCnt, timeoutMs);
      toLog(s, 1);
      break;
    }
  }
  return gotEventTimeout;
}

/////////////////////////////////////////////////////////////////////

bool USmrCl::isMessageInBuffer()
{
  bool result;
  char * p1;
  //
  if (replyCnt == 0)
    result = false;
  else
  {
    p1 = strchr(reply, '\n');
    result = (p1 != NULL);
  }
  return result;
}

/////////////////////////////////////////////////////////////////////

bool USmrCl::listen(int timeout, // in ms
                    const char ** gotReply,
                    bool * handled,
                    bool * handledID,
                    bool * handledEventTimeout,
                    bool * handledStream)
{
  bool result;
  bool lineUsed = false;
  bool gotData;
  //
  {
    result = false;
    if (gotReply != NULL)
      *gotReply = NULL;
    if (handledID != NULL)
      *handledID = false;
    if (handledEventTimeout != NULL)
      *handledEventTimeout = false;
    if (handledStream != NULL)
      *handledStream = false;
    // not finised until a timeout is encountered
    gotData = getLineFromSocket(timeout);
    if (gotData)
    { // handle IDs and stream data here
      lineUsed = setDriveState(handledID, handledEventTimeout, handledStream);
      result = true;
    }
    else
    { // received a timeout
      // discard old data
      replyCnt = 0;
    }
  }
  if (gotReply != NULL)
    // reply line is zero terminated
    *gotReply = replyLine;
  if (handled != NULL)
    *handled = lineUsed;
  return result;
}

/////////////////////////////////////////////////////////////////////

bool USmrCl::startOdoStream(const int samplesInterval)
{
  const int MSL = 300;
  char s[MSL];
  bool result;
  //
  //odoEvery = double(samplesInterval) / 100.0;
  // change is needed here only (readflags will else be set to 0).
  // Stream always returns time as first "variable"
  // NB! stream can de divided into more commands (max 8 in each command)
  // $xkalman" "$ykalman" "$thkalman
  // workaround
  Wait(0.11);
  // workaround end
  snprintf(s, MSL,
           "stream %d \"$odox\" \"$odoy\" \"$odoth\" "
           "\"$odovelocity\" "
           "\"$xkalman\" \"$ykalman\" \"$thkalman\" "
           "\"$kalmanstatus\"\n",
           samplesInterval);
  result = sendString(s);
  // workaround
  Wait(0.11);
  // workaround end
  //printf("send (%s) to smr '%s'\n", bool2str(result), s);

  //GPSquality: 1=nofix 2=floating, 3=fix
  snprintf(s, MSL,
           "stream %d "
               "\"$gpsdop\" \"$gpsquality\"\n",
               samplesInterval);
  sendString(s);
  // workaround
  Wait(0.11);
  // workaround end
  //printf("send %s to smr '%s'\n", bool2str(result), s);

/* from smrdemo.c:
  p->pimuroll = putinternvar("imuroll");
  p->pimupitch = putinternvar("imupitch");
  p->pimuyaw = putinternvar("imuyaw");
  p->pimuaccx = putinternvar("imuaccx");
  p->pimuaccy = putinternvar("imuaccy");
  p->pimuaccz = putinternvar("imuaccz");
  p->pimutemp = putinternvar("imutemp");
  p->pimutime = putinternvar("imutime");*/
// stream also ins
  if (streamImu)
  { // added hakomanual
    snprintf(s, MSL,
           "stream %d  \"$imuaccx\" \"$imuaccy\" \"$imuaccz\" "
                      "\"$imuroll\" \"$imupitch\" \"$imuyaw\" \"$imutime\" \"$hakomanual\"\n",
               samplesInterval);
    sendString(s);
  }
  else
  {
    snprintf(s, MSL,
             "stream %d \"$hakomanual\" \"$hakoliftinggearpos\" \"$hakopowertakeoffspeed\"\n",
                 samplesInterval);
    sendString(s);
  }
  //printf("send %s to smr '%s'\n", bool2str(result), s);
  return result;
}

//////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////

bool USmrCl::getLineFromSocket(int timeout)
{ // timeout is in ms
  bool res = false;
  char * p1;
  int n;
  // continue reading until ordered to stop
  while (connected)
  { // listen for a full line
    // check for a full message in reply buffer
    p1 = strchr(reply, '\n');
    if (p1 != NULL)
    { // a line is available
      n = p1 - reply;
      // copy complete line to line buffer
      memcpy(replyLine, reply, n);
      replyLine[n] = '\0';
      // leave remining data in buffer
      // advanve to first usable character
      while ((*p1 != '\0') and (*p1 <= ' '))
        p1++;
      n = p1 - reply;
      if (*p1 == '\0' or (n >= replyCnt))
      { // no more data
        replyCnt = 0;
      }
      else
      { // save remaining part of data
        replyCnt -= n;
        memmove(reply, p1, replyCnt);
      }
      reply[replyCnt] = '\0';
      res = true;
      break;
    }
    // get new data from line
    if (not getDataFromLine(timeout, false))
    {
      break;
    }
  }
  if (res)
    readCnt++;
  else if (errCnt > (txCnt / 10 + 5))
  {
    // debug
    printf("USmrCl::closing connection to smr txCnt=%d errCnt=%d\n", txCnt, errCnt);
    // debug end
    toLog("Closed connection due to errCnt (too many timeouts)", 0);
    // do not try any further
    close(sock);
    sock = -1;
    connected = false;
    connectionChange(false);
  }
  return res;
}

////////////////////////////////////////////////////////

bool USmrCl::getDataFromLine(int timeout, bool waitFullTime)
{
  bool result = false;
  int len;
  int err;
  struct pollfd pollStatus;
  bool dataAvailable = false;
  int pollTime;
  UTime t;
  const int MSL = 200;
  char s[MSL];
  // get start time
  if (waitFullTime)
    pollTime = timeout;
  else
  { // divide into smaller events, to reduce excess poll time
    // wait at least 6 ms (to avoid waiting 0 ms)
    pollTime = maxi(6, timeout / 20);
    t.Now();
  }
  // preare status poll
  pollStatus.fd = sock;
  pollStatus.events = POLLIN  +   /*  0x0001  There is data to read */
                      POLLPRI +   /*  0x0002  There is urgent data to read */
                      /* POLLOUT 0x0004  Writing now will not block */
                      POLLERR +   /*  0x0008  Error condition */
                      POLLHUP +   /*  0x0010  Hung up */
                      POLLNVAL;   /*  0x0020  not valid */
  pollStatus.revents = 0;  // clear returned events
  if (connected)
  {
    while (connected)
    { // get connection status and wait for up to 'pollTime' ms
      dataAvailable = false;
      err = poll(&pollStatus, 1, pollTime);
      if (err < 0)
      { // could not poll - no connection
        info("Connection lost (poll err)", scWarning);
        connected = false;
      }
      else if (pollStatus.revents != 0)
      { // status is not timeout
        if ((pollStatus.revents & POLLIN) > 0)
          // normal data available
          dataAvailable = true;
        else if ((pollStatus.revents & POLLPRI) > 0)
          // priority data
          dataAvailable = true;
        else if (((pollStatus.revents & POLLERR) > 0) or
            ((pollStatus.revents & POLLHUP) > 0)
                )
        { // error situation (error, hangup or not valid)
          info("Connection lost (poll)", scWarning);
          connected = false;
        }
        else if ((pollStatus.revents & POLLNVAL) > 0)
        {
          info("not valid error - never mind", scWarning);
        }
        else
        { // error situation (error, hangup or not valid)
          info("Connection lost (unknown poll error)", scWarning);
          connected = false;
        }
      }
      else
      { // poll timeout
        break;
      }
      if (dataAvailable)
      { // data available to read
        // get what is available up to buffer length
        len = recv(sock, &reply[replyCnt], MAX_REPLY_LENGTH - replyCnt - 1, 0);
        if (len > 0)
        { // add to reply string
          replyCnt += len;
          reply[replyCnt] = '\0';
          result = true;
        }
        if ((len <= 0) and (errno != EAGAIN))
        { // error - connection is lost       h_errno
          perror("USmrCl::getDataFromLine:");
          snprintf(s, MSL, "No data and not EAGAIN!? errno = %d : %s", errno, strerror(errno));
          toLog(s, 2);
          Wait(0.5);
          connected = false;
        }
      }
      if (dataAvailable)
        break;
      if (replyCnt > (MAX_REPLY_LENGTH - 30))
        break;
      if (roundi(t.getTimePassed() * 1000.0) > timeout)
        break;
    }
    if (not connected)
      connectionChange(false);
  }
  // debug
  if (result)
    toLog(reply, 2);
  // debug end
  return result;
}

///////////////////////////////////////////////////

void USmrCl::closeConnection()
{ // shutdown connection -- tell the other end
  tryHoldConnection = false;
  //
  if (sock >= 0)
  { // close connection, but do not stop MRC
    if (connected)
    { // save MRC log
      saveMrcLog(false);
      // remove logvars
      sendString("control \"removelogvars\"\n");
      Wait(0.3);
      // close connection, but let mrc live
      sendString("logoff\n");
    }
    if (connected)
      // read any pending messages
      handleGetevents(500);
    shutdown(sock, SHUT_RDWR);
    close(sock);
    sock = -1;
    connectionChange(false);
  }
  connected = false;
}

////////////////////////////////////////////////////////////

void USmrCl::saveMrcLog(bool restart)
{
  UTime t;
  const int MSL = 30;
  char s[MSL];
  const int MRL = 300;
  char reply[MRL];
  // close logfile
  sendString("control \"stoplog\"\n");
  t.now();
  t.getForFilename(s);
  snprintf(reply, MRL, "control \"savelog\" \"mrc_%s.logg\"\n", s);
  sendString(reply);
  sendString("control \"resetlog\"\n");
  if (restart)
    sendString("control \"startlog\"\n");
}

////////////////////////////////////////////////////////////

void USmrCl::toLog(const char * logString, const int logLevel)
{
  UTime t;
  if (logIO.isOpen() and (logLevel <= logIOMode))
  {
    t.Now();
    if (logLevel == 1)
      fprintf(logIO.getF(), "%lu.%06lu %d %d %d %d (%d %d) %s\n",
            t.getSec(), t.getMicrosec(), logLevel,
            cmdLineQueued, cmdLineStarted, cmdLineFinished,
            errCnt, txCnt,
            logString);
    else
      fprintf(logIO.getF(), "%lu.%06lu %d %s\n",
            t.getSec(), t.getMicrosec(), logLevel, logString);
    //fflush(logIO);
  }
}

////////////////////////////////////////////////////////////

// bool USmrCl::sendSMR(const char * cmd,
//                      double timeoutSec,
//                      const char ** lastReply)
// {
//   bool res;
//   const int MSL = 100;
//   char s[MSL];
//   bool handledID;
//   bool handledEventTimeout;
//   bool handled;
//   int timeout = 120;
//   UTime t;
//   //
//   if (connected)
//   { // running live
//     lock();
//     t.Now();
//     res = sendOnly(cmd); // and log
//     while (connected)
//     {
//       res = listen(timeout, lastReply, &handled, &handledID,
//                      &handledEventTimeout, NULL);
//       if (res and (handledID or not handled))
//         break;
//       else if (t.getTimePassed() > timeoutSec)
//       {  // there should always be a reply
//         errCnt++;
//         snprintf(s, MSL, "error %d failed to get reply within %gs",
//                  errCnt, timeoutSec);
//         // debug
//         printf("USmrCl::sendSMR: %s\n", s);
//         // debug end
//         toLog(s, 0);
//         break;
//       }
//     }
//     unlock();
//   }
//   else
//   {
//     res = false;
//     // debug printout
//     printf("smr not connected: (%s)\n", cmd);
//     // debug end
//   }
//   //
//   // debug
//   if (connected and not res)
//   {
//     snprintf(s, MSL, "Failed to finish '%s'", cmd);
//     info(s, scWarning);
//   }
//   // debug end
//   //
//   return res;
// }

////////////////////////////////////////////////////////////

// bool USmrCl::sendSMRgetId(const char * cmd,
//                           double timeoutSec,
//                           int * lineID,
//                           bool mustBeQueued)
// {
//   bool res;
//   const int MSL = 100;
//   char s[MSL];
//   int n = 0;
//   int tOut = 120; // to allow HACO to comply
//   bool handledID = false;
//   bool handled = false;
//   UTime t;
//   int qID;
//   //
//   if (connected)
//   { // running live
//     lock();
//     res = sendOnly(cmd);
//     t.Now();
//     // collect data in XX ms - should ensure reply to this request
//     qID = cmdLineQueued;
//     while (connected)
//     {
//       n++;
//       res = listen(tOut, NULL, &handled, &handledID, NULL, NULL);
//       // debug
//       // printf("USmrCl:: got (%s) %s\n", bool2str(res), replyLine);
//       // debug end
//       if (mustBeQueued)
//       { // test if a new line is queued
//         if (qID != cmdLineQueued)
//           break;
//       }
//       else if (res)
//         // just test for ID reply (or any other unhandled data)
//         if (handledID or not handled)
//           break;
//       // always space for NN stream events
//       if (n > 30)
//       {
//         if (t.getTimePassed() > timeoutSec)
//         {
//           errCnt++;
//           res = false;
//           break;
//         }
//         // debug
//         // printf("USmrCl::sendSMRgetId waiting for reply, tried %d times\n", n);
//         // debug end
//       }
//     }
//     unlock();
//     if (logIOMode >= 3)
//     {
//       snprintf(s, MSL, "listen()'ed n=%d times (time passed = %f) tOut=%dms", n, t.getTimePassed(), tOut);
//       toLog(s, 3);
//     }
//   }
//   else
//   {
//     res = false;
//     // debug printout
//     printf("smr not connected: (%s)\n", cmd);
//     // debug end
//   }
//   //
//   if (lineID != NULL)
//     *lineID = cmdLineQueued;
//   // debug
//   if (connected and not res)
//   {
//     snprintf(s, MSL, "sendSMRGetId(to=%gs) Failed to finish '%s'", timeoutSec, cmd);
//     info(s, scWarning);
//     toLog(s, 1);
//   }
//   // debug end
//   //
//   return res;
// }

////////////////////////////////////////////////////////////

bool USmrCl::sendSMReval(const char * varName, double timeoutSec,
                         double * value, char * valStr, const int valStrCnt)
{
  bool res;
  int n = 0, m;
  double val;
  UTime t;
  //
  if (connected)
  { // running live
    evalReq++;
    res = sendString("eval ", varName, "\n");
    t.Now();
    if (res)
    {
      while (connected and evalReq != evalRes)
      {
        n++;
        if (t.getTimePassed() > timeoutSec)
        {
          res = false;
          break;
        }
      }
    }
    if (res)
    {
      m = sscanf(evalResult, "%lg", &val);
      res = (m == 1);
    }
  }
  else
  {
    res = false;
    // debug printout
    printf("smr not connected: (eval %s)\n", varName);
    // debug end
  }
  if (value != NULL)
    *value = val;
  //
  return res;
}

////////////////////////////////////////////////////////////

bool USmrCl::setDriveState(bool * idEevent, bool * eventTimeout, bool * streamData)
{
  bool result = true;
  int line;
  const int MSS = 100;
  char s[MSS];
  int sc, n;
  const int OC = 7;
  double val[OC];
  double gpsNew[maxGpsVals];
  const int MESL = 100;
  char es[MESL];
  UPoseTime poseOld;
  bool isID = false;
  bool isTimeout = false;
  bool isStream = false;
//  bool isUserEvent = false;
  char *p1, *p2;
  bool gpsChange;
  double v, wTime;
  //
  n = sscanf(replyLine, "ID%d %s %d", &line, s, &sc);
  if (n >= 2)
  { // this is a line ID status
    if (strncmp(s, "queued", 6) == 0)
      cmdLineQueued = line;
    else if (strncmp(s, "started", 7) == 0)
      cmdLineStarted = line;
    else if (strncmp(s, "stopcond", 8) == 0)
    {
      cmdLineFinished = line;
      if (n == 3)
        cmdLineStopCnd = sc;
    }
    else if (strncmp(s, "flushed", 7) == 0)
      // all queued lines are terminated
      cmdLineFinished = cmdLineQueued;
    else if (strncmp(s, "assignment", 10) == 0)
      // typically for "targethere" command, this assignment
      // signal the end of the command
      cmdLineFinished = line;
    else if (strncmp(s, "syntaxerror", 11) == 0)
    {
      cmdLineSyntaxError = line;
      cmdLineFinished = line;
    }
    else
    {
      result = false;
      snprintf(es, MESL, "unknown ID%d reply: '%s'", line, s);
      printf("USmrCl::setDriveState: %s\n", es);
      toLog(es, 0);
    }
    isID = true;
    lineStateUpdated();
    gotGeteventReply = true;
  }
  else if (strncmp(replyLine, "stream", 6) == 0)
  { // decode stream -- see startOdoStream() for ordered sequence
    // debug
    //printf("reply '%s'\n", replyLine);
    // debug end
    n = sscanf(replyLine, "stream %lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
                   &val[0], &val[1], &val[2],
                   &val[3], &val[4], // &val[5], // &val[6],
                   // xkalman, ykalman,    thkalman,  kalmanstatus, gpsdop, gpsFixMode
                   &gpsNew[0], &gpsNew[1], &gpsNew[2], &gpsNew[3], &gpsNew[4], &gpsNew[5],
           &ins[0], &ins[1], &ins[2], &ins[3], &ins[4], &ins[5], &ins[6]);
    if (n >= 5)
    { // lock state while beeing updated
      odoState.lock();
      poseOld = odoState.pose;
      odoState.pose.set(val[1], val[2], val[3]);
      odoState.pose.t.setTime(val[0]);
      odoState.dt = val[0] - poseOld.t.getDecSec();
      odoState.dh = limitToPi(val[3] - poseOld.h);
      odoState.velocity = val[4];
      odoState.dist = 0.0; //val[5];
/*      if (n == 7)
        odoState.readFlags = roundi(val[6]);
      else*/
      odoState.readFlags = 0;
      odoState.unlock();
      eventPoseUpdated(true);
      if (streamShowOdoEvery > 0)
      { // print updates to console
        if (streamShowOdoCnt % streamShowOdoEvery == 0)
        {
          printf("Odo update #%d %.3fx, %.3fy %.4fth\n",
                 streamShowOdoCnt, val[1], val[2], val[3]);
        }
      }
      streamShowOdoCnt++;
      if (n >= 9) // northing
      { // there may be a gps value
        gpsChange = false;
        for (int i = 0; i < maxGpsVals; i++)
        {
          if (gpsNew[i] != gpsVals[i])
          {
            gpsVals[i] = gpsNew[i];
            gpsChange = true;
          }
        }
        //
        if (true or gpsChange)
        { // count number of stream gps updates
          // tell system that a gps update is received
          // eventGpsUpdate(UposeTime, east, north, heading, qual, kalman status (satl), dop)
          eventGpsUpdate(odoState.pose, gpsNew[0], gpsNew[1], gpsNew[2],
                         gpsNew[5], gpsNew[3], gpsNew[4]);
          if (streamShowGpsEvery > 0)
          { // print updates to console
            if (streamShowGpsCnt % streamShowGpsEvery == 0)
            {
              printf("GPS update %d %.3fE, %.3fN %.0fsats %.1fdop, %.1fqual\n",
                     streamShowGpsCnt, gpsVals[0], gpsVals[1],
                     gpsVals[2], gpsVals[3], gpsVals[4]);
            }
          }
          streamShowGpsCnt++;
        }
      }
      if (not streamImu and  n > 12)
      { // ins values are available too
        //"stream %d \"$hakomanual\" \"$hakoliftinggearpos\" \"$hakopowertakeoffspeed\"\n",
        eventHakoVarUpdate(roundi(ins[0]), roundi(ins[1]), roundi(ins[2]));
      }
      if (streamImu and  n > 16)
      { // ins values are available too
        if (true or (ins[6] != streamInsTime))
        { // accx, accy, accz, dRoll, dTilt, dPan, updTime (and hakomanual)
          eventInsUpdate(odoState.pose, ins[0], ins[1], ins[2],
                       ins[3], ins[4], ins[5], ins[6]);
          streamInsTime = ins[6];
        }
        if (streamShowInsEvery > 0)
        { // print updates to console
          if (streamShowInsCnt % streamShowInsEvery == 0)
          {
            printf("INS update %d %gx %gy %gz %gO %gP %gK %g(time)\n",
                   streamShowInsCnt,ins[0], ins[1], ins[2],
                   ins[3], ins[4], ins[5], ins[6]);
          }
        }
        streamShowInsCnt++;
      }
      // save on odometry log (if open)
/*      if (odoLog != NULL)
        fprintf(odoLog, "%s\n", &replyLine[7]);*/
    }
    else
      toLog("Stream reply decoding failed", 0);
    isStream = true;
  }
  else if (strncmp(replyLine, "userevent", 9) == 0)
  {
    //isUserEvent = true;
    p1 = &replyLine[9];
    while (isspace(*p1))
      p1++;
    // assume that the userevent is a number - an integer
    line = strtol(p1, &p2, 0);
    if (p1 != p2)
    { // line is OK - save
      cmdLineUserEvent = line;
      lineStateUpdated();
    }
    else if (strncmp(p1, "ev", 2) == 0)
    {
      gotUserEvent(p1);
    }
    else
      printf("got an unknown userevent: %s\n", replyLine);
  }
  else if (strncmp(replyLine, "eventtimeout", 12) == 0)
    isTimeout = true;
  else if (strncmp(replyLine, "motioncontrol", 13) == 0)
    // also a reply to 'getevent', so handled
    ;
  else if (strncmp(replyLine, "watch", 5) == 0)
  {
    p1 = &replyLine[6];
    while (isspace(*p1))
      p1++;
    p2 = strstr(p1, "fired");
    if (p2 != NULL)
    {
      n = p2 - p1 - 1;
      n = mini(n, MSS - 1);
      strncpy(s, p1, n);
      s[n] = '\0';
      p2 += 5;
      wTime = strtod(p2, &p1);
      eventWatchFired(s, wTime);
    }
  }
  else
  { // probably an eval reply
    p1 = replyLine;
    n = sscanf(p1, "%lg", &v);
    if (n == 1)
    { // result can be evaluated - assume eval result.
      strncpy(evalResult, p1, MaxEvalLen);
      evalRes = evalReq;
    }
    else
      // unknown ignored result
      result = false;
  }
  //
  if (idEevent != NULL and isID)
    *idEevent = isID;
  if (streamData != NULL and isStream)
    *streamData = isStream;
  if (eventTimeout != NULL and isTimeout)
    *eventTimeout = isTimeout;

  // debug
  if (isStream and not result)
    printf("This is an odd event - handled=false and istream=true!!!!!!!\n");
  // debug end

  return result;
}

////////////////////////////////////////////////////////////

bool USmrCl::sendOnly(const char * cmd)
{
  bool result = true;
  int d, j, n;
  struct pollfd pollStatus;
  const int MSL = 300;
  char s1[MSL];
  int length;
  int timeoutMs = 50;
  //
  if (connected)
  {
    // preare status poll
    pollStatus.fd = sock;
    pollStatus.events = POLLOUT +   /*  0x0004  */
                        POLLERR +   /*  0x0008  Error condition */
                        POLLHUP +   /*  0x0010  Hung up */
                        POLLNVAL;   /*  0x0020  not valid */
    pollStatus.revents = 0;
    //
    // get connection status
    j = poll(&pollStatus, 1, timeoutMs);
    if ((j > 0) and ((pollStatus.revents & 0x0038) > 0))
    { // not POLLOUT, connection lost
      info("SMRCL Connection lost", scWarning);
      connected = false;
      result = false;
    }
    // status out is not used, if no error, then just try
    if (result)
    { // still connected (no error (yet))
      // add a newline
      //snprintf(s, MSL, "%s\n", cmd);
      length = strlen(cmd);
      d = 0;
      for (j = 0; j < 10; j++)
      { // may require more than one transmission
        // try up to 10 times to send length bytes.
        n = send(sock, &cmd[d], length - d, 0);
        d += n;
        if ((d == length) or not connected)
          // may be disconnected by read status
          break;
      }
      result = (d == length);
      if (not result)
      {
        errCnt++;
        snprintf(s1, MSL, "Send %d of %d bytes only (of '%s')",
          d, length, cmd);
        //info(s, scWarning);
        toLog(s1, 1);
      }
      txCnt++;
    }
    if (result)
      toLog(cmd, 1);
  }
  //
  return result;
}


////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////

void USmrCl::setIOLogClosed()
{
  logIO.closeLog();
}

////////////////////////////////////////////////////////////

// bool USmrCl::setIOLog(const char * path, const char * name,
//                       const int mode)
// {
//   const int MFL =500;
//   char fn[MFL];
//   char ts[30];
//   UTime t;
//   //
//   t.Now();
//   snprintf(fn, MFL, "%s/%s", path, name);
//   //
//   if (logIO != NULL)
//     fclose(logIO);
//   logIO = fopen(fn, "w");
//   if (logIO != NULL)
//   {
//     t.getTimeAsString(ts, true);
//     snprintf(fn, MFL, "%s logfile opened", ts);
//     toLog(fn, 0);
//   }
//   logIOMode = mode;
//   //
//   return (logIO != NULL);
// }

//////////////////////////////////////////////////////////////

bool USmrCl::getSimulatedPose()
{ // should be provided by a virtual function at a higher level.
  // and loaded into odoState
  return false;
}

//////////////////////////////////////////////////////////////

void USmrCl::lineStateUpdated()
{
  ; // nothing to do here -- could be overwritten for use
}

//////////////////////////////////////////////////////////////

void USmrCl::connectionChange(bool connected)
{
  printf("Connected=%s\n", bool2str(isConnected())); //nothing here
}

////////////////////////////////////////////////////////////////

bool USmrCl::sendString(const char * s1, const char * s2, const char * s3)
{
  bool result;
  lock();
  result = sendOnly(s1);
  if (s2 != NULL)
  {
    result = sendOnly(s2);
  }
  if (s3 != NULL)
  {
    result = sendOnly(s3);
  }
  unlock();
  return result;
}

////////////////////////////////////////////

void *startSmrCl(void *ptr)
{ // called by create_thread
  USmrCl * obj;
  // pointer is an UResSmrCtl instance
  obj = (USmrCl *) ptr;
  // run main loop
  obj->run();
  return NULL;
}

/////////////////////////////////////////////

bool USmrCl::start()
{
  bool result = false;
  pthread_attr_t  thConAttr;
  int err;
  //
  if (not running)
  {
    stopRead = false;
    // Starts socket client thread 'runSockClient'
    pthread_attr_init(&thConAttr);
    // create socket client thread
    err = pthread_create(&thRead, &thConAttr, &startSmrCl, (void *)this);
    result = (err == 0);
    pthread_attr_destroy(&thConAttr);
  }
  //
  return result;
}

/////////////////////////////////////////////

void USmrCl::stop(bool andWait)
{
  stopRead = true;
  if (running)
  {
    if (andWait)
    {
      printf("USmrCl:: stopping read loop ...\n");
      pthread_join(thRead, NULL);
      printf("\t[OK]\n");
    }
    else
      Wait(0.05);
  }
}

/////////////////////////////////////////////


void USmrCl::run()
{
  int getEventCnt = 0;
  const double getEventCntInterval = 0.25; // else just listen to line
  int notConnectedLoop = 0;
  UTime t, t2;
  //
  running = true;
  gotGeteventReply = true;
  t.now();
  t2.now();
  while (not stopRead)
  { // test driver status
    if (isConnected())
    { // NB! smr interface is locked during get-event and line handling ...
      getEventCnt++;
      if ((t.getTimePassed() > getEventCntInterval))
      {
        sendString("getevent\n");
        gotGeteventReply = false;
        t.now();
        // debug
        //printf("USmrCl::run: send %d getevent after %.3f\n", getEventCnt, t2.getTimePassed());
        // debug end
      }
      else
      { // handle any pending streaming data
        handleLineData(20);
      }
    }
    else
    { // not connected
      if (tryHoldConnection and ((notConnectedLoop % 32) == 16))
      // try a (re)connect
        tryConnect();
      //
      if (not isConnected())
      {
        notConnectedLoop++;
        Wait(0.1);
      }
    }
  }
  running = false;
}

