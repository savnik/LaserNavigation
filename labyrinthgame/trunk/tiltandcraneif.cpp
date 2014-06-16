/***************************************************************************
 *   Copyright (C) 2013 by DTU (Christian Andersen, Andreas Emborg, m.fl.) *
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


/**
 * Labyrinth game start and end code */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ugen4/utime.h>
#include <ugen4/ucommon.h>
#include <ugen4/ulock.h>
#include <urob4/ulogfile.h>
#include <../libs/ugen4/ulock.h>
#include "labyrinthgame.h"
#include "tiltcontrol.h"
#include "tiltandcraneif.h"
#include "fwguppy.h"

/// is interface control  thread running
bool tcrifThreadRunning;
/// stop running interface control  thread
bool stopTcrifThread;
/// interface control thread handle
pthread_t thTcrif;
/// is interface read thread running
bool readThreadRunning;
/// stop running interface read thread
bool stopReadThread;
/// interface read thread handle
pthread_t thTcrifRead;
/// start game control thread
bool startTcrifCtrl();
/// stop game control thread
void stopTcrifCtrl();
// is ball available for crane
bool ballAvailable;
// is crane swing switch closed
bool craneSwitch;
// is start switch pushed
bool startGameSwitch;
// flag to signal that ball shouldbe loadedwith crane
bool loadBallWithCrane = false;
// tilt angle
float tiltXangle, tiltYangle;
/// interface device handle for the control interface file
int devif = -1;
// error flag for this unit
bool errorTiltAndCraneIf;
// semaphore to tell output communication that new values are available
USemaphore semOutputCtrl;
// debug string to crane-tilt interface
char debugIfString[MIOL];
// ballance point for borar in command units [0..255]
int xyBalance[2] = {128,128};
/// old control values - to save a bit of time in IO interface
int xi_old = -1, yi_old = -1;
/// distance from park position to ball - in stepper units
int craneParkToBall = 80;
/// distance from park position to ball drop position - in stepper units
int craneParkToDrop = 470;
/// distance down to pick ball in stepper units from park position
int craneDownToBall = 1680;
/// distance to lift ball to test if got ball in stepper units from park position
int craneDownToTest = 1400;
/// distance where ball is dropped to game
int craneDownToDrop = 200;
/// time of last status
UTime statusTime;
/// is power on to the hardware
bool powerOn = false;
int ctrlCnt = 0;
// enum controlState { IF_INIT, IF_WAIT, IF_CRANE_INIT, 
//                     IF_CRANE_LEFT_TO_BALL, IF_CRANE_DOWN_TO_BALL, IF_CRANE_UP_TO_TEST, 
//                     IF_CRANE_UP, IF_CRANE_LEFT_TO_DROP, IF_CRANE_DOWN_TO_DROP,
//                     IF_CRANE_UP_TO_PARK, IF_CRANE_RIGHT_TO_PARK,
//                     IF_TILT};
/// 
controlState ifState;
/// tilt scale from tilt angle to tilt units
float tiltScale = 10000;
/// timing char send
char timingCharSend = 'a';
/// timing char received
char timingCharReceived = '!';
// logfile for interface
ULogFile iflog;


/// /////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////

void getStatus();

// bool loadBall()
// {
//   for (int i = 0; i < 5; i++)
//   {
//     getStatus();
//     printf("loadBallWithCrane: Loading ball ...\n");
//     Wait(0.5);
//   }
//   printf("loadBallWithCrane: Faked ball load.\n");
//   return true;
// }

/// ///////////////////////////////////////////////////////////////////////////

/**
 * log status and for communication interface */
void logStatus(UTime t, char direction, char * poststr)
{
  if (iflog.isOpen())
  {
//     UTime t;
//     t.now();
    char * p1 = strrchr(poststr, '\n');
    if (p1 != NULL)
      *p1 = '\0';
    fprintf(iflog.getF(), "%ld.%06ld %c %s\n",
                    t.getSec(), t.GetMicrosec(), direction, poststr);
  }
}


/** send string to interface device
 * \param s is string to send - should include a newline (\n) 
 * \returns true if send */
bool sendStringToIOCtrl(const char * s)
{
  bool isOK = false;
  if (globalDebug == 0)
  {
    int n = strlen(s);
    int m = write(devif, s, n);
    if (m < 0)
      printf("tiltandcraneif: failed to send %d chars '%s' to device\n", n, s);
    else if (m == 0)
      printf("tiltandcraneif: nothing send\n");
    else if (m < n)
      printf("tiltandcraneif: Send %d of %d chars - of '%s'\n", m, n, s);
    else
    { // send all
      fsync(devif);
      // debug
      if (s[0] != 'B' and s[0] != 'p' and s[0] != 'P' and s[0] != 'j')
        printf("tiltandcraneif: send to IO ctrl:%s", s);
      // debug
      isOK = true;
    }
  }
  else
  {
    printf("tiltandcraneif: would send %s\n", s);
    isOK = true;
  }
  errorTiltAndCraneIf = not isOK;
  return isOK;
}

/**
 * send tilt angles to interface
 * \param x is angle on the x - axis (roll) - 'p' control
 * \param y is tilt on y axis (pitch) - 'P' control
 * \returns true if send */
bool sendNewAngle(float tiltXangle, float tiltYangle)
{
  const int MSL = 20;
  char s[MSL];
  s[0] = '\0';
  int xi = roundi(tiltXangle * tiltScale) + xyBalance[0];
  int yi = roundi(tiltYangle * tiltScale) + xyBalance[1];
  bool isOK = true;
  UTime t;
  //
  xi = limiti(xi, 0, 255);
  yi = limiti(yi, 0, 255);
  if ((xi != xi_old) and (yi != yi_old))
    snprintf(s, MSL, "p%03d\nP%03d\nj=%c\n", xi, yi, timingCharSend);
  else if (xi != xi_old)
    snprintf(s, MSL, "p%03d\n", xi);
  else if (yi != yi_old)
    snprintf(s, MSL, "P%03d\n", yi);
  // debug removed
//   xi_old = xi;
//   yi_old = yi;
  // debug removed
  ctrlCnt++;
  if (s[0] != '\0')
  {
    t.now();
    isOK = sendStringToIOCtrl(s);
    logStatus(t, '>', s);
  }
  if (ctrlCnt % 10 == 0)
  {
    if (timingCharSend > 'z')
      timingCharSend = 'a';
//     if (s[0] != '\0')
//       printf("ctrl %.5fx, %.5fy, %s", tiltXangle, tiltYangle, s);
  }
  return isOK;
}

/// ///////////////////////////////////////////////////////////////////////////

/**
 * Get switch status from interface controller */
void * runIfRead(void * notUsed)
{
  #define MAX_LEN 100
  char rxbuffer[MAX_LEN];
  int i = 0;
  int rxErrorCnt = 0;
  bool handled = false;
  UTime tOn, tOff;
  //
  readThreadRunning = true;
  //
  printf("interface read thread is running\n");
  //
  if (debugLog)
    iflog.openLog("interface");
  //
  if (iflog.isOpen())
    printf("interface log %s is open\n", iflog.getLogFileName());
  //
  i = 0;
  rxbuffer[0] = '\0';
  tOff.now();
  while (not stopReadThread)
  {
    char c = '\0';
    if (globalDebug == 0)
    {
      int e = read(devif, &c, 1);
      if (stopReadThread)
        break;
      if (e <= 0)
      {
        i = 0;
        rxErrorCnt++;
      }
      else
      {
        if (c >= ' ' and c < '~')
        { // save usable characters only
          rxbuffer[i] = c;
          if (i < MAX_LEN)
            i++;
        }
      }
    }
    else
    { // just wait for thread to be stopped?
      Wait(0.1);
    }
    if (rxbuffer[0] != 'F' or i > 10)
    { // must start with this character
      // we only listen to status, and no message is longer than 10 chars
      i = 0;
    }
    if ((c == '\n' or c == '\r') and rxbuffer[0] != '>')
    { // there is a message
      rxbuffer[i] = '\0'; // terminate as string
      if (i == 6 and rxbuffer[0] == 'F')
      { // got full message
        // printf("Received %d bytes, got '%s'\n", i, rxbuffer);
        ballAvailable = rxbuffer[1] == '0';
        startGameSwitch = rxbuffer[3] == '0';
        craneSwitch = rxbuffer[2] != '0';
        powerOn = rxbuffer[4] == '1';
        timingCharReceived = rxbuffer[5];
        statusTime.Now();
        logStatus(statusTime, '<', rxbuffer);
        handled = true;
      }
      if (i > 1)
      { // 
        if (not handled)
          printf("tiltandcraneif: got %d chars: '%s'\n", i, rxbuffer);
        i = 0;
        handled = false;
      }
    }
  }
  readThreadRunning = false;
  iflog.closeLog();
  pthread_exit(NULL);
  return NULL;
}

/// ////////////////////////////////////////////////////////////

void getStatus()
{
  const int MSL = 6;
  char s[MSL] = "j=!\n";
  UTime t;
  s[2] = timingCharSend++;
  if (timingCharSend > 'z')
    timingCharSend = 'a';
  sendStringToIOCtrl(s);
  t.now();
  logStatus(t, '>', s);
}

/// ////////////////////////////////////////////////////////////

void * runTcrif(void * notUsed)
{
//   enum gameStates { IF_INIT, IF_WAIT, IF_CRANE_INIT, 
//                     IF_CRANE_LEFT_TO_BALL, IF_CRANE_DOWN_TO_BALL, IF_CRANE_UP_TO_TEST, 
//                     IF_CRANE_UP, IF_CRANE_LEFT_TO_DROP, IF_CRANE_DOWN_TO_DROP,
//                     IF_CRANE_UP_TO_PARK, IF_CRANE_RIGHT_TO_PARK,
//                     IF_TILT};
//   gameStates ifState;
  // debug
  ifState = IF_INIT;
  // debug end
  bool isOK;
  int ifCnt = 0;
  UTime t, twait, trun;
  int n;
  const float tickPerSecLR = 110.0;
  const float tickPerSecUD = 135.0;
  const int MSL = 30;
  char s[MSL];
  int ballCatchCnt = 0;
  int waitCnt = 0;
  int wait_ms = 0;
  int stepCnt = 0;
  const int maxStepCnt = 500;
  //
  tcrifThreadRunning = true;
  // initialice interface to not use local echo
  isOK = sendStringToIOCtrl("i=0\n");
  if (not isOK)
    printf("*** Interface to crane and tilt is not running!\n");
  getStatus();
  Wait(0.1);
  twait.now();
  t.now();
  while (not stopTcrifThread)
  {
    isOK = true;
    switch (ifState)
    {
      case IF_INIT:
        // printf("tiltandcraneif: in DEBUG mode, waiting for 'go' command\n");
        semOutputCtrl.wait();
        if (stopTcrifThread)
          break;
        if (n == 0)
          getStatus();
        //
        if (not powerOn)
          // if no power on interface, then start switch seems pressed
          break;
        n = strlen(debugIfString);
        if (n > 0 and n < MIOL - 1)
        {
          if (debugIfString[n-1] != '\n')
          { // add return and re-terminate
            debugIfString[n++] = '\n';
            debugIfString[n]   = '\0';
          }
          if (strncasecmp(debugIfString, "go", 2) == 0)
          { // go live - wait for ball load command
            ifState = IF_WAIT;
            printf("tiltandcraneif: went live, waiting for crane load command\n");
            break;
          }
          sendStringToIOCtrl(debugIfString);
        }
        else if (n > 0)
          printf("tiltandcraneif: bad string (length %d) not send\n", n);
        // clear string
        debugIfString[0] = '\0';
        break;
      case IF_WAIT:
        if (waitCnt % 100 == 0)
        { // request new status (start switch etc.)
          getStatus();
        }
        if (loadBallWithCrane)
        {
          printf("tiltandcraneif: going to craning initialization (maybe)\n");
          ifState = IF_CRANE_INIT;
        }
        if (readyToControl)
        { // activate control mode
          printf("tiltandcraneif: going to control mode\n");
          sendStringToIOCtrl("F7=1\n");
          ifState = IF_TILT;
        }
        break;
      case IF_CRANE_INIT:
        if (false) // and craneInitialized)
        { // skip init - better to rely on current park position
          if (craneSwitch)
          { //cranePositionLR = 0;
            ifState = IF_CRANE_LEFT_TO_BALL;
          }
          else
          { // move right a bit
            sendStringToIOCtrl("R010\n");
            Wait(0.2);
          }
        }
        else
        {
          int cnt = mini(craneParkToBall, maxStepCnt);
          ifState = IF_CRANE_LEFT_TO_BALL;
          printf("tiltandcraneif: crane left to ball\n");
          snprintf(s, MSL, "L%03d\n", cnt);
          isOK = sendStringToIOCtrl(s);
          wait_ms = (cnt * 1000) / tickPerSecLR;
          waitCnt = 0;
          stepCnt = cnt;
        }
        break;
      case IF_CRANE_LEFT_TO_BALL:
        if (waitCnt >= wait_ms)
        { // finished, send commands for next state
          int cnt2 = craneDownToBall;
          ifState = IF_CRANE_DOWN_TO_BALL;
          stepCnt = 0;
          printf("tiltandcraneif: crane down to ball\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "N%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          wait_ms = (cnt2 * 1000) / tickPerSecUD;
          waitCnt = 0;
        }
        break;
      case IF_CRANE_DOWN_TO_BALL:
        if (waitCnt > wait_ms)
        { // finished - get ball
          // start magnet
          isOK = sendStringToIOCtrl("F6=1\n");
          // go up to test
          ifState = IF_CRANE_UP_TO_TEST;
          wait_ms = 1;
          waitCnt = 0;
          ballCatchCnt = 0;
        }
        break;
      case IF_CRANE_UP_TO_TEST:
        if (waitCnt > wait_ms)
        { // finished - go a bit up for test
          int cnt2 = craneDownToBall - craneDownToTest;
          // go up to test
          ifState = IF_CRANE_BALL_TEST;
          stepCnt = 0;
          printf("tiltandcraneif: crane up to test ball\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "O%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          wait_ms = (cnt2 * 1000) / tickPerSecUD;
          waitCnt = 0;
        }
        break;
        //
      case IF_CRANE_BALL_TEST:
        if (waitCnt > wait_ms)
        { // finished - send command for next
          if (ballAvailable)
          { // failed to pick ball - go down again
            int cnt2 = craneDownToBall - craneDownToTest;
            isOK = (ballCatchCnt < 4);
            if (not isOK)
            { // try no more than 3 times
              printf("tiltandcraneif: failed to pick ball (tried %d times)\n", ballCatchCnt);
              // move crane back to parking position
              // release magnet
              isOK = sendStringToIOCtrl("F6=0\n");
              // go up
              cnt2 = craneDownToTest;
              stepCnt = 0;
              while (stepCnt < cnt2)
              {
                int cnt = mini(cnt2 - stepCnt, maxStepCnt);
                snprintf(s, MSL, "O%03d\n", cnt);
                isOK = sendStringToIOCtrl(s);
                stepCnt += cnt;
              }
              // go right
              cnt2 = craneParkToBall;
              stepCnt = 0;
              while (stepCnt < cnt2)
              {
                int cnt = mini(cnt2 - stepCnt, maxStepCnt);
                snprintf(s, MSL, "R%03d\n", cnt);
                isOK = sendStringToIOCtrl(s);
                stepCnt += cnt;
              }
              // will finish in good time, so just fall out to IF_WAIT
              // finished craning
              loadBallWithCrane = false;
              isOK = false;
              break;
            }
            // go down and try again
            ifState = IF_CRANE_UP_TO_TEST;
            printf("tiltandcraneif: crane pick ball (cnt=%d)\n", ballCatchCnt);
            // move a bit left
            craneParkToBall += 3;
            snprintf(s, MSL, "L003\n");
            isOK = sendStringToIOCtrl(s);
            stepCnt = 0;
            while (stepCnt < cnt2)
            {
              int cnt = mini(cnt2 - stepCnt, maxStepCnt);
              snprintf(s, MSL, "N%03d\n", cnt);
              isOK = sendStringToIOCtrl(s);
              stepCnt += cnt;
            }
            wait_ms = (cnt2 * 1000) / tickPerSecUD;
            waitCnt = 0;
            ballCatchCnt++;
          }
          else
          { // success ball is lifted away
            int cnt2 = craneDownToTest;
            stepCnt = 0;
            ifState = IF_CRANE_UP;
            printf("tiltandcraneif: crane up to test ball\n");
            while (stepCnt < cnt2)
            {
              int cnt = mini(cnt2 - stepCnt, maxStepCnt);
              snprintf(s, MSL, "O%03d\n", cnt);
              isOK = sendStringToIOCtrl(s);
              stepCnt += cnt;
            }
            wait_ms = (cnt2 * 1000) / tickPerSecUD;
            waitCnt = 0;
          }
        }
        break;
      case IF_CRANE_UP:
        if (waitCnt >= wait_ms)
        { // finished - now to drop zone
          int cnt2 = craneParkToDrop - craneParkToBall;
          ifState = IF_CRANE_LEFT_TO_DROP;
          stepCnt = 0;
          printf("tiltandcraneif: crane left to drop zone\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "L%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          wait_ms = (cnt2 * 1000) / tickPerSecLR;
          waitCnt = 0;
        }
        break;
      case IF_CRANE_LEFT_TO_DROP:
        if (waitCnt >= wait_ms)
        { // finished - now down to drop height
          int cnt2 = craneDownToDrop;
          ifState = IF_CRANE_DOWN_TO_DROP;
          stepCnt = 0;
          printf("tiltandcraneif: crane down to drop height\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "N%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          // tilt for new start - after going left
          sendStringToIOCtrl("F7=1\n");
          sendStringToIOCtrl("p180\nP130\n");
          sendStringToIOCtrl("F7=0\n");
          //
          wait_ms = (cnt2 * 1000) / tickPerSecUD;
          waitCnt = 0;
        }
        break;
      case IF_CRANE_DOWN_TO_DROP:
        if (waitCnt >= wait_ms)
        { // finished - now drop ball and go up
          int cnt2 = craneDownToDrop;
          // release magnet on port F6
          isOK = sendStringToIOCtrl("F6=0\n");
          ifState = IF_CRANE_UP_TO_PARK;
          stepCnt = 0;
          printf("tiltandcraneif: crane up to parking height\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "O%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          wait_ms = (cnt2 * 1000) / tickPerSecUD;
          waitCnt = 0;
        }
        break;
      case IF_CRANE_UP_TO_PARK:
        if (waitCnt >= wait_ms)
        { // finished - back to parking
          int cnt2 = craneParkToDrop;
          ifState = IF_CRANE_RIGHT_TO_PARK;
          stepCnt = 0;
          printf("tiltandcraneif: crane right to parking\n");
          while (stepCnt < cnt2)
          {
            int cnt = mini(cnt2 - stepCnt, maxStepCnt);
            snprintf(s, MSL, "R%03d\n", cnt);
            isOK = sendStringToIOCtrl(s);
            stepCnt += cnt;
          }
          wait_ms = (cnt2 * 1000) / tickPerSecLR / 2;
          waitCnt = 0;
        }
        break;
      case IF_CRANE_RIGHT_TO_PARK:
        if (waitCnt >= wait_ms)
        {
          printf("tiltandcraneif: crane parked\n");
          loadBallWithCrane = false;
          ifState = IF_WAIT;
        }
        break;
        //
      case IF_TILT:
        semOutputCtrl.wait();
        isOK = sendNewAngle(tiltXangle, tiltYangle);
        if (not readyToControl)
        { // stop control
          sendStringToIOCtrl("F7=0\n");
          // back to wait for next game
          ifState = IF_WAIT;
        }
        ifCnt++;
//         if (ifCnt % 20 == 0)
//           getStatus();
        break;
    }
    if (not isOK)
      ifState = IF_WAIT;
    if (not powerOn)
      ifState = IF_INIT;
    Wait(0.001);
    int msPassed = roundi(t.getTimePassed() * 1000);
    waitCnt += msPassed;
    t.Now();
  }
  // make one last status request, to make sure that rx thread can stop
  getStatus();
  // we are finished here
  tcrifThreadRunning = false;
  pthread_exit(NULL);
  return NULL;
}


/// ///////////////////////////////////////////////////////////////////////////
// start control interface to tilt position and crane
bool startTcrifCtrl()
{
  pthread_attr_t  thAttr;
  int i;
  const char * devName = "/dev/ttyACM0";
  // open connection
  if (globalDebug == 0)
  {
    devif = open(devName, O_RDWR);
    if (devif < 0)
      printf("tiltandcraneif:: failed to open control interface %s\n", devName);
  }
  //
  if ((devif >= 0 or globalDebug > 0) and not tcrifThreadRunning)
  { // start thread
    pthread_attr_init(&thAttr);
    //
    stopTcrifThread = false;
    // create thread for tilt and crane control
    if (pthread_create(&thTcrif, &thAttr, &runTcrif, NULL) != 0)
      // report error
      perror("Tcrif control thread");
    // create thread for reading from device
    if (pthread_create(&thTcrif, &thAttr, &runIfRead, NULL) != 0)
      // report error
      perror("Tcrif read thread");
    // wait for threads to initialize
    i = 0;
    while ((not tcrifThreadRunning) and (++i < 100))
      Wait(0.05);
    i = 0;
    while ((not readThreadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not tcrifThreadRunning or not readThreadRunning)
    { // failed to start
      printf("startTcrif: Failed to start threads - in time (5 sec)\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return tcrifThreadRunning;
}

/// ///////////////////////////////////////////////////////////////////////////
// stop control interface to tilt position and crane
void stopTcrifCtrl()
{
  if (tcrifThreadRunning)
  {
    stopTcrifThread = true;
    stopReadThread = true;
    if (tcrifThreadRunning)
    { // make sure there is some communication
      getStatus();
      // wait for thread to finish
      pthread_join(thTcrif, NULL);
    }
    if (readThreadRunning)
    {
      semOutputCtrl.post();
      pthread_join(thTcrif, NULL);
    }
    // debug
    printf("stopTcrif: thread stopped\n");
  }
  if (devif >= 0)
    close(devif);
  devif =  -1;
}

///////////////////////////////////////////////

const char * getControlStateString()
{
//   enum controlState { IF_INIT, IF_WAIT, IF_CRANE_INIT, 
//                     IF_CRANE_LEFT_TO_BALL, IF_CRANE_DOWN_TO_BALL, IF_CRANE_UP_TO_TEST, 
//                     IF_CRANE_UP, IF_CRANE_LEFT_TO_DROP, IF_CRANE_DOWN_TO_DROP,
//                     IF_CRANE_UP_TO_PARK, IF_CRANE_RIGHT_TO_PARK,
//                     IF_TILT};

  switch (ifState)
  {
    case IF_INIT: return "off"; break;
    case IF_WAIT: return "wait"; break;
    case IF_CRANE_INIT: return "crane init"; break;
    case IF_CRANE_LEFT_TO_BALL: return "left to ball"; break;
    case IF_CRANE_DOWN_TO_BALL: return "down to ball"; break;
    case IF_CRANE_UP_TO_TEST: return "up to ball test"; break;
    case IF_CRANE_BALL_TEST: return "ball test"; break;
    case IF_CRANE_UP: return "ball up"; break;
    case IF_CRANE_LEFT_TO_DROP: return "to drop zone"; break;
    case IF_CRANE_DOWN_TO_DROP: return "down to drop"; break;
    case IF_CRANE_UP_TO_PARK: return "up to park"; break;
    case IF_CRANE_RIGHT_TO_PARK: return "right to park"; break;
    case IF_TILT: return "tilt control"; break;
  }
  return "error";
}

/////////////////////////////////////////////////