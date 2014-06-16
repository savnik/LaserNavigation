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

#define SVN_REV "$Rev: 230 $"
/**
 * Labyrinth game start and end code */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ugen4/ulock.h>
#include "labyrinthgame.h"
#include "fwguppy.h"
#include "showimage.h"
#include "kalman.h"
#include "track.h"
#include "refgen.h"
#include "tiltcontrol.h"
#include "tiltandcraneif.h"
#include "gamesim.h"

/**
 * Path waypoints in image coordinates [x,y]
 * Starting with element 0 is the ball start position */
int pathPoints[MAXPUNKT][2];
/// number of points in path
int pathPointsCnt = 0;
/// is game thread running
bool gameThreadRunning;
/// stop running game thread
bool stopGameThread;
/// game thread handle
pthread_t thGame;
/// start game control thread
bool startGameCtrl();
/// stop game control thread
void stopGameCtrl();
/// game start time - from start of control
UTime gameStartTime;
/// game stop time - at GAME_OVER (ball lost)
UTime gameStopTime;
/// read configuration from ini-file
void loadConfiguration();
/// save configuration to ini-file
void saveConfiguation();
/// configuration file
const int MSC = 100;
char * pretext[MSC];
// debug log flag
bool debugLog;
/// main state of game
gameStates gameState = GAME_WAIT_TO_START;
// /// should game stop
// extern int *gStop;
//extern int      *gCmdSema, *gCmdID, *gReturnValue, *gDAReady;
int globalDebug;

int main(int argc, char * argv[])
{
  int isOK;
  const char MLL = 100;
  char line[MLL];
  int top = 0;
  int shut = 400;
  int gain = 40;
  //
  globalDebug = 0;
  for (int k = 0; k < MSC; k++)
    pretext[k] = NULL;
  loadConfiguration();
  //
  if (argc > 1)
  {
    globalDebug = strtol(argv[1], NULL, 0);
    if (globalDebug == 0)
    { // print help
      printf("Usage (svn " SVN_REV "):\n");
      printf("  labyrinthgame            normal live run of game\n");
      printf("  labyrinthgame 1          debug of firewire camera (debug loop)\n");
      printf("  labyrinthgame 2          no camera, loaded image, simulate movement\n");
      printf("  labyrinthgame 3          no camera, no sim, live ball finding in loaded image\n");
      printf("  labyrinthgame -h         this help text\n");
      return 0;
    }
  }
  if (globalDebug == 1)
  { // do nothing but firewire looptest
    looptest();
  }
  // for debug logging
  strcpy(dataPath, argv[0]);
  char * p1 = strrchr(dataPath, '/');
  if (p1 != NULL)
    *p1 = '\0';
  // debug logging end
  if (globalDebug == 0 or globalDebug > 1)
  {
    setROIrequest(2, 64);
    isOK = startCamera();
    if (isOK)
      isOK = startShowImage();
    if (isOK)
      isOK = startControl();
    if (isOK)
      isOK = startGameCtrl();
    if (isOK)
    {
      isOK = startTcrifCtrl();
      if (not isOK)
        printf("failed to open labyrinth interface - proceeding however\n");
      isOK = true;
    }
//     if (isOK)
//       isOK = startSim();
    // listen to keyboard
    line[0] = 'h';
    line[1] = '\0';
    Wait(0.1);
    while (isOK and line[0] != 'q')
    {
      printf(">> ");
      fgets(line, MLL, stdin);
      // get possible start number
      switch (line[0])
      {
        case 'd':
          top = frameTop + 20;
          if (top > 420)
            top = 420;
          printf("setting top to %d\n", top);
          setROIrequest(top, 60);
          break;
        case 'u':
          top = frameTop - 20;
          if (top < 0)
            top = 0;
          printf("setting top to %d\n", top);
          setROIrequest(top, 60);
          break;
        case 's':
          shut = strtol(&line[1], NULL, 0);
          if (shut < 1)
            shut = 1;
          else if (shut > 4096)
            shut = 4096;
          printf("setting shutter to %d\n", shut);
          shutterValue = shut;
          break;
        case 'a':
          gain = strtol(&line[1], NULL, 0);
          if (gain > 64)
            gain = 64;
          else if (gain < 16)
            gain = 16;
          printf("setting gain to %d\n", gain);
          gainValue = gain;
          break;
        case 'i':
          imgFull->lock();
          imgFull->savePNG("imgFull.png");
          imgFull->saveBMP("imgFull.bmp");
          imgFull->unlock();
          imgBuff->lock();
          imgBuff->savePNG("imgRaw.png");
          imgBuff->saveBMP("imgRaw.bmp");
          imgBuff->unlock();
          imgBufferDebug->lock();
          imgBufferDebug->savePNG("imgDisplay.png");
          imgBufferDebug->unlock();
          break;
        case 'r':
          showImageClickRoute = not showImageClickRoute;
          break;
        case 'f':
          showImageClickFrame = not showImageClickFrame;
          break;
        case 'w':
          saveConfiguation();
          break;
        case 't':
          tiltScale = strtof(&line[1], NULL);
          break;
//         case 'c': // print status
//           // tell interface to go into "waitForBall" mode
//           strncpy(debugIfString, "go", MIOL);
//           semOutputCtrl.clearPosts();
//           semOutputCtrl.post();
//           break;
        case 'g': // print status
          strncpy(debugIfString, &line[1], MIOL);
          semOutputCtrl.clearPosts();
          semOutputCtrl.post();
          break;
        case 'q': // print status
          break;
        default: // help
          printf("Short help list:\n");
          printf("  q               Quit server\n");
          printf("  u               move image slize up   (-20), is %d\n", frameTop);
          printf("  d               move image slize down (+20), is %d\n", frameTop);
          printf("  s N             shutter value,   is %d (1..4096)\n", shutterValue);
          printf("  a N             video gain value, is %d (16-64)\n", gainValue);
          printf("  r               Click route on/off is on %s\n", bool2str(showImageClickRoute));
          printf("  f               Click frame top-left CCV %s\n", bool2str(showImageClickFrame));
          printf("  i               Save raw image, full image and display image\n");
          printf("  w               Write frame and route to disk\n");
          printf("  t               tilt scale - is %g\n", tiltScale);
//          printf("  c               start listen to start switch\n");
          printf("  g str           write string (after g) to game interface\n");
          printf("  q               Quit server\n");
          printf("  help            Main help text\n");
          break;
      }
    }
    // stop all threads
    stopSim();
    stopTcrifCtrl();
    stopControl();
    stopShowImage();
    stopCamera();
  }
  printf("Labyrinth game finished\n");
  // release pretext strings
  int k = 0;
  while (pretext[k] != NULL)
    free(pretext[k++]);
  if (imgFull != NULL)
    delete imgFull;
  return 0;
}

/// //////////////////////////////////////////////////////////////////////////

/*
 * Convert this image position to frame position
 * \param r is image row number
 * \param c image column
 * \param x is set to x position in frame coordinates 0,0 is top-left x is right y is down
 * \param y is set to y position in framed coordinates */
void imageToFrame(float r, float c, float * x, float * y)
{
  float px = c  - framePoints[0][1];
  float py = r  - framePoints[0][0];
  *x = px *  cos(frameAngle) + py * sin(frameAngle);
  *y = px * -sin(frameAngle) + py * cos(frameAngle);
}

/*
 * Convert this frame position to image position
 * \param x is in frame coordinates 0,0 is top-left cornet of frame x is right y is down
 * \param y is in frame coordinates
 * \param r is set to corresponding image row
 * \param c is set to corresponding image coumn */
void frameToImage(float x, float y, float * r, float * c)
{
  *c = x *  cos(frameAngle) + y * -sin(frameAngle) + framePoints[0][1];
  *r = x *  sin(frameAngle) + y *  cos(frameAngle) + framePoints[0][0];
}


/// ///////////////////////////////////////////////////////////////////////////

void * runGame(void * notUsed)
{
  gameThreadRunning = true;
//   enum gameStates { GAME_WAIT_TO_START, GAME_CRANING, GAME_START, GAME_RUN, GAME_OVER};
//   gameStates gameState = GAME_WAIT_TO_START;
  int waitCnt = 0;
  ballOK = false;
  UTime t1;
  printf("game master: in 'wait to start' mode\n");
  while (not stopGameThread)
  {
    switch (gameState)
    {
      case GAME_WAIT_TO_START:
        if (not powerOn)
          break;
        if (ballOK and startGameSwitch)
        { // ball is in game, but game is not running
          printf("game master: ball in game and start, so starting game\n");
          // get interface out of (possible) debug state
          strncpy(debugIfString, "go", MIOL);
          gameState = GAME_START;
        }
        else if (ballAvailable and startGameSwitch)
        { // ball is in start position crane it to game
          printf("game master: ball available, starting crane\n");
          loadBallWithCrane = true;
          // get interface out of (possible) debug state
          strncpy(debugIfString, "go", MIOL);
          gameState = GAME_CRANING;
        }
        else if (startGameSwitch)
        { // ball is in start position crane it to game
          printf("game master: No ball in sight - place ball for crane.\n");
          // wait for next status 
          startGameSwitch = 0;
        }
        else
        { // wait a bit and maybe scan for ball in game
          waitCnt++;
        }
        break;
      case GAME_CRANING:
        // Wait until craning is finished
        if (not loadBallWithCrane)
        { // 
          if (ballOK)
            gameState = GAME_START;
          else
          {
            printf("game master: crane action failed - can not see ball\n");
            gameState = GAME_WAIT_TO_START;
          }
        }
        break;
      case GAME_START:
        // start game
        // ball is in place start roling
        //isOK = turnOnLight(true);
        printf("game master: starting game\n");
        ballLost = false;
        readyToControl = true;
        gameState = GAME_RUN;
        gameStartTime.now();
        break;
      case GAME_RUN:
        if (ballLost)
        { // ballLost is maintained only when readyToControl is true
          gameState = GAME_OVER;
          readyToControl = false;
        }
        break;
      case GAME_OVER:
        gameStopTime.now();
        printf("Game over (after %.2f secs)\n", gameStartTime - gameStopTime);
        gameState = GAME_WAIT_TO_START;
        break;
    }
    if (not powerOn)
    {
      gameState = GAME_WAIT_TO_START;
      if (t1.getTimePassed() > 2.0)
      {
        printf("=== seems like there is no power on game control box?\n");
        t1.now();
      }
    }
    Wait(0.1);
    if (statusTime.getTimePassed() > 0.05)
    {
      semOutputCtrl.clearPosts();
      semOutputCtrl.post();
    }
  }
  gameThreadRunning = false;
  pthread_exit(NULL);
  return NULL;
}


/// ///////////////////////////////////////////////////////////////////////////
/// start game control thread
bool startGameCtrl()
{
  pthread_attr_t  thAttr;
  int i = 0;
  //
  if (not gameThreadRunning)
  { // start thread
    pthread_attr_init(&thAttr);
    //
    stopGameThread = false;
    // create socket server thread
    if (pthread_create(&thGame, &thAttr, &runGame, NULL) != 0)
      // report error
      perror("game thread");
      // wait for thread to initialize
    while ((not gameThreadRunning) and (++i < 100))
      Wait(0.05);
    // test for started thread
    if (not gameThreadRunning)
    { // failed to start
      printf("startGame: Failed to start thread - in time (5 sec)\n");
    }
    pthread_attr_destroy(&thAttr);
  }
  return gameThreadRunning;
}

/// ///////////////////////////////////////////////////////////////////////////
/// stop game control thread
void stopGameCtrl()
{
  if (gameThreadRunning)
  {
    stopGameThread = true;
    if (gameThreadRunning)
      pthread_join(thGame, NULL);
    // debug
    printf("stopGame: thread stopped\n");
  }
}

/// ///////////////////////////////////////////////////////////////////

void loadConfiguration()
{
  FILE * f;
  const int MLL = 200;
  char s[MLL];
  char * p1, *p2;
  enum loadsection {prestring, log, frame, cam, route, green, ballance, crane, end};
  loadsection loading = prestring;
  int i = 0, k;
  int craneCnt = 0;
  f = fopen("labyrinthgame.ini", "r");
  if (f != NULL)
  {
    k = 0;
    p1 = s;
    while (p1 != NULL)
    {
      p1 = fgets(s, MLL, f);
      if (s[0] == '#')
      {
        if (strcasestr(s, "#frame#") != NULL)
          loading = frame;
        else if (strcasestr(s, "#log#") != NULL)
          loading = log;
        else if (strcasestr(s, "#cam#") != NULL)
          loading = cam;
        else if (strcasestr(s, "#route#") != NULL)
          loading = route;
        else if (strcasestr(s, "#green#") != NULL)
          loading = green;
        else if (strcasestr(s, "#ballance#") != NULL)
          loading = ballance;
        else if (strcasestr(s, "#crane#") != NULL)
          loading = crane;
        else if (strcasestr(s, "#end") != NULL)
          loading = end;
        else
        { // prestring or comments - saved in case of write back to changed file
          int n = strlen(s) + 1;
          pretext[k] = (char *) malloc(n + 1);
          strncpy(pretext[k], s, n);
          pretext[k][n] = '\0';
          k++;
        }
        i = 0;
      }
      else
      {
        int r, c;
        p2 = s;
        r = strtol(p2, &p2, 0);
        if (p2 != NULL)
          c = strtol(p2, &p2, 0);
        switch (loading)
        {
          case frame:
            if (i < 4)
            {
              framePoints[i][0] = r;
              framePoints[i][1] = c;
              i++;
            }
            break;
          case route:
            if (r > 0 and c > 0)
            {
              routePoints[routePointsCnt][0] = r; // * 10.6 / 9.5 + 8;
              routePoints[routePointsCnt][1] = c; // * 10.3 / 9.5 + 15;
              if (routePointsCnt + 1 < MRP)
                routePointsCnt++;
            }
            break;
          case green:
            greenOffset[0] = r;
            greenOffset[1] = c;
            break;
          case ballance:
            xyBalance[0] = r;
            xyBalance[1] = c;
            break;
          case cam:
            gainValue = r;
            shutterValue = c;
            break;
          case log:
            debugLog = r;
            break;
          case crane:
            switch (craneCnt)
            {
              case 0: craneParkToBall = r; break;
              case 1: craneParkToDrop = r; break;
              case 2: craneDownToBall = r; break;
              case 3: craneDownToTest = r; break;
              case 4: craneDownToDrop = r; break;
              default: break;
            }
            craneCnt++;
            break;
          default:
            break;
        }
      }
    }
    fclose(f);
  }
}

///  ///////////////////////////////////////////////////////////

void saveConfiguation()
{
  FILE * f;
  f = fopen("labyrinthgame.ini", "w");
  if (f != NULL)
  {
    int k = 0;
    while (pretext[k] != NULL)
      fprintf(f, "%s", pretext[k++]);
    fprintf(f, "#log# make debug log of selected events, 1=true, 0=false\n");
    fprintf(f, "%d\n", debugLog);
    fprintf(f, "#cam# set gain and shutter value for camera gain [15..64]dB, shutter [1..4096] but should be < 180 (1.8ms))\n");
    fprintf(f, "%d %d\n", gainValue, shutterValue);
    //
    fprintf(f, "#green# gain offset BGGR first is added to green in top (odd) row, second is added to green in bottom row\n");
    fprintf(f, "%d %d\n", greenOffset[0], greenOffset[1]);
    //
    fprintf(f, "#frame# 4 points each r,c CV first two is used to calculate frame angle\n");
    for (int i = 0; i < 4; i++)
    {
      fprintf(f, "%d %d\n", framePoints[i][0], framePoints[i][1]);
    }
    fprintf(f, "#route# has %d points each r,c\n", routePointsCnt);
    for (int i = 0; i < routePointsCnt; i++)
    {
      fprintf(f, "%d %d\n", routePoints[i][0], routePoints[i][1]);
    }
    fprintf(f, "#end\n");
    fclose(f);
  }
}

/// ///////////////////////////////////////////////////////////////


// enum gameStates { GAME_WAIT_TO_START, GAME_CRANING, GAME_START, GAME_RUN, GAME_OVER};
// extern gameStates gameState;

const char * getGameStateString()
{
  switch (gameState)
  {
    case GAME_WAIT_TO_START: return "wait to start"; break;
    case GAME_CRANING: return "craning"; break;
    case GAME_START: return "starting"; break;
    case GAME_RUN: return "running"; break;
    case GAME_OVER: return "OVER"; break;
  }
  return "error";
}


